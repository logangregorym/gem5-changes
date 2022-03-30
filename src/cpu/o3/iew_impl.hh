/*
 * Copyright (c) 2010-2013 ARM Limited
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Kevin Lim
 */

#ifndef __CPU_O3_IEW_IMPL_IMPL_HH__
#define __CPU_O3_IEW_IMPL_IMPL_HH__

// @todo: Fix the instantaneous communication among all the stages within
// iew.  There's a clear delay between issue and execute, yet backwards
// communication happens simultaneously.

#include <queue>

#include "arch/utility.hh"
#include "arch/x86/superop/trace_based_graph.hh"
#include "config/the_isa.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/o3/dep_graph.hh"
#include "cpu/o3/fu_pool.hh"
#include "cpu/o3/iew.hh"
#include "cpu/timebuf.hh"
#include "debug/Activity.hh"
#include "debug/Drain.hh"
#include "debug/ExecResult.hh"
#include "debug/IEW.hh"
#include "debug/LVP.hh"
#include "debug/O3PipeView.hh"
#include "params/DerivO3CPU.hh"

using namespace std;

template<class Impl>
DefaultIEW<Impl>::DefaultIEW(O3CPU *_cpu, DerivO3CPUParams *params)
    : issueToExecQueue(params->backComSize, params->forwardComSize),
      cpu(_cpu),
      instQueue(_cpu, this, params),
      ldstQueue(_cpu, this, params),
      fuPool(params->fuPool),
      commitToIEWDelay(params->commitToIEWDelay),
      renameToIEWDelay(params->renameToIEWDelay),
      issueToExecuteDelay(params->issueToExecuteDelay),
      dispatchWidth(params->dispatchWidth),
      issueWidth(params->issueWidth),
      wbWidth(params->wbWidth),
      numThreads(params->numThreads)
{
    if (dispatchWidth > Impl::MaxWidth)
        fatal("dispatchWidth (%d) is larger than compiled limit (%d),\n"
             "\tincrease MaxWidth in src/cpu/o3/impl.hh\n",
             dispatchWidth, static_cast<int>(Impl::MaxWidth));
    if (issueWidth > Impl::MaxWidth)
        fatal("issueWidth (%d) is larger than compiled limit (%d),\n"
             "\tincrease MaxWidth in src/cpu/o3/impl.hh\n",
             issueWidth, static_cast<int>(Impl::MaxWidth));
    if (wbWidth > Impl::MaxWidth)
        fatal("wbWidth (%d) is larger than compiled limit (%d),\n"
             "\tincrease MaxWidth in src/cpu/o3/impl.hh\n",
             wbWidth, static_cast<int>(Impl::MaxWidth));

    _status = Active;
    exeStatus = Running;
    wbStatus = Idle;

    // Setup wire to read instructions coming from issue.
    fromIssue = issueToExecQueue.getWire(-issueToExecuteDelay);

    // Instruction queue needs the queue between issue and execute.
    instQueue.setIssueToExecuteQueue(&issueToExecQueue);

    for (ThreadID tid = 0; tid < numThreads; tid++) {
        dispatchStatus[tid] = Running;
        fetchRedirect[tid] = false;
    }

    updateLSQNextCycle = false;

    skidBufferMax = (renameToIEWDelay + 4) * params->renameWidth;

    loadPred = params->loadPred;

    enableValuePredForwarding = params->enableValuePredForwarding;
}

template <class Impl>
std::string
DefaultIEW<Impl>::name() const
{
    return cpu->name() + ".iew";
}

template <class Impl>
void
DefaultIEW<Impl>::regProbePoints()
{
    ppDispatch = new ProbePointArg<DynInstPtr>(cpu->getProbeManager(), "Dispatch");
    ppMispredict = new ProbePointArg<DynInstPtr>(cpu->getProbeManager(), "Mispredict");
    /**
     * Probe point with dynamic instruction as the argument used to probe when
     * an instruction starts to execute.
     */
    ppExecute = new ProbePointArg<DynInstPtr>(cpu->getProbeManager(),
                                              "Execute");
    /**
     * Probe point with dynamic instruction as the argument used to probe when
     * an instruction execution completes and it is marked ready to commit.
     */
    ppToCommit = new ProbePointArg<DynInstPtr>(cpu->getProbeManager(),
                                               "ToCommit");
}

template <class Impl>
void
DefaultIEW<Impl>::regStats()
{
    using namespace Stats;

    instQueue.regStats();
    ldstQueue.regStats();

    iewIdleCycles
        .name(name() + ".iewIdleCycles")
        .desc("Number of cycles IEW is idle");

    iewSquashCycles
        .name(name() + ".iewSquashCycles")
        .desc("Number of cycles IEW is squashing");

    iewBlockCycles
        .name(name() + ".iewBlockCycles")
        .desc("Number of cycles IEW is blocking");

    iewUnblockCycles
        .name(name() + ".iewUnblockCycles")
        .desc("Number of cycles IEW is unblocking");

    iewDispatchedInsts
        .name(name() + ".iewDispatchedInsts")
        .desc("Number of instructions dispatched to IQ");

    iewDispSquashedInsts
        .name(name() + ".iewDispSquashedInsts")
        .desc("Number of squashed instructions skipped by dispatch");

    iewDispLoadInsts
        .name(name() + ".iewDispLoadInsts")
        .desc("Number of dispatched load instructions");

    iewDispStoreInsts
        .name(name() + ".iewDispStoreInsts")
        .desc("Number of dispatched store instructions");

    iewDispSpeculativeLoadInsts
        .name(name() + ".iewDispSpeculativeLoadInsts")
        .desc("Number of dispatched Speculative load instructions");

    iewDispSpeculativeStoreInsts
        .name(name() + ".iewDispSpeculativeStoreInsts")
        .desc("Number of dispatched Speculative store instructions");


    iewDispNonSpecInsts
        .name(name() + ".iewDispNonSpecInsts")
        .desc("Number of dispatched non-speculative instructions");

    iewIQFullEvents
        .name(name() + ".iewIQFullEvents")
        .desc("Number of times the IQ has become full, causing a stall");

    iewLSQFullEvents
        .name(name() + ".iewLSQFullEvents")
        .desc("Number of times the LSQ has become full, causing a stall");

    blockingDueToBandwidthFull
        .name(name() + ".blockingDueToBandwidthFull")
        .desc("Number of times the we run out of instructions to run, causing a stall");

    stalledDueToCommit
        .name(name() + ".stalledDueToCommit")
        .desc("Number of times we wait for ROB to finish squashing");

    stalledDueToIQFull
        .name(name() + ".stalledDueToIQFull")
        .desc("Number of times the IQ is full forcing us to wait");

    memOrderViolationEvents
        .name(name() + ".memOrderViolationEvents")
        .desc("Number of memory order violations");

    predictedTakenIncorrect
        .name(name() + ".predictedTakenIncorrect")
        .desc("Number of branches that were predicted taken incorrectly");

    predictedNotTakenIncorrect
        .name(name() + ".predictedNotTakenIncorrect")
        .desc("Number of branches that were predicted not taken incorrectly");

    branchMispredicts
        .name(name() + ".branchMispredicts")
        .desc("Number of branch mispredicts detected at execute");

    branchMispredicts = predictedTakenIncorrect + predictedNotTakenIncorrect;


    specTracePredTakenIncorrectFromUopCache
        .name(name() + ".specTracePredTakenIncorrectFromUopCache")
        .desc("");
    
    specTracePredTakenIncorrectFromBranchPred
        .name(name() + ".specTracePredTakenIncorrectFromBranchPred")
        .desc("");
    
    specTracePredNotTakenIncorrectFromBranchPred
        .name(name() + ".specTracePredNotTakenIncorrectFromBranchPred")
        .desc("");
    
    specTracePredNotTakenIncorrectFromUopCache
        .name(name() + ".specTracePredNotTakenIncorrectFromUopCache")
        .desc("");
    
    specTracePredTakenIncorrectFromTrace
        .name(name() + ".specTracePredTakenIncorrectFromTrace")
        .desc("");
    
    specTracePredNotTakenIncorrectFromTrace
        .name(name() + ".specTracePredNotTakenIncorrectFromTrace")
        .desc("");

    specTracePredTakenIncorrectFromICache
        .name(name() + ".specTracePredTakenIncorrectFromICache")
        .desc("");
    
    specTracePredNotTakenIncorrectFromICache
        .name(name() + ".specTracePredNotTakenIncorrectFromICache")
        .desc("");
    
    predictedIndirectIncorrect
        .name(name() + ".predictedIndirectIncorrect")
        .desc("");
    
    specTracePredIndirectIncorrectFromBranchPred
        .name(name() + ".specTracePredIndirectIncorrectFromBranchPred")
        .desc("");
    
    specTracePredIndirectIncorrectFromTrace
        .name(name() + ".specTracePredIndirectIncorrectFromTrace")
        .desc("");
    
    specTracePredIndirectIncorrectFromUopCache
        .name(name() + ".specTracePredIndirectIncorrectFromUopCache")
        .desc("");
            
    specTracePredIndirectIncorrectFromICache
        .name(name() + ".specTracePredIndirectIncorrectFromICache")
        .desc("");
    
    predictedUnknownBranchIncorrect
        .name(name() + ".predictedUnknownBranchIncorrect")
        .desc("");


    iewExecutedInsts
        .name(name() + ".iewExecutedInsts")
        .desc("Number of executed instructions");

    iewExecLoadInsts
        .init(cpu->numThreads)
        .name(name() + ".iewExecLoadInsts")
        .desc("Number of load instructions executed")
        .flags(total);

    iewExecSquashedInsts
        .name(name() + ".iewExecSquashedInsts")
        .desc("Number of squashed instructions skipped in execute");

    iewExecutedSwp
        .init(cpu->numThreads)
        .name(name() + ".exec_swp")
        .desc("number of swp insts executed")
        .flags(total);

    iewExecutedNop
        .init(cpu->numThreads)
        .name(name() + ".exec_nop")
        .desc("number of nop insts executed")
        .flags(total);

    iewExecutedRefs
        .init(cpu->numThreads)
        .name(name() + ".exec_refs")
        .desc("number of memory reference insts executed")
        .flags(total);

    iewExecutedBranches
        .init(cpu->numThreads)
        .name(name() + ".exec_branches")
        .desc("Number of branches executed")
        .flags(total);

    iewExecStoreInsts
        .name(name() + ".exec_stores")
        .desc("Number of stores executed")
        .flags(total);
    iewExecStoreInsts = iewExecutedRefs - iewExecLoadInsts;

    iewExecRate
        .name(name() + ".exec_rate")
        .desc("Inst execution rate")
        .flags(total);

    iewExecRate = iewExecutedInsts / cpu->numCycles;

    iewInstsToCommit
        .init(cpu->numThreads)
        .name(name() + ".wb_sent")
        .desc("cumulative count of insts sent to commit")
        .flags(total);

    writebackCount
        .init(cpu->numThreads)
        .name(name() + ".wb_count")
        .desc("cumulative count of insts written-back")
        .flags(total);

    producerInst
        .init(cpu->numThreads)
        .name(name() + ".wb_producers")
        .desc("num instructions producing a value")
        .flags(total);

    consumerInst
        .init(cpu->numThreads)
        .name(name() + ".wb_consumers")
        .desc("num instructions consuming a value")
        .flags(total);

    wbFanout
        .name(name() + ".wb_fanout")
        .desc("average fanout of values written-back")
        .flags(total);

    wbFanout = producerInst / consumerInst;

    wbRate
        .name(name() + ".wb_rate")
        .desc("insts written-back per cycle")
        .flags(total);
    wbRate = writebackCount / cpu->numCycles;

    loopsFromLtage
        .name(name() + ".loopsFromLtage")
        .desc("times LTAGE branch predictor detected the end of a loop");

    confidenceThresholdPassed
        .init(cpu->numThreads)
        .name(name() + ".confidenceThresholdPassed")
        .desc("loads which gained a prediction between fetch and iew")
        .flags(total);

    predictionChanged
        .init(cpu->numThreads)
        .name(name() + ".predictionChanged")
        .desc("loads whose prediction changed between fetch and iew")
        .flags(total);

    predictionInvalidated
        .init(cpu->numThreads)
        .name(name() + ".predictionInvalidated")
        .desc("loads which received a prediction in fetch that was invalidated by iew.")
        .flags(total);

    confidenceThresholdPassedPercent
        .name(name() + ".confidenceThresholdPassedPercent")
        .desc("loads which gained a prediction between fetch and iew (percent).")
        .flags(total);

    confidenceThresholdPassedPercent = confidenceThresholdPassed / iewExecLoadInsts * 100;

    predictionChangedPercent
        .name(name() + ".predictionChangedPercent")
        .desc("loads whose prediction changed between fetch and iew (percent).")
        .flags(total);

    predictionChangedPercent = predictionChanged / iewExecLoadInsts * 100;

    predictionInvalidatedPercent
        .name(name() + ".predictionInvalidatedPercent")
        .desc("loads which received a prediction in fetch that was invalidated by iews (percent).")
        .flags(total);

    predictionInvalidatedPercent = predictionInvalidated / iewExecLoadInsts * 100;

    instsSquashedByLVP
        .init(cpu->numThreads)
        .name(name() + ".instsSquashedByLVP")
        .desc("insts squashed due to load value mispredictions")
        .flags(total);

    squashedLoadsPresentInBothCaches
        .init(cpu->numThreads)
        .name(name() + ".squashedLoadsPresentInBothCaches")
        .desc("load value mispredictions that haven't been evicted from either cache")
        .flags(total);

    squashedLoadsOnlyInUopCache
        .init(cpu->numThreads)
        .name(name() + ".squashedLoadsOnlyInUopCache")
        .desc("load value mispredictions that have been evicted from speculative cache")
        .flags(total);

    squashedLoadsOnlyInSpecCache
        .init(cpu->numThreads)
        .name(name() + ".squashedLoadsOnlyInSpecCache")
        .desc("load value mispredictions that have been evicted from uop cache")
        .flags(total);

    squashedLoadsInNeitherCache
        .init(cpu->numThreads)
        .name(name() + ".squashedLoadsInNeitherCache")
        .flags(total);

    totalNumOfTimesControlSourcesOfTracesAreMisspredicted
        .name(name() + ".totalNumOfTimesControlSourcesOfTracesAreMisspredicted")
        .desc("total Number Of Times Control Sources Of Traces Are Misspredicted");

    totalNumOfTimesPredictionSourcesOfTracesAreMisspredicted
        .name(name() + ".totalNumOfTimesPredictionSourcesOfTracesAreMisspredicted")
        .desc("total Number Of Times Prediction Sources Of Traces Are Misspredicted");

    LQFullScenariosDist
        .init(0,100,10)
        .name(name() + ".LQFullScenariosDist")
        .desc("Percentage of speculative to non-speculative loads in case of a LQ full event")
        .flags(Stats::pdf)
        ;
    SQFullScenariosDist
        .init(0,100,10)
        .name(name() + ".SQFullScenariosDist")
        .desc("Percentage of speculative to non-speculative stores in case of a SQ full event")
        .flags(Stats::pdf)
        ;

}

template<class Impl>
void
DefaultIEW<Impl>::startupStage()
{
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        toRename->iewInfo[tid].usedIQ = true;
        toRename->iewInfo[tid].freeIQEntries =
            instQueue.numFreeEntries(tid);

        toRename->iewInfo[tid].usedLSQ = true;
        toRename->iewInfo[tid].freeLQEntries = ldstQueue.numFreeLoadEntries(tid);
        toRename->iewInfo[tid].freeSQEntries = ldstQueue.numFreeStoreEntries(tid);
    }

    // Initialize the checker's dcache port here
    if (cpu->checker) {
        cpu->checker->setDcachePort(&cpu->getDataPort());
    }

    cpu->activateStage(O3CPU::IEWIdx);
}

template<class Impl>
void
DefaultIEW<Impl>::setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr)
{
    timeBuffer = tb_ptr;

    // Setup wire to read information from time buffer, from commit.
    fromCommit = timeBuffer->getWire(-commitToIEWDelay);

    // Setup wire to write information back to previous stages.
    toRename = timeBuffer->getWire(0);

    toFetch = timeBuffer->getWire(0);

    // Instruction queue also needs main time buffer.
    instQueue.setTimeBuffer(tb_ptr);
}

template<class Impl>
void
DefaultIEW<Impl>::setRenameQueue(TimeBuffer<RenameStruct> *rq_ptr)
{
    renameQueue = rq_ptr;

    // Setup wire to read information from rename queue.
    fromRename = renameQueue->getWire(-renameToIEWDelay);
}

template<class Impl>
void
DefaultIEW<Impl>::setIEWQueue(TimeBuffer<IEWStruct> *iq_ptr)
{
    iewQueue = iq_ptr;

    // Setup wire to write instructions to commit.
    toCommit = iewQueue->getWire(0);
}

template<class Impl>
void
DefaultIEW<Impl>::setActiveThreads(list<ThreadID> *at_ptr)
{
    activeThreads = at_ptr;

    ldstQueue.setActiveThreads(at_ptr);
    instQueue.setActiveThreads(at_ptr);
}

template<class Impl>
void
DefaultIEW<Impl>::setScoreboard(Scoreboard *sb_ptr)
{
    scoreboard = sb_ptr;
}

template <class Impl>
bool
DefaultIEW<Impl>::isDrained() const
{
    bool drained = ldstQueue.isDrained() && instQueue.isDrained();

    for (ThreadID tid = 0; tid < numThreads; tid++) {
        if (!insts[tid].empty()) {
            DPRINTF(Drain, "%i: Insts not empty.\n", tid);
            drained = false;
        }
        if (!skidBuffer[tid].empty()) {
            DPRINTF(Drain, "%i: Skid buffer not empty.\n", tid);
            drained = false;
        }
        drained = drained && dispatchStatus[tid] == Running;
    }

    // Also check the FU pool as instructions are "stored" in FU
    // completion events until they are done and not accounted for
    // above
    if (drained && !fuPool->isDrained()) {
        DPRINTF(Drain, "FU pool still busy.\n");
        drained = false;
    }

    return drained;
}

template <class Impl>
void
DefaultIEW<Impl>::drainSanityCheck() const
{
    assert(isDrained());

    instQueue.drainSanityCheck();
    ldstQueue.drainSanityCheck();
}

template <class Impl>
void
DefaultIEW<Impl>::takeOverFrom()
{
    // Reset all state.
    _status = Active;
    exeStatus = Running;
    wbStatus = Idle;

    instQueue.takeOverFrom();
    ldstQueue.takeOverFrom();
    fuPool->takeOverFrom();

    startupStage();
    cpu->activityThisCycle();

    for (ThreadID tid = 0; tid < numThreads; tid++) {
        dispatchStatus[tid] = Running;
        fetchRedirect[tid] = false;
    }

    updateLSQNextCycle = false;

    for (int i = 0; i < issueToExecQueue.getSize(); ++i) {
        issueToExecQueue.advance();
    }
}

template<class Impl>
void
DefaultIEW<Impl>::squash(ThreadID tid)
{
    DPRINTF(IEW, "[tid:%i]: Squashing all instructions.\n", tid);

    // Tell the IQ to start squashing.
    instQueue.squash(tid);

    // Tell the LDSTQ to start squashing.
    ldstQueue.squash(fromCommit->commitInfo[tid].doneSeqNum,fromCommit->commitInfo[tid].squashDueToLVP , tid);
    updatedQueues = true;

    // Clear the skid buffer in case it has any data in it.
    DPRINTF(IEW, "[tid:%i]: Removing skidbuffer instructions until [sn:%i].\n",
            tid, fromCommit->commitInfo[tid].doneSeqNum);

    while (!skidBuffer[tid].empty()) {
        if (skidBuffer[tid].front()->isLoad()) {
            toRename->iewInfo[tid].dispatchedToLQ++;
        }
        if (skidBuffer[tid].front()->isStore()) {
            toRename->iewInfo[tid].dispatchedToSQ++;
        }

        toRename->iewInfo[tid].dispatched++;

        skidBuffer[tid].pop();
    }

    emptyRenameInsts(tid);
}

template<class Impl>
void
DefaultIEW<Impl>::squashDueToBranch(DynInstPtr &inst, ThreadID tid)
{
    DPRINTF(IEW, "[tid:%i]: Branch misprediction, squashing from a specific instruction, PC: %s "
            "[sn:%i].\n", tid, inst->pcState(), inst->seqNum);

    if (!toCommit->squash[tid] ||
            inst->seqNum < toCommit->squashedSeqNum[tid]) {
        toCommit->squash[tid] = true;
        toCommit->squashedSeqNum[tid] = inst->seqNum;
        toCommit->branchTaken[tid] = inst->pcState().branching();
        toCommit->oldpc[tid] = inst->pcState();

        TheISA::PCState pc = inst->pcState();
        TheISA::advancePC(pc, inst->staticInst);

        toCommit->pc[tid] = pc;
        assert(inst);
        toCommit->mispredictInst[tid] = inst;
        toCommit->includeSquashInst[tid] = false;

        toCommit->squashDueToLVP[tid] = false;

        wroteToTimeBuffer = true;
    }

}

//*****CHANGE START**********
template<class Impl>
void
DefaultIEW<Impl>::squashDueToLoad(DynInstPtr &inst, DynInstPtr &firstDependent, ThreadID tid)
{
    assert(inst->isStreamedFromSpeculativeCache());

    DPRINTF(LVP, "[tid:%i]: Memory misprediction, squashing younger "
            "insts from %i, PC: %s [sn:%i].\n", tid, inst->seqNum, inst->pcState(), inst->seqNum);

    // If already squashing, LVP takes precedence
    // Using < instead of <= would give branch precedence
    if ((!toCommit->squash[tid] || inst->seqNum <= toCommit->squashedSeqNum[tid])) {
        toCommit->squash[tid] = true;
        //toCommit->squashedSeqNum[tid] = firstDependent->seqNum;
        toCommit->squashedSeqNum[tid] = inst->seqNum;
        toCommit->pc[tid] = inst->pcState();
        toCommit->oldpc[tid] = inst->pcState();
        assert(inst);
        toCommit->mispredictInst[tid] = inst; // not a branch misprediction
        toCommit->includeSquashInst[tid] = false;
        inst->forwardOldRegs();
        inst->setSquashedAndCommited(true);
        toCommit->squashDueToLVP[tid] = true;

        wroteToTimeBuffer = true;
        //instsSquashedByLVP[tid] += (cpu->globalSeqNum - inst->seqNum);
    } else if (toCommit->squash[tid] && inst->seqNum > toCommit->squashedSeqNum[tid]) {
        DPRINTF(LVP, "Already squashing from [sn:%i], so skipping\n", toCommit->squashedSeqNum[tid]);
    }

    // New stats
    bool isInUop = cpu->fetch.decoder[tid]->isHitInUopCache(inst->pcState().instAddr());
    // bool isInSpec = cpu->fetch.decoder[tid]->isHitInSpeculativeCache(inst->pcState().instAddr());
    // if (isInUop && isInSpec) {
    squashedLoadsPresentInBothCaches[tid]++;
    if (isInUop) {
        squashedLoadsOnlyInUopCache[tid]++;
    } else {
        squashedLoadsInNeitherCache[tid]++;
    }
}

template<class Impl>
void
DefaultIEW<Impl>::squashDueToForwardedValueMisprediction(DynInstPtr &inst, DynInstPtr &firstDependent, ThreadID tid)
{
    DPRINTF(LVP, "[tid:%i]: Forwarded value misprediction, squashing younger "
            "insts from %i, PC: %s [sn:%i].\n", tid, inst->seqNum, inst->pcState(), inst->seqNum);

    // If already squashing, LVP takes precedence
    // Using < instead of <= would give branch precedence
    if ((!toCommit->squash[tid] || inst->seqNum <= toCommit->squashedSeqNum[tid])) {
        toCommit->squash[tid] = true;
        //toCommit->squashedSeqNum[tid] = firstDependent->seqNum;
        toCommit->squashedSeqNum[tid] = inst->seqNum;

        toCommit->oldpc[tid] = inst->pcState();
        TheISA::PCState pc = inst->pcState();
        TheISA::advancePC(pc, inst->staticInst);
        toCommit->pc[tid] = pc;

        assert(inst);
        toCommit->mispredictInst[tid] = inst; // not a branch misprediction
        toCommit->includeSquashInst[tid] = false;
       // inst->forwardOldRegs();
        //inst->setSquashedAndCommited(true);
        toCommit->squashDueToLVP[tid] = false;//false??

        wroteToTimeBuffer = true;
        //instsSquashedByLVP[tid] += (cpu->globalSeqNum - inst->seqNum);
    } else if (toCommit->squash[tid] && inst->seqNum > toCommit->squashedSeqNum[tid]) {
        DPRINTF(LVP, "Already squashing from [sn:%i], so skipping\n", toCommit->squashedSeqNum[tid]);
    }
}
//*****CHANGE END**********

template<class Impl>
void
DefaultIEW<Impl>::squashDueToMemOrder(DynInstPtr &inst, ThreadID tid)
{
    DPRINTF(IEW, "[tid:%i]: Memory violation, squashing violator and younger "
            "insts, PC: %s [sn:%i].\n", tid, inst->pcState(), inst->seqNum);
    // Need to include inst->seqNum in the following comparison to cover the
    // corner case when a branch misprediction and a memory violation for the
    // same instruction (e.g. load PC) are detected in the same cycle.  In this
    // case the memory violator should take precedence over the branch
    // misprediction because it requires the violator itself to be included in
    // the squash.
    if (!toCommit->squash[tid] ||
            inst->seqNum <= toCommit->squashedSeqNum[tid]) {
        toCommit->squash[tid] = true;

        toCommit->squashedSeqNum[tid] = inst->seqNum;
        toCommit->pc[tid] = inst->pcState();
        toCommit->oldpc[tid] = inst->pcState();
        toCommit->mispredictInst[tid] = NULL;
        
        // in this way we can find out whether mem order violation was in the trace or not
        if (inst->isStreamedFromSpeculativeCache()) {
            assert(inst->staticInst->getTraceID());
            toCommit->squashDueToLVP[tid] = true;
            toCommit->currentTraceID[tid] = inst->staticInst->getTraceID();
        } else {
            toCommit->squashDueToLVP[tid] = false;
        }
        
        // Must include the memory violator in the squash.
        toCommit->includeSquashInst[tid] = true;

        wroteToTimeBuffer = true;
	DPRINTF(IEW, "[tid:%i]: Memory order violation, squashing younger insts from PC: %s [sn:%i].\n", tid, inst->pcState(), inst->seqNum);
    }
}

template<class Impl>
void
DefaultIEW<Impl>::block(ThreadID tid)
{
    DPRINTF(IEW, "[tid:%u]: Blocking.\n", tid);

    if (dispatchStatus[tid] != Blocked &&
        dispatchStatus[tid] != Unblocking) {
        toRename->iewBlock[tid] = true;
        wroteToTimeBuffer = true;
    }

    // Add the current inputs to the skid buffer so they can be
    // reprocessed when this stage unblocks.
    skidInsert(tid);

    dispatchStatus[tid] = Blocked;
}

template<class Impl>
void
DefaultIEW<Impl>::unblock(ThreadID tid)
{
    DPRINTF(IEW, "[tid:%i]: Reading instructions out of the skid "
            "buffer %u.\n",tid, tid);

    // If the skid bufffer is empty, signal back to previous stages to unblock.
    // Also switch status to running.
    if (skidBuffer[tid].empty()) {
        toRename->iewUnblock[tid] = true;
        wroteToTimeBuffer = true;
        DPRINTF(IEW, "[tid:%i]: Done unblocking.\n",tid);
        dispatchStatus[tid] = Running;
    }
}

template<class Impl>
void
DefaultIEW<Impl>::wakeDependents(DynInstPtr &inst)
{
    instQueue.wakeDependents(inst);
}


// this method is only for forwarding load microops! We have a diffrent method for non-load microops
template<class Impl>
bool
DefaultIEW<Impl>::forwardLoadValuePredictionToDependents(DynInstPtr &inst)
{   
    //assert(0);
    assert(inst->isLoad()); // just to make sure we are not using it by mistake!

    bool forwarded = false;

    forwarded = instQueue.forwardLoadValuePredictionToDependents(inst);
    if (forwarded)
    {
            assert(inst->numDestRegs() == 1); // We know that loads always have one dest reg
            // Maybe do this for superopt: if (inst->staticInst->liveOutPredicted[i]) continue;
            scoreboard->setReg(inst->renamedDestRegIdx(0));
            //iewStage->scoreboard->setReg(new_inst->renamedDestRegIdx(i));
            //instQueue.regScoreboard[inst->renamedDestRegIdx(0)->flatIndex()] = true;

            DPRINTF(LVP, "Load Value Forwarded and Updated scoreboard for register %i.\n", inst->renamedDestRegIdx(0));
    }
    else 
    {
        DPRINTF(LVP, "Couldn't forward the predicted value for Load microop [sn:%i]\n", inst->seqNum);
    }

    

    return forwarded;
    
}

template<class Impl>
void
DefaultIEW<Impl>::rescheduleMemInst(DynInstPtr &inst)
{
    instQueue.rescheduleMemInst(inst);
}

template<class Impl>
void
DefaultIEW<Impl>::replayMemInst(DynInstPtr &inst)
{
    instQueue.replayMemInst(inst);
}

template<class Impl>
void
DefaultIEW<Impl>::blockMemInst(DynInstPtr& inst)
{
    instQueue.blockMemInst(inst);
}

template<class Impl>
void
DefaultIEW<Impl>::cacheUnblocked()
{
    instQueue.cacheUnblocked();
}

template<class Impl>
void
DefaultIEW<Impl>::instToCommit(DynInstPtr &inst)
{
    // This function should not be called after writebackInsts in a
    // single cycle.  That will cause problems with an instruction
    // being added to the queue to commit without being processed by
    // writebackInsts prior to being sent to commit.

    // First check the time slot that this instruction will write
    // to.  If there are free write ports at the time, then go ahead
    // and write the instruction to that time.  If there are not,
    // keep looking back to see where's the first time there's a
    // free slot.
    while ((*iewQueue)[wbCycle].insts[wbNumInst]) {
        ++wbNumInst;
        if (wbNumInst == wbWidth) {
            ++wbCycle;
            wbNumInst = 0;
        }
    }

    DPRINTF(IEW, "Current wb cycle: %i, width: %i, numInst: %i\nwbActual:%i\n",
            wbCycle, wbWidth, wbNumInst, wbCycle * wbWidth + wbNumInst);
    // Add finished instruction to queue to commit.
    (*iewQueue)[wbCycle].insts[wbNumInst] = inst;
    (*iewQueue)[wbCycle].size++;
}

template <class Impl>
unsigned
DefaultIEW<Impl>::validInstsFromRename()
{
    unsigned inst_count = 0;

    for (int i=0; i<fromRename->size; i++) {
        if (!fromRename->insts[i]->isSquashed())
            inst_count++;
    }

    return inst_count;
}

template<class Impl>
void
DefaultIEW<Impl>::skidInsert(ThreadID tid)
{
    DynInstPtr inst = NULL;

    while (!insts[tid].empty()) {
        inst = insts[tid].front();

        insts[tid].pop();

        DPRINTF(IEW,"[tid:%i]: Inserting [sn:%lli] PC:%s into "
                "dispatch skidBuffer %i\n",tid, inst->seqNum,
                inst->pcState(),tid);

        skidBuffer[tid].push(inst);
    }

    assert(skidBuffer[tid].size() <= skidBufferMax &&
           "Skidbuffer Exceeded Max Size");
}

template<class Impl>
int
DefaultIEW<Impl>::skidCount()
{
    int max=0;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;
        unsigned thread_count = skidBuffer[tid].size();
        if (max < thread_count)
            max = thread_count;
    }

    return max;
}

template<class Impl>
bool
DefaultIEW<Impl>::skidsEmpty()
{
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (!skidBuffer[tid].empty())
            return false;
    }

    return true;
}

template <class Impl>
void
DefaultIEW<Impl>::updateStatus()
{
    bool any_unblocking = false;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (dispatchStatus[tid] == Unblocking) {
            any_unblocking = true;
            break;
        }
    }

    // If there are no ready instructions waiting to be scheduled by the IQ,
    // and there's no stores waiting to write back, and dispatch is not
    // unblocking, then there is no internal activity for the IEW stage.
    instQueue.intInstQueueReads++;
    if (_status == Active && !instQueue.hasReadyInsts() &&
        !ldstQueue.willWB() && !any_unblocking) {
        DPRINTF(IEW, "IEW switching to idle\n");

        deactivateStage();

        _status = Inactive;
    } else if (_status == Inactive && (instQueue.hasReadyInsts() ||
                                       ldstQueue.willWB() ||
                                       any_unblocking)) {
        // Otherwise there is internal activity.  Set to active.
        DPRINTF(IEW, "IEW switching to active\n");

        activateStage();

        _status = Active;
    }
}

template <class Impl>
void
DefaultIEW<Impl>::resetEntries()
{
    instQueue.resetEntries();
    ldstQueue.resetEntries();
}

template <class Impl>
bool
DefaultIEW<Impl>::checkStall(ThreadID tid)
{
    bool ret_val(false);

    if (fromCommit->commitInfo[tid].robSquashing) {
        ++ stalledDueToCommit;
        DPRINTF(IEW,"[tid:%i]: Stall from Commit stage detected.\n",tid);
        ret_val = true;
    } else if (instQueue.isFull(tid)) {
        ++ stalledDueToIQFull;
        DPRINTF(IEW,"[tid:%i]: Stall: IQ  is full.\n",tid);
        ret_val = true;
    }

    return ret_val;
}

template <class Impl>
void
DefaultIEW<Impl>::checkSignalsAndUpdate(ThreadID tid)
{
    // Check if there's a squash signal, squash if there is
    // Check stall signals, block if there is.
    // If status was Blocked
    //     if so then go to unblocking
    // If status was Squashing
    //     check if squashing is not high.  Switch to running this cycle.

    if (fromCommit->commitInfo[tid].squash) {
        squash(tid);

        if (dispatchStatus[tid] == Blocked ||
            dispatchStatus[tid] == Unblocking) {
            toRename->iewUnblock[tid] = true;
            wroteToTimeBuffer = true;
        }

        dispatchStatus[tid] = Squashing;
        fetchRedirect[tid] = false;
        return;
    }

    if (fromCommit->commitInfo[tid].robSquashing) {
         DPRINTF(IEW, "[tid:%i]: ROB is still squashing.\n", tid);
        dispatchStatus[tid] = Squashing;
        emptyRenameInsts(tid);
        wroteToTimeBuffer = true;
    }

    if (checkStall(tid)) {
        block(tid);
        dispatchStatus[tid] = Blocked;
        return;
    }

    if (dispatchStatus[tid] == Blocked) {
        // Status from previous cycle was blocked, but there are no more stall
        // conditions.  Switch over to unblocking.
        DPRINTF(IEW, "[tid:%i]: Done blocking, switching to unblocking.\n",
                tid);

        dispatchStatus[tid] = Unblocking;

        unblock(tid);

        return;
    }

    if (dispatchStatus[tid] == Squashing) {
        // Switch status to running if rename isn't being told to block or
        // squash this cycle.
        DPRINTF(IEW, "[tid:%i]: Done squashing, switching to running.\n",
                tid);

        dispatchStatus[tid] = Running;

        return;
    }
}

template <class Impl>
void
DefaultIEW<Impl>::sortInsts()
{
    int insts_from_rename = fromRename->size;
#ifdef DEBUG
    for (ThreadID tid = 0; tid < numThreads; tid++)
        assert(insts[tid].empty());
#endif
    for (int i = 0; i < insts_from_rename; ++i) {
        insts[fromRename->insts[i]->threadNumber].push(fromRename->insts[i]);
    }
}

template <class Impl>
void
DefaultIEW<Impl>::emptyRenameInsts(ThreadID tid)
{
    DPRINTF(IEW, "[tid:%i]: Removing incoming rename instructions\n", tid);

    while (!insts[tid].empty()) {

        if (insts[tid].front()->isLoad()) {
            toRename->iewInfo[tid].dispatchedToLQ++;
        }
        if (insts[tid].front()->isStore()) {
            toRename->iewInfo[tid].dispatchedToSQ++;
        }

        toRename->iewInfo[tid].dispatched++;

        insts[tid].pop();
    }
}

template <class Impl>
void
DefaultIEW<Impl>::wakeCPU()
{
    cpu->wakeCPU();
}

template <class Impl>
void
DefaultIEW<Impl>::activityThisCycle()
{
    DPRINTF(Activity, "Activity this cycle.\n");
    cpu->activityThisCycle();
}

template <class Impl>
inline void
DefaultIEW<Impl>::activateStage()
{
    DPRINTF(Activity, "Activating stage.\n");
    cpu->activateStage(O3CPU::IEWIdx);
}

template <class Impl>
inline void
DefaultIEW<Impl>::deactivateStage()
{
    DPRINTF(Activity, "Deactivating stage.\n");
    cpu->deactivateStage(O3CPU::IEWIdx);
}

template<class Impl>
void
DefaultIEW<Impl>::dispatch(ThreadID tid)
{
    // If status is Running or idle,
    //     call dispatchInsts()
    // If status is Unblocking,
    //     buffer any instructions coming from rename
    //     continue trying to empty skid buffer
    //     check if stall conditions have passed

    if (dispatchStatus[tid] == Blocked) {
        ++iewBlockCycles;

    } else if (dispatchStatus[tid] == Squashing) {
        ++iewSquashCycles;
    }

    // Dispatch should try to dispatch as many instructions as its bandwidth
    // will allow, as long as it is not currently blocked.
    if (dispatchStatus[tid] == Running ||
        dispatchStatus[tid] == Idle) {
        DPRINTF(IEW, "[tid:%i] Not blocked, so attempting to run "
                "dispatch.\n", tid);

        dispatchInsts(tid);
    } else if (dispatchStatus[tid] == Unblocking) {
        // Make sure that the skid buffer has something in it if the
        // status is unblocking.
        assert(!skidsEmpty());

        // If the status was unblocking, then instructions from the skid
        // buffer were used.  Remove those instructions and handle
        // the rest of unblocking.
        dispatchInsts(tid);

        ++iewUnblockCycles;

        if (validInstsFromRename()) {
            // Add the current inputs to the skid buffer so they can be
            // reprocessed when this stage unblocks.
            skidInsert(tid);
        }

        unblock(tid);
    }
}

template <class Impl>
void
DefaultIEW<Impl>::dispatchInsts(ThreadID tid)
{
    // Obtain instructions from skid buffer if unblocking, or queue from rename
    // otherwise.
    std::queue<DynInstPtr> &insts_to_dispatch =
        dispatchStatus[tid] == Unblocking ?
        skidBuffer[tid] : insts[tid];

    int insts_to_add = insts_to_dispatch.size();

    DynInstPtr inst;
    bool add_to_iq = false;
    int dis_num_inst = 0;

    // Loop through the instructions, putting them in the instruction
    // queue.
    for ( ; dis_num_inst < insts_to_add &&
              dis_num_inst < dispatchWidth;
          ++dis_num_inst)
    {
        inst = insts_to_dispatch.front();

        if (dispatchStatus[tid] == Unblocking) {
            DPRINTF(IEW, "[tid:%i]: Issue: Examining instruction from skid "
                    "buffer\n", tid);
        }

        // Make sure there's a valid instruction there.
        assert(inst);

        DPRINTF(IEW, "[tid:%i]: Issue: Adding PC %s [sn:%lli] [tid:%i] to "
                "IQ.\n",
                tid, inst->pcState(), inst->seqNum, inst->threadNumber);

        // Be sure to mark these instructions as ready so that the
        // commit stage can go ahead and execute them, and mark
        // them as issued so the IQ doesn't reprocess them.

        // Check for squashed instructions.
        if (inst->isSquashed()) {
            DPRINTF(IEW, "[tid:%i]: Issue: Squashed instruction encountered, "
                    "not adding to IQ.\n", tid);

            ++iewDispSquashedInsts;

            insts_to_dispatch.pop();

            //Tell Rename That An Instruction has been processed
            if (inst->isLoad()) {
                toRename->iewInfo[tid].dispatchedToLQ++;
            }
            if (inst->isStore()) {
                toRename->iewInfo[tid].dispatchedToSQ++;
            }

            toRename->iewInfo[tid].dispatched++;

            continue;
        }

        // Check for full conditions.
        if (instQueue.isFull(tid)) {
            DPRINTF(IEW, "[tid:%i]: Issue: IQ has become full.\n", tid);

            // Call function to start blocking.
            block(tid);

            // Set unblock to false. Special case where we are using
            // skidbuffer (unblocking) instructions but then we still
            // get full in the IQ.
            toRename->iewUnblock[tid] = false;

            ++iewIQFullEvents;
            break;
        }

        // Check LSQ if inst is LD/ST
        if ((inst->isLoad() && ldstQueue.lqFull(tid)) ||
            (inst->isStore() && ldstQueue.sqFull(tid))) {
            DPRINTF(IEW, "[tid:%i]: Issue: %s has become full.\n",tid,
                    inst->isLoad() ? "LQ" : "SQ");

            // Call function to start blocking.
            block(tid);

            // Set unblock to false. Special case where we are using
            // skidbuffer (unblocking) instructions but then we still
            // get full in the IQ.
            toRename->iewUnblock[tid] = false;

            // LQ Full
            if (inst->isLoad() && ldstQueue.lqFull(tid))
            {
                int totalNumOfLoads = ldstQueue.numLoads(tid);
                int totalNumOfSpeculativeLoads = ldstQueue.numSpeculativeLoads(tid);
                double percentage = (double) (totalNumOfSpeculativeLoads * 100.0)/ ((double)totalNumOfLoads);
                assert(percentage <= 100.0);
                LQFullScenariosDist.sample((unsigned)percentage);
            }
            // SQ Full
            else
            {
                int totalNumOfStores = ldstQueue.numStores(tid);
                int totalNumOfSpeculativeStores = ldstQueue.numSpeculativeStores(tid);
                double percentage = (double) (totalNumOfSpeculativeStores * 100.0)/ ((double)totalNumOfStores);
                assert(percentage <= 100.0);
                SQFullScenariosDist.sample((unsigned)percentage);
            }

            ++iewLSQFullEvents;
            break;
        }

        // Otherwise issue the instruction just fine.
        if (inst->isLoad()) {
            DPRINTF(IEW, "[tid:%i]: Issue: Memory instruction "
                    "encountered, adding to LSQ.\n", tid);

            // Reserve a spot in the load store queue for this
            // memory access.
            ldstQueue.insertLoad(inst);

            if (inst->isStreamedFromSpeculativeCache())
            {
                ++iewDispSpeculativeLoadInsts;
            }
        
            ++iewDispLoadInsts;
            

            

            add_to_iq = true;

            toRename->iewInfo[tid].dispatchedToLQ++;
        } else if (inst->isStore()) {
            DPRINTF(IEW, "[tid:%i]: Issue: Memory instruction "
                    "encountered, adding to LSQ.\n", tid);

            ldstQueue.insertStore(inst);

            if (inst->isStreamedFromSpeculativeCache())
            {
                ++iewDispSpeculativeStoreInsts;
            }
            
            ++iewDispStoreInsts;
            
            

            if (inst->isStoreConditional()) {
                // Store conditionals need to be set as "canCommit()"
                // so that commit can process them when they reach the
                // head of commit.
                // @todo: This is somewhat specific to Alpha.
                inst->setCanCommit();
                instQueue.insertNonSpec(inst);
                add_to_iq = false;

                ++iewDispNonSpecInsts;
            } else {
                add_to_iq = true;
            }

            toRename->iewInfo[tid].dispatchedToSQ++;
        } else if (inst->isMemBarrier() || inst->isWriteBarrier()) {
            // Same as non-speculative stores.
            inst->setCanCommit();
            instQueue.insertBarrier(inst);
            add_to_iq = false;
        } else if (inst->isNop()) {
            DPRINTF(IEW, "[tid:%i]: Issue: Nop instruction encountered, "
                    "skipping.\n", tid);

            inst->setIssued();
            inst->setExecuted();
            inst->setCanCommit();

            instQueue.recordProducer(inst);

            iewExecutedNop[tid]++;

            add_to_iq = false;
        } else {
            assert(!inst->isExecuted());
            add_to_iq = true;
        }

        if (add_to_iq && inst->isNonSpeculative()) {
            DPRINTF(IEW, "[tid:%i]: Issue: Nonspeculative instruction "
                    "encountered, skipping.\n", tid);

            // Same as non-speculative stores.
            inst->setCanCommit();

            // Specifically insert it as nonspeculative.
            instQueue.insertNonSpec(inst);

            ++iewDispNonSpecInsts;

            add_to_iq = false;
        }

        // If the instruction queue is not full, then add the
        // instruction.
        if (add_to_iq) {
            instQueue.insert(inst);
        }

        insts_to_dispatch.pop();

        toRename->iewInfo[tid].dispatched++;

        ++iewDispatchedInsts;

#if TRACING_ON
        inst->dispatchTick = curTick() - inst->fetchTick;
#endif
        ppDispatch->notify(inst);
    }

    if (!insts_to_dispatch.empty()) {
        ++ blockingDueToBandwidthFull;
        DPRINTF(IEW,"[tid:%i]: Issue: Bandwidth Full. Blocking.\n", tid);
        block(tid);
        toRename->iewUnblock[tid] = false;
    }

    if (dispatchStatus[tid] == Idle && dis_num_inst) {
        dispatchStatus[tid] = Running;

        updatedQueues = true;
    }

    dis_num_inst = 0;
}

template <class Impl>
void
DefaultIEW<Impl>::printAvailableInsts()
{
    int inst = 0;

    std::cout << "Available Instructions: ";

    while (fromIssue->insts[inst]) {

        if (inst%3==0) std::cout << "\n\t";

        std::cout << "PC: " << fromIssue->insts[inst]->pcState()
             << " TN: " << fromIssue->insts[inst]->threadNumber
             << " SN: " << fromIssue->insts[inst]->seqNum << " | ";

        inst++;

    }

    std::cout << "\n";
}

template <class Impl>
void
DefaultIEW<Impl>::executeInsts()
{
    wbNumInst = 0;
    wbCycle = 0;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;
        fetchRedirect[tid] = false;
    }

    // Uncomment this if you want to see all available instructions.
    // @todo This doesn't actually work anymore, we should fix it.
//    printAvailableInsts();

    // Execute/writeback any instructions that are available.
    int insts_to_execute = fromIssue->size;
    int inst_num = 0;
    for (; inst_num < insts_to_execute;
          ++inst_num) {

        DPRINTF(IEW, "Execute: Executing instructions from IQ.\n");

        DynInstPtr inst = instQueue.getInstToExecute();

        DPRINTF(IEW, "Execute: Processing PC %s, [tid:%i] [sn:%i].\n",
                inst->pcState(), inst->threadNumber,inst->seqNum);

        // Notify potential listeners that this instruction has started
        // executing
        ppExecute->notify(inst);

        // Check if the instruction is squashed; if so then skip it
        if (inst->isSquashed()) {
            DPRINTF(IEW, "Execute: Instruction was squashed. PC: %s, [tid:%i]"
                         " [sn:%i]\n", inst->pcState(), inst->threadNumber,
                         inst->seqNum);

            // Consider this instruction executed so that commit can go
            // ahead and retire the instruction.
            inst->setExecuted();

            // Not sure if I should set this here or just let commit try to
            // commit any squashed instructions.  I like the latter a bit more.
            inst->setCanCommit();

            ++iewExecSquashedInsts;

            continue;
        }

        Fault fault = NoFault;

        ThreadID tid = inst->threadNumber;
        // Execute instruction. 
        // Note that if the instruction faults, it will be handled
        // at the commit stage.
        if (inst->isMemRef()) {
            DPRINTF(IEW, "Execute: Calculating address for memory "
                    "reference.\n");

            // Tell the LDSTQ to execute this instruction (if it is a load).
            if (inst->isLoad()) {
                if (enableValuePredForwarding && (inst->staticInst->predictedLoad && (!cpu->fetch.decoder[tid]->isSuperOptimizationPresent || !inst->isStreamedFromSpeculativeCache()))) {
                    assert(!inst->isSquashed());
                    
                    //inst->memoryAccessStartCycle = cpu->numCycles.value();

                    DPRINTF(LVP, "Fetch Predicted (%d) Value: %#x and Confidence %d\n", 
                                inst->staticInst->predictedLoad, inst->staticInst->predictedValue, inst->staticInst->confidence);


                    if ((inst->staticInst->confidence >= cpu->fetch.decoder[tid]->traceConstructor->predictionConfidenceThreshold)) {
                        if ( inst->getFault() == NoFault) {
                            DPRINTF(LVP, "Waking dependencies of [sn:%i] early with prediction\n", inst->seqNum);
                            //forwardLoadValuePredictionToDependents(inst);
                        }
                           
                    }


                }
                // Loads will mark themselves as executed, and their writeback
                // event adds the instruction to the queue to commit
                fault = ldstQueue.executeLoad(inst);

                if (inst->isTranslationDelayed() &&
                    fault == NoFault) {
                    // A hw page table walk is currently going on; the
                    // instruction must be deferred.
                    DPRINTF(IEW, "Execute: Delayed translation, deferring "
                            "load.\n");
                    instQueue.deferMemInst(inst);
                    continue;
                }

                if (inst->isDataPrefetch() || inst->isInstPrefetch()) {
                    inst->fault = NoFault;
                }
            } else if (inst->isStore()) {
                fault = ldstQueue.executeStore(inst);

                if (inst->isTranslationDelayed() &&
                    fault == NoFault) {
                    // A hw page table walk is currently going on; the
                    // instruction must be deferred.
                    DPRINTF(IEW, "Execute: Delayed translation, deferring "
                            "store.\n");
                    instQueue.deferMemInst(inst);
                    continue;
                }

                // If the store had a fault then it may not have a mem req
                if (fault != NoFault || !inst->readPredicate() ||
                        !inst->isStoreConditional()) {
                    // If the instruction faulted, then we need to send it along
                    // to commit without the instruction completing.
                    // Send this instruction to commit, also make sure iew stage
                    // realizes there is activity.
                    inst->setExecuted();
                    instToCommit(inst);
                    activityThisCycle();
                }

                // Store conditionals will mark themselves as
                // executed, and their writeback event will add the
                // instruction to the queue to commit.
            } else {
                panic("Unexpected memory type!\n");
            }

        } else {
            // If the instruction has already faulted, then skip executing it.
            // Such case can happen when it faulted during ITLB translation.
            // If we execute the instruction (even if it's a nop) the fault
            // will be replaced and we will lose it.
            if (inst->getFault() == NoFault) {
                if (!inst->staticInst->dummyMicroop) {
                    inst->execute(); // this is op specific! so need to read dest reg to get returned value
                }

                if (!inst->readPredicate() || inst->staticInst->dummyMicroop) {
                    
                    if (inst->staticInst->dummyMicroop)
                        DPRINTF(IEW, "Instruction is a dummy microop at the end of super-optimized trace! Not executing it! PC=%s\n", inst->pcState());

                    inst->forwardOldRegs();
		        }
                // Unconditional LVP update for arithmatic instructions
                if (loadPred->predictingArithmetic && inst->staticInst->predictedLoad) 
                { 
                    //assert(inst->isStreamedFromSpeculativeCache());
                    assert(inst->isStreamedFromUOpCache());
                    assert(!inst->isStreamedFromSpeculativeCache());
                    assert(!inst->isTracePredictionSource());
                    inst->memoryAccessEndCycle = cpu->numCycles.value(); 

                    updateLoadValuePredictor(inst);
                }
                // here we decide whether we want to squash or not due to a LVP missprediction
                if (inst->isStreamedFromSpeculativeCache() && inst->isTracePredictionSource())
                {
                    assert(inst->isStreamedFromSpeculativeCache());
                    assert(!inst->isStreamedFromUOpCache());
                    assert(!inst->staticInst->predictedLoad); // prediction sources that are coing from spec cache never should have this set
                    assert(loadPred->predictingArithmetic); // only when LVP is enabled for arithmatic operations
                    
                    checkForLVPMissprediction(inst);
                }
                
                if (inst->isSpeculativlyForwarded())
                {
                    assert(inst->isStreamedFromSpeculativeCache());
                    assert(!inst->isStreamedFromSpeculativeCache());
                    assert(inst->staticInst->predictedLoad); // prediction sources that are coing from spec cache never should have this set
                    //assert(loadPred->predictingArithmetic); // only when LVP is enabled for arithmatic operations

                    checkForLVPMissprediction(inst);
                }

            	
            }

            inst->setExecuted();

            instToCommit(inst);

            if (inst->potentialLoopEnd) {
                ++loopsFromLtage;
                // cout << "SN:" << inst->seqNum << " marked as loop end" << endl;
                DPRINTF(LVP, "Inst sn:%i marked as loop end\n", inst->seqNum);
                // loadPred->lastMisprediction = cpu->numCycles.value();
            }
        }

        updateExeInstStats(inst);

        // if branch misprediction is detectable by this point, we should have a nextInstAddr()...
        loadPred->updateGtables(inst->instAddr(), inst->nextInstAddr(), inst->isControl());

        // Check if branch prediction was correct, if not then we need
        // to tell commit to squash in flight instructions.  Only
        // handle this if there hasn't already been something that
        // redirects fetch in this group of instructions.

        // This probably needs to prioritize the redirects if a different
        // scheduler is used.  Currently the scheduler schedules the oldest
        // instruction first, so the branch resolution order will be correct.
        //ThreadID tid = inst->threadNumber;

        if (!fetchRedirect[tid] ||
            !toCommit->squash[tid] ||
            toCommit->squashedSeqNum[tid] > inst->seqNum) {

            // Prevent testing for misprediction on load instructions,
            // that have not been executed.
            bool loadNotExecuted = !inst->isExecuted() && inst->isLoad();

            TheISA::PCState tempPC = inst->pcState();
            TheISA::advancePC(tempPC, inst->staticInst);
            
            if ((!inst->isStreamedFromSpeculativeCache() || (inst->isControl() && inst->isLastMicroop() && !inst->staticInst->dummyMicroop)) && inst->mispredicted() && !loadNotExecuted) {
                DPRINTF(IEW, "mismatch? target PC=%s, predicted PC=%s\n", tempPC, inst->readPredTarg());
                fetchRedirect[tid] = true;

                DPRINTF(IEW, "Execute: Branch mispredict detected.\n");
                DPRINTF(IEW, "Predicted target was PC: %s.\n",
                        inst->readPredTarg());
                DPRINTF(IEW, "Execute: Redirecting fetch to PC: %s.\n",
                        inst->pcState());

                updateTraceBranchConfidence(inst, tempPC, false);       
                // If incorrect, then signal the ROB that it must be squashed.
                squashDueToBranch(inst, tid);

                ppMispredict->notify(inst);

                //bool specTraceCorrect = inst->staticInst->predictedTarget.instAddr() == tempPC.instAddr() && inst->staticInst->predictedTaken == inst->readPredTaken();
               // bool thisInstCorrect = false;


                //if (!inst->isStreamedFromSpeculativeCache())
                //{
                    if (!inst->isIndirectCtrl() && inst->readPredTaken()) {
                        predictedTakenIncorrect++;
                        if(inst->isStreamedFromSpeculativeCache()){
                            if (inst->branchPredFromPredictor){
                                specTracePredTakenIncorrectFromBranchPred ++;
                            }
                            if(inst->branchPredFromTrace){
                                specTracePredTakenIncorrectFromTrace ++;
                            }
                        } else if (inst->isStreamedFromUOpCache()){
                            specTracePredTakenIncorrectFromUopCache++;
                        } else {
                            specTracePredTakenIncorrectFromICache++;
                        }
                    } else if (!inst->isIndirectCtrl()) {
                        predictedNotTakenIncorrect++;
                        if(inst->isStreamedFromSpeculativeCache()){
                            if (inst->branchPredFromPredictor){
                                specTracePredNotTakenIncorrectFromBranchPred ++;
                            }
                            if(inst->branchPredFromTrace){
                                specTracePredNotTakenIncorrectFromTrace ++;
                            }
                        } else if (inst->isStreamedFromUOpCache()){
                            specTracePredNotTakenIncorrectFromUopCache++;
                        } else {
                            specTracePredNotTakenIncorrectFromICache++;
                        }
                    } else if (inst->isIndirectCtrl()) {
                        predictedIndirectIncorrect++;
                        if(inst->isStreamedFromSpeculativeCache()){
                            if (inst->branchPredFromPredictor){
                                specTracePredIndirectIncorrectFromBranchPred ++;
                            }
                            if(inst->branchPredFromTrace){
                                specTracePredIndirectIncorrectFromTrace ++;
                            }
                        } else if (inst->isStreamedFromUOpCache()){
                            specTracePredIndirectIncorrectFromUopCache++;
                        } else {
                            specTracePredIndirectIncorrectFromICache++;
                        }
                    } else {
                        predictedUnknownBranchIncorrect++;
                    }
                //}

            } else if (ldstQueue.violation(tid)) {
                assert(inst->isMemRef());
                // If there was an ordering violation, then get the
                // DynInst that caused the violation.  Note that this
                // clears the violation signal.
                DynInstPtr violator;
                violator = ldstQueue.getMemDepViolator(tid);

                DPRINTF(IEW, "LDSTQ detected a violation. Violator PC: %s "
                        "[sn:%lli], inst PC: %s [sn:%lli]. Addr is: %#x.\n",
                        violator->pcState(), violator->seqNum,
                        inst->pcState(), inst->seqNum, inst->physEffAddrLow);

                fetchRedirect[tid] = true;

                // Tell the instruction queue that a violation has occured.
                instQueue.violation(inst, violator);

                // Squash.
                DPRINTF(LVP, "Starting mem order violation squash from %i\n", violator->seqNum);
		squashDueToMemOrder(violator, tid);

                ++memOrderViolationEvents;
            }
        } else {
            // Reset any state associated with redirects that will not
            // be used.
            if (ldstQueue.violation(tid)) {
                assert(inst->isMemRef());

                DynInstPtr violator = ldstQueue.getMemDepViolator(tid);

                DPRINTF(IEW, "LDSTQ detected a violation.  Violator PC: "
                        "%s, inst PC: %s.  Addr is: %#x.\n",
                        violator->pcState(), inst->pcState(),
                        inst->physEffAddrLow);
                DPRINTF(IEW, "Violation will not be handled because "
                        "already squashing\n");

                ++memOrderViolationEvents;
            }
        }
    }

    // Update and record activity if we processed any instructions.
    if (inst_num) {
        if (exeStatus == Idle) {
            exeStatus = Running;
        }

        updatedQueues = true;

        cpu->activityThisCycle();
    }

    // Need to reset this in case a writeback event needs to write into the
    // iew queue.  That way the writeback event will write into the correct
    // spot in the queue.
    wbNumInst = 0;

}

template <class Impl>
void
DefaultIEW<Impl>::writebackInsts()
{
    // Loop through the head of the time buffer and wake any
    // dependents.  These instructions are about to write back.  Also
    // mark scoreboard that this instruction is finally complete.
    // Either have IEW have direct access to scoreboard, or have this
    // as part of backwards communication.
    for (int inst_num = 0; inst_num < wbWidth &&
             toCommit->insts[inst_num]; inst_num++) {
        DynInstPtr inst = toCommit->insts[inst_num];
        ThreadID tid = inst->threadNumber;

        DPRINTF(IEW, "Sending instructions to commit, [sn:%lli] PC %s.\n",
                inst->seqNum, inst->pcState());

        iewInstsToCommit[tid]++;
        // Notify potential listeners that execution is complete for this
        // instruction.
        ppToCommit->notify(inst);

        // Some instructions will be sent to commit without having
        // executed because they need commit to handle them.
        // E.g. Strictly ordered loads have not actually executed when they
        // are first sent to commit.  Instead commit must tell the LSQ
        // when it's ready to execute the strictly ordered load.
        if (!inst->isSquashed() && inst->isExecuted() && inst->getFault() == NoFault) {
            int dependents = 0;
            //if(!inst->isSpeculativlyForwarded()){
                dependents = instQueue.wakeDependents(inst);
            //}
            for (int i = 0; i < inst->numDestRegs(); i++) {
                if (inst->staticInst->liveOutPredicted[i]) continue;
                //mark as Ready
                DPRINTF(IEW,"Setting Destination Register %i (%s)\n",
                        inst->renamedDestRegIdx(i)->index(),
                        inst->renamedDestRegIdx(i)->className());
                //instQueue.unsetPredictedReg(inst->renamedDestRegIdx(i)->flatIndex());
                scoreboard->setReg(inst->renamedDestRegIdx(i));
            }

            if (dependents) {
                producerInst[tid]++;
                consumerInst[tid]+= dependents;
            }
            writebackCount[tid]++;
        }
    }
}

template<class Impl>
void
DefaultIEW<Impl>::tick()
{
    wbNumInst = 0;
    wbCycle = 0;

    wroteToTimeBuffer = false;
    updatedQueues = false;

    sortInsts();

    // Free function units marked as being freed this cycle.
    fuPool->processFreeUnits();

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    // Check stall and squash signals, dispatch any instructions.
    while (threads != end) {
        ThreadID tid = *threads++;

        DPRINTF(IEW,"Issue: Processing [tid:%i]\n",tid);

        checkSignalsAndUpdate(tid);
        dispatch(tid);
    }

    if (exeStatus != Squashing) {
        executeInsts();

        writebackInsts();

        // Have the instruction queue try to schedule any ready instructions.
        // (In actuality, this scheduling is for instructions that will
        // be executed next cycle.)
        instQueue.scheduleReadyInsts();

        // Also should advance its own time buffers if the stage ran.
        // Not the best place for it, but this works (hopefully).
        issueToExecQueue.advance();
    }

    bool broadcast_free_entries = false;

    if (updatedQueues || exeStatus == Running || updateLSQNextCycle) {
        exeStatus = Idle;
        updateLSQNextCycle = false;

        broadcast_free_entries = true;
    }

    // Writeback any stores using any leftover bandwidth.
    ldstQueue.writebackStores();

    // Check the committed load/store signals to see if there's a load
    // or store to commit.  Also check if it's being told to execute a
    // nonspeculative instruction.
    // This is pretty inefficient...

    threads = activeThreads->begin();
    while (threads != end) {
        ThreadID tid = (*threads++);

        DPRINTF(IEW,"Processing [tid:%i]\n",tid);

        // Update structures based on instructions committed.
        if (fromCommit->commitInfo[tid].doneSeqNum != 0 &&
            !fromCommit->commitInfo[tid].squash &&
            !fromCommit->commitInfo[tid].robSquashing) {

            ldstQueue.commitStores(fromCommit->commitInfo[tid].doneSeqNum,tid);

            ldstQueue.commitLoads(fromCommit->commitInfo[tid].doneSeqNum,tid);

            updateLSQNextCycle = true;
            instQueue.commit(fromCommit->commitInfo[tid].doneSeqNum,tid);
        }

        if (fromCommit->commitInfo[tid].nonSpecSeqNum != 0) {

            //DPRINTF(IEW,"NonspecInst from thread %i",tid);
            if (fromCommit->commitInfo[tid].strictlyOrdered) {
                instQueue.replayMemInst(
                    fromCommit->commitInfo[tid].strictlyOrderedLoad);
                fromCommit->commitInfo[tid].strictlyOrderedLoad->setAtCommit();
            } else {
                instQueue.scheduleNonSpec(
                    fromCommit->commitInfo[tid].nonSpecSeqNum);
            }
        }

        if (broadcast_free_entries) {
            toFetch->iewInfo[tid].iqCount =
                instQueue.getCount(tid);
            toFetch->iewInfo[tid].ldstqCount =
                ldstQueue.getCount(tid);

            toRename->iewInfo[tid].usedIQ = true;
            toRename->iewInfo[tid].freeIQEntries =
                instQueue.numFreeEntries(tid);
            toRename->iewInfo[tid].usedLSQ = true;

            toRename->iewInfo[tid].freeLQEntries =
                ldstQueue.numFreeLoadEntries(tid);
            toRename->iewInfo[tid].freeSQEntries =
                ldstQueue.numFreeStoreEntries(tid);

            wroteToTimeBuffer = true;
        }

        DPRINTF(IEW, "[tid:%i], Dispatch dispatched %i instructions.\n",
                tid, toRename->iewInfo[tid].dispatched);
    }

    DPRINTF(IEW, "IQ has %i free entries (Can schedule: %i).  "
            "LQ has %i free entries. SQ has %i free entries.\n",
            instQueue.numFreeEntries(), instQueue.hasReadyInsts(),
            ldstQueue.numFreeLoadEntries(), ldstQueue.numFreeStoreEntries());

    updateStatus();

    if (wroteToTimeBuffer) {
        DPRINTF(Activity, "Activity this cycle.\n");
        cpu->activityThisCycle();
    }
}

template <class Impl>
void
DefaultIEW<Impl>::updateExeInstStats(DynInstPtr &inst)
{
    ThreadID tid = inst->threadNumber;

    iewExecutedInsts++;

#if TRACING_ON
    if (DTRACE(O3PipeView)) {
        inst->completeTick = curTick() - inst->fetchTick;
    }
#endif

    //
    //  Control operations
    //
    if (inst->isControl())
        iewExecutedBranches[tid]++;

    //
    //  Memory operations
    //
    if (inst->isMemRef()) {
        iewExecutedRefs[tid]++;

        if (inst->isLoad()) {
            iewExecLoadInsts[tid]++;
        }
    }
}

template <class Impl>
void
DefaultIEW<Impl>::checkMisprediction(DynInstPtr &inst)
{

    //assert(0);
    ThreadID tid = inst->threadNumber;

    if (!fetchRedirect[tid] ||
        !toCommit->squash[tid] ||
        toCommit->squashedSeqNum[tid] > inst->seqNum) {

        TheISA::PCState tempPC = inst->pcState();
        TheISA::advancePC(tempPC, inst->staticInst);
        DPRINTF(IEW, "mismatch? target PC=%s, predicted PC=%s\n", tempPC, inst->readPredTarg());
        if ((!inst->isStreamedFromSpeculativeCache() || (inst->isControl() && inst->isLastMicroop() && !inst->staticInst->dummyMicroop)) && inst->mispredicted()) {
            fetchRedirect[tid] = true;

            DPRINTF(IEW, "Execute: Branch mispredict detected.\n");
            DPRINTF(IEW, "Predicted target was PC:%#x, NPC:%#x.\n",
                    inst->predInstAddr(), inst->predNextInstAddr());
            DPRINTF(IEW, "Execute: Redirecting fetch to PC: %#x,"
                    " NPC: %#x.\n", inst->nextInstAddr(),
                    inst->nextInstAddr());
            // If incorrect, then signal the ROB that it must be squashed.
            squashDueToBranch(inst, tid);
            updateTraceBranchConfidence(inst, tempPC ,false);

            if (inst->isStreamedFromSpeculativeCache()){
                cout << "WE DONT COUNT THIS" << endl;
            }
            //if (!inst->isStreamedFromSpeculativeCache())
            //{
            //    if (inst->readPredTaken()) {
            //        predictedTakenIncorrect++;
            //    } else {
            //        predictedNotTakenIncorrect++;
            //    }
            //}

        }
    }
}

template <class Impl>
void
DefaultIEW<Impl>::updateTraceBranchConfidence(DynInstPtr &inst, TheISA::PCState& tempPC, bool predicted)
{
    ThreadID tid = inst->threadNumber;

    

    if (inst->isStreamedFromSpeculativeCache() && inst->branchPredFromTrace)
    {
        
                
        uint64_t traceID = inst->staticInst->getTraceID();
        
        // this should never be zero
        assert(traceID);
        if (!predicted) totalNumOfTimesControlSourcesOfTracesAreMisspredicted++;

        // this trace may have been flushed therfore there is no need to update its confidence
        if (cpu->fetch.decoder[tid]->traceConstructor->traceMap.find(traceID) == cpu->fetch.decoder[tid]->traceConstructor->traceMap.end())
        {
            // this may get trigered if the trace get evicted from spec cache. TODO: implement transient trace in spec cache 
            DPRINTF(LVP, "DefaultIEW::executeInsts():: Couldn't find trace %d in traceMap! Is it flushed? \n", traceID);
            return;
        }

        DPRINTF(LVP, "updateTraceBranchConfidence:: TragetPC=%s Updating trace %d cofidence level!\n",tempPC.instAddr(), traceID);

        // missprediction
        if (!predicted) 
        {
            cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].totalNumOfTimesControlSourcesAreMisspredicted++;
            bool updated = false;
            for (size_t idx = 0; idx < 2; idx++)
            {
                if (cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].controlSources[idx].valid &&
                    cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].controlSources[idx].value ==  tempPC.instAddr())
                {
                    if (cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].controlSources[idx].confidence > 0 )
                    {
                        cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].controlSources[idx].confidence--;
                        cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].controlSources[idx].numOfTimesMisspredicted++;
                        DPRINTF(LVP, "DefaultIEW::executeInsts():: Missprediction! Traget: %s Decreasing trace %d confidence! Confidence level is %d!  Number of times this branch source is misspredicted: %d\n", 
                        tempPC.instAddr(), traceID, cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].controlSources[idx].confidence,
                        cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].controlSources[idx].numOfTimesMisspredicted);
                        
                    }
                    //cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].totalNumOfTimesControlSourcesAreMisspredicted++;
                    updated = true;
                }
            }

            updated = updated;
            // assert(updated);

            
        } 
        //correct prediction
        else 
        {
            for (size_t idx = 0; idx < 2; idx++)
            {
                if (cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].controlSources[idx].valid &&
                    cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].controlSources[idx].value ==  tempPC.instAddr())
                {
                    if (cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].controlSources[idx].confidence < 9 )
                    {
                        cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].controlSources[idx].confidence++;
                        DPRINTF(LVP, "DefaultIEW::executeInsts():: Correct! Increasing trace %d confidence! Confidence level is %d\n", traceID, 
                            cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].controlSources[idx].confidence);
                    }
                }
            }
        }       
    } 
}


template <class Impl>
void
DefaultIEW<Impl>::updateTraceConfidence(DynInstPtr &inst)
{
    ThreadID tid = inst->threadNumber;
    Addr addr = inst->pcState().instAddr();
    uint64_t traceID = inst->staticInst->getTraceID();
    
    // this should never be zero
    assert(traceID);
    if (inst->lvMispred)  
    {
        totalNumOfTimesPredictionSourcesOfTracesAreMisspredicted++; 
    }

    // this trace may have been flushed therfore there is no need to update its confidence
    if (cpu->fetch.decoder[tid]->traceConstructor->traceMap.find(traceID) == cpu->fetch.decoder[tid]->traceConstructor->traceMap.end())
    {
        // this may get trigered if the trace get evicted from spec cache. TODO: implement transient trace in spec cache 
        DPRINTF(LVP, "DefaultIEW::executeInsts():: Couldn't find trace %d in traceMap! Is it flushed? \n", traceID);
        return;
    }

    DPRINTF(LVP, "updateTraceConfidence:: Updating trace %d cofidence level!\n", traceID);
    if (!inst->lvMispred)
    {
        
        bool updated = false;
        // We just update the confidence in trace because Decoder::isTraceAvaible() is going to use this confidence value
        for (int i=0; i<4; i++) {
            if (cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].source[i].valid && 
                cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].source[i].addr.pcAddr == addr) 
            {
                // Is there a limit on confidence? I'm assuming 9.
                if (cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].source[i].confidence < 9) 
                    cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].source[i].confidence++;
                
                DPRINTF(LVP, "DefaultIEW::executeInsts():: Correct Prediction! Increasing trace %d confidence! Confidence level is %d\n", traceID, 
                        cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].source[i].confidence);
                updated = true;
            }
        }

        
        assert(updated);

    }
    else 
    {
        
        bool updated = false;
        // We just update the confidence in trace because Decoder::isTraceAvaible() is going to use this confidence value
        for (int i=0; i<4; i++) {
            if (cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].source[i].valid && 
                cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].source[i].addr.pcAddr == addr) 
            {
                // Is there a limit on confidence? I'm assuming 0.
                if (cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].source[i].confidence > 0) 
                    cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].source[i].confidence--;
                
                cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].source[i].numOfTimesMisspredicted++;
                
                DPRINTF(LVP, "DefaultIEW::executeInsts():: Missprediction! Decreasing trace %d confidence! Confidence level is %d! Number of times this prediction source misspredicted: %d\n", 
                        traceID, 
                        cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].source[i].confidence, 
                        cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].source[i].numOfTimesMisspredicted);

                        
                cpu->fetch.decoder[tid]->traceConstructor->traceMap[traceID].totalNumOfTimesPredictionSourcesAreMisspredicted++;
                updated = true;
            }
        }
        
        assert(updated);

    }


}

template <class Impl>
void
DefaultIEW<Impl>::updateLoadValuePredictor(DynInstPtr& inst)
{


        DPRINTF(LVP, "Sending a response to LVP from [sn:%i]\n", inst->seqNum);
        DPRINTF(LVP, "Inst->confidence is %d at time of return\n", inst->staticInst->confidence); 
	
        int numIntDestRegs = 0 ;
        for (int i=0; i<inst->numDestRegs(); i++) {
            PhysRegIdPtr dest_reg = inst->renamedDestRegIdx(i);
            uint64_t value;
            switch (dest_reg->classValue()) 
            {
			    // Note: changed memoryAccessStartCycle to cycleFetched in all these
                case IntRegClass:
                    numIntDestRegs++;
                    value = cpu->readIntReg(dest_reg);
                    DPRINTF(LVP, "IntRegClass: Returning register value %llx to LVP i.e. %llx\n", value, cpu->readIntReg(dest_reg));

                    // gathering some statics
                    //cpu->fetch.decoder[inst->threadNumber]->insertReturnedValueIntoUopCacheStatics(inst->pcState(), value, inst->staticInst->predictedValue);
                            
                    loadPred->processPacketRecieved(inst->pcState(), inst->staticInst, value, inst->threadNumber, inst->staticInst->predictedValue, inst->staticInst->confidence, inst->memoryAccessEndCycle - inst->memoryAccessStartCycle, cpu->numCycles.value());
                    break;
                            
                case CCRegClass:
                case FloatRegClass:
                case VecRegClass:
                case VecElemClass:
                case MiscRegClass:
                    break;
                default:
                    panic("Unsupported Register Class: %d", (int)dest_reg->classValue());
            }
        }

        // always should be one! We never predict for non int dest regs
        assert(numIntDestRegs == 1);
                    


}

template <class Impl>
void
DefaultIEW<Impl>::checkForLVPMissprediction(DynInstPtr& inst)
{
 
        //assert(!inst->isSpeculativlyForwarded()); // for now disable LV forwarding

        // Check for missprediction
        uint64_t reg_value;
        int64_t mask = -1; // Might be a problem (unsigned?)
        int numIntDestRegs = 0 ;
        for (int i=0; i<inst->numDestRegs(); i++) {
            mask = -1;
            PhysRegIdPtr dest_reg = inst->renamedDestRegIdx(i);
            
            switch (dest_reg->classValue()) 
            {
			    // Note: changed memoryAccessStartCycle to cycleFetched in all these
                case IntRegClass:
                    if (inst->staticInst->liveOutPredicted[i]) 
                    {
                        assert(inst->isStreamedFromSpeculativeCache());
                        // this is not a real dest reg, it's a dest reg added to dump all the live outs at the end of a trace
                        // therefore, just igonore it
                        continue;
                    }
                    numIntDestRegs++;
                    reg_value = cpu->readIntReg(dest_reg);     
                   
                    if (inst->staticInst->getDataSize() != 8) { 
                        mask = (1ull << (inst->staticInst->getDataSize()*8)) - 1;
                    }
                    DPRINTF(LVP, "before masking: predicted value=%#x actual value=%#x\n", inst->staticInst->predictedValue, reg_value);
                    DPRINTF(LVP, "after masking: predicted value=%#x actual value=%#x data size=%i\n", inst->staticInst->predictedValue & mask, reg_value & mask, inst->staticInst->getDataSize());
                    inst->lvMispred = ((reg_value & mask) != (inst->staticInst->predictedValue & mask)); 
                    
                    loadPred->processPacketRecieved(inst->pcState(), inst->staticInst, reg_value & mask, inst->threadNumber, inst->staticInst->predictedValue & mask, inst->staticInst->confidence, inst->memoryAccessEndCycle - inst->memoryAccessStartCycle, cpu->numCycles.value());
                    
                    //if (inst->staticInst->confidence >= 5) {
                       // loadPred->processPacketRecieved(inst->pcState(), inst->staticInst, inst->staticInst->predictedValue-100, inst->threadNumber, inst->staticInst->predictedValue, inst->staticInst->confidence, inst->memoryAccessEndCycle - inst->memoryAccessStartCycle, cpu->numCycles.value());
                       // inst->lvMispred = true;
                    //}
                    break;
                            
                case CCRegClass:
                case FloatRegClass:
                case VecRegClass:
                case VecElemClass:
                case MiscRegClass:
                    break;
                default:
                    panic("Unsupported Register Class: %d", (int)dest_reg->classValue());
            }
        }

        // always should be one! We never predict for non int dest regs
        assert(numIntDestRegs == 1);

        if (inst->lvMispred  && inst->isStreamedFromSpeculativeCache() && inst->isTracePredictionSource()) 
        {
            DPRINTF(LVP, "DefaultIEW::executeInsts():: OH NO! processPacketRecieved returned false :(\n");
            DPRINTF(LVP, "DefaultIEW::executeInsts():: Missprediction for a trace prediction source!\n");
            // cpu->fetch.updateConstantBuffer(inst->pcState().instAddr(), false);
            loadPred->lastMisprediction = inst->memoryAccessEndCycle;
            // Moved from commit
            squashDueToLoad(inst, inst, inst->threadNumber);
        }
                    
        // we can have non-load instructions which are streamed from speculative cache and thier values are forwaded speculativly
        else if (inst->lvMispred && inst->isSpeculativlyForwarded()) 
        {
            assert(inst->isLoad());
            assert(!inst->isStreamedFromSpeculativeCache());
            DPRINTF(LVP, "DefaultIEW::executeInsts():: OH NO! processPacketRecieved returned false :(\n");
            DPRINTF(LVP, "DefaultIEW::executeInsts():: Missprediction for a instruction which is not a trace prediction source!\n");
            //DPRINTF(LVP, "DefaultIEW::executeInsts():: isStreamedFromSpeculativeCache? %d\n", inst->isStreamedFromSpeculativeCache());
            // cpu->fetch.updateConstantBuffer(inst->pcState().instAddr(), false);
            loadPred->lastMisprediction = inst->memoryAccessEndCycle;
            // Moved from commit
            squashDueToForwardedValueMisprediction(inst, inst, inst->threadNumber);
                    
        }

        // logic to update the trace confidences base on prediction result
        if ( inst->isStreamedFromSpeculativeCache() && inst->isTracePredictionSource())
        {
            updateTraceConfidence(inst);
        }
    

}




#endif//__CPU_O3_IEW_IMPL_IMPL_HH__
