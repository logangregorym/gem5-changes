/*
 * Copyright 2014 Google, Inc.
 * Copyright (c) 2010-2014, 2017 ARM Limited
 * All rights reserved
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
 *          Korey Sewell
 */
#ifndef __CPU_O3_COMMIT_IMPL_HH__
#define __CPU_O3_COMMIT_IMPL_HH__

#include <algorithm>
#include <set>
#include <string>

#include "arch/utility.hh"
#include "arch/x86/decoder_structs.hh"
#include "base/loader/symtab.hh"
#include "base/cp_annotate.hh"
#include "config/the_isa.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/o3/commit.hh"
#include "cpu/o3/thread_state.hh"
#include "cpu/base.hh"
#include "cpu/exetrace.hh"
#include "cpu/timebuf.hh"
#include "debug/Activity.hh"
#include "debug/Commit.hh"
#include "debug/CommitRate.hh"
#include "debug/Drain.hh"
#include "debug/ExecFaulting.hh"
#include "debug/LVP.hh"
#include "debug/O3PipeView.hh"
#include "params/DerivO3CPU.hh"
#include "sim/faults.hh"
#include "sim/full_system.hh"
#include "debug/SuperOpSanityCheck.hh"
#include "debug/PerfAnalysis.hh"
#include "debug/TraceEviction.hh"

using namespace std;

template <class Impl>
void
DefaultCommit<Impl>::processTrapEvent(ThreadID tid)
{
    // This will get reset by commit if it was switched out at the
    // time of this event processing.
    trapSquash[tid] = true;
}

template <class Impl>
DefaultCommit<Impl>::DefaultCommit(O3CPU *_cpu, DerivO3CPUParams *params)
    : cpu(_cpu),
      iewToCommitDelay(params->iewToCommitDelay),
      commitToIEWDelay(params->commitToIEWDelay),
      renameToROBDelay(params->renameToROBDelay),
      fetchToCommitDelay(params->commitToFetchDelay),
      renameWidth(params->renameWidth),
      commitWidth(params->commitWidth),
      numThreads(params->numThreads),
      drainPending(false),
      drainImminent(false),
      trapLatency(params->trapLatency),
      canHandleInterrupts(true),
      avoidQuiesceLiveLock(false)
{
    if (commitWidth > Impl::MaxWidth)
        fatal("commitWidth (%d) is larger than compiled limit (%d),\n"
             "\tincrease MaxWidth in src/cpu/o3/impl.hh\n",
             commitWidth, static_cast<int>(Impl::MaxWidth));

    _status = Active;
    _nextStatus = Inactive;
    std::string policy = params->smtCommitPolicy;

    //Convert string to lowercase
    std::transform(policy.begin(), policy.end(), policy.begin(),
                   (int(*)(int)) tolower);

    //Assign commit policy
    if (policy == "aggressive"){
        commitPolicy = Aggressive;

        DPRINTF(Commit,"Commit Policy set to Aggressive.\n");
    } else if (policy == "roundrobin"){
        commitPolicy = RoundRobin;

        //Set-Up Priority List
        for (ThreadID tid = 0; tid < numThreads; tid++) {
            priority_list.push_back(tid);
        }

        DPRINTF(Commit,"Commit Policy set to Round Robin.\n");
    } else if (policy == "oldestready"){
        commitPolicy = OldestReady;

        DPRINTF(Commit,"Commit Policy set to Oldest Ready.");
    } else {
        assert(0 && "Invalid SMT Commit Policy. Options Are: {Aggressive,"
               "RoundRobin,OldestReady}");
    }

    for (ThreadID tid = 0; tid < numThreads; tid++) {
        commitStatus[tid] = Idle;
        changedROBNumEntries[tid] = false;
        checkEmptyROB[tid] = false;
        trapInFlight[tid] = false;
        committedStores[tid] = false;
        trapSquash[tid] = false;
        tcSquash[tid] = false;
        pc[tid].set(0);
        lastCommitedSeqNum[tid] = 0;
        squashAfterInst[tid] = NULL;
    }
    interrupt = NoFault;

    checkpointAtInstr = params->checkpoint_at_instr;
    afterExecCnt = 1;

    numMicroopsShrunken = 0;
}

template <class Impl>
std::string
DefaultCommit<Impl>::name() const
{
    return cpu->name() + ".commit";
}

template <class Impl>
void
DefaultCommit<Impl>::regProbePoints()
{
    ppCommit = new ProbePointArg<DynInstPtr>(cpu->getProbeManager(), "Commit");
    ppCommitStall = new ProbePointArg<DynInstPtr>(cpu->getProbeManager(), "CommitStall");
    ppSquash = new ProbePointArg<DynInstPtr>(cpu->getProbeManager(), "Squash");
}

template <class Impl>
void
DefaultCommit<Impl>::regStats()
{
    using namespace Stats;
    commitSquashedInsts
        .name(name() + ".commitSquashedInsts")
        .desc("The number of squashed insts skipped by commit")
        .prereq(commitSquashedInsts);

    commitSquashedInstFromSpecLVP
        .name(name() + ".commitSquashedInstFromSpecLVP")
        .desc("The number of squashed insts skipped by commit, streamed from Speculative Cache, due to LVP");

    commitSquashedInstsFromSpecNotLVP
        .name(name() + ".commitSquashedInstsFromSpecNotLVP")
        .desc("The number of squashed insts skipped by commit, streamed from Speculative Cache, not due to LVP");
    
    commitSquashedInstFromUopLVP
        .name(name() + ".commitSquashedInstFromUopLVP")
        .desc("The number of squashed insts skipped by commit, streamed from Uop Cache, due to LVP");

    commitSquashedInstFromUopNotLVP
        .name(name() + ".commitSquashedInstFromUopNotLVP")
        .desc("The number of squashed insts skipped by commit, streamed from Uop Cache, not due to LVP");

    commitSquashedInstFromICacheLVP
        .name(name() + ".commitSquashedInstFromICacheLVP")
        .desc("The number of squashed insts skipped by commit, streamed from Instruction Cache, due to LVP");

    commitSquashedInstFromICacheNotLVP
        .name(name() + ".commitSquashedInstFromICacheNotLVP")
        .desc("The number of squashed insts skipped by commit, streamed from Instruction Cache, not due to LVP");

    commitNonSpecStalls
        .name(name() + ".commitNonSpecStalls")
        .desc("The number of times commit has been forced to stall to "
              "communicate backwards")
        .prereq(commitNonSpecStalls);

    branchMispredicts
        .name(name() + ".branchMispredicts")
        .desc("The number of times a branch was mispredicted")
        .prereq(branchMispredicts);

    numCommittedDist
        .init(0,commitWidth,1)
        .name(name() + ".committed_per_cycle")
        .desc("Number of insts commited each cycle")
        .flags(Stats::pdf)
        ;

    // 18 is the maximum length of a superoptmized trace
    numCommittedSuperoptmizedInstDist
        .init(0,18,1)
        .name(name() + ".committed_superoptmized_insts_per_trace_size")
        .desc("Number of commited superoptmized insts per trace size")
        .flags(Stats::pdf)
        ;
    instsCommitted
        .init(cpu->numThreads)
        .name(name() + ".committedInsts")
        .desc("Number of instructions committed")
        .flags(total)
        ;

    opsCommitted
        .init(cpu->numThreads)
        .name(name() + ".committedOps")
        .desc("Number of ops (including micro ops) committed")
        .flags(total)
        ;

    statComSwp
        .init(cpu->numThreads)
        .name(name() + ".swp_count")
        .desc("Number of s/w prefetches committed")
        .flags(total)
        ;

    statComRefs
        .init(cpu->numThreads)
        .name(name() +  ".refs")
        .desc("Number of memory references committed")
        .flags(total)
        ;

    statComLoads
        .init(cpu->numThreads)
        .name(name() +  ".loads")
        .desc("Number of loads committed")
        .flags(total)
        ;

    statComMembars
        .init(cpu->numThreads)
        .name(name() +  ".membars")
        .desc("Number of memory barriers committed")
        .flags(total)
        ;

    statComBranches
        .init(cpu->numThreads)
        .name(name() + ".branches")
        .desc("Number of branches committed")
        .flags(total)
        ;

    statComFloating
        .init(cpu->numThreads)
        .name(name() + ".fp_insts")
        .desc("Number of committed floating point instructions.")
        .flags(total)
        ;

    statComVector
        .init(cpu->numThreads)
        .name(name() + ".vec_insts")
        .desc("Number of committed Vector instructions.")
        .flags(total)
        ;

    statComInteger
        .init(cpu->numThreads)
        .name(name()+".int_insts")
        .desc("Number of committed integer instructions.")
        .flags(total)
        ;

    statComFunctionCalls
        .init(cpu->numThreads)
        .name(name()+".function_calls")
        .desc("Number of function calls committed.")
        .flags(total)
        ;

    statCommittedInstType
        .init(numThreads,Enums::Num_OpClass)
        .name(name() + ".op_class")
        .desc("Class of committed instruction")
        .flags(total | pdf | dist)
        ;
    statCommittedInstType.ysubnames(Enums::OpClassStrings);

    commitEligibleSamples
        .name(name() + ".bw_lim_events")
        .desc("number cycles where commit BW limit reached")
        ;

    lvpPredsCommitted
        .name(name() + ".lvpPredsCommitted")
        .desc("Number of lvp predictions used and not squashed")
        ;

    lvpPredCommitPercent
        .name(name() + ".lvpPredCommitPercent")
        .desc("Percent of committed loads that have a predictions")
        .precision(6)
        ;
    lvpPredCommitPercent = (lvpPredsCommitted / statComLoads) * 100;

    reducableCommitted
        .name(name() + ".reducableCommitted")
        .desc("Number of reducable instructions that are committed")
        ;

    reducableCommitPercent1
        .name(name() + ".reducableCommittedPercent1")
        .desc("Percent of committed instructions that are reducable (with instsCommitted)")
        ;
    reducableCommitPercent1 = (reducableCommitted / instsCommitted) * 100;

    reducableCommitPercent2
        .name(name() + ".reducableCommitPercent2")
        .desc("Percent of committed instructions that are reducable (with opsCommitted)")
        ;
    reducableCommitPercent2 = (reducableCommitted / opsCommitted) * 100;

    exec_cnt
        .name(name() + ".progress")
        .desc("Progress counter")
        ;

}

template <class Impl>
void
DefaultCommit<Impl>::setThreads(std::vector<Thread *> &threads)
{
    thread = threads;
}

template <class Impl>
void
DefaultCommit<Impl>::setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr)
{
    timeBuffer = tb_ptr;

    // Setup wire to send information back to IEW.
    toIEW = timeBuffer->getWire(0);

    // Setup wire to read data from IEW (for the ROB).
    robInfoFromIEW = timeBuffer->getWire(-iewToCommitDelay);
}

template <class Impl>
void
DefaultCommit<Impl>::setFetchQueue(TimeBuffer<FetchStruct> *fq_ptr)
{
    fetchQueue = fq_ptr;

    // Setup wire to get instructions from rename (for the ROB).
    fromFetch = fetchQueue->getWire(-fetchToCommitDelay);
}

template <class Impl>
void
DefaultCommit<Impl>::setRenameQueue(TimeBuffer<RenameStruct> *rq_ptr)
{
    renameQueue = rq_ptr;

    // Setup wire to get instructions from rename (for the ROB).
    fromRename = renameQueue->getWire(-renameToROBDelay);
}

template <class Impl>
void
DefaultCommit<Impl>::setIEWQueue(TimeBuffer<IEWStruct> *iq_ptr)
{
    iewQueue = iq_ptr;

    // Setup wire to get instructions from IEW.
    fromIEW = iewQueue->getWire(-iewToCommitDelay);
}

template <class Impl>
void
DefaultCommit<Impl>::setIEWStage(IEW *iew_stage)
{
    iewStage = iew_stage;
}

template<class Impl>
void
DefaultCommit<Impl>::setActiveThreads(list<ThreadID> *at_ptr)
{
    activeThreads = at_ptr;
}

template <class Impl>
void
DefaultCommit<Impl>::setRenameMap(RenameMap rm_ptr[])
{
    for (ThreadID tid = 0; tid < numThreads; tid++)
        renameMap[tid] = &rm_ptr[tid];
}

template <class Impl>
void
DefaultCommit<Impl>::setROB(ROB *rob_ptr)
{
    rob = rob_ptr;
}

template <class Impl>
void
DefaultCommit<Impl>::startupStage()
{
    rob->setActiveThreads(activeThreads);
    rob->resetEntries();

    // Broadcast the number of free entries.
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        toIEW->commitInfo[tid].usedROB = true;
        toIEW->commitInfo[tid].freeROBEntries = rob->numFreeEntries(tid);
        toIEW->commitInfo[tid].emptyROB = true;
    }

    // Commit must broadcast the number of free entries it has at the
    // start of the simulation, so it starts as active.
    cpu->activateStage(O3CPU::CommitIdx);

    cpu->activityThisCycle();
}

template <class Impl>
void
DefaultCommit<Impl>::drain()
{
    drainPending = true;
}

template <class Impl>
void
DefaultCommit<Impl>::drainResume()
{
    drainPending = false;
    drainImminent = false;
}

template <class Impl>
void
DefaultCommit<Impl>::drainSanityCheck() const
{
    assert(isDrained());
    rob->drainSanityCheck();
}

template <class Impl>
bool
DefaultCommit<Impl>::isDrained() const
{
    /* Make sure no one is executing microcode. There are two reasons
     * for this:
     * - Hardware virtualized CPUs can't switch into the middle of a
     *   microcode sequence.
     * - The current fetch implementation will most likely get very
     *   confused if it tries to start fetching an instruction that
     *   is executing in the middle of a ucode sequence that changes
     *   address mappings. This can happen on for example x86.
     */
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        if (pc[tid].microPC() != 0)
            return false;
    }

    /* Make sure that all instructions have finished committing before
     * declaring the system as drained. We want the pipeline to be
     * completely empty when we declare the CPU to be drained. This
     * makes debugging easier since CPU handover and restoring from a
     * checkpoint with a different CPU should have the same timing.
     */
    return rob->isEmpty() &&
        interrupt == NoFault;
}

template <class Impl>
void
DefaultCommit<Impl>::takeOverFrom()
{
    _status = Active;
    _nextStatus = Inactive;
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        commitStatus[tid] = Idle;
        changedROBNumEntries[tid] = false;
        trapSquash[tid] = false;
        tcSquash[tid] = false;
        squashAfterInst[tid] = NULL;
    }
    rob->takeOverFrom();
}

template <class Impl>
void
DefaultCommit<Impl>::deactivateThread(ThreadID tid)
{
    list<ThreadID>::iterator thread_it = std::find(priority_list.begin(),
            priority_list.end(), tid);

    if (thread_it != priority_list.end()) {
        priority_list.erase(thread_it);
    }
}


template <class Impl>
void
DefaultCommit<Impl>::updateStatus()
{
    // reset ROB changed variable
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        changedROBNumEntries[tid] = false;

        // Also check if any of the threads has a trap pending
        if (commitStatus[tid] == TrapPending ||
            commitStatus[tid] == FetchTrapPending) {
            _nextStatus = Active;
        }
    }

    if (_nextStatus == Inactive && _status == Active) {
        DPRINTF(Activity, "Deactivating stage.\n");
        cpu->deactivateStage(O3CPU::CommitIdx);
    } else if (_nextStatus == Active && _status == Inactive) {
        DPRINTF(Activity, "Activating stage.\n");
        cpu->activateStage(O3CPU::CommitIdx);
    }

    _status = _nextStatus;
}

template <class Impl>
bool
DefaultCommit<Impl>::changedROBEntries()
{
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (changedROBNumEntries[tid]) {
            return true;
        }
    }

    return false;
}

template <class Impl>
size_t
DefaultCommit<Impl>::numROBFreeEntries(ThreadID tid)
{
    return rob->numFreeEntries(tid);
}

template <class Impl>
void
DefaultCommit<Impl>::generateTrapEvent(ThreadID tid, Fault inst_fault)
{
    DPRINTF(Commit, "Generating trap event for [tid:%i]\n", tid);

    EventFunctionWrapper *trap = new EventFunctionWrapper(
        [this, tid]{ processTrapEvent(tid); },
        "Trap", true, Event::CPU_Tick_Pri);

    Cycles latency = dynamic_pointer_cast<SyscallRetryFault>(inst_fault) ?
                     cpu->syscallRetryLatency : trapLatency;

    cpu->schedule(trap, cpu->clockEdge(latency));
    trapInFlight[tid] = true;
    thread[tid]->trapPending = true;
}

template <class Impl>
void
DefaultCommit<Impl>::generateTCEvent(ThreadID tid)
{
    assert(!trapInFlight[tid]);
    DPRINTF(Commit, "Generating TC squash event for [tid:%i]\n", tid);

    tcSquash[tid] = true;
}

template <class Impl>
void
DefaultCommit<Impl>::squashAll(ThreadID tid)
{
    // If we want to include the squashing instruction in the squash,
    // then use one older sequence number.
    // Hopefully this doesn't mess things up.  Basically I want to squash
    // all instructions of this thread.
    InstSeqNum squashed_inst = rob->isEmpty(tid) ?
        lastCommitedSeqNum[tid] : rob->readHeadInst(tid)->seqNum - 1;

    // All younger instructions will be squashed. Set the sequence
    // number as the youngest instruction in the ROB (0 in this case.
    // Hopefully nothing breaks.)
    youngestSeqNum[tid] = lastCommitedSeqNum[tid];

    rob->squash(squashed_inst, tid, false);
    changedROBNumEntries[tid] = true;

    // Send back the sequence number of the squashed instruction.
    toIEW->commitInfo[tid].doneSeqNum = squashed_inst;

    // Send back the squash signal to tell stages that they should
    // squash.
    toIEW->commitInfo[tid].squash = true;

    // Send back the rob squashing signal so other stages know that
    // the ROB is in the process of squashing.
    toIEW->commitInfo[tid].robSquashing = true;

    toIEW->commitInfo[tid].mispredictInst = NULL;
    toIEW->commitInfo[tid].squashInst = NULL;

    toIEW->commitInfo[tid].pc = pc[tid];
}

template <class Impl>
void
DefaultCommit<Impl>::squashFromTrap(ThreadID tid)
{
    squashAll(tid);

    DPRINTF(Commit, "Squashing from trap, restarting at PC %s\n", pc[tid]);

    thread[tid]->trapPending = false;
    thread[tid]->noSquashFromTC = false;
    trapInFlight[tid] = false;

    trapSquash[tid] = false;

    commitStatus[tid] = ROBSquashing;
    cpu->activityThisCycle();
}

template <class Impl>
void
DefaultCommit<Impl>::squashFromTC(ThreadID tid)
{
    squashAll(tid);

    DPRINTF(Commit, "Squashing from TC, restarting at PC %s\n", pc[tid]);

    thread[tid]->noSquashFromTC = false;
    assert(!thread[tid]->trapPending);

    commitStatus[tid] = ROBSquashing;
    cpu->activityThisCycle();

    tcSquash[tid] = false;
}

template <class Impl>
void
DefaultCommit<Impl>::squashFromSquashAfter(ThreadID tid)
{
    DPRINTF(Commit, "Squashing after squash after request, "
            "restarting at PC %s\n", pc[tid]);

    squashAll(tid);
    // Make sure to inform the fetch stage of which instruction caused
    // the squash. It'll try to re-fetch an instruction executing in
    // microcode unless this is set.
    toIEW->commitInfo[tid].squashInst = squashAfterInst[tid];
    squashAfterInst[tid] = NULL;

    commitStatus[tid] = ROBSquashing;
    cpu->activityThisCycle();
}

template <class Impl>
void
DefaultCommit<Impl>::squashAfter(ThreadID tid, DynInstPtr &head_inst)
{
    DPRINTF(Commit, "Executing squash after for [tid:%i] inst [sn:%lli]\n",
            tid, head_inst->seqNum);

    assert(!squashAfterInst[tid] || squashAfterInst[tid] == head_inst);
    commitStatus[tid] = SquashAfterPending;
    squashAfterInst[tid] = head_inst;
}

template <class Impl>
void
DefaultCommit<Impl>::tick()
{
    wroteToTimeBuffer = false;
    _nextStatus = Inactive;

    if (activeThreads->empty())
        return;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    // Check if any of the threads are done squashing.  Change the
    // status if they are done.
    while (threads != end) {
        ThreadID tid = *threads++;

        // Clear the bit saying if the thread has committed stores
        // this cycle.
        committedStores[tid] = false;

        if (commitStatus[tid] == ROBSquashing) {

            if (rob->isDoneSquashing(tid)) {
                commitStatus[tid] = Running;
            } else {
                DPRINTF(Commit,"[tid:%u]: Still Squashing, cannot commit any"
                        " insts this cycle.\n", tid);
                rob->doSquash(tid, toIEW->commitInfo[tid].squashDueToLVP);
                toIEW->commitInfo[tid].robSquashing = true;
                wroteToTimeBuffer = true;
            }
        }
    }

    commit();

    markCompletedInsts();

    threads = activeThreads->begin();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (!rob->isEmpty(tid) && rob->readHeadInst(tid)->readyToCommit()) {
            // The ROB has more instructions it can commit. Its next status
            // will be active.
            _nextStatus = Active;

            DynInstPtr inst = rob->readHeadInst(tid);

            DPRINTF(Commit,"[tid:%i]: Instruction [sn:%lli] PC %s is head of"
                    " ROB and ready to commit\n",
                    tid, inst->seqNum, inst->pcState());

        } else if (!rob->isEmpty(tid)) {
            DynInstPtr inst = rob->readHeadInst(tid);

            ppCommitStall->notify(inst);

            DPRINTF(Commit,"[tid:%i]: Can't commit, Instruction [sn:%lli] PC "
                    "%s is head of ROB and not ready\n",
                    tid, inst->seqNum, inst->pcState());
        }

        DPRINTF(Commit, "[tid:%i]: ROB has %d insts & %d free entries.\n",
                tid, rob->countInsts(tid), rob->numFreeEntries(tid));
    }


    if (wroteToTimeBuffer) {
        DPRINTF(Activity, "Activity This Cycle.\n");
        cpu->activityThisCycle();
    }

    updateStatus();
}

template <class Impl>
void
DefaultCommit<Impl>::handleInterrupt()
{
    // Verify that we still have an interrupt to handle
    if (!cpu->checkInterrupts(cpu->tcBase(0))) {
        DPRINTF(Commit, "Pending interrupt is cleared by master before "
                "it got handled. Restart fetching from the orig path.\n");
        toIEW->commitInfo[0].clearInterrupt = true;
        interrupt = NoFault;
        avoidQuiesceLiveLock = true;
        return;
    }

    // Wait until all in flight instructions are finished before enterring
    // the interrupt.
    if (canHandleInterrupts && cpu->instList.empty()) {
        // Squash or record that I need to squash this cycle if
        // an interrupt needed to be handled.
        DPRINTF(Commit, "Interrupt detected.\n");

        // Clear the interrupt now that it's going to be handled
        toIEW->commitInfo[0].clearInterrupt = true;

        assert(!thread[0]->noSquashFromTC);
        thread[0]->noSquashFromTC = true;

        if (cpu->checker) {
            cpu->checker->handlePendingInt();
        }

        // CPU will handle interrupt. Note that we ignore the local copy of
        // interrupt. This is because the local copy may no longer be the
        // interrupt that the interrupt controller thinks is being handled.
        cpu->processInterrupts(cpu->getInterrupts());

        thread[0]->noSquashFromTC = false;

        commitStatus[0] = TrapPending;

        interrupt = NoFault;

        // Generate trap squash event.
        generateTrapEvent(0, interrupt);

        avoidQuiesceLiveLock = false;
    } else {
        DPRINTF(Commit, "Interrupt pending: instruction is %sin "
                "flight, ROB is %sempty\n",
                canHandleInterrupts ? "not " : "",
                cpu->instList.empty() ? "" : "not " );
    }
}

template <class Impl>
void
DefaultCommit<Impl>::propagateInterrupt()
{
    // Don't propagate intterupts if we are currently handling a trap or
    // in draining and the last observable instruction has been committed.
    if (commitStatus[0] == TrapPending || interrupt || trapSquash[0] ||
            tcSquash[0] || drainImminent)
        return;

    // Process interrupts if interrupts are enabled, not in PAL
    // mode, and no other traps or external squashes are currently
    // pending.
    // @todo: Allow other threads to handle interrupts.

    // Get any interrupt that happened
    interrupt = cpu->getInterrupts();

    // Tell fetch that there is an interrupt pending.  This
    // will make fetch wait until it sees a non PAL-mode PC,
    // at which point it stops fetching instructions.
    if (interrupt != NoFault)
        toIEW->commitInfo[0].interruptPending = true;
}

template <class Impl>
void
DefaultCommit<Impl>::commit()
{
    if (FullSystem) {
        // Check if we have a interrupt and get read to handle it
        if (cpu->checkInterrupts(cpu->tcBase(0)))
            propagateInterrupt();
    }

    ////////////////////////////////////
    // Check for any possible squashes, handle them first
    ////////////////////////////////////
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    int num_squashing_threads = 0;
    squashDueToLVP = false;
    while (threads != end) {
        ThreadID tid = *threads++;

        // Not sure which one takes priority.  I think if we have
        // both, that's a bad sign.
        if (trapSquash[tid]) {
            assert(!tcSquash[tid]);
            squashFromTrap(tid);
        } else if (tcSquash[tid]) {
            assert(commitStatus[tid] != TrapPending);
            squashFromTC(tid);
        } else if (commitStatus[tid] == SquashAfterPending) {
            // A squash from the previous cycle of the commit stage (i.e.,
            // commitInsts() called squashAfter) is pending. Squash the
            // thread now.
            squashFromSquashAfter(tid);
        }

        // Squashed sequence number must be older than youngest valid
        // instruction in the ROB. This prevents squashes from younger
        // instructions overriding squashes from older instructions.
        if (fromIEW->squash[tid] &&
            commitStatus[tid] != TrapPending &&
            fromIEW->squashedSeqNum[tid] <= youngestSeqNum[tid]) {

            if (fromIEW->mispredictInst[tid]) {
                DPRINTF(Commit,
                    "[tid:%i]: Squashing due to branch or a value mispred PC:%#x [sn:%i]\n",
                    tid,
                    fromIEW->mispredictInst[tid]->instAddr(),
                    fromIEW->squashedSeqNum[tid]);
            } else {
                DPRINTF(Commit,
                    "[tid:%i]: Squashing due to order violation [sn:%i]\n",
                    tid, fromIEW->squashedSeqNum[tid]);
            }

            DPRINTF(Commit, "[tid:%i]: Redirecting to PC %#x\n",
                    tid,
                    fromIEW->pc[tid].nextInstAddr());

            commitStatus[tid] = ROBSquashing;

            // If we want to include the squashing instruction in the squash,
            // then use one older sequence number.
            InstSeqNum squashed_inst = fromIEW->squashedSeqNum[tid];

            //*****CHANGE START**********
            squashDueToLVP = squashDueToLVP || fromIEW->squashDueToLVP[tid];
            toIEW->commitInfo[tid].squashDueToLVP = fromIEW->squashDueToLVP[tid]; //HERE
            toIEW->commitInfo[tid].currentTraceID = fromIEW->currentTraceID[tid];
            //*****CHANGE END**********

            if (fromIEW->includeSquashInst[tid]) {
                squashed_inst--;
            }

            // All younger instructions will be squashed. Set the sequence
            // number as the youngest instruction in the ROB.
            youngestSeqNum[tid] = squashed_inst;

            rob->squash(squashed_inst, tid, fromIEW->squashDueToLVP[tid]);
            changedROBNumEntries[tid] = true;

            toIEW->commitInfo[tid].doneSeqNum = squashed_inst;

            toIEW->commitInfo[tid].squash = true;

            // Send back the rob squashing signal so other stages know that
            // the ROB is in the process of squashing.
            toIEW->commitInfo[tid].robSquashing = true;

            toIEW->commitInfo[tid].mispredictInst =
                fromIEW->mispredictInst[tid];
            toIEW->commitInfo[tid].branchTaken =
                fromIEW->branchTaken[tid];
            toIEW->commitInfo[tid].squashInst =
                                    rob->findInst(tid, squashed_inst);
            if (toIEW->commitInfo[tid].mispredictInst) {
                if (toIEW->commitInfo[tid].mispredictInst->isUncondCtrl()) {
                     toIEW->commitInfo[tid].branchTaken = true;
                }
                ++branchMispredicts;
            }

            toIEW->commitInfo[tid].pc = fromIEW->pc[tid];
            toIEW->commitInfo[tid].oldpc = fromIEW->oldpc[tid];
        }

        if (commitStatus[tid] == ROBSquashing) {
            num_squashing_threads++;
        }
    }


    // If commit is currently squashing, then it will have activity for the
    // next cycle. Set its next status as active.
    if (num_squashing_threads) {
        _nextStatus = Active;
    }
    

    if (num_squashing_threads != numThreads) {
        // If we're not currently squashing, then get instructions.
        getInsts();

        // Try to commit any instructions.
        commitInsts();
    }

    //Check for any activity
    threads = activeThreads->begin();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (changedROBNumEntries[tid]) {
            toIEW->commitInfo[tid].usedROB = true;
            toIEW->commitInfo[tid].freeROBEntries = rob->numFreeEntries(tid);

            wroteToTimeBuffer = true;
            changedROBNumEntries[tid] = false;
            if (rob->isEmpty(tid))
                checkEmptyROB[tid] = true;
        }

        // ROB is only considered "empty" for previous stages if: a)
        // ROB is empty, b) there are no outstanding stores, c) IEW
        // stage has received any information regarding stores that
        // committed.
        // c) is checked by making sure to not consider the ROB empty
        // on the same cycle as when stores have been committed.
        // @todo: Make this handle multi-cycle communication between
        // commit and IEW.
        if (checkEmptyROB[tid] && rob->isEmpty(tid) &&
            !iewStage->hasStoresToWB(tid) && !committedStores[tid]) {
            checkEmptyROB[tid] = false;
            toIEW->commitInfo[tid].usedROB = true;
            toIEW->commitInfo[tid].emptyROB = true;
            toIEW->commitInfo[tid].freeROBEntries = rob->numFreeEntries(tid);
            wroteToTimeBuffer = true;
        }

    }
}

template <class Impl>
void
DefaultCommit<Impl>::commitInsts()
{
    ////////////////////////////////////
    // Handle commit
    // Note that commit will be handled prior to putting new
    // instructions in the ROB so that the ROB only tries to commit
    // instructions it has in this current cycle, and not instructions
    // it is writing in during this cycle.  Can't commit and squash
    // things at the same time...
    ////////////////////////////////////
    
    DPRINTF(Commit, "Trying to commit instructions in the ROB.\n");

    unsigned num_committed = 0;

    DynInstPtr head_inst;

    // Commit as many instructions as possible until the commit bandwidth
    // limit is reached, or it becomes impossible to commit any more.
    while (num_committed < commitWidth) {
        // Check for any interrupt that we've already squashed for
        // and start processing it.
        if (interrupt != NoFault)
            handleInterrupt();

        ThreadID commit_thread = getCommittingThread();

        if (commit_thread == -1 || !rob->isHeadReady(commit_thread))
            break;

        head_inst = rob->readHeadInst(commit_thread);

        ThreadID tid = head_inst->threadNumber;

        assert(tid == commit_thread);

        DPRINTF(Commit, "Trying to commit head instruction, [sn:%i] [tid:%i]\n",
                head_inst->seqNum, tid);

        // If the head instruction is squashed, it is ready to retire
        // (be removed from the ROB) at any time.
        if (head_inst->isSquashed()) {

            DPRINTF(Commit, "Retiring squashed instruction from "
                    "ROB.\n");

            rob->retireHead(commit_thread);

            ++commitSquashedInsts; //HERE   

            if (head_inst->isStreamedFromSpeculativeCache()){
                if (head_inst->lvMispred){//squashDueToLVP){
                    commitSquashedInstFromSpecLVP ++;
                } else {
                    commitSquashedInstsFromSpecNotLVP ++;
                }
            } else if (head_inst->isStreamedFromUOpCache()){
                if (head_inst->lvMispred){//squashDueToLVP){
                    commitSquashedInstFromUopLVP ++;
                } else {
                    commitSquashedInstFromUopNotLVP ++;
                }
            } else {
                if (head_inst->lvMispred){//squashDueToLVP){
                    commitSquashedInstFromICacheLVP ++;
                } else {
                    commitSquashedInstFromICacheNotLVP ++;
                } 
            }
            //6
            //isStreamedFromUOpCache
            //            toIEW->commitInfo[tid].squashDueToLVP = fromIEW->squashDueToLVP[tid]; //HERE
            // Notify potential listeners that this instruction is squashed
            ppSquash->notify(head_inst);

            // Record that the number of ROB entries has changed.
            changedROBNumEntries[tid] = true;
        } else {
            pc[tid] = head_inst->pcState();

            // Increment the total number of non-speculative instructions
            // executed.
            // Hack for now: it really shouldn't happen until after the
            // commit is deemed to be successful, but this count is needed
            // for syscalls.
            thread[tid]->funcExeInst++;

            // Try to commit the head instruction.
            bool commit_success = commitHead(head_inst, num_committed);

            if (commit_success) {
                ++num_committed;
                statCommittedInstType[tid][head_inst->opClass()]++;

                panic_if(head_inst->isSquashedAndCommited() && !head_inst->isTracePredictionSource(), "A none prediction source instruction is getting suqashed and commited!\n");
                panic_if(head_inst->isSquashedAndCommited() && !head_inst->isStreamedFromSpeculativeCache(), "A non-speculative instruction is getting suqashed and commited!\n");
                if (head_inst->isStreamedFromSpeculativeCache() && head_inst->isTracePredictionSource() && head_inst->isSquashedAndCommited() )
                {
                    statCommittedInstType[tid][OpClass::SquahedAndCommited]++;

                    if (head_inst->isLoad())
                        statCommittedInstType[tid][OpClass::LoadSquahedAndCommited]++;
                    else
                        statCommittedInstType[tid][OpClass::ArithmeticSquashedAndCommited]++;
                }
                
                // only count insts that are not suqashed and committed due to speculative execution
                if (!head_inst->isSquashedAndCommited())
                {
                    if (head_inst->getName() == "and")
                        statCommittedInstType[tid][OpClass::AND]++;
                    else if (head_inst->getName() == "andi")
                        statCommittedInstType[tid][OpClass::ANDI]++;
                    else if ( head_inst->getName() == "or")
                        statCommittedInstType[tid][OpClass::OR]++;
                    else if ( head_inst->getName() == "ori")
                        statCommittedInstType[tid][OpClass::ORI]++;
                    else if ( head_inst->getName() == "xor")
                        statCommittedInstType[tid][OpClass::XOR]++;
                    else if ( head_inst->getName() == "xori")
                        statCommittedInstType[tid][OpClass::XORI]++;
                    else if ( head_inst->getName() == "sub")
                        statCommittedInstType[tid][OpClass::SUB]++;
                    else if ( head_inst->getName() == "subi")
                        statCommittedInstType[tid][OpClass::SUBI]++;
                    else if ( head_inst->getName() == "add")
                        statCommittedInstType[tid][OpClass::ADD]++;
                    else if ( head_inst->getName() == "addi")
                        statCommittedInstType[tid][OpClass::ADDI]++;
                    else if ( head_inst->getName() == "slli")
                        statCommittedInstType[tid][OpClass::SLLI]++;
                    else if ( head_inst->getName() == "srli")
                        statCommittedInstType[tid][OpClass::SRLI]++;
                    else if ( head_inst->getName() == "lea")
                        statCommittedInstType[tid][OpClass::LEA]++;
                    else if ( head_inst->getName() == "sexti")
                        statCommittedInstType[tid][OpClass::SEXTI]++;
                    else if ( head_inst->getName() == "zexti")
                        statCommittedInstType[tid][OpClass::ZEXTI]++;
                    else if ( head_inst->getName() == "limm")
                        statCommittedInstType[tid][OpClass::LIMM]++;
                    else if ( head_inst->getName() == "rdip")
                        statCommittedInstType[tid][OpClass::RDIP]++;
                }

                ppCommit->notify(head_inst);

                changedROBNumEntries[tid] = true;

                // Set the doneSeqNum to the youngest committed instruction.
                toIEW->commitInfo[tid].doneSeqNum = head_inst->seqNum;

                if (tid == 0) {
                    canHandleInterrupts =  (!head_inst->isDelayedCommit()) &&
                                           ((THE_ISA != ALPHA_ISA) ||
                                             (!(pc[0].instAddr() & 0x3)));
                }

                // at this point store conditionals should either have
                // been completed or predicated false
                assert(!head_inst->isStoreConditional() ||
                       head_inst->isCompleted() ||
                       !head_inst->readPredicate());

                // Updates misc. registers.
                head_inst->updateMiscRegs();

                // Check instruction execution if it successfully commits and
                // is not carrying a fault.
                if (cpu->checker) {
                    cpu->checker->verify(head_inst);
                }

                cpu->traceFunctions(pc[tid].instAddr());

                TheISA::advancePC(pc[tid], head_inst->staticInst);

                // Keep track of the last sequence number commited
                lastCommitedSeqNum[tid] = head_inst->seqNum;

                // If this is an instruction that doesn't play nicely with
                // others squash everything and restart fetch
                if (head_inst->isSquashAfter())
                    squashAfter(tid, head_inst);

                if (drainPending) {
                    if (pc[tid].microPC() == 0 && interrupt == NoFault &&
                        !thread[tid]->trapPending) {
                        // Last architectually committed instruction.
                        // Squash the pipeline, stall fetch, and use
                        // drainImminent to disable interrupts
                        DPRINTF(Drain, "Draining: %i:%s\n", tid, pc[tid]);
                        squashAfter(tid, head_inst);
                        cpu->commitDrained(tid);
                        drainImminent = true;
                    }
                }

                bool onInstBoundary = !head_inst->isMicroop() ||
                                      head_inst->isLastMicroop() ||
                                      !head_inst->isDelayedCommit();

                if (onInstBoundary) {
                    int count = 0;
                    Addr oldpc;
                    // Make sure we're not currently updating state while
                    // handling PC events.
                    assert(!thread[tid]->noSquashFromTC &&
                           !thread[tid]->trapPending);
                    do {
                        oldpc = pc[tid].instAddr();
                        cpu->system->pcEventQueue.service(thread[tid]->getTC());
                        count++;
                    } while (oldpc != pc[tid].instAddr());
                    if (count > 1) {
                        DPRINTF(Commit,
                                "PC skip function event, stopping commit\n");
                        break;
                    }
                }

                // Check if an instruction just enabled interrupts and we've
                // previously had an interrupt pending that was not handled
                // because interrupts were subsequently disabled before the
                // pipeline reached a place to handle the interrupt. In that
                // case squash now to make sure the interrupt is handled.
                //
                // If we don't do this, we might end up in a live lock situation
                if (!interrupt && avoidQuiesceLiveLock &&
                    onInstBoundary && cpu->checkInterrupts(cpu->tcBase(0)))
                    squashAfter(tid, head_inst);
            } else {
                DPRINTF(Commit, "Unable to commit head instruction PC:%s "
                        "[tid:%i] [sn:%i].\n",
                        head_inst->pcState(), tid ,head_inst->seqNum);
                break;
            }
        }
    }

    DPRINTF(CommitRate, "%i\n", num_committed);
    numCommittedDist.sample(num_committed);

    if (num_committed == commitWidth) {
        commitEligibleSamples++;
    }
}

template <class Impl>
bool
DefaultCommit<Impl>::commitHead(DynInstPtr &head_inst, unsigned inst_num)
{
    assert(head_inst);
    assert(head_inst->staticInst);

    ThreadID tid = head_inst->threadNumber;

    // If the instruction is not executed yet, then it will need extra
    // handling.  Signal backwards that it should be executed.
    if (!head_inst->isExecuted()) {
        // Keep this number correct.  We have not yet actually executed
        // and committed this instruction.
        thread[tid]->funcExeInst--;

        // Make sure we are only trying to commit un-executed instructions we
        // think are possible.
        assert(head_inst->isNonSpeculative() || head_inst->isStoreConditional()
               || head_inst->isMemBarrier() || head_inst->isWriteBarrier() ||
               (head_inst->isLoad() && head_inst->strictlyOrdered()));

        DPRINTF(Commit, "Encountered a barrier or non-speculative "
                "instruction [sn:%lli] at the head of the ROB, PC %s.\n",
                head_inst->seqNum, head_inst->pcState());

        if (inst_num > 0 || iewStage->hasStoresToWB(tid)) {
            DPRINTF(Commit, "Waiting for all stores to writeback.\n");
            return false;
        }

        toIEW->commitInfo[tid].nonSpecSeqNum = head_inst->seqNum;

        // Change the instruction so it won't try to commit again until
        // it is executed.
        head_inst->clearCanCommit();

        if (head_inst->isLoad() && head_inst->strictlyOrdered()) {
            DPRINTF(Commit, "[sn:%lli]: Strictly ordered load, PC %s.\n",
                    head_inst->seqNum, head_inst->pcState());
            toIEW->commitInfo[tid].strictlyOrdered = true;
            toIEW->commitInfo[tid].strictlyOrderedLoad = head_inst;
        } else {
            ++commitNonSpecStalls;
        }

        return false;
    }


    if (head_inst->isThreadSync()) {
        // Not handled for now.
        panic("Thread sync instructions are not handled yet.\n");
    }

    // Check if the instruction caused a fault.  If so, trap.
    Fault inst_fault = head_inst->getFault();

    // Stores mark themselves as completed.
    if (!head_inst->isStore() && inst_fault == NoFault) {
        head_inst->setCompleted();
    }

    if (inst_fault != NoFault) {
        DPRINTF(Commit, "Inst [sn:%lli] PC %s has a fault\n",
                head_inst->seqNum, head_inst->pcState());

        if (iewStage->hasStoresToWB(tid) || inst_num > 0) {
            DPRINTF(Commit, "Stores outstanding, fault must wait.\n");
            return false;
        }

        head_inst->setCompleted();

        // If instruction has faulted, let the checker execute it and
        // check if it sees the same fault and control flow.
        if (cpu->checker) {
            // Need to check the instruction before its fault is processed
            cpu->checker->verify(head_inst);
        }

        assert(!thread[tid]->noSquashFromTC);

        // Mark that we're in state update mode so that the trap's
        // execution doesn't generate extra squashes.
        thread[tid]->noSquashFromTC = true;

        // Execute the trap.  Although it's slightly unrealistic in
        // terms of timing (as it doesn't wait for the full timing of
        // the trap event to complete before updating state), it's
        // needed to update the state as soon as possible.  This
        // prevents external agents from changing any specific state
        // that the trap need.
        cpu->trap(inst_fault, tid,
                  head_inst->notAnInst() ?
                      StaticInst::nullStaticInstPtr :
                      head_inst->staticInst);

        // Exit state update mode to avoid accidental updating.
        thread[tid]->noSquashFromTC = false;

        commitStatus[tid] = TrapPending;

        DPRINTF(Commit, "Committing instruction with fault [sn:%lli]\n",
            head_inst->seqNum);
        if (head_inst->traceData) {
            if (DTRACE(ExecFaulting)) {
                head_inst->traceData->setFetchSeq(head_inst->seqNum);
                head_inst->traceData->setCPSeq(thread[tid]->numOp);
                head_inst->traceData->dump();
            }
            delete head_inst->traceData;
            head_inst->traceData = NULL;
        }

        // Generate trap squash event.
        generateTrapEvent(tid, inst_fault);
        return false;
    }

    updateComInstStats(head_inst);

    if (FullSystem) {
        if (thread[tid]->profile) {
            thread[tid]->profilePC = head_inst->instAddr();
            ProfileNode *node = thread[tid]->profile->consume(
                    thread[tid]->getTC(), head_inst->staticInst);

            if (node)
                thread[tid]->profileNode = node;
        }
        if (CPA::available()) {
            if (head_inst->isControl()) {
                ThreadContext *tc = thread[tid]->getTC();
                CPA::cpa()->swAutoBegin(tc, head_inst->nextInstAddr());
            }
        }
    }


    // logic to end the simulation
    if (cpu->fetch.decoder[tid]->isSuperOptimizationPresent)
    {
        if (head_inst->isStreamedFromSpeculativeCache()) { 

            numMicroopsShrunken += head_inst->staticInst->shrunkenLength;

            DPRINTF(Commit, "Number of shrunken microops after commiting this instruction %d. Number of total commited microops: %d\n", numMicroopsShrunken, ((uint64_t)cpu->committedOps[tid].value() + numMicroopsShrunken));
            if (head_inst->isSquashedAndCommited()) 
            {
                numMicroopsShrunken--;
                DPRINTF(Commit, "Found a suqashed and commited instruction! numMicroopsShrunken: %d\n", numMicroopsShrunken);
                DPRINTF(Commit, "Number of shrunken microops after commiting this instruction %d. Number of total commited microops: %d\n", numMicroopsShrunken, ((uint64_t)cpu->committedOps[tid].value() + numMicroopsShrunken));
            }

            // gather some stats about this superoptmized instrcution
            assert( head_inst->staticInst->getTraceID()); 
            assert( head_inst->staticInst->getTraceLength());
            numCommittedSuperoptmizedInstDist.sample(head_inst->staticInst->getTraceLength(),1);

           
            assert(cpu->fetch.decoder[tid]->traceConstructor->traceMap.find(head_inst->staticInst->getTraceID()) != cpu->fetch.decoder[tid]->traceConstructor->traceMap.end());
            cpu->fetch.decoder[tid]->traceConstructor->traceMap[head_inst->staticInst->getTraceID()].totalNumOfMicroopsCommitedFromTrace++;
        }

        if ((numMicroopsShrunken + (uint64_t)cpu->committedOps[tid].value()) >= checkpointAtInstr && checkpointAtInstr) {

            DPRINTF(TraceEviction, "---------------Live Traces in Spec Cache at the end of the simulation!------------------\n");
            for (auto &it : cpu->fetch.decoder[tid]->traceConstructor->traceMap)
            {
                if (it.second.state == SpecTrace::Complete)
                {   
                    // Dump some debug analysis information for evicted traces
                    DPRINTF(TraceEviction, "@ %d,%d,%d,%d,%d,%d,%x,%x,%d,%d,%d,%d,%d,%d\n", 
                                    it.second.id, 
                                    it.second.insertion_tick,
                                    it.second.eviction_tick,
                                    it.second.hotness,
                                    it.second.length,
                                    it.second.shrunkLength,
                                    it.second.getTraceHeadAddr().pcAddr,
                                    it.second.end.pcAddr,
                                    it.second.branchesFolded,
                                    it.second.validPredSources,
                                    it.second.totalNumOfTimesControlSourcesAreMisspredicted,
                                    it.second.totalNumOfTimesPredictionSourcesAreMisspredicted,
                                    it.second.totalNumOfMicroopsFetchedFromTrace,
                                    it.second.totalNumOfMicroopsCommitedFromTrace
                            );
                }
            }
            
            exitSimLoop("super-optimization simpoint reached", 0);
        }
    }
    
    //dumping int arch regs for sanity check
    if (false &&( head_inst->isReturn() || head_inst->isCall()))
    {   
        
        stringstream reg_values;
        std::string sym_str;
        Addr sym_addr;
        Addr cur_pc = head_inst->instAddr();

        //reg_values << std::dec << (uint64_t)cpu->committedInsts[tid].value() << ": ";

        if (debugSymbolTable && 
            /* (!FullSystem || !inUserMode(thread)) && */
            debugSymbolTable->findNearestSymbol(cur_pc, sym_str, sym_addr)) 
        {
            if (cur_pc != sym_addr)
                sym_str += csprintf("+%d",cur_pc - sym_addr);
            reg_values << "@" << sym_str << " ";
        } 
        else 
        {
            reg_values << "0x" << hex << cur_pc << " ";
        }

        reg_values << head_inst->staticInst->disassemble(cur_pc, debugSymbolTable) << "\t";


        std::string arch_regs_name[16] = {"RAX", "RCX", "RDX", "RBX", "RSP", "RBP", "RSI", "RDI", "R8", "R9","R10","R11","R12","R13","R14","R15"}; 
         
        for (int i = 0; i < 16; i++)
        {
            if (i < 16)
                reg_values <<  arch_regs_name[i] << "=" << std::hex <<  cpu->readArchIntReg(i,tid) << " ";
            // else 
            //     reg_values << "t" << i << "=" << std::hex <<  cpu->readArchIntReg(i,tid) << " ";

        }
  
        DPRINTF(SuperOpSanityCheck, "%s\n", reg_values.str());
    }
    else if (head_inst->isStore())
    {
        DPRINTF(SuperOpSanityCheck, "%#x %d\n", head_inst->instAddr(), numMicroopsShrunken + (uint64_t)cpu->committedOps[tid].value());
        //DPRINTF(SuperOpSanityCheck, "%#x\n", head_inst->instAddr());
    }

    if (false &&
        (afterExecCnt == ((uint64_t)cpu->committedOps[tid].value() + numMicroopsShrunken)/100000) &&
        ((uint64_t)cpu->committedOps[tid].value() + numMicroopsShrunken) % 100000 >= 0 && 
        ((uint64_t)cpu->committedOps[tid].value() + numMicroopsShrunken) % 100000 <= 20)
    {
        afterExecCnt++;
        std::cout <<
        std::dec  << ((uint64_t)cpu->committedOps[tid].value() + numMicroopsShrunken) << 
        " " << std::dec  << ((double) ((uint64_t)cpu->committedOps[tid].value() + numMicroopsShrunken)) / (double) cpu->numCycles.value()  << 
        std::endl;

    }
    
    uint64_t uops_plus_shrunken = (uint64_t)cpu->committedOps[tid].value();
    if (cpu->fetch.decoder[tid]->isSuperOptimizationPresent) {
        uops_plus_shrunken += numMicroopsShrunken;
    }

    if (uops_plus_shrunken % 100000 == 0) {
        std::cout << "NumOfUops: " << uops_plus_shrunken << std::endl;
        std::cout << "CPuops: " << std::dec << (((double) cpu->numCycles.value()) / ((double) uops_plus_shrunken)) << std::endl;
    }

    if (true && 
        cpu->fetch.decoder[tid]->isSuperOptimizationPresent && 
        (uint64_t)cpu->committedInsts[tid].value() % 100000 == 0 &&
            !head_inst->isNop() &&
            !head_inst->isInstPrefetch() &&
            head_inst->isLastMicroop()
           )
    {   
            bool pass = true;
            std::map<unsigned int,unsigned int> spec_count;
            for (int idx = 0; idx < cpu->fetch.decoder[tid]->SPEC_CACHE_NUM_SETS; idx++) {
                for (int way = 0; way < cpu->fetch.decoder[tid]->SPEC_CACHE_NUM_WAYS; way++) {
                    if (cpu->fetch.decoder[tid]->speculativeValidArray[idx][way]) {
                        spec_count[cpu->fetch.decoder[tid]->speculativeTraceIDArray[idx][way]] = 0;
                    }
                }
            }
            uint64_t uops_plus_shrunken = ((uint64_t)cpu->committedOps[tid].value()) + numMicroopsShrunken;
            std::cout <<
            "--------------------START OF EPOCH----------------------------" <<
           // std::endl << std::dec << "Ticks: " << (uint64_t)head_inst->traceData->getWhen() <<
            std::endl << std::dec << "NumOfInsts: " << (uint64_t)cpu->committedInsts[tid].value() <<
            std::endl << std::dec << "CPUop: " << (((double) cpu->numCycles.value()) / ((double) uops_plus_shrunken)) <<
            std::endl << std::dec << "traceMapSize: " << cpu->fetch.decoder[tid]->traceConstructor->traceMap.size() <<   
		    std::endl << std::dec << "spec_count Size: " << spec_count.size() << 
            std::endl << std::dec << "Shrinkage Ratio: " << ((double)numMicroopsShrunken / (double)(numMicroopsShrunken + (uint64_t)cpu->committedOps[tid].value())) * 100 << std::endl;       
            for (int idx = 0; idx < cpu->fetch.decoder[tid]->SPEC_CACHE_NUM_SETS; idx++){
                std::cout << "Idx " << idx  << " : " ;
                for (int way = 0; way < cpu->fetch.decoder[tid]->SPEC_CACHE_NUM_WAYS; way++) {
                    
                    if (cpu->fetch.decoder[tid]->speculativeValidArray[idx][way]) {
                        pass &= true; 
                        std::cout << cpu->fetch.decoder[tid]->speculativeTraceIDArray[idx][way] << " (" << cpu->fetch.decoder[tid]->speculativeEvictionStat[idx][way] << ") ";
                   }
                    else if (cpu->fetch.decoder[tid]->speculativeTraceIDArray[idx][way] != 0)
                    {
                        std::cout << "{" << cpu->fetch.decoder[tid]->speculativeTraceIDArray[idx][way] << "} (" << " (" << cpu->fetch.decoder[tid]->speculativeEvictionStat[idx][way] << ") ";
                        pass &= false; 
                    }
                    else 
                    {
                        std::cout  << cpu->fetch.decoder[tid]->speculativeTraceIDArray[idx][way] << " (" << cpu->fetch.decoder[tid]->speculativeEvictionStat[idx][way] << ") ";
                        pass &= true; 
                    }
                }
                std::cout << std::endl;
            }
            std::cout << std::endl;  
            assert(pass);
            
            pass = true;
            for (auto const &it : cpu->fetch.decoder[tid]->traceConstructor->traceMap)
            {
                if (spec_count.find(it.first) == spec_count.end() && it.second.state == SpecTrace::Complete)
                {   
                    std::cout << "Can't find trace " << it.first << " in spec$ but it is present in trace map and its state is complete!\n";
                    pass &= false;   
                }
                else if (spec_count.find(it.first) == spec_count.end() && it.second.state != SpecTrace::Complete)
                {
                    panic_if(it.second.state == SpecTrace::Evicted, "Found an evicted trace in traceMap!\n");
                }
            }
            assert(pass);


            // Performance analysis 
            DPRINTF(PerfAnalysis, "%f,%d,%d,%d\n", 
                ((double)numMicroopsShrunken / (double)(numMicroopsShrunken + (uint64_t)cpu->committedOps[tid].value())) * 100, 
                 iewStage->totalNumOfTimesPredictionSourcesOfTracesAreMisspredicted.value(),
                 iewStage->totalNumOfTimesControlSourcesOfTracesAreMisspredicted.value(), 
                 iewStage->branchMispredicts.total());
            

    }
    else if (true && !cpu->fetch.decoder[tid]->isSuperOptimizationPresent && 
        (uint64_t)cpu->committedInsts[tid].value() % 100000 == 0 &&
            !head_inst->isNop() &&
            !head_inst->isInstPrefetch() &&
            head_inst->isLastMicroop()
           )
    {   
        std::cout <<
            "--------------------START OF EPOCH----------------------------" <<
           // std::endl << std::dec << "Ticks: " << (uint64_t)head_inst->traceData->getWhen() <<
            std::endl << std::dec << "NumOfInsts: " << (uint64_t)cpu->committedInsts[tid].value() <<
            std::endl << std::dec << "Total CPI: " << (double) cpu->totalCpi.total() <<
            std::endl;
    }

    DPRINTF(Commit, "Committing instruction with [sn:%lli] PC %s\n",
            head_inst->seqNum, head_inst->pcState());
    if (head_inst->traceData) {
        head_inst->traceData->setFetchSeq(head_inst->seqNum);
        head_inst->traceData->setCPSeq(thread[tid]->numOp);
        head_inst->traceData->dump();
        delete head_inst->traceData;
        head_inst->traceData = NULL;
    }
    if (head_inst->isReturn()) {
        DPRINTF(Commit,"Return Instruction Committed [sn:%lli] PC %s \n",
                        head_inst->seqNum, head_inst->pcState());
    }

    // Update the commit rename map
    for (int i = 0; i < head_inst->numDestRegs(); i++) {
        renameMap[tid]->setEntry(head_inst->flattenedDestRegIdx(i),
                                 head_inst->renamedDestRegIdx(i));
    }

    // Finally clear the head ROB entry.
    rob->retireHead(tid);

#if TRACING_ON
    if (DTRACE(O3PipeView)) {
        head_inst->commitTick = curTick() - head_inst->fetchTick;
    }
#endif

    // If this was a store, record it for this cycle.
    if (head_inst->isStore())
        committedStores[tid] = true;

    // Return true to indicate that we have committed an instruction.
    return true;
}

template <class Impl>
void
DefaultCommit<Impl>::getInsts()
{
    DPRINTF(Commit, "Getting instructions from Rename stage.\n");

    // Read any renamed instructions and place them into the ROB.
    int insts_to_process = std::min((int)renameWidth, fromRename->size);

    for (int inst_num = 0; inst_num < insts_to_process; ++inst_num) {
        DynInstPtr inst;

        inst = fromRename->insts[inst_num];
        ThreadID tid = inst->threadNumber;

        if (!inst->isSquashed() &&
            commitStatus[tid] != ROBSquashing &&
            commitStatus[tid] != TrapPending) {
            changedROBNumEntries[tid] = true;

            DPRINTF(Commit, "Inserting PC %s [sn:%i] [tid:%i] into ROB.\n",
                    inst->pcState(), inst->seqNum, tid);

            rob->insertInst(inst);

            assert(rob->getThreadEntries(tid) <= rob->getMaxEntries(tid));

            youngestSeqNum[tid] = inst->seqNum;
        } else {
            DPRINTF(Commit, "Instruction PC %s [sn:%i] [tid:%i] was "
                    "squashed, skipping.\n",
                    inst->pcState(), inst->seqNum, tid);
        }
    }
}

template <class Impl>
void
DefaultCommit<Impl>::markCompletedInsts()
{
    // Grab completed insts out of the IEW instruction queue, and mark
    // instructions completed within the ROB.
    for (int inst_num = 0; inst_num < fromIEW->size; ++inst_num) {
        assert(fromIEW->insts[inst_num]);
        if (!fromIEW->insts[inst_num]->isSquashed()) {
            DPRINTF(Commit, "[tid:%i]: Marking PC %s, [sn:%lli] ready "
                    "within ROB.\n",
                    fromIEW->insts[inst_num]->threadNumber,
                    fromIEW->insts[inst_num]->pcState(),
                    fromIEW->insts[inst_num]->seqNum);

            // Mark the instruction as ready to commit.
            fromIEW->insts[inst_num]->setCanCommit();
        }
    }
}

template <class Impl>
void
DefaultCommit<Impl>::updateComInstStats(DynInstPtr &inst)
{
    ThreadID tid = inst->threadNumber;

    if (!inst->isMicroop() || inst->isLastMicroop())
        instsCommitted[tid]++;
    opsCommitted[tid]++;

    if (inst->reducableAtFetch) { reducableCommitted++; }

    // To match the old model, don't count nops and instruction
    // prefetches towards the total commit count.
    if (!(inst->getName() == "NOP" || inst->getName() == "fault") && 
        !inst->isNop() && !inst->isInstPrefetch()) 
    {
        cpu->instDone(tid, inst);
    }

    //
    //  Control Instructions
    //
    if (inst->isControl())
        statComBranches[tid]++;

    //
    //  Memory references
    //
    if (inst->isMemRef()) {
        statComRefs[tid]++;

        if (inst->isLoad()) {
            statComLoads[tid]++;
            if (inst->staticInst->confidence >= 0) { ++lvpPredsCommitted; }
        }
    }

    if (inst->isMemBarrier()) {
        statComMembars[tid]++;
    }

    // Integer Instruction
    if (inst->isInteger())
        statComInteger[tid]++;

    // Floating Point Instruction
    if (inst->isFloating())
        statComFloating[tid]++;
    // Vector Instruction
    if (inst->isVector())
        statComVector[tid]++;

    // Function Calls
    if (inst->isCall())
        statComFunctionCalls[tid]++;

}

////////////////////////////////////////
//                                    //
//  SMT COMMIT POLICY MAINTAINED HERE //
//                                    //
////////////////////////////////////////
template <class Impl>
ThreadID
DefaultCommit<Impl>::getCommittingThread()
{
    if (numThreads > 1) {
        switch (commitPolicy) {

          case Aggressive:
            //If Policy is Aggressive, commit will call
            //this function multiple times per
            //cycle
            return oldestReady();

          case RoundRobin:
            return roundRobin();

          case OldestReady:
            return oldestReady();

          default:
            return InvalidThreadID;
        }
    } else {
        assert(!activeThreads->empty());
        ThreadID tid = activeThreads->front();

        if (commitStatus[tid] == Running ||
            commitStatus[tid] == Idle ||
            commitStatus[tid] == FetchTrapPending) {
            return tid;
        } else {
            return InvalidThreadID;
        }
    }
}

template<class Impl>
ThreadID
DefaultCommit<Impl>::roundRobin()
{
    list<ThreadID>::iterator pri_iter = priority_list.begin();
    list<ThreadID>::iterator end      = priority_list.end();

    while (pri_iter != end) {
        ThreadID tid = *pri_iter;

        if (commitStatus[tid] == Running ||
            commitStatus[tid] == Idle ||
            commitStatus[tid] == FetchTrapPending) {

            if (rob->isHeadReady(tid)) {
                priority_list.erase(pri_iter);
                priority_list.push_back(tid);

                return tid;
            }
        }

        pri_iter++;
    }

    return InvalidThreadID;
}

template<class Impl>
ThreadID
DefaultCommit<Impl>::oldestReady()
{
    unsigned oldest = 0;
    bool first = true;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (!rob->isEmpty(tid) &&
            (commitStatus[tid] == Running ||
             commitStatus[tid] == Idle ||
             commitStatus[tid] == FetchTrapPending)) {

            if (rob->isHeadReady(tid)) {

                DynInstPtr head_inst = rob->readHeadInst(tid);

                if (first) {
                    oldest = tid;
                    first = false;
                } else if (head_inst->seqNum < oldest) {
                    oldest = tid;
                }
            }
        }
    }

    if (!first) {
        return oldest;
    } else {
        return InvalidThreadID;
    }
}
#endif//__CPU_O3_COMMIT_IMPL_HH__
