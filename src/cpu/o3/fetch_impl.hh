/*
 * Copyright (c) 2010-2014 ARM Limited
 * Copyright (c) 2012-2013 AMD
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
 *          Korey Sewell
 */

#ifndef __CPU_O3_FETCH_IMPL_HH__
#define __CPU_O3_FETCH_IMPL_HH__

#include <algorithm>
#include <cstring>
#include <list>
#include <map>
#include <queue>

#include "arch/generic/tlb.hh"
#include "arch/isa_traits.hh"
#include "arch/utility.hh"
#include "arch/vtophys.hh"
#include "base/random.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "cpu/base.hh"

//#include "cpu/checker/cpu.hh"
#include "cpu/exetrace.hh"
#include "cpu/o3/fetch.hh"
#include "cpu/o3/isa_specific.hh"
#include "cpu/pred/ltage.hh"
#include "debug/Activity.hh"
#include "debug/Drain.hh"
#include "debug/Fetch.hh"
#include "debug/LVP.hh"
#include "debug/LSD.hh"
#include "debug/O3PipeView.hh"
#include "debug/OptStream.hh"
#include "debug/SuperOp.hh"
#include "debug/VW.hh"
#include "mem/packet.hh"
#include "params/DerivO3CPU.hh"
#include "sim/byteswap.hh"
#include "sim/core.hh"
#include "sim/eventq.hh"
#include "sim/full_system.hh"
#include "sim/system.hh"
#include "debug/TraceQueue.hh"

#include <iostream>
using namespace std;

template<class Impl>
DefaultFetch<Impl>::DefaultFetch(O3CPU *_cpu, DerivO3CPUParams *params)
    : cpu(_cpu),
      usingTrace(params->usingTrace),
      // constantBufferSize(params->constantBufferSize),
      dumpFrequency(params->dumpFrequency),
      decodeToFetchDelay(params->decodeToFetchDelay),
      renameToFetchDelay(params->renameToFetchDelay),
      iewToFetchDelay(params->iewToFetchDelay),
      commitToFetchDelay(params->commitToFetchDelay),
      fetchWidth(params->fetchWidth),
      isLoopStreamDetectionPresent(params->enable_loop_stream_detection),
      isUopCachePresent(params->enable_microop_cache),
      isMicroFusionPresent(params->enable_micro_fusion),
      isSuperOptimizationPresent(params->enable_superoptimization),
      isVectorWideningPresent(params->enable_vector_widening),
      warmupCycles(params->superoptimization_warmup_cycles),
      decodeWidth(params->decodeWidth),
      retryPkt(NULL),
      retryTid(InvalidThreadID),
      cacheBlkSize(cpu->cacheLineSize()),
      fetchBufferSize(params->fetchBufferSize),
      fetchBufferMask(fetchBufferSize - 1),
      fetchQueueSize(params->fetchQueueSize),
      numThreads(params->numThreads),
      numFetchingThreads(params->smtNumFetchingThreads),
      finishTranslationEvent(this)
{
    if (numThreads > Impl::MaxThreads)
        fatal("numThreads (%d) is larger than compiled limit (%d),\n"
              "\tincrease MaxThreads in src/cpu/o3/impl.hh\n",
              numThreads, static_cast<int>(Impl::MaxThreads));
    if (fetchWidth > Impl::MaxWidth)
        fatal("fetchWidth (%d) is larger than compiled limit (%d),\n"
             "\tincrease MaxWidth in src/cpu/o3/impl.hh\n",
             fetchWidth, static_cast<int>(Impl::MaxWidth));
    if (fetchBufferSize > cacheBlkSize)
        fatal("fetch buffer size (%u bytes) is greater than the cache "
              "block size (%u bytes)\n", fetchBufferSize, cacheBlkSize);
    if (cacheBlkSize % fetchBufferSize)
        fatal("cache block (%u bytes) is not a multiple of the "
              "fetch buffer (%u bytes)\n", cacheBlkSize, fetchBufferSize);

    std::string policy = params->smtFetchPolicy;

    // Convert string to lowercase
    std::transform(policy.begin(), policy.end(), policy.begin(),
                   (int(*)(int)) tolower);

    // Figure out fetch policy
    if (policy == "singlethread") {
        fetchPolicy = SingleThread;
        if (numThreads > 1)
            panic("Invalid Fetch Policy for a SMT workload.");
    } else if (policy == "roundrobin") {
        fetchPolicy = RoundRobin;
        DPRINTF(Fetch, "Fetch policy set to Round Robin\n");
    } else if (policy == "branch") {
        fetchPolicy = Branch;
        DPRINTF(Fetch, "Fetch policy set to Branch Count\n");
    } else if (policy == "iqcount") {
        fetchPolicy = IQ;
        DPRINTF(Fetch, "Fetch policy set to IQ count\n");
    } else if (policy == "lsqcount") {
        fetchPolicy = LSQ;
        DPRINTF(Fetch, "Fetch policy set to LSQ count\n");
    } else {
        fatal("Invalid Fetch Policy. Options Are: {SingleThread,"
              " RoundRobin,LSQcount,IQcount}\n");
    }

    // Get the size of an instruction.
    instSize = sizeof(TheISA::MachInst);

    for (int i = 0; i < Impl::MaxThreads; i++) {
        decoder[i] = NULL;
        fetchBuffer[i] = NULL;
        fetchBufferPC[i] = 0;
        fetchBufferValid[i] = false;
    }

    branchPred = params->branchPred;
    loadPred = params->loadPred;
    lsd = params->lsd;
    vw = params->vw;
    //loadPred->cpuInsts = &(cpu->instList);

    // constantLoads = vector<Addr>(constantBufferSize, 0);
    // constantLoadAddrs = {0};
    // constantLoadValidBits = {0};

    isSuperOptimizationActivated = false;
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        decoder[tid] = new TheISA::Decoder(params->isa[tid], params);
        decoder[tid]->setUopCachePresent(isUopCachePresent);
        decoder[tid]->setSpeculativeCachePresent(isSuperOptimizationActivated);
        decoder[tid]->setSuperOptimizationPresent(isSuperOptimizationActivated);
        decoder[tid]->setCPU(cpu, tid);
        // Create space to buffer the cache line data,
        // which may not hold the entire cache line.
        fetchBuffer[tid] = new uint8_t[fetchBufferSize];
    }
}

template <class Impl>
std::string
DefaultFetch<Impl>::name() const
{
    return cpu->name() + ".fetch";
}

template <class Impl>
void
DefaultFetch<Impl>::regProbePoints()
{
    ppFetch = new ProbePointArg<DynInstPtr>(cpu->getProbeManager(), "Fetch");
    ppFetchRequestSent = new ProbePointArg<RequestPtr>(cpu->getProbeManager(),
                                                       "FetchRequest");

}

template <class Impl>
void
DefaultFetch<Impl>::regStats()
{
    icacheStallCycles
        .name(name() + ".icacheStallCycles")
        .desc("Number of cycles fetch is stalled on an Icache miss")
        .prereq(icacheStallCycles);

    uopCacheHitInsts
        .name(name() + ".uopCacheHitInsts")
        .desc("Number of hits (instructions) in the micro-op cache")
        .prereq(uopCacheHitInsts);

    uopCacheHitOps
        .name(name() + ".uopCacheHitOps")
        .desc("Number of hits (ops) in the micro-op cache")
        .prereq(uopCacheHitOps);

    uopCacheMissOps
        .name(name() + ".uopCacheMissOps")
        .desc("Number of misses (ops) in the micro-op cache")
        .prereq(uopCacheMissOps);

    fetchedInsts
        .name(name() + ".Insts")
        .desc("Number of instructions fetch has processed")
        .prereq(fetchedInsts);

    fetchedOps
        .name(name() + ".Ops")
        .desc("Number of uops fetch has processed")
        ;

    fetchedReducable
        .name(name() + ".fetchedReducable")
        .desc("Number of fetched instructions that are reducable")
        ;

    fetchedReducablePercent1
        .name(name() + ".fetchedReducablePercent1")
        .desc("Percent of fetched instructions that are reducable (Insts)")
        ;
    fetchedReducablePercent1 = (fetchedReducable / fetchedInsts) * 100;

    fetchedReducablePercent2
        .name(name() + ".fetchedReducablePercent2")
        .desc("Percent of fetched instructions that are reducable (Ops)")
        ;
    fetchedReducablePercent2 = (fetchedReducable / fetchedOps) * 100;

    fetchedBranches
        .name(name() + ".Branches")
        .desc("Number of branches that fetch encountered")
        .prereq(fetchedBranches);

    predictedBranches
        .name(name() + ".predictedBranches")
        .desc("Number of branches that fetch has predicted taken")
        .prereq(predictedBranches);

    fetchCycles
        .name(name() + ".Cycles")
        .desc("Number of cycles fetch has run and was not squashing or"
              " blocked")
        .prereq(fetchCycles);

    fetchSquashCycles
        .name(name() + ".SquashCycles")
        .desc("Number of cycles fetch has spent squashing")
        .prereq(fetchSquashCycles);

    fetchTlbCycles
        .name(name() + ".TlbCycles")
        .desc("Number of cycles fetch has spent waiting for tlb")
        .prereq(fetchTlbCycles);

    fetchIdleCycles
        .name(name() + ".IdleCycles")
        .desc("Number of cycles fetch was idle")
        .prereq(fetchIdleCycles);

    fetchBlockedCycles
        .name(name() + ".BlockedCycles")
        .desc("Number of cycles fetch has spent blocked")
        .prereq(fetchBlockedCycles);

    fetchedCacheLines
        .name(name() + ".CacheLines")
        .desc("Number of cache lines fetched")
        .prereq(fetchedCacheLines);

    fetchMiscStallCycles
        .name(name() + ".MiscStallCycles")
        .desc("Number of cycles fetch has spent waiting on interrupts, or "
              "bad addresses, or out of MSHRs")
        .prereq(fetchMiscStallCycles);

    fetchPendingDrainCycles
        .name(name() + ".PendingDrainCycles")
        .desc("Number of cycles fetch has spent waiting on pipes to drain")
        .prereq(fetchPendingDrainCycles);

    fetchNoActiveThreadStallCycles
        .name(name() + ".NoActiveThreadStallCycles")
        .desc("Number of stall cycles due to no active thread to fetch from")
        .prereq(fetchNoActiveThreadStallCycles);

    fetchPendingTrapStallCycles
        .name(name() + ".PendingTrapStallCycles")
        .desc("Number of stall cycles due to pending traps")
        .prereq(fetchPendingTrapStallCycles);

    fetchPendingQuiesceStallCycles
        .name(name() + ".PendingQuiesceStallCycles")
        .desc("Number of stall cycles due to pending quiesce instructions")
        .prereq(fetchPendingQuiesceStallCycles);

    fetchIcacheWaitRetryStallCycles
        .name(name() + ".IcacheWaitRetryStallCycles")
        .desc("Number of stall cycles due to full MSHR")
        .prereq(fetchIcacheWaitRetryStallCycles);

    fetchIcacheSquashes
        .name(name() + ".IcacheSquashes")
        .desc("Number of outstanding Icache misses that were squashed")
        .prereq(fetchIcacheSquashes);

    fetchTlbSquashes
        .name(name() + ".ItlbSquashes")
        .desc("Number of outstanding ITLB misses that were squashed")
        .prereq(fetchTlbSquashes);

    fetchNisnDist
        .init(/* base value */ 0,
              /* last value */ fetchWidth,
              /* bucket size */ 1)
        .name(name() + ".rateDist")
        .desc("Number of instructions fetched each cycle (Total)")
        .flags(Stats::pdf);

    idleRate
        .name(name() + ".idleRate")
        .desc("Percent of cycles fetch was idle")
        .prereq(idleRate);
    idleRate = fetchIdleCycles * 100 / cpu->numCycles;

    branchRate
        .name(name() + ".branchRate")
        .desc("Number of branch fetches per cycle")
        .flags(Stats::total);
    branchRate = fetchedBranches / cpu->numCycles;

    fetchRate
        .name(name() + ".rate")
        .desc("Number of inst fetches per cycle")
        .flags(Stats::total);
    fetchRate = fetchedInsts / cpu->numCycles;

    uopCacheHitRate
        .name(name() + ".uopCacheHitrate")
        .desc("Uop Cache Hit Rate")
        .precision(6);
    uopCacheHitRate = uopCacheHitOps/(fetchedOps);

    uopCacheInstHitRate
        .name(name() + ".uopCacheInstHitrate")
        .desc("Uop Cache Instruction Hit Rate")
        .precision(6);
    uopCacheInstHitRate = uopCacheHitInsts/(fetchedInsts);

    statFetchMicro
        .init(cpu->numThreads)
        .name(name() + ".micro")
        .desc("Number of instructions with 1:1 macro-micro mapping.")
        .flags(Stats::total)
        ;

    decoder[0]->regStats();
        directCtrlBranchesFetched
                .name(name() + ".directCtrlBranchesFetched")
                .desc("Branches fetched where isDirectCtrl() is true")
                ;
        indirectCtrlBranchesFetched
                .name(name() + ".indirectCtrlBranchesFetched")
                .desc("Branches fetched where isIndirectCtrl() is true")
                ;
        otherBranchesFetched
                .name(name() + ".otherBranchesFetched")
                .desc("Branches where neither isDirectCtrl() nor isIndirectCtrl() is true")
                ;
	deadCodeInsts
		.name(name() + ".deadCodeInsts")
		.desc("Number of fetched insts which were marked as dead code")
		;
	instsPartOfOptimizedTrace
		.name(name() + ".instsPartOfOptimizedTrace")
		.desc("Number of fetched insts which had speculative cache translations")
		;
	instsNotPartOfOptimizedTrace
		.name(name() + ".instsNotPartOfOptimizedTrace")
		.desc("Number of fetched insts which did not have speculative cache translations")
		;

	maxHotness
		.name(name() + ".maxHotness")
		.desc("Top end of range of hotness values");
	maxLength
		.name(name() + ".maxLength")
		.desc("Top end of range of trace lengths");
	maxConfidence
		.name(name() + ".maxConfidence")
		.desc("Top end of range of confidences");
	maxDelay
		.name(name() + ".maxDelay")
		.desc("Top end of range of load resolution delay");

    numFetchBWCyclesWastedDuringSwitch
	    .name(name() + ".numFetchBWCyclesWastedDuringSwitch")
	    .desc("Number of fetch bandwidth cycles wasted due to switch from Spec$ to Uop$/Decoder and vice versa");


    specCacheHitOps
        .name(name() + ".specCacheHitOps")
        .desc("Number of hits (ops) in the spec cache")
        .prereq(specCacheHitOps);

    specCacheMissOps
        .name(name() + ".specCacheMissOps")
        .desc("Number of misses (ops) in the spec cache")
        .prereq(specCacheMissOps);

    specCacheHitRate
        .name(name() + ".specCacheHitRate")
        .desc("Spec Cache Hit Rate")
        .precision(6);
    specCacheHitRate = specCacheHitOps/(fetchedOps);
}

template<class Impl>
void
DefaultFetch<Impl>::setTimeBuffer(TimeBuffer<TimeStruct> *time_buffer)
{
    timeBuffer = time_buffer;

    // Create wires to get information from proper places in time buffer.
    fromDecode = timeBuffer->getWire(-decodeToFetchDelay);
    fromRename = timeBuffer->getWire(-renameToFetchDelay);
    fromIEW = timeBuffer->getWire(-iewToFetchDelay);
    fromCommit = timeBuffer->getWire(-commitToFetchDelay);
}

template<class Impl>
void
DefaultFetch<Impl>::setActiveThreads(std::list<ThreadID> *at_ptr)
{
    activeThreads = at_ptr;
}

template<class Impl>
void
DefaultFetch<Impl>::setFetchQueue(TimeBuffer<FetchStruct> *ftb_ptr)
{
    // Create wire to write information to proper place in fetch time buf.
    toDecode = ftb_ptr->getWire(0);
}

template<class Impl>
void
DefaultFetch<Impl>::startupStage()
{
    assert(priorityList.empty());
    resetStage();

}

template<class Impl>
void
DefaultFetch<Impl>::resetStage()
{
    numInst = 0;
    interruptPending = false;
    cacheBlocked = false;

    priorityList.clear();

    // Setup PC and nextPC with initial state.
    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        fetchStatus[tid] = Running;
        pc[tid] = cpu->pcState(tid);
        fetchOffset[tid] = 0;
        macroop[tid] = NULL;

        delayedCommit[tid] = false;
        memReq[tid] = NULL;

        stalls[tid].decode = false;
        stalls[tid].drain = false;

        fetchBufferPC[tid] = 0;
        fetchBufferValid[tid] = false;

        fetchQueue[tid].clear();

        priorityList.push_back(tid);
    }

    wroteToTimeBuffer = false;
    _status = Inactive;
}

template<class Impl>
void
DefaultFetch<Impl>::processCacheCompletion(PacketPtr pkt)
{
    ThreadID tid = cpu->contextToThread(pkt->req->contextId());

    DPRINTF(Fetch, "[tid:%u] Waking up from cache miss.\n", tid);
    assert(!cpu->switchedOut());

    // Only change the status if it's still waiting on the icache access
    // to return.
    if (fetchStatus[tid] != IcacheWaitResponse ||
        pkt->req != memReq[tid]) {
        ++fetchIcacheSquashes;
        delete pkt;
        return;
    }

    memcpy(fetchBuffer[tid], pkt->getConstPtr<uint8_t>(), fetchBufferSize);
    fetchBufferValid[tid] = true;

    // Wake up the CPU (if it went to sleep and was waiting on
    // this completion event).
    cpu->wakeCPU();

    DPRINTF(Activity, "[tid:%u] Activating fetch due to cache completion\n",
            tid);

    switchToActive();

    // Only switch to IcacheAccessComplete if we're not stalled as well.
    if (checkStall(tid)) {
        fetchStatus[tid] = Blocked;
    } else {
        fetchStatus[tid] = IcacheAccessComplete;
    }

    pkt->req->setAccessLatency();
    cpu->ppInstAccessComplete->notify(pkt);
    // Reset the mem req to NULL.
    delete pkt;
    memReq[tid] = NULL;
}

template <class Impl>
void
DefaultFetch<Impl>::drainResume()
{
    for (ThreadID i = 0; i < numThreads; ++i) {
        stalls[i].decode = false;
        stalls[i].drain = false;
    }
}

template <class Impl>
void
DefaultFetch<Impl>::drainSanityCheck() const
{
    assert(isDrained());
    assert(retryPkt == NULL);
    assert(retryTid == InvalidThreadID);
    assert(!cacheBlocked);
    assert(!interruptPending);

    for (ThreadID i = 0; i < numThreads; ++i) {
        assert(!memReq[i]);
        assert(fetchStatus[i] == Idle || stalls[i].drain);
    }

    branchPred->drainSanityCheck();
}

template <class Impl>
bool
DefaultFetch<Impl>::isDrained() const
{
    /* Make sure that threads are either idle of that the commit stage
     * has signaled that draining has completed by setting the drain
     * stall flag. This effectively forces the pipeline to be disabled
     * until the whole system is drained (simulation may continue to
     * drain other components).
     */
    for (ThreadID i = 0; i < numThreads; ++i) {
        // Verify fetch queues are drained
        if (!fetchQueue[i].empty())
            return false;

        // Return false if not idle or drain stalled
        if (fetchStatus[i] != Idle) {
            if (fetchStatus[i] == Blocked && stalls[i].drain)
                continue;
            else
                return false;
        }
    }

    /* The pipeline might start up again in the middle of the drain
     * cycle if the finish translation event is scheduled, so make
     * sure that's not the case.
     */
    return !finishTranslationEvent.scheduled();
}

template <class Impl>
void
DefaultFetch<Impl>::takeOverFrom()
{
    assert(cpu->getInstPort().isConnected());
    resetStage();

}

template <class Impl>
void
DefaultFetch<Impl>::drainStall(ThreadID tid)
{
    assert(cpu->isDraining());
    assert(!stalls[tid].drain);
    DPRINTF(Drain, "%i: Thread drained.\n", tid);
    stalls[tid].drain = true;
}

template <class Impl>
void
DefaultFetch<Impl>::wakeFromQuiesce()
{
    DPRINTF(Fetch, "Waking up from quiesce\n");
    // Hopefully this is safe
    // @todo: Allow other threads to wake from quiesce.
    fetchStatus[0] = Running;
}

template <class Impl>
inline void
DefaultFetch<Impl>::switchToActive()
{
    if (_status == Inactive) {
        DPRINTF(Activity, "Activating stage.\n");

        cpu->activateStage(O3CPU::FetchIdx);

        _status = Active;
    }
}

template <class Impl>
inline void
DefaultFetch<Impl>::switchToInactive()
{
    if (_status == Active) {
        DPRINTF(Activity, "Deactivating stage.\n");

        cpu->deactivateStage(O3CPU::FetchIdx);

        _status = Inactive;
    }
}

template <class Impl>
void
DefaultFetch<Impl>::deactivateThread(ThreadID tid)
{
    // Update priority list
    auto thread_it = std::find(priorityList.begin(), priorityList.end(), tid);
    if (thread_it != priorityList.end()) {
        priorityList.erase(thread_it);
    }
}




template <class Impl>
bool
DefaultFetch<Impl>::lookupAndUpdateNextPC(
        DynInstPtr &inst, TheISA::PCState &nextPC)
{
    // Do branch prediction check here.
    // A bit of a misnomer...next_PC is actually the current PC until
    // this function updates it.
    bool predict_taken;

    if (!inst->isControl()) {
        TheISA::advancePC(nextPC, inst->staticInst);
        inst->setPredTarg(nextPC);
        inst->setPredTaken(false);
        return false;
    }

    ThreadID tid = inst->threadNumber;

/**
    LTAGE *ltage = dynamic_cast<LTAGE *>(branchPred);
    if (ltage) {
        inst->potentialLoopEnd = ltage->checkLoopEnding(nextPC.instAddr());
        // DPRINTF(LVP, "LTAGE predictor checkLoopEnding returned %d\n", inst->potentialLoopEnd);
    }
**/

    predict_taken = branchPred->predict(inst->staticInst, inst->seqNum,
                                        nextPC, tid);

/**
        if (inst->staticInst->isDirectCtrl()) {
                directCtrlBranchesFetched++;
        } else if (inst->staticInst->isIndirectCtrl()) {
                indirectCtrlBranchesFetched++;
        } else {
                otherBranchesFetched++;
        }
**/

    if (predict_taken) {
        DPRINTF(Fetch, "[tid:%i]: [sn:%i]:  Branch predicted to be taken to %s.\n",
                tid, inst->seqNum, nextPC);
    } else {
        DPRINTF(Fetch, "[tid:%i]: [sn:%i]:Branch predicted to be not taken.\n",
                tid, inst->seqNum);
    }

    DPRINTF(Fetch, "[tid:%i]: [sn:%i] Branch predicted to go to %s.\n",
            tid, inst->seqNum, nextPC);
    inst->setPredTarg(nextPC);
    inst->setPredTaken(predict_taken);

    ++fetchedBranches;

    if (predict_taken) {
        ++predictedBranches;
    }

    return predict_taken;
}

template <class Impl>
bool
DefaultFetch<Impl>::fetchCacheLine(Addr vaddr, ThreadID tid, Addr pc)
{
    Fault fault = NoFault;

    assert(!cpu->switchedOut());

    // @todo: not sure if these should block translation.
    //AlphaDep
    if (cacheBlocked) {
        DPRINTF(Fetch, "[tid:%i] Can't fetch cache line, cache blocked\n",
                tid);
        return false;
    } else if (checkInterrupt(pc) && !delayedCommit[tid]) {
        // Hold off fetch from getting new instructions when:
        // Cache is blocked, or
        // while an interrupt is pending and we're not in PAL mode, or
        // fetch is switched out.
        DPRINTF(Fetch, "[tid:%i] Can't fetch cache line, interrupt pending\n",
                tid);
        return false;
    }

    // Align the fetch address to the start of a fetch buffer segment.
    Addr fetchBufferBlockPC = fetchBufferAlignPC(vaddr);

    DPRINTF(Fetch, "[tid:%i] Fetching cache line %#x for addr %#x\n",
            tid, fetchBufferBlockPC, vaddr);

    // Setup the memReq to do a read of the first instruction's address.
    // Set the appropriate read size and flags as well.
    // Build request here.
    RequestPtr mem_req = std::make_shared<Request>(
        tid, fetchBufferBlockPC, fetchBufferSize,
        Request::INST_FETCH, cpu->instMasterId(), pc,
        cpu->thread[tid]->contextId());

    mem_req->taskId(cpu->taskId());

    memReq[tid] = mem_req;

    // Initiate translation of the icache block
    fetchStatus[tid] = ItlbWait;
    FetchTranslation *trans = new FetchTranslation(this);
    cpu->itb->translateTiming(mem_req, cpu->thread[tid]->getTC(),
                              trans, BaseTLB::Execute);
    return true;
}

template <class Impl>
void
DefaultFetch<Impl>::finishTranslation(const Fault &fault,
                                      const RequestPtr &mem_req)
{
    ThreadID tid = cpu->contextToThread(mem_req->contextId());
    Addr fetchBufferBlockPC = mem_req->getVaddr();

    assert(!cpu->switchedOut());

    // Wake up CPU if it was idle
    cpu->wakeCPU();

    if (fetchStatus[tid] != ItlbWait || mem_req != memReq[tid] ||
        mem_req->getVaddr() != memReq[tid]->getVaddr()) {
        DPRINTF(Fetch, "[tid:%i] Ignoring itlb completed after squash\n",
                tid);
        ++fetchTlbSquashes;
        return;
    }


    // If translation was successful, attempt to read the icache block.
    if (fault == NoFault) {
        // Check that we're not going off into random memory
        // If we have, just wait around for commit to squash something and put
        // us on the right track
        if (!cpu->system->isMemAddr(mem_req->getPaddr())) {
            warn("Address %#x is outside of physical memory, stopping fetch\n",
                    mem_req->getPaddr());
            fetchStatus[tid] = NoGoodAddr;
            memReq[tid] = NULL;
            return;
        }

        // Build packet here.
        PacketPtr data_pkt = new Packet(mem_req, MemCmd::ReadReq);
        data_pkt->dataDynamic(new uint8_t[fetchBufferSize]);

        fetchBufferPC[tid] = fetchBufferBlockPC;
        fetchBufferValid[tid] = false;
        DPRINTF(Fetch, "Fetch: Doing instruction read.\n");

        fetchedCacheLines++;

        // Access the cache.
        if (!cpu->getInstPort().sendTimingReq(data_pkt)) {
            assert(retryPkt == NULL);
            assert(retryTid == InvalidThreadID);
            DPRINTF(Fetch, "[tid:%i] Out of MSHRs!\n", tid);

            fetchStatus[tid] = IcacheWaitRetry;
            retryPkt = data_pkt;
            retryTid = tid;
            cacheBlocked = true;
        } else {
            DPRINTF(Fetch, "[tid:%i]: Doing Icache access.\n", tid);
            DPRINTF(Activity, "[tid:%i]: Activity: Waiting on I-cache "
                    "response.\n", tid);
            lastIcacheStall[tid] = curTick();
            fetchStatus[tid] = IcacheWaitResponse;
            // Notify Fetch Request probe when a packet containing a fetch
            // request is successfully sent
            ppFetchRequestSent->notify(mem_req);
        }
    } else {
        int adjustedFetchWidth = fetchWidth;
        if (!decoder[tid]->isSpeculativeCacheActive() && !decoder[tid]->isUopCacheActive()){
            adjustedFetchWidth = fetchWidth - 1;
        }
        if (!(numInst < adjustedFetchWidth) || !(fetchQueue[tid].size() < fetchQueueSize)) {
            assert(!finishTranslationEvent.scheduled());
            finishTranslationEvent.setFault(fault);
            finishTranslationEvent.setReq(mem_req);
            cpu->schedule(finishTranslationEvent,
                          cpu->clockEdge(Cycles(1)));
            return;
        }
        DPRINTF(Fetch, "[tid:%i] Got back req with addr %#x but expected %#x\n",
                tid, mem_req->getVaddr(), memReq[tid]->getVaddr());
        // Translation faulted, icache request won't be sent.
        memReq[tid] = NULL;

        // Send the fault to commit.  This thread will not do anything
        // until commit handles the fault.  The only other way it can
        // wake up is if a squash comes along and changes the PC.
        TheISA::PCState fetchPC = pc[tid];

        DPRINTF(Fetch, "[tid:%i]: Translation faulted, building noop.\n", tid);
        // We will use a nop in ordier to carry the fault.
        DynInstPtr instruction = buildInst(tid, StaticInst::nopStaticInstPtr,
                                           NULL, fetchPC, fetchPC, false);
        instruction->setNotAnInst();

        instruction->setPredTarg(fetchPC);
        instruction->fault = fault;
        wroteToTimeBuffer = true;

        DPRINTF(Activity, "Activity this cycle.\n");
        cpu->activityThisCycle();

        fetchStatus[tid] = TrapPending;

        DPRINTF(Fetch, "[tid:%i]: Blocked, need to handle the trap.\n", tid);
        DPRINTF(Fetch, "[tid:%i]: fault (%s) detected @ PC %s.\n",
                tid, fault->name(), pc[tid]);
    }
    _status = updateFetchStatus();
}

template <class Impl>
inline void
DefaultFetch<Impl>::doSquash(const TheISA::PCState &newPC,
                             const DynInstPtr squashInst, bool squashDueToLVP, ThreadID tid)
{
    DPRINTF(Fetch, "[tid:%i]: Squashing, setting PC to: %s.\n",
            tid, newPC);


    pc[tid] = newPC;
    fetchOffset[tid] = 0;
    if (squashInst && squashInst->pcState().instAddr() == newPC.instAddr() && !squashInst->isStreamedFromSpeculativeCache()) //&& !decoder[tid]->isSpeculativeCacheActive() && !decoder[tid]->isUopCacheActive())
        macroop[tid] = squashInst->macroop;
    else
        macroop[tid] = NULL;
    decoder[tid]->reset();

    decoder[tid]->doSquash(fromCommit->commitInfo[tid].oldpc.instAddr());

    // Clear the icache miss if it's outstanding.
    if (fetchStatus[tid] == IcacheWaitResponse) {
        DPRINTF(Fetch, "[tid:%i]: Squashing outstanding Icache miss.\n",
                tid);
        memReq[tid] = NULL;
    } else if (fetchStatus[tid] == ItlbWait) {
        DPRINTF(Fetch, "[tid:%i]: Squashing outstanding ITLB miss.\n",
                tid);
        memReq[tid] = NULL;
    }

    // Get rid of the retrying packet if it was from this thread.
    if (retryTid == tid) {
        assert(cacheBlocked);
        if (retryPkt) {
            delete retryPkt;
        }
        retryPkt = NULL;
        retryTid = InvalidThreadID;
    }

    fetchStatus[tid] = Squashing;

    // Empty fetch queue
    fetchQueue[tid].clear();

    // microops are being squashed, it is not known wheather the
    // youngest non-squashed microop was  marked delayed commit
    // or not. Setting the flag to true ensures that the
    // interrupts are not handled when they cannot be, though
    // some opportunities to handle interrupts may be missed.
    delayedCommit[tid] = true;

//    stalls[tid].decode = false;

    ++fetchSquashCycles;
}

template<class Impl>
void
DefaultFetch<Impl>::squashFromDecode(const TheISA::PCState &newPC,
                                     const DynInstPtr squashInst,
                                     const InstSeqNum seq_num, ThreadID tid)
{
    DPRINTF(Fetch, "[tid:%i]: Squashing from decode.\n", tid);

    panic("squashFromDecode is called!");
    doSquash(newPC, squashInst, false, tid);


    // Tell the CPU to remove any instructions that are in flight between
    // fetch and decode.
    cpu->removeInstsUntil(seq_num, tid);
}

template<class Impl>
bool
DefaultFetch<Impl>::checkStall(ThreadID tid) const
{
    bool ret_val = false;

    if (stalls[tid].drain) {
        assert(cpu->isDraining());
        DPRINTF(Fetch,"[tid:%i]: Drain stall detected.\n",tid);
        ret_val = true;
    }

    return ret_val;
}

template<class Impl>
typename DefaultFetch<Impl>::FetchStatus
DefaultFetch<Impl>::updateFetchStatus()
{
    //Check Running
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (fetchStatus[tid] == Running ||
            fetchStatus[tid] == Squashing ||
            fetchStatus[tid] == IcacheAccessComplete) {

            if (_status == Inactive) {
                DPRINTF(Activity, "[tid:%i]: Activating stage.\n",tid);

                if (fetchStatus[tid] == IcacheAccessComplete) {
                    DPRINTF(Activity, "[tid:%i]: Activating fetch due to cache"
                            "completion\n",tid);
                }

                cpu->activateStage(O3CPU::FetchIdx);
            }

            return Active;
        }
    }

    // Stage is switching from active to inactive, notify CPU of it.
    if (_status == Active) {
        DPRINTF(Activity, "Deactivating stage.\n");

        cpu->deactivateStage(O3CPU::FetchIdx);
    }

    return Inactive;
}

template <class Impl>
void
DefaultFetch<Impl>::squash(const TheISA::PCState &newPC,
                           const InstSeqNum seq_num,
                           DynInstPtr squashInst,
                           bool squashDueToLVP,
                            ThreadID tid)

{
    DPRINTF(Fetch, "[tid:%u]: Squash from commit.\n", tid);

    doSquash(newPC, squashInst, squashDueToLVP , tid);


    // Tell the CPU to remove any instructions that are not in the ROB.
    cpu->removeInstsNotInROB(tid, squashDueToLVP);
}

template <class Impl>
void
DefaultFetch<Impl>::tick()
{
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();
    bool status_change = false;

    wroteToTimeBuffer = false;

    for (ThreadID i = 0; i < numThreads; ++i) {
        issuePipelinedIfetch[i] = false;
        if ((warmupCycles == 0) && !isSuperOptimizationActivated && isSuperOptimizationPresent) { 
            std::cout << "Activating superoptimization\n";
            isSuperOptimizationActivated = true;
            decoder[i]->setSpeculativeCachePresent(isSuperOptimizationActivated);
            decoder[i]->setSuperOptimizationPresent(isSuperOptimizationActivated);
        }
    }
    warmupCycles--;

    while (threads != end) {
        ThreadID tid = *threads++;

        decoder[tid]->tickAllHotnessCounters();

        // Check the signals for each thread to determine the proper status
        // for each thread.
        bool updated_status = checkSignalsAndUpdate(tid);
        status_change =  status_change || updated_status;
        // DPRINTF(SuperOp, "Cycle:%i Frequency:%i Cycle mod Freq:%i Value:%i\n", (int) cpu->numCycles.value(), dumpFrequency, (int) cpu->numCycles.value() % dumpFrequency, (int) cpu->numCycles.value() % dumpFrequency == 0);
        if ((int) cpu->numCycles.value() % dumpFrequency == 0) {
            // DPRINTF(SuperOp, "Dump for cycle %i\n", cpu->numCycles.value());
            // decoder[tid]->dumpMicroopCache();
            // DPRINTF(SuperOp, "\n");
            // dumpConstantBuffer();
            // DPRINTF(SuperOp, "\n\n\n");
        }
    }

    DPRINTF(Fetch, "Running stage.\n");

    if (FullSystem) {
        if (fromCommit->commitInfo[0].interruptPending) {
            interruptPending = true;
        }

        if (fromCommit->commitInfo[0].clearInterrupt) {
            interruptPending = false;
        }
    }

    for (threadFetched = 0; threadFetched < numFetchingThreads;
         threadFetched++) {
        // Fetch each of the actively fetching threads.
        fetch(status_change);
    }

    // Record number of instructions fetched this cycle for distribution.
    fetchNisnDist.sample(numInst);

    if (status_change) {
        // Change the fetch stage status if there was a status change.
        _status = updateFetchStatus();
    }

    // Issue the next I-cache request if possible.
    for (ThreadID i = 0; i < numThreads; ++i) {
        if (issuePipelinedIfetch[i]) {
            pipelineIcacheAccesses(i);
        }
    }

    // Send instructions enqueued into the fetch queue to decode.
    // Limit rate by fetchWidth.  Stall if decode is stalled.
    unsigned insts_to_decode = 0;
    unsigned available_insts = 0;

    for (auto tid : *activeThreads) {
        if (!stalls[tid].decode) {
            available_insts += fetchQueue[tid].size();
        }
    }

    // Pick a random thread to start trying to grab instructions from
    auto tid_itr = activeThreads->begin();
    std::advance(tid_itr, random_mt.random<uint8_t>(0, activeThreads->size() - 1));

    unsigned fused_insts_to_decode = insts_to_decode;
    while (available_insts != 0 && fused_insts_to_decode < decodeWidth && insts_to_decode < Impl::MaxWidth) {
        ThreadID tid = *tid_itr;
        if (!stalls[tid].decode && !fetchQueue[tid].empty()) {
            auto inst = fetchQueue[tid].front();
            toDecode->insts[toDecode->size++] = inst;
            DPRINTF(Fetch, "[tid:%i][sn:%i]: Sending instruction to decode from "
                    "fetch queue. Fetch queue size: %i.\n",
                    tid, inst->seqNum, fetchQueue[tid].size());

            wroteToTimeBuffer = true;
            fetchQueue[tid].pop_front();
            if (!inst->fused) fused_insts_to_decode++;
            insts_to_decode++;
            available_insts--;
        }

        tid_itr++;
        // Wrap around if at end of active threads list
        if (tid_itr == activeThreads->end())
            tid_itr = activeThreads->begin();
    }

    // If there was activity this cycle, inform the CPU of it.
    if (wroteToTimeBuffer) {
        DPRINTF(Activity, "Activity this cycle.\n");
        cpu->activityThisCycle();
    }

    // Reset the number of the instruction we've fetched.
    numInst = 0;

    // if (!IsHeapProfilerRunning()) 
    //     HeapProfilerStart("SuperOptimizer");
    if (isSuperOptimizationActivated)
    {
        for (int tid = 0; tid < numThreads; tid++) {
            decoder[tid]->traceConstructor->generateNextTraceInst(); 
        }
    }
    // if (IsHeapProfilerRunning())
    //     HeapProfilerStop();
    
}

template <class Impl>
bool
DefaultFetch<Impl>::checkSignalsAndUpdate(ThreadID tid)
{
    // Update the per thread stall statuses.
    if (fromDecode->decodeBlock[tid]) {
        stalls[tid].decode = true;
    }

    if (fromDecode->decodeUnblock[tid]) {
        assert(stalls[tid].decode);
        assert(!fromDecode->decodeBlock[tid]);
        stalls[tid].decode = false;
    }

    // Check squash signals from commit.
    if (fromCommit->commitInfo[tid].squash) {

        DPRINTF(Fetch, "[tid:%u]: Squashing instructions due to squash "
                "from commit.\n",tid);
        // In any case, squash.
        squash(fromCommit->commitInfo[tid].pc,
               fromCommit->commitInfo[tid].doneSeqNum,
               fromCommit->commitInfo[tid].squashInst, 
               fromCommit->commitInfo[tid].squashDueToLVP,
               tid);

        // reset state of both cache, in the next lines of the code we will decide whether to enable or disable them
        decoder[tid]->setSpeculativeCacheActive(false);
        decoder[tid]->setUopCacheActive(false);

        //*****CHANGE START**********
        
        // LVP missprediction               <== mispredictInst != NULL && squashDueToLVP == true;
        // folded branch missprediction     <== mispredictInst != NULL && squashDueToLVP == false;
        // MemOrder Violation               <== mispredictInst == NULL && squashDueToLVP == true;
        // branch missprediction
        if (fromCommit->commitInfo[tid].mispredictInst)
        {
            // LVP Missprediction
            if (fromCommit->commitInfo[tid].squashDueToLVP)
            {

                // LVP missprediction, therefore, deactivate speculative cache and fetch from uop/decoder
                DPRINTF(Fetch, "LVP misprediction: inst[sn:%i]\n", fromCommit->commitInfo[tid].mispredictInst->seqNum);
                decoder[tid]->setSpeculativeCacheActive(false);
                decoder[tid]->redirectDueToLVPSquashing = true;
            }
            // folded branch missprediction
            else if (fromCommit->commitInfo[tid].mispredictInst->isStreamedFromSpeculativeCache())
            {
                // again deactivate speculative cache
                DPRINTF(Fetch, "folded branch misprediction: inst[sn:%i]\n", fromCommit->commitInfo[tid].mispredictInst->seqNum);
                decoder[tid]->setSpeculativeCacheActive(false);
                decoder[tid]->redirectDueToLVPSquashing = true;
            }
            // not a folded branch or LVP missprediction
            else 
            {
                DPRINTF(Fetch, "branch misprediction: inst[sn:%i]\n", fromCommit->commitInfo[tid].mispredictInst->seqNum);
                decoder[tid]->setSpeculativeCacheActive(false);
                decoder[tid]->redirectDueToLVPSquashing = false;
                currentTraceID = 0;
            }

        }
        else 
        {
            // memorder violation in a speculative trace
            if (fromCommit->commitInfo[tid].squashDueToLVP)
            {
                // activate speculative cache so we can fetch from it again
                DPRINTF(Fetch, "memory order violation at PC %s\n", fromCommit->commitInfo[tid].pc);
                currentTraceID = fromCommit->commitInfo[tid].currentTraceID;
                assert(currentTraceID);
                decoder[tid]->setSpeculativeCacheActive(true, currentTraceID);
                decoder[tid]->traceConstructor->streamTrace.id = currentTraceID;
                decoder[tid]->updateStreamTrace(currentTraceID, fromCommit->commitInfo[tid].pc); 
            }
            else {
                DPRINTF(Fetch, "something else\n");
                // This squash is not due to LVP missprediction, therefore always deactivate the spec$ and the fetch will handle re-activation
                decoder[tid]->setSpeculativeCacheActive(false);
                decoder[tid]->redirectDueToLVPSquashing = false;
                currentTraceID = 0;
            }
        }

        

        //*****CHANGE END**********


        // If it was a branch mispredict on a control instruction, update the
        // branch predictor with that instruction, otherwise just kill the
        // invalid state we generated in after sequence number
        if (fromCommit->commitInfo[tid].mispredictInst &&
            fromCommit->commitInfo[tid].mispredictInst->isControl() && 
            (!fromCommit->commitInfo[tid].mispredictInst->isStreamedFromSpeculativeCache() || 
            (fromCommit->commitInfo[tid].mispredictInst->branchPredFromPredictor && 
            fromCommit->commitInfo[tid].mispredictInst->isLastMicroop()))) {
            branchPred->squash(fromCommit->commitInfo[tid].doneSeqNum,
                              fromCommit->commitInfo[tid].pc,
                              fromCommit->commitInfo[tid].branchTaken,
                              tid);
            DPRINTF(Fetch, "Squashing branch from commit at pc: 0x%x\n", fromCommit->commitInfo[tid].mispredictInst->instAddr());
        } else {
            branchPred->squash(fromCommit->commitInfo[tid].doneSeqNum,
                              tid);
        }

        return true;
    } else if (fromCommit->commitInfo[tid].doneSeqNum) {
        // Update the branch predictor if it wasn't a squashed instruction
        // that was broadcasted.
        if (!fromCommit->commitInfo[tid].mispredictInst || // I dont think this will ever fail TODO
            !fromCommit->commitInfo[tid].mispredictInst->isStreamedFromSpeculativeCache() || 
            fromCommit->commitInfo[tid].mispredictInst->branchPredFromPredictor){
                branchPred->update(fromCommit->commitInfo[tid].doneSeqNum, tid);
        }
    }

    // Check squash signals from decode.
    if (fromDecode->decodeInfo[tid].squash) {
        DPRINTF(Fetch, "[tid:%u]: Squashing instructions due to squash "
                "from decode.\n",tid);

        // Update the branch predictor.
        if (fromDecode->decodeInfo[tid].branchMispredict &&
            fromDecode->decodeInfo[tid].mispredictInst->isControl() && 
            (!fromDecode->decodeInfo[tid].mispredictInst->isStreamedFromSpeculativeCache() || 
            (fromDecode->decodeInfo[tid].mispredictInst->branchPredFromPredictor &&
            fromDecode->decodeInfo[tid].mispredictInst->isLastMicroop()))) {
            branchPred->squash(fromDecode->decodeInfo[tid].doneSeqNum,
                              fromDecode->decodeInfo[tid].nextPC,
                              fromDecode->decodeInfo[tid].branchTaken,
                              tid);
            DPRINTF(Fetch, "Squashing branch from decode at pc: 0x%x\n", fromDecode->decodeInfo[tid].mispredictInst->instAddr());
        } else {
            branchPred->squash(fromDecode->decodeInfo[tid].doneSeqNum,
                              tid);
        }
        


        if (fetchStatus[tid] != Squashing) {

            DPRINTF(Fetch, "Squashing from decode with PC = %s\n",
                fromDecode->decodeInfo[tid].nextPC);
            // Squash unless we're already squashing
            squashFromDecode(fromDecode->decodeInfo[tid].nextPC,
                             fromDecode->decodeInfo[tid].squashInst,
                             fromDecode->decodeInfo[tid].doneSeqNum,
                             tid);

            return true;
        }
    }

    if (checkStall(tid) &&
        fetchStatus[tid] != IcacheWaitResponse &&
        fetchStatus[tid] != IcacheWaitRetry &&
        fetchStatus[tid] != ItlbWait &&
        fetchStatus[tid] != QuiescePending) {
        DPRINTF(Fetch, "[tid:%i]: Setting to blocked\n",tid);

        fetchStatus[tid] = Blocked;

        return true;
    }

    if (fetchStatus[tid] == Blocked ||
        fetchStatus[tid] == Squashing) {
        // Switch status to running if fetch isn't being told to block or
        // squash this cycle.
        DPRINTF(Fetch, "[tid:%i]: Done squashing, switching to running.\n",
                tid);

        fetchStatus[tid] = Running;

        return true;
    }

    // If we've reached this point, we have not gotten any signals that
    // cause fetch to change its status.  Fetch remains the same as before.
    return false;
}

template<class Impl>
typename Impl::DynInstPtr
DefaultFetch<Impl>::buildInst(ThreadID tid, StaticInstPtr staticInst,
                              StaticInstPtr curMacroop, TheISA::PCState thisPC,
                              TheISA::PCState nextPC, bool trace)
{
    assert(staticInst);

    // Get a sequence number.
    InstSeqNum seq;
    seq = cpu->getAndIncrementInstSeq();

    // Create a new DynInst from the instruction fetched.
    DynInstPtr instruction =
        new DynInst(staticInst, curMacroop, thisPC, nextPC, seq, cpu);
    instruction->setTid(tid);

    instruction->setASID(tid);

    instruction->setThreadState(cpu->thread[tid]);

    // Make load value prediction if necessary
    if (staticInst->isStreamedFromSpeculativeCache()) {
        ++specCacheHitOps;
    } else {
        ++specCacheMissOps;
    }



    if (instruction->staticInst->isStreamedFromUOpCache() && decoder[tid]->traceConstructor->IsValuePredictible(instruction->staticInst)) 
    {
        // don't check against new prediction is the instruction is part of a spec trace
        DPRINTF(LVP, "MakePrediction called by inst [sn:%i] from fetch!\n", seq);
        instruction->cycleFetched = cpu->numCycles.value();
        loadPred->stored_seq_no = seq;
	    
	    int inflight_count = 0;
        for (const DynInstPtr& cpuInst : cpu->instList) {
                if (cpuInst->instAddr() == thisPC.pc()) {
                    inflight_count += 1;
                }
        }
        loadPred->stored_inflight = inflight_count;	    

        LVPredUnit::lvpReturnValues ret;
        ret = loadPred->makePrediction(thisPC, tid, cpu->numCycles.value());
            
        if (loadPred->lvpredType == "eves")
            loadPred->lvLookups++;

	    //instruction->staticInst->lvpData = &ret;
		instruction->staticInst->predictedValue = ret.predictedValue;
		instruction->staticInst->confidence = ret.confidence;
		// if (instruction->staticInst->confidence) {std::cout << "Confident prediction at seqno " << instruction->seqNum << endl;}
		instruction->staticInst->predVtage = ret.predVtage;
		instruction->staticInst->predStride = ret.predStride;
		// if (instruction->staticInst->predStride) {std::cout << "Still confident...\n";}
		instruction->staticInst->prediction_result = ret.prediction_result;
		for (int i=0; i<9; i++) {
			instruction->staticInst->GTAG[i] = ret.GTAG[i];
			instruction->staticInst->GI[i] = ret.GI[i];
		}
		for (int i=0; i<3; i++) {
			instruction->staticInst->TAGSTR[i] = ret.TAGSTR[i];
			instruction->staticInst->B[i] = ret.B[i];
		}
		instruction->staticInst->STHIT = ret.STHIT;
		// cout << "Fetch received HitBank value " << ret.HitBank << "for sn " << instruction->seqNum << endl;
		instruction->staticInst->HitBank = ret.HitBank;
		DPRINTF(LVP, "fetch predicted %x with confidence %i\n", ret.predictedValue, ret.confidence);
        if (false && (cpu->numCycles.value() - loadPred->lastMisprediction < loadPred->resetDelay) && loadPred->dynamicThreshold) {
            DPRINTF(LVP, "Misprediction occured %i cycles ago, setting confidence to -1\n", cpu->numCycles.value() - loadPred->lastMisprediction);
            staticInst->predictedValue = ret.predictedValue;
            staticInst->confidence = 0;
            staticInst->predictedLoad = false;
        } else {
            DPRINTF(LVP, "Fetch Predicted value for Inst with PC: %#x SeqNum[%d] Setting Value to: %#x Setting confidence to: %d: \n", 
                          thisPC.instAddr(), instruction->seqNum, ret.predictedValue, ret.confidence);
            staticInst->predictedValue = ret.predictedValue;
            staticInst->confidence = ret.confidence;
            staticInst->predictedLoad = true;
        } 
        
    }


    DPRINTF(Fetch, "[tid:%i]: Instruction PC %#x (%d) created "
            "[sn:%lli].\n", tid, thisPC.instAddr(),
            thisPC.microPC(), seq);

    DPRINTF(Fetch, "[tid:%i]: Instruction is: %s\n", tid,
            instruction->staticInst->
            disassemble(thisPC.instAddr()));

#if TRACING_ON
    if (trace) {
        instruction->traceData =
            cpu->getTracer()->getInstRecord(curTick(), cpu->tcBase(tid),
                    instruction->staticInst, thisPC, curMacroop);
    }
#else
    instruction->traceData = NULL;
#endif


    	// Add instruction to the CPU's list of instructions.
    	instruction->setInstListIt(cpu->addInst(instruction));

    	// Write the instruction to the first slot in the queue
    	// that heads to decode.
    	assert(numInst < fetchWidth);
    	fetchQueue[tid].push_back(instruction);
    	assert(computeFetchQueueSize(tid) <= fetchQueueSize);
    	DPRINTF(Fetch, "[tid:%i]: Fetch queue entry created (%i/%i).\n",
    	        tid, fetchQueue[tid].size(), fetchQueueSize);
    	//toDecode->insts[toDecode->size++] = instruction;

    	// Keep track of if we can take an interrupt at this boundary
    	delayedCommit[tid] = instruction->isDelayedCommit();

        fetchedOps++;
    

    return instruction;
}

template<class Impl>
bool
DefaultFetch<Impl>::isStreamingFromSpeculativeCache(TheISA::PCState thisPC)
{
    // check the speculative cache even before the microop cache
    ThreadID tid = getFetchingThread(fetchPolicy);
    if (isSuperOptimizationActivated && !decoder[tid]->isSpeculativeCacheActive() && thisPC.upc() == 0) {

        currentTraceID = decoder[tid]->isTraceAvailable(FullUopAddr(thisPC.instAddr(), thisPC.upc()));

        if (currentTraceID != 0 && decoder[tid]->redirectDueToLVPSquashing) {
            DPRINTF(Fetch, "A trace is available but due to a previous LVP missprediction squash we can't fetch from spec$! Available TraceID is %i.\n", currentTraceID);
        }
        else if (currentTraceID != 0 && !decoder[tid]->redirectDueToLVPSquashing)
        {
            DPRINTF(Fetch, "A trace is available and Spec$ will be activated! Available TraceID is %i.\n", currentTraceID);
            
        }
        else if (currentTraceID == 0)
        {
            DPRINTF(Fetch, "No trace is Avaialble at this PC.\n", currentTraceID);
        }
    }

    
    if (isSuperOptimizationActivated && decoder[tid]->isSpeculativeCacheActive())
    {
        DPRINTF(Fetch, "Continue fetching from speculative cache at Pc %s. currentTraceID is %d\n", thisPC, currentTraceID);
        return true;
    }
    if (isSuperOptimizationActivated && currentTraceID && !decoder[tid]->redirectDueToLVPSquashing) 
    {
    
        DPRINTF(Fetch, "Setting speculative cache active at Pc %s. currentTraceID is %d\n", thisPC, currentTraceID);
        decoder[tid]->setSpeculativeCacheActive(true, currentTraceID);
        return true;
    }
    // we will set currentTraceID to zero in this function
    decoder[tid]->setSpeculativeCacheActive(false);
    return false;
}

template<class Impl>
bool
DefaultFetch<Impl>::isStreamingFromUopCache(TheISA::PCState thisPC)
{
    // Check the micro-op cache.  If we already find a translation
    // in the micro-op cache, bypass icache fetch and decode.
    ThreadID tid = getFetchingThread(fetchPolicy);
    if (isUopCachePresent && decoder[tid]->isHitInUopCache(thisPC.instAddr())) {
        DPRINTF(Fetch, "isStreamingFromUopCache: Setting microop cache active at Pc %s.\n", thisPC);
        //decoder[tid]->setUopCacheActive(true);
        // fetchBufferValid[tid] = false;
        return true;
    }

    //decoder[tid]->setUopCacheActive(false);
    return false;
}

template<class Impl>
void
DefaultFetch<Impl>::fetch(bool &status_change)
{

    #define ENABLE_DEBUG 0
    //////////////////////////////////////////
    // Start actual fetch
    //////////////////////////////////////////
    ThreadID tid = getFetchingThread(fetchPolicy);

    assert(!cpu->switchedOut());

    if (tid == InvalidThreadID) {
        // Breaks looping condition in tick()
        threadFetched = numFetchingThreads;

        if (numThreads == 1) {  // @todo Per-thread stats
            profileStall(0);
        }

        return;
    }

    DPRINTF(Fetch, "Attempting to fetch from [tid:%i]\n", tid);

    // The current PC.
    TheISA::PCState thisPC = pc[tid];

    Addr pcOffset = fetchOffset[tid];
    Addr fetchAddr = (thisPC.instAddr() + pcOffset) & BaseCPU::PCMask;

    bool inRom = isRomMicroPC(thisPC.microPC());

    // If returning from the delay of a cache miss, then update the status
    // to running, otherwise do the cache access.  Possibly move this up
    // to tick() function.
    if (fetchStatus[tid] == IcacheAccessComplete) {
        DPRINTF(Fetch, "[tid:%i]: Icache miss is complete.\n", tid);

        fetchStatus[tid] = Running;
        status_change = true;
    } else if (fetchStatus[tid] == Running) {
        // Align the fetch PC so its at the start of a fetch buffer segment.
        Addr fetchBufferBlockPC = fetchBufferAlignPC(fetchAddr);

        // We check spec cache first due to left to right associativity of &&
        if (isStreamingFromSpeculativeCache(thisPC))
        {
            fetchBufferValid[tid] = false;
        }
        else if ( isStreamingFromUopCache(thisPC))
        {
            decoder[tid]->setUopCacheActive(true);
            fetchBufferValid[tid] = false;
        }

        if (!decoder[tid]->isSpeculativeCacheActive() && !decoder[tid]->isUopCacheActive() &&
            !(fetchBufferValid[tid] && fetchBufferBlockPC == fetchBufferPC[tid])
            && !inRom && !macroop[tid]) {
            // If buffer is no longer valid or fetchAddr has moved to point
            // to the next cache block, AND we have no remaining ucode
            // from a macro-op, then start fetch from icache.
            DPRINTF(Fetch, "[tid:%i]: Attempting to translate and read "
                    "instruction from legacy decoder, starting at PC %s.\n", tid, thisPC);

            fetchCacheLine(fetchAddr, tid, thisPC.instAddr());

            if (fetchStatus[tid] == IcacheWaitResponse)
                ++icacheStallCycles;
            else if (fetchStatus[tid] == ItlbWait)
                ++fetchTlbCycles;
            else
                ++fetchMiscStallCycles;
            return;
        }
        if (!isStreamingFromSpeculativeCache(thisPC) && !isStreamingFromUopCache(thisPC) &&
            (checkInterrupt(thisPC.instAddr()) && !delayedCommit[tid])) {
            // Stall CPU if an interrupt is posted and we're not issuing
            // an delayed commit micro-op currently (delayed commit instructions
            // are not interruptable by interrupts, only faults)
            ++fetchMiscStallCycles;
            DPRINTF(Fetch, "[tid:%i]: Fetch is stalled!\n", tid);
            return;
        }
    } else {
        if (fetchStatus[tid] == Idle) {
            ++fetchIdleCycles;
            DPRINTF(Fetch, "[tid:%i]: Fetch is idle!\n", tid);
        }

        // Status is Idle, so fetch should do nothing.
        return;
    }

    ++fetchCycles;

#if 0
    DPRINTF(Fetch, "Cache Block before store forwarding:\n");
    for (int i=0; i<cacheBlkSize; i+=16) {
      RPRINTF(Fetch, "0x%x: ",icacheBlockAlignPC(fetchAddr)+i);
      for (int j=i; j<i+16; j++)
        RPRINTF(Fetch, "%x ", *(fetchBuffer[tid]+j));
      RPRINTF(Fetch, "\n");
    }

    // Try store to fetch forwarding -- just in case this is self modifying code
    cpu->tryStoreToFetchForwarding(tid, icacheBlockAlignPC(fetchAddr), cacheBlkSize, fetchBuffer[tid]);

    DPRINTF(Fetch, "Cache Block after store forwarding:\n");
    for (int i=0; i<cacheBlkSize; i+=16) {
      RPRINTF(Fetch, "0x%x: ",icacheBlockAlignPC(fetchAddr)+i);
      for (int j=i; j<i+16; j++)
        RPRINTF(Fetch, "%x ", *(fetchBuffer[tid]+j));
      RPRINTF(Fetch, "\n");
    }
#endif

    TheISA::PCState nextPC = thisPC;

    StaticInstPtr staticInst = NULL;
    StaticInstPtr curMacroop = macroop[tid];

    // If the read of the first instruction was successful, then grab the
    // instructions from the rest of the cache line and put them into the
    // queue heading to decode.

    DPRINTF(Fetch, "[tid:%i]: Adding instructions to queue to "
            "decode.\n", tid);

    // Need to keep track of whether or not a predicted branch
    // ended this fetch block.
    bool predictedBranch = false;

    // Need to halt fetch if quiesce instruction detected
    bool quiesce = false;

    const unsigned numInsts = fetchBufferSize / instSize;
    unsigned blkOffset = (fetchAddr - fetchBufferPC[tid]) / instSize;

    // every cycle check to see if we are coming back from a LVP/folded branch squash
    // one cycle is enough to fall back from speculative cache to uop cache in case of a LVP/folded branch missprediction
    if (decoder[tid]->redirectDueToLVPSquashing) decoder[tid]->redirectDueToLVPSquashing = false;


    bool switchToDecodePipeline = false;
    bool switchToSpecUopCachePipeline = false;
    // Loop through instruction memory from the cache.
    // Keep issuing while fetchWidth is available and branch is not
    // predicted taken
    int adjustedFetchWidth = fetchWidth;
    if (!decoder[tid]->isSpeculativeCacheActive() && !decoder[tid]->isUopCacheActive()){
        adjustedFetchWidth = fetchWidth - 1;
    }
    while (numInst < adjustedFetchWidth && computeFetchQueueSize(tid) < fetchQueueSize //LOGAN TODO LOWER FETCH WIDTH FROM LEGACY DECODE
           && !predictedBranch && !quiesce) {
        // We need to process more memory if we aren't going to get a
        // StaticInst from the rom, the current macroop, or what's already
        // in the decoder.


        //*****CHANGE START**********
        

        bool needMem = !inRom && !curMacroop && !decoder[tid]->isSpeculativeCacheActive() && !decoder[tid]->isUopCacheActive() && 
            !decoder[tid]->instReady();
        //*****CHANGE START**********
        DPRINTF(Fetch, "Flags Status: needMem = %d, instReady = %d, isSpeculativeCacheActive = %d, isUopCacheActive = %d\n", 
                        needMem, decoder[tid]->instReady(), decoder[tid]->isSpeculativeCacheActive(), decoder[tid]->isUopCacheActive());

        fetchAddr = (thisPC.instAddr() + pcOffset) & BaseCPU::PCMask;
        Addr fetchBufferBlockPC = fetchBufferAlignPC(fetchAddr);

        if (needMem) {
            TheISA::MachInst *cacheInsts =
                reinterpret_cast<TheISA::MachInst *>(fetchBuffer[tid]);

            // If buffer is no longer valid or fetchAddr has moved to point
            // to the next cache block then start fetch from icache.
            if (!fetchBufferValid[tid] ||
                fetchBufferBlockPC != fetchBufferPC[tid])
                break;

            if (blkOffset >= numInsts) {
                // We need to process more memory, but we've run out of the
                // current block.
                break;
            }
            

            MachInst inst = TheISA::gtoh(cacheInsts[blkOffset]);
            decoder[tid]->moreBytes(thisPC, fetchAddr, inst);

            if (decoder[tid]->needMoreBytes()) {
                blkOffset++;
                fetchAddr += instSize;
                pcOffset += instSize;
            }
        }

        // Extract as many instructions and/or microops as we can from
        // the memory we've processed so far.
        StaticInstPtr prevSpecInst = NULL;
        do {
                bool newMacro = false, fused = false;

                //*****CHANGE START**********
                if (isSuperOptimizationActivated && decoder[tid]->isSpeculativeCacheActive()) {

                    
                    // uop cache may be activated but the legacy decoder should never be active when spec or uop caches are active
                    
                    
                    bool predict_taken = false;
                    // fetch next microop and also update the nextPC, so we can decide whether there is
                    // TODO: fetchBufferPC[tid] ?
                    DPRINTF(Fetch, "Asking SPEC for microop at %s and to update %s (%d). currentTraceID is %d\n", thisPC, nextPC, nextPC.valid, currentTraceID);

                    staticInst = decoder[tid]->getSuperOptimizedMicroop(currentTraceID, thisPC, nextPC, predict_taken);
                    assert(staticInst);
                    
                    if (staticInst->isEndOfTrace()) {
                        DPRINTF(Fetch, "Received from SPEC nextPC %s (%d) and the last microop of the trace! currentTraceID is %d\n", nextPC, nextPC.valid, currentTraceID);
                    } else {
                        DPRINTF(Fetch, "Received from SPEC nextPC %s (%d) and a valid next inst! currentTraceID is %d\n", nextPC, nextPC.valid, currentTraceID);
                    }

                    // to make sure that we never use decoder/uop cache and speculative cache in parallel 
                    curMacroop = NULL;

                    // to make sure that we always have the macroop for every microop
                    //assert(staticInst->macroOp != StaticInst::nullStaticInstPtr);

                    panic_if(staticInst->getName() != "syscall" && !staticInst->macroOp, "Microop without macroOp! %s\n",staticInst->getName());
                    staticInst->setStreamedFromSpeculativeCache(true);

                    /* Micro-fusion. */
                    if (isMicroFusionPresent && prevSpecInst && prevSpecInst->isStreamedFromSpeculativeCache()) {
                        if ((staticInst->isInteger() || staticInst->isNop() ||
                            staticInst->isControl() || staticInst->isMicroBranch()) &&
                            prevSpecInst->isLoad() && !prevSpecInst->isRipRel()) {
                            fused = true;
                        }
                    }
                    prevSpecInst = staticInst;
 
                    DynInstPtr instruction = buildInst(tid, staticInst, staticInst->macroOp,
                                                        thisPC, nextPC, true);
                    instruction->fused = fused;

                    //std::cout << "IsSpecCacheActive = " << decoder[tid]->isSpeculativeCacheActive() << " IsUopCacheActive = " << decoder[tid]->isUopCacheActive() << std::endl;
                
                    DPRINTF(Fetch, "Speculative instruction created: [sn:%lli]:%s thisPC = %s nextPC = %s. currentTraceID is %d \n", 
                                instruction->seqNum, instruction->pcState(), thisPC, nextPC, currentTraceID);
                    DPRINTF(Fetch, "pc:%#x, npc:%#x, upc:%#x, nupc:%#x, size:%#x\n", instruction->pcState().pc(), instruction->pcState().npc(), instruction->pcState().upc(), instruction->pcState().nupc(), instruction->pcState().size());
                    for (int j=0; j<staticInst->numSrcRegs(); j++) {
                        if (staticInst->sourcesPredicted[j])
                            DPRINTF(Fetch, "Speculative instruction has propagated constant %#x at operand %#x\n", staticInst->sourcePredictions[j], j);
                    }
                    for (int i = 0; i < staticInst->numDestRegs(); i++) {
                        if (staticInst->liveOutPredicted[i]) {
                            DPRINTF(Fetch, "Operand %i has a live out value %#x\n", i, staticInst->liveOut[i]);
                        }
                    }

                    ppFetch->notify(instruction);
                    numInst++;
                    
                    /*if (staticInst->rasPushIndicator) {
                        DPRINTF(Fetch, "RAS push indication received for branch to be folded: %s\n", staticInst->rasPushAddress);
                        branchPred->RAS[tid].push(staticInst->rasPushAddress);
                    } else if (staticInst->rasPopIndicator) {
                        DPRINTF(Fetch, "RAS pop indication received for branch to be folded\n");
                        staticInst->rasPopAddress = branchPred->RAS[tid].top();
                        branchPred->RAS[tid].pop();
                    }*/
                    assert (!(staticInst->rasPushIndicator && staticInst->rasPopIndicator));
                    if (staticInst->rasPushIndicator || staticInst->rasPopIndicator){
                        branchPred->createPredHistoryForFoldedBranch(staticInst, instruction->seqNum, thisPC, tid);
                    }

                    bool thwartMisprediction = false;
                    if (!instruction->isControl()) {
                        //assert(thisPC.instAddr() == nextPC.instAddr());
                        instruction->setPredTarg(nextPC);
                        instruction->setPredTaken(false);
    
                    } else if (instruction->staticInst->dummyMicroop) {
                        DPRINTF(Fetch, "Encountered a DUMMY MICROOP which is a control, skipping branch preiction\n");
                        DPRINTF(Fetch, "Speculative instruction: [sn:%lli]:%s thisPC = %s nextPC = %s. currentTraceID is %d \n", 
                                instruction->seqNum, instruction->pcState(), thisPC, nextPC, currentTraceID);
                        DPRINTF(Fetch, "pc:%#x, npc:%#x, upc:%#x, nupc:%#x, size:%#x\n", instruction->pcState().pc(), instruction->pcState().npc(), instruction->pcState().upc(), instruction->pcState().nupc(), instruction->pcState().size());
                    
                        //instruction->setPredTarg(instruction->staticInst->predictedTarget);
                        //instruction->setPredTaken(instruction->staticInst->predictedTaken);
                        //predict_taken = instruction->staticInst->predictedTaken;
                        //nextPC = instruction->staticInst->predictedTarget;

                        instruction->setPredTarg(nextPC);
                        instruction->setPredTaken(false);
                        //predict_taken = instruction->staticInst->predictedTaken;
                        //nextPC = instruction->staticInst->predictedTarget;

                        //instruction->branchPredFromTrace = true;

                        
                        DPRINTF(Fetch, "Folded branch target: %s.\n", nextPC);

                        DPRINTF(Fetch, "Folded branch predicted to go to %s.\n", nextPC);
                        DPRINTF(Fetch, "Speculative instruction: [sn:%lli]:%s thisPC = %s nextPC = %s. currentTraceID is %d \n", 
                                instruction->seqNum, instruction->pcState(), thisPC, nextPC, currentTraceID);
                        DPRINTF(Fetch, "pc:%#x, npc:%#x, upc:%#x, nupc:%#x, size:%#x\n", instruction->pcState().pc(), instruction->pcState().npc(), instruction->pcState().upc(), instruction->pcState().nupc(), instruction->pcState().size());
                    
                    } else {
                        //branch without a high confidence prediction at the end of a macroop at the end of a trace, 
                        //in which case we aren't able to update nextPC and predicted_branch accurately from the speculative cache
                        // here we use the default lookupAndUpdate mechanism to update the nextPC
                        // here I assume that this branch is at the end of a trace, therefore the next time we call getSuperoptimizedMicroop
                        // we should always get StaticInst::nullStaticInstPtr.
                        
                        DPRINTF(Fetch, "Branch detected with PC = %s\n", thisPC);
                        assert(!instruction->branchPredFromPredictor);
                        assert(!instruction->branchPredFromTrace);

                        bool bpred_predict_taken = false; 
                        TheISA::PCState bpred_nextPC = thisPC;
                        bpred_predict_taken = branchPred->predict(instruction->staticInst, instruction->seqNum,
                                bpred_nextPC, tid);

                        // Find the trace's confidence of this branch
                        unsigned trace_confidence = 0;
                        for (size_t idx = 0; idx < 2; idx++)
                        {
                            if (decoder[tid]->traceConstructor->traceMap[currentTraceID].controlSources[idx].valid &&
                                decoder[tid]->traceConstructor->traceMap[currentTraceID].controlSources[idx].value ==  thisPC.instAddr())
                            {
                                trace_confidence = decoder[tid]->traceConstructor->traceMap[currentTraceID].controlSources[idx].confidence;
                            }
                        }

                        //Determine which branch prediction to use
                        if (trace_confidence > decoder[tid]->traceConstructor->controlPredictionConfidenceThreshold){
                            DPRINTF(Fetch, "Trace confidence of contro sorce is high, using its prediction\n");
                            predict_taken = instruction->staticInst->predictedTaken;
                            nextPC = instruction->staticInst->predictedTarget;
                        } else {
                            DPRINTF(Fetch, "Trace confidence of control source is low, using branch predictor\n");
                            thwartMisprediction = true;
                            predict_taken = bpred_predict_taken;
                            nextPC = bpred_nextPC;
                        }
                        instruction->setPredTarg(nextPC);
                        instruction->setPredTaken(predict_taken);

                        //Assign blame
                        if (instruction->staticInst->predictedTarget.instAddr() == nextPC.instAddr() && instruction->staticInst->predictedTaken == predict_taken){
                            instruction->branchPredFromTrace = true;
                        }
                        if (bpred_nextPC.instAddr() == nextPC.instAddr() && bpred_predict_taken == predict_taken){
                            instruction->branchPredFromPredictor = true;
                        }
                        assert(instruction->branchPredFromTrace || instruction->branchPredFromPredictor);
                        /*
                        if (!nextPC.valid) {
                            predict_taken = bpred_predict_taken;
                            nextPC = bpred_nextPC;
                            instruction->setPredTarg(nextPC);
                            instruction->setPredTaken(predict_taken);
                            instruction->branchPredFromPredictor = false;
                        } else {
                            //instruction->setPredTarg(instruction->staticInst->predictedTarget);
                            //instruction->setPredTaken(instruction->staticInst->predictedTaken);
                            if (instruction->staticInst->predictedTarget.instAddr() == bpred_nextPC.instAddr() && instruction->staticInst->predictedTaken == bpred_predict_taken) {
                                instruction->setPredTarg(instruction->staticInst->predictedTarget);
                                instruction->setPredTaken(instruction->staticInst->predictedTaken);
                                instruction->branchPredFromPredictor = true;
                                predict_taken = instruction->staticInst->predictedTaken;
                                //nextPC = instruction->staticInst->predictedTarget; // This shouldn't change anything
                            } else {
                                DPRINTF(Fetch, "Thwarting misprediction\n");
                                thwartMisprediction = true;
                                predict_taken = bpred_predict_taken;
                                nextPC = bpred_nextPC;
                                instruction->setPredTarg(nextPC);
                                instruction->setPredTaken(predict_taken);
                                instruction->branchPredFromPredictor = true;
                            }
                            //predict_taken = instruction->staticInst->predictedTaken;
                            //instruction->branchPredFromPredictor = false;
                        }*/
                        
                        if (thisPC.branching()) {
                            DPRINTF(Fetch, "Folded branch detected with PC = %s\n", thisPC);
                        }

                        if (predict_taken) {
                            DPRINTF(Fetch, "[tid:%i]: [sn:%i]: Folded branch predicted to be taken to %s.\n",
                                        tid, instruction->seqNum, nextPC);
                        } else {
                            DPRINTF(Fetch, "[tid:%i]: [sn:%i]: Folded branch predicted to be not taken.\n",
                                        tid, instruction->seqNum);
                        }

                        DPRINTF(Fetch, "[tid:%i]: [sn:%i] Folded branch predicted to go to %s.\n",
                                    tid, instruction->seqNum, nextPC);
                        

                        ++fetchedBranches;

                        if (predict_taken) {
                            ++predictedBranches;
                        }
                    }

                    
                    thisPC = nextPC;

                 
                    ++instsPartOfOptimizedTrace;
                    

                    if (instruction->isQuiesce()) {
                        DPRINTF(Fetch,
                                "Quiesce instruction encountered, halting fetch!\n");
                        fetchStatus[tid] = QuiescePending;
                        status_change = true;
                        quiesce = true;
                        decoder[tid]->setSpeculativeCacheActive(false);
                        break;
                    }

                    if (staticInst->isEndOfTrace() || thwartMisprediction) {
                        
                        decoder[tid]->setSpeculativeCacheActive(false);
                        decoder[tid]->setUopCacheActive(false);
                        // set where to fetch for the next cycle
                        // we need to make sure that fetchBuffer has the necessary data for the next macroop fetch
                        // do we need pcOffset? 
                        // fetchBufferPC[tid] is updated at the begining of streaming microops from speculative cache 
                        thisPC = nextPC; 
                        thisPC.upc(0); thisPC.nupc(1);
                        decoder[tid]->traceConstructor->streamTrace.id = 0;
                        
                        // 
                        // When we are done with streaming from speculative cache we always know that we are at the 
                        // end of the prevoius macroop, therefore, pcOffset is always zero for a new macroop
                        pcOffset = 0;  
                        curMacroop = NULL;
                        newMacro = true;

                        // check if we can stream from the spec cache or uop cache -- if yes, this will set the spec cache, uop cache active
                        // Switching State Machine
                        if (isStreamingFromSpeculativeCache(thisPC))            
                        {
                            switchToDecodePipeline = false;
                            switchToSpecUopCachePipeline = false;
                            decoder[tid]->resetDecoder();
                        }
                        else if (isStreamingFromUopCache(thisPC))               
                        {
                            currentTraceID = 0;
                            switchToDecodePipeline = false;
                            switchToSpecUopCachePipeline = false;
                            decoder[tid]->setUopCacheActive(true);
                            decoder[tid]->resetDecoder();
                        }
                        else                                                    
                        {
                            currentTraceID = 0;
                            switchToDecodePipeline = true;
                            switchToSpecUopCachePipeline = false;
                            // Disable both caches! 
                            // streaming from the current trace.  i.e., look for a brand new trace to stream from.
                            decoder[tid]->setUopCacheActive(false);
                            decoder[tid]->resetDecoder();
                        }

                        // lets check to see if there is enough bytes in the current fetchBuffer so the decoder can use them   
                        // if we have passed the border between two fetch buffer we need to issue an I-cache request
                        // here just break, at the end of fetch() function it will handel this automaticly  
                        // if the next instruction is a branch, this will handel that automaticly too. 

                        
                        fetchAddr = thisPC.instAddr() & BaseCPU::PCMask;
                        Addr fetchBufferBlockPC = fetchBufferAlignPC(fetchAddr);
                        // fortunatly fetchBufferPC[tid] is updated somewhere else
                        if (fetchBufferBlockPC != fetchBufferPC[tid]) {
                            DPRINTF(Fetch, "[tid:%i]: Done streaming from speculative cache and "
                                            "fetch buffer is not valid anymore!. Fetch continues to stream from %s\n", tid, thisPC);
                            
                            fetchBufferValid[tid] = false;

                            
                            blkOffset = (fetchAddr - fetchBufferPC[tid]) / instSize;

                            DPRINTF(Fetch, "fetchBuffer is not valid with PC: %#x fetchAddr: %#x fetchBufferPC: %#x blkoffset: %d fetchBuffer: 0x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x\n", thisPC.instAddr() ,fetchAddr, fetchBufferPC[tid], blkOffset,
                                fetchBuffer[tid][0],fetchBuffer[tid][1],fetchBuffer[tid][2],fetchBuffer[tid][3],fetchBuffer[tid][4],fetchBuffer[tid][5],fetchBuffer[tid][6],fetchBuffer[tid][7],
                                fetchBuffer[tid][8],fetchBuffer[tid][9],fetchBuffer[tid][10],fetchBuffer[tid][11],fetchBuffer[tid][12],fetchBuffer[tid][13],fetchBuffer[tid][14],fetchBuffer[tid][15]            
                            );
                            // when we break, we need to make sure that decoder[tid]->instReady() always return false;
                            // otherwise we will go into a loop!
                            // if this assert gets activated we need to reset instReady manualy in decoder.  
                            assert(!decoder[tid]->instReady()); // just to make sure!
                            continue;
                        }
                        
                        // if one of these asserts gets activated it means there is a corner case
                        // it doesn't make sense (fetchBufferBlockPC == fetchBufferPC[tid]) to false
                        // assert(fetchBufferBlockPC == fetchBufferPC[tid]);

                        DPRINTF(Fetch, "[tid:%i]: Done streaming from speculative cache and "
                                            "fetch buffer is still valid!.\n", tid);
                        fetchBufferValid[tid] = false;

                        
                        fetchAddr = thisPC.instAddr() & BaseCPU::PCMask;
                        blkOffset = (fetchAddr - fetchBufferPC[tid]) / instSize;
                       

                        DPRINTF(Fetch, "fetchBuffer is valid with PC: %#x fetchAddr: %#x fetchBufferPC: %#x blkoffset: %d fetchBuffer: 0x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x\n", thisPC.instAddr() ,fetchAddr, fetchBufferPC[tid], blkOffset,
                            fetchBuffer[tid][0],fetchBuffer[tid][1],fetchBuffer[tid][2],fetchBuffer[tid][3],fetchBuffer[tid][4],fetchBuffer[tid][5],fetchBuffer[tid][6],fetchBuffer[tid][7],
                            fetchBuffer[tid][8],fetchBuffer[tid][9],fetchBuffer[tid][10],fetchBuffer[tid][11],fetchBuffer[tid][12],fetchBuffer[tid][13],fetchBuffer[tid][14],fetchBuffer[tid][15]            
                        );
                        
                        // decoder never should say the next instruction is ready at anytime when returning from the speculative cache
                        // if this assert gets activated we need to reset instReady manualy in decoder. 
                        assert(!decoder[tid]->instReady()); 

                        // let's count number of wasted fetch bandwidth cycles due to switch
                        assert(numInst <= adjustedFetchWidth);
                    }

                    // jumps to the while loop condition and if:
                    // if  (numInst < fetchWidth && computeFetchQueueSize(tid) < fetchQueueSize) continue fetching from microop cache
                    continue;
                } else {
                    ++instsNotPartOfOptimizedTrace;
                }   

                // this chunk of code is for uop cache and legacy decoder
            	if (!(curMacroop || inRom)) {
               	    if (decoder[tid]->instReady() || decoder[tid]->isUopCacheActive()) 
                    {
                        // Spec cache should never be active if we are here
                        if (isSuperOptimizationActivated) assert(!decoder[tid]->isSpeculativeCacheActive());
                        
                        // if uop cache is active it should always results in hit!
                        if (isUopCachePresent && decoder[tid]->isUopCacheActive()) assert(decoder[tid]->isHitInUopCache(thisPC.instAddr()));

                    	staticInst = decoder[tid]->decode(thisPC, cpu->numCycles.value(), tid);

                    	for (int i=0; i<staticInst->numSrcRegs(); i++) {
                          DPRINTF(Fetch, "arch:%d \n", staticInst->srcRegIdx(i));
                    	}
                    	DPRINTF(Fetch, "\n");

                    	// Increment stat of fetched instructions.
                    	++fetchedInsts;

                    	if (decoder[tid]->isUopCacheActive()) {
                    	  ++uopCacheHitInsts;
                    	}

                    	if (staticInst->isMacroop()) {
                    	    curMacroop = staticInst;
                    	} else {
                    	    pcOffset = 0;
                    	}
                        // Try to queue a new trace if this is macroop is hot!
                        if (isUopCachePresent && staticInst->isStreamedFromUOpCache())
                            DPRINTF(TraceQueue, "%#x\n", thisPC.instAddr());
                        if (isSuperOptimizationActivated && staticInst->isStreamedFromUOpCache()
                                && staticInst->isUOpCacheHotTrace() && thisPC.upc() == 0) {
                            decoder[tid]->traceConstructor->QueueHotTraceForSuperOptimization(thisPC);
                        }
                            

                    } else {
                    	// We need more bytes for this instruction so blkOffset and
                    	// pcOffset will be updated
                    	break;
                    }
            	}
            	// Whether we're moving to a new macroop because we're at the
            	// end of the current one, or the branch predictor incorrectly
            	// thinks we are...
                if ( (curMacroop || inRom)) 
                {
                    if (inRom) {
                    	staticInst = cpu->microcodeRom.fetchMicroop(thisPC.microPC(), curMacroop);
			            staticInst->macroOp = curMacroop;
                    } else {
                    	staticInst = curMacroop->fetchMicroop(thisPC.microPC());
                        staticInst->macroOp = curMacroop;
                        /* Micro-fusion. */
                    	if (isMicroFusionPresent && thisPC.microPC() != 0) {
                    	    StaticInstPtr prevStaticInst = curMacroop->fetchMicroop(thisPC.microPC()-1);
                            if (!prevStaticInst->isStreamedFromSpeculativeCache() && prevStaticInst->isStreamedFromUOpCache()) {
                                prevStaticInst->macroOp = curMacroop;
                                if ((staticInst->isInteger() || staticInst->isNop() ||
                                        staticInst->isControl() || staticInst->isMicroBranch()) &&
                                        prevStaticInst->isLoad() && !prevStaticInst->isRipRel()) {
                                    fused = true;
                                }
                            }

                    	}
                    	if (decoder[tid]->isUopCacheActive() && !staticInst->isStreamedFromSpeculativeCache()) 
                        {
                            assert(decoder[tid]->isHitInUopCache(thisPC.instAddr()));
                    	    ++uopCacheHitOps;
                    	    DPRINTF(Fetch,"Counting pc=%s as a uopCache hit \n",thisPC);
                    	} 
                        else 
                        {
                    	    ++uopCacheMissOps;
                    	    DPRINTF(Fetch,"Counting pc=%s as a uopCache miss \n",thisPC);
                    	}

                    	if (staticInst->isFirstMicroop() && staticInst->isLastMicroop())
                    	    statFetchMicro[tid]++;
                    }
                    for (int i=0; i<staticInst->numSrcRegs(); i++) {
                      DPRINTF(Fetch, "arch:%d \n", staticInst->srcRegIdx(i));
                    }
                    DPRINTF(Fetch, "\n");
                    newMacro |= staticInst->isLastMicroop();
            	}

                bool skipInst = false;
                if (isLoopStreamDetectionPresent) {
                    DPRINTF(LSD, "PC: 0x%lx, uPC: 0x%lx, disas: %s\n", thisPC.pc(), thisPC.upc(), staticInst->disassemble(thisPC.pc()));

                    struct fullAddr addr(thisPC.pc(), thisPC.upc());
                    lsd->update(addr);

                    if (lsd->inLoop()) {
                        if (lsd->isLoopHead()) {
                            if (lsd->isFirstOfficialIteration()){
                                DPRINTF(LSD, "LOOP_DETECTED\nBEGIN\n%sEND\n", lsd->generateLoopElemsStr());
                                
                                if (isVectorWideningPresent) {
                                    struct loopInfo loop_info = lsd->getLoopInfo();
                                    vw->reset(loop_info.start_addr, loop_info.end_addr);
                                }
                            }
                            DPRINTF(LSD, "LOOP: cur_iter = %u\n", lsd->getLoopIteration());
                        }
                        if (isVectorWideningPresent) {
                            bool widenInst = false;
                            vw->processInst(addr, staticInst, lsd->getLoopIteration(), widenInst, skipInst);
                            if (widenInst) {
                                DPRINTF(Fetch, "Widening instruction!\n");
                            }
                            if (skipInst) {
                                assert(!thisPC.branching() && !staticInst->isControl() && staticInst->isVector());
                                DPRINTF(Fetch, "Skipping instruction!\n");
                            }
                        }
                    } else if (isVectorWideningPresent) {
                        vw->deactivate();
                    }
                }
                if (skipInst) assert(isVectorWideningPresent);

                DynInstPtr instruction;
                if (!skipInst) {
                    instruction = buildInst(tid, staticInst, curMacroop, thisPC, nextPC, true);
                    instruction->fused = fused;
            
                    DPRINTF(Fetch, "instruction created: [sn:%lli]:%s\n", instruction->seqNum, instruction->pcState());

                    ppFetch->notify(instruction);
                    if (!instruction->fused) numInst++;
                }

                nextPC = thisPC;

                // If we're branching after this instruction, quit fetching
                // from the same block.
                DPRINTF(Fetch, "PC.pc():%#x PC.npc():%#x PC.size():%#x\n", thisPC.pc(), thisPC.npc(), thisPC.size());
                DPRINTF(Fetch, "PC.upc():%#x PC.nupc():%#x\n", thisPC.upc(), thisPC.nupc());
                DPRINTF(Fetch, "PC.branching():%d\n", thisPC.branching());
                predictedBranch |= thisPC.branching();
                if (skipInst) {
                    TheISA::advancePC(nextPC, staticInst);
                } else {
                    predictedBranch |=
                        lookupAndUpdateNextPC(instruction, nextPC);
                }
                if (predictedBranch) {
                    DPRINTF(Fetch, "Branch detected with PC = %s\n", thisPC);
                }

                newMacro |= thisPC.instAddr() != nextPC.instAddr();

                // Move to the next instruction, unless we have a branch.
                thisPC = nextPC;
                inRom = isRomMicroPC(thisPC.microPC());

                if (newMacro) {
                    
                    fetchAddr = thisPC.instAddr() & BaseCPU::PCMask;
                    blkOffset = (fetchAddr - fetchBufferPC[tid]) / instSize;
                    pcOffset = 0;
                    curMacroop = NULL;
                    DPRINTF(Fetch, "A new macro is needed with fetchAddr: %#x fetchBufferPC: %#x instSize: %d blkoffset: %d\n", fetchAddr, fetchBufferPC[tid], instSize, blkOffset);
                }

                if (!skipInst && instruction->isQuiesce()) {
                    DPRINTF(Fetch,
                            "Quiesce instruction encountered, halting fetch!\n");
                    fetchStatus[tid] = QuiescePending;
                    status_change = true;
                    quiesce = true;
                    break;
                }



                /* Model penalty due to switch to decode pipeline. */
                if (newMacro) 
                {
                    // Becuase spec cache streaming logic is completely decoupled from uop cache/legacy decoder, if we are here it should always be disabled
                    // We will never be here if we are streaming from spec cache but we may be here if we are streaming from uop cache
                    assert(!decoder[tid]->isSpeculativeCacheActive());

                    // Scenario #1: In this cycle we were fetching from uop cache. 
                    // Now isStreamingFromSpeculativeCache() tells us that we can start fetch from spec cache therefore we need to switch!
                    // but this doesnt have any kind of penalty as both caches are active and indexed at the same time
                    if (isSuperOptimizationActivated && decoder[tid]->isUopCacheActive() && isStreamingFromSpeculativeCache(thisPC)) 
                    {
                        DPRINTF(Fetch, "Switching from Uop Cache to Spec Cache at PC:%s!\n", thisPC);
                        assert(decoder[tid]->isSpeculativeCacheActive());
                        // reset decoder state whenever it's disabled
                        decoder[tid]->resetDecoder();
                        switchToSpecUopCachePipeline = false;
                        switchToDecodePipeline = false;
                        fetchBufferValid[tid] = false;
                    
                    } 
                    // Scenario #2: In this cycle we were fetching from legacy decoder, therefore both caches were disabled. 
                    // Now isStreamingFromSpeculativeCache() tells us that we can start fetch from spec cache therefore we need to switch!
                    else if (isSuperOptimizationActivated && !decoder[tid]->isUopCacheActive() && isStreamingFromSpeculativeCache(thisPC)) 
                    {
                        DPRINTF(Fetch, "Switching from Legacy Decoder to Spec Cache pipline at PC:%s!\n", thisPC);
                        assert(decoder[tid]->isSpeculativeCacheActive());
                        // reset decoder state whenever we are sqitching to uop/sepc cache pipeline
                        decoder[tid]->resetDecoder();
                        switchToSpecUopCachePipeline = true;
                        switchToDecodePipeline = false;
                        break;
                    
                    } 
                    // Scenario #3: In this cycle we were fetching from legacy decoder, therefore both caches were disabled. 
                    // Now isStreamingFromUopCache() tells us that we can start fetch from uop cache therefore we need to switch!
                    else if (isUopCachePresent  && !decoder[tid]->isUopCacheActive() && isStreamingFromUopCache(thisPC)) 
                    {
                        
                        DPRINTF(Fetch, "Switching from Legacy Decoder to Uop Cache pipeline at PC:%s!\n", thisPC);
                        decoder[tid]->setUopCacheActive(true);
                        //assert(decoder[tid]->isUopCacheActive());
                        // reset decoder state whenever it's disabled
                        decoder[tid]->resetDecoder();
                        switchToSpecUopCachePipeline = true;
                        switchToDecodePipeline = false;
                        break;
                        
                    }
                    // Scenario #4: In this cycle we were fetching from legacy decoder, therefore both caches were disabled. 
                    // Now isStreamingFromUopCache() tells us that there is no hit in uop cache. Therfore, we continue fetching from Lagacy Decoder
                    else if (isUopCachePresent  && !decoder[tid]->isUopCacheActive() && !isStreamingFromUopCache(thisPC)) 
                    {
                        
                        DPRINTF(Fetch, "Continue fetching from Legacy Decoder pipeline at PC:%s!\n", thisPC);
                        assert(!decoder[tid]->isUopCacheActive());
                        switchToSpecUopCachePipeline = false;
                        switchToDecodePipeline = false;
                        
                    }
                    // Scenario #5: In this cycle we were fetching from uop cache. Therefore, only uop cache is enabled.
                    // Now isStreamingFromUopCache() tells us that we STILL can fetch from uop cache therefore we don't need to switch!
                    else if (isUopCachePresent &&  decoder[tid]->isUopCacheActive() && isStreamingFromUopCache(thisPC)) 
                    {
                        
                        DPRINTF(Fetch, "Continue fetching from Uop Cache pipeline at PC:%s!\n", thisPC);
                        assert(decoder[tid]->isUopCacheActive());
                        // reset decoder state whenever it's disabled
                        decoder[tid]->resetDecoder();
                        switchToSpecUopCachePipeline = false;
                        switchToDecodePipeline = false;
                        fetchBufferValid[tid] = false;
                        
                    }
                    // Scenario #6: In this cycle we were fetching from uop cache. Therefore, only uop cache is enabled.
                    // Now isStreamingFromUopCache() tells us that we can't continue fetching from uop cache anymore therefore we need to switch!
                    else if (isUopCachePresent  && decoder[tid]->isUopCacheActive() && !isStreamingFromUopCache(thisPC)) 
                    {
                        
                        DPRINTF(Fetch, "Switching from Uop Cache pipeline to lagacy decoder pipeline at PC:%s\n", thisPC);
                        //assert(!decoder[tid]->isUopCacheActive());
                        decoder[tid]->setUopCacheActive(false);
                        // reset decoder state whenever it's disabled
                        decoder[tid]->resetDecoder();
                        switchToSpecUopCachePipeline = false;
                        switchToDecodePipeline = true;
                        fetchBufferValid[tid] = false;
                        break;
                        
                    }
                    else 
                    {
                        assert(0);
                        DPRINTF(Fetch, "Uop and Spec Cache are not present!\n");
                        assert(!decoder[tid]->isUopCacheActive());
                        switchToSpecUopCachePipeline = false;
                        switchToDecodePipeline = false;
                    }
                }

                if (!skipInst && !instruction->isStreamedFromSpeculativeCache() && !instruction->isStreamedFromUOpCache()){
                    //assert(adjustedFetchWidth == fetchWidth - 1);
                    adjustedFetchWidth = fetchWidth - 1;
                }

        } 
        while (
                  (
                    (isSuperOptimizationActivated && decoder[tid]->isSpeculativeCacheActive()) ||  // for spec cache
                    (isUopCachePresent && decoder[tid]->isUopCacheActive()) ||  // for uop cache
                    (curMacroop || decoder[tid]->instReady())  // for legacy decoder/uop cache
                  ) && 
                  numInst < adjustedFetchWidth &&
                  computeFetchQueueSize(tid) < fetchQueueSize
        );

        if (switchToDecodePipeline) 
        {
            DPRINTF(Fetch, "Switching to Legacy Decoder pipeline!\n");
            assert(!decoder[tid]->isUopCacheActive());
            assert(!decoder[tid]->isSpeculativeCacheActive());
            break;
        }
        else if (switchToSpecUopCachePipeline) 
        {
            DPRINTF(Fetch, "Switching to Spec/Uop Caches pipeline!\n");
            assert(decoder[tid]->isUopCacheActive() || decoder[tid]->isSpeculativeCacheActive());
            break;
        }
    }

    if (switchToDecodePipeline) {
        // if we are here, it means we couldn't find 
        DPRINTF(Fetch, "[tid:%i]: microop cache miss.\n", tid);
    }
    if (predictedBranch) {
        DPRINTF(Fetch, "[tid:%i]: Done fetching, predicted branch "
                "instruction encountered.\n", tid);
    } else if (numInst >= adjustedFetchWidth) {
        DPRINTF(Fetch, "[tid:%i]: Done fetching, reached fetch bandwidth "
                "for this cycle.\n", tid);
    } else if (blkOffset >= fetchBufferSize) {
        DPRINTF(Fetch, "[tid:%i]: Done fetching, reached the end of the"
                "fetch buffer.\n", tid);
    }

    macroop[tid] = curMacroop;
    fetchOffset[tid] = pcOffset;

    if (numInst > 0) {
        wroteToTimeBuffer = true;
    }

    pc[tid] = thisPC;

    // pipeline a fetch if we're crossing a fetch buffer boundary and not in
    // a state that would preclude fetching
    fetchAddr = (thisPC.instAddr() + pcOffset) & BaseCPU::PCMask;
    Addr fetchBufferBlockPC = fetchBufferAlignPC(fetchAddr);

    
    //*****CHANGE START**********
    // I added inSpeculativeCache flag here, because when we know that in the next cycle we are going to fetch from speculative cache ...
    // we should never issue an I-cache fetch
    // hopefully it's not gonna change the behaviour of fetch stage
    issuePipelinedIfetch[tid] = fetchBufferBlockPC != fetchBufferPC[tid] &&
        fetchStatus[tid] != IcacheWaitResponse &&
        fetchStatus[tid] != ItlbWait &&
        fetchStatus[tid] != IcacheWaitRetry &&
        fetchStatus[tid] != QuiescePending &&
        !curMacroop && !decoder[tid]->isSpeculativeCacheActive() && !decoder[tid]->isUopCacheActive();
    //*****CHANGE END**********
}

template<class Impl>
void
DefaultFetch<Impl>::recvReqRetry()
{
    if (retryPkt != NULL) {
        assert(cacheBlocked);
        assert(retryTid != InvalidThreadID);
        assert(fetchStatus[retryTid] == IcacheWaitRetry);

        if (cpu->getInstPort().sendTimingReq(retryPkt)) {
            fetchStatus[retryTid] = IcacheWaitResponse;
            retryPkt = NULL;
            retryTid = InvalidThreadID;
            cacheBlocked = false;
        }
    } else {
        assert(retryTid == InvalidThreadID);
        // Access has been squashed since it was sent out.  Just clear
        // the cache being blocked.
        cacheBlocked = false;
    }
}

///////////////////////////////////////
//                                   //
//  SMT FETCH POLICY MAINTAINED HERE //
//                                   //
///////////////////////////////////////
template<class Impl>
ThreadID
DefaultFetch<Impl>::getFetchingThread(FetchPriority &fetch_priority)
{
    if (numThreads > 1) {
        switch (fetch_priority) {

          case SingleThread:
            return 0;

          case RoundRobin:
            return roundRobin();

          case IQ:
            return iqCount();

          case LSQ:
            return lsqCount();

          case Branch:
            return branchCount();

          default:
            return InvalidThreadID;
        }
    } else {
        list<ThreadID>::iterator thread = activeThreads->begin();
        if (thread == activeThreads->end()) {
            return InvalidThreadID;
        }

        ThreadID tid = *thread;

        if (fetchStatus[tid] == Running ||
            fetchStatus[tid] == IcacheAccessComplete ||
            fetchStatus[tid] == Idle) {
            return tid;
        } else {
            return InvalidThreadID;
        }
    }
}


template<class Impl>
ThreadID
DefaultFetch<Impl>::roundRobin()
{
    list<ThreadID>::iterator pri_iter = priorityList.begin();
    list<ThreadID>::iterator end      = priorityList.end();

    ThreadID high_pri;

    while (pri_iter != end) {
        high_pri = *pri_iter;

        assert(high_pri <= numThreads);

        if (fetchStatus[high_pri] == Running ||
            fetchStatus[high_pri] == IcacheAccessComplete ||
            fetchStatus[high_pri] == Idle) {

            priorityList.erase(pri_iter);
            priorityList.push_back(high_pri);

            return high_pri;
        }

        pri_iter++;
    }

    return InvalidThreadID;
}

template<class Impl>
ThreadID
DefaultFetch<Impl>::iqCount()
{
    //sorted from lowest->highest
    std::priority_queue<unsigned,vector<unsigned>,
                        std::greater<unsigned> > PQ;
    std::map<unsigned, ThreadID> threadMap;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;
        unsigned iqCount = fromIEW->iewInfo[tid].iqCount;

        //we can potentially get tid collisions if two threads
        //have the same iqCount, but this should be rare.
        PQ.push(iqCount);
        threadMap[iqCount] = tid;
    }

    while (!PQ.empty()) {
        ThreadID high_pri = threadMap[PQ.top()];

        if (fetchStatus[high_pri] == Running ||
            fetchStatus[high_pri] == IcacheAccessComplete ||
            fetchStatus[high_pri] == Idle)
            return high_pri;
        else
            PQ.pop();

    }

    return InvalidThreadID;
}

template<class Impl>
ThreadID
DefaultFetch<Impl>::lsqCount()
{
    //sorted from lowest->highest
    std::priority_queue<unsigned,vector<unsigned>,
                        std::greater<unsigned> > PQ;
    std::map<unsigned, ThreadID> threadMap;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;
        unsigned ldstqCount = fromIEW->iewInfo[tid].ldstqCount;

        //we can potentially get tid collisions if two threads
        //have the same iqCount, but this should be rare.
        PQ.push(ldstqCount);
        threadMap[ldstqCount] = tid;
    }

    while (!PQ.empty()) {
        ThreadID high_pri = threadMap[PQ.top()];

        if (fetchStatus[high_pri] == Running ||
            fetchStatus[high_pri] == IcacheAccessComplete ||
            fetchStatus[high_pri] == Idle)
            return high_pri;
        else
            PQ.pop();
    }

    return InvalidThreadID;
}

template<class Impl>
ThreadID
DefaultFetch<Impl>::branchCount()
{
#if 0
    list<ThreadID>::iterator thread = activeThreads->begin();
    assert(thread != activeThreads->end());
    ThreadID tid = *thread;
#endif

    panic("Branch Count Fetch policy unimplemented\n");
    return InvalidThreadID;
}

template<class Impl>
void
DefaultFetch<Impl>::pipelineIcacheAccesses(ThreadID tid)
{
    if (!issuePipelinedIfetch[tid]) {
        return;
    }

    // The next PC to access.
    TheISA::PCState thisPC = pc[tid];

    if (isRomMicroPC(thisPC.microPC())) {
        return;
    }

    if (isUopCachePresent && (!fetchBufferValid[tid] || !macroop[tid]) &&
            decoder[tid]->isHitInUopCache(thisPC.instAddr())) {
        return;
    }

    Addr pcOffset = fetchOffset[tid];
    Addr fetchAddr = (thisPC.instAddr() + pcOffset) & BaseCPU::PCMask;

    // Align the fetch PC so its at the start of a fetch buffer segment.
    Addr fetchBufferBlockPC = fetchBufferAlignPC(fetchAddr);

    // Unless buffer already got the block, fetch it from icache.
    if (!(fetchBufferValid[tid] && fetchBufferBlockPC == fetchBufferPC[tid])) {
        DPRINTF(Fetch, "[tid:%i]: Issuing a pipelined I-cache access, "
                "starting at PC %s.\n", tid, thisPC);

        fetchCacheLine(fetchAddr, tid, thisPC.instAddr());
    }
}

template<class Impl>
void
DefaultFetch<Impl>::profileStall(ThreadID tid) {
    DPRINTF(Fetch,"There are no more threads available to fetch from.\n");

    // @todo Per-thread stats

    if (stalls[tid].drain) {
        ++fetchPendingDrainCycles;
        DPRINTF(Fetch, "Fetch is waiting for a drain!\n");
    } else if (activeThreads->empty()) {
        ++fetchNoActiveThreadStallCycles;
        DPRINTF(Fetch, "Fetch has no active thread!\n");
    } else if (fetchStatus[tid] == Blocked) {
        ++fetchBlockedCycles;
        DPRINTF(Fetch, "[tid:%i]: Fetch is blocked!\n", tid);
    } else if (fetchStatus[tid] == Squashing) {
        ++fetchSquashCycles;
        DPRINTF(Fetch, "[tid:%i]: Fetch is squashing!\n", tid);
    } else if (fetchStatus[tid] == IcacheWaitResponse) {
        ++icacheStallCycles;
        DPRINTF(Fetch, "[tid:%i]: Fetch is waiting cache response!\n",
                tid);
    } else if (fetchStatus[tid] == ItlbWait) {
        ++fetchTlbCycles;
        DPRINTF(Fetch, "[tid:%i]: Fetch is waiting ITLB walk to "
                "finish!\n", tid);
    } else if (fetchStatus[tid] == TrapPending) {
        ++fetchPendingTrapStallCycles;
        DPRINTF(Fetch, "[tid:%i]: Fetch is waiting for a pending trap!\n",
                tid);
    } else if (fetchStatus[tid] == QuiescePending) {
        ++fetchPendingQuiesceStallCycles;
        DPRINTF(Fetch, "[tid:%i]: Fetch is waiting for a pending quiesce "
                "instruction!\n", tid);
    } else if (fetchStatus[tid] == IcacheWaitRetry) {
        ++fetchIcacheWaitRetryStallCycles;
        DPRINTF(Fetch, "[tid:%i]: Fetch is waiting for an I-cache retry!\n",
                tid);
    } else if (fetchStatus[tid] == NoGoodAddr) {
            DPRINTF(Fetch, "[tid:%i]: Fetch predicted non-executable address\n",
                    tid);
    } else {
        DPRINTF(Fetch, "[tid:%i]: Unexpected fetch stall reason (Status: %i).\n",
             tid, fetchStatus[tid]);
    }
}

#endif//__CPU_O3_FETCH_IMPL_HH__
