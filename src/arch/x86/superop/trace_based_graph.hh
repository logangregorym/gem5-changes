#ifndef __ARCH_X86_SUPEROP_TRACE_BASED_GRAPH_HH__
#define __ARCH_X86_SUPEROP_TRACE_BASED_GRAPH_HH__

#include <vector>
#include "arch/x86/decoder.hh"
#include "arch/x86/decoder_structs.hh"
#include "arch/x86/regs/misc.hh"
#include "arch/x86/types.hh"
#include "base/bitfield.hh"
#include "base/logging.hh"
#include "base/statistics.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "cpu/static_inst.hh"
#include "cpu/pred/bpred_unit.hh"
#include "cpu/pred/lvpred_unit.hh"
#include "cpu/pred/ltage.hh"
#include "debug/SuperOp.hh"
#include "params/TraceBasedGraph.hh"
#include "sim/sim_object.hh"
#include <queue>
#include <gperftools/heap-profiler.h>
#include "cpu/pred/big_sat_counter.hh"

using namespace std;

class ISA;


class LVPredUnit;

class TraceBasedGraph : public SimObject
{
    public:
    // Constructor
    TraceBasedGraph(TraceBasedGraphParams *p);

    X86ISA::Decoder* decoder;


    bool usingControlTracking = false;
    bool usingCCTracking = false;

    uint16_t predictionConfidenceThreshold = 5; 
    uint16_t specCacheNumWays = 8;
    uint16_t specCacheNumSets = 32;
    uint16_t specCacheNumUops = 6;
    uint16_t specCacheNumTicks = 10;
    uint16_t numOfTracePredictionSources = 4;
    uint64_t debugTraceGen = 0;

    bool QueueHotTraceForSuperOptimization(const X86ISA::PCState& pc);

    bool updateSpecTrace(SpecTrace &trace, bool& isDeadCode, bool propagated);

    bool isPredictionSource(SpecTrace& trace, FullUopAddr addr, uint64_t &value, unsigned &confidence, unsigned &latency);

    bool generateNextTraceInst();

    bool IsValuePredictible(const StaticInstPtr inst);

    bool isPredictiableCC(StaticInstPtr inst);
    void updateCCFlagsForPredictedSource(StaticInstPtr inst);
    bool probePredictorForMakingPrediction();


    // Trace ID to map
    map<uint64_t, SpecTrace> traceMap;

    // Outstanding trace requests
    queue<SpecTrace> traceQueue;

    // Current trace being optimized
    SpecTrace currentTrace;

    // Current trace being streamed
    SpecTrace streamTrace;

    // Register context block of the trace being optimized
    RegisterContext regCtx[38]; // 38 integer registers including implicit ones

    // Condition codes
    uint64_t PredccFlagBits;
    uint64_t PredcfofBits;
    uint64_t PreddfBit;
    uint64_t PredecfBit;
    uint64_t PredezfBit;
    uint64_t oldCCRegs[5];
    OpClass ccRegFrom = No_OpClass;
    bool ccValid;
    bool oldCCValid = false;

    BPredUnit* branchPred;
    LVPredUnit *loadPred;

    // Dump live outs
    void dumpLiveOuts(StaticInstPtr inst, bool dumpOnlyArchRegs);

    // Propagation Functions
    bool propagateLea(StaticInstPtr inst);
    bool propagateMov(StaticInstPtr inst);
    bool propagateMovI(StaticInstPtr inst);
    bool propagateLimm(StaticInstPtr inst);
    bool propagateAnd(StaticInstPtr inst);
    bool propagateAndI(StaticInstPtr inst);
    bool propagateAdd(StaticInstPtr inst);
    bool propagateAddI(StaticInstPtr inst);
    bool propagateSub(StaticInstPtr inst);
    bool propagateSubI(StaticInstPtr inst);
    bool propagateOr(StaticInstPtr inst);
    bool propagateOrI(StaticInstPtr inst);
    bool propagateXor(StaticInstPtr inst);
    bool propagateXorI(StaticInstPtr inst);
    bool propagateSllI(StaticInstPtr inst);
    bool propagateSrlI(StaticInstPtr inst);
    bool propagateSExtI(StaticInstPtr inst);
    bool propagateZExtI(StaticInstPtr inst);
    bool propagateWrip(StaticInstPtr inst);
    bool propagateWripI(StaticInstPtr inst);
    
    // dump trace for debugging
    void dumpTrace(SpecTrace trace);

    // advance to next micro-op.
    bool advanceIfControlTransfer(SpecTrace &trace, Addr &target);
    Addr advanceTrace(SpecTrace &trace);
    unsigned computeLength(SpecTrace trace);

    void regStats();

    Stats::Scalar tracesPoppedFromQueue;
    Stats::Scalar tracesWithInvalidHead;

    /** Distribution of the number of microops in superoptimized trace */
    Stats::Distribution numMicroopsInTraceDist;

}; // class TraceBasedGraph


#endif // __ARCH_X86_TRACE_BASED_GRAPH_HH__
