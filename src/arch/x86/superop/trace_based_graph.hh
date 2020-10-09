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
#include "cpu/pred/ltage.hh"
#include "debug/SuperOp.hh"
#include "params/TraceBasedGraph.hh"
#include "sim/sim_object.hh"
#include <queue>

using namespace std;

class ISA;

// tracks instructions with confident value predictions
struct PredictionSource
{
    FullUopAddr addr;
    bool valid;
    bool isBranch;
    int64_t value;
    unsigned confidence;
    unsigned latency;

    PredictionSource() {valid = false; isBranch = false;}
};

// Register context block for liveness analysis
struct RegisterContext {
    int64_t value;
    bool valid;
    bool source;

    RegisterContext() {value = 0; valid = false; source = false;}
};

// Speculative Trace
struct SpecTrace
{
    // Trace ID
    unsigned id;

    // (idx, way, uop) of head of the trace
    FullCacheIdx head;

    // (idx, way, uop) of head of the optimized trace
    FullCacheIdx optimizedHead;

    // (idx, way, uop) of the instruction being optimized
    FullCacheIdx addr;

    // address of the instruction being optimized
    FullUopAddr instAddr;

    // instruction being optimized
    StaticInstPtr inst;

    // address of the last instruction in the trace
    FullUopAddr end;

    enum State {
        Invalid,
        
        QueuedForFirstTimeOptimization,
        QueuedForReoptimization,

        // first time optimization
        OptimizationInProcess,

        // re-optimization (e.g., due to a new pred source)
        ReoptimizationInProcess,

        // evicted before we could process it
        Evicted,

        Complete
    };

    // Trace Satte
    State state;

    // Prediction Sources (at most 4)
    PredictionSource source[4];

    // Trace Length
    unsigned length;

    // Shrunk length
    unsigned shrunkLength;

    // ID of the trace being re-optimized in case this is a re-optimization
    unsigned reoptId;

    // Counter to assign trace IDs
    static unsigned traceIDCounter;

    SpecTrace() {
        state = Invalid;
        length = 0;
        shrunkLength = 0;
        head = FullCacheIdx();
        addr = FullCacheIdx();
        inst = NULL;
    }
};

class TraceBasedGraph : public SimObject
{
    public:
    // Constructor
    TraceBasedGraph(TraceBasedGraphParams *p);

    X86ISA::Decoder* decoder;

    bool usingControlTracking = false;

    void predictValue(Addr addr, unsigned uopAddr, int64_t value, unsigned confidence, unsigned latency);

    bool updateSpecTrace(SpecTrace &trace);

    bool isPredictionSource(SpecTrace trace, FullUopAddr addr, int64_t &value, unsigned &confidence, unsigned &latency);

    bool generateNextTraceInst();

    // Trace ID to map
    map<unsigned, SpecTrace> traceMap;

    // Outstanding trace requests
    queue<SpecTrace> traceQueue;

    // Current trace being optimized
    SpecTrace currentTrace;

    // Current trace being streamed
    SpecTrace streamTrace;

    // Register context block of the trace being optimized
    RegisterContext regCtx[38]; // 38 integer registers including implicit ones

    BPredUnit* branchPred;

    // Propagation Functions
    bool propagateLastUse(StaticInstPtr inst);
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
    
    // dump trace for debugging
    void dumpTrace(SpecTrace trace);

    // advance to next micro-op.
    bool advanceIfControlTransfer(SpecTrace &trace);
    void advanceTrace(SpecTrace &trace);
    unsigned computeLength(SpecTrace trace);

    void regStats();

    Stats::Scalar tracesPoppedFromQueue;
    Stats::Scalar tracesWithInvalidHead;

}; // class TraceBasedGraph

#endif // __ARCH_X86_TRACE_BASED_GRAPH_HH__
