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
  uint64_t value;

  PredictionSource() {valid = false; isBranch = false;}
};

// Speculative Trace
struct SpecTrace
{
  // Trace ID
  unsigned id;

  // (idx, way, uop) of head of the trace
  FullCacheIdx head;

  // (idx, way, uop) of the instruction being optimized
  FullCacheIdx addr;

  // address of the instruction being optimized
  FullUopAddr instAddr;

  // instruction being optimized
  StaticInstPtr inst;

  enum State {
    Invalid,
    
    QueuedForFirstTimeOptimization,
    QueuedForReoptimization,

    // first time optimization
    OptimizationInProcess,

    // re-optimization (e.g., due to a new pred source)
    ReoptimizationInProcess,

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

	void predictValue(Addr addr, unsigned uopAddr, int64_t value);

	bool updateSpecTrace(SpecTrace &trace);

	// void moveTraceInstOneForward(int i1, int i2, int i3);

	// void invalidateTraceInst(int i1, int i2, int i3);

	void invalidateBranch(Addr addr);

	bool isPredictionSource(SpecTrace trace, FullUopAddr addr);

	void flushMisprediction(Addr addr, unsigned uop);

	// void flushMisprediction(unsigned predId);

	// void registerRemovalOfTraceInst(int i1, int i2, int i3);

	FullCacheIdx getNextCacheIdx(FullCacheIdx);
	FullCacheIdx getPrevCacheIdx(FullCacheIdx);

	bool generateNextTraceInst();

	void incrementPC(FullCacheIdx specIdx, X86ISA::PCState &nextPC, bool &predict_taken);
	bool isTakenBranch(FullUopAddr addr, FullCacheIdx specIdx);

	uint64_t registerValue[256] = {0};
	bool registerValid[256] = {0};

  // Trace ID to map
  map<unsigned, SpecTrace> traceMap;

  // Outstanding trace requests
	queue<SpecTrace> traceQueue;

  // Current trace being optimized
  SpecTrace currentTrace;

  // Current trace being streamed
  SpecTrace streamTrace;

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
	// bool propagateWrip(StaticInstPtr inst);
	// bool propagateAcrossControlDependency(unsigned branchIndex, FullUopAddr propagatingTo);
	
  // dump trace for debugging
  void dumpTrace(SpecTrace trace);

  // advance to next micro-op.
  void advanceTrace(SpecTrace &trace);

	void regStats();

	Stats::Scalar tracesPoppedFromQueue;
	Stats::Scalar tracesWithInvalidHead;

}; // class TraceBasedGraph

#endif // __ARCH_X86_TRACE_BASED_GRAPH_HH__
