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

class TraceBasedGraph : public SimObject
{
  public:
	// Constructor
	TraceBasedGraph(TraceBasedGraphParams *p);

	X86ISA::Decoder* decoder;

	bool usingControlTracking = false;

	void predictValue(Addr addr, unsigned uopAddr, int64_t value);

	void updateSpecTrace(int i1, int i2, int i3, unsigned traceID);

	// void moveTraceInstOneForward(int i1, int i2, int i3);

	// void invalidateTraceInst(int i1, int i2, int i3);

	void invalidateBranch(Addr addr);

	bool isPredictionSource(Addr addr, unsigned uop, unsigned traceID);

	void flushMisprediction(Addr addr, unsigned uop);

	// void flushMisprediction(unsigned predId);

	// void registerRemovalOfTraceInst(int i1, int i2, int i3);

	FullCacheIdx getNextCacheIdx(FullCacheIdx);
	FullCacheIdx getPrevCacheIdx(FullCacheIdx);

	bool generateNextTraceInst();
	FullCacheIdx simplifyIdx = FullCacheIdx();

	void incrementPC(FullCacheIdx specIdx, X86ISA::PCState &nextPC, bool &predict_taken);
	bool isTakenBranch(FullUopAddr addr, FullCacheIdx specIdx);

	uint64_t registerValue[256] = {0};
	bool registerValid[256] = {0};

	FullUopAddr predictionSource[4096];
	uint64_t predictedValue[4096] = {0};
	bool predictionSourceValid[4096] = {0};
	bool sourceIsBranch[4096] = {0};

	queue<unsigned> traceQueue;
	unsigned currentTrace = 0;
	FullCacheIdx traceHead[4096]; // tweak this number?
	bool traceComplete[4096] = {0};
	unsigned traceSources[4096][4] = {{0}}; // prediction sources to use

	BPredUnit* branchPred;

	// Propagation Functions
	bool propagateLastUse(int idx, int way, int uop);
	bool propagateMov(int idx, int way, int uop);
	bool propagateMovI(int idx, int way, int uop);
	bool propagateLimm(int idx, int way, int uop);
	bool propagateAnd(int idx, int way, int uop);
	bool propagateAndI(int idx, int way, int uop);
	bool propagateAdd(int idx, int way, int uop);
	bool propagateAddI(int idx, int way, int uop);
	bool propagateSub(int idx, int way, int uop);
	bool propagateSubI(int idx, int way, int uop);
	bool propagateOr(int idx, int way, int uop);
	bool propagateOrI(int idx, int way, int uop);
	bool propagateXor(int idx, int way, int uop);
	bool propagateXorI(int idx, int way, int uop);
	bool propagateSllI(int idx, int way, int uop);
	bool propagateSrlI(int idx, int way, int uop);
	bool propagateSExtI(int idx, int way, int uop);
	bool propagateZExtI(int idx, int way, int uop);
	// bool propagateWrip(int idx, int way, int uop);
	// bool propagateAcrossControlDependency(unsigned branchIndex, FullUopAddr propagatingTo);

	void regStats();

}; // class TraceBasedGraph

#endif // __ARCH_X86_TRACE_BASED_GRAPH_HH__
