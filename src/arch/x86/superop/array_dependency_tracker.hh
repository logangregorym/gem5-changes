#ifndef __ARCH_X86_SUPEROP_ARRAY_DEPENDENCY_TRACKER_HH__
#define __ARCH_X86_SUPEROP_ARRAY_DEPENDENCY_TRACKER_HH__

#include <vector>

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
#include "params/ArrayDependencyTracker.hh"
#include "sim/sim_object.hh"
#include <set>

using namespace std;

class ISA;

class ArrayDependencyTracker : public SimObject
{
  public:
	// Constructor
	ArrayDependencyTracker(ArrayDependencyTrackerParams *p);

	X86ISA::Decoder* decoder;

	bool usingControlTracking = false;

	unsigned connectionCount = 4096;

	void addToGraph(StaticInstPtr uop, Addr addr, unsigned uopAddr, unsigned cycleAdded);

	void removeFromGraph(Addr addr, unsigned uopAddr, unsigned cycledRemoved);

	void removeAtIndex(int i1, int i2, int i3);

	void predictValue(Addr addr, unsigned uopAddr, uint64_t value);

	bool simplifyGraph();
	unsigned simplifyIdx = 0;
	unsigned simplifyWay = 0;
	unsigned simplifyUop = 0;

	void updateSpecTrace(int i1, int i2, int i3);

	void moveTraceInstOneForward(int i1, int i2, int i3);

	void invalidateTraceInst(int i1, int i2, int i3);

	void invalidateConnection(unsigned connectionIndex);

	void invalidateBranch(unsigned branchIndex);

	void markLastUse(unsigned regIdx);

	struct FullUopAddr {
		Addr pcAddr;
		unsigned uopAddr;

		FullUopAddr(Addr p, unsigned u) {
			pcAddr = p;
			uopAddr = u;
		}

		FullUopAddr() {
			pcAddr = 0;
			uopAddr = 0;
		}

		bool operator==(const FullUopAddr& rhs) {
			return (pcAddr == rhs.pcAddr) && (uopAddr == rhs.uopAddr);
		}

		bool operator!=(const FullUopAddr& rhs) {
			return (pcAddr != rhs.pcAddr) || (uopAddr != rhs.uopAddr);
		}
	};

	struct FullCacheIdx {
		int idx;
		int way;
		int uop;
		bool valid;

		FullCacheIdx(int i, int w, int u) {
			idx = i;
			way = w;
			uop = u;
			valid = true;
		}

		FullCacheIdx() {
			idx = 0;
			way = 0;
			uop = 0;
			valid = false;
		}
	};

	FullCacheIdx getNextCacheIdx(FullCacheIdx);
	FullCacheIdx getPrevCacheIdx(FullCacheIdx);

	unsigned registerRenameMapSpec[256] = {0};
	FullUopAddr registerProducerMapSpec[256];
	FullUopAddr mostRecentConsumer[256];
	bool consumedInWindow[256] = {0};
	bool registerValidMapSpec[256] = {0};
	unsigned nextRegNameSpec = 1;

	struct InformationFlowPath {
		FullUopAddr producer = FullUopAddr(0, 0);
		FullUopAddr consumer = FullUopAddr(0, 0);
		unsigned archRegIdx;
		unsigned renamedRegIdx;
		uint64_t value;
		bool valid;
		bool lastUse;
		unsigned directControlDependency = 0;
		unsigned indirectControlDependency = 0;
		unsigned dataDependencies[8] = {0};

		InformationFlowPath() {
			producer = FullUopAddr(0,0);
			consumer = FullUopAddr(0,0);
			archRegIdx = 0;
			renamedRegIdx = 0;
			value = 0;
			valid = false;
			lastUse = false;
			directControlDependency = 0;
			indirectControlDependency = 0;
		}

		InformationFlowPath(FullUopAddr p, FullUopAddr c, unsigned a, unsigned r) {
			producer = p;
			consumer = c;
			archRegIdx = a;
			renamedRegIdx = r;
			value = 0;
			valid = false;
			lastUse = false;
			directControlDependency = 0;
			indirectControlDependency = 0;
		}

		InformationFlowPath(FullUopAddr p, FullUopAddr c, unsigned a, unsigned r, uint64_t v, bool b, bool d) {
			producer = p;
			consumer = c;
			archRegIdx = a;
			renamedRegIdx = r;
			value = v;
			valid = b;
			lastUse = false;
			directControlDependency = 0;
			indirectControlDependency = 0;
		}

		void predict(uint64_t v, unsigned predID) {
			value = v;
			valid = true;
			bool found = false;
			for (int i = 0; i < 8; i++) {
				if (!found && (dataDependencies[i] == 0)) {
					dataDependencies[i] = predID;
					found = true;
				}
			}
		}

		void addDependency(unsigned id) {
			bool found = false;
			for (int i = 0; i<8; i++) {
				if (!found && (dataDependencies[i] == 0)) {
					dataDependencies[i] = id;
					found = true;
				}
			}
		}
	};

	struct ControlFlowPath {
		FullUopAddr branchAddr = FullUopAddr(0,0);
		FullUopAddr nextPc = FullUopAddr(0,0);
		bool targetValid = false;
		FullUopAddr propagatingTo = FullUopAddr(0,0);
		unsigned registerRenameMap[256];
		FullUopAddr registerProducerMap[256];
		bool registerValidMap[256];

		ControlFlowPath() {
			branchAddr = FullUopAddr(0,0);
			nextPc = FullUopAddr(0,0);
			for (int i=0; i<256; i++) {
				registerRenameMap[i] = 0;
				registerProducerMap[i] = FullUopAddr(0,0);
				registerValidMap[i] = false;
			}
		}

		ControlFlowPath(FullUopAddr bf, FullUopAddr bt) {
			branchAddr = bf;
			nextPc = bt;
			for (int i=0; i<256; i++) {
				registerRenameMap[i] = 0;
				registerProducerMap[i] = FullUopAddr(0,0);
				registerValidMap[i] = false;
			}
		}
	};

	struct DependGraphEntry {
		FullUopAddr thisInst = FullUopAddr(0, 0);
		unsigned producers[256] = {0};
		unsigned consumers[256] = {0};
		bool seen = false;
		unsigned cycleAdded = 0;
		bool predicted = false;
		uint64_t value = 0;
		bool deadCode = false;
		FullCacheIdx specIdx = FullCacheIdx();

		DependGraphEntry() {
			thisInst = FullUopAddr(0,0);
			for (int i=0; i<256; i++) {
				producers[i] = 0;
				consumers[i] = 0;
			}
			cycleAdded = 0;
			seen = false;
			value = 0;
			predicted = false;
		}

		DependGraphEntry(FullUopAddr a) {
			thisInst = a;
			for (int i=0; i<256; i++) {
				producers[i] = 0;
				consumers[i] = 0;
			}
			cycleAdded = 0;
			seen = false;
			value = 0;
			predicted = false;
		}
	};

	DependGraphEntry* speculativeDependencyGraph[32][8][6];
	FullUopAddr microopAddrArray[32][8][6];

	InformationFlowPath connections[8192]; // Must be kept identical to connectionCount
	bool connectionsValidSpec[8192] = {0}; // Must be kept identical to connectionCount
	ControlFlowPath branches[4096];
	bool branchesValid[4096] = {0};
	unsigned maxRecursiveDepth = 8;
	Addr predictionSource[4096] = {0};
	bool predictionSourceValid[4096] = {0};

	// Exploration and stats
	void measureChain(Addr addr, unsigned uopAddr);
	void measureChain(FullUopAddr addr, unsigned recursionLevel); // , vector<FullUopAddr>& checked);
	bool isReducable(Addr addr, unsigned uopAddr);
	void describeEntry(int idx, int way, int uop);
	void describeFullGraph();

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
	bool propagateWrip(int idx, int way, int uop);
	bool propagateAcrossControlDependency(unsigned branchIndex, FullUopAddr propagatingTo);

	Stats::Scalar numChainsMeasured;
	Stats::Scalar totalDependentInsts;
	Stats::Scalar reducableInstCount;
	Stats::Scalar totalOpsInCache;
	Stats::Scalar totalReducable;
	Stats::Formula averageDependentInsts;
	Stats::Formula averageNumberReducable;
	Stats::Scalar branchesOnChains;
	Stats::Scalar confidentBranchesOnChains;
	Stats::Formula percentChainBranchesConfident;
	Stats::Scalar totalCyclesInUopCache;
	Stats::Scalar evictionsFromUopCache;
	Stats::Scalar totalCyclesInSpecCache;
	Stats::Scalar evictionsFromSpecCache;
	Stats::Formula averageCyclesInUopCache;
	Stats::Formula averageCyclesInSpecCache;
	void regStats();

}; // class ArrayDependencyTracker

#endif // __ARCH_X86_SUPEROP_ARRAY_DEPENDENCY_TRACKER_HH__
