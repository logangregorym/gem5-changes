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
#include "debug/SuperOp.hh"
#include "params/ArrayDependencyTracker.hh"
#include "sim/sim_object.hh"

using namespace std;

class ISA;

class ArrayDependencyTracker : public SimObject
{
  public:
        // Constructor
        ArrayDependencyTracker(ArrayDependencyTrackerParams *p);

        X86ISA::Decoder* decoder;

        void addToGraph(StaticInstPtr uop, Addr addr, unsigned uopAddr);

        void removeFromGraph(Addr addr, unsigned uopAddr);

        void predictValue(Addr addr, unsigned uopAddr, uint64_t value);

        bool simplifyGraph();

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
                        return pcAddr == rhs.pcAddr && uopAddr == rhs.uopAddr;
                }
        };

  private:
        unsigned registerRenameMap[256] = {0};

        FullUopAddr registerProducerMap[256];

        bool registerValidMap[256] = {0};

        unsigned nextRegName = 0;

  public:
        struct InformationFlowPath
        {
                FullUopAddr producer = FullUopAddr(0, 0);
                FullUopAddr consumer = FullUopAddr(0, 0);
                unsigned archRegIdx;
                unsigned renamedRegIdx;
                uint64_t value;
                bool valid;
                bool lastUse;

                InformationFlowPath() {
                        producer = FullUopAddr(0,0);
                        consumer = FullUopAddr(0,0);
                        archRegIdx = 0;
                        renamedRegIdx = 0;
                        value = 0;
                        valid = false;
                        lastUse = false;
                }

                InformationFlowPath(FullUopAddr p, FullUopAddr c, unsigned a, unsigned r) {
                        producer = p;
                        consumer = c;
                        archRegIdx = a;
                        renamedRegIdx = r;
                        value = 0;
                        valid = false;
                        lastUse = false;
                }

                InformationFlowPath(FullUopAddr p, FullUopAddr c, unsigned a, unsigned r, uint64_t v, bool b) {
                        producer = p;
                        consumer = c;
                        archRegIdx = a;
                        renamedRegIdx = r;
                        value = v;
                        valid = b;
                        lastUse = false;
                }
        };

        struct DependGraphEntry
        {
                FullUopAddr thisInst = FullUopAddr(0, 0);
                unsigned producers[256] = {0};
                unsigned consumers[256] = {0};
                bool seen = false;

                DependGraphEntry() {
                        thisInst = FullUopAddr(0,0);
                        for (int i=0; i<256; i++) {
                                producers[i] = 0;
                                consumers[i] = 0;
                        }
                        seen = false;
                }

                DependGraphEntry(FullUopAddr a) {
                        thisInst = a;
                        for (int i=0; i<256; i++) {
                                producers[i] = 0;
                                consumers[i] = 0;
                        }
                        seen = false;
                }
        };

  private:
        DependGraphEntry microopDependencyGraph[32][8][6];
        DependGraphEntry speculativeDependencyGraph[32][8][6];
        InformationFlowPath connections[4096];
        bool connectionsValid[4096] = {0};
        unsigned maxRecursiveDepth = 64;

  public:
        void measureChain(Addr addr, unsigned uopAddr);
        void measureChain(FullUopAddr addr, unsigned recursionLevel, vector<FullUopAddr>& checked);
        bool isReducable(Addr addr, unsigned uopAddr);

  public:
        Stats::Scalar numChainsMeasured;
        Stats::Scalar totalDependentInsts;
        Stats::Scalar reducableInstCount;
        Stats::Scalar totalOpsInCache;
        Stats::Scalar totalReducable;
        Stats::Formula averageDependentInsts;
        Stats::Formula averageNumberReducable;
        void regStats();

}; // class ArrayDependencyTracker

#endif // __ARCH_X86_SUPEROP_ARRAY_DEPENDENCY_TRACKER_HH__
