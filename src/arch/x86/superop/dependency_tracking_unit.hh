#ifndef __ARCH_X86_SUPEROP_DEPENDENCY_TRACKING_UNIT_HH__
#define __ARCH_X86_SUPEROP_DEPENDENCY_TRACKING_UNIT_HH__

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
#include "params/DependencyTrackingUnit.hh"
#include "sim/sim_object.hh"

using namespace std;

// namespace X86ISA
// {

class ISA;

// Expects to recieve new instructions in program order
class DependencyTrackingUnit : public SimObject
{
  public:
        // Constructor
        DependencyTrackingUnit(DependencyTrackingUnitParams *p);

        // Decoder it belongs to
        X86ISA::Decoder* decoder;

         // When an instruction gets added to the micro-op cache, call this to add it to the graph
    void addToGraph(StaticInstPtr uop, Addr addr, unsigned uopAddr);

        // When an instruction is evicted from the micro-op cache, call this to remove it from the graph
    void removeFromGraph(StaticInstPtr uop, Addr addr, unsigned uopAddr);

        // Record a predicted value and propagate along the graph
        void predictValue(Addr addr, unsigned uopAddr, uint64_t value);

        // Make a single pass, reducing instructions if possible
        // Return true iff a reduction was performed
        bool simplifyGraph();

        // Combines PC address and micro-op address
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
        // For each architectural int register, what is its current renaming at the end of the window?
    unsigned registerRenameMap[256] = {0};

        // For each architectural int register, what is its current producer at the end of the window?
    FullUopAddr registerProducerMap[256];

        // To fully specify each producer, must include a micro-op number
//	unsigned registerUopMap[256] = {0};

        // For each architectural int register, is the current producer valid?
        bool registerValidMap[256] = {0};

        unsigned nextRegName = 0;

  public:
        // Tracks information transfer between instructions
        struct InformationFlowPath
        {
                FullUopAddr producer = FullUopAddr(0, 0);
                // unsigned producerOp;
                FullUopAddr consumer = FullUopAddr(0, 0);
                // unsigned consumerOp;
                unsigned archRegIdx;
                unsigned renamedRegIdx;
                uint64_t value; // If this has been predicted or forwarded
                bool valid; // Tells whether "value" is usable after consumption
                bool lastUse; // Tells whether "value" is dead after consumption

                InformationFlowPath(FullUopAddr p, FullUopAddr c, unsigned a, unsigned r) {
                        producer = p;
                        // producerOp = pop;
                        consumer = c;
                        // consumerOp = cop;
                        archRegIdx = a;
                        renamedRegIdx = r;
                        value = 0;
                        valid = false;
                        lastUse = false;
                }

                InformationFlowPath(FullUopAddr p, FullUopAddr c, unsigned a, unsigned r, uint64_t v, bool b) {
                        producer = p;
                        // producerOp = pop;
                        consumer = c;
                        // consumerOp = cop;
                        archRegIdx = a;
                        renamedRegIdx = r;
                        value = v;
                        valid = b;
                        lastUse = false;
                }
        };

        // Stores producer and consumer paths for an instruction
        struct DependGraphEntry
        {
                FullUopAddr thisInst = FullUopAddr(0, 0);
                // unsigned thisOp;
                std::vector<InformationFlowPath> producers;
                std::vector<InformationFlowPath> consumers;

                DependGraphEntry(FullUopAddr a) {
                        thisInst = a;
                        // thisOp = op;
                        producers = std::vector<InformationFlowPath>();
                        consumers = std::vector<InformationFlowPath>();
                }

                DependGraphEntry(FullUopAddr a, std::vector<InformationFlowPath>& p, std::vector<InformationFlowPath>& q) {
                        thisInst = a;
                        // thisOp = op;
                        producers = p;
                        consumers = q;
                }
        };

  private:
        // The graph itself, one version each for microop and speculative caches
        std::vector<DependGraphEntry> microopDependencyGraph;
        std::vector<DependGraphEntry> speculativeDependencyGraph;

        // Measures dependency chains
        unsigned maxRecursiveDepth = 64;
        void measureChain(DependGraphEntry& start);
        void measureChain(DependGraphEntry& start, unsigned recursionLevel, vector<FullUopAddr> checked);

  public:
        // Return an iterator to the entry for the provided address
        vector<DependGraphEntry>::iterator findInSpeculativeGraph(Addr addr, unsigned uopAddr);
        vector<DependGraphEntry>::iterator findInSpeculativeGraph(FullUopAddr fullAddr);
        vector<DependGraphEntry>::iterator findInOriginalGraph(Addr addr, unsigned uopAddr);
        vector<DependGraphEntry>::iterator findInOriginalGraph(FullUopAddr fullAddr);

        Stats::Scalar numChainsMeasured;
        Stats::Scalar totalDependentInsts;
        Stats::Scalar reducableInstCount;
        Stats::Formula averageDependentInsts;
        Stats::Formula averageNumberReducable;
        void regStats();

}; // class DependencyTrackingUnit

// } // namespace X86ISA


#endif // __ARCH_X86_SUPEROP_DEPENDENCY_TRACKING_UNIT_HH__
