#include "arch/x86/decoder.hh"
#include "arch/x86/regs/misc.hh"
#include "arch/x86/superop/array_dependency_tracker.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/SuperOp.hh"

using namespace std;

ArrayDependencyTracker::ArrayDependencyTracker(ArrayDependencyTrackerParams *p) : SimObject(p) {}

void ArrayDependencyTracker::regStats()
{
        numChainsMeasured
                .name("system.switch_cpus.decode.superop.dependencyTracker.numChainsMeasured")
                .desc("Number of dependency chains measured")
                ;
        totalDependentInsts
                .name("system.switch_cpus.decode.superop.dependencyTracker.totalDependentInsts")
                .desc("Total count of instructions in dependency chains")
                ;
        reducableInstCount
                .name("system.switch_cpus.decode.superop.dependencyTracker.reducableInstCount")
                .desc("Number of instructions with all values ready")
                ;
        averageDependentInsts
                .name("system.switch_cpus.decode.superop.dependencyTracker.averageDependentInsts")
                .desc("Average number of instructions per dependency chain")
                ;
        averageDependentInsts = totalDependentInsts / numChainsMeasured;
        averageNumberReducable
                .name("system.switch_cpus.decode.superop.dependencyTracker.averageNumberReducable")
                .desc("Average number of instructions per chain with all values ready")
                ;
        averageNumberReducable = reducableInstCount / numChainsMeasured;
        totalReducable
                .name("system.switch_cpus.decode.superop.dependencyTracker.totalReducable")
                .desc("Count of reducable instructions with no repeats")
                ;
        totalOpsInCache
                .name("system.switch_cpus.decode.superop.dependencyTracker.totalOpsInCache")
                .desc("Count of instructions loaded into the cache")
                ;
}

void ArrayDependencyTracker::addToGraph(StaticInstPtr uop, Addr addr, unsigned uopAddr)
{
        ++totalOpsInCache;
        // Clear the slot
        removeFromGraph(addr, uopAddr);

        // Get the cache address and constrcut a fullAddr type
        ArrayDependencyTracker::FullUopAddr fullAddr = ArrayDependencyTracker::FullUopAddr(addr, uopAddr);
        int idx = (addr >> 5) & 0x1f;
        uint64_t tag = (addr >> 10);
        int uopway = 0;
        int specway = 0;
        for (int w = 0; w < 8; w++) {
                if (decoder->uopValidArray[idx][w] && decoder->uopTagArray[idx][w] == tag && decoder->uopAddrArray[idx][w][uopAddr] == addr) {
                        uopway = w;
                        break;
                }
        }
        for (int w = 0; w < 8; w++) {
                if (decoder->speculativeValidArray[idx][w] && decoder->speculativeTagArray[idx][w] == tag && decoder->speculativeAddrArray[idx][w][uopAddr] == addr) {
                        specway = w;
                        break;
                }
        }

        if (!(microopDependencyGraph[idx][uopway][uopAddr].thisInst == fullAddr) || !(speculativeDependencyGraph[idx][specway][uopAddr].thisInst == fullAddr)) {

        // Clear the slot
        removeFromGraph(addr, uopAddr);

        // Debugging
        // Are we sure this slot is empty?
        for (int i=0; i<256; i++) {
                assert(microopDependencyGraph[idx][uopway][uopAddr].producers[i] == 0);
                assert(microopDependencyGraph[idx][uopway][uopAddr].consumers[i] == 0);
                assert(speculativeDependencyGraph[idx][specway][uopAddr].producers[i] == 0);
                assert(speculativeDependencyGraph[idx][specway][uopAddr].consumers[i] == 0);
        }

        // Search for source registers in graph
        for (int i = 0; i < uop->numSrcRegs(); i++) {
                RegId srcReg = uop->srcRegIdx(i);
                if (srcReg.isIntReg()) {
                        if (registerValidMap[srcReg.flatIndex()]) {
                                // if written in window, create an information flow path to this inst
                                unsigned flowPathIdx = 1; // 0 reserved for invalid
                                bool found = false;
                                while (flowPathIdx < 4096 && !found) {
                                        if (!connectionsValid[flowPathIdx]) {
                                                found = true;
                                                break;
                                        } else {
                                                flowPathIdx++;
                                        }
                                }
                                if (found) {
                                        connections[flowPathIdx] = InformationFlowPath(registerProducerMap[srcReg.flatIndex()], fullAddr, srcReg.flatIndex(), registerRenameMap[srcReg.flatIndex()]);

                                        // Add the path to this inst's producers; first uop...
                                        unsigned prodIdx = 0;
                                        found = false; // reusing
                                        while (prodIdx < 256 && !found) {
                                                if (microopDependencyGraph[idx][uopway][uopAddr].producers[prodIdx] == 0) {
                                                        found = true;
                                                        microopDependencyGraph[idx][uopway][uopAddr].producers[prodIdx] = flowPathIdx;
                                                } else if (microopDependencyGraph[idx][uopway][uopAddr].producers[prodIdx] == flowPathIdx) {
                                                        found = true;
                                                } else {
                                                        prodIdx++;
                                                }
                                        }
                                        if (!found) {
                                                DPRINTF(SuperOp, "Not enough entries in producers for inst %s (%i)\n", uop->getName(), uop->numSrcRegs());
                                        }

                                        // ...now spec
                                        prodIdx = 0;
                                        found = false;
                                        while (prodIdx < 256 && !found) {
                                                if (speculativeDependencyGraph[idx][specway][uopAddr].producers[prodIdx] == 0) {
                                                        found = true;
                                                        speculativeDependencyGraph[idx][specway][uopAddr].producers[prodIdx] = flowPathIdx;
                                                } else if (speculativeDependencyGraph[idx][specway][uopAddr].producers[prodIdx] == flowPathIdx) {
                                                        found = true;
                                                } else {
                                                        prodIdx++;
                                                }
                                        }
                                        if (!found) {
                                                DPRINTF(SuperOp, "Not enough entries in producers for inst %s (%i)\n", uop->getName(), uop->numSrcRegs());
                                        }

                                        // Add the path to the producing inst's consumers
                                        ArrayDependencyTracker::FullUopAddr prod = registerProducerMap[srcReg.flatIndex()];
                                        prodIdx = (prod.pcAddr >> 5) & 0x1f;
                                        uint64_t prodTag = (prod.pcAddr >> 10);
                                        int prodUopWay = 0;
                                        int prodSpecWay = 0;
                                        for (int w = 0; w < 8; w++) {
                                                if (decoder->uopValidArray[prodIdx][w] && decoder->uopTagArray[prodIdx][w] == prodTag && decoder->uopAddrArray[prodIdx][w][uopAddr] == prod.pcAddr) {
                                                        prodUopWay = w;
                                                }
                                                if (decoder->speculativeValidArray[prodIdx][w] && decoder->speculativeTagArray[prodIdx][w] == prodTag && decoder->speculativeAddrArray[prodIdx][w][uopAddr] == prod.pcAddr) {
                                                        prodSpecWay = w;
                                                }
                                        }
                                        unsigned prodUop = prod.uopAddr;
                                        // first uop...
                                        unsigned conIdx = 0;
                                        found = false;
                                        while (conIdx < 256 && !found) {
                                                if (microopDependencyGraph[prodIdx][prodUopWay][prodUop].consumers[conIdx] == 0) {
                                                        found = true;
                                                        microopDependencyGraph[prodIdx][prodUopWay][prodUop].consumers[conIdx] = flowPathIdx;
                                                } else if (microopDependencyGraph[prodIdx][prodUopWay][prodUop].consumers[conIdx] == flowPathIdx) {
                                                        found = true;
                                                } else {
                                                        conIdx++;
                                                }
                                        }
                                        if (!found) {
                                                // Using spec uop bc StaticInst instead of EMI
                                                DPRINTF(SuperOp, "Not enough entries in consumers for inst %s (%i) microop\n", decoder->speculativeCache[prodIdx][prodUopWay][prodUop]->getName(), decoder->speculativeCache[prodIdx][prodUopWay][prodUop]->numDestRegs());
                                        }

                                        // ...now spec
                                        conIdx = 0;
                                        found = false;
                                        while (conIdx < 256 && !found) {
                                                if (speculativeDependencyGraph[prodIdx][prodSpecWay][prodUop].consumers[conIdx] == 0) {
                                                        found = true;
                                                        speculativeDependencyGraph[prodIdx][prodSpecWay][prodUop].consumers[conIdx] = flowPathIdx;
                                                } else if (speculativeDependencyGraph[prodIdx][prodSpecWay][prodUop].consumers[conIdx] == flowPathIdx) {
                                                        found = true;
                                                } else {
                                                        conIdx++;
                                                }
                                        }
                                        if (!found) {
                                                DPRINTF(SuperOp, "Not enough entries in consumers for inst %s (%i) speculative\n", decoder->speculativeCache[prodIdx][prodSpecWay][prodUop]->getName(), decoder->speculativeCache[prodIdx][prodSpecWay][prodUop]->numDestRegs());
                                                for (int i = 0; i < 256; i++) {
                                                        DPRINTF(SuperOp, "IFP at idx %i with consumer %x (%i)\n", speculativeDependencyGraph[prodIdx][prodSpecWay][prodUop].consumers[i], connections[speculativeDependencyGraph[prodIdx][prodSpecWay][prodUop].consumers[i]].consumer.pcAddr, connections[speculativeDependencyGraph[prodIdx][prodSpecWay][prodUop].consumers[i]].consumer.uopAddr);
                                                }
                                                assert(false);
                                        }
                                } else {
                                        DPRINTF(SuperOp, "Not enough entries in connections\n");
                                }
                        }
                }
        }

        // Update information for registers written by this instruction
        for (int i = 0; i < uop->numDestRegs(); i++) {
                RegId destReg = uop->destRegIdx(i);
                if (destReg.isIntReg()) {
                        // TODOEventually: Mark previous write as "bottom"???
                        // Update rename table
                        // DPRINTF(SuperOp, "Updating producer for register %i\n", destReg.flatIndex());
                        registerProducerMap[destReg.flatIndex()] = fullAddr;
                        registerValidMap[destReg.flatIndex()] = true;
                        registerRenameMap[destReg.flatIndex()] = nextRegName;
                        nextRegName++;
                }
        }
        }
}

void ArrayDependencyTracker::removeFromGraph(Addr addr, unsigned uopAddr)
{
        ArrayDependencyTracker::FullUopAddr fullAddr = ArrayDependencyTracker::FullUopAddr(addr, uopAddr);
        int idx = (addr >> 5) & 0x1f;
        uint64_t tag = (addr >> 10);
        int way = 0;

        // Mark IFPs invalid, first microop...
        for (int w=0; w < 8; w++) {
                if (decoder->uopValidArray[idx][w] && decoder->uopTagArray[idx][w] == tag && decoder->uopAddrArray[idx][w][uopAddr] == addr) {
                        way = w;
                        break;
                }
        }
        for (int i = 0; i < 256; i++) {
                unsigned prodIdx = microopDependencyGraph[idx][way][uopAddr].producers[i];
                if (prodIdx != 0) {
                        connectionsValid[prodIdx] = false;
                        microopDependencyGraph[idx][way][uopAddr].producers[i] = 0;
                }
                unsigned conIdx = microopDependencyGraph[idx][way][uopAddr].consumers[i];
                if (conIdx != 0) {
                        connectionsValid[conIdx] = false;
                        microopDependencyGraph[idx][way][uopAddr].consumers[i] = 0;
                }
        }
        microopDependencyGraph[idx][way][uopAddr] = DependGraphEntry();

        // ...now spec
        way = 0;
        for (int w=0; w < 8; w++) {
                if (decoder->speculativeValidArray[idx][w] && decoder->speculativeTagArray[idx][w] == tag && decoder->speculativeAddrArray[idx][w][uopAddr] == addr) {
                        way = w;
                        // DPRINTF(SuperOp, "found match at way %i\n", way);
                        break;
                }
        }
        for (int i = 0; i < 256; i++) {
                // DPRINTF(SuperOp, "Zeroing out way %i idx %i\n", way, i);
                unsigned prodIdx = speculativeDependencyGraph[idx][way][uopAddr].producers[i];
                if (prodIdx != 0) {
                        connectionsValid[prodIdx] = false;
                        speculativeDependencyGraph[idx][way][uopAddr].producers[i] = 0;
                }
                unsigned conIdx = speculativeDependencyGraph[idx][way][uopAddr].consumers[i];
                if (conIdx != 0) {
                        connectionsValid[conIdx] = false;
                        speculativeDependencyGraph[idx][way][uopAddr].consumers[i] = 0;
                }
        }
        speculativeDependencyGraph[idx][way][uopAddr] = DependGraphEntry();

        // update register rename info
        for (int i=0; i<256; i++) {
                if (registerProducerMap[i] == fullAddr) {
                        registerValidMap[i] = false;
                }
        }

        // Debugging, will delete
        idx = (addr >> 5) & 0x1f;
        tag = (addr >> 10);
        int uopway = 0;
        int specway = 0;
        for (int w = 0; w < 8; w++) {
                if (decoder->uopValidArray[idx][w] && decoder->uopTagArray[idx][w] == tag && decoder->uopAddrArray[idx][w][uopAddr] == addr) {
                        uopway = w;
                        break;
                }
        }
        for (int w=0; w<8; w++) {
                if (decoder->speculativeValidArray[idx][w] && decoder->speculativeTagArray[idx][w] == tag && decoder->speculativeAddrArray[idx][w][uopAddr] == addr) {
                        specway = w;
                        // DPRINTF(SuperOp, "found match at way %i\n", specway);
                        break;
                }
        }

        // Debugging
        // Are we sure this slot is empty?
        for (int i=0; i<256; i++) {
                assert(microopDependencyGraph[idx][uopway][uopAddr].producers[i] == 0);
                assert(microopDependencyGraph[idx][uopway][uopAddr].consumers[i] == 0);
                // DPRINTF(SuperOp, "Checking way %i idx %i\n", specway, i);
                assert(speculativeDependencyGraph[idx][specway][uopAddr].producers[i] == 0);
                assert(speculativeDependencyGraph[idx][specway][uopAddr].consumers[i] == 0);
        }
}

void ArrayDependencyTracker::predictValue(Addr addr, unsigned uopAddr, uint64_t value)
{
        int idx = (addr >> 5) & 0x1f;
        uint64_t tag = (addr >> 10);
        int way = 0;

        // Mark IFPs ready with value, only spec...
        for (int w=0; w < 8; w++) {
                if (decoder->speculativeValidArray[idx][w] && decoder->speculativeTagArray[idx][w] == tag && decoder->speculativeAddrArray[idx][w][uopAddr] == addr) {
                        way = w;
                        break;
                }
        }
        for (int i=0; i<256; i++) {
                if (speculativeDependencyGraph[idx][way][uopAddr].consumers[i] != 0) {
                        connections[speculativeDependencyGraph[idx][way][uopAddr].consumers[i]].value = value;
                        connections[speculativeDependencyGraph[idx][way][uopAddr].consumers[i]].valid = true;
                }
        }
}

bool ArrayDependencyTracker::simplifyGraph() {
        // Propagate constants
        for (int i1=0; i1<32; i1++) {
                for (int i2=0; i2<8; i2++) {
                        for (int i3=0; i3<6; i3++) {
                                if (decoder->speculativeValidArray[i1][i2] && decoder->speculativeCache[i1][i2][i3]) {
                                        string type = decoder->speculativeCache[i1][i2][i3]->getName();
                                        if (type == "mov" || type == "movi") {
                                                // TODO: pass values through, 1 src
                                        } else if (type == "xor" || type == "or" || type == "and" || type == "sub" || type == "add") {
                                                // TODO: bitwise C++, 2 srcs
                                        } else if (type == "subi" || type == "addi" || type == "slli" || type == "srli" || type == "lea" || type == "sexti" || type == "zexti") {
                                                // TODO: bitwise C++, 1 src and imm
                                        } else if (type == "mul1s" || type == "mul1u" || type == "mulel" || type == "muleh") {
                                                // TODO: two dest regs with different values? maybe too complex arithmetic?
                                        } else if (type == "limm") {
                                                // TODO: send imm to dests
                                        } else if (type == "rflags" || type == "wrflags" || type == "ruflags" || type == "wruflags") {
                                                // TODO: add control registers to graph?
                                        } else if (type == "rdtsc" || type == "rdval") {
                                                // TODO: determine whether direct register file access needs to be handled differently?
                                        } else if (type == "panic" || type == "NOP" || type == "CPUID") {
                                                // TODO: possibly remove, what is purpose?
                                        } else if (type == "st" || type == "stis" || type == "stfp" || type == "ld" || type == "ldis" || type == "ldst" || type == "syscall" || type == "halt" || type == "fault" || type == "call_far_Mp") {
                                                // TODO: cannot remove
                                        } else if (type == "wrip" || type == "rdip" || type == "wripi") {
                                                // TODO: if sequence length has changed, need to change these?
                                        } else {
                                                DPRINTF(SuperOp, "Inst type not covered: %s\n", type);
                                        }
                                }
                        }
                }
        }

        // TODO
        return false;
}

void ArrayDependencyTracker::measureChain(Addr addr, unsigned uopAddr)
{
        numChainsMeasured++;
        totalDependentInsts++;
        reducableInstCount++;

        vector<FullUopAddr> checked = vector<FullUopAddr>();
        checked.push_back(ArrayDependencyTracker::FullUopAddr(addr, uopAddr));

        int idx = (addr >> 5) & 0x1f;
        uint64_t tag = (addr >> 10);
        int way = 0;

        for (int w=0; w < 8; w++) {
                if (decoder->speculativeValidArray[idx][w] && decoder->speculativeTagArray[idx][w] == tag && decoder->speculativeAddrArray[idx][w][uopAddr] == addr) {
                        way = w;
                        break;
                }
        }

        // Update total reducable insts
        if (!speculativeDependencyGraph[idx][way][uopAddr].seen) {
                totalReducable++;
                speculativeDependencyGraph[idx][way][uopAddr].seen = true;
        }

        // validate consumers to continue chain
        for (int i=0; i<256; i++) {
                if (speculativeDependencyGraph[idx][way][uopAddr].consumers[i] != 0) {
                        connections[speculativeDependencyGraph[idx][way][uopAddr].consumers[i]].valid = true;
                }
        }

        for (int i=0; i<256; i++) {
                if (speculativeDependencyGraph[idx][way][uopAddr].consumers[i] != 0) {
                        // found a consumer
                        FullUopAddr conAddr = connections[speculativeDependencyGraph[idx][way][uopAddr].consumers[i]].consumer;
                        if (std::count(checked.begin(), checked.end(), conAddr) == 0) {
                                // recursive call
                                measureChain(conAddr, 1, checked);
                        }
                }
        }
}

void ArrayDependencyTracker::measureChain(FullUopAddr addr, unsigned recursionLevel, vector<FullUopAddr>& checked)
{
        if (recursionLevel > maxRecursiveDepth) { return; }

        int idx = (addr.pcAddr >> 5) & 0x1f;
        uint64_t tag = (addr.pcAddr >> 10);
        int way = 0;
        unsigned uopAddr = addr.uopAddr;

        for (int w=0; w < 8; w++) {
                if (decoder->speculativeValidArray[idx][w] && decoder->speculativeTagArray[idx][w] == tag && decoder->speculativeAddrArray[idx][w][uopAddr] == addr.pcAddr) {
                        way = w;
                        break;
                }
        }

        totalDependentInsts++;
        bool allReady = true;
        for (int i=0; i<256; i++) {
                if (speculativeDependencyGraph[idx][way][uopAddr].producers[i] != 0) {
                        if (!connections[speculativeDependencyGraph[idx][way][uopAddr].producers[i]].valid) {
                                allReady = false;
                        }
                }
        }
        if (allReady) {
                reducableInstCount++;
                if (!speculativeDependencyGraph[idx][way][uopAddr].seen) {
                        totalReducable++;
                        speculativeDependencyGraph[idx][way][uopAddr].seen = true;
                }
                // validate consumers to continue chain
                for (int i=0; i<256; i++) {
                        if (speculativeDependencyGraph[idx][way][uopAddr].consumers[i] != 0) {
                                connections[speculativeDependencyGraph[idx][way][uopAddr].consumers[i]].valid = true;
                        }
                }
        }

        for (int i=0; i<256; i++) {
                if (speculativeDependencyGraph[idx][way][uopAddr].consumers[i] != 0) {
                        // Found a consumer
                        FullUopAddr conAddr = connections[speculativeDependencyGraph[idx][way][uopAddr].consumers[i]].consumer;
                        if (std::count(checked.begin(), checked.end(), conAddr) == 0) {
                                // Recursive call
                                measureChain(conAddr, recursionLevel+1, checked);
                        }
                }
        }
}

bool ArrayDependencyTracker::isReducable(Addr addr, unsigned uopAddr) {
        int idx = (addr >> 5) & 0x1f;
        uint64_t tag = (addr >> 10);
        int way = 0;

        for (int w = 0; w < 8; w++) {
                if (decoder->speculativeValidArray[idx][w] && decoder->speculativeTagArray[idx][w] == tag && decoder->speculativeAddrArray[idx][w][uopAddr] == addr) {
                        way = w;
                        break;
                }
        }

        return speculativeDependencyGraph[idx][way][uopAddr].seen;
}

ArrayDependencyTracker* ArrayDependencyTrackerParams::create() {
        return new ArrayDependencyTracker(this);
}
