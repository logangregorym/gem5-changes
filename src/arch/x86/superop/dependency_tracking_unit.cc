#include <algorithm>

#include "arch/x86/decoder.hh"
#include "arch/x86/regs/misc.hh"
#include "arch/x86/superop/dependency_tracking_unit.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/SuperOp.hh"

using namespace std;

// namespace X86ISA {

DependencyTrackingUnit::DependencyTrackingUnit(DependencyTrackingUnitParams *p) : SimObject(p) {
        microopDependencyGraph = vector<DependencyTrackingUnit::DependGraphEntry>();
        speculativeDependencyGraph = vector<DependencyTrackingUnit::DependGraphEntry>();
}

void DependencyTrackingUnit::regStats()
{
        // using namespace Stats;
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
}

void DependencyTrackingUnit::addToGraph(StaticInstPtr uop, Addr addr, unsigned uopAddr) {
        // DPRINTF(SuperOp, "Called addToGraph; inst 0x%x (%i) has %i source registers and %i destination registers\n", addr, uopAddr, uop->numSrcRegs(), uop->numDestRegs());
        DependencyTrackingUnit::FullUopAddr fullAddr = DependencyTrackingUnit::FullUopAddr(addr, uopAddr);
        DependencyTrackingUnit::DependGraphEntry newEntry = DependencyTrackingUnit::DependGraphEntry(fullAddr);
        for (int i = 0; i < uop->numSrcRegs(); i++) {
                RegId srcReg = uop->srcRegIdx(i);
                // DPRINTF(SuperOp, "Instruction 0x%x (%i) has source register %i\n", addr, uopAddr, srcReg.flatIndex());
                if (srcReg.isIntReg()) { // || srcReg.isFloatReg() || srcReg.isCCReg()) {
                        if (registerValidMap[srcReg.flatIndex()]) {
                                InformationFlowPath path = InformationFlowPath(registerProducerMap[srcReg.flatIndex()], fullAddr, srcReg.flatIndex(), registerRenameMap[srcReg.flatIndex()]);
                                newEntry.producers.push_back(path);
                                // DPRINTF(SuperOp, "Added a producer to inst 0x%x (%i) for register %i\n", addr, uopAddr, srcReg.flatIndex());
                                // Now, find that producer and add this inst to its consumers
                                DependencyTrackingUnit::FullUopAddr prod = registerProducerMap[srcReg.flatIndex()];
                                // unsigned prodOp = registerUopMap[srcReg.flatIndex()];
                                for (vector<DependencyTrackingUnit::DependGraphEntry>::iterator it = microopDependencyGraph.begin(); it < microopDependencyGraph.end(); it++) {
                                        if ((*it).thisInst == prod) { // && (*it).thisOp == prodOp) {
                                                (*it).consumers.push_back(path);
                                        }
                                }
                                for (vector<DependencyTrackingUnit::DependGraphEntry>::iterator it2 = speculativeDependencyGraph.begin(); it2 < speculativeDependencyGraph.end(); it2++) {
                                        if ((*it2).thisInst == prod) { // && (*it2).thisOp == prodOp) {
                                                (*it2).consumers.push_back(path);
                                        }
                                }
                        }
                }
        }
        microopDependencyGraph.push_back(newEntry);
        speculativeDependencyGraph.push_back(newEntry);
        for (int i = 0; i < uop->numDestRegs(); i++) {
                RegId destReg = uop->destRegIdx(i);
                // DPRINTF(SuperOp, "Instruction 0x%x (%i) has destination register %i\n", addr, uopAddr, destReg.flatIndex());
                if (destReg.isIntReg()) { // || destReg.isFloatReg() || destReg.isCCReg()) {

/**
                        // previous value just became dead! annotate last read as last use?
                        // access last read with a backwards pass through the graph?
                        bool foundLastUse = false;
                        vector<DependencyTrackingUnit::DependGraphEntry>::iterator lastEntry = speculativeDependencyGraph.end();
                        while (lastEntry != speculativeDependencyGraph.begin() && !foundLastUse) {
                                // check all producers for matching register
                                // don't need to know producer since last use is produced most recently
                                for (vector<DependencyTrackingUnit::InformationFlowPath>::iterator possibleUse = lastEntry->producers.begin(); possibleUse < lastEntry->producers.end(); lastEntry++) {
                                        if (possibleUse->renamedRegIdx == registerRenameMap[destReg.flatIndex()] && registerValidMap[destReg.flatIndex()]) {
                                                possibleUse->lastUse = true;
                                                foundLastUse = true;
                                        }
                                }
                                lastEntry--;
                        }
**/

                        registerProducerMap[destReg.flatIndex()] = fullAddr;
                        // registerUopMap[destReg.flatIndex()] = uopAddr;
                        registerValidMap[destReg.flatIndex()] = true;
                        registerRenameMap[destReg.flatIndex()] = nextRegName;
                        nextRegName++;
                        // Not adding as a producer, because expects insts in program order
                }
        }
}

void DependencyTrackingUnit::removeFromGraph(StaticInstPtr uop, Addr addr, unsigned uopAddr) {
        DependencyTrackingUnit::FullUopAddr fullAddr = DependencyTrackingUnit::FullUopAddr(addr, uopAddr);
        vector<DependencyTrackingUnit::DependGraphEntry>::iterator it = microopDependencyGraph.begin();
        while (it != microopDependencyGraph.end()) {
                if ((*it).thisInst == fullAddr) { // && (*it).thisOp == uopAddr) {
                        it = microopDependencyGraph.erase(it);
                } else {
                        vector<InformationFlowPath>::iterator it2 = (*it).producers.begin();
                        while (it2 != (*it).producers.end()) {
                                if ((*it2).producer == fullAddr) { // && (*it2).producerOp == uopAddr) {
                                        it2 = (*it).producers.erase(it2);
                                } else {
                                        it2++;
                                }
                        }
                        it2 = (*it).consumers.begin();
                        while (it2 != (*it).consumers.end()) {
                                if ((*it2).consumer == fullAddr) { // && (*it2).consumer == uopAddr) {
                                        it2 = (*it).consumers.erase(it2);
                                } else {
                                        it2++;
                                }
                        }
                        it++;
                }
        }
        vector<DependencyTrackingUnit::DependGraphEntry>::iterator it3 = speculativeDependencyGraph.begin();
        while (it3 != speculativeDependencyGraph.end()) {
                if ((*it3).thisInst == fullAddr) { // && (*it3).thisOp == uopAddr) {
                        it3 = speculativeDependencyGraph.erase(it3);
                } else {
                        vector<InformationFlowPath>::iterator it4 = (*it3).producers.begin();
                        while (it4 != (*it3).producers.end()) {
                                if ((*it4).producer == fullAddr) { // && (*it4).producerOp == uopAddr) {
                                        it4 = (*it3).producers.erase(it4);
                                } else {
                                        it4++;
                                }
                        }
                        it4 = (*it3).consumers.begin();
                        while (it4 != (*it3).consumers.end()) {
                                if ((*it4).consumer == fullAddr) { // && (*it4).consumer == uopAddr) {
                                        it4 = (*it3).consumers.erase(it4);
                                } else {
                                        it4++;
                                }
                        }
                        it3++;
                }
        }
        for (int i = 0; i < 256; i++) {
                // 256 should change!
                if (registerProducerMap[i] == fullAddr) { // && registerUopMap[i] == uopAddr) {
                        registerValidMap[i] = false;
                }
        }
}

bool DependencyTrackingUnit::simplifyGraph() {
        bool changeMade = false;
        vector<DependencyTrackingUnit::DependGraphEntry>::iterator it = speculativeDependencyGraph.begin();
        while (it != speculativeDependencyGraph.end()) {
                bool allReady = true;
                int count = 0;
                int idx = (it->thisInst.pcAddr >> 5) & 0x1f;
                uint64_t tag = (it->thisInst.pcAddr >> 10);
                StaticInstPtr staticInst = NULL;
                for (int way = 0; way < 8; way++) {
                        if (decoder->speculativeValidArray[idx][way] && decoder->speculativeTagArray[idx][way] == tag && decoder->speculativeAddrArray[idx][way][it->thisInst.uopAddr] == it->thisInst.pcAddr) {
                                staticInst = decoder->speculativeCache[idx][way][it->thisInst.uopAddr];
                        }
                }
                if (!staticInst) {
                        DPRINTF(SuperOp, "Address %x (%i) not found in cache\n", it->thisInst.pcAddr, it->thisInst.uopAddr);
                } else {
                        for (vector<DependencyTrackingUnit::InformationFlowPath>::iterator p = it->producers.begin(); p != it->producers.end(); p++) {
                                if (!p->valid) {
                                        allReady = false;
                                        assert(p->consumer == it->thisInst);
                                        // DPRINTF(SuperOp, "Value produced by inst 0x%x (%i) not ready for inst 0x%x (%i)\n", p->producer.pcAddr, p->producer.uopAddr, p->consumer.pcAddr, p->consumer.uopAddr);
                                        DPRINTF(SuperOp, "%s waiting on value\n", staticInst->getName());
                                }
                                count++;
                        }
                        if (allReady && count > 0) {
                                // DPRINTF(SuperOp, "Instruction 0x%x (%i) of has all %i values, candidate for reduction\n", it->thisInst.pcAddr, it->thisInst.uopAddr, count);
                        } else if (count > 0) {
                                //DPRINTF(SuperOp, "Instruction 0x%x (%i) only has %i values, not a candidate for reduction\n", it->thisInst.pcAddr, it->thisInst.uopAddr, count);
                        }
                        if (allReady) {
                                DPRINTF(SuperOp, "%s ready with %i values\n", staticInst->getName(), count);
                        }
                }
                it++;
        }
        return changeMade;
}

void DependencyTrackingUnit::predictValue(Addr addr, unsigned uopAddr, uint64_t value) {
        for (vector<DependencyTrackingUnit::DependGraphEntry>::iterator it = speculativeDependencyGraph.begin(); it != speculativeDependencyGraph.end(); it++) {
                if (it->thisInst.pcAddr == addr && it->thisInst.uopAddr == uopAddr) {
                        // DPRINTF(SuperOp, "Marking consumers of %x (%i) ready\n", it->thisInst.pcAddr, it->thisInst.uopAddr);
                        for (vector<DependencyTrackingUnit::InformationFlowPath>::iterator c = it->consumers.begin(); c != it->consumers.end(); c++) {
                                // Annotate each consuming path with the value
                                c->value = value;
                                c->valid = true;
                                vector<DependencyTrackingUnit::DependGraphEntry>::iterator p = findInSpeculativeGraph(c->consumer);
                                for (vector<DependencyTrackingUnit::InformationFlowPath>::iterator prod = p->producers.begin(); prod != p->producers.end(); prod++) {
                                        if (prod->producer.pcAddr == addr && prod->producer.uopAddr == uopAddr) {
                                                prod->value = value;
                                                prod->valid = true;
                                        }
                                }
                        }
                }
        }
}

vector<DependencyTrackingUnit::DependGraphEntry>::iterator DependencyTrackingUnit::findInSpeculativeGraph(Addr addr, unsigned uopAddr) {
        vector<DependencyTrackingUnit::DependGraphEntry>::iterator it = speculativeDependencyGraph.begin();
        while (it != speculativeDependencyGraph.end()) {
                if (it->thisInst.pcAddr == addr && it->thisInst.uopAddr == uopAddr) {
                        return it;
                }
                it++;
        }
        return it;
}

vector<DependencyTrackingUnit::DependGraphEntry>::iterator DependencyTrackingUnit::findInSpeculativeGraph(DependencyTrackingUnit::FullUopAddr fullAddr) {
        vector<DependencyTrackingUnit::DependGraphEntry>::iterator it = speculativeDependencyGraph.begin();
        while (it != speculativeDependencyGraph.end()) {
                if (it->thisInst == fullAddr) {
                        return it;
                }
                it++;
        }
        return it;
}

vector<DependencyTrackingUnit::DependGraphEntry>::iterator DependencyTrackingUnit::findInOriginalGraph(Addr addr, unsigned uopAddr) {
        vector<DependencyTrackingUnit::DependGraphEntry>::iterator it = microopDependencyGraph.begin();
        while (it != microopDependencyGraph.end()) {
                if (it->thisInst.pcAddr == addr && it->thisInst.uopAddr == uopAddr) {
                        return it;
                }
                it++;
        }
        return it;
}

vector<DependencyTrackingUnit::DependGraphEntry>::iterator DependencyTrackingUnit::findInOriginalGraph(DependencyTrackingUnit::FullUopAddr fullAddr) {
        vector<DependencyTrackingUnit::DependGraphEntry>::iterator it = microopDependencyGraph.begin();
        while (it != microopDependencyGraph.end()) {
                if (it->thisInst == fullAddr) {
                        return it;
                }
                it++;
        }
        return it;
}

void DependencyTrackingUnit::measureChain(DependencyTrackingUnit::DependGraphEntry& start) {
        // First call starts a new chain, is a new inst, and is reducable
        numChainsMeasured++;
        totalDependentInsts++;
        reducableInstCount++;

        vector<FullUopAddr> checked = vector<FullUopAddr>();
        checked.push_back(start.thisInst);
        vector<InformationFlowPath>::iterator it = start.consumers.begin();
        while (it != start.consumers.end()) {
                // Two passes to ensure all marked valid before recursive call
                it->valid = true;
                it++;
        }
        it = start.consumers.begin();
        while (it != start.consumers.end()) {
                // For each consumer...
                if (std::count(checked.begin(), checked.end(), it->consumer) == 0) {
                        // If not already visited, recursive call
                        vector<DependGraphEntry>::iterator con = findInOriginalGraph(it->consumer);
                        if (con != microopDependencyGraph.end()) {
                                measureChain((*con), 1, checked);
                        }
                }
                it++;
        }
}

void DependencyTrackingUnit::measureChain(DependencyTrackingUnit::DependGraphEntry& start, unsigned recursionLevel, vector<FullUopAddr> checked) {
        if (recursionLevel > maxRecursiveDepth) { return; }

        // Subsequent calls add an inst and may or may not be reducable
        totalDependentInsts++;
        bool allReady = true;
        vector<InformationFlowPath>::iterator it;
        for (it = start.producers.begin(); it != start.producers.end(); it++) {
                if (!it->valid) {
                        allReady = false;
                }
        }
        if (allReady) {
                reducableInstCount++;
                for (it = start.consumers.begin(); it != start.consumers.end(); it++) {
                        it->valid = true;
                }
        }
        it = start.consumers.begin();
        while (it != start.consumers.end()) {
                if (std::count(checked.begin(), checked.end(), it->consumer) == 0) {
                        vector<DependGraphEntry>::iterator con = findInOriginalGraph(it->consumer);
                        if (con != microopDependencyGraph.end()) {
                                measureChain((*con), recursionLevel+1, checked);
                        }
                }
                it++;
        }
}

DependencyTrackingUnit* DependencyTrackingUnitParams::create() {
        return new DependencyTrackingUnit(this);
}

// } // namespace X86ISA
