#include "arch/x86/types.hh"
#include "arch/x86/decoder.hh"
#include "arch/x86/insts/microop.hh"
#include "arch/x86/insts/microregop.hh"
#include "arch/x86/regs/misc.hh"
#include "arch/x86/superop/array_dependency_tracker.hh"
#include "base/bitfield.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/SuperOp.hh"
#include "debug/ConstProp.hh"

using namespace std;

ArrayDependencyTracker::ArrayDependencyTracker(ArrayDependencyTrackerParams *p) : SimObject(p), usingControlTracking(p->usingControlTracking) {
        for (int i=0; i<256; i++) {
                registerProducerMapUop[i] = FullUopAddr(0,0);
                registerProducerMapSpec[i] = FullUopAddr(0,0);
                mostRecentConsumer[i] = FullUopAddr(0,0);
        }
}

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
    branchesOnChains
                .name("system.switch_cpus.decode.superop.dependencyTracker.branchesOnChains")
                .desc("Count of branches found while measuring chains")
                ;
    confidentBranchesOnChains
                .name("system.switch_cpus.decode.superop.dependencyTracker.confidentBranchesOnChains")
                .desc("Count of confident branches found while measuring chains")
                ;
    percentChainBranchesConfident
                .name("system.switch_cpus.decode.superop.dependencyTracker.percentChainBranchesConfident")
                .desc("Percent of branches found while measuring chains which were confident")
                .precision(6);
    percentChainBranchesConfident = (confidentBranchesOnChains / branchesOnChains)*100;
        totalCyclesInUopCache
                .name("system.switch_cpus.decode.superop.dependencyTracker.totalCyclesInUopCache")
                .desc("Total number of cycles any instruction spends in the uop cache")
                ;
        evictionsFromUopCache
                .name("system.switch_cpus.decode.superop.dependencyTracker.evictionsFromUopCache")
                .desc("Total evictions from the uop cache")
                ;
        totalCyclesInSpecCache
                .name("system.switch_cpus.decode.superop.dependencyTracker.totalCyclesInSpecCache")
                .desc("Total number of cycles any instruction spends in the speculative cache")
                ;
        evictionsFromSpecCache
                .name("system.switch_cpus.decode.superop.dependencyTracker.evictionsFromSpecCache")
                .desc("Total evictions from the speculative cache")
                ;
        averageCyclesInUopCache
                .name("system.switch_cpus.decode.superop.dependencyTracker.averageCyclesInUopCache")
                .desc("Average number of cycles an instruction spends in the uop cache")
                .precision(6);
        averageCyclesInSpecCache
                .name("system.switch_cpus.decode.superop.dependencyTracker.averageCyclesInSpecCache")
                .desc("Average number of cycles an instruction spends in the speculative cache")
                .precision(6);
        averageCyclesInUopCache = totalCyclesInUopCache / evictionsFromUopCache;
        averageCyclesInSpecCache = totalCyclesInSpecCache / evictionsFromSpecCache;
}

void ArrayDependencyTracker::addToGraph(StaticInstPtr uop, Addr addr, unsigned uopAddr, unsigned cycleAdded, bool usingUopGraph)
{
        if (usingUopGraph) { ++totalOpsInCache; }
        DPRINTF(ConstProp, "Adding entry for %x.%i to graph (uop? %i)\n", addr, uopAddr, usingUopGraph);


    // Clear the slot
    removeFromGraph(addr, uopAddr, cycleAdded, usingUopGraph);

    ArrayDependencyTracker::FullUopAddr fullAddr = ArrayDependencyTracker::FullUopAddr(addr, uopAddr);
    int idx = (addr >> 5) & 0x1f;
    uint64_t tag = (addr >> 10);

        if (usingUopGraph) {

                // Add to uop graph
                int uopway = 0;
                int uopuop = 0;
                bool foundUop = false;
                for (int w = 0; w < 8; w++) {
                if (decoder->uopValidArray[idx][w] && decoder->uopTagArray[idx][w] == tag && !foundUop) {
                assert(decoder->uopCountArray[idx][w] > 0);
                                for (int uop=0; uop < decoder->uopCountArray[idx][w]; uop++) {
                                        if (microopAddrArray[idx][w][uop] == fullAddr && !foundUop) {
                                                assert(microopDependencyGraph[idx][w][uop]);
                                                uopway = w;
                                                uopuop = uop;
                                                foundUop = true;
                                        }
                                }
                }
                }

                if (!foundUop) {
                        DPRINTF(ConstProp, "Not adding %x.%i to uop graph bc not found in cache :(\n", addr, uopAddr);
                }

                assert(!(microopDependencyGraph[idx][uopway][uopuop]->thisInst == fullAddr)); // means same inst loaded twice

                DPRINTF(ConstProp, "Adding at uop[%i][%i][%i]\n", idx, uopway, uopuop);

                microopDependencyGraph[idx][uopway][uopuop]->thisInst = fullAddr;
                microopDependencyGraph[idx][uopway][uopuop]->cycleAdded = cycleAdded;

        // Search for source registers in graph
        for (int i = 0; i < uop->numSrcRegs(); i++) {
                RegId srcReg = uop->srcRegIdx(i);
                        if (registerValidMapUop[srcReg.flatIndex()]) {
                                DPRINTF(ConstProp, "Reg %i was written by inst %x.%i\n", srcReg.flatIndex(), registerProducerMapUop[srcReg.flatIndex()].pcAddr, registerProducerMapUop[srcReg.flatIndex()].uopAddr);
                                int prodIdx = (registerProducerMapUop[srcReg.flatIndex()].pcAddr >> 5) & 0x1f;
                                uint64_t prodTag = (registerProducerMapUop[srcReg.flatIndex()].pcAddr >> 10);
                                bool foundProducerEntry = false;
                                int prodWay = 0;
                                int prodUop = 0;
                                for (int w = 0; w < 8; w++) {
                                        if (decoder->uopValidArray[prodIdx][w] && decoder->uopTagArray[prodIdx][w] == prodTag && !foundProducerEntry) {
                                                assert(decoder->uopCountArray[prodIdx][w] != 0);
                                                for (int u=0; u < decoder->uopCountArray[prodIdx][w]; u++) {
                                                        if (microopAddrArray[prodIdx][w][u] == registerProducerMapUop[srcReg.flatIndex()] && !foundProducerEntry) {
                                                                assert(microopDependencyGraph[prodIdx][w][u]);
                                                                prodWay = w;
                                                                prodUop = u;
                                                                foundProducerEntry = true;
                                                        }
                                                }
                                        }
                                }
                // if written in window, create an information flow path to this inst
                unsigned flowPathIdx = 1; // 0 reserved for invalid
                bool foundSource = false;
                while (flowPathIdx < 4096 && !foundSource) {
                    if (!connectionsValidUop[flowPathIdx] && !connectionsValidSpec[flowPathIdx]) {
                        foundSource = true;
                                                DPRINTF(ConstProp, "Found an empty IFP slot at %i\n", flowPathIdx);
                        break;
                    } else if (!connectionsValidUop[flowPathIdx]) {
                                                // TODO: check whether the same connection exists in spec, set valid and move on if possible
                                                flowPathIdx++;
                                        } else {
                        flowPathIdx++;
                    }
                }
                                if (foundSource) {
                                        if (foundProducerEntry) {
                                                DPRINTF(ConstProp, "Found entry for producer at uop[%i][%i][%i]\n", prodIdx, prodWay, prodUop);
                                                // Add to the producer inst as a consumer!
                                                bool foundConsumerIdx = false;
                                                int consumerIndex = 0;
                                                while (consumerIndex < 256 && !foundConsumerIdx) {
                                                        if (microopDependencyGraph[prodIdx][prodWay][prodUop]->consumers[consumerIndex] == 0) {
                                                                foundConsumerIdx = true;
                                                                microopDependencyGraph[prodIdx][prodWay][prodUop]->consumers[consumerIndex] = flowPathIdx;
                                                        } else if (microopDependencyGraph[prodIdx][prodWay][prodUop]->consumers[consumerIndex] == flowPathIdx) {
                                                                DPRINTF(ConstProp, "Consumer at connections[%i] has already been added???\n", flowPathIdx);
                                                                foundConsumerIdx = true;
                                                        } else {
                                                                consumerIndex++;
                                                        }
                                                }
                                                if (!foundConsumerIdx) {
                                                        DPRINTF(ConstProp, "Not enough entries in consumers for inst at uop[%i][%i][%i]\n", prodIdx, prodWay, prodUop);
                                                }
                                        } else {
                                                panic("Failed to find entry for producer %x.%i with tag %x and index %i in uop graph\n", registerProducerMapUop[srcReg.flatIndex()].pcAddr, registerProducerMapUop[srcReg.flatIndex()].uopAddr, prodTag, prodIdx);
                                        }

                                        connections[flowPathIdx] = InformationFlowPath(registerProducerMapUop[srcReg.flatIndex()], fullAddr, srcReg.flatIndex(), registerRenameMapUop[srcReg.flatIndex()]);
                                        connectionsValidUop[flowPathIdx] = true;
                                        DPRINTF(ConstProp, "Created connection at index %i through register %i (SSA ID %i) from %x.%i to %x.%i\n", flowPathIdx, srcReg.flatIndex(), registerRenameMapUop[srcReg.flatIndex()], registerProducerMapUop[srcReg.flatIndex()].pcAddr, registerProducerMapUop[srcReg.flatIndex()].uopAddr, fullAddr.pcAddr, fullAddr.uopAddr);

                                        // Add the path to this inst's producers
                                        unsigned prodIdx = 0;
                                        bool found = false;
                                        while (prodIdx < 256 && !found) {
                                                if (microopDependencyGraph[idx][uopway][uopuop]->producers[prodIdx] == 0) {
                                                        found = true;
                                                        microopDependencyGraph[idx][uopway][uopuop]->producers[prodIdx] = flowPathIdx;
                                                } else if (microopDependencyGraph[idx][uopway][uopuop]->producers[prodIdx] == flowPathIdx) {
                                                        DPRINTF(ConstProp, "Producer at connections [%i] has already been added???\n", flowPathIdx);
                                                        found = true;
                                                } else {
                                                        prodIdx++;
                                                }
                                        }
                                        if (!found) {
                                                DPRINTF(SuperOp, "Not enough entries in producers for inst %s (%i)\n", uop->getName(), uop->numSrcRegs());
                                        }
                                } else {
                    DPRINTF(SuperOp, "Not enough entries in connections\n");
                }
            } else {
                                // Add dummy producer
                    unsigned prodIdx = 0;
                    bool found = false;
                    while (prodIdx < 256 && !found) {
                        if (microopDependencyGraph[idx][uopway][uopuop]->producers[prodIdx] == 0) {
                            found = true;
                            microopDependencyGraph[idx][uopway][uopuop]->producers[prodIdx] = 5000;
                        } else {
                            prodIdx++;
                        }
                    }
                    if (!found) {
                        DPRINTF(SuperOp, "Not enough entries in producers for inst %s (%i)\n", uop->getName(), uop->numSrcRegs());
                    }
                        }
                }

                // Update information for registers written by this instruction
        for (int i = 0; i < uop->numDestRegs(); i++) {
                        // TODO: Mark previous value overwritten a bottom of chain?
                RegId destReg = uop->destRegIdx(i);
                registerProducerMapUop[destReg.flatIndex()] = fullAddr;
                registerValidMapUop[destReg.flatIndex()] = true;
                registerRenameMapUop[destReg.flatIndex()] = nextRegNameUop;
                nextRegNameUop++;
                }
        } else {
                // Add to spec graph
                int specway = 0;
                int specuop = 0;
                bool foundSpec = false;
                for (int w = 0; w < 8; w++) {
                if (decoder->uopValidArray[idx][w] && decoder->uopTagArray[idx][w] == tag && !foundSpec) { // changed to uop
                assert(decoder->uopCountArray[idx][w] > 0); // changed to uop
                                for (int uop=0; uop < decoder->uopCountArray[idx][w]; uop++) { // changed to uop
                                        if (microopAddrArray[idx][w][uop] == fullAddr && !foundSpec) { // changed to uop
                                                assert(speculativeDependencyGraph[idx][w][uop]);
                                                specway = w;
                                                specuop = uop;
                                                foundSpec = true;
                                        }
                                }
                }
                }

                if (!foundSpec) {
                        DPRINTF(ConstProp, "Not adding %x.%i to spec graph bc not found in cache :(\n", addr, uopAddr);
                }

        assert(!(speculativeDependencyGraph[idx][specway][specuop]->thisInst == fullAddr)); // means same inst loaded twice

                DPRINTF(ConstProp, "Adding at spec[%i][%i][%i]\n", idx, specway, specuop);

                speculativeDependencyGraph[idx][specway][specuop]->thisInst = fullAddr;
                speculativeDependencyGraph[idx][specway][specuop]->cycleAdded = cycleAdded;

                // Branches handled differently
                if (uop->isControl() && usingControlTracking) {
                        DPRINTF(ConstProp, "Branch found with disassembly: %s\n", uop->disassemble(fullAddr.pcAddr));

                // Add one new control path for taken, and one for not taken
                        unsigned takenIndex = 0;
                        unsigned notTakenIndex = 0;
                        for (int i=1; i<4096; i++) {
                                if (!branchesValid[i] && takenIndex == 0) {
                                        takenIndex = i;
                                } else if (!branchesValid[i] && notTakenIndex == 0) {
                                        notTakenIndex = i;
                                }
                        }

                        string type = uop->getName();
                        if (type == "wripi") {
                                // PC-relative branches can be resolved at fetch
                                DPRINTF(ConstProp, "Branch with type wripi\n");
                        } else if (type == "wrip") {
                                DPRINTF(ConstProp, "Branch with type wrip\n");
                        } else {
                                DPRINTF(ConstProp, "Branch with type %s\n", type);
                        }

                        branches[takenIndex].branchAddr = fullAddr;
                        branches[takenIndex].nextPc = FullUopAddr(0,0); // TODO: replace if available
                        branches[takenIndex].targetValid = false;
                        branches[takenIndex].propagatingTo = FullUopAddr(0,0);

                        Addr nextPc = fullAddr.pcAddr + uop->machInst.instSize;
                        DPRINTF(ConstProp, "Not branching from %x to %x\n", fullAddr.pcAddr, nextPc);

                        branches[notTakenIndex].branchAddr = fullAddr;
                        branches[notTakenIndex].nextPc = FullUopAddr(nextPc, 0);
                        branches[notTakenIndex].targetValid = true;
                        branches[notTakenIndex].propagatingTo = FullUopAddr(nextPc, 0);

                        speculativeDependencyGraph[idx][specway][specuop]->consumers[0] = takenIndex;
                        speculativeDependencyGraph[idx][specway][specuop]->consumers[1] = notTakenIndex;

                        for (int i=0; i<256; i++) {
                                branches[takenIndex].registerRenameMap[i] = registerRenameMapSpec[i];
                                branches[takenIndex].registerProducerMap[i] = registerProducerMapSpec[i];
                                branches[takenIndex].registerValidMap[i] = registerValidMapSpec[i];

                                branches[notTakenIndex].registerRenameMap[i] = registerRenameMapSpec[i];
                                branches[notTakenIndex].registerProducerMap[i] = registerProducerMapSpec[i];
                                branches[notTakenIndex].registerValidMap[i] = registerValidMapSpec[i];

                                registerValidMapSpec[i] = false;
                                consumedInWindow[i] = false;
                        }
                        DPRINTF(ConstProp, "Control signals are wild. Here are the bools for this %s: isControl %i; isCall %i; isReturn %i; isDirectCtrl %i; isIndirectCtrl %i; isCondCtrl %i; isUncondCtrl %i; isCondDelaySlot %i\n", uop->getName(), uop->isControl(), uop->isCall(), uop->isReturn(), uop->isDirectCtrl(), uop->isIndirectCtrl(), uop->isCondCtrl(), uop->isUncondCtrl(), uop->isCondDelaySlot());
        }

        // Search for source registers in graph
        for (int i = 0; i < uop->numSrcRegs(); i++) {
                RegId srcReg = uop->srcRegIdx(i);
                        if (registerValidMapSpec[srcReg.flatIndex()]) {
                                DPRINTF(ConstProp, "Reg %i was written by inst %x.%i\n", srcReg.flatIndex(), registerProducerMapSpec[srcReg.flatIndex()].pcAddr, registerProducerMapSpec[srcReg.flatIndex()].uopAddr);
                                // Update consumer tracking for last use
                                mostRecentConsumer[srcReg.flatIndex()] = fullAddr;
                                consumedInWindow[srcReg.flatIndex()] = true;
                                int prodIdx = (registerProducerMapSpec[srcReg.flatIndex()].pcAddr >> 5) & 0x1f;
                                uint64_t prodTag = (registerProducerMapSpec[srcReg.flatIndex()].pcAddr >> 10);
                                bool foundProducerEntry = false;
                                int prodWay = 0;
                                int prodUop = 0;
                                for (int w = 0; w < 8; w++) {
                                        if (decoder->uopValidArray[prodIdx][w] && decoder->uopTagArray[prodIdx][w] == prodTag && !foundProducerEntry) { // changed to uop
                                                assert(decoder->uopCountArray[prodIdx][w] != 0); // changed to  uop
                                                for (int u=0; u < decoder->uopCountArray[prodIdx][w]; u++) { // changed to uop
                                                        if (microopAddrArray[prodIdx][w][u] == registerProducerMapSpec[srcReg.flatIndex()] && !foundProducerEntry) { // changed to uop
                                                                assert(speculativeDependencyGraph[prodIdx][w][u]);
                                                                prodWay = w;
                                                                prodUop = u;
                                                                foundProducerEntry = true;
                                                        }
                                                }
                                        }
                                }

                // if written in window, create an information flow path to this inst
                unsigned flowPathIdx = 1; // 0 reserved for invalid
                bool foundSource = false;
                while (flowPathIdx < 4096 && !foundSource) {
                    if (!connectionsValidUop[flowPathIdx] && !connectionsValidSpec[flowPathIdx]) {
                        foundSource = true;
                                                DPRINTF(ConstProp, "Found an empty IFP slot at %i\n", flowPathIdx);
                        break;
                    } else if (!connectionsValidSpec[flowPathIdx]) {
                                                flowPathIdx++;
                                        } else {
                        flowPathIdx++;
                    }
                }
                                if (foundSource) {
                                        if (foundProducerEntry) {
                                                DPRINTF(ConstProp, "Found entry for producer at spec[%i][%i][%i]\n", prodIdx, prodWay, prodUop);
                                                // Add to the producer inst as a consumer!
                                                bool foundConsumerIdx = false;
                                                int consumerIndex = 0;
                                                while (consumerIndex < 256 && !foundConsumerIdx) {
                                                        if (speculativeDependencyGraph[prodIdx][prodWay][prodUop]->consumers[consumerIndex] == 0) {
                                                                foundConsumerIdx = true;
                                                                speculativeDependencyGraph[prodIdx][prodWay][prodUop]->consumers[consumerIndex] = flowPathIdx;
                                                        } else if (speculativeDependencyGraph[prodIdx][prodWay][prodUop]->consumers[consumerIndex] == flowPathIdx) {
                                                                DPRINTF(ConstProp, "Consumer at connections[%i] has already been added???\n", flowPathIdx);
                                                                foundConsumerIdx = true;
                                                        } else {
                                                                consumerIndex++;
                                                        }
                                                }
                                                if (!foundConsumerIdx) {
                                                        DPRINTF(ConstProp, "Not enough entries in consumers for inst at uop[%i][%i][%i]\n", prodIdx, prodWay, prodUop);
                                                }

                                                // mark with value if already predicted
                                                if (speculativeDependencyGraph[prodIdx][prodWay][prodUop]->predicted) {
                                                        connections[flowPathIdx].valid = speculativeDependencyGraph[prodIdx][prodWay][prodUop]->value;
                                                        connections[flowPathIdx].value = speculativeDependencyGraph[prodIdx][prodWay][prodUop]->value;
                                                }
                                        } else {
                                                panic("Failed to find entry for producer %x.%i with tag %x and index %i in spec graph\n", registerProducerMapSpec[srcReg.flatIndex()].pcAddr, registerProducerMapSpec[srcReg.flatIndex()].uopAddr, prodTag, prodIdx);
                                        }

                                        connections[flowPathIdx] = InformationFlowPath(registerProducerMapSpec[srcReg.flatIndex()], fullAddr, srcReg.flatIndex(), registerRenameMapSpec[srcReg.flatIndex()]);
                                        connectionsValidSpec[flowPathIdx] = true;
                                        DPRINTF(ConstProp, "Created connection at index %i through register %i (SSA ID %i) from %x.%i to %x.%i\n", flowPathIdx, srcReg.flatIndex(), registerRenameMapSpec[srcReg.flatIndex()], registerProducerMapSpec[srcReg.flatIndex()].pcAddr, registerProducerMapSpec[srcReg.flatIndex()].uopAddr, fullAddr.pcAddr, fullAddr.uopAddr);

                                        // Add the path to this inst's producers
                                        unsigned prodIdx = 0;
                                        bool found = false;
                                        while (prodIdx < 256 && !found) {
                                                if (speculativeDependencyGraph[idx][specway][specuop]->producers[prodIdx] == 0) {
                                                        found = true;
                                                        speculativeDependencyGraph[idx][specway][specuop]->producers[prodIdx] = flowPathIdx;
                                                } else if (speculativeDependencyGraph[idx][specway][specuop]->producers[prodIdx] == flowPathIdx) {
                                                        DPRINTF(ConstProp, "Producer at connections [%i] has already been added???\n", flowPathIdx);
                                                        found = true;
                                                } else {
                                                        prodIdx++;
                                                }
                                        }
                                        if (!found) {
                                                DPRINTF(SuperOp, "Not enough entries in producers for inst %s (%i)\n", uop->getName(), uop->numSrcRegs());
                                        }
                                } else {
                    DPRINTF(SuperOp, "Not enough entries in connections\n");
                }
            } else {
                                // Add dummy producer
                    unsigned prodIdx = 0;
                    bool found = false;
                    while (prodIdx < 256 && !found) {
                        if (speculativeDependencyGraph[idx][specway][specuop]->producers[prodIdx] == 0) {
                            found = true;
                            speculativeDependencyGraph[idx][specway][specuop]->producers[prodIdx] = 5000;
                        } else {
                            prodIdx++;
                        }
                    }
                    if (!found) {
                        DPRINTF(SuperOp, "Not enough entries in producers for inst %s (%i)\n", uop->getName(), uop->numSrcRegs());
                    }
                        }
                }

                // Update information for registers written by this instruction
                if (uop->isControl() && usingControlTracking) {
                        for (int i=0; i<uop->numDestRegs(); i++) {
                                // Note: since control insts lack consumers, this code should never be run
                                RegId destReg = uop->destRegIdx(i);
                                branches[speculativeDependencyGraph[idx][specway][specuop]->consumers[0]].registerProducerMap[destReg.flatIndex()] = fullAddr;
                                branches[speculativeDependencyGraph[idx][specway][specuop]->consumers[0]].registerValidMap[destReg.flatIndex()] = true;
                                branches[speculativeDependencyGraph[idx][specway][specuop]->consumers[0]].registerRenameMap[destReg.flatIndex()] = nextRegNameSpec;
                                nextRegNameSpec++;
                                branches[speculativeDependencyGraph[idx][specway][specuop]->consumers[1]].registerProducerMap[destReg.flatIndex()] = fullAddr;
                                branches[speculativeDependencyGraph[idx][specway][specuop]->consumers[1]].registerValidMap[destReg.flatIndex()] = true;
                                branches[speculativeDependencyGraph[idx][specway][specuop]->consumers[1]].registerRenameMap[destReg.flatIndex()] = nextRegNameSpec;
                                nextRegNameSpec++;
                                markLastUse(destReg.flatIndex());
                        }
                } else {
                for (int i = 0; i < uop->numDestRegs(); i++) {
                        RegId destReg = uop->destRegIdx(i);
                        registerProducerMapSpec[destReg.flatIndex()] = fullAddr;
                        registerValidMapSpec[destReg.flatIndex()] = true;
                        registerRenameMapSpec[destReg.flatIndex()] = nextRegNameSpec;
                        nextRegNameSpec++;
                                markLastUse(destReg.flatIndex());
                        }
                }
        }
        if (uop->isControl()) {
                simplifyGraph();
        }
}

void ArrayDependencyTracker::markLastUse(unsigned regIdx) {
        if (!consumedInWindow[regIdx]) {
                DPRINTF(ConstProp, "Consumer of reg %i not in window\n", regIdx);
                return;
        }
        ArrayDependencyTracker::FullUopAddr consumer = mostRecentConsumer[regIdx];
        int idx = (consumer.pcAddr >> 5) & 0x1f;
    uint64_t tag = (consumer.pcAddr >> 10);
    int way = 0;
        int uop = 0;
        bool foundMatch = false;
    for (int w=0; w < 8; w++) {
        if (decoder->uopValidArray[idx][w] && decoder->uopTagArray[idx][w] == tag && !foundMatch) { // changed to uop
                        assert(decoder->uopCountArray[idx][w] > 0); // changed to uop
            for (int u=0; u < decoder->uopCountArray[idx][w]; u++) { // changed to uop
                                if (microopAddrArray[idx][w][u] == consumer && !foundMatch) { // changed to  uop
                                        way = w;
                                        uop = u;
                                        foundMatch = true;
                        break;
                                }
                        }
        }
    }
        if (!foundMatch) {
                DPRINTF(ConstProp, "Can't mark last use of %i because consumer %x.%i with idx %i and tag %x not found\n", regIdx, consumer.pcAddr, consumer.uopAddr, idx, tag);
                return;
        }
        for (int i=0; i<256; i++) {
                if (speculativeDependencyGraph[idx][way][uop]->producers[i] != 0 && speculativeDependencyGraph[idx][way][uop]->producers[i] != 5000) {
                        if (connectionsValidSpec[speculativeDependencyGraph[idx][way][uop]->producers[i]] && connections[speculativeDependencyGraph[idx][way][uop]->producers[i]].archRegIdx == regIdx) {
                                connections[speculativeDependencyGraph[idx][way][uop]->producers[i]].lastUse = true;
                        }
                }
        }
}

void ArrayDependencyTracker::removeFromGraph(Addr addr, unsigned uopAddr, unsigned cycleRemoved, bool usingUopGraph)
{
    DPRINTF(ConstProp, "Removing entry for %x.%i from (uop? %i)\n", addr, uopAddr, usingUopGraph);
    ArrayDependencyTracker::FullUopAddr fullAddr = ArrayDependencyTracker::FullUopAddr(addr, uopAddr);

        if (!usingUopGraph) {
                for (int i=0; i<256; i++) {
                        if (mostRecentConsumer[i] == fullAddr) {
                                consumedInWindow[i] = false;
                        }
                }
        }

    int idx = (addr >> 5) & 0x1f;
    uint64_t tag = (addr >> 10);

        if (usingUopGraph) {
        int uopway = 0;
                int uopuop = 0;
                bool foundUopMatch = false;
        // Mark IFPs invalid, first microop...
        for (int w=0; w < 8; w++) {
                if (decoder->uopValidArray[idx][w] && decoder->uopTagArray[idx][w] == tag && !foundUopMatch) {
                                assert(decoder->uopCountArray[idx][w] > 0);
                for (int uop = 0; uop < decoder->uopCountArray[idx][w]; uop++) {
                                        if (microopAddrArray[idx][w][uop] == fullAddr && !foundUopMatch) {
                                                // if (decoder->uopAddrArray[idx][w][uop] == addr && !foundUopMatch) {
                                                uopway = w;
                                                uopuop = uop;
                                                foundUopMatch = true;
                                                DPRINTF(ConstProp, "found uop match at way %i uop %i\n", uopway, uopuop);
                                break;
                                        }
                                }
                }
        }
                if (foundUopMatch && microopDependencyGraph[idx][uopway][uopuop]) {
                        DPRINTF(ConstProp, "Removing from uop[%i][%i][%i]\n", idx, uopway, uopuop);
                        evictionsFromUopCache++;
                        totalCyclesInUopCache += (cycleRemoved - microopDependencyGraph[idx][uopway][uopuop]->cycleAdded);
                        removeAtIndex(idx, uopway, uopuop, true);
                } else if (foundUopMatch) {
                        microopDependencyGraph[idx][uopway][uopuop] = new DependGraphEntry();
                }

                // update register rename info
        for (int i=0; i<256; i++) {
                if (registerProducerMapUop[i] == fullAddr) {
                        registerValidMapUop[i] = false;
                }
        }
        } else {
        int specway = 0;
                int specuop = 0;
                bool foundSpecMatch = false;
        for (int w=0; w < 8; w++) {
                if (decoder->uopValidArray[idx][w] && decoder->uopTagArray[idx][w] == tag && !foundSpecMatch) { // changed to uop
                                assert(decoder->uopCountArray[idx][w] > 0); // changed to uop
                for (int uop=0; uop < decoder->uopCountArray[idx][w]; uop++) { // changed to uop
                                        // if (decoder->speculativeAddrArray[idx][w][uop] == addr && !foundSpecMatch) {
                                        if (microopAddrArray[idx][w][uop] == fullAddr && !foundSpecMatch) { // changed to uop
                                                specway = w;
                                                specuop = uop;
                                                foundSpecMatch = true;
                                DPRINTF(ConstProp, "found spec match at way %i uop %i\n", specway, specuop);
                                break;
                                        }
                                }
                }
        }
                if (foundSpecMatch && speculativeDependencyGraph[idx][specway][specuop]) {
                        DPRINTF(ConstProp, "Removing from spec[%i][%i][%i]\n", idx, specway, specuop);
                        evictionsFromSpecCache++;
                        totalCyclesInSpecCache += (cycleRemoved - speculativeDependencyGraph[idx][specway][specuop]->cycleAdded);
                        removeAtIndex(idx, specway, specuop, false);
                } else if (foundSpecMatch) {
                        speculativeDependencyGraph[idx][specway][specuop] = new DependGraphEntry();
                }

                // update register rename info
        for (int i=0; i<256; i++) {
                if (registerProducerMapSpec[i] == fullAddr) {
                        registerValidMapSpec[i] = false;
                }
        }
        }
}

void ArrayDependencyTracker::removeAtIndex(int i1, int i2, int i3, bool removingFromUopGraph) {
        if (removingFromUopGraph && microopDependencyGraph[i1][i2][i3]) {
                DPRINTF(ConstProp, "Removing entry at uop[%i][%i][%i]\n", i1, i2, i2);
                for (int i = 0; i < 256; i++) {
                        unsigned prodIdx = microopDependencyGraph[i1][i2][i3]->producers[i];
                if (prodIdx != 0 && prodIdx != 5000) {
                                invalidateConnection(prodIdx, true);
                                DPRINTF(ConstProp, "Invalidating connection at index %i in uop\n", prodIdx);
                }
                unsigned conIdx = microopDependencyGraph[i1][i2][i3]->consumers[i];
                if (conIdx != 0 && conIdx != 5000) {
                invalidateConnection(conIdx, true);
                                DPRINTF(ConstProp, "Invalidating connection at index %i in uop\n", conIdx);
                }
                        if (registerProducerMapUop[i] == microopDependencyGraph[i1][i2][i3]->thisInst) {
                        registerValidMapUop[i] = false;
                }
        }
                delete microopDependencyGraph[i1][i2][i3];
                microopDependencyGraph[i1][i2][i3] = new DependGraphEntry();
                DPRINTF(ConstProp, "Just created uop[%i][%i][%i]\n", i1, i2, i3);
        } else if (!removingFromUopGraph && speculativeDependencyGraph[i1][i2][i3]) {
                DPRINTF(ConstProp, "Removing entry at spec[%i][%i][%i]\n", i1, i2, i3);
                StaticInstPtr decodedEMI = decoder->decodeInst(decoder->uopCache[i1][i2][i3]);
                if (decodedEMI && decodedEMI->isMacroop()) { decodedEMI = decodedEMI->fetchMicroop(microopAddrArray[i1][i2][i3].uopAddr); }
                for (int i=0; i<256; i++) {
                        if (mostRecentConsumer[i] == speculativeDependencyGraph[i1][i2][i3]->thisInst) {
                                consumedInWindow[i] = false;
                        }
                }
                for (int i = 0; i < 256; i++) {
                unsigned prodIdx = speculativeDependencyGraph[i1][i2][i3]->producers[i];
                if (prodIdx != 0 && prodIdx != 5000) {
                                invalidateConnection(prodIdx, false);
                                DPRINTF(ConstProp, "Invalidating connection at index %i in spec\n", prodIdx);
                }
                        if (decodedEMI->isControl() && usingControlTracking) {
                                unsigned conIdx = speculativeDependencyGraph[i1][i2][i3]->consumers[i];
                                if (conIdx != 0) {
                                        invalidateBranch(conIdx);
                                        DPRINTF(ConstProp, "Invalidating branch at index %i in spec\n", conIdx);
                                        speculativeDependencyGraph[i1][i2][i3]->consumers[i] = 0;
                                }
                        } else {
                        unsigned conIdx = speculativeDependencyGraph[i1][i2][i3]->consumers[i];
                        if (conIdx != 0 && conIdx != 5000) {
                                        invalidateConnection(conIdx, false);
                                        DPRINTF(ConstProp, "Invalidating connection at index %i in spec\n", conIdx);
                        }
                        }
                        if (registerProducerMapSpec[i] == speculativeDependencyGraph[i1][i2][i3]->thisInst) {
                                registerValidMapSpec[i] = false;
                        }
        }
                delete speculativeDependencyGraph[i1][i2][i3];
                speculativeDependencyGraph[i1][i2][i3] = new DependGraphEntry();
                DPRINTF(ConstProp, "Just created spec[%i][%i][%i]\n", i1, i2, i3);
        } else if (removingFromUopGraph) {
                DPRINTF(ConstProp, "Couldn't remove uop[%i][%i][%i] because it doesn't exist\n", i1, i2, i3);
        } else {
                DPRINTF(ConstProp, "Couldn't remove spec[%i][%i][%i] because it doesn't exist\n", i1, i2, i3);
        }
}

void ArrayDependencyTracker::invalidateConnection(unsigned connectionIndex, bool inUopGraph) {
        FullUopAddr prodAddr = connections[connectionIndex].producer;
        FullUopAddr conAddr = connections[connectionIndex].consumer;
        // connectionsValid[connectionIndex] = false;
        int prodIdx = (prodAddr.pcAddr >> 5) & 0x1f;
        uint64_t prodTag = (prodAddr.pcAddr >> 10);
        int conIdx = (conAddr.pcAddr >> 5) & 0x1f;
        uint64_t conTag = (conAddr.pcAddr >> 10);

        if (inUopGraph) {
                // Replace producer with dummy in uop cache
                connectionsValidUop[connectionIndex] = false;
                int prodUopWay = 0;
                int prodUopUop = 0;
                bool foundProdUop = false;
                for (int w=0; w<8; w++) {
                        if (decoder->uopValidArray[prodIdx][w] && decoder->uopTagArray[prodIdx][w] == prodTag && !foundProdUop) {
                                assert(decoder->uopCountArray[prodIdx][w] != 0);
                                for (int u=0; u<decoder->uopCountArray[prodIdx][w]; u++) {
                                        if (microopAddrArray[prodIdx][w][u] == prodAddr && !foundProdUop) {
                                        // if (decoder->uopAddrArray[prodIdx][w][u] == prodAddr.pcAddr && !foundProdUop && microopDependencyGraph[prodIdx][w][u + prodAddr.uopAddr]) {
                                                assert(microopDependencyGraph[prodIdx][w][u]);
                                                prodUopWay = w;
                                                prodUopUop = u;
                                                foundProdUop = true;
                                        }
                                }
                        }
                }

                if (foundProdUop) {
                        DPRINTF(ConstProp, "Found match for producer %x.%i to invalidate at uop[%i][%i][%i]\n", prodAddr.pcAddr, prodAddr.uopAddr, prodIdx, prodUopWay, prodUopUop);
                        for (int i = 0; i<256; i++) {
                                if (microopDependencyGraph[prodIdx][prodUopWay][prodUopUop]->consumers[i] == connectionIndex) {
                                        microopDependencyGraph[prodIdx][prodUopWay][prodUopUop]->consumers[i] = 5000;
                                }
                        }
                } else {
                        DPRINTF(ConstProp, "Failed to find match for producer %x.%i in uop graph\n", prodAddr.pcAddr, prodAddr.uopAddr);
                        DPRINTF(ConstProp, "Hit in uop cache? %i\n", decoder->isHitInUopCache(prodAddr.pcAddr));
                }

                // Replace consumer with dummy in uop cache
                int conUopWay = 0;
                int conUopUop = 0;
                bool foundConUop = false;
                for (int w=0; w<8; w++) {
                        if (decoder->uopValidArray[conIdx][w] && decoder->uopTagArray[conIdx][w] == conTag && !foundConUop) {
                                assert(decoder->uopCountArray[conIdx][w] != 0);
                                for (int u=0; u<decoder->uopCountArray[conIdx][w]; u++) {
                                        // if (decoder->uopAddrArray[conIdx][w][u] == conAddr.pcAddr && !foundConUop && microopDependencyGraph[conIdx][w][u+conAddr.uopAddr]) {
                                        if (microopAddrArray[conIdx][w][u] == conAddr && !foundConUop) {
                                                if (!microopDependencyGraph[conIdx][w][u]) {
                                                        DPRINTF(ConstProp, "Error: entry at uop[%i][%i][%i] matches address %x.%i but does not exist\n", conIdx, w, u, conAddr.pcAddr, conAddr.uopAddr);
                                                }
                                                assert(microopDependencyGraph[conIdx][w][u]);
                                                conUopWay = w;
                                                conUopUop = u;
                                                foundConUop = true;
                                        }
                                }
                        }
                }

                if (foundConUop) {
                        DPRINTF(ConstProp, "Found match for consumer %x.%i to invalidate at uop[%i][%i][%i]\n", conAddr.pcAddr, conAddr.uopAddr, conIdx, conUopWay, conUopUop);
                        for (int i=0; i<256; i++) {
                                if (microopDependencyGraph[conIdx][conUopWay][conUopUop]->producers[i] == connectionIndex) {
                                        microopDependencyGraph[conIdx][conUopWay][conUopUop]->producers[i] = 5000;
                                }
                        }
                } else {
                        DPRINTF(ConstProp, "Failed to find match for consumer %x.%i in uop graph\n", conAddr.pcAddr, conAddr.uopAddr);
                        DPRINTF(ConstProp, "Hit in uop cache? %i\n", decoder->isHitInUopCache(conAddr.pcAddr));
                }
        } else {
                // Replace producer with dummy in spec cache
                connectionsValidSpec[connectionIndex] = false;
                int prodSpecWay = 0;
                int prodSpecUop = 0;
                bool foundProdSpec = false;
                for (int w=0; w<8; w++) {
                        if (decoder->uopValidArray[prodIdx][w] && decoder->uopTagArray[prodIdx][w] == prodTag && !foundProdSpec) { // changed to uop
                                assert(decoder->uopCountArray[prodIdx][w] != 0); // changed to uop
                                for (int u=0; u<decoder->uopCountArray[prodIdx][w]; u++) { // changed to uop
                                        // if (decoder->speculativeAddrArray[prodIdx][w][u] == prodAddr.pcAddr && !foundProdSpec && speculativeDependencyGraph[prodIdx][w][u+prodAddr.uopAddr]) {
                                        if (microopAddrArray[prodIdx][w][u] == prodAddr && !foundProdSpec) { // changed to uop
                                                assert(speculativeDependencyGraph[prodIdx][w][u]);
                                                prodSpecWay = w;
                                                prodSpecUop = u;
                                                foundProdSpec = true;
                                        }
                                }
                        }
                }

                if (foundProdSpec) {
                        DPRINTF(ConstProp, "Found match for producer %x.%i to invalidate at spec[%i][%i][%i]\n", prodAddr.pcAddr, prodAddr.uopAddr, prodIdx, prodSpecWay, prodSpecUop);
                        for (int i=0; i<256; i++) {
                                if (speculativeDependencyGraph[prodIdx][prodSpecWay][prodSpecUop]->consumers[i] == connectionIndex) {
                                        speculativeDependencyGraph[prodIdx][prodSpecWay][prodSpecUop]->consumers[i] = 5000;
                                }
                        }
                } else {
                        DPRINTF(ConstProp, "Failed to find match for producer %x.%i in spec graph\n", prodAddr.pcAddr, prodAddr.uopAddr);
                        DPRINTF(ConstProp, "Hit in microop cache? %i\n", decoder->isHitInUopCache(prodAddr.pcAddr)); // changed to uop
                }

                // Replace consumer with dummy in spec cache
                int conSpecWay = 0;
                int conSpecUop = 0;
                bool foundConSpec = false;
                for (int w=0; w<8; w++) {
                        if (decoder->uopValidArray[conIdx][w] && decoder->uopTagArray[conIdx][w] == conTag && !foundConSpec) { // changed to uop
                                assert(decoder->uopCountArray[conIdx][w] != 0); // changed to uop
                                for (int u=0; u<decoder->uopCountArray[conIdx][w]; u++) { // changed to uop
                                        // if (decoder->speculativeAddrArray[conIdx][w][u] == conAddr.pcAddr && !foundConSpec && speculativeDependencyGraph[conIdx][w][u+conAddr.uopAddr]) {
                                        if (microopAddrArray[conIdx][w][u] == conAddr && !foundConSpec) { // changed to  uop
                                                assert(speculativeDependencyGraph[conIdx][w][u]);
                                                conSpecWay = w;
                                                conSpecUop = u;
                                                foundConSpec = true;
                                        }
                                }
                        }
                }

                if (foundConSpec) {
                        DPRINTF(ConstProp, "Found match for consumer %x.%i to invalidate at spec[%i][%i][%i]\n", conAddr.pcAddr, conAddr.uopAddr, conIdx, conSpecWay, conSpecUop);
                        for (int i=0; i<256; i++) {
                                if (speculativeDependencyGraph[conIdx][conSpecWay][conSpecUop]->producers[i] == connectionIndex) {
                                        speculativeDependencyGraph[conIdx][conSpecWay][conSpecUop]->producers[i] = 5000;
                                }
                        }
                } else {
                        DPRINTF(ConstProp, "Failed to find match for consumer %x.%i in spec graph\n", conAddr.pcAddr, conAddr.uopAddr);
                        DPRINTF(ConstProp, "Hit in microop cache? %i\n", decoder->isHitInUopCache(conAddr.pcAddr)); // changed to uop
                }
        }
}

void ArrayDependencyTracker::invalidateBranch(unsigned branchIndex) {
        for (int i=1; i<4096; i++) {
                if (connections[i].directControlDependency == branchIndex) {
                        connectionsValidSpec[i] = false;
                } else if (connections[i].indirectControlDependency == branchIndex) {
                        connections[i].valid = false;
                }
        }
        branchesValid[branchIndex] = false;
}

void ArrayDependencyTracker::predictValue(Addr addr, unsigned uopAddr, uint64_t value)
{
    DPRINTF(ConstProp, "Predicted %x for inst at %x.%i\n", value, addr, uopAddr);
        int idx = (addr >> 5) & 0x1f;
    uint64_t tag = (addr >> 10);
    int specway = 0;
        int specuop = 0;
        bool foundMatch = false;
    // Mark IFPs ready with value, only spec...
    for (int w=0; w < 8; w++) {
        if (decoder->uopValidArray[idx][w] && decoder->uopTagArray[idx][w] == tag && !foundMatch) { // changed to uop
                        assert(decoder->uopCountArray[idx][w] != 0); // changed to uop
            for (int uop = 0; uop < decoder->uopCountArray[idx][w]; uop++) { // changed to uop
                                // if (decoder->speculativeAddrArray[idx][w][uop] == addr && !foundMatch) {
                                if (microopAddrArray[idx][w][uop] == FullUopAddr(addr, uopAddr) && !foundMatch) { // changed to uop
                                        assert(speculativeDependencyGraph[idx][w][uop]);
                                        specway = w;
                                        specuop = uop;
                        foundMatch = true;
                                }
                        }
        }
    }
        if (!foundMatch) { return; }

        DPRINTF(ConstProp, "Found match for prediction at spec[%i][%i][%i]\n", idx, specway, specuop);
        DPRINTF(ConstProp, "Before prediction:\n");
        describeEntry(idx, specway, specuop);

        speculativeDependencyGraph[idx][specway][specuop]->predicted = true;
        speculativeDependencyGraph[idx][specway][specuop]->value = value;

    for (int i=0; i<256; i++) {
        if (speculativeDependencyGraph[idx][specway][specuop]->consumers[i] != 0 && speculativeDependencyGraph[idx][specway][specuop]->consumers[i] != 5000) {
                        DPRINTF(ConstProp, "Found a consumer at connections[%i] to predict\n", speculativeDependencyGraph[idx][specway][specuop]->consumers[i]);
            connections[speculativeDependencyGraph[idx][specway][specuop]->consumers[i]].value = value;
            connections[speculativeDependencyGraph[idx][specway][specuop]->consumers[i]].valid = true;
                        connections[speculativeDependencyGraph[idx][specway][specuop]->consumers[i]].predict(value);
        }
    }

        DPRINTF(ConstProp, "After prediction:\n");
        describeEntry(idx, specway, specuop);
}

bool ArrayDependencyTracker::simplifyGraph() {
    // Propagate constants
        bool changedGraph = false;
    for (int i1=0; i1<32; i1++) {
        for (int i2=0; i2<8; i2++) {
            for (int i3=0; i3<6; i3++) {
                if (decoder->uopValidArray[i1][i2] && speculativeDependencyGraph[i1][i2][i3]) { // changed to uop
                                        StaticInstPtr decodedEMI = decoder->decodeInst(decoder->uopCache[i1][i2][i3]);
                                        if (decodedEMI->isMacroop()) { decodedEMI = decodedEMI->fetchMicroop(microopAddrArray[i1][i2][i3].uopAddr); }
                    string type = decodedEMI->getName();
                                        if (decodedEMI->isControl()) {
                                                DPRINTF(ConstProp, "Control instruction found, so why no WRIP? Type is %s\n", type);
                                        }
                                        if (type == "mov") {
                                                DPRINTF(ConstProp, "Found a MOV at spec[%i][%i][%i], trying to propagate...\n", i1, i2, i3);
                                                describeEntry(i1, i2, i3);
                                                changedGraph = changedGraph || propagateMov(i1, i2, i3);
                                                DPRINTF(ConstProp, "After:\n");
                                                describeEntry(i1, i2, i3);
                                        } else if (type == "wrip" || type == "wripi") {
                                                DPRINTF(ConstProp, "Found a WRIP/WRIPI branch at spec[%i][%i][%i], trying to propagate across...\n", i1, i2, i3);
                                        //	if (type == "wripi") { printf("WRIPI with immediate %i\n", decodedEMI->getImmediate()); }
                                                describeEntry(i1, i2, i3);
                                                changedGraph = changedGraph || propagateWrip(i1, i2, i3);
                                                DPRINTF(ConstProp, "After:\n");
                                                describeEntry(i1, i2, i3);
                                        } else if (type == "movi") {
                                                DPRINTF(ConstProp, "Found a MOVI at spec[%i][%i][%i], trying to propagate...\n", i1, i2, i3);
                                                describeEntry(i1, i2, i3);
                                                changedGraph = changedGraph || propagateMovI(i1, i2, i3);
                                                DPRINTF(ConstProp, "After:\n");
                                                describeEntry(i1, i2, i3);
                    } else if (type == "and") {
                                                DPRINTF(ConstProp, "Found an AND at spec[%i][%i][%i], trying to propagate...\n", i1, i2, i3);
                                                describeEntry(i1, i2, i3);
                                                changedGraph = changedGraph || propagateAnd(i1, i2, i3);
                                                DPRINTF(ConstProp, "After:\n");
                                                describeEntry(i1, i2, i3);
                                        } else if (type == "add") {
                                                DPRINTF(ConstProp, "Found an ADD at spec[%i][%i][%i], trying to propagate...\n", i1, i2, i3);
                                                describeEntry(i1, i2, i3);
                                                changedGraph = changedGraph || propagateAdd(i1, i2, i3);
                                                DPRINTF(ConstProp, "After:\n");
                                                describeEntry(i1, i2, i3);
                                        } else if (type == "sub") {
                                                DPRINTF(ConstProp, "Found a SUB at spec[%i][%i][%i], trying to propagate...\n", i1, i2, i3);
                                                describeEntry(i1, i2, i3);
                                                changedGraph = changedGraph || propagateSub(i1, i2, i3);
                                                DPRINTF(ConstProp, "After:\n");
                                                describeEntry(i1, i2, i3);
                                        } else if (type == "xor") {
                                                DPRINTF(ConstProp, "Found an XOR at spec[%i][%i][%i], trying to propagate...\n", i1, i2, i3);
                                                describeEntry(i1, i2, i3);
                                                changedGraph = changedGraph || propagateXor(i1, i2, i3);
                                                DPRINTF(ConstProp, "After:\n");
                                                describeEntry(i1, i2, i3);
                                        } else if (type == "or") {
                        DPRINTF(ConstProp, "Found an OR at spec[%i][%i][%i], trying to propagate...\n", i1, i2, i3);
                                                describeEntry(i1, i2, i3);
                                                changedGraph = changedGraph || propagateOr(i1, i2, i3);
                                                DPRINTF(ConstProp, "After:\n");
                                                describeEntry(i1, i2, i3);
                    } else if (type == "subi") {
                                                DPRINTF(ConstProp, "Found a SUBI at spec[%i][%i][%i], trying to propagate...\n", i1, i2, i3);
                                                describeEntry(i1, i2, i3);
                                                changedGraph = changedGraph || propagateSubI(i1, i2, i3);
                                                DPRINTF(ConstProp, "After:\n");
                                                describeEntry(i1, i2, i3);
                                        } else if (type == "addi") {
                                                DPRINTF(ConstProp, "Found an ADDI at spec[%i][%i][%i], trying to propagate...\n", i1, i2, i3);
                                                printf("ADDI with immediate %i\n", decodedEMI->getImmediate());
                                                describeEntry(i1, i2, i3);
                                                changedGraph = changedGraph || propagateAddI(i1, i2, i3);
                                                DPRINTF(ConstProp, "After:\n");
                                                describeEntry(i1, i2, i3);
                                        } else if (type == "slli") {
                                                DPRINTF(ConstProp, "Found a SLLI at spec[%i][%i][%i], trying to propagate...\n", i1, i2, i3);
                                                describeEntry(i1, i2, i3);
                                                changedGraph = changedGraph || propagateSllI(i1, i2, i3);
                                                DPRINTF(ConstProp, "After:\n");
                                                describeEntry(i1, i2, i3);
                                        } else if (type == "srli") {
                                                DPRINTF(ConstProp, "Found a SRLI at spec[%i][%i][%i], trying to propagate...\n", i1, i2, i3);
                                                describeEntry(i1, i2, i3);
                                                changedGraph = changedGraph || propagateSrlI(i1, i2, i3);
                                                DPRINTF(ConstProp, "After:\n");
                                                describeEntry(i1, i2, i3);
                                        } else if (type == "lea") {
                                                DPRINTF(ConstProp, "Type is LEA");
                                                // Requires multiple ALU operations to propagate, not using
                                        } else if (type == "sexti") {
                                                // Implementation has multiple ALU operations, but this is not required by the nature of the operation
                                                DPRINTF(ConstProp, "Found a SEXTI at spec[%i][%i][%i], trying to propagate...\n", i1, i2, i3);
                                                // printf("ADDI with immediate %i\n", decodedEMI->getImmediate());
                                                describeEntry(i1, i2, i3);
                                                changedGraph = changedGraph || propagateSExtI(i1, i2, i3);
                                                DPRINTF(ConstProp, "After:\n");
                                                describeEntry(i1, i2, i3);
                                        } else if (type == "zexti") {
                                                // Implementation has multiple ALU operations, but this is not required by the nature of the operation
                        DPRINTF(ConstProp, "Found a ZEXTI at spec[%i][%i][%i], trying to propagate...\n", i1, i2, i3);
                                                // printf("ADDI with immediate %i\n", decodedEMI->getImmediate());
                                                describeEntry(i1, i2, i3);
                                                changedGraph = changedGraph || propagateZExtI(i1, i2, i3);
                                                DPRINTF(ConstProp, "After:\n");
                                                describeEntry(i1, i2, i3);
                    } else if (type == "mul1s" || type == "mul1u" || type == "mulel" || type == "muleh") {
                                                DPRINTF(ConstProp, "Type is MUL1S, MUL1U, MULEL, or MULEH\n");
                        // TODO: two dest regs with different values? maybe too complex arithmetic?
                    } else if (type == "limm") {
                                                DPRINTF(ConstProp, "Type is LIMM\n");
                                                // printf("LIMM with immediate %i\n", decodedEMI->getImmediate());
                                                // DPRINTF(ConstProp, "Found a LIMM at spec[%i][%i][%i], trying to propagate...\n", i1, i2, i3);
                                                // describeEntry(i1, i2, i3);
                        // changedGraph = changedGraph || propagateLimm(i1, i2, i3);
                                                // DPRINTF(ConstProp, "After:\n");
                                                // describeEntry(i1, i2, i3);
                    } else if (type == "rflags" || type == "wrflags" || type == "ruflags" || type == "wruflags") {
                                                DPRINTF(ConstProp, "Type  is RFLAGS, WRFLAGS, RUFLAGS, or WRUFLAGS\n");
                        // TODO: add control registers to graph?
                    } else if (type == "rdtsc" || type == "rdval") {
                                                DPRINTF(ConstProp, "Type is RDTSC or RDVAL\n");
                        // TODO: determine whether direct register file access needs to be handled differently?
                    } else if (type == "panic" || type == "NOP" || type == "CPUID") {
                                                DPRINTF(ConstProp, "Type is PANIC, NOP, or CPUID\n");
                        // TODO: possibly remove, what is purpose?
                    } else if (type == "st" || type == "stis" || type == "stfp" || type == "ld" || type == "ldis" || type == "ldst" || type == "syscall" || type == "halt" || type == "fault" || type == "call_far_Mp" || "rdip") {
                                                DPRINTF(ConstProp, "Type is ST, STIS, STFP, LD, LDIS, LDST, SYSCALL, HALT, FAULT, CALL_FAR_MP, or RDIP\n");
                        // TODO: cannot remove
                    } else {
                        DPRINTF(ConstProp, "Inst type not covered: %s\n", type);
                    }
                                        // Propagate Last Use
                                        changedGraph = changedGraph || propagateLastUse(i1, i2, i3);
                                        if (changedGraph) { DPRINTF(ConstProp, "CHANGED\n"); describeFullGraph(); }
                }
            }
        }
    }
    return changedGraph;
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
        int uop = 0;
        bool foundMatch = false;
    for (int w=0; w < 8; w++) {
        if (decoder->speculativeValidArray[idx][w] && decoder->speculativeTagArray[idx][w] == tag && !foundMatch) {
                        assert(decoder->speculativeCountArray[idx][w] > 0);
                        for (int u=0; u<decoder->speculativeCountArray[idx][w]; u++) {
                                // if (decoder->speculativeAddrArray[idx][w][u] == addr && !foundMatch) {
                                if (speculativeAddrArray[idx][w][u] == FullUopAddr(addr, uopAddr) && !foundMatch) {
                                        assert(speculativeDependencyGraph[idx][w][u]);
                                        way = w;
                                        uop = u;
                                        foundMatch = true;
                                }
                        }
        }
    }
        if (!foundMatch) { return; }

    // Update total reducable insts
    if (!speculativeDependencyGraph[idx][way][uop]->seen) {
        totalReducable++;
        speculativeDependencyGraph[idx][way][uop]->seen = true;
    }

    // validate consumers to continue chain
    for (int i=0; i<256; i++) {
        if (speculativeDependencyGraph[idx][way][uop]->consumers[i] != 0 && speculativeDependencyGraph[idx][way][uop]->consumers[i] != 5000) {
            connections[speculativeDependencyGraph[idx][way][uop]->consumers[i]].valid = true;
        }
    }

    for (int i=0; i<256; i++) {
        if (speculativeDependencyGraph[idx][way][uop]->consumers[i] != 0 && speculativeDependencyGraph[idx][way][uop]->consumers[i] != 5000) {
            // found a consumer
            FullUopAddr conAddr = connections[speculativeDependencyGraph[idx][way][uop]->consumers[i]].consumer;
            if (std::count(checked.begin(), checked.end(), conAddr) == 0) {
                // recursive call
                measureChain(conAddr, 1, checked);
            }
        }
    }
}

void ArrayDependencyTracker::measureChain(FullUopAddr addr, unsigned recursionLevel, vector<FullUopAddr>& checked)
{
    DPRINTF(ConstProp, "Measuring chain for %x.%i at recursion level %i\n", addr.pcAddr, addr.uopAddr, recursionLevel);
    if (recursionLevel > maxRecursiveDepth) { return; }

        checked.push_back(addr);

    int idx = (addr.pcAddr >> 5) & 0x1f;
    uint64_t tag = (addr.pcAddr >> 10);
    int way = 0;
        int uop = 0;
        bool foundMatch = false;
    for (int w=0; w < 8; w++) {
        if (decoder->speculativeValidArray[idx][w] && decoder->speculativeTagArray[idx][w] == tag && !foundMatch) {
                        assert(decoder->speculativeCountArray[idx][w] > 0);
                        for (int u=0; u<decoder->speculativeCountArray[idx][w]; u++) {
                // if (decoder->speculativeAddrArray[idx][w][u] == addr.pcAddr && !foundMatch) {
                                if (speculativeAddrArray[idx][w][u] == addr && !foundMatch) {
                                        assert(speculativeDependencyGraph[idx][w][u]);
                                        way = w;
                        uop = u;
                                        foundMatch = true;
                                }
                        }
        }
    }
        if (!foundMatch) { return; }

    if (decoder->speculativeCache[idx][way][uop]) {
                if ((decoder->speculativeCache[idx][way][uop]->isControl() || decoder->speculativeCache[idx][way][uop]->isCall() || decoder->speculativeCache[idx][way][uop]->isReturn() || decoder->speculativeCache[idx][way][uop]->isDirectCtrl() || decoder->speculativeCache[idx][way][uop]->isIndirectCtrl() || decoder->speculativeCache[idx][way][uop]->isCondCtrl() || decoder->speculativeCache[idx][way][uop]->isUncondCtrl() || decoder->speculativeCache[idx][way][uop]->isCondDelaySlot())) {
                branchesOnChains++;
                        if (branchPred->getConfidenceForSSO(addr.pcAddr)) {
                        confidentBranchesOnChains++;
                        }
                }
    }

    totalDependentInsts++;
    int allReady = 0;
    for (int i=0; i<256; i++) {
        if (speculativeDependencyGraph[idx][way][uop]->producers[i] != 0 && speculativeDependencyGraph[idx][way][uop]->producers[i] != 5000) {
            if (speculativeDependencyGraph[idx][way][uop]->producers[i] != 5000 && connections[speculativeDependencyGraph[idx][way][uop]->producers[i]].valid) {
                allReady++;
            }
        }
    }
    if (decoder->speculativeCache[idx][way][uop] && allReady == decoder->speculativeCache[idx][way][uop]->numSrcRegs() && allReady != 0) {
        reducableInstCount++;
        if (!speculativeDependencyGraph[idx][way][uop]->seen) {
            totalReducable++;
            speculativeDependencyGraph[idx][way][uop]->seen = true;
        }
        // validate consumers to continue chain
        for (int i=0; i<256; i++) {
            if (speculativeDependencyGraph[idx][way][uop]->consumers[i] != 0 && speculativeDependencyGraph[idx][way][uop]->consumers[i] != 5000) {
                connections[speculativeDependencyGraph[idx][way][uop]->consumers[i]].valid = true;
            }
        }
    }

    for (int i=0; i<256; i++) {
        if (speculativeDependencyGraph[idx][way][uop]->consumers[i] != 0 && speculativeDependencyGraph[idx][way][uop]->consumers[i] != 5000) {
            // Found a consumer
            FullUopAddr conAddr = connections[speculativeDependencyGraph[idx][way][uop]->consumers[i]].consumer;
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
        int uop = 0;
        bool foundMatch = false;
    for (int w = 0; w < 8; w++) {
        if (decoder->uopValidArray[idx][w] && decoder->uopTagArray[idx][w] == tag && !foundMatch) { // changed to uop
                        assert(decoder->uopCountArray[idx][w] > 0); // changed to uop
                        for (int u=0; u<decoder->uopCountArray[idx][w]; u++) { // changed to uop
                                // if (decoder->speculativeAddrArray[idx][w][u] == addr && !foundMatch) {
                                if (microopAddrArray[idx][w][u] == FullUopAddr(addr, uopAddr) && !foundMatch) { // changed to uop
                                        assert(speculativeDependencyGraph[idx][w][u]);
                                        way = w;
                        uop = u;
                                foundMatch = true;
                                }
                        }
        }
    }

    return speculativeDependencyGraph[idx][way][uop] && speculativeDependencyGraph[idx][way][uop]->seen;
}

ArrayDependencyTracker* ArrayDependencyTrackerParams::create() {
    return new ArrayDependencyTracker(this);
}

bool ArrayDependencyTracker::propagateLastUse(int idx, int way, int uop) {
        ArrayDependencyTracker::DependGraphEntry* entry = speculativeDependencyGraph[idx][way][uop];

        // Check that inst is reducable
        int allReady = 0;
    for (int i=0; i<256; i++) {
        if (entry->producers[i] != 0 && entry->producers[i] != 5000) {
            if (connections[entry->producers[i]].valid) {
                allReady++;
            }
        }
    }
        StaticInstPtr decodedEMI = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
        if (decodedEMI && decodedEMI->isMacroop()) { decodedEMI = decodedEMI->fetchMicroop(entry->thisInst.uopAddr); }

    if (allReady == 0 || (decodedEMI && allReady < decodedEMI->numSrcRegs())) { // changed to uop
                return false;
        }

        // Collect registers dead on exit
        vector<unsigned> deadRegs = vector<unsigned>();
        for (int i=0; i<256; i++) {
                if (entry->consumers[i] != 0 && entry->consumers[i] != 5000 && connectionsValidSpec[entry->consumers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath path = connections[entry->consumers[i]];
                        if (path.lastUse) {
                                deadRegs.push_back(path.archRegIdx);
                        }
                }
        }

        // Check that dead values aren't live elsewhere
        for (int i=0; i<256; i++) {
                if (entry->consumers[i] != 0 && entry->consumers[i] != 5000 && connectionsValidSpec[entry->consumers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath path = connections[entry->consumers[i]];
                        vector<unsigned>::iterator loc = std::find(deadRegs.begin(), deadRegs.end(), path.archRegIdx);
                        if (loc != deadRegs.end()) {
                                deadRegs.erase(loc);
                        }
                }
        }
        if (deadRegs.size() == 0) {
                return false; // no dead values found
        }

        // Find producers of these values
        bool foundMatch = false;

        for (int i=0; i<256; i++) {
                if (entry->producers[i] != 0 && entry->producers[i] != 5000 && connectionsValidSpec[entry->producers[i]]) {
                        for (int r=0; r<deadRegs.size(); r++) {
                                if (connections[entry->producers[i]].archRegIdx == deadRegs[r]) {
                                        connections[entry->producers[i]].lastUse = true;
                                        foundMatch = true;
                                }
                        }
                }
        }

        // Graph has changed iff a producer was marked dead
        return foundMatch;
}

bool ArrayDependencyTracker::propagateMov(int idx, int way, int uop) {
        ArrayDependencyTracker::DependGraphEntry* entry = speculativeDependencyGraph[idx][way][uop];
        StaticInstPtr decodedEMI = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
        if (decodedEMI->isMacroop()) { decodedEMI = decodedEMI->fetchMicroop(entry->thisInst.uopAddr); }
        if (decodedEMI->numSrcRegs() > 3) {
                DPRINTF(SuperOp, "Skipping cmov at specCache[%i][%i][%i]\n", idx, way, uop);
                return false;
        }

        unsigned destRegId = decodedEMI->srcRegIdx(0).flatIndex();
        unsigned srcRegId = decodedEMI->srcRegIdx(1).flatIndex();
        uint64_t destVal = 0;
        uint64_t srcVal = 0;
        unsigned valsFound = 0;
        bool foundSource = false;
        for (int i=0; i<256; i++) {
                if (entry->producers[i] != 0 && entry->producers[i] != 5000 && connectionsValidSpec[entry->producers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath path = connections[entry->producers[i]];
                        if (path.archRegIdx == destRegId && path.valid) {
                                destVal = path.value;
                                valsFound++;
                        }
                        if (path.archRegIdx == srcRegId && path.valid) {
                                srcVal = path.value;
                                valsFound++;
                                foundSource = true;
                        }
                }
        }

        uint8_t size = decodedEMI->getDataSize();
        if ((valsFound < 2 && size < 8) || !foundSource) {
                return false;
        }

        // construct the value
        uint64_t forwardVal;
        if (size == 8) {
                forwardVal = srcVal;
        } else if (size == 4) {
                forwardVal = (destVal & 0xffffffff00000000) | (srcVal & 0xffffffff); // mask 4 bytes
        } else if (size == 2) {
                forwardVal = (destVal & 0xffffffffffff0000) | (srcVal & 0xffff); // mask 2 bytes
        } else if (size == 1) {
                forwardVal = (destVal & 0xffffffffffffff00) | (srcVal & 0xff); // mask 1 byte
        } else {
                return false;
        }

        // now pass it along!
        bool foundDest = false;
        for (int i=0; i<256; i++) {
                if (entry->consumers[i] != 0 && entry->consumers[i] != 5000 && connectionsValidSpec[entry->consumers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath& path = connections[entry->consumers[i]];
                        if (path.archRegIdx == destRegId && !path.valid) {
                                path.value = forwardVal;
                                path.valid = true;
                                foundDest = true;
                        }
                }
        }
        return foundDest;
}

bool ArrayDependencyTracker::propagateLimm(int idx, int way, int uop) {
        ArrayDependencyTracker::DependGraphEntry* entry = speculativeDependencyGraph[idx][way][uop];
        StaticInstPtr decodedEMI = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
        if (decodedEMI && decodedEMI->isMacroop()) { decodedEMI = decodedEMI->fetchMicroop(entry->thisInst.uopAddr); }
        if (decodedEMI->numSrcRegs() > 0) {
                DPRINTF(SuperOp, "Skipping limm at specCache[%i][%i][%i] becaause it has %i sources\n", idx, way, uop, decodedEMI->numSrcRegs());
                return false;
        }

        uint64_t forwardVal = decodedEMI->getImmediate();
        unsigned destRegId = decodedEMI->destRegIdx(0).flatIndex();
        DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);

        bool foundDest = false;
        for (int i=0; i<256; i++) {
                if (entry->consumers[i] != 0 && entry->consumers[i] != 5000 && connectionsValidSpec[entry->consumers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath& path = connections[entry->consumers[i]];
                        if (path.archRegIdx == destRegId && !path.valid) {
                                path.value = forwardVal;
                                path.valid = true;
                                foundDest = true;
                        }
                }
        }
        return foundDest;
}

bool ArrayDependencyTracker::propagateAdd(int idx, int way, int uop) {
        // check number of sources
        ArrayDependencyTracker::DependGraphEntry* entry = speculativeDependencyGraph[idx][way][uop];
        StaticInstPtr decodedEMI = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
        if (decodedEMI && decodedEMI->isMacroop()) { decodedEMI = decodedEMI->fetchMicroop(entry->thisInst.uopAddr); }
        if (decodedEMI->numSrcRegs() > 2) {
                DPRINTF(SuperOp, "Skipping add at specCache[%i][%i][%i] becaause it has %i sources\n", idx, way, uop, decodedEMI->numSrcRegs());
                return false;
        }

        // collect sources
        unsigned destRegId = decodedEMI->srcRegIdx(0).flatIndex();
        unsigned srcRegId = decodedEMI->srcRegIdx(1).flatIndex();
        uint64_t destVal = 0;
        uint64_t srcVal = 0;
        unsigned valsFound = 0;
        for (int i=0; i<256; i++) {
                if (entry->producers[i] != 0 && entry->producers[i] != 5000 && connectionsValidSpec[entry->producers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath path = connections[entry->producers[i]];
                        if (path.archRegIdx == destRegId && path.valid) {
                                destVal = path.value;
                                valsFound++;
                        }
                        if (path.archRegIdx == srcRegId && path.valid) {
                                srcVal = path.value;
                                valsFound++;
                        }
                }
        }
        if (valsFound < 2) {
                return false;
        }

        // value construction is easy for this one
        uint64_t forwardVal = destVal + srcVal;

        bool foundDest = false;
        for (int i=0; i<256; i++) {
                if (entry->consumers[i] != 0 && entry->consumers[i] != 5000 && connectionsValidSpec[entry->consumers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath& path = connections[entry->consumers[i]];
                        if (path.archRegIdx == destRegId && !path.valid) {
                                path.value = forwardVal;
                                path.valid = true;
                                foundDest = true;
                        }
                }
        }
        return foundDest;
}

bool ArrayDependencyTracker::propagateSub(int idx, int way, int uop) {
        // check number of sources
        ArrayDependencyTracker::DependGraphEntry* entry = speculativeDependencyGraph[idx][way][uop];
        StaticInstPtr decodedEMI = decoder->decodeInst(decoder->uopCache[idx][way][uop]); // changed to uop
        if (decodedEMI->numSrcRegs() > 2) {
                DPRINTF(SuperOp, "Skipping sub at specCache[%i][%i][%i] becaause it has %i sources\n", idx, way, uop, decodedEMI->numSrcRegs());
                return false;
        }

        // collect sources
        unsigned destRegId = decodedEMI->srcRegIdx(0).flatIndex();
        unsigned srcRegId = decodedEMI->srcRegIdx(1).flatIndex();
        uint64_t destVal = 0;
        uint64_t srcVal = 0;
        unsigned valsFound = 0;
        for (int i=0; i<256; i++) {
                if (entry->producers[i] != 0 && entry->producers[i] != 5000 && connectionsValidSpec[entry->producers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath path = connections[entry->producers[i]];
                        if (path.archRegIdx == destRegId && path.valid) {
                                destVal = path.value;
                                valsFound++;
                        }
                        if (path.archRegIdx == srcRegId && path.valid) {
                                srcVal = path.value;
                                valsFound++;
                        }
                }
        }
        if (valsFound < 2) {
                return false;
        }

        // value construction is easy for this one
        uint64_t forwardVal = destVal - srcVal;

        bool foundDest = false;
        for (int i=0; i<256; i++) {
                if (entry->consumers[i] != 0 && entry->consumers[i] != 5000 && connectionsValidSpec[entry->consumers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath& path = connections[entry->consumers[i]];
                        if (path.archRegIdx == destRegId && !path.valid) {
                                path.value = forwardVal;
                                path.valid = true;
                                foundDest = true;
                        }
                }
        }
        return foundDest;
}

bool ArrayDependencyTracker::propagateAnd(int idx, int way, int uop) {
        // check number of sources
        ArrayDependencyTracker::DependGraphEntry* entry = speculativeDependencyGraph[idx][way][uop];
        StaticInstPtr decodedEMI = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
        if (decodedEMI && decodedEMI->isMacroop()) { decodedEMI = decodedEMI->fetchMicroop(entry->thisInst.uopAddr); }
        if (decodedEMI->numSrcRegs() > 2) {
                DPRINTF(SuperOp, "Skipping and at specCache[%i][%i][%i] becaause it has %i sources\n", idx, way, uop, decodedEMI->numSrcRegs());
                return false;
        }

        // collect sources
        unsigned destRegId = decodedEMI->srcRegIdx(0).flatIndex();
        unsigned srcRegId = decodedEMI->srcRegIdx(1).flatIndex();
        uint64_t destVal = 0;
        uint64_t srcVal = 0;
        unsigned valsFound = 0;
        for (int i=0; i<256; i++) {
                if (entry->producers[i] != 0 && entry->producers[i] != 5000 && connectionsValidSpec[entry->producers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath path = connections[entry->producers[i]];
                        if (path.archRegIdx == destRegId && path.valid) {
                                destVal = path.value;
                                valsFound++;
                        }
                        if (path.archRegIdx == srcRegId && path.valid) {
                                srcVal = path.value;
                                valsFound++;
                        }
                }
        }
        if (valsFound < 2) {
                return false;
        }

        // value construction is easy for this one
        uint64_t forwardVal = destVal & srcVal;

        bool foundDest = false;
        for (int i=0; i<256; i++) {
                if (entry->consumers[i] != 0 && entry->consumers[i] != 5000 && connectionsValidSpec[entry->consumers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath& path = connections[entry->consumers[i]];
                        if (path.archRegIdx == destRegId && !path.valid) {
                                path.value = forwardVal;
                                path.valid = true;
                                foundDest = true;
                        }
                }
        }
        return foundDest;
}

bool ArrayDependencyTracker::propagateOr(int idx, int way, int uop) {
        // check number of sources
        ArrayDependencyTracker::DependGraphEntry* entry = speculativeDependencyGraph[idx][way][uop];
        StaticInstPtr decodedEMI = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
        if (decodedEMI && decodedEMI->isMacroop()) { decodedEMI = decodedEMI->fetchMicroop(entry->thisInst.uopAddr); }
        if (decodedEMI->numSrcRegs() > 2) {
                DPRINTF(SuperOp, "Skipping or at specCache[%i][%i][%i] becaause it has %i sources\n", idx, way, uop, decodedEMI->numSrcRegs());
                return false;
        }

        // collect sources
        unsigned destRegId = decodedEMI->srcRegIdx(0).flatIndex();
        unsigned srcRegId = decodedEMI->srcRegIdx(1).flatIndex();
        uint64_t destVal = 0;
        uint64_t srcVal = 0;
        unsigned valsFound = 0;
        for (int i=0; i<256; i++) {
                if (entry->producers[i] != 0 && entry->producers[i] != 5000 && connectionsValidSpec[entry->producers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath path = connections[entry->producers[i]];
                        if (path.archRegIdx == destRegId && path.valid) {
                                destVal = path.value;
                                valsFound++;
                        }
                        if (path.archRegIdx == srcRegId && path.valid) {
                                srcVal = path.value;
                                valsFound++;
                        }
                }
        }
        if (valsFound < 2) {
                return false;
        }

        // value construction is easy for this one
        uint64_t forwardVal = destVal | srcVal;

        bool foundDest = false;
        for (int i=0; i<256; i++) {
                if (entry->consumers[i] != 0 && entry->consumers[i] != 5000 && connectionsValidSpec[entry->consumers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath& path = connections[entry->consumers[i]];
                        if (path.archRegIdx == destRegId && !path.valid) {
                                path.value = forwardVal;
                                path.valid = true;
                                foundDest = true;
                        }
                }
        }
        return foundDest;
}

bool ArrayDependencyTracker::propagateXor(int idx, int way, int uop) {
        // check number of sources
        ArrayDependencyTracker::DependGraphEntry* entry = speculativeDependencyGraph[idx][way][uop];
        StaticInstPtr decodedEMI = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
        if (decodedEMI && decodedEMI->isMacroop()) { decodedEMI = decodedEMI->fetchMicroop(entry->thisInst.uopAddr); }
        if (decodedEMI->numSrcRegs() > 2) {
                DPRINTF(SuperOp, "Skipping xor at specCache[%i][%i][%i] becaause it has %i sources\n", idx, way, uop, decodedEMI->numSrcRegs());
                return false;
        }

        // collect sources
        unsigned destRegId = decodedEMI->srcRegIdx(0).flatIndex();
        unsigned srcRegId = decodedEMI->srcRegIdx(1).flatIndex();
        uint64_t destVal = 0;
        uint64_t srcVal = 0;
        unsigned valsFound = 0;
        for (int i=0; i<256; i++) {
                if (entry->producers[i] != 0 && entry->producers[i] != 5000 && connectionsValidSpec[entry->producers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath path = connections[entry->producers[i]];
                        if (path.archRegIdx == destRegId && path.valid) {
                                destVal = path.value;
                                valsFound++;
                        }
                        if (path.archRegIdx == srcRegId && path.valid) {
                                srcVal = path.value;
                                valsFound++;
                        }
                }
        }
        if (valsFound < 2) {
                return false;
        }

        // value construction is easy for this one
        uint64_t forwardVal = destVal ^ srcVal;

        bool foundDest = false;
        for (int i=0; i<256; i++) {
                if (entry->consumers[i] != 0 && entry->consumers[i] != 5000 && connectionsValidSpec[entry->consumers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath& path = connections[entry->consumers[i]];
                        if (path.archRegIdx == destRegId && !path.valid) {
                                path.value = forwardVal;
                                path.valid = true;
                                foundDest = true;
                        }
                }
        }
        return foundDest;
}

bool ArrayDependencyTracker::propagateMovI(int idx, int way, int uop) {
        // check number of sources
        ArrayDependencyTracker::DependGraphEntry* entry = speculativeDependencyGraph[idx][way][uop];
        StaticInstPtr decodedEMI = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
        if (decodedEMI && decodedEMI->isMacroop()) { decodedEMI = decodedEMI->fetchMicroop(entry->thisInst.uopAddr); }
        if (decodedEMI->numSrcRegs() > 0) {
                DPRINTF(SuperOp, "Skipping movi at specCache[%i][%i][%i] becaause it has %i sources\n", idx, way, uop, decodedEMI->numSrcRegs());
                return false;
        }

        // collect value
        uint64_t forwardVal = decodedEMI->getImmediate();
        unsigned destRegId = decodedEMI->destRegIdx(0).flatIndex();
        DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);

        // forward to dest
        bool foundDest = false;
        for (int i=0; i<256; i++) {
                if (entry->consumers[i] != 0 && entry->consumers[i] != 5000 && connectionsValidSpec[entry->consumers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath& path = connections[entry->consumers[i]];
                        if (path.archRegIdx == destRegId && !path.valid) {
                                path.value = forwardVal;
                                path.valid = true;
                                foundDest = true;
                        }
                }
        }
        return foundDest;
}

bool ArrayDependencyTracker::propagateSubI(int idx, int way, int uop) {
        // check number of sources
        ArrayDependencyTracker::DependGraphEntry* entry = speculativeDependencyGraph[idx][way][uop];
        StaticInstPtr decodedEMI = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
        if (decodedEMI && decodedEMI->isMacroop()) { decodedEMI = decodedEMI->fetchMicroop(entry->thisInst.uopAddr); }
        if (decodedEMI->numSrcRegs() > 1) {
                DPRINTF(SuperOp, "Skipping subi at specCache[%i][%i][%i] becaause it has %i sources\n", idx, way, uop, decodedEMI->numSrcRegs());
                return false;
        }

        // collect sources
        unsigned destRegId = decodedEMI->srcRegIdx(0).flatIndex();
        uint64_t destVal = 0;
        unsigned valsFound = 0;
        for (int i=0; i<256; i++) {
                if (entry->producers[i] != 0 && entry->producers[i] != 5000 && connectionsValidSpec[entry->producers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath path = connections[entry->producers[i]];
                        if (path.archRegIdx == destRegId && path.valid) {
                                destVal = path.value;
                                valsFound++;
                        }
                }
        }
        if (valsFound < 1) {
                return false;
        }

        // value construction is easy for this one
        uint64_t forwardVal = destVal - decodedEMI->getImmediate();

        bool foundDest = false;
        for (int i=0; i<256; i++) {
                if (entry->consumers[i] != 0 && entry->consumers[i] != 5000 && connectionsValidSpec[entry->consumers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath& path = connections[entry->consumers[i]];
                        if (path.archRegIdx == destRegId && !path.valid) {
                                path.value = forwardVal;
                                path.valid = true;
                                foundDest = true;
                        }
                }
        }
        return foundDest;
}

bool ArrayDependencyTracker::propagateAddI(int idx, int way, int uop) {
        // check number of sources
        ArrayDependencyTracker::DependGraphEntry* entry = speculativeDependencyGraph[idx][way][uop];
        StaticInstPtr decodedEMI = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
        if (decodedEMI && decodedEMI->isMacroop()) { decodedEMI = decodedEMI->fetchMicroop(entry->thisInst.uopAddr); }
        if (decodedEMI->numSrcRegs() > 1) {
                DPRINTF(SuperOp, "Skipping addi at specCache[%i][%i][%i] becaause it has %i sources\n", idx, way, uop, decodedEMI->numSrcRegs());
                return false;
        }

        // collect sources
        unsigned destRegId = decodedEMI->srcRegIdx(0).flatIndex();
        uint64_t destVal = 0;
        unsigned valsFound = 0;
        for (int i=0; i<256; i++) {
                if (entry->producers[i] != 0 && entry->producers[i] != 5000 && connectionsValidSpec[entry->producers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath path = connections[entry->producers[i]];
                        if (path.archRegIdx == destRegId && path.valid) {
                                destVal = path.value;
                                valsFound++;
                        }
                }
        }
        if (valsFound < 1) {
                return false;
        }

        // value construction is easy for this one
        uint64_t forwardVal = destVal + decodedEMI->getImmediate();

        bool foundDest = false;
        for (int i=0; i<256; i++) {
                if (entry->consumers[i] != 0 && entry->consumers[i] != 5000 && connectionsValidSpec[entry->consumers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath& path = connections[entry->consumers[i]];
                        if (path.archRegIdx == destRegId && !path.valid) {
                                path.value = forwardVal;
                                path.valid = true;
                                foundDest = true;
                        }
                }
        }
        return foundDest;
}

bool ArrayDependencyTracker::propagateAndI(int idx, int way, int uop) {
        // check number of sources
        ArrayDependencyTracker::DependGraphEntry* entry = speculativeDependencyGraph[idx][way][uop];
        StaticInstPtr decodedEMI = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
        if (decodedEMI && decodedEMI->isMacroop()) { decodedEMI = decodedEMI->fetchMicroop(entry->thisInst.uopAddr); }
        if (decodedEMI->numSrcRegs() > 1) {
                DPRINTF(SuperOp, "Skipping andi at specCache[%i][%i][%i] becaause it has %i sources\n", idx, way, uop, decodedEMI->numSrcRegs());
                return false;
        }

        // collect sources
        unsigned destRegId = decodedEMI->srcRegIdx(0).flatIndex();
        uint64_t destVal = 0;
        unsigned valsFound = 0;
        for (int i=0; i<256; i++) {
                if (entry->producers[i] != 0 && entry->producers[i] != 5000 && connectionsValidSpec[entry->producers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath path = connections[entry->producers[i]];
                        if (path.archRegIdx == destRegId && path.valid) {
                                destVal = path.value;
                                valsFound++;
                        }
                }
        }
        if (valsFound < 1) {
                return false;
        }

        // value construction is easy for this one
        uint64_t forwardVal = destVal & decodedEMI->getImmediate();

        bool foundDest = false;
        for (int i=0; i<256; i++) {
                if (entry->consumers[i] != 0 && entry->consumers[i] != 5000 && connectionsValidSpec[entry->consumers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath& path = connections[entry->consumers[i]];
                        if (path.archRegIdx == destRegId && !path.valid) {
                                path.value = forwardVal;
                                path.valid = true;
                                foundDest = true;
                        }
                }
        }
        return foundDest;
}

bool ArrayDependencyTracker::propagateOrI(int idx, int way, int uop) {
        // check number of sources
        ArrayDependencyTracker::DependGraphEntry* entry = speculativeDependencyGraph[idx][way][uop];
        StaticInstPtr decodedEMI = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
        if (decodedEMI && decodedEMI->isMacroop()) { decodedEMI = decodedEMI->fetchMicroop(entry->thisInst.uopAddr); }
        if (decodedEMI->numSrcRegs() > 1) {
                DPRINTF(SuperOp, "Skipping ori at specCache[%i][%i][%i] becaause it has %i sources\n", idx, way, uop, decodedEMI->numSrcRegs());
                return false;
        }

        // collect sources
        unsigned destRegId = decodedEMI->srcRegIdx(0).flatIndex();
        uint64_t destVal = 0;
        unsigned valsFound = 0;
        for (int i=0; i<256; i++) {
                if (entry->producers[i] != 0 && entry->producers[i] != 5000 && connectionsValidSpec[entry->producers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath path = connections[entry->producers[i]];
                        if (path.archRegIdx == destRegId && path.valid) {
                                destVal = path.value;
                                valsFound++;
                        }
                }
        }
        if (valsFound < 1) {
                return false;
        }

        // value construction is easy for this one
        uint64_t forwardVal = destVal | decodedEMI->getImmediate();

        bool foundDest = false;
        for (int i=0; i<256; i++) {
                if (entry->consumers[i] != 0 && entry->consumers[i] != 5000 && connectionsValidSpec[entry->consumers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath& path = connections[entry->consumers[i]];
                        if (path.archRegIdx == destRegId && !path.valid) {
                                path.value = forwardVal;
                                path.valid = true;
                                foundDest = true;
                        }
                }
        }
        return foundDest;
}

bool ArrayDependencyTracker::propagateXorI(int idx, int way, int uop) {
        // check number of sources
        ArrayDependencyTracker::DependGraphEntry* entry = speculativeDependencyGraph[idx][way][uop];
        StaticInstPtr decodedEMI = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
        if (decodedEMI && decodedEMI->isMacroop()) { decodedEMI = decodedEMI->fetchMicroop(entry->thisInst.uopAddr); }
        if (decodedEMI->numSrcRegs() > 1) {
                DPRINTF(SuperOp, "Skipping xori at specCache[%i][%i][%i] becaause it has %i sources\n", idx, way, uop, decodedEMI->numSrcRegs());
                return false;
        }

        // collect sources
        unsigned destRegId = decodedEMI->srcRegIdx(0).flatIndex();
        uint64_t destVal = 0;
        unsigned valsFound = 0;
        for (int i=0; i<256; i++) {
                if (entry->producers[i] != 0 && entry->producers[i] != 5000 && connectionsValidSpec[entry->producers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath path = connections[entry->producers[i]];
                        if (path.archRegIdx == destRegId && path.valid) {
                                destVal = path.value;
                                valsFound++;
                        }
                }
        }
        if (valsFound < 1) {
                return false;
        }

        // value construction is easy for this one
        uint64_t forwardVal = destVal ^ decodedEMI->getImmediate();

        bool foundDest = false;
        for (int i=0; i<256; i++) {
                if (entry->consumers[i] != 0 && entry->consumers[i] != 5000 && connectionsValidSpec[entry->consumers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath& path = connections[entry->consumers[i]];
                        if (path.archRegIdx == destRegId && !path.valid) {
                                path.value = forwardVal;
                                path.valid = true;
                                foundDest = true;
                        }
                }
        }
        return foundDest;
}

bool ArrayDependencyTracker::propagateSllI(int idx, int way, int uop) {
        // check number of sources
        ArrayDependencyTracker::DependGraphEntry* entry = speculativeDependencyGraph[idx][way][uop];
        StaticInstPtr decodedEMI = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
        if (decodedEMI && decodedEMI->isMacroop()) { decodedEMI = decodedEMI->fetchMicroop(entry->thisInst.uopAddr); }
        if (decodedEMI->numSrcRegs() > 1) {
                DPRINTF(SuperOp, "Skipping slli at specCache[%i][%i][%i] becaause it has %i sources\n", idx, way, uop, decodedEMI->numSrcRegs());
                return false;
        }

        // collect sources
        unsigned destRegId = decodedEMI->srcRegIdx(0).flatIndex();
        uint64_t destVal = 0;
        unsigned valsFound = 0;
        for (int i=0; i<256; i++) {
                if (entry->producers[i] != 0 && entry->producers[i] != 5000 && connectionsValidSpec[entry->producers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath path = connections[entry->producers[i]];
                        if (path.archRegIdx == destRegId && path.valid) {
                                destVal = path.value;
                                valsFound++;
                        }
                }
        }
        if (valsFound < 1) {
                return false;
        }

        // value construction is easy for this one
        uint64_t forwardVal = destVal << decodedEMI->getImmediate();

        bool foundDest = false;
        for (int i=0; i<256; i++) {
                if (entry->consumers[i] != 0 && entry->consumers[i] != 5000 && connectionsValidSpec[entry->consumers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath& path = connections[entry->consumers[i]];
                        if (path.archRegIdx == destRegId && !path.valid) {
                                path.value = forwardVal;
                                path.valid = true;
                                foundDest = true;
                        }
                }
        }
        return foundDest;
}

bool ArrayDependencyTracker::propagateSrlI(int idx, int way, int uop) {
        // check number of sources
        ArrayDependencyTracker::DependGraphEntry* entry = speculativeDependencyGraph[idx][way][uop];
        StaticInstPtr decodedEMI = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
        if (decodedEMI && decodedEMI->isMacroop()) { decodedEMI = decodedEMI->fetchMicroop(entry->thisInst.uopAddr); }
        if (decodedEMI->numSrcRegs() > 1) {
                DPRINTF(SuperOp, "Skipping srli at specCache[%i][%i][%i] becaause it has %i sources\n", idx, way, uop, decodedEMI->numSrcRegs());
                return false;
        }

        // collect sources
        unsigned destRegId = decodedEMI->srcRegIdx(0).flatIndex();
        uint64_t destVal = 0;
        unsigned valsFound = 0;
        for (int i=0; i<256; i++) {
                if (entry->producers[i] != 0 && entry->producers[i] != 5000 && connectionsValidSpec[entry->producers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath path = connections[entry->producers[i]];
                        if (path.archRegIdx == destRegId && path.valid) {
                                destVal = path.value;
                                valsFound++;
                        }
                }
        }
        if (valsFound < 1) {
                return false;
        }

        // value construction is easy for this one
        uint64_t forwardVal = destVal >> decodedEMI->getImmediate();

        bool foundDest = false;
        for (int i=0; i<256; i++) {
                if (entry->consumers[i] != 0 && entry->consumers[i] != 5000 && connectionsValidSpec[entry->consumers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath& path = connections[entry->consumers[i]];
                        if (path.archRegIdx == destRegId && !path.valid) {
                                path.value = forwardVal;
                                path.valid = true;
                                foundDest = true;
                        }
                }
        }
        return foundDest;
}

bool ArrayDependencyTracker::propagateSExtI(int idx, int way, int uop) {
        // check number of sources
        ArrayDependencyTracker::DependGraphEntry* entry = speculativeDependencyGraph[idx][way][uop];
        StaticInstPtr decodedEMI = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
        if (decodedEMI && decodedEMI->isMacroop()) { decodedEMI = decodedEMI->fetchMicroop(entry->thisInst.uopAddr); }
        if (decodedEMI->numSrcRegs() > 1) {
                DPRINTF(SuperOp, "Skipping sexti at specCache[%i][%i][%i] becaause it has %i sources\n", idx, way, uop, decodedEMI->numSrcRegs());
                return false;
        }

        // collect sources
        unsigned srcRegId = decodedEMI->srcRegIdx(0).flatIndex();
        uint64_t srcVal = 0;
        unsigned destRegId = decodedEMI->destRegIdx(0).flatIndex();
        unsigned valsFound = 0;
        for (int i=0; i<256; i++) {
                if (entry->producers[i] != 0 && entry->producers[i] != 5000 && connectionsValidSpec[entry->producers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath path = connections[entry->producers[i]];
                        if (path.archRegIdx == srcRegId && path.valid) {
                                srcVal = path.value;
                                valsFound++;
                        }
                }
        }
        if (valsFound < 1) {
                return false;
        }

        // value construction taken from isa file
        DPRINTF(ConstProp, "Using immediate %i in a sexti as bit to extend, not value to be extended\n", decodedEMI->getImmediate());
        int signBit = bits(srcVal, decodedEMI->getImmediate(), decodedEMI->getImmediate());
        uint64_t signMask = mask(decodedEMI->getImmediate());
        uint64_t forwardVal = signBit ? (srcVal | ~signMask) : (srcVal & signMask);
        DPRINTF(ConstProp, "Produced value %x from sign bit %i and mask %x\n", forwardVal, signBit, signMask);

        bool foundDest = false;
        for (int i=0; i<256; i++) {
                if (entry->consumers[i] != 0 && entry->consumers[i] != 5000 && connectionsValidSpec[entry->consumers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath& path = connections[entry->consumers[i]];
                        if (path.archRegIdx == destRegId && !path.valid) {
                                path.value = forwardVal;
                                path.valid = true;
                                foundDest = true;
                        }
                }
        }
        return foundDest;
}

bool ArrayDependencyTracker::propagateZExtI(int idx, int way, int uop) {
        // check number of sources
        ArrayDependencyTracker::DependGraphEntry* entry = speculativeDependencyGraph[idx][way][uop];
        StaticInstPtr decodedEMI = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
        if (decodedEMI && decodedEMI->isMacroop()) { decodedEMI = decodedEMI->fetchMicroop(entry->thisInst.uopAddr); }
        if (decodedEMI->numSrcRegs() > 1) {
                DPRINTF(SuperOp, "Skipping zexti at specCache[%i][%i][%i] becaause it has %i sources\n", idx, way, uop, decodedEMI->numSrcRegs());
                return false;
        }

        // collect sources
        unsigned srcRegId = decodedEMI->srcRegIdx(0).flatIndex();
        uint64_t srcVal = 0;
        unsigned destRegId = decodedEMI->destRegIdx(0).flatIndex();
        unsigned valsFound = 0;
        for (int i=0; i<256; i++) {
                if (entry->producers[i] != 0 && entry->producers[i] != 5000 && connectionsValidSpec[entry->producers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath path = connections[entry->producers[i]];
                        if (path.archRegIdx == srcRegId && path.valid) {
                                srcVal = path.value;
                                valsFound++;
                        }
                }
        }
        if (valsFound < 1) {
                return false;
        }

        // value construction taken from isa file
        DPRINTF(ConstProp, "Using immediate %i in a zexti as bit to extend, not value to be extended\n", decodedEMI->getImmediate());
        uint64_t forwardVal = bits(srcVal, decodedEMI->getImmediate(), 0);
        DPRINTF(ConstProp, "Extracted bits 0 to %i of %x for value %x\n", decodedEMI->getImmediate(), srcVal, forwardVal);

        bool foundDest = false;
        for (int i=0; i<256; i++) {
                if (entry->consumers[i] != 0 && entry->consumers[i] != 5000 && connectionsValidSpec[entry->consumers[i]]) {
                        ArrayDependencyTracker::InformationFlowPath& path = connections[entry->consumers[i]];
                        if (path.archRegIdx == destRegId && !path.valid) {
                                path.value = forwardVal;
                                path.valid = true;
                                foundDest = true;
                        }
                }
        }
        return foundDest;
}

bool ArrayDependencyTracker::propagateWrip(int idx, int way, int uop) {
        bool changedEither = false;

        // Not taken
        unsigned notTakenIndex = speculativeDependencyGraph[idx][way][uop]->consumers[1];
        if (notTakenIndex != 0) {
                assert(branches[notTakenIndex].targetValid);
                changedEither = propagateAcrossControlDependency(notTakenIndex, branches[notTakenIndex].propagatingTo);
        }

        // Taken
        unsigned takenIndex = speculativeDependencyGraph[idx][way][uop]->consumers[0];
        if (takenIndex != 0) {
                if (branches[takenIndex].targetValid) {
                        changedEither = propagateAcrossControlDependency(takenIndex, branches[takenIndex].propagatingTo);
                } else {
                        // Try to resolve target here
                }
        }
        return changedEither;
}

bool ArrayDependencyTracker::propagateAcrossControlDependency(unsigned branchIndex, FullUopAddr propagatingTo) {
        int idx = (propagatingTo.pcAddr >> 5) & 0x1f;
    uint64_t tag = (propagatingTo.pcAddr >> 10);
        int way = 0;
        int uop = 0;
        bool found = false;
        for (int w = 0; w < 8; w++) {
        if (decoder->uopValidArray[idx][w] && decoder->uopTagArray[idx][w] == tag && !found) { // changed to uop
            assert(decoder->uopCountArray[idx][w] > 0); // changed to uop
                        for (int u=0; u < decoder->uopCountArray[idx][w]; u++) { // changed to uop
                                if (microopAddrArray[idx][w][u] == propagatingTo && !found) { // changed to uop
                                        assert(speculativeDependencyGraph[idx][w][u]);
                                        way = w;
                                        uop = uop;
                                        found = true;
                                }
                        }
        }
        }

        if (!found) { return false; }

        ArrayDependencyTracker::DependGraphEntry* entry = speculativeDependencyGraph[idx][way][uop];
        StaticInstPtr decodedEMI = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
        if (decodedEMI && decodedEMI->isMacroop()) { decodedEMI = decodedEMI->fetchMicroop(entry->thisInst.uopAddr); }

        if (decodedEMI->isControl()) { return false; }

        bool changeMade = false;

        for (int i=0; i<decodedEMI->numSrcRegs(); i++) {
        RegId srcReg = decodedEMI->srcRegIdx(i);
                if (branches[branchIndex].registerValidMap[srcReg.flatIndex()]) {
                        // Check whether value is already produced along path
                        bool alreadyProduced = false;
                        for (int i=1; i<256; i++) {
                                if (entry->producers[i] != 0 && connectionsValidSpec[entry->producers[i]] && connections[entry->producers[i]].archRegIdx == srcReg.flatIndex() && (connections[entry->producers[i]].directControlDependency == branchIndex || connections[entry->producers[i]].indirectControlDependency == branchIndex)) {
                                        alreadyProduced = true;
                                }
                        }

                        if (!alreadyProduced) {
                                // Confirm that producer is still in graph
                                FullUopAddr prodAddr = branches[branchIndex].registerProducerMap[srcReg.flatIndex()];

                                int prodIdx = (prodAddr.pcAddr >> 5) & 0x1f;
                                uint64_t prodTag = (prodAddr.pcAddr >> 10);
                                int prodWay = 0;
                                int prodUop = 0;
                                bool foundProd = false;
                                for (int w=0; w<8; w++) {
                                        if (decoder->uopValidArray[prodIdx][w] && decoder->uopTagArray[prodIdx][w] == prodTag && !foundProd) {
                                                assert(decoder->uopCountArray[prodIdx][w] > 0);
                                                for (int u=0; u<decoder->uopCountArray[prodIdx][w]; u++) {
                                                        if (microopAddrArray[prodIdx][w][u] == prodAddr && !foundProd) {
                                                                assert(speculativeDependencyGraph[prodIdx][w][u]);
                                                                prodWay = w;
                                                                prodUop = u;
                                                                foundProd = true;
                                                        }
                                                }
                                        }
                                }

                                if (foundProd) {
                                        // Get index to add connection at
                                        unsigned connectionIndex = 0;
                                        bool connectionFound = false;
                                        for (int c=1; c<4096; c++) {
                                                if (!connectionsValidSpec[c] && !connectionFound) {
                                                        connectionIndex = c;
                                                        connectionFound = true;
                                                }
                                        }
                                        if (connectionFound) {
                                                changeMade = true;
                                                connections[connectionIndex].producer = prodAddr;
                                                connections[connectionIndex].consumer = propagatingTo;
                                                connections[connectionIndex].archRegIdx = srcReg.flatIndex();
                                        connections[connectionIndex].renamedRegIdx = branches[branchIndex].registerRenameMap[srcReg.flatIndex()];
                                        connections[connectionIndex].value = 0;
                                        connections[connectionIndex].valid = false;
                                        connections[connectionIndex].lastUse = false;
                                                connections[connectionIndex].directControlDependency = branchIndex;
                                                connections[connectionIndex].indirectControlDependency = 0;
                                                connectionsValidSpec[connectionIndex] = true;
                                                bool addedAsProducer = false;
                                                bool addedAsConsumer = false;
                                                for (int j=0; j<256; j++) {
                                                        if (speculativeDependencyGraph[idx][way][uop]->producers[i] == 0 && !addedAsProducer) {
                                                                speculativeDependencyGraph[idx][way][uop]->producers[i] = connectionIndex;
                                                                addedAsProducer = true;
                                                        }
                                                        if (speculativeDependencyGraph[prodIdx][prodWay][prodUop]->consumers[i] == 0 && !addedAsConsumer) {
                                                                speculativeDependencyGraph[prodIdx][prodWay][prodUop]->consumers[i] = connectionIndex;
                                                                addedAsConsumer = true;
                                                        }
                                                }
                                        } else {
                                                DPRINTF(ConstProp, "Not enough entries in connections! Can't add new branch-crossing connection\n");
                                        }
                                }
                        }
                }
        }

        // TODO: increment propagatingTo
        FullUopAddr nextFullAddr;
        if ((!decodedEMI->isMacroop()) || propagatingTo.uopAddr < decodedEMI->getNumMicroops()) {
                Addr nextPc = propagatingTo.pcAddr + decodedEMI->machInst.instSize;
                nextFullAddr = FullUopAddr(nextPc, 0);
        } else {
                nextFullAddr = FullUopAddr(propagatingTo.pcAddr, propagatingTo.uopAddr + 1);
        }
        branches[branchIndex].propagatingTo = nextFullAddr;

        return changeMade;
}

void ArrayDependencyTracker::describeEntry(int idx, int way, int uop) {
        ArrayDependencyTracker::DependGraphEntry* entry = speculativeDependencyGraph[idx][way][uop];
        StaticInstPtr decodedEMI = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
        if (decodedEMI && decodedEMI->isMacroop()) { decodedEMI = decodedEMI->fetchMicroop(entry->thisInst.uopAddr); }
        DPRINTF(ConstProp, "Entry for inst %x.%i\n", entry->thisInst.pcAddr, entry->thisInst.uopAddr);
        DPRINTF(ConstProp, "Disassembly: %s\n", decodedEMI->disassemble(entry->thisInst.pcAddr));
        DPRINTF(ConstProp, "SrcRegs (%i):\n", decodedEMI->numSrcRegs());
        for (int i=0; i<decodedEMI->numSrcRegs(); i++) {
                RegId srcReg = decodedEMI->srcRegIdx(i);
                DPRINTF(ConstProp, "Register %i\n", srcReg.flatIndex());
        }
        DPRINTF(ConstProp, "Dest Regs (%i):\n", decodedEMI->numDestRegs());
        for (int i=0; i<decodedEMI->numDestRegs(); i++) {
                RegId destReg = decodedEMI->destRegIdx(i);
                DPRINTF(ConstProp, "Register %i\n", destReg.flatIndex());
        }
        DPRINTF(ConstProp, "Incoming data paths:\n");
        for (int i=0; i<256; i++) {
                if (entry->producers[i] == 5000) {
                        DPRINTF(ConstProp, "Source not ready\n");
                } else if (entry->producers[i] != 0) {
                        if (connectionsValidSpec[entry->producers[i]]) {
                                ArrayDependencyTracker::InformationFlowPath path = connections[entry->producers[i]];
                                DPRINTF(ConstProp, "Connections[%i]: Register %i (SSA ID %i) from %x.%i to %x.%i with value %x (%i) along control path branches[%i] - lastUse? %i\n", entry->producers[i], path.archRegIdx, path.renamedRegIdx, path.producer.pcAddr, path.producer.uopAddr, path.consumer.pcAddr, path.consumer.uopAddr, path.value, path.valid, path.directControlDependency, path.lastUse);
                        } else {
                                DPRINTF(ConstProp, "Connection through %i has been invalidated\n", entry->producers[i]);
                        }
                }
        }
        DPRINTF(ConstProp, "Outgoing data paths:\n");
        for (int i=0; i<256; i++) {
                if (entry->consumers[i] == 5000) {
                        DPRINTF(ConstProp, "Dest not ready\n");
                } else if (entry->consumers[i] != 0) {
                        if (connectionsValidSpec[entry->consumers[i]]) {
                                ArrayDependencyTracker::InformationFlowPath path = connections[entry->consumers[i]];
                                DPRINTF(ConstProp, "Connections[%i]: Register %i (SSA ID %i) from %x.%i to %x.%i with value %x (%i) along control path branches[%i] - lastUse? %i\n", entry->consumers[i], path.archRegIdx, path.renamedRegIdx, path.producer.pcAddr, path.producer.uopAddr, path.consumer.pcAddr, path.consumer.uopAddr, connections[speculativeDependencyGraph[idx][way][uop]->consumers[i]].value, connections[speculativeDependencyGraph[idx][way][uop]->consumers[i]].valid, path.directControlDependency, path.lastUse);
                        } else {
                                DPRINTF(ConstProp, "Connection through %i has been invalidated\n", entry->consumers[i]);
                        }
                }
        }
        DPRINTF(ConstProp, "Done Describing\n");
}

void ArrayDependencyTracker::describeFullGraph() {
        for (int i1=0; i1<32; i1++) {
        for (int i2=0; i2<8; i2++) {
            for (int i3=0; i3<6; i3++) {
                if (speculativeDependencyGraph[i1][i2][i3]) {
                                        DPRINTF(ConstProp, "Describing entry at spec[%i][%i][%i]:\n", i1, i2, i3);
                                        describeEntry(i1, i2, i3);
                        }
                        }
                }
        }
}
