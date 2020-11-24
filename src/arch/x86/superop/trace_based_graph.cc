#include "arch/isa_traits.hh"
#include "arch/x86/types.hh"
#include "arch/x86/generated/decoder.hh"
#include "arch/x86/insts/microop.hh"
#include "arch/x86/insts/microregop.hh"
#include "arch/x86/regs/misc.hh"
#include "arch/x86/superop/trace_based_graph.hh"
#include "base/bitfield.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/SuperOp.hh"
#include "debug/ConstProp.hh"
#include "cpu/reg_class.hh"

using namespace X86ISA;
using namespace std;



unsigned SpecTrace::traceIDCounter = 1;

TraceBasedGraph::TraceBasedGraph(TraceBasedGraphParams *p) : SimObject(p), usingControlTracking(p->usingControlTracking), usingCCTracking(p->usingCCTracking) {
    DPRINTF(SuperOp, "Control tracking: %i\n", usingControlTracking);
    DPRINTF(SuperOp, "CC tracking: %i\n", usingCCTracking);
}

TraceBasedGraph* TraceBasedGraphParams::create() {
    return new TraceBasedGraph(this);
}

void TraceBasedGraph::regStats()
{
    tracesPoppedFromQueue
        .name("system.switch_cpus.decode.superop.traceConstructor.tracesPoppedFromQueue")
        .desc("# of traces popped from the trace queue to generate")
        ;
    tracesWithInvalidHead
        .name("system.switch_cpus.decode.superop.traceConstructor.tracesWithInvalidHead")
        .desc("# of traces whose first inst wasn't found in the uop cache")
        ;
}

void TraceBasedGraph::predictValue(Addr addr, unsigned uopAddr, int64_t value, int8_t confidence, unsigned latency)
{

    assert(confidence >= 0);

    DPRINTF(SuperOp, "predictValue: addr=%#x:%x value=%#x confidence=%d latency=%d\n", addr, uopAddr, value, (unsigned) confidence, latency);
    
    /* Check if we have an optimized trace with this prediction source -- isTraceAvailable returns the most profitable trace. */
    int idx = (addr >> 5) & 0x1f;
    for (auto it = traceMap.begin(); it != traceMap.end(); it++) {
        //SpecTrace trace = it->second;
        int numPredSources = 0;
        for (int i=0; i<4; i++, numPredSources++) {
            /* Do we already consider this as a prediction source? */
            if (it->second.source[i].valid && 
                it->second.source[i].addr == FullUopAddr(addr, uopAddr) &&
                it->second.source[i].value == value) 
            {
                DPRINTF(SuperOp, "Trace Map already holds a trace for this prediction source! addr = %#x uopAddr = %d  value = %#x  confidence = %d\n", 
                                  addr, uopAddr, it->second.source[i].value, it->second.source[i].confidence);
                
                return;
            }
        }
        if (it->second.state != SpecTrace::QueuedForFirstTimeOptimization && it->second.state != SpecTrace::QueuedForReoptimization) {
            continue;
        }
        if (idx != it->second.head.idx) {
            continue;
        }
        DPRINTF(SuperOp, "Incoming trace request has the same index as trace %i already in queue\n", it->second.id);
        int numWays = 0;
        for (int way = it->second.head.way; way != 10 && numWays < 8; way = decoder->uopNextWayArray[idx][way]) {
            if (decoder->uopValidArray[idx][way]) {
                DPRINTF(SuperOp, "Looking up uop[%i][%i] of size %d\n", idx, way, decoder->uopCountArray[idx][way]);
                for (int uop = it->second.head.uop; uop < decoder->uopCountArray[idx][way]; uop++) {
                    if (decoder->uopAddrArray[idx][it->second.head.way][uop] == FullUopAddr(addr, uopAddr)) {
                        if (numPredSources < 4) {
                            it->second.source[numPredSources].valid = true;
                            it->second.source[numPredSources].addr = FullUopAddr(addr, uopAddr);
                            it->second.source[numPredSources].value = value;
                            it->second.source[numPredSources].confidence = (unsigned) confidence;
                        }
                        return;
                    }
                }
            }
            numWays++;
        }
    }

    unsigned int traceId = 0; 
    bool reopt = true;
    int spec_way = -1;
    int spec_uop = -1;
    for (int way=0; way<8; way++) {
        if (decoder->speculativeValidArray[idx][way])
        {
            for (int uop=0; uop<6; uop++) {
                if (decoder->speculativeAddrArray[idx][way][uop].pcAddr == addr &&
                    decoder->speculativeAddrArray[idx][way][uop].uopAddr == uopAddr) {
                    traceId = decoder->speculativeTraceIDArray[idx][way];
                    spec_way = way;
                    spec_uop = uop;
                    break;
                } 
            }
            if (traceId) {
                assert(traceMap.find(traceId) != traceMap.end());
                for (int i=0; i<4; i++) {
                    /* Do we already consider this as a prediction source? */
                    if (traceMap[traceId].source[i].valid && 
                        traceMap[traceId].source[i].addr == FullUopAddr(addr, uopAddr) && 
                        traceMap[traceId].source[i].value == value) 
                    {
                        DPRINTF(SuperOp, "Speculative Cache already holds a trace for this prediction source! addr = %#x uopAddr = %d value = %#x  confidence = %d\n", 
                                        addr, uopAddr, traceMap[traceId].source[i].value, traceMap[traceId].source[i].confidence);

                        reopt = false;
                        break;
                    }
                }
                break;
            }
        }
    }

    // if an already optimized trace has low confidence, it is time to phase it out
    if (traceId && reopt) {
        DPRINTF(SuperOp, "Trace %i exists for this prediction source! addr = %#x uopAddr = %d\n", traceId, addr, uopAddr);
        assert(traceMap.find(traceId) != traceMap.end());
        for (int i=0; i<4; i++) {
            /* Have we exhausted all prediction sources? If not, we can further compact this trace.    */
            if (!traceMap[traceId].source[i].valid) {
                
                //assert(0); // Haven't seen any reoptimize yet! 
                SpecTrace newTrace;
                
                for (int j=0; j<4; j++) {
                    if (traceMap[traceId].source[j].valid) {
                        newTrace.source[j] = traceMap[traceId].source[j];
                    }
                }
                newTrace.source[i].valid = true;
                newTrace.source[i].addr = FullUopAddr(addr, uopAddr);
                newTrace.source[i].value = value;
                newTrace.source[i].confidence = (unsigned) confidence;
                newTrace.source[i].latency = latency;
                newTrace.state = SpecTrace::QueuedForReoptimization;
                newTrace.reoptId = traceId;

                unsigned length = computeLength(newTrace);
                if (length < 4) { // TODO: revisit: pretty low bar
                    DPRINTF(SuperOp, "Rejecting trace request to re-optimize trace at uop[%i][%i][%i]\n", idx, spec_way, spec_uop);
                    DPRINTF(SuperOp, "Prediction source: %#x:%i=%#x\n", addr, uopAddr, value);
                    DPRINTF(SuperOp, "length=%i\n", length);
                    return;
                }
                newTrace.head = newTrace.addr = traceMap[traceId].optimizedHead;
                newTrace.headAddr = traceMap[traceId].headAddr;
                newTrace.id = SpecTrace::traceIDCounter++;
                traceMap[newTrace.id] = newTrace;
                traceQueue.push(newTrace);
                DPRINTF(SuperOp, "Queueing up new trace request %i to reoptimize trace %i at spec[%i][%i][%i]\n", newTrace.id, newTrace.reoptId,
                                    newTrace.addr.idx, newTrace.addr.way, newTrace.addr.uop);
                return;
            }
        }
    } else {
        for (int way=0; way<8; way++) {
            DPRINTF(SuperOp, "Looking up uop[%i][%i] of size %d\n", idx, way, decoder->uopCountArray[idx][way]);
            for (int uop=0; uop<decoder->uopCountArray[idx][way]; uop++) {
                if (decoder->uopValidArray[idx][way] && 
                    decoder->uopAddrArray[idx][way][uop].pcAddr == addr && 
                    decoder->uopAddrArray[idx][way][uop].uopAddr == uopAddr) 
                {
                    SpecTrace newTrace;
                    newTrace.source[0].valid = true;
                    newTrace.source[0].addr = FullUopAddr(addr, uopAddr);
                    newTrace.source[0].value = value;
                    newTrace.source[0].confidence = (unsigned) confidence;
                    newTrace.source[0].latency = latency;
                    newTrace.state = SpecTrace::QueuedForFirstTimeOptimization;
                    newTrace.head = newTrace.addr = FullCacheIdx(idx, way, uop);
                    newTrace.headAddr = FullUopAddr(addr, uopAddr);
                    newTrace.instAddr = FullUopAddr(0, 0);

                    // before adding it to the queue, check if profitable -- we prefer long hot traces
                    unsigned hotness = decoder->uopHotnessArray[idx][way].read();
                    unsigned length = computeLength(newTrace);
                    if (hotness < 7 || length < 4) { // TODO: revisit: pretty low bar
                        DPRINTF(SuperOp, "Rejecting trace request to optimize trace at uop[%i][%i][%i]\n", idx, way, uop);
                        DPRINTF(SuperOp, "Prediction source: %#x:%i=%#x\n", addr, uopAddr, value);
                        DPRINTF(SuperOp, "hotness:%i length=%i\n", hotness, length);
                        return;
                    }

                    newTrace.id = SpecTrace::traceIDCounter++;
                    traceMap[newTrace.id] = newTrace;
                    traceQueue.push(newTrace);
                    DPRINTF(SuperOp, "Queueing up new trace request %i to optimize trace at uop[%i][%i][%i]\n", newTrace.id, idx, way, uop);
                    DPRINTF(SuperOp, "Prediction source: %#x:%i=%#x\n", addr, uopAddr, value);
                    DPRINTF(SuperOp, "hotness:%i length=%i\n", hotness, length);
                    return;
                }
            }
        }
    }
}

bool TraceBasedGraph::isPredictionSource(SpecTrace& trace, FullUopAddr addr, uint64_t &value, unsigned &confidence, unsigned &latency) {
    for (int i=0; i<4; i++) {
        if (trace.source[i].valid && trace.source[i].addr == addr) {
            value = trace.source[i].value;
            confidence = trace.source[i].confidence;
            latency = trace.source[i].latency;
            return true;
        }
    }
    return false;
}

bool TraceBasedGraph::advanceIfControlTransfer(SpecTrace &trace) {
    // don't do this for re-optimizations
    if (trace.state != SpecTrace::QueuedForFirstTimeOptimization && trace.state != SpecTrace::OptimizationInProcess)
        return false;

    // don't fold more than 2 branches
    if (trace.branchesFolded >= 2)
        return false;

    StaticInstPtr decodedMacroOp = decoder->decodeInst(decoder->uopCache[trace.addr.idx][trace.addr.way][trace.addr.uop]);
    StaticInstPtr decodedMicroOp = decodedMacroOp;
    if (decodedMacroOp->isMacroop()) {
        Addr uopAddr = decoder->uopAddrArray[trace.addr.idx][trace.addr.way][trace.addr.uop].uopAddr;
        decodedMicroOp = decodedMacroOp->fetchMicroop(uopAddr);
        decodedMicroOp->macroOp = decodedMacroOp;
    }

    // not a control transfer -- advance normally
    if (!decodedMicroOp->isControl()) {
        if (decodedMacroOp->isMacroop()) { 
			decodedMacroOp->deleteMicroOps();
			decodedMacroOp = NULL;
		}
        return false;
    }

    // end the trace if a return or a branch without a confident prediction is encountered
    Addr pcAddr = decoder->uopAddrArray[trace.addr.idx][trace.addr.way][trace.addr.uop].pcAddr;
    if (decodedMicroOp->isReturn() || !branchPred->getConfidenceForSSO(pcAddr)) {
        DPRINTF(SuperOp, "Ending trace -- return or a branch without a confident prediction is encountered\n");
        trace.addr.valid = false;
        if (decodedMacroOp->isMacroop()) { 
			decodedMacroOp->deleteMicroOps();
			decodedMacroOp = NULL;
		}
        return true;
    }

    std::string disas = decodedMicroOp->disassemble(pcAddr);

    // if it is a direct call or a jump, fold the branch (provided it is predicted taken)
    Addr target = pcAddr + decodedMacroOp->machInst.instSize;
    if (disas.find("CALL_NEAR_I") != std::string::npos || disas.find("JMP_I") != std::string::npos || decodedMicroOp->isCondCtrl()) {
        target = pcAddr + decodedMacroOp->machInst.instSize + decodedMacroOp->machInst.immediate;
    } 

    // not a taken branch -- advance normally
    bool predTaken = branchPred->lookupWithoutUpdate(0, pcAddr);
    if (!(disas.find("CALL_NEAR_I") != std::string::npos || disas.find("JMP_I") != std::string::npos) && !predTaken) {
        target = pcAddr + decodedMacroOp->machInst.instSize;
    }

    // indirect branch -- lookup indirect predictor
    if (decodedMacroOp->machInst.opcode.op == 0xFF) {
        TheISA::PCState targetPC;
        branchPred->iPred.lookup(pcAddr, branchPred->getGHR(0, NULL), targetPC, 0); // assuming tid = 0
        target = targetPC.instAddr();
    }

    // pivot to jump target if it is found in the uop cache
    int idx = (target >> 5) & 0x1f;
    uint64_t tag = (target >> 10);
    for (int way = 0; way < 8; way++) {
        if (decoder->uopValidArray[idx][way] && decoder->uopTagArray[idx][way] == tag) {
            for (int uop = 0; uop < decoder->uopCountArray[idx][way]; uop++) {
                if (decoder->uopAddrArray[idx][way][uop].pcAddr == target &&
                        decoder->uopAddrArray[idx][way][uop].uopAddr == 0) {
                    trace.addr.idx = idx;
                    trace.addr.way = way;
                    trace.addr.uop = uop;
                    if (decodedMacroOp->isMacroop()) { 
                        decodedMacroOp->deleteMicroOps();
                        decodedMacroOp = NULL;
                    }
                    DPRINTF(ConstProp, "Control Tracking: jumping to address %#x: uop[%i][%i][%i]\n", target, idx, way, uop);
                   
                    if (!traceMap[trace.id].controlSources[trace.branchesFolded].valid) {
                        traceMap[trace.id].controlSources[trace.branchesFolded].confidence = 9;
                        traceMap[trace.id].controlSources[trace.branchesFolded].valid = true;
                        traceMap[trace.id].controlSources[trace.branchesFolded].value = target;
                    }

                    trace.branchesFolded++;
                    
                    return true;
                }
            }
        }
    }

    if (decodedMacroOp->isMacroop()) { 
        decodedMacroOp->deleteMicroOps();
        decodedMacroOp = NULL;
    }

    DPRINTF(SuperOp, "Ending trace -- uop cache miss at branch target\n");
    trace.addr.valid = false;
    return false;
}

void TraceBasedGraph::advanceTrace(SpecTrace &trace) {
    if (!usingControlTracking || !advanceIfControlTransfer(trace)) {
        trace.addr.uop++;
        trace.addr.valid = false;
        // select cache to advance from
        if (trace.state == SpecTrace::QueuedForFirstTimeOptimization || trace.state == SpecTrace::OptimizationInProcess) {
            FullUopAddr prevAddr = decoder->uopAddrArray[trace.addr.idx][trace.addr.way][trace.addr.uop-1];
            FullUopAddr nextAddr = decoder->uopAddrArray[trace.addr.idx][trace.addr.way][trace.addr.uop];
            
            if (prevAddr.pcAddr == nextAddr.pcAddr) {
                trace.addr.valid = true;
            } else {
                nextAddr.pcAddr = prevAddr.pcAddr + decoder->uopCache[trace.addr.idx][trace.addr.way][trace.addr.uop-1].instSize;
                if (trace.addr.idx != ((nextAddr.pcAddr >> 5) & 0x1f)) { // we have exhausted all ways
                    return;
                }
                uint64_t tag = (nextAddr.pcAddr >> 10);
                for (int way = 0; way < 8; way++) {
                    if (decoder->uopValidArray[trace.addr.idx][way] && decoder->uopTagArray[trace.addr.idx][way] == tag) {
                        for (int uop = 0; uop < decoder->uopCountArray[trace.addr.idx][way]; uop++) {
                            if (decoder->uopAddrArray[trace.addr.idx][way][uop].pcAddr == nextAddr.pcAddr) {
                                trace.addr.way = way;
                                trace.addr.uop = uop;
                                trace.addr.valid = true;
                                return;
                            }
                        }
                    }
                }
            }
        } else {
            if (trace.addr.uop < decoder->speculativeCountArray[trace.addr.idx][trace.addr.way]) {
                trace.addr.valid = true;
            } else if (decoder->speculativeNextWayArray[trace.addr.idx][trace.addr.way] != 10) {
                trace.addr.uop = 0;
                trace.addr.way = decoder->speculativeNextWayArray[trace.addr.idx][trace.addr.way];
                trace.addr.valid = true;
            }
        }
    }
}

void TraceBasedGraph::dumpTrace(SpecTrace trace) {
    // not a valid trace
    if (!trace.addr.valid) {
        return;
    }

    int idx = trace.addr.idx;
    int way = trace.addr.way;
    int uop = trace.addr.uop;

    // select cache to dump from
    if (trace.state == SpecTrace::QueuedForFirstTimeOptimization || trace.state == SpecTrace::OptimizationInProcess) {
        Addr pcAddr = decoder->uopAddrArray[idx][way][uop].pcAddr;
        Addr uopAddr = decoder->uopAddrArray[idx][way][uop].uopAddr;
        StaticInstPtr decodedMacroOp = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
        StaticInstPtr decodedMicroOp = decodedMacroOp;
        if (decodedMacroOp->isMacroop()) {
            decodedMicroOp = decodedMacroOp->fetchMicroop(uopAddr);
            decodedMicroOp->macroOp = decodedMacroOp;
        }
        DPRINTF(SuperOp, "%p:%i -- uop[%i][%i][%i] -- %s\n", pcAddr, uopAddr, idx, way, uop, decodedMicroOp->disassemble(pcAddr));    

        if (decodedMacroOp->isMacroop()) { 
			decodedMacroOp->deleteMicroOps();
			decodedMacroOp = NULL;
		}
    } else {
        Addr pcAddr = decoder->speculativeAddrArray[idx][way][uop].pcAddr;
        Addr uopAddr = decoder->speculativeAddrArray[idx][way][uop].uopAddr;
        StaticInstPtr decodedMicroOp = decoder->speculativeCache[idx][way][uop];
        DPRINTF(SuperOp, "%p:%i -- spec[%i][%i][%i] -- %s\n", pcAddr, uopAddr, idx, way, uop, decodedMicroOp->disassemble(pcAddr));    
		for (int i=0; i<decodedMicroOp->numSrcRegs(); i++) {
			// LAYNE : TODO : print predicted inputs (checking syntax)
			if (decodedMicroOp->sourcesPredicted[i]) {
				DPRINTF(SuperOp, "\tSource for register %i predicted as %x\n", decodedMicroOp->srcRegIdx(i), decodedMicroOp->sourcePredictions[i]);
			}
		}
		for (int i=0; i<decodedMicroOp->numDestRegs(); i++) {
			if (decodedMicroOp->liveOutPredicted[i]) {
				DPRINTF(SuperOp, "\tLive out for register %i predicted as %x\n", decodedMicroOp->destRegIdx(i), decodedMicroOp->liveOut[i]);
			}
		}
    }

    advanceTrace(trace);
    dumpTrace(trace);
}

unsigned TraceBasedGraph::computeLength(SpecTrace trace) {
    if (!trace.addr.valid) {
        return 0;
    }

    advanceTrace(trace);

    return (1 + computeLength(trace));
}

bool TraceBasedGraph::generateNextTraceInst() {
    if (!currentTrace.addr.valid) { 
        // Finalize old trace
        if (currentTrace.state != SpecTrace::Complete && currentTrace.state != SpecTrace::Evicted) {
            DPRINTF(SuperOp, "Done optimizing trace %i with actual length %i, shrunk to length %i\n", currentTrace.id, currentTrace.length, currentTrace.shrunkLength);
            DPRINTF(SuperOp, "Before optimization: \n");
            currentTrace.addr = currentTrace.head;
            dumpTrace(currentTrace);
            DPRINTF(SuperOp, "After optimization: \n");
            int idx = currentTrace.head.idx;
            int way = currentTrace.optimizedHead.way;
            if (decoder->speculativeValidArray[idx][way] && decoder->speculativeTraceIDArray[idx][way] == currentTrace.id) {
                // mark end of trace and propagate live outs
                DPRINTF(SuperOp, "End of Trace at %s!\n", currentTrace.prevNonEliminatedInst->getName());
                
                // here we mark 'prevNonEliminatedInst' as end of the trace because sometimes an eliminated instruction can be set as end of the trace
                currentTrace.prevNonEliminatedInst->setEndOfTrace();
                currentTrace.prevNonEliminatedInst->shrunkLength = currentTrace.shrunkLength;
                if (!currentTrace.prevNonEliminatedInst->isControl()) { // control prevNonEliminatedInstructions already propagate live outs
                    for (int i=0; i<16; i++) { // 16 int registers
                        if (regCtx[i].valid && !regCtx[i].source) {
                            currentTrace.prevNonEliminatedInst->liveOut[currentTrace.prevNonEliminatedInst->numDestRegs()] = regCtx[i].value;
                            currentTrace.prevNonEliminatedInst->liveOutPredicted[currentTrace.prevNonEliminatedInst->numDestRegs()] = true;
                            currentTrace.prevNonEliminatedInst->addDestReg(RegId(IntRegClass, i));
                            currentTrace.prevNonEliminatedInst->_numIntDestRegs++;
                        }
                    }
                    if (ccValid) {
                        currentTrace.prevNonEliminatedInst->liveOut[currentTrace.prevNonEliminatedInst->numDestRegs()] = PredccFlagBits;
                        currentTrace.prevNonEliminatedInst->liveOutPredicted[currentTrace.prevNonEliminatedInst->numDestRegs()] = true;
                        currentTrace.prevNonEliminatedInst->addDestReg(RegId(CCRegClass, CCREG_ZAPS));
                        currentTrace.prevNonEliminatedInst->liveOut[currentTrace.prevNonEliminatedInst->numDestRegs()] = PredcfofBits;
                        currentTrace.prevNonEliminatedInst->liveOutPredicted[currentTrace.prevNonEliminatedInst->numDestRegs()] = true;
                        currentTrace.prevNonEliminatedInst->addDestReg(RegId(CCRegClass, CCREG_CFOF));
                        currentTrace.prevNonEliminatedInst->liveOut[currentTrace.prevNonEliminatedInst->numDestRegs()] = PreddfBit;
                        currentTrace.prevNonEliminatedInst->liveOutPredicted[currentTrace.prevNonEliminatedInst->numDestRegs()] = true;
                        currentTrace.prevNonEliminatedInst->addDestReg(RegId(CCRegClass, CCREG_DF));
                        currentTrace.prevNonEliminatedInst->liveOut[currentTrace.prevNonEliminatedInst->numDestRegs()] = PredecfBit;
                        currentTrace.prevNonEliminatedInst->liveOutPredicted[currentTrace.prevNonEliminatedInst->numDestRegs()] = true;
                        currentTrace.prevNonEliminatedInst->addDestReg(RegId(CCRegClass, CCREG_ECF));
                        currentTrace.prevNonEliminatedInst->liveOut[currentTrace.prevNonEliminatedInst->numDestRegs()] = PredezfBit;
                        currentTrace.prevNonEliminatedInst->liveOutPredicted[currentTrace.prevNonEliminatedInst->numDestRegs()] = true;
                        currentTrace.prevNonEliminatedInst->addDestReg(RegId(CCRegClass, CCREG_EZF));
                        currentTrace.prevNonEliminatedInst->_numCCDestRegs += 5;
                    }
                }
                currentTrace.addr = currentTrace.optimizedHead;
                currentTrace.state = SpecTrace::Complete;
                traceMap[currentTrace.id] = currentTrace;
                dumpTrace(currentTrace);
                DPRINTF(SuperOp, "Live Outs:\n");
                for (int i=0; i<16; i++) {
                    if (regCtx[i].valid && !regCtx[i].source)
                        DPRINTF(SuperOp, "reg[%i]=%#x\n", i, regCtx[i].value);
                }
                if (ccValid) {
                    DPRINTF(SuperOp, "PredccFlagBits: %#x\n", PredccFlagBits);
                    DPRINTF(SuperOp, "PredcfofBits: %#x\n", PredcfofBits);
                    DPRINTF(SuperOp, "PreddfBit: %#x\n", PreddfBit);
                    DPRINTF(SuperOp, "PredecfBit: %#x\n", PredecfBit);
                    DPRINTF(SuperOp, "PredezfBit: %#x\n", PredezfBit);
                } else {
                    DPRINTF(SuperOp, "No live out CC\n");
                }
                for (int i=0; i<4; i++) {
                    DPRINTF(SuperOp, "Prediction Source %i\n", i);
                    if (currentTrace.source[i].valid) {
                        // set the predecitions sources in spec$

                        DPRINTF(SuperOp, "Address=%#x:%i, Value=%#x, Confidence=%i, Latency=%i\n",
                                    currentTrace.source[i].addr.pcAddr,  currentTrace.source[i].addr.uopAddr,
                                    currentTrace.source[i].value, currentTrace.source[i].confidence,
                                    currentTrace.source[i].latency);
                    }
                }
            }
        }

        // Pop a new trace from the queue, start at top
        do {
            if (traceQueue.empty()) {
                currentTrace.addr.valid = false;
                currentTrace.id = 0;
                return false; 
            }

            currentTrace = traceQueue.front();
            traceQueue.pop();
            tracesPoppedFromQueue++;

            assert(currentTrace.id);

            int idx = currentTrace.addr.idx;
            int way = currentTrace.addr.way;
            int uop = currentTrace.addr.uop;
            if (!(currentTrace.state == SpecTrace::QueuedForFirstTimeOptimization &&
                  decoder->uopValidArray[idx][way] &&
                  decoder->uopAddrArray[idx][way][uop] == currentTrace.headAddr) &&
                !(currentTrace.state == SpecTrace::QueuedForReoptimization &&
                  decoder->speculativeValidArray[idx][way] &&
                  decoder->speculativeAddrArray[idx][way][uop] == currentTrace.headAddr)) 
            {
                DPRINTF(SuperOp, "Trace %i at (%i,%i,%i) evicted before we could process it.\n", currentTrace.id, currentTrace.addr.idx, currentTrace.addr.way, currentTrace.addr.uop);
                currentTrace.addr.valid = false;
                currentTrace.state = SpecTrace::Evicted;
                

                // remove it from traceMap
                DPRINTF(Decoder, "Removing traceID: %d (reoptID: %d) from Trace Map because of eviction!.\n", currentTrace.id, currentTrace.reoptId );
                assert(traceMap.find(currentTrace.id) != traceMap.end());
                traceMap.erase(currentTrace.id);

                currentTrace.id = 0;
                
            }
        } while (!currentTrace.addr.valid);

        DPRINTF(SuperOp, "Optimizing trace %i at (%i,%i,%i)\n", currentTrace.id, currentTrace.addr.idx, currentTrace.addr.way, currentTrace.addr.uop);
        dumpTrace(currentTrace);

        if (currentTrace.state == SpecTrace::QueuedForFirstTimeOptimization) {
            currentTrace.state = SpecTrace::OptimizationInProcess;
        } else if (currentTrace.state == SpecTrace::QueuedForReoptimization) {
            currentTrace.state = SpecTrace::ReoptimizationInProcess;
        }

        /* Clear Reg Context Block. */
        for (int i=0; i<38; i++) {
            regCtx[i].valid = regCtx[i].source = ccValid = false;
            PredccFlagBits = PredcfofBits = PreddfBit = PredecfBit = PredezfBit = 0;
        }
    } else { 
        assert(currentTrace.state == SpecTrace::OptimizationInProcess || currentTrace.state == SpecTrace::ReoptimizationInProcess);
    }

    if (!currentTrace.addr.valid) { return false; } // no traces in queue to pop

    int idx = currentTrace.addr.idx;
    int way = currentTrace.addr.way;
    int uop = currentTrace.addr.uop;
    SpecTrace::State state = currentTrace.state;

    StaticInstPtr decodedMacroOp = NULL;
    bool newMacro = false;
    if (state == SpecTrace::OptimizationInProcess) {
        DPRINTF(ConstProp, "Trace %i: Processing instruction at uop[%i][%i][%i]\n", currentTrace.id, idx, way, uop);
        if (!decoder->uopValidArray[idx][way] || (uop >= decoder->uopCountArray[idx][way])) {
            tracesWithInvalidHead++;
            DPRINTF(SuperOp, "Trace was evicted out of the micro-op cache before we could optimize it\n");
            currentTrace.addr.valid = false;
            return false;
        }
        decodedMacroOp = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
        if (decodedMacroOp->getName() == "NOP" || decodedMacroOp->getName() == "fault" || decodedMacroOp->getName() == "popcnt_Gv_Ev") {
            currentTrace.length++;
            advanceTrace(currentTrace);
            return true;
        }
        if (decodedMacroOp->isMacroop()) {
            StaticInstPtr inst = decodedMacroOp->fetchMicroop(decoder->uopAddrArray[idx][way][uop].uopAddr);
            if (inst->getName() == "NOP" || inst->getName() == "fault" || inst->getName() == "popcnt_Gv_Ev") {
                currentTrace.length++;
                advanceTrace(currentTrace);
                return true;
            }
            currentTrace.inst = inst;
            currentTrace.inst->macroOp = decodedMacroOp;
        } else {
            currentTrace.inst = decodedMacroOp;
        }
        currentTrace.lastAddr = currentTrace.instAddr;
        currentTrace.instAddr = decoder->uopAddrArray[idx][way][uop];
        newMacro = (currentTrace.lastAddr.pcAddr != currentTrace.instAddr.pcAddr);
    } else if (state == SpecTrace::ReoptimizationInProcess) {
        DPRINTF(ConstProp, "Trace %i: Processing instruction at spec[%i][%i][%i]\n", currentTrace.id, idx, way, uop);
        if (!decoder->speculativeValidArray[idx][way] || (uop >= decoder->speculativeCountArray[idx][way])) {
            tracesWithInvalidHead++;
            DPRINTF(SuperOp, "Trace was evicted out of the speculative cache before we could optimize it\n");
            currentTrace.addr.valid = false;
            return false;
        }
        decodedMacroOp = decoder->decodeInst(decoder->speculativeCache[idx][way][uop]->macroOp->machInst);
        if (decodedMacroOp->isMacroop()) {
            currentTrace.inst = decodedMacroOp->fetchMicroop(decoder->speculativeAddrArray[idx][way][uop].uopAddr);
            currentTrace.inst->macroOp = decodedMacroOp;
        }
        currentTrace.lastAddr = currentTrace.instAddr;
        currentTrace.instAddr = decoder->speculativeAddrArray[idx][way][uop];
        newMacro = (currentTrace.lastAddr.pcAddr != currentTrace.instAddr.pcAddr);
    }
    decodedMacroOp = NULL;

    if (newMacro) {
        /* Clear Micro Registers in Reg Context Block. */
        for (int i=16; i<38; i++) {
            regCtx[i].valid = regCtx[i].source = false;
        }
    }

    bool updateSuccessful = false;

    // Any inst in a trace may be a prediction source
    DPRINTF(ConstProp, "Trace %i: Processing instruction: %p:%i -- %s\n", currentTrace.id, currentTrace.instAddr.pcAddr, currentTrace.instAddr.uopAddr, currentTrace.inst->getName());
    uint64_t value = 0;
    unsigned confidence = 0;
    unsigned latency;
    string type = currentTrace.inst->getName();
    if (type != "rdip" && type != "limm" && type != "movi" && isPredictionSource(currentTrace, currentTrace.instAddr, value, confidence, latency)) {
        // Step 1: Get predicted value from LVP
        // Step 2: Determine dest register(s)
        // Step 3: Annotate dest register entries with that value
        // Step 4: Add inst to speculative trace
        for (int i = 0; i < currentTrace.inst->numDestRegs(); i++) {
            RegId destReg = currentTrace.inst->destRegIdx(i);
            if (destReg.classValue() == IntRegClass) {
				DPRINTF(SuperOp, "Setting regCtx[%i] to %x from %s inst\n", destReg.flatIndex(), value, type);
                regCtx[destReg.flatIndex()].value = currentTrace.inst->predictedValue = value;
                regCtx[destReg.flatIndex()].valid = regCtx[destReg.flatIndex()].source = currentTrace.inst->predictedLoad = true;
                currentTrace.inst->confidence = confidence;
            }
        }
        bool isDeadCode = false;
        updateSuccessful = updateSpecTrace(currentTrace, isDeadCode, false);
        
        // source predictions never get eliminated
        if (!isDeadCode)
        {
            currentTrace.prevNonEliminatedInst = currentTrace.inst;
        }
        else 
        {
            panic("Prediction Source is a dead code?!");
        }

    } else {

        bool propagated = false;
        bool folded = false;
        // Propagate predicted values
        if (type == "mov") {
            DPRINTF(ConstProp, "Found a MOV at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagated = propagateMov(currentTrace.inst);
        } else if (type == "rdip") {
            DPRINTF(ConstProp, "Found an RDIP at [%i][%i][%i], compacting...\n", idx, way, uop);
            RegId destReg = currentTrace.inst->destRegIdx(0);
            regCtx[destReg.flatIndex()].value = currentTrace.instAddr.pcAddr + currentTrace.inst->macroOp->getMacroopSize();
            regCtx[destReg.flatIndex()].valid = propagated = true;
            DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", regCtx[destReg.flatIndex()].value, destReg.flatIndex());
        } else if (type == "wrip") {
            DPRINTF(ConstProp, "Found a WRIP branch at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagated = folded = propagateWrip(currentTrace.inst);
        } else if (type == "wripi") {
            DPRINTF(ConstProp, "Found a WRIPI branch at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagated = folded = propagateWripI(currentTrace.inst);
        } else if (currentTrace.inst->isControl()) {
            // printf("Control instruction of type %s\n", type);
            // TODO: check for stopping condition or predicted target
        } else if (type == "movi") {
            DPRINTF(ConstProp, "Found a MOVI at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagated = propagateMovI(currentTrace.inst);
        } else if (type == "and") {
            DPRINTF(ConstProp, "Found an AND at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagated = propagateAnd(currentTrace.inst);
        } else if (type == "add") {
            DPRINTF(ConstProp, "Found an ADD at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagated = propagateAdd(currentTrace.inst);
        } else if (type == "sub") {
            DPRINTF(ConstProp, "Found a SUB at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagated = propagateSub(currentTrace.inst);
        } else if (type == "xor") {
            DPRINTF(ConstProp, "Found an XOR at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagated = propagateXor(currentTrace.inst);
        } else if (type == "or") {
            DPRINTF(ConstProp, "Found an OR at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagated = propagateOr(currentTrace.inst);
        } else if (type == "subi") {
            DPRINTF(ConstProp, "Found a SUBI at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagated = propagateSubI(currentTrace.inst);
        } else if (type == "addi") {
            DPRINTF(ConstProp, "Found an ADDI at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagated = propagateAddI(currentTrace.inst);
        } else if (type == "slli") {
            DPRINTF(ConstProp, "Found a SLLI at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagated = propagateSllI(currentTrace.inst);
        } else if (type == "srli") {
            DPRINTF(ConstProp, "Found a SRLI at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagated = propagateSrlI(currentTrace.inst);
        } else if (type == "lea") {
            DPRINTF(ConstProp, "Type is LEA");
            // Requires multiple ALU operations to propagate, not using
        } else if (type == "sexti") {
            // Implementation has multiple ALU operations, but this is not required by the nature of the operation
            DPRINTF(ConstProp, "Found a SEXTI at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagated = propagateSExtI(currentTrace.inst);
        } else if (type == "zexti") {
            // Implementation has multiple ALU operations, but this is not required by the nature of the operation
            DPRINTF(ConstProp, "Found a ZEXTI at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagated = propagateZExtI(currentTrace.inst);
        } else if (type == "mul1s" || type == "mul1u" || type == "mulel" || type == "muleh") {
            DPRINTF(ConstProp, "Type is MUL1S, MUL1U, MULEL, or MULEH\n");
            // TODO: two dest regs with different values? maybe too complex arithmetic?
        } else if (type == "limm") {
            DPRINTF(ConstProp, "Found a LIMM at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagated = propagateLimm(currentTrace.inst);
        } else if (type == "rflags" || type == "wrflags" || type == "ruflags" || type == "wruflags") {
            DPRINTF(ConstProp, "Type    is RFLAGS, WRFLAGS, RUFLAGS, or WRUFLAGS\n");
            // TODO: add control registers to graph?
        } else if (type == "rdtsc" || type == "rdval") {
            DPRINTF(ConstProp, "Type is RDTSC or RDVAL\n");
            // TODO: determine whether direct register file access needs to be handled differently?
        } else if (type == "panic" || type == "CPUID") {
            DPRINTF(ConstProp, "Type is PANIC or CPUID\n");
            // TODO: possibly remove, what is purpose?
        } else if (type == "st" || type == "stis" || type == "stfp" || type == "ld" || type == "ldis" || type == "ldst" || type == "syscall" || type == "halt" || type == "fault" || type == "call_far_Mp") {
            DPRINTF(ConstProp, "Type is ST, STIS, STFP, LD, LDIS, LDST, SYSCALL, HALT, FAULT, or CALL_FAR_MP\n");
            // TODO: cannot remove
        } else {
            DPRINTF(ConstProp, "Inst type not covered: %s\n", type);
        }

        bool isDeadCode = false;
        if (!folded) {
            updateSuccessful = updateSpecTrace(currentTrace, isDeadCode, propagated);
        }
        // if it's not a dead code, then update the last non-eliminated microop of the specTrace
        if (!isDeadCode && !folded)
        {
            currentTrace.prevNonEliminatedInst = currentTrace.inst;
        }
    }

    // Simulate a stall if update to speculative cache wasn't successful
    if (updateSuccessful) {
        advanceTrace(currentTrace);
    }

    // Propagate live outs at the end of each control instruction
    if (currentTrace.inst->isControl() && updateSuccessful) {
        for (int i=0; i<16; i++) { // 16 int registers
            if (regCtx[i].valid && !regCtx[i].source) {
                currentTrace.inst->liveOut[currentTrace.inst->numDestRegs()] = regCtx[i].value;
                currentTrace.inst->liveOutPredicted[currentTrace.inst->numDestRegs()] = true;
                currentTrace.inst->addDestReg(RegId(IntRegClass, i));
                currentTrace.inst->_numIntDestRegs++;
            }
        }
        if (ccValid) {
            currentTrace.inst->liveOut[currentTrace.inst->numDestRegs()] = PredccFlagBits;
            currentTrace.inst->liveOutPredicted[currentTrace.inst->numDestRegs()] = true;
            currentTrace.inst->addDestReg(RegId(CCRegClass, CCREG_ZAPS));
            currentTrace.inst->liveOut[currentTrace.inst->numDestRegs()] = PredcfofBits;
            currentTrace.inst->liveOutPredicted[currentTrace.inst->numDestRegs()] = true;
            currentTrace.inst->addDestReg(RegId(CCRegClass, CCREG_CFOF));
            currentTrace.inst->liveOut[currentTrace.inst->numDestRegs()] = PreddfBit;
            currentTrace.inst->liveOutPredicted[currentTrace.inst->numDestRegs()] = true;
            currentTrace.inst->addDestReg(RegId(CCRegClass, CCREG_DF));
            currentTrace.inst->liveOut[currentTrace.inst->numDestRegs()] = PredecfBit;
            currentTrace.inst->liveOutPredicted[currentTrace.inst->numDestRegs()] = true;
            currentTrace.inst->addDestReg(RegId(CCRegClass, CCREG_ECF));
            currentTrace.inst->liveOut[currentTrace.inst->numDestRegs()] = PredezfBit;
            currentTrace.inst->liveOutPredicted[currentTrace.inst->numDestRegs()] = true;
            currentTrace.inst->addDestReg(RegId(CCRegClass, CCREG_EZF));
            currentTrace.inst->_numCCDestRegs += 5;
        }
    }
    DPRINTF(SuperOp, "Live Outs:\n");
    for (int i=0; i<16; i++) {
        if (regCtx[i].valid && !regCtx[i].source)
            DPRINTF(SuperOp, "reg[%i]=%#x\n", i, regCtx[i].value);
    }
    if (ccValid) {
        DPRINTF(SuperOp, "PredccFlagBits: %#x\n", PredccFlagBits);
        DPRINTF(SuperOp, "PredcfofBits: %#x\n", PredcfofBits);
        DPRINTF(SuperOp, "PreddfBit: %#x\n", PreddfBit);
        DPRINTF(SuperOp, "PredecfBit: %#x\n", PredecfBit);
        DPRINTF(SuperOp, "PredezfBit: %#x\n", PredezfBit);
    } else {
        DPRINTF(SuperOp, "No live out CC\n");
    }

    return true;
}

bool TraceBasedGraph::updateSpecTrace(SpecTrace &trace, bool &isDeadCode , bool propagated) {
    // IMPORTANT NOTE: This is written assuming the trace will be traversed in order, and so the register map will be accurate for the current point in the trace
    trace.length++;

    // Rather than checking dests, check sources; if all sources, then all dests in trace
    bool allSrcsReady = true;
/*    for (int i=0; i<trace.inst->numSrcRegs(); i++) {
        RegId srcReg = trace.inst->srcRegIdx(i);
        allSrcsReady = allSrcsReady && regCtx[srcReg.flatIndex()].valid;
    }*/

    string type = trace.inst->getName();
    isDeadCode = (type == "rdip") || (allSrcsReady && (type == "mov" || type == "movi" || type == "limm" || type == "add" || type == "addi" || type == "sub" || type == "subi" || type == "and" || type == "andi" || type == "or" || type == "ori" || type == "xor" || type == "xori" || type == "slri" || type == "slli" || type == "sexti" || type == "zexti"));

    // Prevent an inst registering as dead if it is a prediction source or if it is a return or it modifies CC
    uint64_t value;
    unsigned confidence;
    unsigned latency;
    bool isPredSource = isPredictionSource(trace, trace.instAddr, value, confidence, latency) && type != "limm" && type != "movi" && type != "rdip";
    isDeadCode &= (propagated && !isPredSource && !((!usingCCTracking && trace.inst->isCC()) || trace.inst->isReturn()));

    DPRINTF(ConstProp, "isDeadCode:%d propagated:%d isPredSource:%d CC:%d Return:%d\n", isDeadCode, propagated, isPredSource, (!usingCCTracking && trace.inst->isCC()), trace.inst->isReturn());
    if (allSrcsReady && (!usingCCTracking && trace.inst->isCC()))
    {
        DPRINTF(ConstProp, "All sources are ready for instruction at %#x:%#x but it is not a dead code as it's a CC inst!\n", trace.instAddr.pcAddr, trace.instAddr.uopAddr);
    }
    else if (allSrcsReady && !propagated)
    {
        DPRINTF(ConstProp, "All sources are ready for instruction at %#x:%#x but it is not a dead code as its data size is less than 4/8 bytes!\n", trace.instAddr.pcAddr, trace.instAddr.uopAddr);
    }

    // Inst will never already be in this trace, single pass
    if (isDeadCode) {
        DPRINTF(ConstProp, "Dead code at %#x:%#x\n", trace.instAddr.pcAddr, trace.instAddr.uopAddr);
        DPRINTF(Decoder, "Skipping microop update in the speculative cache\n");
        return true;
    }

    bool updateSuccessful = decoder->addUopToSpeculativeCache( trace, isPredSource);
    trace.shrunkLength++;

    // Step 3b: Mark all predicted values on the StaticInst -- don't do this for prediction sources
    if (!isPredSource) {
        for (int i=0; i<trace.inst->numSrcRegs(); i++) {
            unsigned srcIdx = trace.inst->srcRegIdx(i).flatIndex();
            DPRINTF(ConstProp, "ConstProp: Examining register %i\n", srcIdx);
            if (regCtx[srcIdx].valid && trace.inst->srcRegIdx(i).classValue() == IntRegClass) {
                DPRINTF(ConstProp, "ConstProp: Propagated constant %#x in reg %i at %#x:%#x\n", regCtx[srcIdx].value, srcIdx, trace.instAddr.pcAddr, trace.instAddr.uopAddr);
                DPRINTF(ConstProp, "ConstProp: Setting trace.inst sourcePrediction to %#x\n", regCtx[srcIdx].value);
                trace.inst->sourcePredictions[i] = regCtx[srcIdx].value;
                trace.inst->sourcesPredicted[i] = true;
            }
        }

        // update live outs
        for (int i=0; i<trace.inst->numDestRegs(); i++) {
            RegId destReg = trace.inst->destRegIdx(i);
            if (destReg.classValue() == IntRegClass) {
                regCtx[destReg.flatIndex()].valid = false;
            }
        }
    }

    // Update head of the optimized trace
    if (!trace.optimizedHead.valid) {
        DPRINTF(Decoder, "updateSpecTrace: Trace %d optimized head is not valid!\n", trace.id);
        trace.optimizedHead.idx = trace.head.idx; 
        trace.optimizedHead.uop = 0;
        for (int way=0; way<8; way++) {
            int idx = trace.head.idx;
            if (decoder->speculativeValidArray[idx][way] && decoder->speculativeTraceIDArray[idx][way] == trace.id) {
                DPRINTF(Decoder, "updateSpecTrace: Trace %d optimized head way is updated to %d!\n", trace.id, way);
                trace.optimizedHead.way = way;
                trace.optimizedHead.valid = true;
                break;
            }
        }
    }

    return updateSuccessful;
}

bool TraceBasedGraph::propagateMov(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "mov");
    
    //if(inst->numSrcRegs() != 3) return false;

    if (inst->isCC() && (!usingCCTracking || !ccValid))
    {
        DPRINTF(ConstProp, "CC Mov Inst! We can't propagate CC insts!\n");
        return false;
    }

    // Mov is both inhereted from RegOp
    X86ISA::RegOp * inst_regop = (X86ISA::RegOp * )inst.get(); 
    const uint8_t dataSize = inst_regop->dataSize;
    assert(dataSize == 8 || dataSize == 4 || dataSize == 2 || dataSize == 1);

    unsigned src1 = inst->srcRegIdx(0).flatIndex(); src1 = src1;
    unsigned src2 = inst->srcRegIdx(1).flatIndex(); // this is the soruce reg in 4 or 8 byte moves

    if (/*(!regCtx[src1].valid) ||*/ (!regCtx[src2].valid)) {
        return false;
    }

    //uint64_t SrcReg1 = regCtx[src1].value;
    uint64_t SrcReg2 = regCtx[src2].value;

    uint64_t forwardVal = 0;
    X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
    uint64_t psrc2 = x86_inst->pick(SrcReg2, 1, dataSize);

	// I think for movs that should be okay? --LAYNE
    // assert(SrcReg2 == psrc2);  // for 4 or 8 bytes move this should always hold but not true for 1 or 2 byts move

    uint16_t ext = inst->getExt();
    if (!inst->isCC() || inst->checkCondition(PredccFlagBits | PredcfofBits | PreddfBit | PredecfBit | PredezfBit, ext)) {
        forwardVal = x86_inst->merge(forwardVal, psrc2, dataSize);
    } else {
        return true;
    }

    RegId destReg = inst->destRegIdx(0);
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destReg.flatIndex());

    regCtx[destReg.flatIndex()].value = forwardVal;
    regCtx[destReg.flatIndex()].valid = true;
    regCtx[destReg.flatIndex()].source = false;

    return true;

}

bool TraceBasedGraph::propagateLimm(StaticInstPtr inst) {
    
    string type = inst->getName();
    assert(type == "limm");

    // Limm (dataSize == 1 || dataSize == 2) has 1 sources and LimmBig (dataSize == 4 || dataSize == 8) has 0 sources
    if (inst->numSrcRegs() > 1) {
        return false;
    }

    if (!usingCCTracking && inst->isCC())
    {
        DPRINTF(ConstProp, "CC Limm Inst! We can't propagate CC insts!\n");
        return false;
    }

    // for 8B LimmBig, Limm, and Lfpimm Only (Memory layput is the same for these classes)
    X86ISAInst::LimmBig * inst_regop = (X86ISAInst::LimmBig * )inst.get(); 

    uint64_t imm = inst->getImmediate();
    const uint8_t dataSize = inst_regop->dataSize;
    assert(dataSize == 8 || dataSize == 4 || dataSize == 2 || dataSize == 1);

    uint64_t forwardVal = 0; 

    if (dataSize >= 4)
    {
        forwardVal = imm & mask(dataSize * 8);
    }
    
    RegId destReg = inst->destRegIdx(0);
    assert(destReg.isIntReg());
    X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get(); 

	if (dataSize < 4 && regCtx[destReg.flatIndex()].valid) {
		forwardVal = x86_inst->merge(regCtx[destReg.flatIndex()].value, imm, dataSize);
	} else if (dataSize < 4) { 
        return false; 
    }

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destReg.flatIndex());

    regCtx[destReg.flatIndex()].value = forwardVal;
    regCtx[destReg.flatIndex()].valid = true;
    regCtx[destReg.flatIndex()].source = false;
        
    return true;
}

bool TraceBasedGraph::propagateAdd(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "add");
    

    // Add (dataSize == 1 || dataSize == 2) has 3 sources and AddBig (dataSize == 4 || dataSize == 8) has 2 sources
    

    if (!usingCCTracking && inst->isCC())
    {
        DPRINTF(ConstProp, "CC Add Inst! We can't propagate CC insts!\n");
        return false;
    }

    // Add and AddBig are both inhereted from RegOp
    // For both src 0 and src 1 are the source operands
    X86ISA::RegOp * inst_regop = (X86ISA::RegOp * )inst.get(); 
    const uint8_t dataSize = inst_regop->dataSize;
    assert(dataSize == 8 || dataSize == 4 || dataSize == 2 || dataSize == 1);

    unsigned src1 = inst->srcRegIdx(0).flatIndex();
    unsigned src2 = inst->srcRegIdx(1).flatIndex();
    if ((!regCtx[src1].valid) || (!regCtx[src2].valid)) {
        if (usingCCTracking && inst->isCC()) {
            ccValid = false;
        }
        return false;
    }

    uint64_t SrcReg1 = regCtx[src1].value;
    uint64_t SrcReg2 = regCtx[src2].value;
    uint64_t forwardVal = 0;
	X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();

    uint64_t psrc1, psrc2;
    if (dataSize >= 4)
    {
        X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
        psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
        psrc2 = x86_inst->pick(SrcReg2, 1, dataSize);
        forwardVal = (psrc1 + psrc2) & mask(dataSize * 8);;
        
    } else {
		RegId destReg = inst->destRegIdx(0);
        assert(destReg.isIntReg());

		if (!regCtx[destReg.flatIndex()].valid) { 
            if (usingCCTracking && inst->isCC()) {
                ccValid = false;
            }
            return false;
        }

		uint64_t DestReg = regCtx[destReg.flatIndex()].value;

		psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
		psrc2 = x86_inst->pick(SrcReg2, 1, dataSize);
		
        forwardVal = x86_inst->merge(DestReg, psrc1+psrc2, dataSize);
    }

    if (usingCCTracking && inst->isCC())
    {
        uint16_t ext = inst->getExt();

        if (((ext & ccFlagMask) == ccFlagMask) || ((ext & ccFlagMask) == 0)) {
            PredccFlagBits = 0; 
        }
        if ((((ext & CFBit) != 0 && (ext & OFBit) != 0) || ((ext & (CFBit | OFBit)) == 0))) {
            PredcfofBits = 0;
        }
        PreddfBit = 0;
        PredecfBit = 0;
        PredezfBit = 0;

        uint64_t newFlags = inst->genFlags(PredccFlagBits | PredcfofBits |
                                           PreddfBit | PredecfBit | PredezfBit,
                                           ext, forwardVal, psrc1, psrc2, true);
        PredcfofBits = newFlags & cfofMask;
        PredecfBit = newFlags & ECFBit;
        PredezfBit = newFlags & EZFBit;
        PreddfBit = newFlags & DFBit;
        PredccFlagBits = newFlags & ccFlagMask;
        ccValid = true;
    }

    RegId destReg = inst->destRegIdx(0);
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destReg.flatIndex());

    regCtx[destReg.flatIndex()].value = forwardVal;
    regCtx[destReg.flatIndex()].valid = true;
    regCtx[destReg.flatIndex()].source = false;

    
    return true;
}

bool TraceBasedGraph::propagateSub(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "sub");
    
    // Sub (dataSize == 1 || dataSize == 2) has 3 sources and SubBig (dataSize == 4 || dataSize == 8) has 2 sources
    

    // Subb and SubbBig are both inhereted from RegOp
    // For both src 0 and src 1 are the source operands
    X86ISA::RegOp * inst_regop = (X86ISA::RegOp * )inst.get(); 
    const uint8_t dataSize = inst_regop->dataSize;
    assert(dataSize == 8 || dataSize == 4 || dataSize == 2 || dataSize == 1);

    if (!usingCCTracking && inst->isCC())
    {
        DPRINTF(ConstProp, "CC And Inst! We can't propagate CC insts!\n");
        return false;
    }

    unsigned src1 = inst->srcRegIdx(0).flatIndex();
    unsigned src2 = inst->srcRegIdx(1).flatIndex();
    if ((!regCtx[src1].valid) || (!regCtx[src2].valid)) {
        if (usingCCTracking && inst->isCC()) {
            ccValid = false;
        }
        return false;
    }

    uint64_t SrcReg1 = regCtx[src1].value;
    uint64_t SrcReg2 = regCtx[src2].value;
    uint64_t forwardVal = 0;
	X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
    uint64_t psrc1, psrc2;
    if (dataSize >= 4)
    {
        X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
        psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
        psrc2 = x86_inst->pick(SrcReg2, 1, dataSize);
        forwardVal = (psrc1 - psrc2) & mask(dataSize * 8);;
    } else {
		RegId destReg = inst->destRegIdx(0);
        assert(destReg.isIntReg());

		if (!regCtx[destReg.flatIndex()].valid) { 
            if (usingCCTracking && inst->isCC()) {
                ccValid = false;
            }
            return false; 
        }

		uint64_t DestReg = regCtx[destReg.flatIndex()].value;
		
        psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
		psrc2 = x86_inst->pick(SrcReg2, 1, dataSize);

	forwardVal = x86_inst->merge(DestReg, (psrc1-psrc2), dataSize);
    }

    if (usingCCTracking && inst->isCC())
    {
        uint16_t ext = inst->getExt();

        if (((ext & ccFlagMask) == ccFlagMask) || ((ext & ccFlagMask) == 0)) {
            PredccFlagBits = 0; 
        }
        if ((((ext & CFBit) != 0 && (ext & OFBit) != 0) || ((ext & (CFBit | OFBit)) == 0))) {
            PredcfofBits = 0;
        }
        PreddfBit = 0;
        PredecfBit = 0;
        PredezfBit = 0;

        uint64_t newFlags = inst->genFlags(PredccFlagBits | PredcfofBits |
                                           PreddfBit | PredecfBit | PredezfBit,
                                           ext, forwardVal, psrc1, ~psrc2, true);
        PredcfofBits = newFlags & cfofMask;
        PredecfBit = newFlags & ECFBit;
        PredezfBit = newFlags & EZFBit;
        PreddfBit = newFlags & DFBit;
        PredccFlagBits = newFlags & ccFlagMask;
        ccValid = true;
    }

    RegId destReg = inst->destRegIdx(0);
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destReg.flatIndex());

    regCtx[destReg.flatIndex()].value = forwardVal;
    regCtx[destReg.flatIndex()].valid = true;
    regCtx[destReg.flatIndex()].source = false;
    
    return true;
}

bool TraceBasedGraph::propagateAnd(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "and");
    
    // And (dataSize == 1 || dataSize == 2) has 3 sources and AndBig (dataSize == 4 || dataSize == 8) has 2 sources
    

    if (!usingCCTracking && inst->isCC())
    {
        DPRINTF(ConstProp, "CC And Inst! We can't propagate CC insts!\n");
        return false;
    }

    // And and AndBig are both inhereted from RegOp
    // For both src 0 and src 1 are the source operands
    X86ISA::RegOp * inst_regop = (X86ISA::RegOp * )inst.get(); 
    const uint8_t dataSize = inst_regop->dataSize;
    assert(dataSize == 8 || dataSize == 4 || dataSize == 2 || dataSize == 1);

    unsigned src1 = inst->srcRegIdx(0).flatIndex();
    unsigned src2 = inst->srcRegIdx(1).flatIndex();
    if ((!regCtx[src1].valid) || (!regCtx[src2].valid)) {
        if (usingCCTracking && inst->isCC()) {
            ccValid = false;
        }
        return false;
    }

    uint64_t SrcReg1 = regCtx[src1].value;
    uint64_t SrcReg2 = regCtx[src2].value;
    uint64_t forwardVal = 0;
	X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();

    uint64_t psrc1, psrc2;
    if (dataSize >= 4)
    { 
        psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
        psrc2 = x86_inst->pick(SrcReg2, 1, dataSize);
        forwardVal = (psrc1 & psrc2) & mask(dataSize * 8);
    } else {
        RegId destReg = inst->destRegIdx(0);
        assert(destReg.isIntReg());

		if (!regCtx[destReg.flatIndex()].valid) { 
            if (usingCCTracking && inst->isCC()) {
                ccValid = false;
            }
            return false;
        }

		uint64_t DestReg = regCtx[destReg.flatIndex()].value;

		psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
		psrc2 = x86_inst->pick(SrcReg2, 1, dataSize);

		forwardVal = x86_inst->merge(DestReg, (psrc1&psrc2), dataSize);
    }
    
    if (usingCCTracking && inst->isCC())
    {
        uint16_t ext = inst->getExt();

        if (((ext & ccFlagMask) == ccFlagMask) || ((ext & ccFlagMask) == 0)) {
            PredccFlagBits = 0; 
        }
        if ((((ext & CFBit) != 0 && (ext & OFBit) != 0) || ((ext & (CFBit | OFBit)) == 0))) {
            PredcfofBits = 0;
        }
        PreddfBit = 0;
        PredecfBit = 0;
        PredezfBit = 0;

        uint64_t mask = CFBit | ECFBit | OFBit;
        uint64_t newFlags = inst->genFlags(PredccFlagBits | PreddfBit |
                             PredezfBit, ext & ~mask, forwardVal, psrc1, psrc2);
        PredezfBit = newFlags & EZFBit;
        PreddfBit = newFlags & DFBit;
        PredccFlagBits = newFlags & ccFlagMask;
        //If a logic microop wants to set these, it wants to set them to 0.
        PredcfofBits = PredcfofBits & ~((CFBit | OFBit) & ext);
        PredecfBit = PredecfBit & ~(ECFBit & ext);
        ccValid = true;
    }

    RegId destReg = inst->destRegIdx(0);
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destReg.flatIndex());

    regCtx[destReg.flatIndex()].value = forwardVal;
    regCtx[destReg.flatIndex()].valid = true;
    regCtx[destReg.flatIndex()].source = false;

    return true;
}

bool TraceBasedGraph::propagateOr(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "or");
    
    // Or (dataSize == 1 || dataSize == 2) has 3 sources and OrBig (dataSize == 4 || dataSize == 8) has 2 sources
    
    
    if (!usingCCTracking && inst->isCC())
    {
        DPRINTF(ConstProp, "CC Or Inst! We can't propagate CC insts!\n");
        return false;
    }

    // Or and OrBig are both inhereted from RegOp
    // For both src 0 and src 1 are the source operands
    X86ISA::RegOp * inst_regop = (X86ISA::RegOp * )inst.get(); 
    const uint8_t dataSize = inst_regop->dataSize;
    assert(dataSize == 8 || dataSize == 4 || dataSize == 2 || dataSize == 1);

    unsigned src1 = inst->srcRegIdx(0).flatIndex();
    unsigned src2 = inst->srcRegIdx(1).flatIndex();
    if ((!regCtx[src1].valid) || (!regCtx[src2].valid)) {
        if (usingCCTracking && inst->isCC()) {
            ccValid = false;
        }
        return false;
    }

    uint64_t SrcReg1 = regCtx[src1].value;
    uint64_t SrcReg2 = regCtx[src2].value;
    uint64_t forwardVal = 0;
	X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
    uint64_t psrc1, psrc2;
    if (dataSize >= 4)
    {
        psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
        psrc2 = x86_inst->pick(SrcReg2, 1, dataSize);
        forwardVal = (psrc1 | psrc2) & mask(dataSize * 8);
    } else {
		RegId destReg = inst->destRegIdx(0);
        assert(destReg.isIntReg());

		if (!regCtx[destReg.flatIndex()].valid) { 
            if (usingCCTracking && inst->isCC()) {
                ccValid = false;
            }
            return false;
        }

		uint64_t DestReg = regCtx[destReg.flatIndex()].value;

		psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
		psrc2 = x86_inst->pick(SrcReg2, 1, dataSize);

		forwardVal = x86_inst->merge(DestReg, (psrc1|psrc2), dataSize);
    }
    
    if (usingCCTracking && inst->isCC())
    {
        uint16_t ext = inst->getExt();

        if (((ext & ccFlagMask) == ccFlagMask) || ((ext & ccFlagMask) == 0)) {
            PredccFlagBits = 0; 
        }
        if ((((ext & CFBit) != 0 && (ext & OFBit) != 0) || ((ext & (CFBit | OFBit)) == 0))) {
            PredcfofBits = 0;
        }
        PreddfBit = 0;
        PredecfBit = 0;
        PredezfBit = 0;

        uint64_t mask = CFBit | ECFBit | OFBit;
        uint64_t newFlags = inst->genFlags(PredccFlagBits | PreddfBit |
                             PredezfBit, ext & ~mask, forwardVal, psrc1, psrc2);
        PredezfBit = newFlags & EZFBit;
        PreddfBit = newFlags & DFBit;
        PredccFlagBits = newFlags & ccFlagMask;
        //If a logic microop wants to set these, it wants to set them to 0.
        PredcfofBits = PredcfofBits & ~((CFBit | OFBit) & ext);
        PredecfBit = PredecfBit & ~(ECFBit & ext);
        ccValid = true;
    }

    RegId destReg = inst->destRegIdx(0);
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destReg.flatIndex());

    regCtx[destReg.flatIndex()].value = forwardVal;
    regCtx[destReg.flatIndex()].valid = true;
    regCtx[destReg.flatIndex()].source = false;
    
    return true;
}

bool TraceBasedGraph::propagateXor(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "xor");
    
    // Xor (dataSize == 1 || dataSize == 2) has 3 sources and XorBig (dataSize == 4 || dataSize == 8) has 2 sources
    

    if (!usingCCTracking && inst->isCC())
    {
        DPRINTF(ConstProp, "CC Xor Inst! We can't propagate CC insts!\n");
        return false;
    }

    // Xor and XorBig are both inhereted from RegOp
    // For both src 0 and src 1 are the source operands
    X86ISA::RegOp * inst_regop = (X86ISA::RegOp * )inst.get(); 
    const uint8_t dataSize = inst_regop->dataSize;
    assert(dataSize == 8 || dataSize == 4 || dataSize == 2 || dataSize == 1);

    unsigned src1 = inst->srcRegIdx(0).flatIndex();
    unsigned src2 = inst->srcRegIdx(1).flatIndex();
    if (src1 == src2) {
        DPRINTF(ConstProp, "NOP: (src1:%d == src2:%d)\n", src1, src2);
        regCtx[src1].value = regCtx[src2].value = 0;
        regCtx[src1].valid = regCtx[src2].valid = true;
    }

    if ((!regCtx[src1].valid) || (!regCtx[src2].valid)) {
        if (usingCCTracking && inst->isCC()) {
            ccValid = false;
        }
        return false;
    }

    uint64_t SrcReg1 = regCtx[src1].value;
    uint64_t SrcReg2 = regCtx[src2].value;
    uint64_t forwardVal = 0;
    X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
 
    uint64_t psrc1, psrc2;
    if (dataSize >= 4)
    {
        psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
        psrc2 = x86_inst->pick(SrcReg2, 1, dataSize);
        forwardVal = (psrc1 ^ psrc2) & mask(dataSize * 8);
    }
    else {
        RegId destReg = inst->destRegIdx(0);
        assert(destReg.isIntReg());

		if (!regCtx[destReg.flatIndex()].valid) { 
            if (usingCCTracking && inst->isCC()) {
                ccValid = false;
            }
            return false;
        }

		uint64_t DestReg = regCtx[destReg.flatIndex()].value;

		psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
		psrc2 = x86_inst->pick(SrcReg2, 1, dataSize);

		forwardVal = x86_inst->merge(DestReg, (psrc1|psrc2), dataSize);
    }
    
    if (usingCCTracking && inst->isCC())
    {
        uint16_t ext = inst->getExt();

        if (((ext & ccFlagMask) == ccFlagMask) || ((ext & ccFlagMask) == 0)) {
            PredccFlagBits = 0; 
        }
        if ((((ext & CFBit) != 0 && (ext & OFBit) != 0) || ((ext & (CFBit | OFBit)) == 0))) {
            PredcfofBits = 0;
        }
        PreddfBit = 0;
        PredecfBit = 0;
        PredezfBit = 0;

        uint64_t mask = CFBit | ECFBit | OFBit;
        uint64_t newFlags = inst->genFlags(PredccFlagBits | PreddfBit |
                             PredezfBit, ext & ~mask, forwardVal, psrc1, psrc2);
        PredezfBit = newFlags & EZFBit;
        PreddfBit = newFlags & DFBit;
        PredccFlagBits = newFlags & ccFlagMask;
        //If a logic microop wants to set these, it wants to set them to 0.
        PredcfofBits = PredcfofBits & ~((CFBit | OFBit) & ext);
        PredecfBit = PredecfBit & ~(ECFBit & ext);
        ccValid = true;
    }

    RegId destReg = inst->destRegIdx(0);
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destReg.flatIndex());

    regCtx[destReg.flatIndex()].value = forwardVal;
    regCtx[destReg.flatIndex()].valid = true;
    regCtx[destReg.flatIndex()].source = false;
    
    return true;
}

bool TraceBasedGraph::propagateMovI(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "movi");
    
    // MovImm has 2 source registers for all datasize and MovFlagsImm has 7 sources
    //if(inst->numSrcRegs() != 2) return false;

    if (inst->isCC() && (!usingCCTracking || !ccValid))
    {
        DPRINTF(ConstProp, "CC Mov Inst! We can't propagate CC insts!\n");
        return false;
    }

    // Mov is both inhereted from RegOp
    X86ISA::RegOpImm * inst_regop = (X86ISA::RegOpImm * )inst.get(); 
    const uint8_t dataSize = inst_regop->dataSize;
    assert(dataSize == 8 || dataSize == 4 || dataSize == 2 || dataSize == 1);

//    if (dataSize < 4) return false;
// LAYNE -- if you get an email change this

  
    X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get(); 

    uint64_t forwardVal = 0;
    uint8_t imm8 = inst_regop->imm8;
    uint16_t ext = inst->getExt();
    if (!inst->isCC() || inst->checkCondition(PredccFlagBits | PredcfofBits | PreddfBit | PredecfBit | PredezfBit, ext)) {
        forwardVal = x86_inst->merge(forwardVal, imm8, dataSize);
    } else {
        return true;
    }

    RegId destReg = inst->destRegIdx(0);
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destReg.flatIndex());
    regCtx[destReg.flatIndex()].value = forwardVal;
    regCtx[destReg.flatIndex()].valid = true;
    regCtx[destReg.flatIndex()].source = false;

    
    return true;
}

bool TraceBasedGraph::propagateSubI(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "subi");
    
    
    // SubImm (dataSize == 1 || dataSize == 2) has 2 sources and SubImmBig (dataSize == 4 || dataSize == 8) has 1 sources
    

    if (!usingCCTracking && inst->isCC())
    {
        DPRINTF(ConstProp, "CC SUBI Inst! We can't propagate CC insts!\n");
        return false;
    }

    // SubImm and SubImmBig are both inhereted from RegOpImm
    X86ISA::RegOpImm * inst_regop = (X86ISA::RegOpImm * )inst.get(); 
    const uint8_t dataSize = inst_regop->dataSize;
    assert(dataSize == 8 || dataSize == 4 || dataSize == 2 || dataSize == 1);

    unsigned src1 = inst->srcRegIdx(0).flatIndex();
    //unsigned src2 = inst->srcRegIdx(1).flatIndex();
    if ((!regCtx[src1].valid) /*|| (!regCtx[src2].valid)*/) {
        if (usingCCTracking && inst->isCC()) {
            ccValid = false;
        }
        return false;
    }

    uint64_t SrcReg1 = regCtx[src1].value;
    //uint64_t SrcReg2 = regCtx[src2].value;

    uint64_t forwardVal = 0;

    X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
    uint8_t imm8 = inst_regop->imm8;
    uint64_t psrc1;
    if (dataSize >= 4)
    {
        X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
        uint8_t imm8 = inst_regop->imm8;
        psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
        forwardVal = (psrc1 - imm8) & mask(dataSize * 8);;
    } else {
        RegId destReg = inst->destRegIdx(0);
        assert(destReg.isIntReg());

		if (!regCtx[destReg.flatIndex()].valid) { 
            if (usingCCTracking && inst->isCC()) {
                ccValid = false;
            }
            return false;
        }

		uint64_t DestReg = regCtx[destReg.flatIndex()].value;

		psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);

		forwardVal = x86_inst->merge(DestReg, (psrc1 - imm8), dataSize);
    }
    
    if (usingCCTracking && inst->isCC())
    {
        uint16_t ext = inst->getExt();

        if (((ext & ccFlagMask) == ccFlagMask) || ((ext & ccFlagMask) == 0)) {
            PredccFlagBits = 0; 
        }
        if ((((ext & CFBit) != 0 && (ext & OFBit) != 0) || ((ext & (CFBit | OFBit)) == 0))) {
            PredcfofBits = 0;
        }
        PreddfBit = 0;
        PredecfBit = 0;
        PredezfBit = 0;

        uint64_t newFlags = inst->genFlags(PredccFlagBits | PredcfofBits |
                                           PreddfBit | PredecfBit | PredezfBit,
                                           ext, forwardVal, psrc1, ~imm8, true);
        PredcfofBits = newFlags & cfofMask;
        PredecfBit = newFlags & ECFBit;
        PredezfBit = newFlags & EZFBit;
        PreddfBit = newFlags & DFBit;
        PredccFlagBits = newFlags & ccFlagMask;
        ccValid = true;
    }

    RegId destReg = inst->destRegIdx(0);
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destReg.flatIndex());

    regCtx[destReg.flatIndex()].value = forwardVal;
    regCtx[destReg.flatIndex()].valid = true;
    regCtx[destReg.flatIndex()].source = false;

    return true;
}

bool TraceBasedGraph::propagateAddI(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "addi");
    
    // AddImm (dataSize == 1 || dataSize == 2) has 2 sources and AddImmBig (dataSize == 4 || dataSize == 8) has 1 sources
    

    if (!usingCCTracking && inst->isCC())
    {
        DPRINTF(ConstProp, "CC ADDI Inst! We can't propagate CC insts!\n");
        return false;
    }

    // SubImm and SubImmBig are both inhereted from RegOpImm
    X86ISA::RegOpImm * inst_regop = (X86ISA::RegOpImm * )inst.get(); 
    const uint8_t dataSize = inst_regop->dataSize;
    assert(dataSize == 8 || dataSize == 4 || dataSize == 2 || dataSize == 1);

    unsigned src1 = inst->srcRegIdx(0).flatIndex();
    //unsigned src2 = inst->srcRegIdx(1).flatIndex();
    if ((!regCtx[src1].valid) /*|| (!regCtx[src2].valid)*/) {
        if (usingCCTracking && inst->isCC()) {
            ccValid = false;
        }
        return false;
    }

    uint64_t SrcReg1 = regCtx[src1].value;
    //uint64_t SrcReg2 = regCtx[src2].value;
    uint64_t forwardVal = 0;

	X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
    uint8_t imm8 = inst_regop->imm8;

    uint64_t psrc1;
    if (dataSize >= 4)
    {
        psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
        forwardVal = (psrc1 + imm8) & mask(dataSize * 8);;
        
    } else {
        RegId destReg = inst->destRegIdx(0);
        assert(destReg.isIntReg());

		if (!regCtx[destReg.flatIndex()].valid) { 
            if (usingCCTracking && inst->isCC()) {
                ccValid = false;
            }
            return false;
        }

		uint64_t DestReg = regCtx[destReg.flatIndex()].value;

		psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);

		forwardVal = x86_inst->merge(DestReg, (psrc1 + imm8), dataSize);
    }
    if (usingCCTracking && inst->isCC()) {
        uint16_t ext = inst->getExt();

        if (((ext & ccFlagMask) == ccFlagMask) || ((ext & ccFlagMask) == 0)) {
            PredccFlagBits = 0; 
        }
        if ((((ext & CFBit) != 0 && (ext & OFBit) != 0) || ((ext & (CFBit | OFBit)) == 0))) {
            PredcfofBits = 0;
        }
        PreddfBit = 0;
        PredecfBit = 0;
        PredezfBit = 0;

        uint64_t newFlags = inst->genFlags(PredccFlagBits | PredcfofBits |
                                           PreddfBit | PredecfBit | PredezfBit,
                                           ext, forwardVal, psrc1, imm8, true);
        PredcfofBits = newFlags & cfofMask;
        PredecfBit = newFlags & ECFBit;
        PredezfBit = newFlags & EZFBit;
        PreddfBit = newFlags & DFBit;
        PredccFlagBits = newFlags & ccFlagMask;
        ccValid = true;
    }

    RegId destReg = inst->destRegIdx(0);
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destReg.flatIndex());

    regCtx[destReg.flatIndex()].value = forwardVal;
    regCtx[destReg.flatIndex()].valid = true;
    regCtx[destReg.flatIndex()].source = false;

    return true;
}

bool TraceBasedGraph::propagateAndI(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "andi");
    
    // AndImm (dataSize == 1 || dataSize == 2) has 2 sources and AndImmBig (dataSize == 4 || dataSize == 8) has 1 sources
    

    if (!usingCCTracking && inst->isCC())
    {
        DPRINTF(ConstProp, "CC ANDI Inst! We can't propagate CC insts!\n");
        return false;
    }

    // SubImm and SubImmBig are both inhereted from RegOpImm
    X86ISA::RegOpImm * inst_regop = (X86ISA::RegOpImm * )inst.get(); 
    const uint8_t dataSize = inst_regop->dataSize;
    assert(dataSize == 8 || dataSize == 4 || dataSize == 2 || dataSize == 1);

    unsigned src1 = inst->srcRegIdx(0).flatIndex();
    //unsigned src2 = inst->srcRegIdx(1).flatIndex();
    if ((!regCtx[src1].valid) /*|| (!regCtx[src2].valid)*/) {
        if (usingCCTracking && inst->isCC()) {
            ccValid = false;
        }
        return false;
    }

    uint64_t SrcReg1 = regCtx[src1].value;
    //uint64_t SrcReg2 = regCtx[src2].value;
    uint64_t forwardVal = 0;

	X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
    uint8_t imm8 = inst_regop->imm8;
    uint64_t psrc1;
    if (dataSize >= 4)
    {
        psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
        forwardVal = (psrc1 & imm8) & mask(dataSize * 8);;
    } else {
        RegId destReg = inst->destRegIdx(0);
        assert(destReg.isIntReg());

		if (!regCtx[destReg.flatIndex()].valid) { 
            if (usingCCTracking && inst->isCC()) {
                ccValid = false;
            }
            return false;
        }

		uint64_t DestReg = regCtx[destReg.flatIndex()].value;

		psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
		forwardVal = x86_inst->merge(DestReg, (psrc1 & imm8), dataSize);
    }
    
    if (usingCCTracking && inst->isCC())
    {
        uint16_t ext = inst->getExt();

        if (((ext & ccFlagMask) == ccFlagMask) || ((ext & ccFlagMask) == 0)) {
            PredccFlagBits = 0; 
        }
        if ((((ext & CFBit) != 0 && (ext & OFBit) != 0) || ((ext & (CFBit | OFBit)) == 0))) {
            PredcfofBits = 0;
        }
        PreddfBit = 0;
        PredecfBit = 0;
        PredezfBit = 0;

        uint64_t mask = CFBit | ECFBit | OFBit;
        uint64_t newFlags = inst->genFlags(PredccFlagBits | PreddfBit |
                             PredezfBit, ext & ~mask, forwardVal, psrc1, imm8);
        PredezfBit = newFlags & EZFBit;
        PreddfBit = newFlags & DFBit;
        PredccFlagBits = newFlags & ccFlagMask;
        //If a logic microop wants to set these, it wants to set them to 0.
        PredcfofBits = PredcfofBits & ~((CFBit | OFBit) & ext);
        PredecfBit = PredecfBit & ~(ECFBit & ext);
        ccValid = true;
    }

    RegId destReg = inst->destRegIdx(0);
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destReg.flatIndex());

    regCtx[destReg.flatIndex()].value = forwardVal;
    regCtx[destReg.flatIndex()].valid = true;
    regCtx[destReg.flatIndex()].source = false;

    return true;
}

bool TraceBasedGraph::propagateOrI(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "ori");
    
    // SubImm (dataSize == 1 || dataSize == 2) has 2 sources and SubImmBig (dataSize == 4 || dataSize == 8) has 1 sources
    

    if (!usingCCTracking && inst->isCC())
    {
        DPRINTF(ConstProp, "CC ORI Inst! We can't propagate CC insts!\n");
        return false;
    }

    // SubImm and SubImmBig are both inhereted from RegOpImm
    X86ISA::RegOpImm * inst_regop = (X86ISA::RegOpImm * )inst.get(); 
    const uint8_t dataSize = inst_regop->dataSize;
    assert(dataSize == 8 || dataSize == 4 || dataSize == 2 || dataSize == 1);

    unsigned src1 = inst->srcRegIdx(0).flatIndex();
    //unsigned src2 = inst->srcRegIdx(1).flatIndex();
    if ((!regCtx[src1].valid) /*|| (!regCtx[src2].valid)*/) {
        if (usingCCTracking && inst->isCC()) {
            ccValid = false;
        }
        return false;
    }

    uint64_t SrcReg1 = regCtx[src1].value;
    //uint64_t SrcReg2 = regCtx[src2].value;
    uint64_t forwardVal = 0;
    X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
    uint8_t imm8 = inst_regop->imm8;

    uint64_t psrc1; 
    if (dataSize >= 4)
    {
        psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
        forwardVal = (psrc1 | imm8) & mask(dataSize * 8);;
    } else {
        RegId destReg = inst->destRegIdx(0);

		if (!regCtx[destReg.flatIndex()].valid) { 
            if (usingCCTracking && inst->isCC()) {
                ccValid = false;
            }
            return false;
        }

		uint64_t DestReg = regCtx[destReg.flatIndex()].value;

		psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);

		forwardVal = x86_inst->merge(DestReg, (psrc1 | imm8), dataSize);
    }
    
    if (usingCCTracking && inst->isCC())
    {
        uint16_t ext = inst->getExt();

        if (((ext & ccFlagMask) == ccFlagMask) || ((ext & ccFlagMask) == 0)) {
            PredccFlagBits = 0; 
        }
        if ((((ext & CFBit) != 0 && (ext & OFBit) != 0) || ((ext & (CFBit | OFBit)) == 0))) {
            PredcfofBits = 0;
        }
        PreddfBit = 0;
        PredecfBit = 0;
        PredezfBit = 0;

        uint64_t mask = CFBit | ECFBit | OFBit;
        uint64_t newFlags = inst->genFlags(PredccFlagBits | PreddfBit |
                             PredezfBit, ext & ~mask, forwardVal, psrc1, imm8);
        PredezfBit = newFlags & EZFBit;
        PreddfBit = newFlags & DFBit;
        PredccFlagBits = newFlags & ccFlagMask;
        //If a logic microop wants to set these, it wants to set them to 0.
        PredcfofBits = PredcfofBits & ~((CFBit | OFBit) & ext);
        PredecfBit = PredecfBit & ~(ECFBit & ext);
        ccValid = true;
    }

    RegId destReg = inst->destRegIdx(0);
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destReg.flatIndex());

    regCtx[destReg.flatIndex()].value = forwardVal;
    regCtx[destReg.flatIndex()].valid = true;
    regCtx[destReg.flatIndex()].source = false;

    return true;
}

bool TraceBasedGraph::propagateXorI(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "xori");
    
    // XorImm (dataSize == 1 || dataSize == 2) has 2 sources and XorImmBig (dataSize == 4 || dataSize == 8) has 1 sources
    

    if (!usingCCTracking && inst->isCC())
    {
        DPRINTF(ConstProp, "CC XORI Inst! We can't propagate CC insts!\n");
        return false;
    }

    // SubImm and SubImmBig are both inhereted from RegOpImm
    X86ISA::RegOpImm * inst_regop = (X86ISA::RegOpImm * )inst.get(); 
    const uint8_t dataSize = inst_regop->dataSize;
    assert(dataSize == 8 || dataSize == 4 || dataSize == 2 || dataSize == 1);

    unsigned src1 = inst->srcRegIdx(0).flatIndex();
    //unsigned src2 = inst->srcRegIdx(1).flatIndex();
    if ((!regCtx[src1].valid) /*|| (!regCtx[src2].valid)*/) {
        if (usingCCTracking && inst->isCC()) {
            ccValid = false;
        }
        return false;
    }

    uint64_t SrcReg1 = regCtx[src1].value;
    //uint64_t SrcReg2 = regCtx[src2].value;
    uint64_t forwardVal = 0;
    X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
    uint8_t imm8 = inst_regop->imm8;
 
    uint64_t psrc1;
    if (dataSize >= 4)
    {
        psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
        forwardVal = (psrc1 ^ imm8) & mask(dataSize * 8);;
    } else {
        RegId destReg = inst->destRegIdx(0);
        assert(destReg.isIntReg());

		if (!regCtx[destReg.flatIndex()].valid) { 
            if (usingCCTracking && inst->isCC()) {
                ccValid = false;
            }
            return false;
        }

		uint64_t DestReg = regCtx[destReg.flatIndex()].value;

		psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);

		forwardVal = x86_inst->merge(DestReg, (psrc1 ^ imm8), dataSize);
    }
    
    if (usingCCTracking && inst->isCC())
    {
        uint16_t ext = inst->getExt();

        if (((ext & ccFlagMask) == ccFlagMask) || ((ext & ccFlagMask) == 0)) {
            PredccFlagBits = 0; 
        }
        if ((((ext & CFBit) != 0 && (ext & OFBit) != 0) || ((ext & (CFBit | OFBit)) == 0))) {
            PredcfofBits = 0;
        }
        PreddfBit = 0;
        PredecfBit = 0;
        PredezfBit = 0;

        uint64_t mask = CFBit | ECFBit | OFBit;
        uint64_t newFlags = inst->genFlags(PredccFlagBits | PreddfBit |
                             PredezfBit, ext & ~mask, forwardVal, psrc1, imm8);
        PredezfBit = newFlags & EZFBit;
        PreddfBit = newFlags & DFBit;
        PredccFlagBits = newFlags & ccFlagMask;
        //If a logic microop wants to set these, it wants to set them to 0.
        PredcfofBits = PredcfofBits & ~((CFBit | OFBit) & ext);
        PredecfBit = PredecfBit & ~(ECFBit & ext);
        ccValid = true;
    }

    RegId destReg = inst->destRegIdx(0);
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destReg.flatIndex());

    regCtx[destReg.flatIndex()].value = forwardVal;
    regCtx[destReg.flatIndex()].valid = true;
    regCtx[destReg.flatIndex()].source = false;
    
    return true;
}

bool TraceBasedGraph::propagateSllI(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "slli");
    
    // SllImm (dataSize == 1 || dataSize == 2) has 2 sources and SllImmBig (dataSize == 4 || dataSize == 8) has 1 sources
    

    if (!usingCCTracking && inst->isCC())
    {
        DPRINTF(ConstProp, "CC SLLI Inst! We can't propagate CC insts!\n");
        return false;
    }

    // SllImm and SslImmBig are both inhereted from RegOpImm
    X86ISA::RegOpImm * inst_regop = (X86ISA::RegOpImm * )inst.get(); 
    const uint8_t dataSize = inst_regop->dataSize;
    assert(dataSize == 8 || dataSize == 4 || dataSize == 2 || dataSize == 1);

    unsigned src1 = inst->srcRegIdx(0).flatIndex();
    //unsigned src2 = inst->srcRegIdx(1).flatIndex();
    if ((!regCtx[src1].valid) /*|| (!regCtx[src2].valid)*/) {
        if (usingCCTracking && inst->isCC()) {
            ccValid = false;
        }
        return false;
    }

    uint64_t SrcReg1 = regCtx[src1].value;
    //uint64_t SrcReg2 = regCtx[src2].value;
    uint64_t forwardVal = 0;
    
	X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
    uint8_t imm8 = inst_regop->imm8;
    uint64_t psrc1;
    uint8_t shiftAmt;
    if (dataSize >= 4)
    {
        psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);

        shiftAmt = (imm8 & ((dataSize == 8) ? mask(6) : mask(5)));
        forwardVal = (psrc1 << shiftAmt) & mask(dataSize * 8);
    } else {
        RegId destReg = inst->destRegIdx(0);
        assert(destReg.isIntReg());

		if (!regCtx[destReg.flatIndex()].valid) { 
            if (usingCCTracking && inst->isCC()) {
                ccValid = false;
            }
            return false;
        }
		uint64_t DestReg = regCtx[destReg.flatIndex()].value;

		psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);

		shiftAmt = (imm8 & ((dataSize == 8) ? mask(6) : mask(5)));

		forwardVal = x86_inst->merge(DestReg, psrc1 << shiftAmt, dataSize);
    }
    
    // If the shift amount is zero, no flags should be modified.
    if (usingCCTracking && shiftAmt && inst->isCC()) {
        uint16_t ext = inst->getExt();

        if (((ext & ccFlagMask) == ccFlagMask) || ((ext & ccFlagMask) == 0)) {
            PredccFlagBits = 0; 
        }
        if ((((ext & CFBit) != 0 && (ext & OFBit) != 0) || ((ext & (CFBit | OFBit)) == 0))) {
            PredcfofBits = 0;
        }
        PreddfBit = 0;
        PredecfBit = 0;
        PredezfBit = 0;
        //Zero out any flags we might modify. This way we only have to
        //worry about setting them.
        PredcfofBits = PredcfofBits & ~(ext & (CFBit | OFBit));
        PredecfBit = PredecfBit & ~(ext & ECFBit);
        int CFBits = 0;
        //Figure out if we -would- set the CF bits if requested.
        if (shiftAmt <= dataSize * 8 &&
                bits(SrcReg1, dataSize * 8 - shiftAmt)) {
            CFBits = 1;
        }
        //If some combination of the CF bits need to be set, set them.
        if ((ext & (CFBit | ECFBit)) && CFBits) {
            PredcfofBits = PredcfofBits | (ext & CFBit);
            PredecfBit = PredecfBit | (ext & ECFBit);
        }
        //Figure out what the OF bit should be.
        if ((ext & OFBit) && (CFBits ^ bits(forwardVal, dataSize * 8 - 1)))
            PredcfofBits = PredcfofBits | OFBit;
        //Use the regular mechanisms to calculate the other flags.
        uint64_t newFlags = inst->genFlags(PredccFlagBits | PreddfBit |
                                           PredezfBit, ext & ~(CFBit | ECFBit | OFBit),
                                           forwardVal, psrc1, imm8);
        PredezfBit = newFlags & EZFBit;
        PreddfBit = newFlags & DFBit;
        PredccFlagBits = newFlags & ccFlagMask;
        ccValid = true;
    }

    RegId destReg = inst->destRegIdx(0);
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destReg.flatIndex());

    regCtx[destReg.flatIndex()].value = forwardVal;
    regCtx[destReg.flatIndex()].valid = true;
    regCtx[destReg.flatIndex()].source = false;

    return true;
}

bool TraceBasedGraph::propagateSrlI(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "srli");
    
    // SrlImm (dataSize == 1 || dataSize == 2) has 2 sources and SrlImmBig (dataSize == 4 || dataSize == 8) has 1 sources
    

    if (!usingCCTracking && inst->isCC())
    {
        DPRINTF(ConstProp, "CC SRLI Inst! We can't propagate CC insts!\n");
        return false;
    }

    // SrlImm and SrllImmBig are both inhereted from RegOpImm
    X86ISA::RegOpImm * inst_regop = (X86ISA::RegOpImm * )inst.get(); 
    const uint8_t dataSize = inst_regop->dataSize;
    assert(dataSize == 8 || dataSize == 4 || dataSize == 2 || dataSize == 1);

    unsigned src1 = inst->srcRegIdx(0).flatIndex();
    //unsigned src2 = inst->srcRegIdx(1).flatIndex();
    if ((!regCtx[src1].valid) /*|| (!regCtx[src2].valid)*/) {
        if (usingCCTracking && inst->isCC()) {
            ccValid = false;
        }
        return false;
    }

    uint64_t SrcReg1 = regCtx[src1].value;
    //uint64_t SrcReg2 = regCtx[src2].value;
    uint64_t forwardVal = 0;
    X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
    uint8_t imm8 = inst_regop->imm8;

    uint64_t psrc1;
    uint8_t shiftAmt;
    if (dataSize >= 4)
    {
        psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);

        shiftAmt = (imm8 & ((dataSize == 8) ? mask(6) : mask(5)));
        uint64_t logicalMask = mask(dataSize * 8 - shiftAmt);
        forwardVal = (psrc1 >> shiftAmt) & logicalMask;
    } else {
        RegId destReg = inst->destRegIdx(0);
        assert(destReg.isIntReg());

		if (!regCtx[destReg.flatIndex()].valid) {
            if (usingCCTracking && inst->isCC()) {
                ccValid = false;
            }
            return false; 
        }

		uint64_t DestReg = regCtx[destReg.flatIndex()].value;

		psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
		shiftAmt = (imm8 & ((dataSize == 8) ? mask(6) : mask(5)));
		uint64_t logicalMask = mask(dataSize * 8 - shiftAmt);

		forwardVal = x86_inst->merge(DestReg, (psrc1 >> shiftAmt) & logicalMask, dataSize);
    }
    
    // If the shift amount is zero, no flags should be modified.
    if (usingCCTracking && shiftAmt && inst->isCC()) {
        uint16_t ext = inst->getExt();

        if (((ext & ccFlagMask) == ccFlagMask) || ((ext & ccFlagMask) == 0)) {
            PredccFlagBits = 0; 
        }
        if ((((ext & CFBit) != 0 && (ext & OFBit) != 0) || ((ext & (CFBit | OFBit)) == 0))) {
            PredcfofBits = 0;
        }
        PreddfBit = 0;
        PredecfBit = 0;
        PredezfBit = 0;
        //Zero out any flags we might modify. This way we only have to
        //worry about setting them.
        PredcfofBits = PredcfofBits & ~(ext & (CFBit | OFBit));
        PredecfBit = PredecfBit & ~(ext & ECFBit);
        //If some combination of the CF bits need to be set, set them.
        if ((ext & (CFBit | ECFBit)) &&
                shiftAmt <= dataSize * 8 &&
                bits(SrcReg1, shiftAmt - 1)) {
            PredcfofBits = PredcfofBits | (ext & CFBit);
            PredecfBit = PredecfBit | (ext & ECFBit);
        }
        //Figure out what the OF bit should be.
        if ((ext & OFBit) && bits(SrcReg1, dataSize * 8 - 1))
            PredcfofBits = PredcfofBits | OFBit;
        //Use the regular mechanisms to calculate the other flags.
        uint64_t newFlags = inst->genFlags(PredccFlagBits | PreddfBit |
                                           PredezfBit, ext & ~(CFBit | ECFBit | OFBit),
                                           forwardVal, psrc1, imm8);
        PredezfBit = newFlags & EZFBit;
        PreddfBit = newFlags & DFBit;
        PredccFlagBits = newFlags & ccFlagMask;
        ccValid = true;
    }

    RegId destReg = inst->destRegIdx(0);
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destReg.flatIndex());

    regCtx[destReg.flatIndex()].value = forwardVal;
    regCtx[destReg.flatIndex()].valid = true;
    regCtx[destReg.flatIndex()].source = false;
    
    return true;
}

bool TraceBasedGraph::propagateSExtI(StaticInstPtr inst) {

    string type = inst->getName();
    assert(type == "sexti");
    
    // SextImm (dataSize == 1 || dataSize == 2) has 2 sources and SextImmBig (dataSize == 4 || dataSize == 8) has 1 sources
    

    if (!usingCCTracking && inst->isCC())
    {
        DPRINTF(ConstProp, "CC SEXTI Inst! We can't propagate CC insts!\n");
        return false;
    }

    // SextImm and SextImmBig are both inhereted from RegOpImm
    X86ISA::RegOpImm * inst_regop = (X86ISA::RegOpImm * )inst.get(); 
    const uint8_t dataSize = inst_regop->dataSize;
    assert(dataSize == 8 || dataSize == 4 || dataSize == 2 || dataSize == 1);

    unsigned src1 = inst->srcRegIdx(0).flatIndex();
    //unsigned src2 = inst->srcRegIdx(1).flatIndex();
    if ((!regCtx[src1].valid) /*|| (!regCtx[src2].valid)*/) {
        if (usingCCTracking && inst->isCC()) {
            ccValid = false;
        }
        return false;
    }

    uint64_t SrcReg1 = regCtx[src1].value;
    //uint64_t SrcReg2 = regCtx[src2].value;

    uint64_t forwardVal = 0;
    X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
    uint8_t imm8 = inst_regop->imm8;
    uint64_t psrc1;
    int sign_bit;
    if (dataSize >= 4)
    {
        psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);

        IntReg val = psrc1;
        // Mask the bit position so that it wraps.
        int bitPos = imm8 & (dataSize * 8 - 1);
        sign_bit = bits(val, bitPos, bitPos);
        uint64_t maskVal = mask(bitPos+1);
        val = sign_bit ? (val | ~maskVal) : (val & maskVal);
        forwardVal = val & mask(dataSize * 8);
    } else {
        RegId destReg = inst->destRegIdx(0);
        assert(destReg.isIntReg());

		if (!regCtx[destReg.flatIndex()].valid) { 
            if (usingCCTracking && inst->isCC()) {
                ccValid = false;
            }
            return false;
        }

		uint64_t DestReg = regCtx[destReg.flatIndex()].value;
		psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);

		IntReg val = psrc1;
		int bitPos = imm8 & (dataSize * 8 - 1);
		sign_bit = bits(val, bitPos, bitPos);
		uint64_t maskVal = mask(bitPos+1);

		val = sign_bit ? (val | ~maskVal) : (val & maskVal);
		forwardVal = x86_inst->merge(DestReg, val, dataSize);
    }
    
    if (usingCCTracking && inst->isCC())
    {
        uint16_t ext = inst->getExt();

        if (((ext & ccFlagMask) == ccFlagMask) || ((ext & ccFlagMask) == 0)) {
            PredccFlagBits = 0; 
        }
        if ((((ext & CFBit) != 0 && (ext & OFBit) != 0) || ((ext & (CFBit | OFBit)) == 0))) {
            PredcfofBits = 0;
        }
        PreddfBit = 0;
        PredecfBit = 0;
        PredezfBit = 0;
        if (!sign_bit) {
            PredccFlagBits = PredccFlagBits & ~(ext & (ZFBit));
            PredcfofBits = PredcfofBits & ~(ext & (CFBit));
            PredecfBit = PredecfBit & ~(ext & ECFBit);
            PredezfBit = PredezfBit & ~(ext & EZFBit);
        } else {
            PredccFlagBits = PredccFlagBits | (ext & (ZFBit));
            PredcfofBits = PredcfofBits | (ext & (CFBit));
            PredecfBit = PredecfBit | (ext & ECFBit);
            PredezfBit = PredezfBit | (ext & EZFBit);
        }
    }

    RegId destReg = inst->destRegIdx(0);
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destReg.flatIndex());

    regCtx[destReg.flatIndex()].value = forwardVal;
    regCtx[destReg.flatIndex()].valid = true;
    regCtx[destReg.flatIndex()].source = false;
    
    return true;
}

bool TraceBasedGraph::propagateZExtI(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "zexti");

    // ZextImm (dataSize == 1 || dataSize == 2) has 2 sources and ZextImmBig (dataSize == 4 || dataSize == 8) has 1 sources
    if (inst->numSrcRegs() > 1) {
        return false;
    }

    if (!usingCCTracking && inst->isCC())
    {
        DPRINTF(ConstProp, "CC SEXTI Inst! We can't propagate CC insts!\n");
        return false;
    }

    // ZextImm and ZextImmBig are both inhereted from RegOpImm
    X86ISA::RegOpImm * inst_regop = (X86ISA::RegOpImm * )inst.get(); 
    const uint8_t dataSize = inst_regop->dataSize;
    assert(dataSize == 8 || dataSize == 4 || dataSize == 2 || dataSize == 1);

    unsigned src1 = inst->srcRegIdx(0).flatIndex();
    //unsigned src2 = inst->srcRegIdx(1).flatIndex();
    if ((!regCtx[src1].valid) /*|| (!regCtx[src2].valid)*/) {
        return false;
    }

    uint64_t SrcReg1 = regCtx[src1].value;
    //uint64_t SrcReg2 = regCtx[src2].value;

    uint64_t forwardVal = 0;

    X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
    uint8_t imm8 = inst_regop->imm8;

    uint64_t psrc1;
    if (dataSize >= 4)
    {
        psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
        forwardVal = bits(psrc1, imm8, 0) & mask(dataSize * 8);;
    }
    else {
        RegId destReg = inst->destRegIdx(0);
        assert(destReg.isIntReg());

		if (!regCtx[destReg.flatIndex()].valid) { 
            return false; 
        }

		uint64_t DestReg = regCtx[destReg.flatIndex()].value;

		psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);	

		forwardVal = x86_inst->merge(DestReg, bits(psrc1, imm8, 0), dataSize);
    }

    RegId destReg = inst->destRegIdx(0);
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destReg.flatIndex());

    regCtx[destReg.flatIndex()].value = forwardVal;
    regCtx[destReg.flatIndex()].valid = true;
    regCtx[destReg.flatIndex()].source = false;
    
    return true;
}

bool TraceBasedGraph::propagateWrip(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "wrip");

    if (inst->isCC() && (!usingCCTracking || !ccValid))
    {
        DPRINTF(ConstProp, "CC WRIP Inst! We can't propagate CC insts!\n");
        return false;
    }

    X86ISA::RegOpImm * inst_regop = (X86ISA::RegOpImm * )inst.get(); 
    const uint8_t dataSize = inst_regop->dataSize;
    assert(dataSize == 8 || dataSize == 4 || dataSize == 2 || dataSize == 1);

    if (dataSize < 4) return false;

    unsigned src1 = inst->srcRegIdx(0).flatIndex();
    unsigned src2 = inst->srcRegIdx(1).flatIndex();
    if ((!regCtx[src1].valid) || (!regCtx[src2].valid)) {
        DPRINTF(ConstProp, "sources (%d, %d) invalid\n", regCtx[src1].valid, regCtx[src2].valid); 
        return false;
    }

    if (currentTrace.branchesFolded >= 2)
        return false;

    uint64_t SrcReg1 = regCtx[src1].value;
    uint64_t SrcReg2 = regCtx[src2].value;
    uint16_t ext = inst->getExt();

    Addr target;

    if (dataSize >= 4) {
        if (!inst->isCC() || inst->checkCondition(PredccFlagBits | PredcfofBits | PreddfBit | PredecfBit | PredezfBit, ext)) {
            X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
            uint64_t psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
            uint64_t psrc2 = x86_inst->pick(SrcReg2, 1, dataSize);
            target = psrc1 + psrc2; // assuming CSBase = 0;
        } else {
            DPRINTF(ConstProp, "Condition failed, advancing trace\n");
            target = currentTrace.instAddr.pcAddr + inst->macroOp->getMacroopSize();
        }
        int idx = (target >> 5) & 0x1f;
        uint64_t tag = (target >> 10);
        for (int way = 0; way < 8; way++) {
            DPRINTF(ConstProp, "CC Tracking: looking up address %#x in uop[%i][%i]: valid:%d tag(%#x==%#x?)\n", target, idx, way, decoder->uopValidArray[idx][way], tag, decoder->uopTagArray[idx][way]);
            if (decoder->uopValidArray[idx][way] && decoder->uopTagArray[idx][way] == tag) {
                for (int uop = 0; uop < decoder->uopCountArray[idx][way]; uop++) {
                    if (decoder->uopAddrArray[idx][way][uop].pcAddr == target &&
                            decoder->uopAddrArray[idx][way][uop].uopAddr == 0) {
                        currentTrace.addr.idx = idx;
                        currentTrace.addr.way = way;
                        currentTrace.addr.uop = uop;
                        currentTrace.addr.valid = true;

                        currentTrace.controlSources[currentTrace.branchesFolded].confidence = 9;
                        currentTrace.controlSources[currentTrace.branchesFolded].valid = true;
                        currentTrace.controlSources[currentTrace.branchesFolded].value = target;

                        currentTrace.branchesFolded++;
                        DPRINTF(ConstProp, "CC Tracking: jumping to address %#x: uop[%i][%i][%i]\n", target, idx, way, uop);
                        return true;
                    }
                }
            }
        }
    } else {
        // still don't know what to do with this microop
        assert(0);
        // assert(inst->srcRegIdx(1).isIntReg());
        // unsigned DestReg = inst->srcRegIdx(1).flatIndex();
        // uint64_t psrc1 = pick(SrcReg1, 0, dataSize);
        // DestReg = merge(DestReg, bits(psrc1, imm8, 0), dataSize);;
    }

    return false;
}

bool TraceBasedGraph::propagateWripI(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "wripi");

    if (inst->isCC() && (!usingCCTracking || !ccValid))
    {
        DPRINTF(ConstProp, "CC WRIP Inst! We can't propagate CC insts!\n");
        return false;
    }

    X86ISA::RegOpImm * inst_regop = (X86ISA::RegOpImm * )inst.get(); 
    const uint8_t dataSize = inst_regop->dataSize;
    assert(dataSize == 8 || dataSize == 4 || dataSize == 2 || dataSize == 1);

    if (dataSize < 4) return false;

    unsigned src1 = inst->srcRegIdx(0).flatIndex();
    //unsigned src2 = inst->srcRegIdx(1).flatIndex();
    if ((!regCtx[src1].valid)/* || (!regCtx[src2].valid)*/) {
        return false;
    }

    if (currentTrace.branchesFolded >= 2)
        return false;

    uint64_t SrcReg1 = regCtx[src1].value;
    //uint64_t SrcReg2 = regCtx[src2].value;
    uint16_t ext = inst->getExt();

    Addr target;

    if (dataSize >= 4) {
        uint8_t imm8 = inst_regop->imm8;
        
        if (!inst->isCC() || inst->checkCondition(PredccFlagBits | PredcfofBits | PreddfBit | PredecfBit | PredezfBit, ext)) {
            X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
            uint64_t psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
            target = psrc1 + imm8; // assuming CSBase = 0;
        } else {
            DPRINTF(ConstProp, "Condition failed, advancing trace\n");
            target = currentTrace.instAddr.pcAddr + inst->macroOp->getMacroopSize();
        }
        int idx = (target >> 5) & 0x1f;
        uint64_t tag = (target >> 10);
        for (int way = 0; way < 8; way++) {
            DPRINTF(ConstProp, "CC Tracking: looking up address %#x in uop[%i][%i]: valid:%d tag(%#x==%#x?)\n", target, idx, way, decoder->uopValidArray[idx][way], tag, decoder->uopTagArray[idx][way]);
            if (decoder->uopValidArray[idx][way] && decoder->uopTagArray[idx][way] == tag) {
                for (int uop = 0; uop < decoder->uopCountArray[idx][way]; uop++) {
                    if (decoder->uopAddrArray[idx][way][uop].pcAddr == target &&
                            decoder->uopAddrArray[idx][way][uop].uopAddr == 0) {
                        currentTrace.addr.idx = idx;
                        currentTrace.addr.way = way;
                        currentTrace.addr.uop = uop;
                        currentTrace.addr.valid = true;

                        currentTrace.controlSources[currentTrace.branchesFolded].confidence = 9;
                        currentTrace.controlSources[currentTrace.branchesFolded].valid = true;
                        currentTrace.controlSources[currentTrace.branchesFolded].value = target;

                        currentTrace.branchesFolded++;
                        DPRINTF(ConstProp, "CC Tracking: jumping to address %#x: uop[%i][%i][%i]\n", target, idx, way, uop);
                        return true;
                    }
                }
            }
        }
    } else {
        // still don't know what to do with this microop
        assert(0);
        // assert(inst->srcRegIdx(1).isIntReg());
        // unsigned DestReg = inst->srcRegIdx(1).flatIndex();
        // uint64_t psrc1 = pick(SrcReg1, 0, dataSize);
        // DestReg = merge(DestReg, bits(psrc1, imm8, 0), dataSize);;
    }

    return false;
}
