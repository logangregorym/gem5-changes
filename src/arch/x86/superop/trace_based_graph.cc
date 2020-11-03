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

using namespace std;

unsigned SpecTrace::traceIDCounter = 1;

TraceBasedGraph::TraceBasedGraph(TraceBasedGraphParams *p) : SimObject(p), usingControlTracking(p->usingControlTracking) {
    DPRINTF(SuperOp, "Control tracking: %i\n", usingControlTracking);
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

void TraceBasedGraph::predictValue(Addr addr, unsigned uopAddr, int64_t value, unsigned confidence, unsigned latency)
{
    /* Check if we have an optimized trace with this prediction source -- isTraceAvailable returns the most profitable trace. */
    for (auto it = traceMap.begin(); it != traceMap.end(); it++) {
        SpecTrace trace = it->second;
        if (trace.state == SpecTrace::Invalid) {
            continue;
        } 
        for (int i=0; i<4; i++) {
            /* Do we already consider this as a prediction source? */
            if (trace.source[i].valid && trace.source[i].addr == FullUopAddr(addr, uopAddr) &&
                trace.source[i].value == value && trace.source[i].confidence >= 5) {
                return;
            }
        }
    }

    unsigned traceId = 0; 
    bool lowConfidence = false;
    int idx = (addr >> 5) & 0x1f;
    for (int way=0; way<8; way++) {
        traceId = decoder->speculativeTraceIDArray[idx][way];
        SpecTrace trace = traceMap[traceId];
        for (int i=0; i<4; i++) {
            /* Do we already consider this as a prediction source? */
            if (trace.source[i].valid && trace.source[i].addr == FullUopAddr(addr, uopAddr) && trace.source[i].value == value) {
                if (trace.source[i].confidence < 5) {
                    lowConfidence = true;
                } else {
                    return;
                }
            }
        }
    }

    // if an already optimized trace has low confidence, it is time to phase it out
    if (traceId && !lowConfidence) {
        SpecTrace trace = traceMap[traceId];
        for (int i=0; i<4; i++) {
            /* Have we exhausted all prediction sources? If not, we can further compact this trace.    */
            if (!trace.source[i].valid) {
                SpecTrace newTrace;
                newTrace.id = SpecTrace::traceIDCounter++;
                for (int j=0; j<4; j++) {
                    if (trace.source[j].valid) {
                        newTrace.source[j] = trace.source[j];
                    }
                }
                newTrace.source[i].valid = true;
                newTrace.source[i].addr = FullUopAddr(addr, uopAddr);
                newTrace.source[i].value = value;
                newTrace.source[i].confidence = confidence;
                newTrace.source[i].latency = latency;
                newTrace.state = SpecTrace::QueuedForReoptimization;
                newTrace.reoptId = traceId;

                int way;
                for (way=0; way<8; way++) {
                    for (int uop=0; uop<6; uop++) {
                        if (decoder->speculativeValidArray[idx][way] && decoder->speculativeAddrArray[idx][way][uop].pcAddr == addr &&
                            decoder->speculativeAddrArray[idx][way][uop].uopAddr == uopAddr && decoder->speculativeTraceIDArray[idx][way] == traceId) {
                            newTrace.head = newTrace.addr = FullCacheIdx(idx, way, uop);
                            traceMap[newTrace.id] = newTrace;
                            traceQueue.push(newTrace);
                            DPRINTF(SuperOp, "Queueing up new trace request %i to reoptimize trace %i at spec[%i][%i][%i]\n", newTrace.id, newTrace.reoptId, idx, way, uopAddr);
                            return;
                        }
                    }
                }
            }
        }
    } else {
        for (int way=0; way<8; way++) {
            for (int uop=0; uop<6; uop++) {
                if (decoder->uopValidArray[idx][way] && decoder->uopAddrArray[idx][way][uop].pcAddr == addr && decoder->uopAddrArray[idx][way][uop].uopAddr == uopAddr) {
                    SpecTrace newTrace;
                    newTrace.source[0].valid = true;
                    newTrace.source[0].addr = FullUopAddr(addr, uopAddr);
                    newTrace.source[0].value = value;
                    newTrace.source[0].confidence = confidence;
                    newTrace.source[0].latency = latency;
                    newTrace.state = SpecTrace::QueuedForFirstTimeOptimization;
                    newTrace.head = newTrace.addr = FullCacheIdx(idx, way, uop);

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

bool TraceBasedGraph::isPredictionSource(SpecTrace trace, FullUopAddr addr, uint64_t &value, unsigned &confidence, unsigned &latency) {
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
        trace.addr.valid = false;
        if (decodedMacroOp->isMacroop()) { 
			decodedMacroOp->deleteMicroOps();
			decodedMacroOp = NULL;
		}
        return true;
    }

    // if it is a direct call or a jump, fold the branch (provided it is predicted taken)
    Addr target = pcAddr + decodedMacroOp->machInst.instSize;
    if (std::string(decodedMicroOp->instMnem) == "CALL_NEAR_I" || std::string(decodedMicroOp->instMnem) == "JMP_I" || decodedMicroOp->isCondCtrl()) {
        target += decodedMacroOp->machInst.immediate;
    } 

    // not a taken branch -- advance normally
    void *bpHistory = NULL;
    bool predTaken = branchPred->lookup(0, pcAddr, bpHistory);
    if (!(std::string(decodedMicroOp->instMnem) == "CALL_NEAR_I" || std::string(decodedMicroOp->instMnem) == "JMP_I") && !predTaken) {
        if (decodedMacroOp->isMacroop()) { 
			decodedMacroOp->deleteMicroOps();
			decodedMacroOp = NULL;
		}
        branchPred->squash(0, bpHistory); // assuming tid = 0 
        return false;
    }

    // indirect branch -- lookup indirect predictor
    if (decodedMacroOp->machInst.opcode.op == 0xFF) {
        TheISA::PCState targetPC;
        branchPred->iPred.lookup(pcAddr, branchPred->getGHR(0, bpHistory), targetPC, 0); // assuming tid = 0
        target = targetPC.instAddr();
    }

    // delete history to prevent memory leak
    branchPred->squash(0, bpHistory);

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
                    return true;
                }
            }
        }
    }

    if (decodedMacroOp->isMacroop()) { 
        decodedMacroOp->deleteMicroOps();
        decodedMacroOp = NULL;
    }
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
                
                if (!currentTrace.inst->isControl()) { // control instructions already propagate live outs
                    for (int i=0; i<16; i++) { // 16 int registers
                        if (regCtx[i].valid && !regCtx[i].source) {
                            currentTrace.inst->liveOut[currentTrace.inst->numDestRegs()] = regCtx[i].value;
                            currentTrace.inst->liveOutPredicted[currentTrace.inst->numDestRegs()] = true;
                            currentTrace.inst->addDestReg(RegId(IntRegClass, i));
                        }
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
                for (int i=0; i<4; i++) {
                    DPRINTF(SuperOp, "Prediction Source %i\n", i);
                    if (currentTrace.source[i].valid) {
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
                return false; 
            }

            currentTrace = traceQueue.front();
            traceQueue.pop();
            tracesPoppedFromQueue++;

            int idx = currentTrace.addr.idx;
            int way = currentTrace.addr.way;
            if (!(currentTrace.state == SpecTrace::QueuedForFirstTimeOptimization && decoder->uopValidArray[idx][way]) &&
                !(currentTrace.state == SpecTrace::QueuedForReoptimization && decoder->speculativeValidArray[idx][way])) {
                DPRINTF(SuperOp, "Trace %i at (%i,%i,%i) evicted before we could process it.\n", currentTrace.id, currentTrace.addr.idx, currentTrace.addr.way, currentTrace.addr.uop);
                currentTrace.addr.valid = false;
                currentTrace.state = SpecTrace::Evicted;
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
            regCtx[i].valid = false;
            regCtx[i].source = false;
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
    if (state == SpecTrace::OptimizationInProcess) {
        if (!decoder->uopValidArray[idx][way] || (uop >= decoder->uopCountArray[idx][way])) {
            tracesWithInvalidHead++;
            DPRINTF(SuperOp, "Trace was evicted out of the micro-op cache before we could optimize it\n");
            currentTrace.addr.valid = false;
            return false;
        }
        decodedMacroOp = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
        if (decodedMacroOp->getName() == "NOP") {
            currentTrace.length++;
            advanceTrace(currentTrace);
            return true;
        }
        if (decodedMacroOp->isMacroop()) {
            StaticInstPtr inst = decodedMacroOp->fetchMicroop(decoder->uopAddrArray[idx][way][uop].uopAddr);
            if (inst->getName() == "NOP") {
                currentTrace.length++;
                advanceTrace(currentTrace);
                return true;
            }
            currentTrace.inst = inst;
            currentTrace.inst->macroOp = decodedMacroOp;
        } else {
            currentTrace.inst = decodedMacroOp;
        }
        currentTrace.instAddr = decoder->uopAddrArray[idx][way][uop];
    } else if (state == SpecTrace::ReoptimizationInProcess) {
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
        currentTrace.instAddr == decoder->speculativeAddrArray[idx][way][uop];
    }
    decodedMacroOp = NULL;

    bool updateSuccessful = false;

    // Any inst in a trace may be a prediction source
    DPRINTF(ConstProp, "Trace %i: Processing instruction: %p:%i -- %s\n", currentTrace.id, currentTrace.instAddr.pcAddr, currentTrace.instAddr.uopAddr, currentTrace.inst->getName());
    uint64_t value = 0;
    unsigned confidence = 0;
    unsigned latency;
    string type = currentTrace.inst->getName();
    if (type != "limm" && type != "movi" && isPredictionSource(currentTrace, currentTrace.instAddr, value, confidence, latency)) {
        // Step 1: Get predicted value from LVP
        // Step 2: Determine dest register(s)
        // Step 3: Annotate dest register entries with that value
        // Step 4: Add inst to speculative trace
        for (int i = 0; i < currentTrace.inst->numDestRegs(); i++) {
            RegId destReg = currentTrace.inst->destRegIdx(i);
            if (destReg.classValue() == IntRegClass) {
                regCtx[destReg.flatIndex()].value = currentTrace.inst->predictedValue = value;
                regCtx[destReg.flatIndex()].valid = regCtx[destReg.flatIndex()].source = currentTrace.inst->predictedLoad = true;
                currentTrace.inst->confidence = confidence;
            }
        }
        bool isDeadCode = false;
        updateSuccessful = updateSpecTrace(currentTrace, isDeadCode);
        // if it's not a dead code, then update the last non-eliminated microop of the specTrace
        if (!isDeadCode)
        {
            currentTrace.prevNonEliminatedInst = currentTrace.inst;
        }

    } else {
        // Propagate predicted values
        if (type == "mov") {
            DPRINTF(ConstProp, "Found a MOV at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagateMov(currentTrace.inst);
        } else if (type == "wrip" || type == "wripi") {
            DPRINTF(ConstProp, "Found a WRIP/WRIPI branch at [%i][%i][%i], compacting...\n", idx, way, uop);
            // propagateWrip(currentTrace.inst);
        } else if (currentTrace.inst->isControl()) {
            // printf("Control instruction of type %s\n", type);
            // TODO: check for stopping condition or predicted target
        } else if (type == "movi") {
            DPRINTF(ConstProp, "Found a MOVI at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagateMovI(currentTrace.inst);
        } else if (type == "and") {
            DPRINTF(ConstProp, "Found an AND at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagateAnd(currentTrace.inst);
        } else if (type == "add") {
            DPRINTF(ConstProp, "Found an ADD at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagateAdd(currentTrace.inst);
        } else if (type == "sub") {
            DPRINTF(ConstProp, "Found a SUB at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagateSub(currentTrace.inst);
        } else if (type == "xor") {
            DPRINTF(ConstProp, "Found an XOR at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagateXor(currentTrace.inst);
        } else if (type == "or") {
            DPRINTF(ConstProp, "Found an OR at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagateOr(currentTrace.inst);
        } else if (type == "subi") {
            DPRINTF(ConstProp, "Found a SUBI at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagateSubI(currentTrace.inst);
        } else if (type == "addi") {
            DPRINTF(ConstProp, "Found an ADDI at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagateAddI(currentTrace.inst);
        } else if (type == "slli") {
            DPRINTF(ConstProp, "Found a SLLI at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagateSllI(currentTrace.inst);
        } else if (type == "srli") {
            DPRINTF(ConstProp, "Found a SRLI at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagateSrlI(currentTrace.inst);
        } else if (type == "lea") {
            DPRINTF(ConstProp, "Type is LEA");
            // Requires multiple ALU operations to propagate, not using
        } else if (type == "sexti") {
            // Implementation has multiple ALU operations, but this is not required by the nature of the operation
            DPRINTF(ConstProp, "Found a SEXTI at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagateSExtI(currentTrace.inst);
        } else if (type == "zexti") {
            // Implementation has multiple ALU operations, but this is not required by the nature of the operation
            DPRINTF(ConstProp, "Found a ZEXTI at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagateZExtI(currentTrace.inst);
        } else if (type == "mul1s" || type == "mul1u" || type == "mulel" || type == "muleh") {
            DPRINTF(ConstProp, "Type is MUL1S, MUL1U, MULEL, or MULEH\n");
            // TODO: two dest regs with different values? maybe too complex arithmetic?
        } else if (type == "limm") {
            DPRINTF(ConstProp, "Found a LIMM at [%i][%i][%i], compacting...\n", idx, way, uop);
            propagateLimm(currentTrace.inst);
        } else if (type == "rflags" || type == "wrflags" || type == "ruflags" || type == "wruflags") {
            DPRINTF(ConstProp, "Type    is RFLAGS, WRFLAGS, RUFLAGS, or WRUFLAGS\n");
            // TODO: add control registers to graph?
        } else if (type == "rdtsc" || type == "rdval") {
            DPRINTF(ConstProp, "Type is RDTSC or RDVAL\n");
            // TODO: determine whether direct register file access needs to be handled differently?
        } else if (type == "panic" || type == "CPUID") {
            DPRINTF(ConstProp, "Type is PANIC or CPUID\n");
            // TODO: possibly remove, what is purpose?
        } else if (type == "st" || type == "stis" || type == "stfp" || type == "ld" || type == "ldis" || type == "ldst" || type == "syscall" || type == "halt" || type == "fault" || type == "call_far_Mp" || "rdip") {
            DPRINTF(ConstProp, "Type is ST, STIS, STFP, LD, LDIS, LDST, SYSCALL, HALT, FAULT, CALL_FAR_MP, or RDIP\n");
            // TODO: cannot remove
        } else {
            DPRINTF(ConstProp, "Inst type not covered: %s\n", type);
        }

        bool isDeadCode = false;
        updateSuccessful = updateSpecTrace(currentTrace, isDeadCode);
        // if it's not a dead code, then update the last non-eliminated microop of the specTrace
        if (!isDeadCode)
        {
            currentTrace.prevNonEliminatedInst = currentTrace.inst;
        }
    }

    // Simulate a stall if update to speculative cache wasn't successful
    if (updateSuccessful) {
        advanceTrace(currentTrace);
    }

    // Propagate live outs at the end of each control instruction
    if (currentTrace.inst->isControl()) {
        for (int i=0; i<16; i++) { // 16 int registers
            if (regCtx[i].valid && !regCtx[i].source) {
                currentTrace.inst->liveOut[currentTrace.inst->numDestRegs()] = regCtx[i].value;
                currentTrace.inst->liveOutPredicted[currentTrace.inst->numDestRegs()] = true;
                currentTrace.inst->addDestReg(RegId(IntRegClass, i));
            }
        }
    }
    DPRINTF(SuperOp, "Live Outs:\n");
    for (int i=0; i<16; i++) {
        if (regCtx[i].valid && !regCtx[i].source)
            DPRINTF(SuperOp, "reg[%i]=%#x\n", i, regCtx[i].value);
    }

    return true;
}

bool TraceBasedGraph::updateSpecTrace(SpecTrace &trace, bool &isDeadCode ) {
    // IMPORTANT NOTE: This is written assuming the trace will be traversed in order, and so the register map will be accurate for the current point in the trace
    trace.length++;

    // Rather than checking dests, check sources; if all sources, then all dests in trace
    bool allSrcsReady = true;
    for (int i=0; i<trace.inst->numSrcRegs(); i++) {
        RegId srcReg = trace.inst->srcRegIdx(i);
        allSrcsReady = allSrcsReady && regCtx[srcReg.flatIndex()].valid;
    }

    string type = trace.inst->getName();
    isDeadCode = allSrcsReady && (type == "mov" || type == "movi" || type == "limm" || type == "add" || type == "addi" || type == "sub" || type == "subi" || type == "and" || type == "andi" || type == "or" || type == "ori" || type == "xor" || type == "xori" || type == "slri" || type == "slli" || type == "sexti" || type == "zexti");

    // Prevent an inst registering as dead if it is a prediction source or if it is a return or it modifies CC
    uint64_t value;
    unsigned confidence;
    unsigned latency;
    bool isPredSource = isPredictionSource(trace, trace.instAddr, value, confidence, latency) && type != "limm" && type != "movi";
    isDeadCode &= (!isPredSource && !(trace.inst->isCC() || trace.inst->isReturn()));

    // Inst will never already be in this trace, single pass
    if (isDeadCode) {
        DPRINTF(ConstProp, "Dead code at %#x:%#x\n", trace.instAddr.pcAddr, trace.instAddr.uopAddr);
        DPRINTF(Decoder, "Skipping microop update in the speculative cache\n");
        return true;
    }

    bool updateSuccessful = decoder->addUopToSpeculativeCache(trace.inst, trace.instAddr.pcAddr, trace.instAddr.uopAddr, trace.id);
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
        trace.optimizedHead.idx = trace.head.idx; 
        trace.optimizedHead.uop = 0;
        for (int way=0; way<8; way++) {
            int idx = trace.head.idx;
            if (decoder->speculativeValidArray[idx][way] && decoder->speculativeTraceIDArray[idx][way] == trace.id) {
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
    
    if (inst->numSrcRegs() > 3) {
        return false;
    }

    unsigned destRegId = inst->srcRegIdx(0).flatIndex();
    unsigned srcRegId = inst->srcRegIdx(1).flatIndex();
    uint8_t size = inst->getDataSize();
    if ((!regCtx[destRegId].valid) || (size < 8 && !regCtx[srcRegId].valid)) {
        return false;
    }
    uint64_t destVal = regCtx[destRegId].value;
    uint64_t srcVal = regCtx[srcRegId].value;

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

    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            regCtx[destReg.flatIndex()].value = forwardVal;
            regCtx[destReg.flatIndex()].valid = true;
        }
    }
    return true;
}

bool TraceBasedGraph::propagateLimm(StaticInstPtr inst) {
    if (inst->numSrcRegs() > 1) { // sometimes limm has an implicit source (the same as dest) to preserve dependencies
        return false;
    }

    // for 8B LimmBig, Limm, and Lfpimm Only (Memory layput is the same for these classes)
    X86ISAInst::Limm * inst_regop = (X86ISAInst::Limm * )inst.get(); 

    uint64_t imm = inst->getImmediate();
    const uint8_t size = inst_regop->dataSize;
    assert(size == 8 || size == 4 || size == 2 || size == 1);

    unsigned destRegId = inst->destRegIdx(0).flatIndex();



    
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {

            uint64_t destVal;
            // if the last value of regCtx is not valid, then we need to get the valid value from ExecContext
            // TODO: This is not completely true, if the dest reg is not valid, it means we need ti make sure 
            // that depending on the size we are not overwriitng bitfilds
            if (!regCtx[destReg.flatIndex()].valid)
            {
                
                destVal = 0;
            }
            else 
            {
                destVal = regCtx[destReg.flatIndex()].value;
            }
            // construct the value
            uint64_t forwardVal;
            
            if (size == 8) {
                forwardVal = imm;
            } else if (size == 4) {
                forwardVal = (destVal & 0xffffffff00000000) | (imm & 0xffffffff); // mask 4 bytes
            } else if (size == 2) {
                forwardVal = (destVal & 0xffffffffffff0000) | (imm & 0xffff); // mask 2 bytes
            } else if (size == 1) {
                forwardVal = (destVal & 0xffffffffffffff00) | (imm & 0xff); // mask 1 byte
            } 
            else 
            {
                assert(0);
            }

            DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);

            regCtx[destReg.flatIndex()].value = forwardVal;
            regCtx[destReg.flatIndex()].valid = true;
        }
    }
    return true;
}

bool TraceBasedGraph::propagateAdd(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "add");
    
    if (inst->numSrcRegs() > 3) {
        return false;
    }

    unsigned destRegId = inst->srcRegIdx(0).flatIndex();
    unsigned srcRegId = inst->srcRegIdx(1).flatIndex();
    if ((!regCtx[srcRegId].valid) || (!regCtx[destRegId].valid)) {
        return false;
    }

    uint64_t destVal = regCtx[destRegId].value;
    uint64_t srcVal = regCtx[srcRegId].value;

    // value construction is easy for this one
    uint64_t forwardVal = destVal + srcVal;

    destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            regCtx[destReg.flatIndex()].value = forwardVal;
            regCtx[destReg.flatIndex()].valid = true;
        }
    }
    return true;
}

bool TraceBasedGraph::propagateSub(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "sub");
    
    if (inst->numSrcRegs() > 3) {
        return false;
    }

    unsigned destRegId = inst->srcRegIdx(0).flatIndex();
    unsigned srcRegId = inst->srcRegIdx(1).flatIndex();
    if ((!regCtx[srcRegId].valid) || (!regCtx[destRegId].valid)) {
        return false;
    }
    uint64_t destVal = regCtx[destRegId].value;
    uint64_t srcVal = regCtx[srcRegId].value;

    // value construction is easy for this one
    uint64_t forwardVal = destVal - srcVal;

    destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            regCtx[destReg.flatIndex()].value = forwardVal;
            regCtx[destReg.flatIndex()].valid = true;
        }
    }
    return true;
}

bool TraceBasedGraph::propagateAnd(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "and");
    
    if (inst->numSrcRegs() > 3) {
        return false;
    }

    unsigned destRegId = inst->srcRegIdx(0).flatIndex();
    unsigned srcRegId = inst->srcRegIdx(1).flatIndex();
    if ((!regCtx[srcRegId].valid) || (!regCtx[destRegId].valid)) {
        return false;
    }
    uint64_t destVal = regCtx[destRegId].value;
    uint64_t srcVal = regCtx[srcRegId].value;

    // value construction is easy for this one
    uint64_t forwardVal = destVal & srcVal;

    destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            regCtx[destReg.flatIndex()].value = forwardVal;
            regCtx[destReg.flatIndex()].valid = true;
        }
    }
    return true;
}

bool TraceBasedGraph::propagateOr(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "or");
    
    if (inst->numSrcRegs() > 3) {
        return false;
    }

    unsigned destRegId = inst->srcRegIdx(0).flatIndex();
    unsigned srcRegId = inst->srcRegIdx(1).flatIndex();
    if ((!regCtx[srcRegId].valid) || (!regCtx[destRegId].valid)) {
        return false;
    }
    uint64_t destVal = regCtx[destRegId].value;
    uint64_t srcVal = regCtx[srcRegId].value;

    // value construction is easy for this one
    uint64_t forwardVal = destVal | srcVal;

    destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            regCtx[destReg.flatIndex()].value = forwardVal;
            regCtx[destReg.flatIndex()].valid = true;
        }
    }
    return true;
}

bool TraceBasedGraph::propagateXor(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "xor");
    
    if (inst->numSrcRegs() > 3) {
        return false;
    }

    unsigned destRegId = inst->srcRegIdx(0).flatIndex();
    unsigned srcRegId = inst->srcRegIdx(1).flatIndex();
    if ((!regCtx[srcRegId].valid) || (!regCtx[destRegId].valid)) {
        return false;
    }
    uint64_t destVal = regCtx[destRegId].value;
    uint64_t srcVal = regCtx[srcRegId].value;

    // value construction is easy for this one
    uint64_t forwardVal = destVal ^ srcVal;

    destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            regCtx[destReg.flatIndex()].value = forwardVal;
            regCtx[destReg.flatIndex()].valid = true;
        }
    }
    return true;
}

bool TraceBasedGraph::propagateMovI(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "movi");
    
    if (inst->numSrcRegs() > 1) {
        return false;
    }

    // Conditional moves
    if (inst->isCC()) {
        return true;
    }

    
    X86ISA::RegOpImm * inst_regop = (X86ISA::RegOpImm * )inst.get(); 
    const uint8_t size = inst_regop->dataSize;
    assert(size == 8 || size == 4 || size == 2 || size == 1);


    uint64_t imm = inst->getImmediate();
    unsigned destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "MOVI: Forwarding value %lx through register %i\n", imm, destRegId);

    RegId destReg = inst->destRegIdx(0);
    assert(destReg.isIntReg());


    uint64_t destVal;
    // if the last value of regCtx is not valid, then we need to get the valid value from ExecContext
    // TODO: This is not completely true, if the dest reg is not valid, it means we need ti make sure 
    // that depending on the size we are not overwriitng bitfilds
    if (!regCtx[destReg.flatIndex()].valid)
    {
                
        destVal = 0;
    }
    else 
    {
        destVal = regCtx[destReg.flatIndex()].value;
    }
    // construct the value
    uint64_t forwardVal;
            
    if (size == 8) {
        forwardVal = imm;
    } else if (size == 4) {
        forwardVal = (destVal & 0xffffffff00000000) | (imm & 0xffffffff); // mask 4 bytes
    } else if (size == 2) {
        forwardVal = (destVal & 0xffffffffffff0000) | (imm & 0xffff); // mask 2 bytes
    } else if (size == 1) {
        forwardVal = (destVal & 0xffffffffffffff00) | (imm & 0xff); // mask 1 byte
    } 
    else 
    {
        assert(0);
    }
    
    regCtx[destReg.flatIndex()].value = forwardVal;
    regCtx[destReg.flatIndex()].valid = true;


    return true;
}

bool TraceBasedGraph::propagateSubI(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "subi");
    
    if (inst->numSrcRegs() > 1) {
        return false;
    }

    unsigned destRegId = inst->srcRegIdx(0).flatIndex();
    if (!regCtx[destRegId].valid) {
        return false;
    }
    uint64_t destVal = regCtx[destRegId].value;
    uint64_t srcVal = inst->getImmediate();

    // value construction is easy for this one
    uint64_t forwardVal = destVal - srcVal;

    destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            regCtx[destReg.flatIndex()].value = forwardVal;
            regCtx[destReg.flatIndex()].valid = true;
        }
    }
    return true;
}

bool TraceBasedGraph::propagateAddI(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "addi");
    
    if (inst->numSrcRegs() > 1) {
        return false;
    }

    unsigned destRegId = inst->srcRegIdx(0).flatIndex();
    if (!regCtx[destRegId].valid) {
        return false;
    }
    uint64_t destVal = regCtx[destRegId].value;
    uint64_t srcVal = inst->getImmediate();

    // value construction is easy for this one
    uint64_t forwardVal = destVal + srcVal;

    destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            regCtx[destReg.flatIndex()].value = forwardVal;
            regCtx[destReg.flatIndex()].valid = true;
        }
    }
    return true;
}

bool TraceBasedGraph::propagateAndI(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "andi");
    
    if (inst->numSrcRegs() > 1) {
        return false;
    }

    unsigned destRegId = inst->srcRegIdx(0).flatIndex();
    if (!regCtx[destRegId].valid) {
        return false;
    }
    uint64_t destVal = regCtx[destRegId].value;
    uint64_t srcVal = inst->getImmediate();

    // value construction is easy for this one
    uint64_t forwardVal = destVal & srcVal;

    destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            regCtx[destReg.flatIndex()].value = forwardVal;
            regCtx[destReg.flatIndex()].valid = true;
        }
    }
    return true;
}

bool TraceBasedGraph::propagateOrI(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "ori");
    
    if (inst->numSrcRegs() > 1) {
        return false;
    }

    unsigned destRegId = inst->srcRegIdx(0).flatIndex();
    if (!regCtx[destRegId].valid) {
        return false;
    }
    uint64_t destVal = regCtx[destRegId].value;
    uint64_t srcVal = inst->getImmediate();

    // value construction is easy for this one
    uint64_t forwardVal = destVal | srcVal;

    destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            regCtx[destReg.flatIndex()].value = forwardVal;
            regCtx[destReg.flatIndex()].valid = true;
        }
    }
    return true;
}

bool TraceBasedGraph::propagateXorI(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "xori");
    
    if (inst->numSrcRegs() > 1) {
        return false;
    }

    unsigned destRegId = inst->srcRegIdx(0).flatIndex();
    if (!regCtx[destRegId].valid) {
        return false;
    }
    uint64_t destVal = regCtx[destRegId].value;
    uint64_t srcVal = inst->getImmediate();

    // value construction is easy for this one
    uint64_t forwardVal = destVal ^ srcVal;

    destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            regCtx[destReg.flatIndex()].value = forwardVal;
            regCtx[destReg.flatIndex()].valid = true;
        }
    }
    return true;
}

bool TraceBasedGraph::propagateSllI(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "slli");
    
    if (inst->numSrcRegs() > 1) {
        return false;
    }

    unsigned destRegId = inst->srcRegIdx(0).flatIndex();
    if (!regCtx[destRegId].valid) {
        return false;
    }
    uint64_t destVal = regCtx[destRegId].value;
    uint64_t srcVal = inst->getImmediate();

    // value construction is easy for this one
    uint64_t forwardVal = destVal << srcVal;

    destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            regCtx[destReg.flatIndex()].value = forwardVal;
            regCtx[destReg.flatIndex()].valid = true;
        }
    }
    return true;
}

bool TraceBasedGraph::propagateSrlI(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "srli");
    
    if (inst->numSrcRegs() > 1) {
        return false;
    }

    unsigned destRegId = inst->srcRegIdx(0).flatIndex();
    if (!regCtx[destRegId].valid) {
        return false;
    }
    uint64_t destVal = regCtx[destRegId].value;
    uint64_t srcVal = inst->getImmediate();

    // value construction is easy for this one
    uint64_t forwardVal = destVal >> srcVal;

    destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            regCtx[destReg.flatIndex()].value = forwardVal;
            regCtx[destReg.flatIndex()].valid = true;
        }
    }
    return true;
}

bool TraceBasedGraph::propagateSExtI(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "sexti");
    
    if (inst->numSrcRegs() > 1) {
        return false;
    }

    unsigned destRegId = inst->srcRegIdx(0).flatIndex();
    if (!regCtx[destRegId].valid) {
        return false;
    }
    uint64_t destVal = regCtx[destRegId].value;
    uint64_t srcVal = inst->getImmediate();

    // value construction taken from isa file
    int signBit = bits(destVal, srcVal, srcVal);
    uint64_t signMask = mask(srcVal);
    uint64_t forwardVal = signBit ? (destVal | ~signMask) : (destVal & signMask);

    destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            regCtx[destReg.flatIndex()].value = forwardVal;
            regCtx[destReg.flatIndex()].valid = true;
        }
    }
    return true;
}

bool TraceBasedGraph::propagateZExtI(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "zexti");
    if (inst->numSrcRegs() > 1) {
        assert(0);
    }

    assert(inst->srcRegIdx(0).isIntReg());
    unsigned srcRegId = inst->srcRegIdx(0).flatIndex();
    if (!regCtx[srcRegId].valid) {
        return false;
    }

    uint64_t DestReg = 0;
    uint64_t SrcReg1 = regCtx[srcRegId].value;
    uint64_t imm8 = inst->getImmediate();

    uint8_t dataSize = inst->getDataSize();

    if (dataSize >= 4)
    {
        X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
        uint64_t psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
        DestReg = bits(psrc1, imm8, 0) & mask(dataSize * 8);
    }
    else {
        // still don't know what this microop is
        assert(0);
        // assert(inst->srcRegIdx(1).isIntReg());
        // unsigned DestReg = inst->srcRegIdx(1).flatIndex();
        // uint64_t psrc1 = pick(SrcReg1, 0, dataSize);
        // DestReg = merge(DestReg, bits(psrc1, imm8, 0), dataSize);;
    }

    RegId destReg = inst->destRegIdx(0);
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "ConstProp Forwarding ZExtI value %lx through register %i\n", DestReg, destReg.flatIndex());
    
    regCtx[destReg.flatIndex()].value = DestReg;
    regCtx[destReg.flatIndex()].valid = true;

    return true;
}
