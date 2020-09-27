#include "arch/isa_traits.hh"
#include "arch/x86/types.hh"
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

void TraceBasedGraph::predictValue(Addr addr, unsigned uopAddr, int64_t value)
{
    /* Check if we have an optimized trace with this prediction source -- isTraceAvailable returns the most profitable trace. */
    for (auto it = traceMap.begin(); it != traceMap.end(); it++) {
        SpecTrace trace = it->second;
        if (trace.state == SpecTrace::Invalid) {
            continue;
        } 
        for (int i=0; i<4; i++) {
            /* Do we already consider this as a prediction source? */
            if (trace.source[i].valid && trace.source[i].addr == FullUopAddr(addr, uopAddr) && trace.source[i].value == value) {
                return;
            }
        }
    }

    unsigned traceId = 0; 
    int idx = (addr >> 5) & 0x1f;
    for (int way=0; way<8; way++) {
        traceId = decoder->speculativeTraceIDArray[idx][way];
        SpecTrace trace = traceMap[traceId];
        for (int i=0; i<4; i++) {
            /* Do we already consider this as a prediction source? */
            if (trace.source[i].valid && trace.source[i].addr == FullUopAddr(addr, uopAddr) && trace.source[i].value == value) {
                return;
            }
        }
    }

    if (traceId) {
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
                    newTrace.state = SpecTrace::QueuedForFirstTimeOptimization;
                    newTrace.head = newTrace.addr = FullCacheIdx(idx, way, uop);

                    // before adding it to the queue, check if profitable -- we prefer long hot traces
                    unsigned hotness = decoder->uopHotnessArray[idx][way].read();
                    unsigned length = computeLength(newTrace);
                    if (hotness > 7 && length > 2) { // TODO: revisit: pretty low bar
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
                    return;
                }
            }
        }
    }
}

bool TraceBasedGraph::isPredictionSource(SpecTrace trace, FullUopAddr addr) {
    for (int i=0; i<4; i++) {
        if (trace.source[i].valid && trace.source[i].addr == addr) {
            return true;
        }
    }
    return false;
}

void TraceBasedGraph::flushMisprediction(Addr addr, unsigned uop) {
    for (auto it = traceMap.begin(); it != traceMap.end(); it++) {
        SpecTrace trace = it->second;
        for (int i=0; i<4; i++) {
            if (trace.source[i].valid && trace.source[i].addr == FullUopAddr(addr, uop)) {
                trace.source[i].valid = false;
            }
        }
    }
}

void TraceBasedGraph::invalidateBranch(Addr addr) {
    for (auto it = traceMap.begin(); it != traceMap.end(); it++) {
        SpecTrace trace = it->second;
        for (int i=0; i<4; i++) {
            if (trace.source[i].valid && trace.source[i].isBranch && trace.source[i].addr.pcAddr == addr) {
                trace.source[i].valid = false;
            }
        }
    }
}

void TraceBasedGraph::advanceTrace(SpecTrace &trace) {
    trace.addr.uop++;
    trace.addr.valid = false;
    // select cache to advance from
    if (trace.state == SpecTrace::QueuedForFirstTimeOptimization || trace.state == SpecTrace::OptimizationInProcess) {
        if (trace.addr.uop < decoder->uopCountArray[trace.addr.idx][trace.addr.way]) {
            trace.addr.valid = true;
        } else if (decoder->uopNextWayArray[trace.addr.idx][trace.addr.way] != 10) {
            trace.addr.uop = 0;
            trace.addr.way = decoder->uopNextWayArray[trace.addr.idx][trace.addr.way];
            trace.addr.valid = true;
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

        if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
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
        // Pop a new trace from the queue, start at top
        DPRINTF(SuperOp, "Done optimizing trace %i with actual length %i, shrunk to length %i\n", currentTrace.id, currentTrace.length, currentTrace.shrunkLength);
        for (int way=0; way<8; way++) {
            int idx = currentTrace.head.idx;
            if (decoder->speculativeValidArray[idx][way] && decoder->speculativeTraceIDArray[idx][way] == currentTrace.id) {
                currentTrace.addr = FullCacheIdx(idx, way, 0);
                currentTrace.state = SpecTrace::Complete;
                traceMap[currentTrace.id] = currentTrace;
                dumpTrace(currentTrace);
                break;
            }
        }

        if (traceQueue.empty()) {
            currentTrace.addr.valid = false;
            return false; 
        }

        currentTrace = traceQueue.front();
        traceQueue.pop();
        tracesPoppedFromQueue++;

        DPRINTF(SuperOp, "Optimizing trace %i at (%i,%i,%i)\n", currentTrace.id, currentTrace.addr.idx, currentTrace.addr.way, currentTrace.addr.uop);
        dumpTrace(currentTrace);

        if (currentTrace.state == SpecTrace::QueuedForFirstTimeOptimization) {
            currentTrace.state = SpecTrace::OptimizationInProcess;
        } else if (currentTrace.state == SpecTrace::QueuedForReoptimization) {
            currentTrace.state = SpecTrace::ReoptimizationInProcess;
        }
        // Invalidate leftover predictions
        for (int i=0; i<256; i++) {
            registerValid[i] = false;
        }
    } else { 
        assert(currentTrace.state == SpecTrace::OptimizationInProcess || currentTrace.state == SpecTrace::ReoptimizationInProcess);
        currentTrace.inst = NULL;
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
        currentTrace.inst = decodedMacroOp;
        if (decodedMacroOp->isMacroop()) {
            currentTrace.inst = decodedMacroOp->fetchMicroop(decoder->uopAddrArray[idx][way][uop].uopAddr);
            currentTrace.inst->macroOp = decodedMacroOp;
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
    
    bool updateSuccessful = false;
    bool foundNOP = false;

    // Any inst in a trace may be a prediction source
    DPRINTF(ConstProp, "Processing instruction: %p:%i -- %s\n", currentTrace.instAddr.pcAddr, currentTrace.instAddr.uopAddr, currentTrace.inst->getName());
    if (isPredictionSource(currentTrace, currentTrace.instAddr)) {
        // Step 1: Get predicted value from LVP
        // Step 2: Determine dest register(s)
        // Step 3: Annotate dest register entries with that value
        // Step 4: Add inst to speculative trace
        uint64_t predictedValue = decoder->cpu->getLVP()->getValuePredicted(decoder->uopAddrArray[idx][way][uop].pcAddr);
        for (int i = 0; i < currentTrace.inst->numDestRegs(); i++) {
            RegId destReg = currentTrace.inst->destRegIdx(i);
            if (destReg.classValue() == IntRegClass) {
                registerValue[destReg.flatIndex()] = predictedValue;
                registerValid[destReg.flatIndex()] = true;
            }
        }
        updateSuccessful = updateSpecTrace(currentTrace);
    } else {
        // Propagate predicted values
        string type = currentTrace.inst->getName();
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
        } else if (type == "NOP") {
            DPRINTF(ConstProp, "Found a NOP at [%i][%i][%i], compacting...\n", idx, way, uop);
            foundNOP = true;
        } else if (type == "panic" || type == "CPUID") {
            DPRINTF(ConstProp, "Type is PANIC or CPUID\n");
            // TODO: possibly remove, what is purpose?
        } else if (type == "st" || type == "stis" || type == "stfp" || type == "ld" || type == "ldis" || type == "ldst" || type == "syscall" || type == "halt" || type == "fault" || type == "call_far_Mp" || "rdip") {
            DPRINTF(ConstProp, "Type is ST, STIS, STFP, LD, LDIS, LDST, SYSCALL, HALT, FAULT, CALL_FAR_MP, or RDIP\n");
            // TODO: cannot remove
        } else {
            DPRINTF(ConstProp, "Inst type not covered: %s\n", type);
        }

        if (!foundNOP) {
            updateSuccessful = updateSpecTrace(currentTrace);
        }
    }

    // Simulate a stall if update to speculative cache wasn't successful
    if (updateSuccessful || foundNOP) {
        advanceTrace(currentTrace);
    }
    //if (decodedMacroOp && decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();

    return true;
}

bool TraceBasedGraph::updateSpecTrace(SpecTrace &trace) {
    // IMPORTANT NOTE: This is written assuming the trace will be traversed in order, and so the register map will be accurate for the current point in the trace
    trace.length++;

    bool updateSuccessful = false;

    // Rather than checking dests, check sources; if all sources, then all dests in trace
    bool allSrcsReady = true;
    for (int i=0; i<trace.inst->numSrcRegs(); i++) {
        RegId srcReg = trace.inst->srcRegIdx(i);
        allSrcsReady = allSrcsReady && registerValid[srcReg.flatIndex()];
    }

    string type = trace.inst->getName();
    bool isDeadCode = allSrcsReady && (type == "mov" || type == "movi" || type == "limm" || type == "add" || type == "addi" || type == "sub" || type == "subi" || type == "and" || type == "andi" || type == "or" || type == "ori" || type == "xor" || type == "xori" || type == "slri" || type == "slli" || type == "sexti" || type == "zexti");

    // No predicted value propagation required for conditional moves or returns
    // not sure if that's true; jump inst may recieve propagated value? -- let's only do this for cmovs and ret
    if ((trace.inst->isCC() && type == "movi") || trace.inst->isReturn()) {
        return updateSuccessful;
    }

    // Prevent an inst registering as dead if it is a prediction source
    isDeadCode &= !isPredictionSource(trace, trace.instAddr);

    // Inst will never already be in this trace, single pass
    if (isDeadCode) {
        updateSuccessful = decoder->updateTagInSpeculativeCacheWithoutAdding(trace.instAddr.pcAddr, trace.instAddr.uopAddr, trace.id);
        DPRINTF(ConstProp, "Dead code at %#x:%#x\n", trace.instAddr.pcAddr, trace.instAddr.uopAddr);
    } else {
        updateSuccessful = decoder->addUopToSpeculativeCache(trace.inst, trace.instAddr.pcAddr, trace.instAddr.uopAddr, trace.id);
        trace.shrunkLength++;

            // Step 3b: Mark all predicted values on the StaticInst
        for (int i=0; i<trace.inst->numSrcRegs(); i++) {
            unsigned srcIdx = trace.inst->srcRegIdx(i).flatIndex();
            DPRINTF(ConstProp, "Examining register %i\n", srcIdx);
            if (registerValid[srcIdx]) {
                DPRINTF(ConstProp, "Propagated constant %#x in reg %i at %#x:%#x\n", registerValue[srcIdx], srcIdx, trace.instAddr.pcAddr, trace.instAddr.uopAddr);
                trace.inst->sourcePredictions[i] = registerValue[srcIdx];
                trace.inst->sourcesPredicted[i] = true;
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
    if ((!registerValid[destRegId]) || (size < 8 && !registerValid[srcRegId])) {
        return false;
    }
    uint64_t destVal = registerValue[destRegId];
    uint64_t srcVal = registerValue[srcRegId];

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
            registerValue[destReg.flatIndex()] = forwardVal;
            registerValid[destReg.flatIndex()] = true;
        }
    }
    return true;
}

bool TraceBasedGraph::propagateLimm(StaticInstPtr inst) {
    if (inst->numSrcRegs() > 0) {
        return false;
    }

    uint64_t forwardVal = inst->getImmediate();
    unsigned destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            registerValue[destReg.flatIndex()] = forwardVal;
            registerValid[destReg.flatIndex()] = true;
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
    if ((!registerValid[srcRegId]) || (!registerValid[destRegId])) {
        return false;
    }

    uint64_t destVal = registerValue[destRegId];
    uint64_t srcVal = registerValue[srcRegId];

    // value construction is easy for this one
    uint64_t forwardVal = destVal + srcVal;

    destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            registerValue[destReg.flatIndex()] = forwardVal;
            registerValid[destReg.flatIndex()] = true;
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
    if ((!registerValid[srcRegId]) || (!registerValid[destRegId])) {
        return false;
    }
    uint64_t destVal = registerValue[destRegId];
    uint64_t srcVal = registerValue[srcRegId];

    // value construction is easy for this one
    uint64_t forwardVal = destVal - srcVal;

    destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            registerValue[destReg.flatIndex()] = forwardVal;
            registerValid[destReg.flatIndex()] = true;
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
    if ((!registerValid[srcRegId]) || (!registerValid[destRegId])) {
        return false;
    }
    uint64_t destVal = registerValue[destRegId];
    uint64_t srcVal = registerValue[srcRegId];

    // value construction is easy for this one
    uint64_t forwardVal = destVal & srcVal;

    destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            registerValue[destReg.flatIndex()] = forwardVal;
            registerValid[destReg.flatIndex()] = true;
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
    if ((!registerValid[srcRegId]) || (!registerValid[destRegId])) {
        return false;
    }
    uint64_t destVal = registerValue[destRegId];
    uint64_t srcVal = registerValue[srcRegId];

    // value construction is easy for this one
    uint64_t forwardVal = destVal | srcVal;

    destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            registerValue[destReg.flatIndex()] = forwardVal;
            registerValid[destReg.flatIndex()] = true;
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
    if ((!registerValid[srcRegId]) || (!registerValid[destRegId])) {
        return false;
    }
    uint64_t destVal = registerValue[destRegId];
    uint64_t srcVal = registerValue[srcRegId];

    // value construction is easy for this one
    uint64_t forwardVal = destVal ^ srcVal;

    destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            registerValue[destReg.flatIndex()] = forwardVal;
            registerValid[destReg.flatIndex()] = true;
        }
    }
    return true;
}

bool TraceBasedGraph::propagateMovI(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "movi");
    
    if (inst->numSrcRegs() > 0) {
        return false;
    }

    // Conditional moves
    if (inst->isCC()) {
        return true;
    }

    uint64_t forwardVal = inst->getImmediate();
    unsigned destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            registerValue[destReg.flatIndex()] = forwardVal;
            registerValid[destReg.flatIndex()] = true;
        }
    }
    return true;
}

bool TraceBasedGraph::propagateSubI(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "subi");
    
    if (inst->numSrcRegs() > 1) {
        return false;
    }

    unsigned destRegId = inst->srcRegIdx(0).flatIndex();
    if (!registerValid[destRegId]) {
        return false;
    }
    uint64_t destVal = registerValue[destRegId];
    uint64_t srcVal = inst->getImmediate();

    // value construction is easy for this one
    uint64_t forwardVal = destVal - srcVal;

    destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            registerValue[destReg.flatIndex()] = forwardVal;
            registerValid[destReg.flatIndex()] = true;
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
    if (!registerValid[destRegId]) {
        return false;
    }
    uint64_t destVal = registerValue[destRegId];
    uint64_t srcVal = inst->getImmediate();

    // value construction is easy for this one
    uint64_t forwardVal = destVal + srcVal;

    destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            registerValue[destReg.flatIndex()] = forwardVal;
            registerValid[destReg.flatIndex()] = true;
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
    if (!registerValid[destRegId]) {
        return false;
    }
    uint64_t destVal = registerValue[destRegId];
    uint64_t srcVal = inst->getImmediate();

    // value construction is easy for this one
    uint64_t forwardVal = destVal & srcVal;

    destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            registerValue[destReg.flatIndex()] = forwardVal;
            registerValid[destReg.flatIndex()] = true;
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
    if (!registerValid[destRegId]) {
        return false;
    }
    uint64_t destVal = registerValue[destRegId];
    uint64_t srcVal = inst->getImmediate();

    // value construction is easy for this one
    uint64_t forwardVal = destVal | srcVal;

    destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            registerValue[destReg.flatIndex()] = forwardVal;
            registerValid[destReg.flatIndex()] = true;
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
    if (!registerValid[destRegId]) {
        return false;
    }
    uint64_t destVal = registerValue[destRegId];
    uint64_t srcVal = inst->getImmediate();

    // value construction is easy for this one
    uint64_t forwardVal = destVal ^ srcVal;

    destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            registerValue[destReg.flatIndex()] = forwardVal;
            registerValid[destReg.flatIndex()] = true;
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
    if (!registerValid[destRegId]) {
        return false;
    }
    uint64_t destVal = registerValue[destRegId];
    uint64_t srcVal = inst->getImmediate();

    // value construction is easy for this one
    uint64_t forwardVal = destVal << srcVal;

    destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            registerValue[destReg.flatIndex()] = forwardVal;
            registerValid[destReg.flatIndex()] = true;
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
    if (!registerValid[destRegId]) {
        return false;
    }
    uint64_t destVal = registerValue[destRegId];
    uint64_t srcVal = inst->getImmediate();

    // value construction is easy for this one
    uint64_t forwardVal = destVal >> srcVal;

    destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            registerValue[destReg.flatIndex()] = forwardVal;
            registerValid[destReg.flatIndex()] = true;
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
    if (!registerValid[destRegId]) {
        return false;
    }
    uint64_t destVal = registerValue[destRegId];
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
            registerValue[destReg.flatIndex()] = forwardVal;
            registerValid[destReg.flatIndex()] = true;
        }
    }
    return true;
}

bool TraceBasedGraph::propagateZExtI(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "zexti");
    
    if (inst->numSrcRegs() > 1) {
        return false;
    }

    unsigned destRegId = inst->srcRegIdx(0).flatIndex();
    if (!registerValid[destRegId]) {
        return false;
    }
    uint64_t destVal = registerValue[destRegId];
    uint64_t srcVal = inst->getImmediate();

    // value construction taken from isa file
    uint64_t forwardVal = bits(destVal, srcVal, 0);

    destRegId = inst->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
    for (int i = 0; i < inst->numDestRegs(); i++) {
        RegId destReg = inst->destRegIdx(i);
        if (destReg.classValue() == IntRegClass) {
            registerValue[destReg.flatIndex()] = forwardVal;
            registerValid[destReg.flatIndex()] = true;
        }
    }
    return true;
}

bool TraceBasedGraph::isTakenBranch(FullUopAddr addr, FullCacheIdx specIdx) {
    // assuming tid=0, and using spec cache entry at the StaticInstPtr
    TheISA::PCState target;
    return branchPred->iPred.lookup(addr.pcAddr, branchPred->getGHR(0, decoder->speculativeCache[specIdx.idx][specIdx.way][specIdx.uop]->branch_hist), target, 0);
}
