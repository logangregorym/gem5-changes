#include "arch/isa_traits.hh"
#include "arch/x86/types.hh"
#include "arch/x86/isa.hh"
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
#include "debug/TraceGen.hh"
#include "cpu/reg_class.hh"
#include "debug/TraceEviction.hh"

using namespace X86ISA;
using namespace std;



uint64_t SpecTrace::traceIDCounter = 1;

TraceBasedGraph::TraceBasedGraph(TraceBasedGraphParams *p) : SimObject(p), 
                                                            usingControlTracking(p->usingControlTracking), 
                                                            usingCCTracking(p->usingCCTracking), 
                                                            predictionConfidenceThreshold(p->predictionConfidenceThreshold) ,
                                                            specCacheNumWays(p->specCacheNumWays),
                                                            specCacheNumSets(p->specCacheNumSets) ,
                                                            numOfTracePredictionSources(p->numOfTracePredictionSources) 

{
    DPRINTF(SuperOp, "Control tracking: %i\n", usingControlTracking);
    DPRINTF(SuperOp, "CC tracking: %i\n", usingCCTracking);
    DPRINTF(SuperOp, "Prediction Confidence Threshold: %i\n", predictionConfidenceThreshold);
    DPRINTF(SuperOp, "Number of ways for speculative cache: %i\n", specCacheNumWays);
    DPRINTF(SuperOp, "Number of sets for speculative cache: %i\n", specCacheNumSets);
    DPRINTF(SuperOp, "Number of prediction sources in a super optimized trace: %i\n", numOfTracePredictionSources);
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
    
    numMicroopsInTraceDist
        .init(0,18,1)
        .name(name() + ".microops_per_trace")
        .desc("Number of microops in super optimized trace")
        .flags(Stats::pdf)
        ;
}


bool TraceBasedGraph::IsValuePredictible(const StaticInstPtr instruction)
{
    assert( (instruction->isStreamedFromUOpCache() && !instruction->isStreamedFromSpeculativeCache()) || 
            (!instruction->isStreamedFromUOpCache() && instruction->isStreamedFromSpeculativeCache()) ||  
            (!instruction->isStreamedFromUOpCache() && !instruction->isStreamedFromSpeculativeCache())
    );

    // Make load value prediction if necessary
    // only predict when the instruction has only one INT dest reg
    // Number of INT registers
    uint16_t numOfIntDestRegs = 0;
    for (int i = 0; i < instruction->numDestRegs(); i++) 
    {
        RegId destReg = instruction->destRegIdx(i);
        if (destReg.classValue() == IntRegClass && destReg.index() != 4) { // exclude stack and FP operations
            numOfIntDestRegs++;
        }
    }


    bool isPredictableType =    (numOfIntDestRegs == 1) && 
                                instruction->isInteger() && 
                                !instruction->isVector() && 
                                !instruction->isFloating() &&
                                /*!instruction->isCC()*/ 
                                instruction->isStreamedFromUOpCache() &&
                                !instruction->isStreamedFromSpeculativeCache();

        // don't pullote predictor with instructions that we already know thier values
    if (instruction->getName() == "rdip" || 
        instruction->getName() == "limm" ||  
        instruction->getName() == "movi") 
    {
        isPredictableType = false;
    }
        

    bool valuePredictable = false;
    if (isPredictableType && instruction->isLoad())
    {
        // this is a load type. Predict for it!
        valuePredictable = true;
    }
    else if ( isPredictableType && loadPred->predictingArithmetic) 
    {   
        // this is a artithmatic type. Predict for it!
        valuePredictable = true;
    }

    return valuePredictable;
}


// In this function we queue a new hot trace just base on the condition that there is a 
// queued trace with the same head address or not
bool TraceBasedGraph::QueueHotTraceForSuperOptimization(const X86ISA::PCState& pc)
{

    uint64_t addr = pc.instAddr();

    DPRINTF(TraceGen, "Trying to queue a new hot trace for inst addr=%#x\n", addr);

    uint64_t uop_cache_idx = (addr >> 5) & 0x1f;
    uint64_t tag = (addr >> 10);
    uint64_t baseAddr = 0;
    uint64_t baseWay = 0;
    // find the the base way for this 32B code region
    for (int way = 0; way < 8; way++) {
        if ((decoder->uopValidArray[uop_cache_idx][way] && decoder->uopTagArray[uop_cache_idx][way] == tag) &&
             (!baseAddr || decoder->uopAddrArray[uop_cache_idx][way][0].pcAddr <= baseAddr)) {
            
            baseAddr = decoder->uopAddrArray[uop_cache_idx][way][0].pcAddr;
            baseWay = way;
        
        }
    }

    // check to see if there is trace with this head address
    for (auto it = traceMap.begin(); it != traceMap.end(); it++) {
        
        if (it->second.getTraceHeadAddr() == FullUopAddr(baseAddr, 0))
        {
            DPRINTF(TraceGen, "Trace Map already holds a trace with id %d for this head address! addr = %#x\n", it->first , addr);
            return false;
        }

    }




    SpecTrace newTrace;
    newTrace.state = SpecTrace::QueuedForFirstTimeOptimization;
    newTrace.head = newTrace.addr = FullCacheIdx(uop_cache_idx, baseWay, 0);
    newTrace.setTraceHeadAddress(FullUopAddr(baseAddr, 0));
    newTrace.instAddr = FullUopAddr(0, 0);

    // before adding it to the queue, check if profitable -- we prefer long hot traces
    uint64_t hotness = decoder->uopHotnessArray[uop_cache_idx][baseWay].read();
    newTrace.hotness = hotness;
    uint64_t length = computeLength(newTrace);
    if (hotness < 7 || length < 4) { // TODO: revisit: pretty low bar
        DPRINTF(TraceGen, "Rejecting trace request to optimize trace at uop[%i][%i][%i]\n", uop_cache_idx, baseWay, 0);
        DPRINTF(TraceGen, "hotness:%i length=%i\n", hotness, length);
        return false;
    }


    newTrace.id = SpecTrace::traceIDCounter++;
    traceMap[newTrace.id] = newTrace;
    traceQueue.push(newTrace);
    DPRINTF(TraceGen, "Queueing up new trace request %i to optimize trace at uop[%i][%i][%i]\n", newTrace.id, uop_cache_idx, baseWay, 0);
    DPRINTF(TraceGen, "hotness:%i length=%i\n", hotness, length);
    dumpTrace(newTrace);
    return true;

    
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

bool TraceBasedGraph::advanceIfControlTransfer(SpecTrace &trace, Addr &target) {
    // don't do this for re-optimizations
    if (trace.state != SpecTrace::QueuedForFirstTimeOptimization  && trace.state != SpecTrace::OptimizationInProcess)
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
        DPRINTF(TraceGen, "Ending trace -- return or a branch without a confident prediction is encountered\n");
        trace.addr.valid = false;
        if (decodedMacroOp->isMacroop()) { 
			decodedMacroOp->deleteMicroOps();
			decodedMacroOp = NULL;
		}
        return true;
    }

    std::string disas = decodedMicroOp->disassemble(pcAddr);

    // if it is a direct call or a jump, fold the branch (provided it is predicted taken)
    target = pcAddr + decodedMacroOp->machInst.instSize;
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
    uint64_t uop_cache_idx = (target >> 5) & 0x1f;
    uint64_t tag = (target >> 10);
    for (int way = 0; way < 8; way++) {
        if (decoder->uopValidArray[uop_cache_idx][way] && decoder->uopTagArray[uop_cache_idx][way] == tag) {
            for (int uop = 0; uop < decoder->uopCountArray[uop_cache_idx][way]; uop++) {
                if (decoder->uopAddrArray[uop_cache_idx][way][uop].pcAddr == target &&
                        decoder->uopAddrArray[uop_cache_idx][way][uop].uopAddr == 0) {
                    trace.addr.idx = uop_cache_idx;
                    trace.addr.way = way;
                    trace.addr.uop = uop;
                    if (decodedMacroOp->isMacroop()) { 
                        decodedMacroOp->deleteMicroOps();
                        decodedMacroOp = NULL;
                    }
                    DPRINTF(TraceGen, "Control Tracking: jumping to address %#x: uop[%i][%i][%i]\n", target, uop_cache_idx, way, uop);
                    
                    trace.controlSources[trace.branchesFolded].confidence = 9;
                    trace.controlSources[trace.branchesFolded].valid = true;
                    trace.controlSources[trace.branchesFolded].value = target;

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

    DPRINTF(TraceGen, "Ending trace -- uop cache miss at branch target\n");
    trace.addr.valid = false;
    return false;
}

Addr TraceBasedGraph::advanceTrace(SpecTrace &trace) {
    Addr target = 0;
    if (!usingControlTracking || !advanceIfControlTransfer(trace, target)) {
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
                    return 0;
                }
                uint64_t tag = (nextAddr.pcAddr >> 10);
                for (int way = 0; way < 8; way++) {
                    if (decoder->uopValidArray[trace.addr.idx][way] && decoder->uopTagArray[trace.addr.idx][way] == tag) {
                        for (int uop = 0; uop < decoder->uopCountArray[trace.addr.idx][way]; uop++) {
                            if (decoder->uopAddrArray[trace.addr.idx][way][uop].pcAddr == nextAddr.pcAddr) {
                                trace.addr.way = way;
                                trace.addr.uop = uop;
                                trace.addr.valid = true;
                                return 0;
                            }
                        }
                    }
                }
            }
        } else {
            assert(trace.state == SpecTrace::Complete);
            assert((trace.addr.way < decoder->SPEC_CACHE_NUM_WAYS) && (trace.addr.idx < decoder->SPEC_CACHE_NUM_SETS));
            if (trace.addr.uop < decoder->speculativeCountArray[trace.addr.idx][trace.addr.way]) {
                trace.addr.valid = true;
            } else if (decoder->speculativeNextWayArray[trace.addr.idx][trace.addr.way] != decoder->SPEC_CACHE_WAY_MAGIC_NUM) {
                trace.addr.uop = 0;
                trace.addr.way = decoder->speculativeNextWayArray[trace.addr.idx][trace.addr.way];
                trace.addr.valid = true;
            }
        }
        return 0;
    }
    return target;
}

void TraceBasedGraph::dumpTrace(SpecTrace trace) {
    // not a valid trace
    if (!trace.addr.valid) {
        for (int i=0; i<4; i++) 
        {
            DPRINTF(SuperOp, "Value Prediction Source %i\n", i);
            if (trace.source[i].valid) {
                DPRINTF(SuperOp, "Address=%#x:%i, Value=%#x, Confidence=%i, Latency=%i\n",
                                trace.source[i].addr.pcAddr,  trace.source[i].addr.uopAddr,
                                trace.source[i].value, trace.source[i].confidence,
                                trace.source[i].latency);
            }
        }
        for (int i=0; i<2; i++) 
        {
            DPRINTF(SuperOp, "Control Prediction Source %i\n", i);
            if (trace.controlSources[i].valid) {
                DPRINTF(SuperOp, "Target=%#x, Confidence=%i\n",
                                trace.controlSources[i].value, trace.controlSources[i].confidence);
            }
        }
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
        DPRINTF(TraceEviction, "%p:%i -- uop[%i][%i][%i] -- %s\n", pcAddr, uopAddr, idx, way, uop, decodedMicroOp->disassemble(pcAddr));    
        DPRINTF(TraceGen, "%p:%i -- uop[%i][%i][%i] -- %s\n", pcAddr, uopAddr, idx, way, uop, decodedMicroOp->disassemble(pcAddr)); 
        if (decodedMacroOp->isMacroop()) { 
			decodedMacroOp->deleteMicroOps();
			decodedMacroOp = NULL;
		}
    } else {

        panic_if(trace.state != SpecTrace::Complete, "Trace State is %d\n", trace.state);
        assert((way< decoder->SPEC_CACHE_NUM_WAYS) && (idx < decoder->SPEC_CACHE_NUM_SETS));

        Addr pcAddr = decoder->speculativeAddrArray[idx][way][uop].pcAddr;
        Addr uopAddr = decoder->speculativeAddrArray[idx][way][uop].uopAddr;
        StaticInstPtr decodedMicroOp = decoder->speculativeCache[idx][way][uop];
        //assert(decodedMicroOp);
        DPRINTF(TraceEviction, "%p:%i -- spec[%i][%i][%i] -- %s -- isCC = %d -- isPredictionSource = %d -- isDummyMicroop = %d\n", pcAddr, uopAddr, idx, way, uop, decodedMicroOp->disassemble(pcAddr), decodedMicroOp->isCC(), decodedMicroOp->isTracePredictionSource(), decodedMicroOp->dummyMicroop);   
        DPRINTF(TraceGen, "%p:%i -- spec[%i][%i][%i] -- %s -- isCC = %d -- isPredictionSource = %d -- isDummyMicroop = %d\n", pcAddr, uopAddr, idx, way, uop, decodedMicroOp->disassemble(pcAddr), decodedMicroOp->isCC(), decodedMicroOp->isTracePredictionSource(), decodedMicroOp->dummyMicroop);   
		for (int i=0; i<decodedMicroOp->numSrcRegs(); i++) {
			// LAYNE : TODO : print predicted inputs (checking syntax)
			if (decodedMicroOp->sourcesPredicted[i]) {
				DPRINTF(TraceEviction, "\tSource for register %i predicted as %x\n", decodedMicroOp->srcRegIdx(i), decodedMicroOp->sourcePredictions[i]);
                DPRINTF(TraceGen, "\tSource for register %i predicted as %x\n", decodedMicroOp->srcRegIdx(i), decodedMicroOp->sourcePredictions[i]);
			}
           
		}
        for (int i=0; i< 5; i++) {
			if (decodedMicroOp->isCCFlagPropagated[i]) {
				DPRINTF(TraceEviction, "\tSource for register %i predicted as %x\n", decodedMicroOp->srcRegIdx(i), decodedMicroOp->sourcePredictions[i]);
                DPRINTF(TraceGen, "\tCC flag %d is propagated as %x\n", i, decodedMicroOp->propgatedCCFlags[i]);
			}
		}
		for (int i=0; i<decodedMicroOp->numDestRegs(); i++) {
			if (decodedMicroOp->liveOutPredicted[i]) {
				DPRINTF(TraceEviction, "\tLive out for register %i predicted as %x\n", decodedMicroOp->destRegIdx(i), decodedMicroOp->liveOut[i]);
                DPRINTF(TraceGen, "\tLive out for register %i predicted as %x\n", decodedMicroOp->destRegIdx(i), decodedMicroOp->liveOut[i]);
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

void TraceBasedGraph::dumpLiveOuts(StaticInstPtr inst, bool dumpOnlyArchRegs) {

    if (inst && !inst->isCarryingLivesOut()) {
        // if the current inst (predicted inst) is the first microop then just dump 16 arch registers and CCs
        if (dumpOnlyArchRegs) {
                for (int i=0; i<16; i++) { // 16 int registers
                    DPRINTF(TraceGen, "Trying to dump live out: regCts[%d]=%#x valid=%d source=%d\n", i, regCtx[i].value, regCtx[i].valid, regCtx[i].source);
                    if (regCtx[i].valid && !regCtx[i].source) {
                        inst->liveOut[inst->numDestRegs()] = regCtx[i].value;
                        inst->liveOutPredicted[inst->numDestRegs()] = true;
                        inst->addDestReg(RegId(IntRegClass, i));
                        inst->_numIntDestRegs++;
                        inst->setCarriesLiveOut(true);
                    }
                }
        } else {
        // if the current instruction is not the first microop, then we need to also dump micro-registers to cover this scenario
        // E.G., 
        // MOV_R_P : rdip   t7, %ctrl153,  (Propagated) (first microop)
        // MOV_R_P : ld   rax, DS:[t7 + 0x3cb4fa] (LVP predicted and currentTrace.inst)
                for (int i=0; i<38; i++) { // 38 int registers
                    DPRINTF(TraceGen, "Trying to dump live out: regCts[%d]=%#x valid=%d source=%d\n", i, regCtx[i].value, regCtx[i].valid, regCtx[i].source);
                    if (regCtx[i].valid && !regCtx[i].source) {
                        inst->liveOut[inst->numDestRegs()] = regCtx[i].value;
                        inst->liveOutPredicted[inst->numDestRegs()] = true;
                        inst->addDestReg(RegId(IntRegClass, i));
                        inst->_numIntDestRegs++;
                        inst->setCarriesLiveOut(true);
                    }
                }
        }

        // always dump all the CC regs no matter what
        if (ccValid) {
                DPRINTF(TraceGen, "Dumping CCs: PredccFlagBits:%#x, PredcfofBits:%#x, PreddfBit:%#x, PredecfBit:%#x, PredezfBit:%#x\n", PredccFlagBits, PredcfofBits, PreddfBit, PredecfBit, PredezfBit);
                inst->liveOut[inst->numDestRegs()] = PredccFlagBits;
                inst->liveOutPredicted[inst->numDestRegs()] = true;
                inst->addDestReg(RegId(CCRegClass, CCREG_ZAPS));
                inst->liveOut[inst->numDestRegs()] = PredcfofBits;
                inst->liveOutPredicted[inst->numDestRegs()] = true;
                inst->addDestReg(RegId(CCRegClass, CCREG_CFOF));
                inst->liveOut[inst->numDestRegs()] = PreddfBit;
                inst->liveOutPredicted[inst->numDestRegs()] = true;
                inst->addDestReg(RegId(CCRegClass, CCREG_DF));
                inst->liveOut[inst->numDestRegs()] = PredecfBit;
                inst->liveOutPredicted[inst->numDestRegs()] = true;
                inst->addDestReg(RegId(CCRegClass, CCREG_ECF));
                inst->liveOut[inst->numDestRegs()] = PredezfBit;
                inst->liveOutPredicted[inst->numDestRegs()] = true;
                inst->addDestReg(RegId(CCRegClass, CCREG_EZF));
                inst->_numCCDestRegs += 5;
                inst->setCarriesLiveOut(true);
        }
    }
}

bool TraceBasedGraph::generateNextTraceInst() {

    //HeapProfilerStart("generateNextTraceInst");
    if (!currentTrace.addr.valid) 
    { 
        // Finalize old trace
        if (currentTrace.state != SpecTrace::Complete && 
            currentTrace.state != SpecTrace::Evicted && 
            currentTrace.state != SpecTrace::Invalid && 
            currentTrace.id != 0) 
        {
            DPRINTF(SuperOp, "Done optimizing trace %i with actual length %i, shrunk to length %i\n", currentTrace.id, currentTrace.length, currentTrace.shrunkLength);

            // make sure always a super-optimized trace is shorter or equal to its original counterpart
            assert(currentTrace.length >= currentTrace.shrunkLength);

            // find number of valid prediction sources for non-zero traces
            size_t validPredSources = 0;
            for (int i = 0; i < 4; i++)
            {
                if (currentTrace.source[i].valid) {
                    validPredSources++;
                }
            }

            // perform this operation at the end so we can queue a new trace to superoptimize
            // if there is no prediction sources in the trace, then it's not a usefull trace just remove the trace
            if (validPredSources == 0)
            {
                // remove it from traceMap
                DPRINTF(TraceGen, "Removing trace: %d from Trace Map because of insufficent prediction sources!.\n", currentTrace.id );
                assert(traceMap.find(currentTrace.id) != traceMap.end());
                traceMap.erase(currentTrace.id);
                currentTrace.addr.valid = false;
                currentTrace.state = SpecTrace::Evicted;
                currentTrace.id = 0;
                // clear spec cache write queue
                for (auto &micro: decoder->specCacheWriteQueue)
                {
                    StaticInstPtr macro = NULL;
                    if (micro.inst->macroOp && micro.inst->macroOp->isMacroop())
                    {
                        macro = micro.inst->macroOp;
                    }
                    if (macro)
                    {
                        macro->deleteMicroOps();
                    }
                    macro = NULL;
                }
                decoder->specCacheWriteQueue.clear();
            }
            // trace is too long to be inserted in the spec cache
            else if (currentTrace.shrunkLength >= 18)
            {
                // now write back to spec cache here!
                assert(decoder->specCacheWriteQueue.size() == currentTrace.shrunkLength);
                // remove it from traceMap
                DPRINTF(TraceGen, "Removing trace: %d from Trace Map because of insufficent space in spec cache for a trace longer than 18 micrrops. Trace length: %d!.\n", currentTrace.id ,  currentTrace.shrunkLength);
                assert(traceMap.find(currentTrace.id) != traceMap.end());
                traceMap.erase(currentTrace.id);
                currentTrace.addr.valid = false;
                currentTrace.state = SpecTrace::Evicted;
                currentTrace.id = 0;
                // clear spec cache write queue
                for (auto &micro: decoder->specCacheWriteQueue)
                {
                    StaticInstPtr macro = NULL;
                    if (micro.inst->macroOp && micro.inst->macroOp->isMacroop())
                    {
                        macro = micro.inst->macroOp;
                    }
                    if (macro)
                    {
                        macro->deleteMicroOps();
                    }
                    macro = NULL;
                }
                decoder->specCacheWriteQueue.clear();


            }
            // Insufficent superoptimization
            else if ((currentTrace.length == currentTrace.shrunkLength))
            {
                // remove it from traceMap
                DPRINTF(TraceGen, "Removing trace: %d from Trace Map because of insufficent super-optimization!.\n", currentTrace.id );
                assert(traceMap.find(currentTrace.id) != traceMap.end());
                traceMap.erase(currentTrace.id);
                currentTrace.addr.valid = false;
                currentTrace.state = SpecTrace::Evicted;
                currentTrace.id = 0;
                // clear spec cache write queue
                for (auto &micro: decoder->specCacheWriteQueue)
                {
                    StaticInstPtr macro = NULL;
                    if (micro.inst->macroOp && micro.inst->macroOp->isMacroop())
                    {
                        macro = micro.inst->macroOp;
                    }
                    if (macro)
                    {
                        macro->deleteMicroOps();
                    }
                    macro = NULL;
                }
                decoder->specCacheWriteQueue.clear();
            }
            else 
            {
                // now write back to spec cache here!
                assert(decoder->specCacheWriteQueue.size() == currentTrace.shrunkLength);
                assert(decoder->specCacheWriteQueue.size() <= 18);
                
                if (currentTrace.interveningDeadInsts) {
                    // E* at the end of the trace.
                    // Pick the last E to dump live outs.
                    assert(currentTrace.prevEliminatedInst);
                    currentTrace.prevEliminatedInst->dummyMicroop = true;
                    decoder->specCacheWriteQueue.push_back(SuperOptimizedMicroop(currentTrace.prevEliminatedInst, currentTrace.lastAddr));

                    currentTrace.shrunkLength++;
                    currentTrace.prevEliminatedInst->shrunkenLength = currentTrace.interveningDeadInsts - 1; // excluding the current one
                    currentTrace.interveningDeadInsts = 0;
                    currentTrace.prevNonEliminatedInst = currentTrace.prevEliminatedInst;
                }

                for (auto &microop: decoder->specCacheWriteQueue)
                {
                    // add trace id to each spec microop for gathering stats in commit stage
                    microop.inst->setTraceID(currentTrace.id);
                    microop.inst->setTraceLength(decoder->specCacheWriteQueue.size());
                    decoder->addUopToSpeculativeCache(currentTrace, microop);
                }

                currentTrace.validPredSources = validPredSources;
                
                

                numMicroopsInTraceDist.sample(decoder->specCacheWriteQueue.size());
                DPRINTF(TraceGen, "Trace id %d added to spec cache with %d valid prediction sources!.\n", currentTrace.id, validPredSources );
                    
                // now that write queue is written to the spec cache, clear it
                // DON'T DELETE MICROOPS HERE! 
                decoder->specCacheWriteQueue.clear();
            
                DPRINTF(SuperOp, "Before optimization: \n");
                currentTrace.addr = currentTrace.head;
                dumpTrace(currentTrace);
        
                DPRINTF(SuperOp, "After optimization: \n");
                // this assertions should never fail!
                assert(currentTrace.getOptimizedHead().valid);
                int idx = currentTrace.getOptimizedHead().idx;
                int way = currentTrace.getOptimizedHead().way;

                assert(decoder->speculativeValidArray[idx][way]);
                assert(decoder->speculativeTraceIDArray[idx][way] == currentTrace.id);
                assert(decoder->speculativePrevWayArray[idx][way] == decoder->SPEC_CACHE_WAY_MAGIC_NUM);
            

                if (decoder->speculativeValidArray[idx][way] && decoder->speculativeTraceIDArray[idx][way] == currentTrace.id) 
                {
                    // mark end of trace and propagate live outs
                    DPRINTF(SuperOp, "End of Trace at %s!\n", currentTrace.prevNonEliminatedInst->getName());
                    
                    assert(currentTrace.prevNonEliminatedInst);
                    // here we mark 'prevNonEliminatedInst' as end of the trace because sometimes an eliminated instruction can be set as end of the trace
                    // Also, unset trace prediction source at the end of a trace as it doesn't matter what the prediction is since it will never be used.
                    currentTrace.prevNonEliminatedInst->setEndOfTrace();

                    dumpLiveOuts(currentTrace.prevNonEliminatedInst, true);

                    if (!currentTrace.prevNonEliminatedInst->isCarryingLivesOut() && !currentTrace.prevNonEliminatedInst->forwardedLiveValueExists) {
                        currentTrace.prevNonEliminatedInst->setTracePredictionSource(false);
                    }

                    assert(currentTrace.id);
                    assert(currentTrace.getOptimizedHead().valid);
                    assert(currentTrace.state == SpecTrace::OptimizationInProcess);

                    currentTrace.addr = currentTrace.getOptimizedHead();
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
                }

            }
            
        } // end of finalize old trace
        else if ((currentTrace.id != 0) && (currentTrace.state == SpecTrace::Evicted || currentTrace.state == SpecTrace::Invalid))
        {
            // how did we get here?!
            assert(0);
        }

        // Pop a new trace from the queue, start at top
        do {
            if (traceQueue.empty()) {
                DPRINTF(SuperOp, "Trace Queue is empty!\n");
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
                  decoder->uopAddrArray[idx][way][uop] == currentTrace.getTraceHeadAddr())
                ) 
            {
                DPRINTF(SuperOp, "Trace %i at (%i,%i,%i) evicted before we could process it.\n", currentTrace.id, currentTrace.addr.idx, currentTrace.addr.way, currentTrace.addr.uop);

                // A trace queued for first time optimization should never have a valid optimized head address 
                // because it never had the chance to get super optimized
                assert(!currentTrace.getOptimizedHead().valid);
                // should always be empty!
                assert(decoder->specCacheWriteQueue.empty());

                currentTrace.addr.valid = false;
                currentTrace.state = SpecTrace::Evicted;
                // remove it from traceMap
                DPRINTF(Decoder, "Removing trace id: %d  from Trace Map because of eviction!.\n", currentTrace.id );
                assert(traceMap.find(currentTrace.id) != traceMap.end());
                traceMap.erase(currentTrace.id);

                currentTrace.id = 0;
                
            }

            if (currentTrace.addr.valid)
            {
                DPRINTF(SuperOp, "Trace %d is selected for super optimization!\n", currentTrace.id);
            }

        } while (!currentTrace.addr.valid);

        DPRINTF(SuperOp, "Optimizing trace %i at (%i,%i,%i)\n", currentTrace.id, currentTrace.addr.idx, currentTrace.addr.way, currentTrace.addr.uop);
       

        if (currentTrace.state == SpecTrace::QueuedForFirstTimeOptimization) {
            currentTrace.state = SpecTrace::OptimizationInProcess;
        } 

        /* Clear Reg Context Block. */
        for (int i=0; i<38; i++) {
            regCtx[i].valid = regCtx[i].source = ccValid = false;
            PredccFlagBits = PredcfofBits = PreddfBit = PredecfBit = PredezfBit = 0;
        }
        
    } else { 
        assert(currentTrace.state == SpecTrace::OptimizationInProcess);
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
            
            // When a trace is being superoptimized it's written to the spec cahce write queue
            // Therefore, optimized head should never be valid before trace super optimization is complete and we decide that we eant to wite back the trace 
            // into the spec cache
            assert(!currentTrace.getOptimizedHead().valid);
            
            // clear spec cahce write queue
            for (auto &micro: decoder->specCacheWriteQueue)
            {
                StaticInstPtr macro = NULL;
                if (micro.inst->macroOp && micro.inst->macroOp->isMacroop())
                {
                    macro = micro.inst->macroOp;
                }
                if (macro)
                {
                    macro->deleteMicroOps();
                }
                macro = NULL;
            }            
            decoder->specCacheWriteQueue.clear();
            
            assert(currentTrace.id);
            // remove it from traceMap
            assert(traceMap.find(currentTrace.id) != traceMap.end());
            traceMap.erase(currentTrace.id);
            currentTrace.addr.valid = false;
            currentTrace.state = SpecTrace::Evicted;
            currentTrace.id = 0;
            
            tracesWithInvalidHead++;
            DPRINTF(SuperOp, "Trace was evicted out of the micro-op cache before we could optimize it\n");
            currentTrace.addr.valid = false;


            return false;
        }



        decodedMacroOp = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
        if (decodedMacroOp->getName() == "NOP" || decodedMacroOp->getName() == "fault" ||
            decodedMacroOp->getName() == "popcnt_Gv_Ev" || decodedMacroOp->getName() == "fdivr" ||
            decodedMacroOp->getName() == "xrstor" || decodedMacroOp->getName() == "prefetch_t0") {
            currentTrace.length++;
            if (currentTrace.prevNonEliminatedInst) {
                currentTrace.prevNonEliminatedInst->shrunkenLength++;
            }
            advanceTrace(currentTrace);
            return true;
        }
        if (decodedMacroOp->isMacroop()) {
            StaticInstPtr inst = decodedMacroOp->fetchMicroop(decoder->uopAddrArray[idx][way][uop].uopAddr);
            if (inst->getName() == "NOP" || inst->getName() == "fault" ||
                inst->getName() == "popcnt_Gv_Ev" || inst->getName() == "fdivr" ||
                inst->getName() == "xrstor" || inst->getName() == "prefetch_t0") {
                currentTrace.length++;
                if (currentTrace.prevNonEliminatedInst) {
                    currentTrace.prevNonEliminatedInst->shrunkenLength++;
                }
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
    
    string type = currentTrace.inst->getName();
    bool propagated = false;
    bool folded = false;
    bool unknownType = false;
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
    } else if (type == "st" || type == "stis" || type == "stfp" || type == "ld" || type == "ldis" || type == "ldst" || type == "halt" || type == "fault" || type == "call_far_Mp") {
            DPRINTF(ConstProp, "Type is ST, STIS, STFP, LD, LDIS, LDST, HALT, FAULT, or CALL_FAR_MP\n");
            // TODO: cannot remove
    } else {
            unknownType = true;
            //dumpLiveOuts(currentTrace.prevNonEliminatedInst, true);
            DPRINTF(ConstProp, "Inst type not covered: %s\n", type);
    }

    bool isDeadCode = false;
    if (!folded) {
        updateSuccessful = updateSpecTrace(currentTrace, isDeadCode, propagated);
        if (unknownType) {
            /*for (int i = 0; i < 16; i++) {
                regCtx[i].valid = regCtx[i].live = false;
            }
            ccValid = false;*/
        }
    }
    
    // now let's check to see if we can make a prediction
    if (!isDeadCode && !folded) {
        uint64_t value = 0;
        unsigned confidence = 0;
        unsigned latency;
        // first prob the predictor to see if we can make a prediction
        if (!isPredictionSource(currentTrace, currentTrace.instAddr, value, confidence, latency))
            probePredictorForMakingPrediction();

        string type = currentTrace.inst->getName();

        // if it's a prediction source, then do the same thin as before
        if (isPredictionSource(currentTrace, currentTrace.instAddr, value, confidence, latency)) 
        {
            // Mark prediction source as valid in register context block.    
            int numIntDestRegs = 0;
            for (int i = 0; i < currentTrace.inst->numDestRegs(); i++) {
                RegId destReg = currentTrace.inst->destRegIdx(i);
                if (destReg.classValue() == IntRegClass) {
                    X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)currentTrace.inst.get();
                    RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(destReg);
                    if (regCtx[dest_reg_idx].valid) {
                        currentTrace.inst->predSourceRegIdx = i;
                        currentTrace.inst->forwardedLiveValue = regCtx[dest_reg_idx].value;
                        currentTrace.inst->forwardedLiveValueExists = true;
                    }
                    numIntDestRegs++;
                    DPRINTF(SuperOp, "Setting regCtx[%i] to %x from %s inst\n", dest_reg_idx, value, type);
                    assert(dest_reg_idx < 38);
                    regCtx[dest_reg_idx].value = value;
                    currentTrace.inst->predictedValue = value;
                    regCtx[dest_reg_idx].valid = true;
                    regCtx[dest_reg_idx].source = true;
                    // currentTrace.inst->predictedLoad = true; // this should never set for trace prediction sources!
                    currentTrace.inst->confidence = confidence;
                }
            }
            assert(numIntDestRegs == 1);

            dumpLiveOuts(currentTrace.inst, currentTrace.inst->isFirstMicroop());

            // We can predict some of the Condition Code instructions based on whether they need source registers to 
            // generate the flags or not
            // Update the flags for these instructions
            

            // set this as a source of prediction so we can verify it later
            currentTrace.inst->setTracePredictionSource(true);

            //now if the instruction is CC we need to update the CC flags
            // if (currentTrace.inst->isCC() && !isPredictiableCC(currentTrace.inst))
                // {
            //     // Because we can't propagate AF, PF, and CF flags, from now on they are not valid
            //     // valid_CF_AF_OF_flags will be set by any instruction that can generate all the flags 
            //     // if we hit wrip and wripi microops and valid_CF_AF_OF_flags is not valid, then we will decide to 
            //     // whether fold or not fold it 
            //     valid_CF_AF_OF_flags = false;

            //     assert(usingCCTracking);
            //     DPRINTF(ConstProp, "From now on CF, AF, and OF flags are invalid dude to a condition code prediction source.\n");
            //     updateCCFlagsForPredictedSource(currentTrace.inst);

            // }
            // else if (currentTrace.inst->isCC() && isPredictiableCC(currentTrace.inst))
            // {
            //     // based on my observation all of the CC codes that we can predict, they set AF, PF, and CF flgas
            //     // https://www.gem5.org/documentation/general_docs/architecture_support/x86_microop_isa/
            //     // therefore, if we are here it means that my assumption is wrong and we need to propagate these flags
            //     assert(0);
            // }
        } else {
            for (int i=0; i<currentTrace.inst->numDestRegs(); i++) {
                RegId destReg = currentTrace.inst->destRegIdx(i);
                if (destReg.classValue() == IntRegClass) {
                    // ah, bh, ch, dh regs use different index values. For example, ah is 64, and etc.
                    // we need to fold these regs before use them 
                    X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)currentTrace.inst.get();
                    RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(destReg);
                    assert(dest_reg_idx < 38);
                    regCtx[dest_reg_idx].valid = false;
                }
            }
        }
        currentTrace.inst->shrunkenLength = currentTrace.interveningDeadInsts;
        currentTrace.interveningDeadInsts = 0;
        currentTrace.prevNonEliminatedInst = currentTrace.inst;
    } else {
        currentTrace.prevEliminatedInst = currentTrace.inst;
        currentTrace.interveningDeadInsts++;
    }
    // Simulate a stall if update to speculative cache wasn't successful
    if (updateSuccessful) {
        currentTrace.end.pcAddr = currentTrace.instAddr.pcAddr;
        currentTrace.end.uopAddr = currentTrace.instAddr.uopAddr;
        currentTrace.end.uopAddr++;
        if (currentTrace.inst->macroOp) {
            if (currentTrace.end.uopAddr == currentTrace.inst->macroOp->getNumMicroops()) {
                currentTrace.end.pcAddr = currentTrace.instAddr.pcAddr + currentTrace.inst->macroOp->getMacroopSize();
                currentTrace.end.uopAddr = 0;
            }
        } else if (type == "syscall") {
            currentTrace.end.pcAddr = currentTrace.instAddr.pcAddr + 2;
            currentTrace.end.uopAddr = 0;
        } 
        else if (type == "CPUID") {
            panic("unsupported instruction without macro-op: %s", type);
        }
        else 
        {
            panic("unsupported instruction without macro-op: %s", type);
        }
        
        DPRINTF(TraceGen, "Setting end of trace PC to: %#x:%#x\n",
                            currentTrace.end.pcAddr, currentTrace.end.uopAddr);
       
        Addr target = advanceTrace(currentTrace);
        if (currentTrace.inst->isControl()) {
            if (target) {
                currentTrace.inst->predictedTarget._pc = target;
                currentTrace.inst->predictedTarget._npc = target + 1;
                currentTrace.inst->predictedTarget._upc = 0;
                currentTrace.inst->predictedTarget._upc = 1;
                currentTrace.inst->predictedTaken = true;
                DPRINTF(TraceGen, "Predicted taken to: %s\n", currentTrace.inst->predictedTarget);
            } else {
                currentTrace.inst->predictedTarget._pc = currentTrace.end.pcAddr;
                currentTrace.inst->predictedTarget._npc = currentTrace.end.pcAddr + 1;
                currentTrace.inst->predictedTarget._upc = currentTrace.end.uopAddr;
                currentTrace.inst->predictedTarget._upc = currentTrace.end.uopAddr + 1;
                currentTrace.inst->predictedTaken = false;
                DPRINTF(TraceGen, "Predicted not taken to: %s\n", currentTrace.inst->predictedTarget);
            }
        }
    }


    // Propagate live outs at the end of each control instruction
    // updateSuccessful is not just for spec cache also for folded instructions
    if ((currentTrace.inst->isControl() || type == "syscall") && 
        updateSuccessful) {
        dumpLiveOuts(currentTrace.inst, true);
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
    //HeapProfilerStop();
}

bool TraceBasedGraph::probePredictorForMakingPrediction()
{
    size_t NumOfValidPredictionSources;
    for (NumOfValidPredictionSources = 0; NumOfValidPredictionSources < 4; NumOfValidPredictionSources++)
    {
        if (!currentTrace.source[NumOfValidPredictionSources].valid) 
            break;
    }

    // do a prediction for the instruction before trying to propagte it
    // For Condition Codes based on the flags we will decide that we need source operand or not
    // If we only need the dest reg value to generate the flags like EZFBit, ZFBit, and etc. then we make a prediction here and in the next step genrate those 
    // flags   
    if (NumOfValidPredictionSources < 4 /*&& isPredictiableCC(currentTrace.inst)*/)
    {
        LVPredUnit::lvpReturnValues ret;
        if (loadPred->makePredictionForTraceGenStage(currentTrace.instAddr.pcAddr, currentTrace.instAddr.uopAddr, 0 , ret))
        {
            if ( ret.confidence >= predictionConfidenceThreshold)
            {
                DPRINTF(TraceGen, "Found a high confidence prediction for address %#x:%d in the predictor! Confidence is %d! Predicted Value = %#x. This prediction source is a CC code? %d!\n", 
                                currentTrace.instAddr.pcAddr, currentTrace.instAddr.uopAddr, ret.confidence, ret.predictedValue, currentTrace.inst->isCC() );

                assert(!currentTrace.source[NumOfValidPredictionSources].valid);

                currentTrace.source[NumOfValidPredictionSources].addr = FullUopAddr(currentTrace.instAddr.pcAddr, currentTrace.instAddr.uopAddr);
                currentTrace.source[NumOfValidPredictionSources].confidence = ret.confidence;
                currentTrace.source[NumOfValidPredictionSources].value = ret.predictedValue;
                currentTrace.source[NumOfValidPredictionSources].latency = ret.latency;
                currentTrace.source[NumOfValidPredictionSources].valid = true;
                return true;

            }
            else 
            {
                DPRINTF(TraceGen, "Found a low confidence prediction for address %#x:%d in the predictor! Confidence is %d! Predicted Value = %#x\n", 
                                currentTrace.instAddr.pcAddr, currentTrace.instAddr.uopAddr, ret.confidence, ret.predictedValue );
                return false;
            }
        }
        else 
        {
            DPRINTF(TraceGen, "Can't find a prediction for address %#x:%d in the predictor!\n", currentTrace.instAddr.pcAddr, currentTrace.instAddr.uopAddr);
            return false;
        }
    }
    else if (NumOfValidPredictionSources == 4)
    {
        DPRINTF(TraceGen, "Not enough space for adding a new prediction source!\n");
        return false;
    }
    else 
    {
        panic_if(NumOfValidPredictionSources > 4, "morethan 4 valid prediction sources!\n");
        return false;
    }
}

void TraceBasedGraph::updateCCFlagsForPredictedSource(StaticInstPtr inst)
{
    uint16_t ext = inst->getExt();
    if (inst->getName() == "and" )
    {

        if (((ext & ccFlagMask) == ccFlagMask) || ((ext & ccFlagMask) == 0)) {
            PredccFlagBits = 0; 
        }
        if ((((ext & CFBit) != 0 && (ext & OFBit) != 0) || ((ext & (CFBit | OFBit)) == 0))) {
            PredcfofBits = 0;
        }
        PreddfBit = 0;
        PredecfBit = 0;
        PredezfBit = 0;

        // from generated exec code
        uint64_t mask = CFBit | ECFBit | OFBit;
        uint64_t newFlags = inst->genFlagsForSuperOptimizer(PredccFlagBits | PreddfBit | PredezfBit , ext & ~mask, inst->predictedValue);

        PredezfBit = newFlags & EZFBit;
        PreddfBit = newFlags & DFBit;
        PredccFlagBits = newFlags & ccFlagMask;
        //If a logic microop wants to set these, it wants to set them to 0.
        PredcfofBits = PredcfofBits & ~((CFBit | OFBit) & ext);   // we can't update this flag with just des reg! we need sources
        PredecfBit = PredecfBit & ~(ECFBit & ext);                // we can't update this flag with just des reg! we need sources

        assert(PredcfofBits == 0 && PredecfBit == 0);

        ccValid = true;

    }
    else if (inst->getName() == "xor")
    {

    }
    else if (inst->getName() == "sub")
    {

    }
    else if ("srli")
    {
        panic_if(true, "unimplemented flag generation code for %s microop! Implement it\n", inst->getName());
    }

}

bool TraceBasedGraph::isPredictiableCC(StaticInstPtr inst)
{
    if (!inst->isCC()) return true;

    if (!usingCCTracking) return false;

    uint64_t flagMask = inst->getExt();

    if (flagMask & (ECFBit | CFBit)) 
        return false;
    else if (flagMask & AFBit)
        return false;
    else if (flagMask & OFBit)
        return false;

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
    isDeadCode = (type == "rdip") || (allSrcsReady && (type == "mov" || type == "movi" || type == "limm" || type == "add" || type == "addi" || type == "sub" || type == "subi" || type == "and" || type == "andi" || type == "or" || type == "ori" || type == "xor" || type == "xori" || type == "srli" || type == "slli" || type == "sexti" || type == "zexti"));

    // Prevent an inst registering as dead if it is a prediction source or if it is a return or it modifies CC
    uint64_t value;
    unsigned confidence;
    unsigned latency;
    bool isPredSource = isPredictionSource(trace, trace.instAddr, value, confidence, latency) ;
    //assert(!isPredSource);
    
    isDeadCode &= (propagated && !isPredSource && !((!usingCCTracking && trace.inst->isCC()) || trace.inst->isReturn()));

    DPRINTF(ConstProp, "isDeadCode:%d propagated:%d isPredSource:%d CC:%d Return:%d\n", isDeadCode, propagated, isPredSource, (/*!usingCCTracking &&*/ trace.inst->isCC()), trace.inst->isReturn());
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

    //bool updateSuccessful = decoder->addUopToSpeculativeCache( trace, isPredSource);
    bool updateSuccessful  = true;
    // instead of adding the trace directly to the spec cache, first wirte it to the spec cache write queue
    decoder->specCacheWriteQueue.push_back(SuperOptimizedMicroop(trace.inst, trace.instAddr));
    /// This is normal to happen but the problem is that we don't have the necessary logic to cause a stall
    panic_if(!updateSuccessful, "Failed to update the spec cache!\n");
    trace.shrunkLength++;

    // Step 3b: Mark all predicted values on the StaticInst
    for (int i=0; i<trace.inst->numSrcRegs(); i++) 
    {
        uint16_t srcIdx = trace.inst->srcRegIdx(i).flatIndex();

        DPRINTF(ConstProp, "ConstProp: Examining register %i\n", srcIdx);
        if (trace.inst->srcRegIdx(i).classValue() == IntRegClass) 
        {
                // ah, bh, ch, dh regs use different index values. For example, ah is 64, and etc.
                // we need to fold these regs before use them 
                X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)trace.inst.get();
                RegIndex src_reg_idx = x86_inst->getUnflattenRegIndex(trace.inst->srcRegIdx(i));
                assert(src_reg_idx < 38);
                if (regCtx[src_reg_idx].valid)
                {
                    //if (fold)  DPRINTF(ConstProp, "ConstProp: Found a folded register! Reg: %d \n", src_reg_idx);
                    DPRINTF(ConstProp, "ConstProp: Propagated constant %#x in reg %i (arch: %d) at %#x:%d\n", regCtx[src_reg_idx].value, srcIdx, src_reg_idx, trace.instAddr.pcAddr, trace.instAddr.uopAddr);
                    DPRINTF(ConstProp, "ConstProp: Setting trace.inst sourcePrediction to %#x\n", regCtx[src_reg_idx].value);
                    trace.inst->sourcePredictions[i] = regCtx[src_reg_idx].value;
                    trace.inst->sourcesPredicted[i] = true;
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
    X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
    const uint8_t dataSize = inst_regop->dataSize;
    assert(dataSize == 8 || dataSize == 4 || dataSize == 2 || dataSize == 1);

    //uint16_t src1 = inst->srcRegIdx(0).flatIndex(); src1 = src1;
    //uint16_t src2 = inst->srcRegIdx(1).flatIndex(); // this is the soruce reg in 4 or 8 byte moves
    uint16_t src1 = x86_inst->getUnflattenRegIndex(inst->srcRegIdx(0)); src1 = src1;
    uint16_t src2 = x86_inst->getUnflattenRegIndex(inst->srcRegIdx(1));
    //assert(src1 < 38); 
    assert(src2 < 38);

    

    // RegIndex src_1_check = x86_inst->getUnflattenRegIndex(inst->srcRegIdx(0));
    // RegIndex src_2_check = x86_inst->getUnflattenRegIndex(inst->srcRegIdx(1));


    if (/*(!regCtx[src1].valid) ||*/ (!regCtx[src2].valid)) {
        return false;
    }

    assert(src2 < 38);
    //uint64_t SrcReg1 = regCtx[src1].value;
    uint64_t SrcReg2 = regCtx[src2].value;

    uint64_t forwardVal = 0;
    
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
    RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, dest_reg_idx);

    // checks to make sure there is no overflow
    assert(dest_reg_idx < 38);
    regCtx[dest_reg_idx].value = forwardVal;
    regCtx[dest_reg_idx].valid = true;
    regCtx[dest_reg_idx].source = false;

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
    RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));

    assert(dest_reg_idx < 38);
	if (dataSize < 4 && regCtx[dest_reg_idx].valid) {
		forwardVal = x86_inst->merge(regCtx[dest_reg_idx].value, imm, dataSize);
	} else if (dataSize < 4) { 
        return false; 
    }

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, dest_reg_idx);
    assert(dest_reg_idx < 38);
    regCtx[dest_reg_idx].value = forwardVal;
    regCtx[dest_reg_idx].valid = true;
    regCtx[dest_reg_idx].source = false;
        
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
    X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
    const uint8_t dataSize = inst_regop->dataSize;
    assert(dataSize == 8 || dataSize == 4 || dataSize == 2 || dataSize == 1);

    // unsigned src1 = inst->srcRegIdx(0).flatIndex(); 
    // unsigned src2 = inst->srcRegIdx(1).flatIndex(); 
    unsigned src1 = x86_inst->getUnflattenRegIndex(inst->srcRegIdx(0));
    unsigned src2 = x86_inst->getUnflattenRegIndex(inst->srcRegIdx(1));
    assert(src1 < 38);
    assert(src2 < 38);
    if ((!regCtx[src1].valid) || (!regCtx[src2].valid)) {
        if (usingCCTracking && inst->isCC()) {
            ccValid = false;
        }
        return false;
    }

    uint64_t SrcReg1 = regCtx[src1].value;
    uint64_t SrcReg2 = regCtx[src2].value;
    uint64_t forwardVal = 0;
	

    uint64_t psrc1, psrc2;
    if (dataSize >= 4)
    {
        X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
        psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
        psrc2 = x86_inst->pick(SrcReg2, 1, dataSize);
        forwardVal = (psrc1 + psrc2) & mask(dataSize * 8);;
        
    } else {
		RegId destReg = inst->destRegIdx(0);
        RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
        assert(destReg.isIntReg());

		if (!regCtx[dest_reg_idx].valid) { 
            if (usingCCTracking && inst->isCC()) {
                ccValid = false;
            }
            return false;
        }

        assert(dest_reg_idx < 38);
		uint64_t DestReg = regCtx[dest_reg_idx].value;

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
    RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, dest_reg_idx);
    assert(dest_reg_idx < 38);
    regCtx[dest_reg_idx].value = forwardVal;
    regCtx[dest_reg_idx].valid = true;
    regCtx[dest_reg_idx].source = false;

    
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

    // unsigned src1 = inst->srcRegIdx(0).flatIndex();
    // unsigned src2 = inst->srcRegIdx(1).flatIndex();
    X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
    unsigned src1 = x86_inst->getUnflattenRegIndex(inst->srcRegIdx(0));
    unsigned src2 = x86_inst->getUnflattenRegIndex(inst->srcRegIdx(1));
    assert(src1 < 38);
    assert(src2 < 38);
    if ((!regCtx[src1].valid) || (!regCtx[src2].valid)) {
        if (usingCCTracking && inst->isCC()) {
            ccValid = false;
        }
        return false;
    }

    uint64_t SrcReg1 = regCtx[src1].value;
    uint64_t SrcReg2 = regCtx[src2].value;
    uint64_t forwardVal = 0;

    uint64_t psrc1, psrc2;
    if (dataSize >= 4)
    {
        X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
        psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
        psrc2 = x86_inst->pick(SrcReg2, 1, dataSize);
        forwardVal = (psrc1 - psrc2) & mask(dataSize * 8);;
    } else {
		RegId destReg = inst->destRegIdx(0);
        RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
        assert(destReg.isIntReg());
        assert(dest_reg_idx < 38);
		if (!regCtx[dest_reg_idx].valid) { 
            if (usingCCTracking && inst->isCC()) {
                ccValid = false;
            }
            return false; 
        }
        assert(dest_reg_idx < 38);
		uint64_t DestReg = regCtx[dest_reg_idx].value;
		
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
    RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
    assert(destReg.isIntReg());
    
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, dest_reg_idx);
    assert(dest_reg_idx < 38);
    regCtx[dest_reg_idx].value = forwardVal;
    regCtx[dest_reg_idx].valid = true;
    regCtx[dest_reg_idx].source = false;
    
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
    X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
    const uint8_t dataSize = inst_regop->dataSize;
    assert(dataSize == 8 || dataSize == 4 || dataSize == 2 || dataSize == 1);

    //unsigned src1 = inst->srcRegIdx(0).flatIndex();
    //unsigned src2 = inst->srcRegIdx(1).flatIndex();
    unsigned src1 = x86_inst->getUnflattenRegIndex(inst->srcRegIdx(0));
    unsigned src2 = x86_inst->getUnflattenRegIndex(inst->srcRegIdx(1));
    assert(src1 < 38);
    assert(src2 < 38);
    if ((!regCtx[src1].valid) || (!regCtx[src2].valid)) {
        if (usingCCTracking && inst->isCC()) {
            ccValid = false;
        }
        return false;
    }

    uint64_t SrcReg1 = regCtx[src1].value;
    uint64_t SrcReg2 = regCtx[src2].value;
    uint64_t forwardVal = 0;
	

    uint64_t psrc1, psrc2;
    if (dataSize >= 4)
    { 
        psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
        psrc2 = x86_inst->pick(SrcReg2, 1, dataSize);
        forwardVal = (psrc1 & psrc2) & mask(dataSize * 8);
    } else {
        RegId destReg = inst->destRegIdx(0);
        RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
        assert(destReg.isIntReg());
        assert(dest_reg_idx < 38);
		if (!regCtx[dest_reg_idx].valid) { 
            if (usingCCTracking && inst->isCC()) {
                ccValid = false;
            }
            return false;
        }
        
		uint64_t DestReg = regCtx[dest_reg_idx].value;

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
    RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, dest_reg_idx);
    assert(dest_reg_idx < 38);
    regCtx[dest_reg_idx].value = forwardVal;
    regCtx[dest_reg_idx].valid = true;
    regCtx[dest_reg_idx].source = false;

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

	X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
    // unsigned src1 = inst->srcRegIdx(0).flatIndex();
    // unsigned src2 = inst->srcRegIdx(1).flatIndex();
    unsigned src1 = x86_inst->getUnflattenRegIndex(inst->srcRegIdx(0));
    unsigned src2 = x86_inst->getUnflattenRegIndex(inst->srcRegIdx(1));
    assert(src1 < 38);
    assert(src2 < 38);
    if ((!regCtx[src1].valid) || (!regCtx[src2].valid)) {
        if (usingCCTracking && inst->isCC()) {
            ccValid = false;
        }
        return false;
    }

    uint64_t SrcReg1 = regCtx[src1].value;
    uint64_t SrcReg2 = regCtx[src2].value;
    uint64_t forwardVal = 0;

    uint64_t psrc1, psrc2;
    if (dataSize >= 4)
    {
        psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
        psrc2 = x86_inst->pick(SrcReg2, 1, dataSize);
        forwardVal = (psrc1 | psrc2) & mask(dataSize * 8);
    } else {
		RegId destReg = inst->destRegIdx(0);
        RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
        assert(destReg.isIntReg());
        assert(dest_reg_idx < 38);
		if (!regCtx[dest_reg_idx].valid) { 
            if (usingCCTracking && inst->isCC()) {
                ccValid = false;
            }
            return false;
        }

		uint64_t DestReg = regCtx[dest_reg_idx].value;

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
    RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, dest_reg_idx);
    assert(dest_reg_idx < 38);
    regCtx[dest_reg_idx].value = forwardVal;
    regCtx[dest_reg_idx].valid = true;
    regCtx[dest_reg_idx].source = false;
    
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

    X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
    //unsigned src1 = inst->srcRegIdx(0).flatIndex();
    //unsigned src2 = inst->srcRegIdx(1).flatIndex();
    unsigned src1 = x86_inst->getUnflattenRegIndex(inst->srcRegIdx(0));
    unsigned src2 = x86_inst->getUnflattenRegIndex(inst->srcRegIdx(1));
    assert(src1 < 38);
    assert(src2 < 38);
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
    
 
    uint64_t psrc1, psrc2;
    if (dataSize >= 4)
    {
        psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
        psrc2 = x86_inst->pick(SrcReg2, 1, dataSize);
        forwardVal = (psrc1 ^ psrc2) & mask(dataSize * 8);
    }
    else {
        RegId destReg = inst->destRegIdx(0);
        RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(destReg);
        assert(destReg.isIntReg());
        assert(dest_reg_idx < 38);
		if (!regCtx[dest_reg_idx].valid) { 
            if (usingCCTracking && inst->isCC()) {
                ccValid = false;
            }
            return false;
        }

		uint64_t DestReg = regCtx[dest_reg_idx].value;

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
    RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(destReg);

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, dest_reg_idx);
    assert(dest_reg_idx < 38);
    regCtx[dest_reg_idx].value = forwardVal;
    regCtx[dest_reg_idx].valid = true;
    regCtx[dest_reg_idx].source = false;
    
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
    RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
    assert(destReg.isIntReg());
    assert(dest_reg_idx < 38);
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, dest_reg_idx);
    regCtx[dest_reg_idx].value = forwardVal;
    regCtx[dest_reg_idx].valid = true;
    regCtx[dest_reg_idx].source = false;

    
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
    assert(src1 < 38);
    //assert(src2 < 38);
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
        RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
        assert(destReg.isIntReg());
        assert(dest_reg_idx < 38);
		if (!regCtx[dest_reg_idx].valid) { 
            if (usingCCTracking && inst->isCC()) {
                ccValid = false;
            }
            return false;
        }

		uint64_t DestReg = regCtx[dest_reg_idx].value;

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
    RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, dest_reg_idx);
    assert(dest_reg_idx < 38);
    regCtx[dest_reg_idx].value = forwardVal;
    regCtx[dest_reg_idx].valid = true;
    regCtx[dest_reg_idx].source = false;

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
    assert(src1 < 38);
    //assert(src2 < 38);
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
        RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
        assert(destReg.isIntReg());
        assert(dest_reg_idx < 38);
		if (!regCtx[dest_reg_idx].valid) { 
            if (usingCCTracking && inst->isCC()) {
                ccValid = false;
            }
            return false;
        }

		uint64_t DestReg = regCtx[dest_reg_idx].value;

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
    RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, dest_reg_idx);
    assert(dest_reg_idx < 38);
    regCtx[dest_reg_idx].value = forwardVal;
    regCtx[dest_reg_idx].valid = true;
    regCtx[dest_reg_idx].source = false;

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
    assert(src1 < 38);
    //assert(src2 < 38);
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
        RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
        assert(destReg.isIntReg());
        assert(dest_reg_idx < 38);
		if (!regCtx[dest_reg_idx].valid) { 
            if (usingCCTracking && inst->isCC()) {
                ccValid = false;
            }
            return false;
        }

		uint64_t DestReg = regCtx[dest_reg_idx].value;

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
    RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, dest_reg_idx);
    assert(dest_reg_idx < 38);
    regCtx[dest_reg_idx].value = forwardVal;
    regCtx[dest_reg_idx].valid = true;
    regCtx[dest_reg_idx].source = false;

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
    assert(src1 < 38);
    //assert(src2 < 38);
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
        RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
        assert(dest_reg_idx < 38);
		if (!regCtx[dest_reg_idx].valid) { 
            if (usingCCTracking && inst->isCC()) {
                ccValid = false;
            }
            return false;
        }

		uint64_t DestReg = regCtx[dest_reg_idx].value;

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
    RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, dest_reg_idx);
    assert(dest_reg_idx < 38);
    regCtx[dest_reg_idx].value = forwardVal;
    regCtx[dest_reg_idx].valid = true;
    regCtx[dest_reg_idx].source = false;

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
    assert(src1 < 38);
    //assert(src2 < 38);
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
        RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
        assert(destReg.isIntReg());
        assert(dest_reg_idx < 38);
		if (!regCtx[dest_reg_idx].valid) { 
            if (usingCCTracking && inst->isCC()) {
                ccValid = false;
            }
            return false;
        }

		uint64_t DestReg = regCtx[dest_reg_idx].value;

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
    RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, dest_reg_idx);
    assert(dest_reg_idx < 38);
    regCtx[dest_reg_idx].value = forwardVal;
    regCtx[dest_reg_idx].valid = true;
    regCtx[dest_reg_idx].source = false;
    
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

	X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
    //unsigned src1 = inst->srcRegIdx(0).flatIndex();
    unsigned src1 = x86_inst->getUnflattenRegIndex(inst->srcRegIdx(0));
    //unsigned src2 = inst->srcRegIdx(1).flatIndex();
    assert(src1 < 38);
    //assert(src2 < 38);
    if ((!regCtx[src1].valid) /*|| (!regCtx[src2].valid)*/) {
        if (usingCCTracking && inst->isCC()) {
            ccValid = false;
        }
        return false;
    }

    uint64_t SrcReg1 = regCtx[src1].value;
    //uint64_t SrcReg2 = regCtx[src2].value;
    uint64_t forwardVal = 0;
    
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
        RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
        assert(destReg.isIntReg());
        assert(dest_reg_idx < 38);
		if (!regCtx[dest_reg_idx].valid) { 
            if (usingCCTracking && inst->isCC()) {
                ccValid = false;
            }
            return false;
        }
		uint64_t DestReg = regCtx[dest_reg_idx].value;

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
    RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, dest_reg_idx);
    assert(dest_reg_idx < 38);
    regCtx[dest_reg_idx].value = forwardVal;
    regCtx[dest_reg_idx].valid = true;
    regCtx[dest_reg_idx].source = false;

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
    
    X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
    //unsigned src1 = inst->srcRegIdx(0).flatIndex();
    unsigned src1 = x86_inst->getUnflattenRegIndex(inst->srcRegIdx(0));
    //unsigned src2 = inst->srcRegIdx(1).flatIndex();
    assert(src1 < 38);
    //assert(src2 < 38);
    if ((!regCtx[src1].valid) /*|| (!regCtx[src2].valid)*/) {
        if (usingCCTracking && inst->isCC()) {
            ccValid = false;
        }
        return false;
    }

    uint64_t SrcReg1 = regCtx[src1].value;
    //uint64_t SrcReg2 = regCtx[src2].value;
    uint64_t forwardVal = 0;

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
        RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
        assert(destReg.isIntReg());
        assert(dest_reg_idx < 38);
		if (!regCtx[dest_reg_idx].valid) {
            if (usingCCTracking && inst->isCC()) {
                ccValid = false;
            }
            return false; 
        }

		uint64_t DestReg = regCtx[dest_reg_idx].value;

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
    RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, dest_reg_idx);
    assert(dest_reg_idx < 38);
    regCtx[dest_reg_idx].value = forwardVal;
    regCtx[dest_reg_idx].valid = true;
    regCtx[dest_reg_idx].source = false;
    
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

    X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
    //unsigned src1 = inst->srcRegIdx(0).flatIndex();
    unsigned src1 = x86_inst->getUnflattenRegIndex(inst->srcRegIdx(0));
    //unsigned src2 = inst->srcRegIdx(1).flatIndex();
    assert(src1 < 38);
    //assert(src2 < 38);
    if ((!regCtx[src1].valid) /*|| (!regCtx[src2].valid)*/) {
        if (usingCCTracking && inst->isCC()) {
            ccValid = false;
        }
        return false;
    }

    uint64_t SrcReg1 = regCtx[src1].value;
    //uint64_t SrcReg2 = regCtx[src2].value;

    uint64_t forwardVal = 0;

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
        RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
        assert(destReg.isIntReg());
        assert(dest_reg_idx < 38);
		if (!regCtx[dest_reg_idx].valid) { 
            if (usingCCTracking && inst->isCC()) {
                ccValid = false;
            }
            return false;
        }
        
		uint64_t DestReg = regCtx[dest_reg_idx].value;
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
    RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, dest_reg_idx);
    assert(dest_reg_idx < 38);
    regCtx[dest_reg_idx].value = forwardVal;
    regCtx[dest_reg_idx].valid = true;
    regCtx[dest_reg_idx].source = false;
    
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

    X86ISA::X86StaticInst * x86_inst = (X86ISA::X86StaticInst *)inst.get();
    //unsigned src1 = inst->srcRegIdx(0).flatIndex();
    unsigned src1 = x86_inst->getUnflattenRegIndex(inst->srcRegIdx(0));
    //unsigned src2 = inst->srcRegIdx(1).flatIndex();
    assert(src1 < 38);
    //assert(src2 < 38);
    if ((!regCtx[src1].valid) /*|| (!regCtx[src2].valid)*/) {
        return false;
    }

    uint64_t SrcReg1 = regCtx[src1].value;
    //uint64_t SrcReg2 = regCtx[src2].value;

    uint64_t forwardVal = 0;


    uint8_t imm8 = inst_regop->imm8;

    uint64_t psrc1;
    if (dataSize >= 4)
    {
        psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);
        forwardVal = bits(psrc1, imm8, 0) & mask(dataSize * 8);;
    }
    else {
        RegId destReg = inst->destRegIdx(0);
        RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
        assert(destReg.isIntReg());
        assert(dest_reg_idx < 38);
		if (!regCtx[dest_reg_idx].valid) { 
            return false; 
        }

		uint64_t DestReg = regCtx[dest_reg_idx].value;

		psrc1 = x86_inst->pick(SrcReg1, 0, dataSize);	

		forwardVal = x86_inst->merge(DestReg, bits(psrc1, imm8, 0), dataSize);
    }

    RegId destReg = inst->destRegIdx(0);
    RegIndex dest_reg_idx = x86_inst->getUnflattenRegIndex(inst->destRegIdx(0));
    assert(destReg.isIntReg());

    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, dest_reg_idx);
    assert(dest_reg_idx < 38);
    regCtx[dest_reg_idx].value = forwardVal;
    regCtx[dest_reg_idx].valid = true;
    regCtx[dest_reg_idx].source = false;
    
    return true;
}

bool TraceBasedGraph::propagateWrip(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "wrip");

    
    if (inst->isCC() && (!usingCCTracking || !ccValid))
    {
        DPRINTF(ConstProp, "CC WRIP Inst! We can't propagate CC insts!\n");
        assert((usingCCTracking && !ccValid) || (!usingCCTracking && !ccValid));
        return false;
    }

    X86ISA::RegOpImm * inst_regop = (X86ISA::RegOpImm * )inst.get(); 
    const uint8_t dataSize = inst_regop->dataSize;
    assert(dataSize == 8 || dataSize == 4 || dataSize == 2 || dataSize == 1);

    if (dataSize < 4) return false;

    unsigned src1 = inst->srcRegIdx(0).flatIndex();
    unsigned src2 = inst->srcRegIdx(1).flatIndex();
    assert(src1 < 38);
    assert(src2 < 38);
    if ((!regCtx[src1].valid) || (!regCtx[src2].valid)) {
        DPRINTF(ConstProp, "sources (%d, %d) invalid\n", regCtx[src1].valid, regCtx[src2].valid); 
        // if the wripi is not folded and cc flags are valid, then propagte them with microop
        // in some corner cases, the cc flags for a wripi are valid but the microop itself cannot be folded because of inssuficint space 
        if (ccValid) {
            DPRINTF(ConstProp, "Propagating CC flags with wripi microop although the wripi microop itself is not propgated!\n");
            inst->propgatedCCFlags[0] = PredccFlagBits;
            inst->isCCFlagPropagated[0] = true;
            inst->propgatedCCFlags[1] = PredcfofBits;
            inst->isCCFlagPropagated[1] = true;
            inst->propgatedCCFlags[2] = PreddfBit;
            inst->isCCFlagPropagated[2] = true;
            inst->propgatedCCFlags[3] = PredecfBit;
            inst->isCCFlagPropagated[3] = true;
            inst->propgatedCCFlags[4] = PredezfBit;
            inst->isCCFlagPropagated[4] = true;
        }
        else 
        {
            DPRINTF(ConstProp, "CC flags are not valid! We can't propagate them!\n");
        }
        return false;
    }

    if (currentTrace.branchesFolded >= 2)
    {
        DPRINTF(ConstProp, "More than 2 branches are folded in this trace! We can't fold this wrip instruction although src 1 and src2 are valid!\n");
        // if the wrip is not folded and cc flags are valid, then propagte them with microop
        // in some corner cases, the cc flags for a wripi are valid but the microop itself cannot be folded because of inssuficint space 
        if (ccValid) {
            DPRINTF(ConstProp, "Propagating CC flags with wripi microop although the wrip microop itself is not propgated!\n");
            inst->propgatedCCFlags[0] = PredccFlagBits;
            inst->isCCFlagPropagated[0] = true;
            inst->propgatedCCFlags[1] = PredcfofBits;
            inst->isCCFlagPropagated[1] = true;
            inst->propgatedCCFlags[2] = PreddfBit;
            inst->isCCFlagPropagated[2] = true;
            inst->propgatedCCFlags[3] = PredecfBit;
            inst->isCCFlagPropagated[3] = true;
            inst->propgatedCCFlags[4] = PredezfBit;
            inst->isCCFlagPropagated[4] = true;
        }
        else 
        {
            DPRINTF(ConstProp, "CC flags are not valid! We can't propagate them!\n");
        }
        return false;
    }
        

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

    // if the wrip is not folded and cc flags are valid, then propagte them with microop
    if (ccValid) {
        DPRINTF(ConstProp, "Propagating CC flags with wrip microop!\n");
        inst->propgatedCCFlags[0] = PredccFlagBits;
        inst->isCCFlagPropagated[0] = true;
        inst->propgatedCCFlags[1] = PredcfofBits;
        inst->isCCFlagPropagated[1] = true;
        inst->propgatedCCFlags[2] = PreddfBit;
        inst->isCCFlagPropagated[2] = true;
        inst->propgatedCCFlags[3] = PredecfBit;
        inst->isCCFlagPropagated[3] = true;
        inst->propgatedCCFlags[4] = PredezfBit;
        inst->isCCFlagPropagated[4] = true;
    }
    else 
    {
        DPRINTF(ConstProp, "CC flags are not valid! We can't propagate them!\n");
    }

    return false;
}

bool TraceBasedGraph::propagateWripI(StaticInstPtr inst) {
    string type = inst->getName();
    assert(type == "wripi");

    
    if (inst->isCC() && (!usingCCTracking || !ccValid))
    {
        assert((usingCCTracking && !ccValid) || (!usingCCTracking && !ccValid));
        DPRINTF(ConstProp, "CC WRIP Inst! We can't propagate CC insts!\n");
        return false;
    }

    X86ISA::RegOpImm * inst_regop = (X86ISA::RegOpImm * )inst.get(); 
    const uint8_t dataSize = inst_regop->dataSize;
    assert(dataSize == 8 || dataSize == 4 || dataSize == 2 || dataSize == 1);

    if (dataSize < 4) return false;

    unsigned src1 = inst->srcRegIdx(0).flatIndex();
    //unsigned src2 = inst->srcRegIdx(1).flatIndex();
    assert(src1 < 38);
    //assert(src2 < 38);
    if ((!regCtx[src1].valid)/* || (!regCtx[src2].valid)*/) {
        DPRINTF(ConstProp, "sources (%d) is invalid\n", regCtx[src1].valid); 
        // if the wripi is not folded and cc flags are valid, then propagte them with microop
        // in some corner cases, the cc flags for a wripi are valid but the microop itself cannot be folded because of inssuficint space 
        if (ccValid) {
            DPRINTF(ConstProp, "Propagating CC flags with wripi microop although the wripi microop itself is not propgated!\n");
            inst->propgatedCCFlags[0] = PredccFlagBits;
            inst->isCCFlagPropagated[0] = true;
            inst->propgatedCCFlags[1] = PredcfofBits;
            inst->isCCFlagPropagated[1] = true;
            inst->propgatedCCFlags[2] = PreddfBit;
            inst->isCCFlagPropagated[2] = true;
            inst->propgatedCCFlags[3] = PredecfBit;
            inst->isCCFlagPropagated[3] = true;
            inst->propgatedCCFlags[4] = PredezfBit;
            inst->isCCFlagPropagated[4] = true;
        }
        else 
        {
            DPRINTF(ConstProp, "CC flags are not valid! We can't propagate them!\n");
        }
        return false;
    }

    if (currentTrace.branchesFolded >= 2)
    {
        DPRINTF(ConstProp, "More than 2 branches are folded in this trace! We can't fold this wripi instruction although src 1 is valid!\n");
        // if the wripi is not folded and cc flags are valid, then propagte them with microop
        // in some corner cases, the cc flags for a wripi are valid but the microop itself cannot be folded because of inssuficint space 
        if (ccValid) {
            DPRINTF(ConstProp, "Propagating CC flags with wripi microop although the wripi microop itself is not propgated!\n");
            inst->propgatedCCFlags[0] = PredccFlagBits;
            inst->isCCFlagPropagated[0] = true;
            inst->propgatedCCFlags[1] = PredcfofBits;
            inst->isCCFlagPropagated[1] = true;
            inst->propgatedCCFlags[2] = PreddfBit;
            inst->isCCFlagPropagated[2] = true;
            inst->propgatedCCFlags[3] = PredecfBit;
            inst->isCCFlagPropagated[3] = true;
            inst->propgatedCCFlags[4] = PredezfBit;
            inst->isCCFlagPropagated[4] = true;
        }
        else 
        {
            DPRINTF(ConstProp, "CC flags are not valid! We can't propagate them!\n");
        }
        return false;
    }
        

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

    // if the wripi is not folded and cc flags are valid, then propagte them with microop
    if (ccValid) {
        DPRINTF(ConstProp, "Propagating CC flags with wripi microop!\n");
        inst->propgatedCCFlags[0] = PredccFlagBits;
        inst->isCCFlagPropagated[0] = true;
        inst->propgatedCCFlags[1] = PredcfofBits;
        inst->isCCFlagPropagated[1] = true;
        inst->propgatedCCFlags[2] = PreddfBit;
        inst->isCCFlagPropagated[2] = true;
        inst->propgatedCCFlags[3] = PredecfBit;
        inst->isCCFlagPropagated[3] = true;
        inst->propgatedCCFlags[4] = PredezfBit;
        inst->isCCFlagPropagated[4] = true;
    }
    else 
    {
        DPRINTF(ConstProp, "CC flags are not valid! We can't propagate them!\n");
    }

    return false;
}
