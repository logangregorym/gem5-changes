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
#include "debug/TraceGen.hh"
#include "cpu/reg_class.hh"

using namespace X86ISA;
using namespace std;



uint64_t SpecTrace::traceIDCounter = 1;



TraceBasedGraph::TraceBasedGraph(TraceBasedGraphParams *p) : SimObject(p), usingControlTracking(p->usingControlTracking), usingCCTracking(p->usingCCTracking) {
    currentTraceIDGettingSuperOptimized = 0;
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

bool TraceBasedGraph::advanceIfControlTransfer(TraceMap::iterator& _trace_it, StaticInstPtr _decodedMicroOp)
{
    return true;
}


bool TraceBasedGraph::advanceTrace(TraceMap::iterator& _trace_it, StaticInstPtr _decodedMicroOp)
{

    // State sanity check
    assert(currentTraceIDGettingSuperOptimized);
    auto trace_it = traceMap.find(currentTraceIDGettingSuperOptimized);
    assert(trace_it != traceMap.end());
    assert((trace_it->second.state == SpecTrace::OptimizationInProcess));
    assert(_trace_it->second.id == currentTraceIDGettingSuperOptimized);

    // macro and micro address of _decodedMicroOp
    Addr macro_addr = _trace_it->second.headAddr.pcAddr;
    uint16_t micro_addr = _trace_it->second.headAddr.uopAddr; 

    assert(_trace_it->second.originalTrace.find(macro_addr) != _trace_it->second.originalTrace.end());

    if (!usingControlTracking || !advanceIfControlTransfer(_trace_it, _decodedMicroOp)) 
    {

        // first check whether we can find next microop for the same macroop
        if (_trace_it->second.originalTrace[macro_addr].find(micro_addr + 1) != _trace_it->second.originalTrace[macro_addr].end())
        {
            // there is still a microop to fetch from the same macroop
            _trace_it->second.headAddr.uopAddr++;
            DPRINTF(SuperOp, "Advancing trace %d to [%#x][%d]\n", currentTraceIDGettingSuperOptimized, _trace_it->second.headAddr.pcAddr, _trace_it->second.headAddr.uopAddr);
            return false;
        }
        else 
        {
            // there is no next microop. advance to the next macroop
            auto macro_it = _trace_it->second.originalTrace.find(macro_addr);
            macro_it++;
            if (macro_it != _trace_it->second.originalTrace.end())
            {
                _trace_it->second.headAddr.pcAddr   = macro_it->first;
                _trace_it->second.headAddr.uopAddr  = 0; 
                DPRINTF(SuperOp, "Advancing trace %d to [%#x][%d]\n", currentTraceIDGettingSuperOptimized, _trace_it->second.headAddr.pcAddr, _trace_it->second.headAddr.uopAddr);
                return false;
            }
            else 
            {
                // end of original trace
                DPRINTF(SuperOp, "End of trace %d at [%#x][%d]\n", currentTraceIDGettingSuperOptimized, _trace_it->second.headAddr.pcAddr, _trace_it->second.headAddr.uopAddr);
                return true;
            }      
        }

    }    

    // we should never be here
    assert(0);
    return false;

}

bool TraceBasedGraph::isPredictionSource(TraceMap::iterator& _trace_it, uint64_t &value, uint64_t &confidence, uint64_t &latency) {

    if (_trace_it == traceMap.end()) return false;
    // macro and micro address of _decodedMicroOp
    Addr macro_addr = _trace_it->second.headAddr.pcAddr;
    uint16_t micro_addr = _trace_it->second.headAddr.uopAddr;

    for (int i=0; i<8; i++) {
        if (_trace_it->second.source[i].valid && _trace_it->second.source[i].addr == FullUopAddr(macro_addr, micro_addr)) {
            value = _trace_it->second.source[i].value;
            confidence = _trace_it->second.source[i].confidence;
            latency = _trace_it->second.source[i].latency;
            return true;
        }
    }
    return false;
}

bool TraceBasedGraph::isTraceStillAvailableInUopCache(SpecTrace::OriginalTrace &trace)
{
    // State sanity check
    assert(currentTraceIDGettingSuperOptimized);
    auto trace_it = traceMap.find(currentTraceIDGettingSuperOptimized);
    assert(trace_it != traceMap.end());
    assert((trace_it->second.state == SpecTrace::OptimizationInProcess) || (trace_it->second.state == SpecTrace::QueuedForFirstTimeOptimization));
        
    // check for trace availablity in microop cache
    bool isTraceStillAvailable = true;
    for (auto const &addr : trace)
    {
        bool found = false;   
        Addr _t_tag = addr.second.begin()->second.tag;
        Addr _t_way = addr.second.begin()->second.way;
        Addr _t_idx = addr.second.begin()->second.idx;

        if (!decoder->uopValidArray[_t_idx][_t_way] || (decoder->uopTagArray[_t_idx][_t_way] != _t_tag))
        {
            isTraceStillAvailable = false;
            break;
        }

        for (size_t u = 0; u < decoder->uopCountArray[_t_idx][_t_way]; u++)
        {   
            
            if (addr.first == decoder->uopAddrArray[_t_idx][_t_way][u].pcAddr)
            {   
                found = true;
                break;
            }
        }

        if (!found)
        {
            isTraceStillAvailable = false;
            break;              
        }
                
    }

    return isTraceStillAvailable;
}

void TraceBasedGraph::predictValue(Addr addr, uint16_t uopAddr, uint64_t value, uint64_t confidence, unsigned latency)
{

    assert(confidence <= 31);

    DPRINTF(SuperOp, "predictValue: addr=%#x:%x value=%#x confidence=%d latency=%d\n", addr, uopAddr, value, confidence, latency);
    
    int idx = (addr >> 5) & 0x1f;
    uint64_t tag = (addr >> 10);


    SpecTrace newTrace;

    // lets find the trace that this prediction belongs to
    uint64_t _max_hotness = 0;
    // capture the trace from uop cache
    for (uint64_t way=0; way < 8; way++) {
        DPRINTF(SuperOp, "Looking up uop[%i][%i] of size %d\n", idx, way, decoder->uopCountArray[idx][way]);
        if (decoder->uopValidArray[idx][way] && decoder->uopTagArray[idx][way] == tag)
        {
            if (decoder->uopHotnessArray[idx][way].read() > _max_hotness) 
                _max_hotness = decoder->uopHotnessArray[idx][way].read();
            

            for (size_t u = 0; u < decoder->uopCountArray[idx][way]; u++)
            {   
                Addr baseAddr = decoder->uopAddrArray[idx][way][u].pcAddr; 
                if (newTrace.originalTrace.find(baseAddr) == newTrace.originalTrace.end())
                {
                    StaticInstPtr original_macro = decoder->decode(decoder->uopCache[idx][way][u], baseAddr);
                    StaticInstPtr super_macro = decoder->decode(decoder->uopCache[idx][way][u], baseAddr);

                    if (original_macro->isMacroop())
                    {
                        for (uint32_t t = 0; t < original_macro->getNumMicroops(); t++)
                        {
                            StaticInstPtr orig_si = original_macro->fetchMicroop((MicroPC)t);
                            orig_si->macroOp = original_macro;
                            newTrace.originalTrace[baseAddr][t] = OriginalMicroop(orig_si ,original_macro, tag, idx, way);

                            StaticInstPtr super_si = super_macro->fetchMicroop((MicroPC)t);
                            super_si->macroOp = super_macro;
                            newTrace.superOptimizedTrace[baseAddr][t] = SuperOptimizedMicroop(super_si ,super_macro, 0 , 0 ,0 , false);                            
                        }
                    }
                    else 
                    {
                        // Does this mean only one microop?
                        // no need to set macroOp
                        newTrace.originalTrace[baseAddr][0] = OriginalMicroop(original_macro ,original_macro, tag, idx, way);
                        newTrace.superOptimizedTrace[baseAddr][0] = SuperOptimizedMicroop(super_macro, super_macro, 0 , 0 ,0 , false);
                    }
                }
            }
        }
            
    }

    assert(!newTrace.originalTrace.empty());
  

    // dump captured trace
    uint64_t length = 0;
    DPRINTF(SuperOp,"Trace in the Uop Cache:\n");
    for (auto const &lv1: newTrace.originalTrace)
    {
        length += lv1.second.size();
        for (auto const &lv2: lv1.second)
        {
            DPRINTF(SuperOp, "[%x][%d] [%s]\n" , lv1.first , lv2.first, lv2.second.microop->disassemble(lv1.first));
        }
    }
    newTrace.length = length;
    newTrace.hotness = _max_hotness;
    // before adding it to the queue, check if profitable -- we prefer long hot traces
    if (newTrace.hotness < 7 || newTrace.length < 4) { // TODO: revisit: pretty low bar
        DPRINTF(SuperOp, "Rejecting trace request to optimize trace\n", idx);
        DPRINTF(SuperOp, "hotness:%i length=%i\n", newTrace.hotness, newTrace.length);
        DPRINTF(SuperOp, "Prediction source: %#x:%i=%#x\n", addr, uopAddr, value);
        return;
    }


    // find head of the original trace in the microop cache
    Addr _t_tag = newTrace.originalTrace.begin()->second.begin()->second.tag;
    Addr _t_way = newTrace.originalTrace.begin()->second.begin()->second.way;
    Addr _t_idx = newTrace.originalTrace.begin()->second.begin()->second.idx;
    assert( _t_tag == decoder->uopTagArray[_t_idx][_t_way] && decoder->uopValidArray[_t_idx][_t_way]);
    for (size_t u = 0; u < decoder->uopCountArray[_t_idx][_t_way]; u++)
    {
        if (decoder->uopAddrArray[_t_idx][_t_way][u].pcAddr == newTrace.originalTrace.begin()->first)
        {
            newTrace.head = FullCacheIdx(_t_idx, _t_way, u);
            DPRINTF(SuperOp, "Trace Head Uop in Cache: UopCache[%d][%d][%d]\n", newTrace.head.idx, newTrace.head.way, newTrace.head.uop);
            break;
        }
    }

    
    

    // sometimes the head is different than the first prediction source  
    // uop: the slot number which holds the first microop for the head instruction
    // uopAddr: the uop at which the prediction source is

    // we always know that the head of a trace is the first microop in the macroop
    newTrace.headAddr = FullUopAddr(newTrace.originalTrace.begin()->first, 0); 
    newTrace.endAddr = FullUopAddr(newTrace.originalTrace.rbegin()->first, newTrace.originalTrace.rbegin()->second.size()-1);

    newTrace.traceHeadAddr = newTrace.originalTrace.begin()->first;
    newTrace.traceEndAddr = newTrace.originalTrace.rbegin()->first;
    DPRINTF(SuperOp, "Trace HeadAddress: %#x Trace EndAddress: %#x\n", newTrace.traceHeadAddr, newTrace.traceEndAddr);
    newTrace.instAddr = FullUopAddr(0, 0); 
    

    
    int numFoundTraces = 0;
    int numTracesWithSamePredSourcesVal = 0;
    int numTracesWithDifferentPredSourcesVal = 0;
    /* Check if we have an optimized trace with this prediction source -- isTraceAvailable returns the most profitable trace. */
    for (auto it = traceMap.begin(); it != traceMap.end(); it++) {

        // if trace head address or end address are not the same then it's not the same trace
        if ((newTrace.traceHeadAddr != it->second.traceHeadAddr) || (newTrace.traceEndAddr != it->second.traceEndAddr)) {
            continue;
        }

        numFoundTraces++;
        // if we are here then it means that both head and end addresses are the same 
        // now let's check if we can find any prediction source with the same address and value
        // In two cases we have different traces:
        // 1) it->second.source[i].addr == FullUopAddr(addr, uopAddr) && it->second.source[i].value != value
        // 2)

        int numValidPredSources = 0;
        int numWithSamePredSourcesVal = 0;
        int numWithDifferentPredSourcesVal = 0;
        for (int i=0; i<8; i++) {

            if (it->second.source[i].valid)
            {
                numValidPredSources++;
            }
            /* Do we already consider this as a prediction source? */
            if (it->second.source[i].valid && 
                it->second.source[i].addr == FullUopAddr(addr, uopAddr) &&
                it->second.source[i].value != value) 
            {
                
                DPRINTF(SuperOp, "Trace Map already holds a trace for this head and end addresses but the predicted value is different! HeadAddr = %#x EndAddr = %#x Trace Predicted Value: %#x New Predicted Value: %#x\n", 
                                  it->second.traceHeadAddr, it->second.traceEndAddr, it->second.source[i].value, value);

                numTracesWithDifferentPredSourcesVal++;
                numWithDifferentPredSourcesVal++;
            }
            else if (it->second.source[i].valid && 
                it->second.source[i].addr == FullUopAddr(addr, uopAddr) &&
                it->second.source[i].value == value) 
            {
                DPRINTF(SuperOp, "Trace Map already holds a trace for this head and end addresses and the predicted value is the same as before! HeadAddr = %#x EndAddr = %#x Trace Predicted Value: %#x New Predicted Value: %#x\n", 
                                  it->second.traceHeadAddr, it->second.traceEndAddr, it->second.source[i].value, value);
                
                numTracesWithSamePredSourcesVal++;
                numWithSamePredSourcesVal++;
            }
        }

        // sanity checks
        assert(numValidPredSources);
        assert(numWithSamePredSourcesVal <= 1);
        assert(numWithDifferentPredSourcesVal <= 1);

        if (it->second.state == SpecTrace::Complete && numWithSamePredSourcesVal == 0 && numWithDifferentPredSourcesVal == 0)
        {
            numTracesWithDifferentPredSourcesVal++;
        }

    }

    DPRINTF(SuperOp, "numFoundTraces: %d, numTracesWithSamePredSourcesVal: %d,  numTracesWithDifferentPredSourcesVal: %d\n", numFoundTraces, numTracesWithSamePredSourcesVal, numTracesWithDifferentPredSourcesVal);
    bool isNewTrace = false;
    if (numFoundTraces == 0)
    {
        isNewTrace = true;
    }
    else if (numFoundTraces > 0 && numTracesWithSamePredSourcesVal > 0 )
    {   
        isNewTrace = false;
    } 
    else if (numFoundTraces > 0 &&  numTracesWithSamePredSourcesVal == 0 && numTracesWithDifferentPredSourcesVal > 0)
    {   
        isNewTrace = true;
    }
    else if (numFoundTraces > 0 && numTracesWithSamePredSourcesVal == 0 && numTracesWithDifferentPredSourcesVal == 0)
    {   
        isNewTrace = false;
    }
      
    if (!isNewTrace) 
    {
        for (auto &lv1: newTrace.originalTrace)
        {
            if (lv1.second[0].macroop->isMacroop())
            {
                lv1.second[0].macroop->deleteMicroOps(); 
            }  
        }
        
        return;
    }



    // capture the prediction source
    newTrace.source[0].valid = true;
    newTrace.source[0].addr = FullUopAddr(addr, uopAddr);
    newTrace.source[0].value = value;
    newTrace.source[0].confidence = confidence;
    newTrace.source[0].latency = latency;
    

    // for all the microops in the trace do a prediction (uop to 7 predictopn sources can be added in addition to original prediction)
    uint64_t pred_source_idx = 1;
    for (auto const& lv1: newTrace.originalTrace)
    {
        for (auto const & lv2: lv1.second)
        {
                
                // don't predcit for the first prediction source
                if (FullUopAddr(addr, uopAddr) == FullUopAddr(lv1.first, lv2.first )) continue;


                LVPredUnit::lvpReturnValues ret;
                if (loadPred->makePredictionForTraceGenStage(lv1.first, lv2.first , 0, ret))
                {   
                    if (ret.confidence >= 5){

                        newTrace.source[pred_source_idx].valid = true;
                        newTrace.source[pred_source_idx].addr = FullUopAddr(lv1.first, lv2.first);
                        newTrace.source[pred_source_idx].value = ret.predictedValue;
                        newTrace.source[pred_source_idx].confidence = ret.confidence;
                        newTrace.source[pred_source_idx].latency = ret.latency;
                        DPRINTF(SuperOp, "Added prediction source: %#x:%i=%#x Confidence: %#x\n",  newTrace.source[pred_source_idx].addr.pcAddr,  newTrace.source[pred_source_idx].addr.uopAddr, newTrace.source[pred_source_idx].value, newTrace.source[pred_source_idx].confidence);

                        pred_source_idx++;
                        if (pred_source_idx == 8) break;
                    }
                }      
                
        }
        if (pred_source_idx == 8) break;
    }
    
    
    
    newTrace.id = SpecTrace::traceIDCounter++;
    newTrace.state = SpecTrace::QueuedForFirstTimeOptimization;
    traceMap[newTrace.id] = newTrace;
    candidateTraceQueue.push(newTrace.id);
    DPRINTF(SuperOp, "Queueing up new trace request %i!\n", newTrace.id);
    DPRINTF(SuperOp, "hotness:%i length=%i\n", newTrace.hotness, newTrace.length);
    DPRINTF(SuperOp, "Prediction source: %#x:%i=%#x\n",  newTrace.source[0].addr.pcAddr,  newTrace.source[0].addr.uopAddr, newTrace.source[0].value);


    return;

}


bool TraceBasedGraph::selectNextTraceForsuperOptimization()
{
    // this function should never called when a trace in currently getting superoptimized
    assert(currentTraceIDGettingSuperOptimized == 0);

    if (candidateTraceQueue.empty())
    {
        DPRINTF(SuperOp, "No Trace is available for super optimization!\n");
        return false;
    }

    // Pop a new trace from the queue, start at top
    while(!candidateTraceQueue.empty()) {
            
        assert(candidateTraceQueue.front());

        currentTraceIDGettingSuperOptimized = candidateTraceQueue.front();
        auto trace_it = traceMap.find(currentTraceIDGettingSuperOptimized);

        assert(trace_it != traceMap.end());

        
        bool isTraceStillAvailable = isTraceStillAvailableInUopCache(trace_it->second.originalTrace);

        // if the trace is evicted don't start the optimization. Simply just go for the next trace
        if (!isTraceStillAvailable)
        {
            DPRINTF(SuperOp, "Trace %i is evicted before we could process it.\n", trace_it->second.id);
            // remove it from traceMap
            DPRINTF(Decoder, "Removing traceID: %d (reoptID: %d) from Trace Map because of eviction!.\n", trace_it->second.id );
            // remove all allocated microops
            //TODO: DELETE ALL MICROOPS IN BOTH ORIGINAL AND SUPEROPTMIZED MAPS
            traceMap.erase(trace_it);
            candidateTraceQueue.pop();
            tracesPoppedFromQueue++;
            currentTraceIDGettingSuperOptimized = 0;

        }
        else 
        {
            // if the trace is available assign it to currentTrace for optimization and break 
            candidateTraceQueue.pop();
            tracesPoppedFromQueue++;

            assert(currentTraceIDGettingSuperOptimized);
            assert(currentTraceIDGettingSuperOptimized == trace_it->second.id);
            assert(trace_it->second.state == SpecTrace::QueuedForFirstTimeOptimization);
            assert(!trace_it->second.originalTrace.empty());
            DPRINTF(SuperOp, "Trace %i is available and selected for super optimization!.\n", trace_it->second.id);

            // Change the trace status
            trace_it->second.state  = SpecTrace::OptimizationInProcess;
            return true;
        }       

    };

    // in case we can't find a valid trace to super optimize
    if (currentTraceIDGettingSuperOptimized == 0)
    {
        DPRINTF(SuperOp, "All the traces were evicted therefore nothing to optimize!\n");
        return false;
    }

    // we should never reach here!
    assert(0);
    return false;

    
}


void TraceBasedGraph::finalizeSuperOptimizedTrace()
{

    // State sanity check
    assert(currentTraceIDGettingSuperOptimized);
    auto trace_it = traceMap.find(currentTraceIDGettingSuperOptimized);
    assert(trace_it != traceMap.end());
    assert(trace_it->second.state == SpecTrace::Complete || trace_it->second.state == SpecTrace::Evicted);

    // don't delete this!
    if (trace_it->second.state == SpecTrace::Evicted)
    {
        // trace was removed from uop cache before we were able to super optmize it
        // remove it
        // remove all allocated microops
        //TODO: DELETE ALL MICROOPS IN BOTH ORIGINAL AND SUPEROPTMIZED MAPS
        traceMap.erase(trace_it);
        currentTraceIDGettingSuperOptimized = 0;
        return;

    }

    //TODO: Reject to insert the trace into the spec cache if shrinkage is not enough!
    

    DPRINTF(SuperOp, "Done optimizing trace %i with actual length %i, shrunk to length %i\n", trace_it->second.id, trace_it->second.length, trace_it->second.shrunkLength);
    DPRINTF(SuperOp, "Before optimization: \n");
    DPRINTF(SuperOp,"Trace in the Uop Cache:\n");
    for (auto const &lv1: trace_it->second.originalTrace)
    {
        for (auto const &lv2: lv1.second)
        {
            DPRINTF(SuperOp, "[%x][%d] [%s]\n" , lv1.first , lv2.first, lv2.second.microop->disassemble(lv1.first));
        }
    }

    // TODO: Sanity check to make sure that trace is in spec cache
    
    DPRINTF(SuperOp, "After optimization: \n");
    DPRINTF(SuperOp,"Trace in the Spec Cache:\n");
    for (auto const &lv1: trace_it->second.superOptimizedTrace)
    {
        for (auto const &lv2: lv1.second)
        {
            if (!lv2.second.compacted)
            {
                DPRINTF(SuperOp, "[%x][%d] [%s]\n" , lv1.first , lv2.first, lv2.second.microop->disassemble(lv1.first));
            }
                
        }
    }

    // set the first and last instructions in super optimized trace
    [&] {
        for (auto addr_it = trace_it->second.superOptimizedTrace.begin(); addr_it != trace_it->second.superOptimizedTrace.end(); addr_it++)
        {
            for (auto micro_it = addr_it->second.begin(); micro_it != addr_it->second.end(); micro_it++)
            {
                if (!micro_it->second.compacted)
                {
                    trace_it->second.superHeadAddr = FullUopAddr(addr_it->first,micro_it->first);
                    return;
                }
            }

        }
    }();


    // Set the prevNonEliminatedInst here! it will be used to propagate the live outs and also as the end of trace
    [&] {
        for (auto addr_it = trace_it->second.superOptimizedTrace.rbegin(); addr_it != trace_it->second.superOptimizedTrace.rend(); addr_it++)
        {
            for (auto micro_it = addr_it->second.rbegin(); micro_it != addr_it->second.rend(); micro_it++)
            {
                if (!micro_it->second.compacted)
                {
                    trace_it->second.prevNonEliminatedInst = micro_it->second.microop;
                    trace_it->second.superEndAddr = FullUopAddr(addr_it->first,micro_it->first);
                    return;
                }
            }

        }
    }();
    // mark end of trace and propagate live outs
    DPRINTF(SuperOp, "End of Trace at %s!\n", trace_it->second.prevNonEliminatedInst->getName());
                
    // here we mark 'prevNonEliminatedInst' as end of the trace because sometimes an eliminated instruction can be set as end of the trace
    trace_it->second.prevNonEliminatedInst->setEndOfTrace();
    trace_it->second.prevNonEliminatedInst->shrunkLength = trace_it->second.shrunkLength;
    trace_it->second.prevNonEliminatedInst->traceLength = trace_it->second.length;
    if (!trace_it->second.prevNonEliminatedInst->isControl()) { // control prevNonEliminatedInstructions already propagate live outs
        for (int i=0; i<16; i++) { // 16 int registers
            if (regCtx[i].valid && !regCtx[i].source) {
                trace_it->second.prevNonEliminatedInst->liveOut[trace_it->second.prevNonEliminatedInst->numDestRegs()] = regCtx[i].value;
                trace_it->second.prevNonEliminatedInst->liveOutPredicted[trace_it->second.prevNonEliminatedInst->numDestRegs()] = true;
                trace_it->second.prevNonEliminatedInst->addDestReg(RegId(IntRegClass, i));
                trace_it->second.prevNonEliminatedInst->_numIntDestRegs++;
            }
        }
        if (ccValid) {
            trace_it->second.prevNonEliminatedInst->liveOut[trace_it->second.prevNonEliminatedInst->numDestRegs()] = PredccFlagBits;
            trace_it->second.prevNonEliminatedInst->liveOutPredicted[trace_it->second.prevNonEliminatedInst->numDestRegs()] = true;
            trace_it->second.prevNonEliminatedInst->addDestReg(RegId(CCRegClass, CCREG_ZAPS));
            trace_it->second.prevNonEliminatedInst->liveOut[trace_it->second.prevNonEliminatedInst->numDestRegs()] = PredcfofBits;
            trace_it->second.prevNonEliminatedInst->liveOutPredicted[trace_it->second.prevNonEliminatedInst->numDestRegs()] = true;
            trace_it->second.prevNonEliminatedInst->addDestReg(RegId(CCRegClass, CCREG_CFOF));
            trace_it->second.prevNonEliminatedInst->liveOut[trace_it->second.prevNonEliminatedInst->numDestRegs()] = PreddfBit;
            trace_it->second.prevNonEliminatedInst->liveOutPredicted[trace_it->second.prevNonEliminatedInst->numDestRegs()] = true;
            trace_it->second.prevNonEliminatedInst->addDestReg(RegId(CCRegClass, CCREG_DF));
            trace_it->second.prevNonEliminatedInst->liveOut[trace_it->second.prevNonEliminatedInst->numDestRegs()] = PredecfBit;
            trace_it->second.prevNonEliminatedInst->liveOutPredicted[trace_it->second.prevNonEliminatedInst->numDestRegs()] = true;
            trace_it->second.prevNonEliminatedInst->addDestReg(RegId(CCRegClass, CCREG_ECF));
            trace_it->second.prevNonEliminatedInst->liveOut[trace_it->second.prevNonEliminatedInst->numDestRegs()] = PredezfBit;
            trace_it->second.prevNonEliminatedInst->liveOutPredicted[trace_it->second.prevNonEliminatedInst->numDestRegs()] = true;
            trace_it->second.prevNonEliminatedInst->addDestReg(RegId(CCRegClass, CCREG_EZF));
            trace_it->second.prevNonEliminatedInst->_numCCDestRegs += 5;
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
    } 
    else 
    {
        DPRINTF(SuperOp, "No live out CC\n");
    }
    for (int i=0; i<8; i++) 
    {
        DPRINTF(SuperOp, "Prediction Source %i\n", i);
        if (trace_it->second.source[i].valid) {
            DPRINTF(SuperOp, "Address=%#x:%i, Value=%#x, Confidence=%i, Latency=%i\n",
                                    trace_it->second.source[i].addr.pcAddr,  trace_it->second.source[i].addr.uopAddr,
                                    trace_it->second.source[i].value, trace_it->second.source[i].confidence,
                                    trace_it->second.source[i].latency);
        }
    }

        
    
    /* Clear Reg Context Block. */
    for (int i=0; i<38; i++) {
        regCtx[i].valid = regCtx[i].source = ccValid = false;
        PredccFlagBits = PredcfofBits = PreddfBit = PredecfBit = PredezfBit = 0;
    }

    // set currentTraceIDGettingSuperOptimized
    currentTraceIDGettingSuperOptimized = 0;


    // insert it into the Spec Cache
    std::vector<uint64_t> evictedTraces; evictedTraces.clear();
    bool updateSuccesfull = decoder->specCache->addToSpeculativeCache(trace_it->second.traceHeadAddr, trace_it->second.id, trace_it->second.shrunkLength, evictedTraces);
    assert(updateSuccesfull);

    // remove all the evictedc traces from spec cache
    for (auto const& elem: evictedTraces)
    {
        assert(traceMap.find(elem) != traceMap.end());
        traceMap.erase(elem);
    }


    DPRINTF(TraceGen,"#########################################################################################################################################\n");
    DPRINTF(TraceGen,"******************************* ORIGINAL TRACE START [%#x][%d] ***************************************\n",  trace_it->second.traceHeadAddr, 0) ;
    for (auto const &lv1: trace_it->second.originalTrace)
    {
            for (auto const &lv2: lv1.second)
            {
                DPRINTF(TraceGen, "[%x][%d] [%s]\n" , lv1.first , lv2.first, lv2.second.microop->disassemble(lv1.first));
            }
    }
    DPRINTF(TraceGen, "-------------------------------- ORIGINAL TRACE END [%#x][%d] -----------------------------------------\n",  trace_it->second.endAddr.pcAddr, trace_it->second.endAddr.uopAddr) ;
    // assert(trace_it->second.traceHeadAddr == trace_it->second.endAddr.pcAddr);
    DPRINTF(TraceGen,"-------------------------------- SUPEROPTIMIZED TRACE START [%#x][%d] -----------------------------------------\n",  trace_it->second.superHeadAddr.pcAddr,  trace_it->second.superHeadAddr.uopAddr) ;
    for (auto const &lv1: trace_it->second.superOptimizedTrace)
    {
        for (auto const &lv2: lv1.second)
        {
                if (!lv2.second.compacted)
                {
                    DPRINTF(TraceGen, "[%x][%d] [%s]\n" , lv1.first , lv2.first, lv2.second.microop->disassemble(lv1.first));
                }
                    
        }
    }
    DPRINTF(TraceGen, "Trace %i with actual length %i, shrunk to length %i\n", trace_it->second.id, trace_it->second.length, trace_it->second.shrunkLength);
    DPRINTF(TraceGen, "Live Outs:\n");
    for (int i=0; i<16; i++) {
            if (regCtx[i].valid && !regCtx[i].source)
                DPRINTF(TraceGen, "reg[%i]=%#x\n", i, regCtx[i].value);
    }
    if (ccValid) {
            DPRINTF(TraceGen, "PredccFlagBits: %#x\n", PredccFlagBits);
            DPRINTF(TraceGen, "PredcfofBits: %#x\n", PredcfofBits);
            DPRINTF(TraceGen, "PreddfBit: %#x\n", PreddfBit);
            DPRINTF(TraceGen, "PredecfBit: %#x\n", PredecfBit);
            DPRINTF(TraceGen, "PredezfBit: %#x\n", PredezfBit);
    } 
    else 
    {
            DPRINTF(TraceGen, "No live out CC\n");
    }
    for (int i=0; i<8; i++) 
    {
            DPRINTF(TraceGen, "Prediction Source %i\n", i);
            if (trace_it->second.source[i].valid) {
                DPRINTF(TraceGen, "Address=%#x:%i, Value=%#x, Confidence=%i, Latency=%i\n",
                                        trace_it->second.source[i].addr.pcAddr,  trace_it->second.source[i].addr.uopAddr,
                                        trace_it->second.source[i].value, trace_it->second.source[i].confidence,
                                        trace_it->second.source[i].latency);
            }
    }

    DPRINTF(TraceGen, "******************************* SUPEROPTIMIZED TRACE END [%#x][%d] ***************************************\n",  trace_it->second.superEndAddr.pcAddr, trace_it->second.superEndAddr.uopAddr) ;
    DPRINTF(TraceGen,"#########################################################################################################################################\n");
    

}


bool TraceBasedGraph::updateSpecTrace(TraceMap::iterator& _trace_it, StaticInstPtr _decodedMicroOp, bool &isDeadCode , bool propagated) {

    // macro and micro address of _decodedMicroOp
    Addr macro_addr = _trace_it->second.headAddr.pcAddr;
    uint16_t micro_addr = _trace_it->second.headAddr.uopAddr;

    // Rather than checking dests, check sources; if all sources, then all dests in trace
    bool allSrcsReady = true;
/*    for (int i=0; i<trace.inst->numSrcRegs(); i++) {
        RegId srcReg = trace.inst->srcRegIdx(i);
        allSrcsReady = allSrcsReady && regCtx[srcReg.flatIndex()].valid;
    }*/

    string type = _decodedMicroOp->getName();
    isDeadCode =    (type == "rdip") || 
                    (allSrcsReady && 
                    (type == "mov" || 
                    type == "movi" || 
                    type == "limm" || 
                    type == "add" || 
                    type == "addi" || 
                    type == "sub" || 
                    type == "subi" || 
                    type == "and" || 
                    type == "andi" || 
                    type == "or" || 
                    type == "ori" || 
                    type == "xor" || 
                    type == "xori" || 
                    type == "slri" || 
                    type == "slli" || 
                    type == "sexti" || 
                    type == "zexti"));

    // Prevent an inst registering as dead if it is a prediction source or if it is a return or it modifies CC
    uint64_t value;
    uint64_t confidence;
    uint64_t latency;
    bool isPredSource = isPredictionSource(_trace_it, value, confidence, latency) && type != "limm" && type != "movi" && type != "rdip";
    isDeadCode &= (propagated && !isPredSource && !((!usingCCTracking && _decodedMicroOp->isCC()) || _decodedMicroOp->isReturn()));

    DPRINTF(ConstProp, "isDeadCode:%d propagated:%d isPredSource:%d CC:%d Return:%d\n", isDeadCode, propagated, isPredSource, (!usingCCTracking && _decodedMicroOp->isCC()), _decodedMicroOp->isReturn());
    if (allSrcsReady && (!usingCCTracking && _decodedMicroOp->isCC()))
    {
        DPRINTF(ConstProp, "All sources are ready for instruction at %#x:%#x but it is not a dead code as it's a CC inst!\n", macro_addr, micro_addr);
    }
    else if (allSrcsReady && !propagated)
    {
        DPRINTF(ConstProp, "All sources are ready for instruction at %#x:%#x but it is not a dead code as its data size is less than 4/8 bytes!\n", macro_addr, micro_addr);
    }

    // Inst will never already be in this trace, single pass
    if (isDeadCode) {
        DPRINTF(ConstProp, "Dead code at %#x:%#x\n", macro_addr, micro_addr);
        DPRINTF(Decoder, "Skipping microop update in the speculative cache\n");
        return true;
    }

    // TODO! add spec cache logic
    //bool updateSuccessful = decoder->addUopToSpeculativeCache( trace, isPredSource);
    bool updateSuccessful = true; updateSuccessful = updateSuccessful;
    _trace_it->second.shrunkLength++;



    // TODO: what is this?!
    // Step 3b: Mark all predicted values on the StaticInst -- don't do this for prediction sources
    // update live outs -- don't do this for prediction sources
    // if (!isPredSource) {
    //     for (int i=0; i<_decodedMicroOp->numDestRegs(); i++) {
    //         RegId destReg = _decodedMicroOp->destRegIdx(i);
    //         if (destReg.classValue() == IntRegClass) {
    //             regCtx[destReg.flatIndex()].valid = false;
    //         }
    //     }
    // }


    // Update head of the optimized trace
    // if (!trace.optimizedHead.valid) {
    //     DPRINTF(Decoder, "updateSpecTrace: Trace %d optimized head is not valid!\n", trace.id);
    //     trace.optimizedHead.idx = trace.head.idx; 
    //     trace.optimizedHead.uop = 0;
    //     for (int way=0; way<8; way++) {
    //         int idx = trace.head.idx;
    //         if (decoder->speculativeValidArray[idx][way] && decoder->speculativeTraceIDArray[idx][way] == trace.id) {
    //             DPRINTF(Decoder, "updateSpecTrace: Trace %d optimized head way is updated to %d!\n", trace.id, way);
    //             trace.optimizedHead.way = way;
    //             trace.optimizedHead.valid = true;
    //             break;
    //         }
    //     }
    // }

    return updateSuccessful;
}

bool TraceBasedGraph::generateNextSuperOptimizedTraceInst() {

    // State sanity check
    assert(currentTraceIDGettingSuperOptimized);
    auto trace_it = traceMap.find(currentTraceIDGettingSuperOptimized);
    assert(trace_it != traceMap.end());
    assert(trace_it->second.state == SpecTrace::OptimizationInProcess);
    assert(trace_it->second.id == currentTraceIDGettingSuperOptimized);

    DPRINTF(SuperOp, "Optimizing trace %i\n", trace_it->second.id);
    


    StaticInstPtr decodedOriginalMicroOp = NULL;
    StaticInstPtr decodedSuperMicroop = NULL;
    // Check trace availablity in uop cache
    if (!isTraceStillAvailableInUopCache(trace_it->second.originalTrace)) 
    {
        tracesWithInvalidHead++;
        DPRINTF(SuperOp, "Trace was evicted out of the micro-op cache before we could optimize it\n");
        // set state to evicted in the traceMap
        trace_it->second.state = SpecTrace::Evicted;
        // return true to signal that optimization is done for this trace
        return true;
    }
        
    Addr macro_addr = trace_it->second.headAddr.pcAddr;
    uint16_t micro_addr = trace_it->second.headAddr.uopAddr;
    assert(trace_it->second.originalTrace.find(macro_addr) != trace_it->second.originalTrace.end());
    assert(trace_it->second.originalTrace[macro_addr].find(micro_addr) != trace_it->second.originalTrace[macro_addr].end());    
    decodedOriginalMicroOp = trace_it->second.originalTrace[macro_addr][micro_addr].microop;
    decodedSuperMicroop = trace_it->second.superOptimizedTrace[macro_addr][micro_addr].microop;
    DPRINTF(ConstProp, "Trace %i: Processing instruction [%#x][%d][%s]\n", currentTraceIDGettingSuperOptimized, macro_addr, micro_addr, decodedOriginalMicroOp->disassemble(macro_addr));

    if (decodedOriginalMicroOp->getName() == "NOP" /*|| decodedOriginalMicroOp->getName() == "fault" || decodedOriginalMicroOp->getName() == "popcnt_Gv_Ev"*/) 
    {
        // trace_it->second.length++;
        trace_it->second.superOptimizedTrace[macro_addr][micro_addr].compacted = true;
        bool isEndOfTrace = advanceTrace(trace_it, decodedOriginalMicroOp);    
        if (isEndOfTrace) 
        {
            trace_it->second.state = SpecTrace::Complete;
            return true;
        }
        return false;
    }

        
    bool newMacro = (micro_addr == 0);
    


    if (newMacro) {
        /* Clear Micro Registers in Reg Context Block. */
        for (int i=16; i<38; i++) {
            regCtx[i].valid = regCtx[i].source = false;
        }
    }

    bool updateSuccessful = false;

    // Any inst in a trace may be a prediction source
    uint64_t value = 0;
    uint64_t confidence = 0;
    uint64_t latency = 0;
    string type = decodedSuperMicroop->getName();
    if (type != "rdip" && type != "limm" && type != "movi" && isPredictionSource(trace_it, value, confidence, latency)) {
        // Step 1: Get predicted value from LVP
        // Step 2: Determine dest register(s)
        // Step 3: Annotate dest register entries with that value
        // Step 4: Add inst to speculative trace
        for (int i = 0; i < decodedSuperMicroop->numDestRegs(); i++) {
            RegId destReg = decodedSuperMicroop->destRegIdx(i);
            if (destReg.classValue() == IntRegClass) {
				DPRINTF(SuperOp, "Setting regCtx[%i] to %x from %s inst\n", destReg.flatIndex(), value, type);
                regCtx[destReg.flatIndex()].value = decodedSuperMicroop->predictedValue = value;
                regCtx[destReg.flatIndex()].valid = regCtx[destReg.flatIndex()].source = decodedSuperMicroop->predictedLoad = true;
                decodedSuperMicroop->confidence = confidence;
            }
        }

        bool isDeadCode = false;
        updateSuccessful = updateSpecTrace(trace_it, decodedSuperMicroop, isDeadCode, false);
        assert(updateSuccessful);
        //TODO: update way, idx, tag later! 
        trace_it->second.superOptimizedTrace[macro_addr][micro_addr].compacted = false;

        // source predictions never get eliminated
        if (isDeadCode)
        {
            panic("Prediction Source is a dead code?!");
        }

    } else {

        bool propagated = false;
        bool folded = false;
        // Propagate predicted values
        if (type == "mov") {
            DPRINTF(ConstProp, "Found a MOV at [%#x][%i], compacting...\n", macro_addr, micro_addr);
            propagated = propagateMov(decodedSuperMicroop);
        } else if (type == "rdip") {
            DPRINTF(ConstProp, "Found an RDIP at [%#x][%i], compacting...\n", macro_addr, micro_addr);
            RegId destReg = decodedSuperMicroop->destRegIdx(0);
            assert(decodedSuperMicroop->macroOp);
            regCtx[destReg.flatIndex()].value = macro_addr + decodedSuperMicroop->macroOp->getMacroopSize();
            regCtx[destReg.flatIndex()].valid = propagated = true;
            DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", regCtx[destReg.flatIndex()].value, destReg.flatIndex());
        } else if (type == "wrip") {
            // TODO! add this logic
            //DPRINTF(ConstProp, "Found a WRIP branch at [%#x][%i], compacting...\n", macro_addr, micro_addr);
            //propagated = folded = propagateWrip(decodedSuperMicroop);
        } else if (type == "wripi") {
            // TODO! add this logic
            //DPRINTF(ConstProp, "Found a WRIPI branch at [%#x][%i], compacting...\n", macro_addr, micro_addr);
            //propagated = folded = propagateWripI(decodedSuperMicroop);
        } else if (decodedSuperMicroop->isControl()) {
            // TODO! add this logic
            // printf("Control instruction of type %s\n", type);
            // TODO: check for stopping condition or predicted target
        } else if (type == "movi") {
            DPRINTF(ConstProp, "Found a MOVI at [%#x][%i], compacting...\n", macro_addr, micro_addr);
            propagated = propagateMovI(decodedSuperMicroop);
        } else if (type == "and") {
            DPRINTF(ConstProp, "Found an AND at [%#x][%i], compacting...\n", macro_addr, micro_addr);
            propagated = propagateAnd(decodedSuperMicroop);
        } else if (type == "add") {
            DPRINTF(ConstProp, "Found an ADD at [%#x][%i], compacting...\n", macro_addr, micro_addr);
            propagated = propagateAdd(decodedSuperMicroop);
        } else if (type == "sub") {
            DPRINTF(ConstProp, "Found a SUB at [%#x][%i], compacting...\n", macro_addr, micro_addr);
            propagated = propagateSub(decodedSuperMicroop);
        } else if (type == "xor") {
            DPRINTF(ConstProp, "Found an XOR at [%#x][%i], compacting...\n", macro_addr, micro_addr);
            propagated = propagateXor(decodedSuperMicroop);
        } else if (type == "or") {
            DPRINTF(ConstProp, "Found an OR at [%#x][%i], compacting...\n", macro_addr, micro_addr);
            propagated = propagateOr(decodedSuperMicroop);
        } else if (type == "subi") {
            DPRINTF(ConstProp, "Found a SUBI at [%#x][%i], compacting...\n", macro_addr, micro_addr);
            propagated = propagateSubI(decodedSuperMicroop);
        } else if (type == "addi") {
            DPRINTF(ConstProp, "Found an ADDI at [%#x][%i], compacting...\n", macro_addr, micro_addr);
            propagated = propagateAddI(decodedSuperMicroop);
        } else if (type == "slli") {
            DPRINTF(ConstProp, "Found a SLLI at [%#x][%i], compacting...\n", macro_addr, micro_addr);
            propagated = propagateSllI(decodedSuperMicroop);
        } else if (type == "srli") {
            DPRINTF(ConstProp, "Found a SRLI at [%#x][%i], compacting...\n", macro_addr, micro_addr);
            propagated = propagateSrlI(decodedSuperMicroop);
        } else if (type == "lea") {
            DPRINTF(ConstProp, "Type is LEA");
            // Requires multiple ALU operations to propagate, not using
        } else if (type == "sexti") {
            // Implementation has multiple ALU operations, but this is not required by the nature of the operation
            DPRINTF(ConstProp, "Found a SEXTI at [%#x][%i], compacting...\n", macro_addr, micro_addr);
            propagated = propagateSExtI(decodedSuperMicroop);
        } else if (type == "zexti") {
            // Implementation has multiple ALU operations, but this is not required by the nature of the operation
            DPRINTF(ConstProp, "Found a ZEXTI at [%#x][%i], compacting...\n", macro_addr, micro_addr);
            propagated = propagateZExtI(decodedSuperMicroop);
        } else if (type == "mul1s" || type == "mul1u" || type == "mulel" || type == "muleh") {
            DPRINTF(ConstProp, "Type is MUL1S, MUL1U, MULEL, or MULEH\n");
            // TODO: two dest regs with different values? maybe too complex arithmetic?
        } else if (type == "limm") {
            DPRINTF(ConstProp, "Found a LIMM at [%#x][%i], compacting...\n", macro_addr, micro_addr);
            propagated = propagateLimm(decodedSuperMicroop);
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


        // TODO: what to do for folded?!
        bool isDeadCode = false;
        if (!folded) {
            updateSuccessful = updateSpecTrace(trace_it, decodedSuperMicroop, isDeadCode, propagated);
            assert(updateSuccessful);
            
        }

        if (propagated)
        {
            trace_it->second.superOptimizedTrace[macro_addr][micro_addr].compacted = true;
        }
        else 
        {
            trace_it->second.superOptimizedTrace[macro_addr][micro_addr].compacted = false;
        }

        
    }



    // Propagate live outs at the end of each control instruction
    if (decodedSuperMicroop->isControl() && updateSuccessful) {
        for (int i=0; i<16; i++) { // 16 int registers
            if (regCtx[i].valid && !regCtx[i].source) {
                decodedSuperMicroop->liveOut[decodedSuperMicroop->numDestRegs()] = regCtx[i].value;
                decodedSuperMicroop->liveOutPredicted[decodedSuperMicroop->numDestRegs()] = true;
                decodedSuperMicroop->addDestReg(RegId(IntRegClass, i));
                decodedSuperMicroop->_numIntDestRegs++;
            }
        }
        if (ccValid) {
            decodedSuperMicroop->liveOut[decodedSuperMicroop->numDestRegs()] = PredccFlagBits;
            decodedSuperMicroop->liveOutPredicted[decodedSuperMicroop->numDestRegs()] = true;
            decodedSuperMicroop->addDestReg(RegId(CCRegClass, CCREG_ZAPS));
            decodedSuperMicroop->liveOut[decodedSuperMicroop->numDestRegs()] = PredcfofBits;
            decodedSuperMicroop->liveOutPredicted[decodedSuperMicroop->numDestRegs()] = true;
            decodedSuperMicroop->addDestReg(RegId(CCRegClass, CCREG_CFOF));
            decodedSuperMicroop->liveOut[decodedSuperMicroop->numDestRegs()] = PreddfBit;
            decodedSuperMicroop->liveOutPredicted[decodedSuperMicroop->numDestRegs()] = true;
            decodedSuperMicroop->addDestReg(RegId(CCRegClass, CCREG_DF));
            decodedSuperMicroop->liveOut[decodedSuperMicroop->numDestRegs()] = PredecfBit;
            decodedSuperMicroop->liveOutPredicted[decodedSuperMicroop->numDestRegs()] = true;
            decodedSuperMicroop->addDestReg(RegId(CCRegClass, CCREG_ECF));
            decodedSuperMicroop->liveOut[decodedSuperMicroop->numDestRegs()] = PredezfBit;
            decodedSuperMicroop->liveOutPredicted[decodedSuperMicroop->numDestRegs()] = true;
            decodedSuperMicroop->addDestReg(RegId(CCRegClass, CCREG_EZF));
            decodedSuperMicroop->_numCCDestRegs += 5;
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

    assert(updateSuccessful);
    bool isEndOfTrace;
    // Simulate a stall if update to speculative cache wasn't successful
    if (updateSuccessful) {
        isEndOfTrace = advanceTrace(trace_it, decodedOriginalMicroOp);
    }

    if (isEndOfTrace) 
    {
        trace_it->second.state = SpecTrace::Complete;
        return true;
    }

    return false;
}

/*
bool TraceBasedGraph::advanceIfControlTransfer(SpecTrace &trace) {
    assert(0);
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


*/
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
    assert(0);
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
    assert(0);
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
