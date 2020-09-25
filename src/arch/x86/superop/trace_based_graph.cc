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
  unsigned traceId = decoder->isTraceAvailable(addr);
	int idx = (addr >> 5) & 0x1f;

  if (traceId) {
    SpecTrace trace = traceMap[traceId];
    for (int i=0; i<4; i++) {
      /* Do we already consider this as a prediction source? */
      if (trace.source[i].valid && trace.source[i].addr == FullUopAddr(addr, uopAddr)) {
        return;
      }
    }
    for (int i=0; i<4; i++) {
      /* Have we exhausted all prediction sources? If not, we can further compact this trace.  */
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
              newTrace.head = newTrace.addr = FullCacheIdx(idx, way, uopAddr);
              traceMap[newTrace.id] = newTrace;
              traceQueue.push(newTrace);
              DPRINTF(SuperOp, "Queueing up new trace %i to reoptimize trace %i at spec[%i][%i][%i]\n", newTrace.id, newTrace.reoptId, idx, way, uopAddr);
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
          newTrace.id = SpecTrace::traceIDCounter++;
          newTrace.source[0].valid = true;
          newTrace.source[0].addr = FullUopAddr(addr, uopAddr);
          newTrace.source[0].value = value;
          newTrace.state = SpecTrace::QueuedForFirstTimeOptimization;
          newTrace.head = newTrace.addr = FullCacheIdx(idx, way, uopAddr);
          traceMap[newTrace.id] = newTrace;
          traceQueue.push(newTrace);
          DPRINTF(SuperOp, "Queueing up new trace %i to optimize trace at uop[%i][%i][%i]\n", newTrace.id, idx, way, uopAddr);
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
  // select cache to advance from
  if (trace.state == SpecTrace::QueuedForFirstTimeOptimization || trace.state == SpecTrace::OptimizationInProcess) {
    if (trace.addr.uop >= (decoder->uopCountArray[trace.addr.idx][trace.addr.way]-1)) {
      trace.addr.uop = 0;
      if (decoder->uopNextWayArray[trace.addr.idx][trace.addr.way] != 10) {
        trace.addr.way = decoder->uopNextWayArray[trace.addr.idx][trace.addr.way];
      }
    } else {
      return;
    }
  } else {
    if (trace.addr.uop >= (decoder->speculativeCountArray[trace.addr.idx][trace.addr.way]-1)) {
      trace.addr.uop = 0;
      if (decoder->speculativeNextWayArray[trace.addr.idx][trace.addr.way] != 10) {
        trace.addr.way = decoder->speculativeNextWayArray[trace.addr.idx][trace.addr.way];
      }
    } else {
      return;
    }
  }
  trace.addr.valid = false;
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
    DPRINTF(SuperOp, "%p:%i -- %s\n", pcAddr, uopAddr, decodedMicroOp->disassemble(pcAddr));  

    if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
  } else {
    Addr pcAddr = decoder->speculativeAddrArray[idx][way][uop].pcAddr;
    Addr uopAddr = decoder->speculativeAddrArray[idx][way][uop].uopAddr;
    StaticInstPtr decodedMicroOp = decoder->speculativeCache[idx][way][uop];
    DPRINTF(SuperOp, "%p:%i -- %s\n", pcAddr, uopAddr, decodedMicroOp->disassemble(pcAddr));  
  }

  advanceTrace(trace);
  dumpTrace(trace);
}

bool TraceBasedGraph::generateNextTraceInst() {
	if (!currentTrace.addr.valid) { 
		// Pop a new trace from the queue, start at top
    DPRINTF(SuperOp, "Done optimizing trace %i with length %i\n", currentTrace.id, currentTrace.length);
    for (int way=0; way<8; way++) {
      int idx = currentTrace.head.idx;
      int uop = currentTrace.head.uop;
      if (decoder->speculativeValidArray[idx][way] && decoder->speculativeTraceIDArray[idx][way] == currentTrace.id) {
        currentTrace.addr = FullCacheIdx(idx, way, uop);
        currentTrace.state = SpecTrace::Complete;
        traceMap[currentTrace.id] = currentTrace;
        dumpTrace(currentTrace);
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
    currentTrace.length++;
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

	// Any inst in a trace may be a prediction source
  if (isPredictionSource(currentTrace, currentTrace.instAddr)) {
    // Step 1: Get predicted value from LVP
    // Step 2: Determine dest register(s)
    // Step 3: Annotate dest register entries with that value
    // Step 4: Add inst to speculative trace
    uint64_t predictedValue = decoder->cpu->getLVP()->getValuePredicted(decoder->uopAddrArray[idx][way][uop].pcAddr);
    for (int i = 0; i < currentTrace.inst->numDestRegs(); i++) {
      RegId destReg = currentTrace.inst->destRegIdx(i);
      registerValue[destReg.flatIndex()] = predictedValue;
      registerValid[destReg.flatIndex()] = true;
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

		updateSuccessful = updateSpecTrace(currentTrace);
	}

	// Simulate a stall if update to speculative cache wasn't successful
	if (updateSuccessful) {
    advanceTrace(currentTrace);
  }
  //if (decodedMacroOp && decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();

	return true;
}

bool TraceBasedGraph::updateSpecTrace(SpecTrace &trace) {
	// IMPORTANT NOTE: This is written assuming the trace will be traversed in order, and so the register map will be accurate for the current point in the trace

  bool updateSuccessful = false;

	// Rather than checking dests, check sources; if all sources, then all dests in trace
	bool allSrcsReady = true;
	for (int i=0; i<trace.inst->numSrcRegs(); i++) {
		RegId srcReg = trace.inst->srcRegIdx(i);
		allSrcsReady = allSrcsReady && registerValid[srcReg.flatIndex()];
	}

	string type = trace.inst->getName();
	bool isDeadCode = allSrcsReady && (type == "mov" || type == "movi" || type == "limm" || type == "add" || type == "addi" || type == "sub" || type == "subi" || type == "and" || type == "andi" || type == "or" || type == "ori" || type == "xor" || type == "xori" || type == "slri" || type == "slli" || type == "sexti" || type == "zexti");

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
			if (registerValid[srcIdx]) {
				trace.inst->sourcePredictions[i] = registerValue[i];
				trace.inst->sourcesPredicted[i] = true;
			}
		}
	}
  return updateSuccessful;
}

bool TraceBasedGraph::propagateMov(StaticInstPtr inst) {
	string type = inst->getName();
	assert(type  == "mov");
	
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
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
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
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
	return true;
}

bool TraceBasedGraph::propagateAdd(StaticInstPtr inst) {
	string type = inst->getName();
	assert(type  == "add");
	
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
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
	return true;
}

bool TraceBasedGraph::propagateSub(StaticInstPtr inst) {
	string type = inst->getName();
	assert(type  == "sub");
	
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
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
	return true;
}

bool TraceBasedGraph::propagateAnd(StaticInstPtr inst) {
	string type = inst->getName();
	assert(type  == "and");
	
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
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
	return true;
}

bool TraceBasedGraph::propagateOr(StaticInstPtr inst) {
	string type = inst->getName();
	assert(type  == "or");
	
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
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
	return true;
}

bool TraceBasedGraph::propagateXor(StaticInstPtr inst) {
	string type = inst->getName();
	assert(type  == "xor");
	
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
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
	return true;
}

bool TraceBasedGraph::propagateMovI(StaticInstPtr inst) {
	string type = inst->getName();
	assert(type  == "movi");
	
	if (inst->numSrcRegs() > 0) {
		return false;
	}

	uint64_t forwardVal = inst->getImmediate();
  unsigned destRegId = inst->destRegIdx(0).flatIndex();
  DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
	for (int i = 0; i < inst->numDestRegs(); i++) {
		RegId destReg = inst->destRegIdx(i);
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
	return true;
}

bool TraceBasedGraph::propagateSubI(StaticInstPtr inst) {
	string type = inst->getName();
	assert(type  == "subi");
	
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
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
	return true;
}

bool TraceBasedGraph::propagateAddI(StaticInstPtr inst) {
	string type = inst->getName();
	assert(type  == "addi");
	
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
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
	return true;
}

bool TraceBasedGraph::propagateAndI(StaticInstPtr inst) {
	string type = inst->getName();
	assert(type  == "andi");
	
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
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
	return true;
}

bool TraceBasedGraph::propagateOrI(StaticInstPtr inst) {
	string type = inst->getName();
	assert(type  == "ori");
	
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
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
	return true;
}

bool TraceBasedGraph::propagateXorI(StaticInstPtr inst) {
	string type = inst->getName();
	assert(type  == "xori");
	
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
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
	return true;
}

bool TraceBasedGraph::propagateSllI(StaticInstPtr inst) {
	string type = inst->getName();
	assert(type  == "slli");
	
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
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
	return true;
}

bool TraceBasedGraph::propagateSrlI(StaticInstPtr inst) {
	string type = inst->getName();
	assert(type  == "srli");
	
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
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
	return true;
}

bool TraceBasedGraph::propagateSExtI(StaticInstPtr inst) {
	string type = inst->getName();
	assert(type  == "sexti");
	
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
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
	return true;
}

bool TraceBasedGraph::propagateZExtI(StaticInstPtr inst) {
	string type = inst->getName();
	assert(type  == "zexti");
	
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
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
	return true;
}

/*
bool TraceBasedGraph::propagateWrip(StaticInstPtr inst) {
	bool changedEither = false;

	DPRINTF(ConstProp, "Propagating WRIP at spec[%i][%i][%i]\n", idx, way, uop);

	// Not taken
	unsigned notTakenIndex = speculativeDependencyGraph[idx][way][uop]->consumers[1];
	if (notTakenIndex != 0) {
		if (branches[notTakenIndex].targetValid) {
			changedEither = propagateAcrossControlDependency(notTakenIndex, branches[notTakenIndex].propagatingTo);
		}
	} else {
		DPRINTF(ConstProp, "notTakenIndex was zero, so cannot propagate\n");
	}

	// Taken
	unsigned takenIndex = speculativeDependencyGraph[idx][way][uop]->consumers[0];
	if (takenIndex != 0) {
		if (not branches[takenIndex].targetValid) {
			// attempt to resolve target
			// TODO: this still needs the tid.
			//TheISA::PCState target;
			//if (branchPred->iPred.lookup(microopAddrArray[idx][way][uop].pcAddr, branchPred->getGHR(tid, inst->branch_hist), target, tid)) {
			// branches[takenIndex].nextPc = FullUopAddr(target.instAddr(), 0);
			// branches[takenIndex].propagatingTo = FullUopAddr(target.instAddr(), 0);
			// branches[takenIndex].targetValid = true;
			//}
		}
		if (branches[takenIndex].targetValid) {
			changedEither = propagateAcrossControlDependency(takenIndex, branches[takenIndex].propagatingTo);
		}  // else just return false
	}
	return changedEither;
}

bool TraceBasedGraph::propagateAcrossControlDependency(unsigned branchIndex, FullUopAddr propagatingTo) {
	int idx = (propagatingTo.pcAddr >> 5) & 0x1f;
	uint64_t tag = (propagatingTo.pcAddr >> 10);
	int way = 0;
	int uop = 0;
	bool found = false;
	for (int w = 0; w < 8; w++) {
		if (decoder->uopValidArray[idx][w] && decoder->uopTagArray[idx][w] == tag && !found) { // changed to uop
			assert(decoder->uopCountArray[idx][w] > 0); // changed to uop
			for (int u=0; u < decoder->uopCountArray[idx][w]; u++) { // changed to uop
				if (decoder->uopAddrArray[idx][w][u] == propagatingTo && !found) { // changed to uop
					assert(speculativeDependencyGraph[idx][w][u]);
					way = w;
					uop = u;
					found = true;
				}
			}
		}
	}

	if (found) { 
		DPRINTF(ConstProp, "Found inst to propagate to at spec[%i][%i][%i]\n", idx, way, uop);
	} else {
		DPRINTF(ConstProp, "Failed to find entry for inst %x.%i to propagate to\n", propagatingTo.pcAddr, propagatingTo.uopAddr);
		return false; 
	}

	DPRINTF(ConstProp, "Propagating across branches[%i] to %x.%i at spec[%i][%i][%i]\n", branchIndex, propagatingTo.pcAddr, propagatingTo.uopAddr, idx, way, uop);

	TraceBasedGraph::DependGraphEntry* entry = speculativeDependencyGraph[idx][way][uop];
	StaticInstPtr decodedMacroOp = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
	StaticInstPtr inst = decodedMacroOp;
	if (decodedMacroOp && decodedMacroOp->isMacroop()) { inst = decodedMacroOp->fetchMicroop(entry->thisInst.uopAddr); inst->macroOp = decodedMacroOp; }

	if (inst->isControl()) { 
		DPRINTF(ConstProp, "Destination is a control inst, so not propagating\n");
    		if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false; 
	}

	bool changeMade = false;

	for (int i=0; i<inst->numSrcRegs(); i++) {
		RegId srcReg = inst->srcRegIdx(i);
    		if (srcReg.classValue() != IntRegClass) {
      			continue;
    		}
		DPRINTF(ConstProp, "Checking register map for producer of %i\n", srcReg.flatIndex());
		if (branches[branchIndex].registerValidMap[srcReg.flatIndex()]) {
			// Check whether value is already produced along path
			bool alreadyProduced = false;
			for (int i=1; i<256; i++) {
				if (entry->producers[i] != 0 && connectionsValidSpec[entry->producers[i]] && connections[entry->producers[i]].archRegIdx == srcReg.flatIndex() && (connections[entry->producers[i]].directControlDependency == branchIndex || connections[entry->producers[i]].indirectControlDependency == branchIndex)) {
					alreadyProduced = true;
				}
			}

			if (!alreadyProduced) {
				FullCacheIdx prodIdx = branches[branchIndex].registerProducerMap[srcReg.flatIndex()];
				if (speculativeDependencyGraph[prodIdx.idx][prodIdx.way][prodIdx.uop]) {
					// Get index to add connection at
					unsigned connectionIndex = 0;
					bool connectionFound = false;
					for (int c=1; c<connectionCount; c++) {
						if (!connectionsValidSpec[c] && !connectionFound) {
							connectionIndex = c;
							connectionFound = true;
						}
					}
					if (connectionFound) {
						changeMade = true;
						connections[connectionIndex].producer = prodIdx;
						connections[connectionIndex].consumer = FullCacheIdx(idx, way, uop);
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
							if (speculativeDependencyGraph[prodIdx.idx][prodIdx.way][prodIdx.uop]->consumers[i] == 0 && !addedAsConsumer) {
								speculativeDependencyGraph[prodIdx.idx][prodIdx.way][prodIdx.uop]->consumers[i] = connectionIndex;
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

	// increment propagatingTo
	FullUopAddr nextFullAddr;
	if (decoder->uopAddrArray[idx][way][uop+1].pcAddr == propagatingTo.pcAddr && decoder->uopAddrArray[idx][way][uop+1].uopAddr == propagatingTo.uopAddr + 1) {
		nextFullAddr = FullUopAddr(propagatingTo.pcAddr, propagatingTo.uopAddr + 1);
	} else {
		DPRINTF(ConstProp, "Next addr is %x.%i which doesn't match :(\n", decoder->uopAddrArray[idx][way][uop+1].pcAddr, decoder->uopAddrArray[idx][way][uop+1].uopAddr);
		Addr nextPc = propagatingTo.pcAddr + inst->machInst.instSize;
		nextFullAddr = FullUopAddr(nextPc, 0);
	}


	DPRINTF(ConstProp, "Incrementing propagatingTo from %x.%i to %x.%i\n", propagatingTo.pcAddr, propagatingTo.uopAddr, nextFullAddr.pcAddr, nextFullAddr.uopAddr);
	branches[branchIndex].propagatingTo = nextFullAddr;

  	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
	return changeMade;
}
*/

/*
void TraceBasedGraph::invalidateTraceInst(int idx, int way, int uop) {
	FullUopAddr affectedAddr = decoder->speculativeAddrArray[idx][way][uop];
	int affectedIdx = (affectedAddr.pcAddr >> 5) & 0x1f;
	uint64_t affectedTag = (affectedAddr.pcAddr >> 10);
	for (int w = 0; w < 8; w++) {
		if (decoder->uopValidArray[affectedIdx][w] && decoder->uopTagArray[affectedIdx][w] == affectedTag) {
			for (int u = 0; u < decoder->uopCountArray[affectedIdx][w]; u++) {
				if (decoder->uopAddrArray[affectedIdx][w][u] == affectedAddr && speculativeDependencyGraph[affectedIdx][w][u]) {
					// When clearing whole ways, may call this on an inst that doesn't exist
					// Addr 0.0 would then match with an empty slot in the uop cache
					// The additional if condition prevents that error
					speculativeDependencyGraph[affectedIdx][w][u]->specIdx = FullCacheIdx();
				}
			}
		}
	}
}

void TraceBasedGraph::moveTraceInstOneForward(int idx, int way, int uop) {
	FullCacheIdx prevIdx = getPrevCacheIdx(FullCacheIdx(idx, way, uop));
	if (!prevIdx.valid) { return; } // should only be called if can do

	decoder->speculativeCache[prevIdx.idx][prevIdx.way][prevIdx.uop] = decoder->speculativeCache[idx][way][uop];
	decoder->speculativeAddrArray[prevIdx.idx][prevIdx.way][prevIdx.uop] = decoder->speculativeAddrArray[idx][way][uop];

	FullUopAddr affectedAddr = decoder->speculativeAddrArray[idx][way][uop];
	int affectedIdx = (affectedAddr.pcAddr >> 5) & 0x1f;
	uint64_t affectedTag = (affectedAddr.pcAddr >> 10);
	for (int w = 0; w < 8; w++) {
		if (decoder->uopValidArray[affectedIdx][w] && decoder->uopTagArray[affectedIdx][w] == affectedTag) {
			for (int u = 0; u < decoder->uopCountArray[affectedIdx][w]; u++) {
				if (decoder->uopAddrArray[affectedIdx][w][u] == affectedAddr && speculativeDependencyGraph[affectedIdx][w][u]) {
					speculativeDependencyGraph[affectedIdx][w][u]->specIdx = prevIdx;
				}
			}
		}
	}
}
*/

/*
void TraceBasedGraph::flushMisprediction(unsigned predId) {
	if (predId > 4096 || !predictionSourceValid[predId]) { return; }
	**
 	* Two steps:
 	* 1. Iterate through connections, remove prediction if dependence on this id
 	* 2. If 32-byte region of code including this inst is present in speculative cache, remove
	* Note that optimizations based on any prediction should be hosted in the 32-byte region corresponding to that prediction, even if they are on the other side of a branch
 	* 
	for (int i=1; i<connectionCount; i++) {
		if (connectionsValidSpec[i] && connections[i].hasDependency(predId)) {
			connections[i].value = 0;
			connections[i].valid = false;
			connections[i].removeDependency(predId);
		}
	}

	if (decoder->isHitInSpeculativeCache(predictionSource[predId].pcAddr, predictionSource[predId].uopAddr)) {
		decoder->invalidateSpecTrace(predictionSource[predId].pcAddr, predictionSource[predId].uopAddr);
	}
}

void TraceBasedGraph::registerRemovalOfTraceInst(int idx, int way, int uop) {
	FullUopAddr affectedAddr = decoder->speculativeAddrArray[idx][way][uop];
	int affectedIdx = (affectedAddr.pcAddr >> 5) & 0x1f;
	uint64_t affectedTag = (affectedAddr.pcAddr >> 10);
	for (int w = 0; w < 8; w++) {
		if (decoder->uopValidArray[affectedIdx][w] && decoder->uopTagArray[affectedIdx][w] == affectedTag) {
			for (int u = 0; u < decoder->uopCountArray[affectedIdx][w]; u++) {
				if (decoder->uopAddrArray[affectedIdx][w][u] == affectedAddr && speculativeDependencyGraph[affectedIdx][w][u]) {
					speculativeDependencyGraph[affectedIdx][w][u]->specIdx = FullCacheIdx();
				}
			}
		}
	}
}
*/

FullCacheIdx TraceBasedGraph::getNextCacheIdx(FullCacheIdx start) {
	if (start.uop < decoder->speculativeCountArray[start.idx][start.way] - 1) {
		return FullCacheIdx(start.idx, start.way, start.uop + 1);
	}
	if (decoder->speculativeNextWayArray[start.idx][start.way] == 10) {
		// No next way, we're done
		return FullCacheIdx();
	}
	return FullCacheIdx(start.idx, decoder->speculativeNextWayArray[start.idx][start.way], 0);
}

FullCacheIdx TraceBasedGraph::getPrevCacheIdx(FullCacheIdx start) {
	if (start.uop > 0) {
		return FullCacheIdx(start.idx, start.way, start.uop - 1);
	}
	if (decoder->speculativePrevWayArray[start.idx][start.way] == 10) {
		return FullCacheIdx();
	}
	return FullCacheIdx(start.idx, decoder->speculativePrevWayArray[start.idx][start.way], 5);
}


void TraceBasedGraph::incrementPC(FullCacheIdx specIdx, X86ISA::PCState &nextPC, bool &predict_taken) {
	// Step 1: Use specIdx and getNextCacheIdx to find next inst
	// Step 2: Set nextPC instAddr and microPC to those of next inst
	// Step 3: If inst at specIdx is a branch, use control flow table to set taken
	// Step 4: If didn't have a next index, or if the StaticInstPtr at the next index is null, this is the end of a trace
	FullCacheIdx nextIdx = getNextCacheIdx(specIdx);
//	int idx = (decoder->speculativeAddrArray[nextIdx.idx][nextIdx.way][nextIdx.uop].pcAddr >> 5) & 0x1f;
//	uint64_t tag = (decoder->speculativeAddrArray[nextIdx.idx][nextIdx.way][nextIdx.uop].pcAddr >> 10);
  bool endOfTrace = true;

	DPRINTF(ConstProp, "Transitioning from specCache[%i][%i][%i] to specCache[%i][%i][%i]\n", specIdx.idx, specIdx.way, specIdx.uop, nextIdx.idx, nextIdx.way, nextIdx.uop);
	DPRINTF(ConstProp, "That is, from addr %#x.%#x to addr %#x.%#x\n", decoder->speculativeAddrArray[specIdx.idx][specIdx.way][specIdx.uop].pcAddr, decoder->speculativeAddrArray[specIdx.idx][specIdx.way][specIdx.uop].uopAddr, decoder->speculativeAddrArray[nextIdx.idx][nextIdx.way][nextIdx.uop].pcAddr, decoder->speculativeAddrArray[nextIdx.idx][nextIdx.way][nextIdx.uop].uopAddr);

	// Replacing commented-out section with this
	if (nextIdx.valid) { endOfTrace = false; }

/*
  	if (nextIdx.valid) {
    		for (int way = 0; way < 8; way++) {
      			if (decoder->uopValidArray[idx][way] && decoder->uopTagArray[idx][way] == tag) {
        			for (int uop = 0; uop < decoder->uopCountArray[idx][way]; uop++) {
          				if (decoder->uopAddrArray[idx][way][uop].pcAddr == decoder->speculativeAddrArray[nextIdx.idx][nextIdx.way][nextIdx.uop].pcAddr && decoder->uopAddrArray[idx][way][uop].uopAddr == decoder->speculativeAddrArray[nextIdx.idx][nextIdx.way][nextIdx.uop].uopAddr) {
            					if (speculativeDependencyGraph[idx][way][uop]->specIdx.valid) {
              						DPRINTF(ConstProp, "Found a valid next index at spec[%i][%i][%i]\n", idx, way, uop);
              						endOfTrace = false;
              						break;
            					}
          				}
        			}
      			}
    		}
  	}
*/

	assert(endOfTrace || decoder->speculativeCache[nextIdx.idx][nextIdx.way][nextIdx.uop]);


	if (!endOfTrace && decoder->speculativeCache[nextIdx.idx][nextIdx.way][nextIdx.uop]) {
		nextPC._pc = decoder->speculativeAddrArray[nextIdx.idx][nextIdx.way][nextIdx.uop].pcAddr;
		nextPC._upc = decoder->speculativeAddrArray[nextIdx.idx][nextIdx.way][nextIdx.uop].uopAddr;
   		// DEBUGGING
    		StaticInstPtr macroOp = decoder->speculativeCache[nextIdx.idx][nextIdx.way][nextIdx.uop];
    		if (!macroOp->isMacroop()) { macroOp = macroOp->macroOp; }
    		int size = macroOp->getMacroopSize();
    		// DEBUGGING
    		nextPC._npc = nextPC._pc + size; // decoder->speculativeCache[nextIdx.idx][nextIdx.way][nextIdx.uop]->macroOp->getMacroopSize();
    		if (nextPC._upc < decoder->speculativeCache[nextIdx.idx][nextIdx.way][nextIdx.uop]->macroOp->getNumMicroops() - 1) {
        		nextPC._nupc = (nextPC._upc + 1 ) % decoder->speculativeCache[nextIdx.idx][nextIdx.way][nextIdx.uop]->macroOp->getNumMicroops();
    		} else {
        		nextPC._nupc = decoder->speculativeCache[nextIdx.idx][nextIdx.way][nextIdx.uop]->macroOp->getNumMicroops();
    		}
		predict_taken = isTakenBranch(decoder->speculativeAddrArray[specIdx.idx][specIdx.way][specIdx.uop], specIdx);
	} else {
		nextPC.valid = false;
	}
}

bool TraceBasedGraph::isTakenBranch(FullUopAddr addr, FullCacheIdx specIdx) {
	TheISA::PCState target;
	// assuming tid=0, and using spec cache entry at the StaticInstPtr
	return branchPred->iPred.lookup(addr.pcAddr, branchPred->getGHR(0, decoder->speculativeCache[specIdx.idx][specIdx.way][specIdx.uop]->branch_hist), target, 0);
/*
	for (int i=0; i<4096; i++) {
		if (branchesValid[i] && branches[i].branchAddr == addr && branches[i].confident) {
			return branches[i].taken;
		}
	}
	return false;
*/
}
