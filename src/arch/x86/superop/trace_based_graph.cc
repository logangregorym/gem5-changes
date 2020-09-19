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

TraceBasedGraph::TraceBasedGraph(TraceBasedGraphParams *p) : SimObject(p), usingControlTracking(p->usingControlTracking) {
	traceQueue = queue<unsigned>();
	for (int i=0; i<4096; i++) {
		traceHead[i] = FullCacheIdx();
	}
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

void TraceBasedGraph::invalidateBranch(Addr addr) {
	for (int i=0; i<4096; i++) {
		if (sourceIsBranch[i] && predictionSource[i].pcAddr == addr) {
			predictionSourceValid[i] = false;
		}
	}
}

void TraceBasedGraph::predictValue(Addr addr, unsigned uopAddr, int64_t value)
{
	// Add prediction source for this inst if not here already
	// TODO: new trace

	unsigned source = 0;

	for (int i=1; i<4096 && !source; i++) {
		if (predictionSource[i] == FullUopAddr(addr, uopAddr) && predictedValue[i] == value) {
			predictionSourceValid[i] = true;
			source = i;
		}
	}
	for (int i=1; i<4096 && !source; i++) {
		if (predictionSourceValid[i] == false) {
			predictionSource[i] = FullUopAddr(addr, uopAddr);
			predictedValue[i] = value;
			predictionSourceValid[i] = true;
			source = i;
		}
	}
	if (!source) {
		panic("Ran out of space in predictionSource array\n");
	}

	// Check for existing trace for this pred ID
	for (int i=1; i<4096; i++) {
		if (traceSources[i][0] == source && traceSources[i][1] == 0 && traceSources[i][2] == 0 && traceSources[i][3] == 0) {
			return;
		}
	}
	unsigned newTrace = 0;
	for (int i=1; i<4096 && !newTrace; i++) {
		if (!traceHead[i].valid) {
			newTrace = i;
		}
	}
	if (!newTrace) {
		panic("Ran out of space in list of traces\n");
	}

	// If found just return
	// If not found, push if hot; need uop addr
	int idx = (addr >> 5) & 0x1f;
    	uint64_t tag = (addr >> 10);
    	for (int way = 0; way < 8; way++) {
      		if (decoder->uopValidArray[idx][way] && decoder->uopTagArray[idx][way] == tag) {
        		for (int uop = 0; uop < decoder->uopCountArray[idx][way]; uop++) {
          			if (decoder->uopAddrArray[idx][way][uop].pcAddr == addr && decoder->uopAddrArray[idx][way][uop].uopAddr == uopAddr) {
            				if (decoder->uopHotnessArray[idx][way].read() < 3) { return; }
					// Time to add the trace
					// std::cout << "Recieved prediction for idx " << idx << "." << way << "." << uop << ": " << value << std::endl;
					traceHead[newTrace] = FullCacheIdx(idx, way, uop);
					traceComplete[newTrace] = false;
					traceSources[newTrace][0] = source;
					traceSources[newTrace][1] = 0;
					traceSources[newTrace][2] = 0;
					traceSources[newTrace][3] = 0;
					traceQueue.push(newTrace);
					return;
          			}
        		}
      		}
    	}
	
}

bool TraceBasedGraph::generateNextTraceInst() {
	// std::cout << "Generating..." << std::endl;
	if (!simplifyIdx.valid) { 
		// Pop a new trace from the queue, start at top
		if (traceQueue.empty()) { return false; }
		if (currentTrace) {
			std::cout << "Done optimizing trace" << currentTrace << " with length " << currentTraceLength << std::endl;
		}
		// std::cout << "Popping a trace id to optimize" << std::endl;
		tracesPoppedFromQueue++;
		currentTrace = traceQueue.front();
		currentTraceLength = 0;
		traceQueue.pop();
		simplifyIdx = traceHead[currentTrace];
		// std::cout << "Updated currentTrace to " << currentTrace << " and simplifyIdx to " << simplifyIdx.idx << "." << simplifyIdx.way << "." << simplifyIdx.uop << std::endl;
		// Invalidate leftover predictions
		for (int i=0; i<256; i++) {
			registerValid[i] = false;
		}
	} else { assert(currentTrace); currentTraceLength++; }

	if (!simplifyIdx.valid) { return false; } // no traces in queue to pop

	int i1 = simplifyIdx.idx;
	int i2 = simplifyIdx.way;
	int i3 = simplifyIdx.uop;
	if ((!decoder->uopValidArray[i1][i2]) || (i3 >= decoder->uopCountArray[i1][i2])) {
		tracesWithInvalidHead++;
		// Mark idx as invalid so a new trace id is popped
		simplifyIdx.valid = false;
		// std::cout << "Can't find " << i1 << "." << i2 << "." << i3 << " along trace " << currentTrace << " starting at " << traceHead[currentTrace].idx << "." << traceHead[currentTrace].way << "." << traceHead[currentTrace].uop << std::endl;
		// panic("Trying to simplify inst which does not exist\n");
	}

	StaticInstPtr decodedMacroOp = decoder->decodeInst(decoder->uopCache[i1][i2][i3]);
	StaticInstPtr decodedMicroOp = decodedMacroOp;
	if (decodedMacroOp->isMacroop()) {
		decodedMicroOp = decodedMacroOp->fetchMicroop(decoder->uopAddrArray[i1][i2][i3].uopAddr);
		decodedMicroOp->macroOp = decodedMacroOp;
	}
	
	// Any inst in a trace may be a prediction source
	bool isSource = false;
	for (int i=0; i<4; i++) {
		if (predictionSourceValid[traceSources[currentTrace][i]]) {
			if (predictionSource[traceSources[currentTrace][i]] == decoder->uopAddrArray[simplifyIdx.idx][simplifyIdx.way][simplifyIdx.uop]) {
				// Step 1: Get inst and determine dest register(s)
				// Step 2: Get predicted value from LVP
				// Step 3: Annotate dest register entries with that value
				// Step 4: Add inst to speculative trace
				uint64_t predictedValue = decoder->cpu->getLVP()->getValuePredicted(decoder->uopAddrArray[i1][i2][i3].pcAddr);
				for (int i = 0; i < decodedMicroOp->numDestRegs(); i++) {
					RegId destReg = decodedMicroOp->destRegIdx(i);
					registerValue[destReg.flatIndex()] = predictedValue;
					registerValid[destReg.flatIndex()] = true;
				}
				updateSpecTrace(i1, i2, i3, currentTrace);
				isSource = true;
			}
		}
	}
	
	// On a trace, but not a source
	if (!isSource) {
		string type = decodedMicroOp->getName();
		if (type == "mov") {
			DPRINTF(ConstProp, "Found a MOV at [%i][%i][%i], compacting...\n", i1, i2, i3);
			propagateMov(i1, i2, i3);
		} else if (type == "wrip" || type == "wripi") {
			DPRINTF(ConstProp, "Found a WRIP/WRIPI branch at [%i][%i][%i], compacting...\n", i1, i2, i3);
			// propagateWrip(i1, i2, i3);
		} else if (decodedMicroOp->isControl()) {
			// printf("Control instruction of type %s\n", type);
			// TODO: check for stopping condition or predicted target
		} else if (type == "movi") {
			DPRINTF(ConstProp, "Found a MOVI at [%i][%i][%i], compacting...\n", i1, i2, i3);
			propagateMovI(i1, i2, i3);
		} else if (type == "and") {
			DPRINTF(ConstProp, "Found an AND at [%i][%i][%i], compacting...\n", i1, i2, i3);
			propagateAnd(i1, i2, i3);
		} else if (type == "add") {
			DPRINTF(ConstProp, "Found an ADD at [%i][%i][%i], compacting...\n", i1, i2, i3);
			propagateAdd(i1, i2, i3);
		} else if (type == "sub") {
			DPRINTF(ConstProp, "Found a SUB at [%i][%i][%i], compacting...\n", i1, i2, i3);
			propagateSub(i1, i2, i3);
		} else if (type == "xor") {
			DPRINTF(ConstProp, "Found an XOR at [%i][%i][%i], compacting...\n", i1, i2, i3);
			propagateXor(i1, i2, i3);
		} else if (type == "or") {
			DPRINTF(ConstProp, "Found an OR at [%i][%i][%i], compacting...\n", i1, i2, i3);
			propagateOr(i1, i2, i3);
		} else if (type == "subi") {
			DPRINTF(ConstProp, "Found a SUBI at [%i][%i][%i], compacting...\n", i1, i2, i3);
			propagateSubI(i1, i2, i3);
		} else if (type == "addi") {
			DPRINTF(ConstProp, "Found an ADDI at [%i][%i][%i], compacting...\n", i1, i2, i3);
			propagateAddI(i1, i2, i3);
		} else if (type == "slli") {
			DPRINTF(ConstProp, "Found a SLLI at [%i][%i][%i], compacting...\n", i1, i2, i3);
			propagateSllI(i1, i2, i3);
		} else if (type == "srli") {
			DPRINTF(ConstProp, "Found a SRLI at [%i][%i][%i], compacting...\n", i1, i2, i3);
			propagateSrlI(i1, i2, i3);
		} else if (type == "lea") {
			DPRINTF(ConstProp, "Type is LEA");
			// Requires multiple ALU operations to propagate, not using
		} else if (type == "sexti") {
			// Implementation has multiple ALU operations, but this is not required by the nature of the operation
			DPRINTF(ConstProp, "Found a SEXTI at [%i][%i][%i], compacting...\n", i1, i2, i3);
			propagateSExtI(i1, i2, i3);
		} else if (type == "zexti") {
			// Implementation has multiple ALU operations, but this is not required by the nature of the operation
			DPRINTF(ConstProp, "Found a ZEXTI at [%i][%i][%i], compacting...\n", i1, i2, i3);
			propagateZExtI(i1, i2, i3);
		} else if (type == "mul1s" || type == "mul1u" || type == "mulel" || type == "muleh") {
			DPRINTF(ConstProp, "Type is MUL1S, MUL1U, MULEL, or MULEH\n");
			// TODO: two dest regs with different values? maybe too complex arithmetic?
		} else if (type == "limm") {
			DPRINTF(ConstProp, "Found a LIMM at [%i][%i][%i], compacting...\n", i1, i2, i3);
			propagateLimm(i1, i2, i3);
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

		updateSpecTrace(i1, i2, i3, currentTrace);
	}

	// Update index
	simplifyIdx.uop++;
	if (simplifyIdx.uop >= decoder->uopCountArray[simplifyIdx.idx][simplifyIdx.way]) {
		simplifyIdx.uop = 0;
		if (decoder->uopNextWayArray[simplifyIdx.idx][simplifyIdx.way] != 10) {
			simplifyIdx.way = decoder->uopNextWayArray[simplifyIdx.idx][simplifyIdx.way];
		} else {
			simplifyIdx.valid = false;
		}
	}

	return true;
}

TraceBasedGraph* TraceBasedGraphParams::create() {
	return new TraceBasedGraph(this);
}

bool TraceBasedGraph::propagateMov(int idx, int way, int uop) {
	StaticInstPtr decodedMacroOp = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
	StaticInstPtr decodedMicroOp = decodedMacroOp;
	if (decodedMacroOp->isMacroop()) {
		decodedMicroOp = decodedMacroOp->fetchMicroop(decoder->uopAddrArray[idx][way][uop].uopAddr);
		decodedMicroOp->macroOp = decodedMacroOp;
	}
	string type = decodedMicroOp->getName();
	assert(type  == "mov");
	
	if (decodedMicroOp->numSrcRegs() > 3) {
		DPRINTF(SuperOp, "Skipping mov at [%i][%i][%i]\n", idx, way, uop);
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}

	unsigned destRegId = decodedMicroOp->srcRegIdx(0).flatIndex();
	unsigned srcRegId = decodedMicroOp->srcRegIdx(1).flatIndex();
	uint8_t size = decodedMicroOp->getDataSize();
	if ((!registerValid[destRegId]) || (size < 8 && !registerValid[srcRegId])) {
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
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

	for (int i = 0; i < decodedMicroOp->numDestRegs(); i++) {
		RegId destReg = decodedMicroOp->destRegIdx(i);
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
	return true;
}

bool TraceBasedGraph::propagateLimm(int idx, int way, int uop) {
 	StaticInstPtr decodedMacroOp = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
	StaticInstPtr decodedMicroOp = decodedMacroOp;
	if (decodedMacroOp->isMacroop()) {
		decodedMicroOp = decodedMacroOp->fetchMicroop(decoder->uopAddrArray[idx][way][uop].uopAddr);
		decodedMicroOp->macroOp = decodedMacroOp;
	}
    if (decodedMicroOp->numSrcRegs() > 0) {
		DPRINTF(SuperOp, "Skipping limm at [%i][%i][%i] because it has %i sources\n", idx, way, uop, decodedMicroOp->numSrcRegs());
        if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
        return false;
    }

    uint64_t forwardVal = decodedMicroOp->getImmediate();
    unsigned destRegId = decodedMicroOp->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
	for (int i = 0; i < decodedMicroOp->numDestRegs(); i++) {
		RegId destReg = decodedMicroOp->destRegIdx(i);
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
	return true;
}

bool TraceBasedGraph::propagateAdd(int idx, int way, int uop) {
 	StaticInstPtr decodedMacroOp = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
	StaticInstPtr decodedMicroOp = decodedMacroOp;
	if (decodedMacroOp->isMacroop()) {
		decodedMicroOp = decodedMacroOp->fetchMicroop(decoder->uopAddrArray[idx][way][uop].uopAddr);
		decodedMicroOp->macroOp = decodedMacroOp;
	}
	string type = decodedMicroOp->getName();
	assert(type  == "add");
	
	if (decodedMicroOp->numSrcRegs() > 3) {
		DPRINTF(SuperOp, "Skipping add at [%i][%i][%i] because it has %i sources\n", idx, way, uop, decodedMicroOp->numSrcRegs());
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}

	unsigned destRegId = decodedMicroOp->srcRegIdx(0).flatIndex();
	unsigned srcRegId = decodedMicroOp->srcRegIdx(1).flatIndex();
	if ((!registerValid[srcRegId]) || (!registerValid[destRegId])) {
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}
	uint64_t destVal = registerValue[destRegId];
	uint64_t srcVal = registerValue[srcRegId];

    // value construction is easy for this one
    uint64_t forwardVal = destVal + srcVal;

    destRegId = decodedMicroOp->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
	for (int i = 0; i < decodedMicroOp->numDestRegs(); i++) {
		RegId destReg = decodedMicroOp->destRegIdx(i);
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
	return true;
}

bool TraceBasedGraph::propagateSub(int idx, int way, int uop) {
	StaticInstPtr decodedMacroOp = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
	StaticInstPtr decodedMicroOp = decodedMacroOp;
	if (decodedMacroOp->isMacroop()) {
		decodedMicroOp = decodedMacroOp->fetchMicroop(decoder->uopAddrArray[idx][way][uop].uopAddr);
		decodedMicroOp->macroOp = decodedMacroOp;
	}
	string type = decodedMicroOp->getName();
	assert(type  == "sub");
	
	if (decodedMicroOp->numSrcRegs() > 3) {
		DPRINTF(SuperOp, "Skipping sub at [%i][%i][%i] because it has %i sources\n", idx, way, uop, decodedMicroOp->numSrcRegs());
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}

	unsigned destRegId = decodedMicroOp->srcRegIdx(0).flatIndex();
	unsigned srcRegId = decodedMicroOp->srcRegIdx(1).flatIndex();
	if ((!registerValid[srcRegId]) || (!registerValid[destRegId])) {
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}
	uint64_t destVal = registerValue[destRegId];
	uint64_t srcVal = registerValue[srcRegId];

    // value construction is easy for this one
    uint64_t forwardVal = destVal - srcVal;

    destRegId = decodedMicroOp->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
	for (int i = 0; i < decodedMicroOp->numDestRegs(); i++) {
		RegId destReg = decodedMicroOp->destRegIdx(i);
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
	return true;
}

bool TraceBasedGraph::propagateAnd(int idx, int way, int uop) {
	StaticInstPtr decodedMacroOp = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
	StaticInstPtr decodedMicroOp = decodedMacroOp;
	if (decodedMacroOp->isMacroop()) {
		decodedMicroOp = decodedMacroOp->fetchMicroop(decoder->uopAddrArray[idx][way][uop].uopAddr);
		decodedMicroOp->macroOp = decodedMacroOp;
	}
	string type = decodedMicroOp->getName();
	assert(type  == "and");
	
	if (decodedMicroOp->numSrcRegs() > 3) {
		DPRINTF(SuperOp, "Skipping and at [%i][%i][%i] because it has %i sources\n", idx, way, uop, decodedMicroOp->numSrcRegs());
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}

	unsigned destRegId = decodedMicroOp->srcRegIdx(0).flatIndex();
	unsigned srcRegId = decodedMicroOp->srcRegIdx(1).flatIndex();
	if ((!registerValid[srcRegId]) || (!registerValid[destRegId])) {
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}
	uint64_t destVal = registerValue[destRegId];
	uint64_t srcVal = registerValue[srcRegId];

    // value construction is easy for this one
    uint64_t forwardVal = destVal & srcVal;

    destRegId = decodedMicroOp->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
	for (int i = 0; i < decodedMicroOp->numDestRegs(); i++) {
		RegId destReg = decodedMicroOp->destRegIdx(i);
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
	return true;
}

bool TraceBasedGraph::propagateOr(int idx, int way, int uop) {
	StaticInstPtr decodedMacroOp = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
	StaticInstPtr decodedMicroOp = decodedMacroOp;
	if (decodedMacroOp->isMacroop()) {
		decodedMicroOp = decodedMacroOp->fetchMicroop(decoder->uopAddrArray[idx][way][uop].uopAddr);
		decodedMicroOp->macroOp = decodedMacroOp;
	}
	string type = decodedMicroOp->getName();
	assert(type  == "or");
	
	if (decodedMicroOp->numSrcRegs() > 3) {
		DPRINTF(SuperOp, "Skipping or at [%i][%i][%i] because it has %i sources\n", idx, way, uop, decodedMicroOp->numSrcRegs());
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}

	unsigned destRegId = decodedMicroOp->srcRegIdx(0).flatIndex();
	unsigned srcRegId = decodedMicroOp->srcRegIdx(1).flatIndex();
	if ((!registerValid[srcRegId]) || (!registerValid[destRegId])) {
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}
	uint64_t destVal = registerValue[destRegId];
	uint64_t srcVal = registerValue[srcRegId];

    // value construction is easy for this one
    uint64_t forwardVal = destVal | srcVal;

    destRegId = decodedMicroOp->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
	for (int i = 0; i < decodedMicroOp->numDestRegs(); i++) {
		RegId destReg = decodedMicroOp->destRegIdx(i);
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
	return true;
}

bool TraceBasedGraph::propagateXor(int idx, int way, int uop) {
	StaticInstPtr decodedMacroOp = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
	StaticInstPtr decodedMicroOp = decodedMacroOp;
	if (decodedMacroOp->isMacroop()) {
		decodedMicroOp = decodedMacroOp->fetchMicroop(decoder->uopAddrArray[idx][way][uop].uopAddr);
		decodedMicroOp->macroOp = decodedMacroOp;
	}
	string type = decodedMicroOp->getName();
	assert(type  == "xor");
	
	if (decodedMicroOp->numSrcRegs() > 3) {
		DPRINTF(SuperOp, "Skipping xor at [%i][%i][%i] because it has %i sources\n", idx, way, uop, decodedMicroOp->numSrcRegs());
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}

	unsigned destRegId = decodedMicroOp->srcRegIdx(0).flatIndex();
	unsigned srcRegId = decodedMicroOp->srcRegIdx(1).flatIndex();
	if ((!registerValid[srcRegId]) || (!registerValid[destRegId])) {
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}
	uint64_t destVal = registerValue[destRegId];
	uint64_t srcVal = registerValue[srcRegId];

    // value construction is easy for this one
    uint64_t forwardVal = destVal ^ srcVal;

    destRegId = decodedMicroOp->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
	for (int i = 0; i < decodedMicroOp->numDestRegs(); i++) {
		RegId destReg = decodedMicroOp->destRegIdx(i);
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
	return true;
}

bool TraceBasedGraph::propagateMovI(int idx, int way, int uop) {
	StaticInstPtr decodedMacroOp = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
	StaticInstPtr decodedMicroOp = decodedMacroOp;
	if (decodedMacroOp->isMacroop()) {
		decodedMicroOp = decodedMacroOp->fetchMicroop(decoder->uopAddrArray[idx][way][uop].uopAddr);
		decodedMicroOp->macroOp = decodedMacroOp;
	}
	string type = decodedMicroOp->getName();
	assert(type  == "movi");
	
	if (decodedMicroOp->numSrcRegs() > 0) {
		DPRINTF(SuperOp, "Skipping movi at [%i][%i][%i] because it has %i sources\n", idx, way, uop, decodedMicroOp->numSrcRegs());
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}

	uint64_t forwardVal = decodedMicroOp->getImmediate();
    unsigned destRegId = decodedMicroOp->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
	for (int i = 0; i < decodedMicroOp->numDestRegs(); i++) {
		RegId destReg = decodedMicroOp->destRegIdx(i);
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
	return true;
}

bool TraceBasedGraph::propagateSubI(int idx, int way, int uop) {
	StaticInstPtr decodedMacroOp = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
	StaticInstPtr decodedMicroOp = decodedMacroOp;
	if (decodedMacroOp->isMacroop()) {
		decodedMicroOp = decodedMacroOp->fetchMicroop(decoder->uopAddrArray[idx][way][uop].uopAddr);
		decodedMicroOp->macroOp = decodedMacroOp;
	}
	string type = decodedMicroOp->getName();
	assert(type  == "subi");
	
	if (decodedMicroOp->numSrcRegs() > 1) {
		DPRINTF(SuperOp, "Skipping subi at [%i][%i][%i] because it has %i sources\n", idx, way, uop, decodedMicroOp->numSrcRegs());
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}

	unsigned destRegId = decodedMicroOp->srcRegIdx(0).flatIndex();
	if (!registerValid[destRegId]) {
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}
	uint64_t destVal = registerValue[destRegId];
	uint64_t srcVal = decodedMicroOp->getImmediate();

    // value construction is easy for this one
    uint64_t forwardVal = destVal - srcVal;

    destRegId = decodedMicroOp->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
	for (int i = 0; i < decodedMicroOp->numDestRegs(); i++) {
		RegId destReg = decodedMicroOp->destRegIdx(i);
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
  	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
	return true;
}

bool TraceBasedGraph::propagateAddI(int idx, int way, int uop) {
	StaticInstPtr decodedMacroOp = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
	StaticInstPtr decodedMicroOp = decodedMacroOp;
	if (decodedMacroOp->isMacroop()) {
		decodedMicroOp = decodedMacroOp->fetchMicroop(decoder->uopAddrArray[idx][way][uop].uopAddr);
		decodedMicroOp->macroOp = decodedMacroOp;
	}
	string type = decodedMicroOp->getName();
	assert(type  == "addi");
	
	if (decodedMicroOp->numSrcRegs() > 1) {
		DPRINTF(SuperOp, "Skipping addi at [%i][%i][%i] because it has %i sources\n", idx, way, uop, decodedMicroOp->numSrcRegs());
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}

	unsigned destRegId = decodedMicroOp->srcRegIdx(0).flatIndex();
	if (!registerValid[destRegId]) {
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}
	uint64_t destVal = registerValue[destRegId];
	uint64_t srcVal = decodedMicroOp->getImmediate();

    // value construction is easy for this one
    uint64_t forwardVal = destVal + srcVal;

    destRegId = decodedMicroOp->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
	for (int i = 0; i < decodedMicroOp->numDestRegs(); i++) {
		RegId destReg = decodedMicroOp->destRegIdx(i);
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
  	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
	return true;
}

bool TraceBasedGraph::propagateAndI(int idx, int way, int uop) {
	StaticInstPtr decodedMacroOp = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
	StaticInstPtr decodedMicroOp = decodedMacroOp;
	if (decodedMacroOp->isMacroop()) {
		decodedMicroOp = decodedMacroOp->fetchMicroop(decoder->uopAddrArray[idx][way][uop].uopAddr);
		decodedMicroOp->macroOp = decodedMacroOp;
	}
	string type = decodedMicroOp->getName();
	assert(type  == "andi");
	
	if (decodedMicroOp->numSrcRegs() > 1) {
		DPRINTF(SuperOp, "Skipping andi at [%i][%i][%i] because it has %i sources\n", idx, way, uop, decodedMicroOp->numSrcRegs());
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}

	unsigned destRegId = decodedMicroOp->srcRegIdx(0).flatIndex();
	if (!registerValid[destRegId]) {
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}
	uint64_t destVal = registerValue[destRegId];
	uint64_t srcVal = decodedMicroOp->getImmediate();

    // value construction is easy for this one
    uint64_t forwardVal = destVal & srcVal;

    destRegId = decodedMicroOp->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
	for (int i = 0; i < decodedMicroOp->numDestRegs(); i++) {
		RegId destReg = decodedMicroOp->destRegIdx(i);
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
  	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
	return true;
}

bool TraceBasedGraph::propagateOrI(int idx, int way, int uop) {
	StaticInstPtr decodedMacroOp = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
	StaticInstPtr decodedMicroOp = decodedMacroOp;
	if (decodedMacroOp->isMacroop()) {
		decodedMicroOp = decodedMacroOp->fetchMicroop(decoder->uopAddrArray[idx][way][uop].uopAddr);
		decodedMicroOp->macroOp = decodedMacroOp;
	}
	string type = decodedMicroOp->getName();
	assert(type  == "ori");
	
	if (decodedMicroOp->numSrcRegs() > 1) {
		DPRINTF(SuperOp, "Skipping ori at [%i][%i][%i] because it has %i sources\n", idx, way, uop, decodedMicroOp->numSrcRegs());
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}

	unsigned destRegId = decodedMicroOp->srcRegIdx(0).flatIndex();
	if (!registerValid[destRegId]) {
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}
	uint64_t destVal = registerValue[destRegId];
	uint64_t srcVal = decodedMicroOp->getImmediate();

    // value construction is easy for this one
    uint64_t forwardVal = destVal | srcVal;

    destRegId = decodedMicroOp->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
	for (int i = 0; i < decodedMicroOp->numDestRegs(); i++) {
		RegId destReg = decodedMicroOp->destRegIdx(i);
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
  	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
	return true;
}

bool TraceBasedGraph::propagateXorI(int idx, int way, int uop) {
	StaticInstPtr decodedMacroOp = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
	StaticInstPtr decodedMicroOp = decodedMacroOp;
	if (decodedMacroOp->isMacroop()) {
		decodedMicroOp = decodedMacroOp->fetchMicroop(decoder->uopAddrArray[idx][way][uop].uopAddr);
		decodedMicroOp->macroOp = decodedMacroOp;
	}
	string type = decodedMicroOp->getName();
	assert(type  == "xori");
	
	if (decodedMicroOp->numSrcRegs() > 1) {
		DPRINTF(SuperOp, "Skipping xori at [%i][%i][%i] because it has %i sources\n", idx, way, uop, decodedMicroOp->numSrcRegs());
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}

	unsigned destRegId = decodedMicroOp->srcRegIdx(0).flatIndex();
	if (!registerValid[destRegId]) {
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}
	uint64_t destVal = registerValue[destRegId];
	uint64_t srcVal = decodedMicroOp->getImmediate();

    // value construction is easy for this one
    uint64_t forwardVal = destVal ^ srcVal;

    destRegId = decodedMicroOp->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
	for (int i = 0; i < decodedMicroOp->numDestRegs(); i++) {
		RegId destReg = decodedMicroOp->destRegIdx(i);
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
  	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
	return true;
}

bool TraceBasedGraph::propagateSllI(int idx, int way, int uop) {
	StaticInstPtr decodedMacroOp = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
	StaticInstPtr decodedMicroOp = decodedMacroOp;
	if (decodedMacroOp->isMacroop()) {
		decodedMicroOp = decodedMacroOp->fetchMicroop(decoder->uopAddrArray[idx][way][uop].uopAddr);
		decodedMicroOp->macroOp = decodedMacroOp;
	}
	string type = decodedMicroOp->getName();
	assert(type  == "slli");
	
	if (decodedMicroOp->numSrcRegs() > 1) {
		DPRINTF(SuperOp, "Skipping slli at [%i][%i][%i] because it has %i sources\n", idx, way, uop, decodedMicroOp->numSrcRegs());
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}

	unsigned destRegId = decodedMicroOp->srcRegIdx(0).flatIndex();
	if (!registerValid[destRegId]) {
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}
	uint64_t destVal = registerValue[destRegId];
	uint64_t srcVal = decodedMicroOp->getImmediate();

    // value construction is easy for this one
    uint64_t forwardVal = destVal << srcVal;

    destRegId = decodedMicroOp->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
	for (int i = 0; i < decodedMicroOp->numDestRegs(); i++) {
		RegId destReg = decodedMicroOp->destRegIdx(i);
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
  	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
	return true;
}

bool TraceBasedGraph::propagateSrlI(int idx, int way, int uop) {
	StaticInstPtr decodedMacroOp = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
	StaticInstPtr decodedMicroOp = decodedMacroOp;
	if (decodedMacroOp->isMacroop()) {
		decodedMicroOp = decodedMacroOp->fetchMicroop(decoder->uopAddrArray[idx][way][uop].uopAddr);
		decodedMicroOp->macroOp = decodedMacroOp;
	}
	string type = decodedMicroOp->getName();
	assert(type  == "srli");
	
	if (decodedMicroOp->numSrcRegs() > 1) {
		DPRINTF(SuperOp, "Skipping srli at [%i][%i][%i] because it has %i sources\n", idx, way, uop, decodedMicroOp->numSrcRegs());
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}

	unsigned destRegId = decodedMicroOp->srcRegIdx(0).flatIndex();
	if (!registerValid[destRegId]) {
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}
	uint64_t destVal = registerValue[destRegId];
	uint64_t srcVal = decodedMicroOp->getImmediate();

    // value construction is easy for this one
    uint64_t forwardVal = destVal >> srcVal;

    destRegId = decodedMicroOp->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
	for (int i = 0; i < decodedMicroOp->numDestRegs(); i++) {
		RegId destReg = decodedMicroOp->destRegIdx(i);
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
  	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
	return true;
}

bool TraceBasedGraph::propagateSExtI(int idx, int way, int uop) {
	StaticInstPtr decodedMacroOp = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
	StaticInstPtr decodedMicroOp = decodedMacroOp;
	if (decodedMacroOp->isMacroop()) {
		decodedMicroOp = decodedMacroOp->fetchMicroop(decoder->uopAddrArray[idx][way][uop].uopAddr);
		decodedMicroOp->macroOp = decodedMacroOp;
	}
	string type = decodedMicroOp->getName();
	assert(type  == "sexti");
	
	if (decodedMicroOp->numSrcRegs() > 1) {
		DPRINTF(SuperOp, "Skipping sexti at [%i][%i][%i] because it has %i sources\n", idx, way, uop, decodedMicroOp->numSrcRegs());
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}

	unsigned destRegId = decodedMicroOp->srcRegIdx(0).flatIndex();
	if (!registerValid[destRegId]) {
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}
	uint64_t destVal = registerValue[destRegId];
	uint64_t srcVal = decodedMicroOp->getImmediate();

	// value construction taken from isa file
	int signBit = bits(destVal, srcVal, srcVal);
	uint64_t signMask = mask(srcVal);
	uint64_t forwardVal = signBit ? (destVal | ~signMask) : (destVal & signMask);

    destRegId = decodedMicroOp->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
	for (int i = 0; i < decodedMicroOp->numDestRegs(); i++) {
		RegId destReg = decodedMicroOp->destRegIdx(i);
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
  	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
	return true;
}

bool TraceBasedGraph::propagateZExtI(int idx, int way, int uop) {
	StaticInstPtr decodedMacroOp = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
	StaticInstPtr decodedMicroOp = decodedMacroOp;
	if (decodedMacroOp->isMacroop()) {
		decodedMicroOp = decodedMacroOp->fetchMicroop(decoder->uopAddrArray[idx][way][uop].uopAddr);
		decodedMicroOp->macroOp = decodedMacroOp;
	}
	string type = decodedMicroOp->getName();
	assert(type  == "zexti");
	
	if (decodedMicroOp->numSrcRegs() > 1) {
		DPRINTF(SuperOp, "Skipping zexti at [%i][%i][%i] because it has %i sources\n", idx, way, uop, decodedMicroOp->numSrcRegs());
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}

	unsigned destRegId = decodedMicroOp->srcRegIdx(0).flatIndex();
	if (!registerValid[destRegId]) {
    	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false;
	}
	uint64_t destVal = registerValue[destRegId];
	uint64_t srcVal = decodedMicroOp->getImmediate();

	// value construction taken from isa file
	uint64_t forwardVal = bits(destVal, srcVal, 0);

    destRegId = decodedMicroOp->destRegIdx(0).flatIndex();
    DPRINTF(ConstProp, "Forwarding value %lx through register %i\n", forwardVal, destRegId);
	for (int i = 0; i < decodedMicroOp->numDestRegs(); i++) {
		RegId destReg = decodedMicroOp->destRegIdx(i);
		registerValue[destReg.flatIndex()] = forwardVal;
		registerValid[destReg.flatIndex()] = true;
	}
  	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
	return true;
}

/*
bool TraceBasedGraph::propagateWrip(int idx, int way, int uop) {
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
			//if (branchPred->iPred.lookup(microopAddrArray[idx][way][uop].pcAddr, branchPred->getGHR(tid, decodedMicroOp->branch_hist), target, tid)) {
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
	StaticInstPtr decodedMicroOp = decodedMacroOp;
	if (decodedMacroOp && decodedMacroOp->isMacroop()) { decodedMicroOp = decodedMacroOp->fetchMicroop(entry->thisInst.uopAddr); decodedMicroOp->macroOp = decodedMacroOp; }

	if (decodedMicroOp->isControl()) { 
		DPRINTF(ConstProp, "Destination is a control inst, so not propagating\n");
    		if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
		return false; 
	}

	bool changeMade = false;

	for (int i=0; i<decodedMicroOp->numSrcRegs(); i++) {
		RegId srcReg = decodedMicroOp->srcRegIdx(i);
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
		Addr nextPc = propagatingTo.pcAddr + decodedMicroOp->machInst.instSize;
		nextFullAddr = FullUopAddr(nextPc, 0);
	}


	DPRINTF(ConstProp, "Incrementing propagatingTo from %x.%i to %x.%i\n", propagatingTo.pcAddr, propagatingTo.uopAddr, nextFullAddr.pcAddr, nextFullAddr.uopAddr);
	branches[branchIndex].propagatingTo = nextFullAddr;

  	if (decodedMacroOp->isMacroop()) decodedMacroOp->deleteMicroOps();
	return changeMade;
}
*/

void TraceBasedGraph::updateSpecTrace(int idx, int way, int uop, unsigned traceID) {
	// IMPORTANT NOTE: This is written assuming the trace will be traversed in order, and so the register map will be accurate for the current point in the trace

	// At least for now, control-dependent paths will be handled separately
	StaticInstPtr decodedMacroOp = decoder->decodeInst(decoder->uopCache[idx][way][uop]);
	StaticInstPtr decodedMicroOp = decodedMacroOp;
	if (decodedMacroOp && decodedMacroOp->isMacroop()) {
		decodedMicroOp = decodedMacroOp->fetchMicroop(decoder->uopAddrArray[idx][way][uop].uopAddr);
		decodedMicroOp->macroOp = decodedMacroOp;
	}

	// Rather than checking dests, check sources; if all sources, then all dests in trace
	bool allSrcsReady = true;
	for (int i=0; i<decodedMicroOp->numSrcRegs(); i++) {
		RegId srcReg = decodedMicroOp->srcRegIdx(i);
		allSrcsReady = allSrcsReady && registerValid[srcReg.flatIndex()];
	}

	string type = decodedMicroOp->getName();
	bool isDeadCode = allSrcsReady && (type == "mov" || type == "movi" || type == "limm" || type == "add" || type == "addi" || type == "sub" || type == "subi" || type == "and" || type == "andi" || type == "or" || type == "ori" || type == "xor" || type == "xori" || type == "slri" || type == "slli" || type == "sexti" || type == "zexti");

	// Prevent an inst registering as dead if it is a prediction source
	isDeadCode &= !isPredictionSource(decoder->uopAddrArray[idx][way][uop].pcAddr, decoder->uopAddrArray[idx][way][uop].uopAddr, traceID);

	// Inst will never already be in this trace, single pass
	if (isDeadCode) {
		// Step 2b: Mark DependGraphEntry as dead code to know to skip
		decoder->updateTagInSpeculativeCacheWithoutAdding(decoder->uopAddrArray[idx][way][uop].pcAddr, decoder->uopAddrArray[idx][way][uop].uopAddr, traceID);
		DPRINTF(ConstProp, "Dead code at idx:%#x way:%#x uop:%#x\n", idx, way, uop);
	} else {
		// Step 3a: If not dead and not present, call decodeInst and add
		decoder->addUopToSpeculativeCache(decodedMicroOp, decoder->uopAddrArray[idx][way][uop].pcAddr, decoder->uopAddrArray[idx][way][uop].uopAddr, traceID);

		// Step 3b: Mark all predicted values on the StaticInst
		for (int i=0; i<decodedMicroOp->numSrcRegs(); i++) {
			unsigned srcIdx = decodedMicroOp->srcRegIdx(i).flatIndex();
			if (registerValid[srcIdx]) {
				decodedMicroOp->sourcePredictions[i] = registerValue[i];
				decodedMicroOp->sourcesPredicted[i] = true;
			}
		}
	}
}

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

bool TraceBasedGraph::isPredictionSource(Addr addr, unsigned uop, unsigned traceID) {
	for (int i=1; i<4096; i++) {
		if (predictionSourceValid[i] && predictionSource[i] == FullUopAddr(addr, uop)) { 
			return true;
		}
	}
	return false;
}

void TraceBasedGraph::flushMisprediction(Addr addr, unsigned uop) {
	unsigned predId = 0;
	for (int i=1; i<4096; i++) {
		if (predictionSourceValid[i] && predictionSource[i] == FullUopAddr(addr, uop)) {
			assert(predId = 0);
			predId = i;
		}
	}
	if (predId == 0) { return; }
	
	predictionSourceValid[predId] = false;
	
	// flushMisprediction(predId);
	return;
}

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
