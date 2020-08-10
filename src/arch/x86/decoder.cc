/*
 * Copyright (c) 2011 Google
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Gabe Black
 */

#include "arch/x86/decoder.hh"

#include "arch/x86/regs/misc.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/Decoder.hh"
#include "debug/ConstProp.hh"
#include "debug/SuperOp.hh"

namespace X86ISA
{

Decoder::State
Decoder::doResetState()
{
    origPC = basePC + offset;
    DPRINTF(Decoder, "Setting origPC to %#x\n", origPC);
    instBytes = &decodePages->lookup(origPC);
    chunkIdx = 0;

    emi.rex = 0;
    emi.legacy = 0;
    emi.vex = 0;

    emi.opcode.type = BadOpcode;
    emi.opcode.op = 0;

    immediateCollected = 0;
    emi.immediate = 0;
    emi.displacement = 0;
    emi.dispSize = 0;

    emi.modRM = 0;
    emi.sib = 0;

    
/**
    if (instBytes->si) {
        return FromCacheState;
    } else {
        instBytes->chunks.clear();
        return PrefixState;
    }
**/
	instBytes->chunks.clear();
	return PrefixState;
}

void
Decoder::process()
{
    //This function drives the decoder state machine.

    //Some sanity checks. You shouldn't try to process more bytes if
    //there aren't any, and you shouldn't overwrite an already
    //decoder ExtMachInst.
    assert(!outOfBytes);
    assert(!instDone);

    if (state == ResetState)
        state = doResetState();
    if (state == FromCacheState) {
        state = doFromCacheState();
    } else {
        instBytes->chunks.push_back(fetchChunk);
    }

    //While there's still something to do...
    while (!instDone && !outOfBytes) {
        uint8_t nextByte = getNextByte();
        switch (state) {
          case PrefixState:
            state = doPrefixState(nextByte);
            break;
          case Vex2Of2State:
            state = doVex2Of2State(nextByte);
            break;
          case Vex2Of3State:
            state = doVex2Of3State(nextByte);
            break;
          case Vex3Of3State:
            state = doVex3Of3State(nextByte);
            break;
          case VexOpcodeState:
            state = doVexOpcodeState(nextByte);
            break;
          case OneByteOpcodeState:
            state = doOneByteOpcodeState(nextByte);
            break;
          case TwoByteOpcodeState:
            state = doTwoByteOpcodeState(nextByte);
            break;
          case ThreeByte0F38OpcodeState:
            state = doThreeByte0F38OpcodeState(nextByte);
            break;
          case ThreeByte0F3AOpcodeState:
            state = doThreeByte0F3AOpcodeState(nextByte);
            break;
          case ModRMState:
            state = doModRMState(nextByte);
            break;
          case SIBState:
            state = doSIBState(nextByte);
            break;
          case DisplacementState:
            state = doDisplacementState();
            break;
          case ImmediateState:
            state = doImmediateState();
            break;
          case ErrorState:
            panic("Went to the error state in the decoder.\n");
          default:
            panic("Unrecognized state! %d\n", state);
        }
    }
}

Decoder::State
Decoder::doFromCacheState()
{
    DPRINTF(Decoder, "Looking at cache state.\n");
    if ((fetchChunk & instBytes->masks[chunkIdx]) !=
            instBytes->chunks[chunkIdx]) {
        DPRINTF(Decoder, "Decode cache miss.\n");
        // The chached chunks didn't match what was fetched. Fall back to the
        // predecoder.
        instBytes->chunks[chunkIdx] = fetchChunk;
        instBytes->chunks.resize(chunkIdx + 1);
        instBytes->si = NULL;
        chunkIdx = 0;
        fetchChunk = instBytes->chunks[0];
        offset = origPC % sizeof(MachInst);
        basePC = origPC - offset;
        return PrefixState;
    } else if (chunkIdx == instBytes->chunks.size() - 1) {
        // We matched the cache, so use its value.
        instDone = true;
        offset = instBytes->lastOffset;
        if (offset == sizeof(MachInst))
            outOfBytes = true;
        return ResetState;
    } else {
        // We matched so far, but need to check more chunks.
        chunkIdx++;
        outOfBytes = true;
        return FromCacheState;
    }
}

//Either get a prefix and record it in the ExtMachInst, or send the
//state machine on to get the opcode(s).
Decoder::State
Decoder::doPrefixState(uint8_t nextByte)
{
    uint8_t prefix = Prefixes[nextByte];
    State nextState = PrefixState;
    // REX prefixes are only recognized in 64 bit mode.
    if (prefix == RexPrefix && emi.mode.submode != SixtyFourBitMode)
        prefix = 0;
    if (prefix)
        consumeByte();
    switch(prefix)
    {
        //Operand size override prefixes
      case OperandSizeOverride:
        DPRINTF(Decoder, "Found operand size override prefix.\n");
        emi.legacy.op = true;
        break;
      case AddressSizeOverride:
        DPRINTF(Decoder, "Found address size override prefix.\n");
        emi.legacy.addr = true;
        break;
        //Segment override prefixes
      case CSOverride:
      case DSOverride:
      case ESOverride:
      case FSOverride:
      case GSOverride:
      case SSOverride:
        DPRINTF(Decoder, "Found segment override.\n");
        emi.legacy.seg = prefix;
        break;
      case Lock:
        DPRINTF(Decoder, "Found lock prefix.\n");
        emi.legacy.lock = true;
        break;
      case Rep:
        DPRINTF(Decoder, "Found rep prefix.\n");
        emi.legacy.rep = true;
        break;
      case Repne:
        DPRINTF(Decoder, "Found repne prefix.\n");
        emi.legacy.repne = true;
        break;
      case RexPrefix:
        DPRINTF(Decoder, "Found Rex prefix %#x.\n", nextByte);
        emi.rex = nextByte;
        break;
      case Vex2Prefix:
        DPRINTF(Decoder, "Found VEX two-byte prefix %#x.\n", nextByte);
        emi.vex.present = 1;
        nextState = Vex2Of2State;
        break;
      case Vex3Prefix:
        DPRINTF(Decoder, "Found VEX three-byte prefix %#x.\n", nextByte);
        emi.vex.present = 1;
        nextState = Vex2Of3State;
        break;
      case 0:
        nextState = OneByteOpcodeState;
        break;

      default:
        panic("Unrecognized prefix %#x\n", nextByte);
    }
    return nextState;
}

Decoder::State
Decoder::doVex2Of2State(uint8_t nextByte)
{
    consumeByte();
    Vex2Of2 vex = nextByte;

    emi.rex.r = !vex.r;

    emi.vex.l = vex.l;
    emi.vex.v = ~vex.v;

    switch (vex.p) {
      case 0:
        break;
      case 1:
        emi.legacy.op = 1;
        break;
      case 2:
        emi.legacy.rep = 1;
        break;
      case 3:
        emi.legacy.repne = 1;
        break;
    }

    emi.opcode.type = TwoByteOpcode;

    return VexOpcodeState;
}

Decoder::State
Decoder::doVex2Of3State(uint8_t nextByte)
{
    if (emi.mode.submode != SixtyFourBitMode && bits(nextByte, 7, 6) == 0x3) {
        // This was actually an LDS instruction. Reroute to that path.
        emi.vex.present = 0;
        emi.opcode.type = OneByteOpcode;
        emi.opcode.op = 0xC4;
        return processOpcode(ImmediateTypeOneByte, UsesModRMOneByte,
                             nextByte >= 0xA0 && nextByte <= 0xA3);
    }

    consumeByte();
    Vex2Of3 vex = nextByte;

    emi.rex.r = !vex.r;
    emi.rex.x = !vex.x;
    emi.rex.b = !vex.b;

    switch (vex.m) {
      case 1:
        emi.opcode.type = TwoByteOpcode;
        break;
      case 2:
        emi.opcode.type = ThreeByte0F38Opcode;
        break;
      case 3:
        emi.opcode.type = ThreeByte0F3AOpcode;
        break;
      default:
        // These encodings are reserved. Pretend this was an undefined
        // instruction so the main decoder will behave correctly, and stop
        // trying to interpret bytes.
        emi.opcode.type = TwoByteOpcode;
        emi.opcode.op = 0x0B;
        instDone = true;
        return ResetState;
    }
    return Vex3Of3State;
}

Decoder::State
Decoder::doVex3Of3State(uint8_t nextByte)
{
    if (emi.mode.submode != SixtyFourBitMode && bits(nextByte, 7, 6) == 0x3) {
        // This was actually an LES instruction. Reroute to that path.
        emi.vex.present = 0;
        emi.opcode.type = OneByteOpcode;
        emi.opcode.op = 0xC5;
        return processOpcode(ImmediateTypeOneByte, UsesModRMOneByte,
                             nextByte >= 0xA0 && nextByte <= 0xA3);
    }

    consumeByte();
    Vex3Of3 vex = nextByte;

    emi.rex.w = vex.w;

    emi.vex.l = vex.l;
    emi.vex.v = ~vex.v;

    switch (vex.p) {
      case 0:
        break;
      case 1:
        emi.legacy.op = 1;
        break;
      case 2:
        emi.legacy.rep = 1;
        break;
      case 3:
        emi.legacy.repne = 1;
        break;
    }

    return VexOpcodeState;
}

Decoder::State
Decoder::doVexOpcodeState(uint8_t nextByte)
{
    DPRINTF(Decoder, "Found VEX opcode %#x.\n", nextByte);

    emi.opcode.op = nextByte;
    consumeByte();

    switch (emi.opcode.type) {
      case TwoByteOpcode:
        return processOpcode(ImmediateTypeTwoByte, UsesModRMTwoByte);
      case ThreeByte0F38Opcode:
        return processOpcode(ImmediateTypeThreeByte0F38,
                             UsesModRMThreeByte0F38);
      case ThreeByte0F3AOpcode:
        return processOpcode(ImmediateTypeThreeByte0F3A,
                             UsesModRMThreeByte0F3A);
      default:
        panic("Unrecognized opcode type %d.\n", emi.opcode.type);
    }
}

// Load the first opcode byte. Determine if there are more opcode bytes, and
// if not, what immediate and/or ModRM is needed.
Decoder::State
Decoder::doOneByteOpcodeState(uint8_t nextByte)
{
    State nextState = ErrorState;
    consumeByte();

    if (nextByte == 0x0f) {
        DPRINTF(Decoder, "Found opcode escape byte %#x.\n", nextByte);
        nextState = TwoByteOpcodeState;
    } else {
        DPRINTF(Decoder, "Found one byte opcode %#x.\n", nextByte);
        emi.opcode.type = OneByteOpcode;
        emi.opcode.op = nextByte;

        nextState = processOpcode(ImmediateTypeOneByte, UsesModRMOneByte,
                                  nextByte >= 0xA0 && nextByte <= 0xA3);
    }
    return nextState;
}

// Load the second opcode byte. Determine if there are more opcode bytes, and
// if not, what immediate and/or ModRM is needed.
Decoder::State
Decoder::doTwoByteOpcodeState(uint8_t nextByte)
{
    State nextState = ErrorState;
    consumeByte();
    if (nextByte == 0x38) {
        nextState = ThreeByte0F38OpcodeState;
        DPRINTF(Decoder, "Found opcode escape byte %#x.\n", nextByte);
    } else if (nextByte == 0x3a) {
        nextState = ThreeByte0F3AOpcodeState;
        DPRINTF(Decoder, "Found opcode escape byte %#x.\n", nextByte);
    } else {
        DPRINTF(Decoder, "Found two byte opcode %#x.\n", nextByte);
        emi.opcode.type = TwoByteOpcode;
        emi.opcode.op = nextByte;

        nextState = processOpcode(ImmediateTypeTwoByte, UsesModRMTwoByte);
    }
    return nextState;
}

// Load the third opcode byte and determine what immediate and/or ModRM is
// needed.
Decoder::State
Decoder::doThreeByte0F38OpcodeState(uint8_t nextByte)
{
    consumeByte();

    DPRINTF(Decoder, "Found three byte 0F38 opcode %#x.\n", nextByte);
    emi.opcode.type = ThreeByte0F38Opcode;
    emi.opcode.op = nextByte;

    return processOpcode(ImmediateTypeThreeByte0F38, UsesModRMThreeByte0F38);
}

// Load the third opcode byte and determine what immediate and/or ModRM is
// needed.
Decoder::State
Decoder::doThreeByte0F3AOpcodeState(uint8_t nextByte)
{
    consumeByte();

    DPRINTF(Decoder, "Found three byte 0F3A opcode %#x.\n", nextByte);
    emi.opcode.type = ThreeByte0F3AOpcode;
    emi.opcode.op = nextByte;

    return processOpcode(ImmediateTypeThreeByte0F3A, UsesModRMThreeByte0F3A);
}

// Generic opcode processing which determines the immediate size, and whether
// or not there's a modrm byte.
Decoder::State
Decoder::processOpcode(ByteTable &immTable, ByteTable &modrmTable,
                       bool addrSizedImm)
{
    State nextState = ErrorState;
    const uint8_t opcode = emi.opcode.op;

    //Figure out the effective operand size. This can be overriden to
    //a fixed value at the decoder level.
    int logOpSize;
    if (emi.rex.w)
        logOpSize = 3; // 64 bit operand size
    else if (emi.legacy.op)
        logOpSize = altOp;
    else
        logOpSize = defOp;

    //Set the actual op size
    emi.opSize = 1 << logOpSize;

    //Figure out the effective address size. This can be overriden to
    //a fixed value at the decoder level.
    int logAddrSize;
    if (emi.legacy.addr)
        logAddrSize = altAddr;
    else
        logAddrSize = defAddr;

    //Set the actual address size
    emi.addrSize = 1 << logAddrSize;

    //Figure out the effective stack width. This can be overriden to
    //a fixed value at the decoder level.
    emi.stackSize = 1 << stack;

    //Figure out how big of an immediate we'll retreive based
    //on the opcode.
    int immType = immTable[opcode];
    if (addrSizedImm)
        immediateSize = SizeTypeToSize[logAddrSize - 1][immType];
    else
        immediateSize = SizeTypeToSize[logOpSize - 1][immType];

    //Determine what to expect next
    if (modrmTable[opcode]) {
        nextState = ModRMState;
    } else {
        if (immediateSize) {
            nextState = ImmediateState;
        } else {
            instDone = true;
            nextState = ResetState;
        }
    }
    return nextState;
}

//Get the ModRM byte and determine what displacement, if any, there is.
//Also determine whether or not to get the SIB byte, displacement, or
//immediate next.
Decoder::State
Decoder::doModRMState(uint8_t nextByte)
{
    State nextState = ErrorState;
    ModRM modRM = nextByte;
    DPRINTF(Decoder, "Found modrm byte %#x.\n", nextByte);
    if (defOp == 1) {
        //figure out 16 bit displacement size
        if ((modRM.mod == 0 && modRM.rm == 6) || modRM.mod == 2)
            displacementSize = 2;
        else if (modRM.mod == 1)
            displacementSize = 1;
        else
            displacementSize = 0;
    } else {
        //figure out 32/64 bit displacement size
        if ((modRM.mod == 0 && modRM.rm == 5) || modRM.mod == 2)
            displacementSize = 4;
        else if (modRM.mod == 1)
            displacementSize = 1;
        else
            displacementSize = 0;
    }

    // The "test" instruction in group 3 needs an immediate, even though
    // the other instructions with the same actual opcode don't.
    if (emi.opcode.type == OneByteOpcode && (modRM.reg & 0x6) == 0) {
       if (emi.opcode.op == 0xF6)
           immediateSize = 1;
       else if (emi.opcode.op == 0xF7)
           immediateSize = (emi.opSize == 8) ? 4 : emi.opSize;
    }

    //If there's an SIB, get that next.
    //There is no SIB in 16 bit mode.
    if (modRM.rm == 4 && modRM.mod != 3) {
            // && in 32/64 bit mode)
        nextState = SIBState;
    } else if (displacementSize) {
        nextState = DisplacementState;
    } else if (immediateSize) {
        nextState = ImmediateState;
    } else {
        instDone = true;
        nextState = ResetState;
    }
    //The ModRM byte is consumed no matter what
    consumeByte();
    emi.modRM = modRM;
    return nextState;
}

//Get the SIB byte. We don't do anything with it at this point, other
//than storing it in the ExtMachInst. Determine if we need to get a
//displacement or immediate next.
Decoder::State
Decoder::doSIBState(uint8_t nextByte)
{
    State nextState = ErrorState;
    emi.sib = nextByte;
    DPRINTF(Decoder, "Found SIB byte %#x.\n", nextByte);
    consumeByte();
    if (emi.modRM.mod == 0 && emi.sib.base == 5)
        displacementSize = 4;
    if (displacementSize) {
        nextState = DisplacementState;
    } else if (immediateSize) {
        nextState = ImmediateState;
    } else {
        instDone = true;
        nextState = ResetState;
    }
    return nextState;
}

//Gather up the displacement, or at least as much of it
//as we can get.
Decoder::State
Decoder::doDisplacementState()
{
    State nextState = ErrorState;

    getImmediate(immediateCollected,
            emi.displacement,
            displacementSize);

    DPRINTF(Decoder, "Collecting %d byte displacement, got %d bytes.\n",
            displacementSize, immediateCollected);

    if (displacementSize == immediateCollected) {
        //Reset this for other immediates.
        immediateCollected = 0;
        //Sign extend the displacement
        switch(displacementSize)
        {
          case 1:
            emi.displacement = sext<8>(emi.displacement);
            break;
          case 2:
            emi.displacement = sext<16>(emi.displacement);
            break;
          case 4:
            emi.displacement = sext<32>(emi.displacement);
            break;
          default:
            panic("Undefined displacement size!\n");
        }
        DPRINTF(Decoder, "Collected displacement %#x.\n",
                emi.displacement);
        if (immediateSize) {
            nextState = ImmediateState;
        } else {
            instDone = true;
            nextState = ResetState;
        }

        emi.dispSize = displacementSize;
    }
    else
        nextState = DisplacementState;
    return nextState;
}

//Gather up the immediate, or at least as much of it
//as we can get
Decoder::State
Decoder::doImmediateState()
{
    State nextState = ErrorState;

    getImmediate(immediateCollected,
            emi.immediate,
            immediateSize);

    DPRINTF(Decoder, "Collecting %d byte immediate, got %d bytes.\n",
            immediateSize, immediateCollected);

    if (immediateSize == immediateCollected)
    {
        //Reset this for other immediates.
        immediateCollected = 0;

        //XXX Warning! The following is an observed pattern and might
        //not always be true!

        //Instructions which use 64 bit operands but 32 bit immediates
        //need to have the immediate sign extended to 64 bits.
        //Instructions which use true 64 bit immediates won't be
        //affected, and instructions that use true 32 bit immediates
        //won't notice.
        switch(immediateSize)
        {
          case 4:
            emi.immediate = sext<32>(emi.immediate);
            break;
          case 1:
            emi.immediate = sext<8>(emi.immediate);
        }

        DPRINTF(Decoder, "Collected immediate %#x.\n",
                emi.immediate);
        instDone = true;
        nextState = ResetState;
    }
    else
        nextState = ImmediateState;
    return nextState;
}

Decoder::InstBytes Decoder::dummy;
// Decoder::InstCacheMap Decoder::instCacheMap;

StaticInstPtr
Decoder::decode(ExtMachInst mach_inst, Addr addr)
{
    // auto iter = instMap->find(mach_inst);
    // if (iter != instMap->end())
    //     return iter->second;

    StaticInstPtr si = decodeInst(mach_inst);
    // (*instMap)[mach_inst] = si;
    return si;
}

void
Decoder::updateLRUBits(int idx, int way)
{
    for (int lru = 0; lru < 8; lru++) {
      if (uopLRUArray[idx][lru] > uopLRUArray[idx][way]) {
        uopLRUArray[idx][lru]--;
      }
    }
    uopLRUArray[idx][way] = 7;
    uopCacheLRUUpdates++;
}

void
Decoder::updateLRUBitsSpeculative(int idx, int way)
{
    for (int lru = 0; lru < 8; lru++) {
        if (speculativeLRUArray[idx][lru] > speculativeLRUArray[idx][way]) {
            speculativeLRUArray[idx][lru]--;
        }
    }
    speculativeLRUArray[idx][way] = 7;
}

bool
Decoder::updateUopInUopCache(ExtMachInst emi, Addr addr, int numUops, int size, unsigned cycleAdded, ThreadID tid)
{
    if (numUops > 6) {
      DPRINTF(Decoder, "More than 6 microops: Could not update microop in the microop cache: %#x.\n", addr);
      return false;
    }

    int idx = (addr >> 5) & 0x1f;
    uint64_t tag = (addr >> 10);
    int numFullWays = 0;
    for (int way = 0; way < 8 && numFullWays < 3; way++) {
      if (uopValidArray[idx][way] && uopTagArray[idx][way] == tag) {
        int waySize = uopCountArray[idx][way];
        if ((waySize + numUops) > 6) {
          uopCountArray[idx][way] = 6;
          numFullWays++;
          continue;
        }
        uopCountArray[idx][way] += numUops;
        unsigned uopAddr = 0;
        for (int uop = waySize; uop < (waySize + numUops); uop++) {
          assert(uopAddr == uop - waySize);
          uopAddrArray[idx][way][uop] = addr;
          depTracker->microopAddrArray[idx][way][uop] = ArrayDependencyTracker::FullUopAddr(addr, uopAddr);
          DPRINTF(ConstProp, "Set microopAddrArray[%i][%i][%i] to %x.%i\n", idx, way, uop, addr, uopAddr);
          emi.instSize = size;
          uopCache[idx][way][uop] = emi;
          StaticInstPtr inst = decodeInst(emi);
          if (inst->isMacroop()) {
            StaticInstPtr new_inst = inst->fetchMicroop(uopAddr);
            new_inst->macroOp = inst;
            inst = new_inst;
            depTracker->addToGraph(inst, addr, uopAddr, cycleAdded, tid); // changed to spec
            uopAddr++;
            if (inst->isLastMicroop()) {
              uopAddr = 0;
            }
            inst->macroOp->deleteMicroOps();
          } else {
            depTracker->addToGraph(inst, addr, uopAddr, cycleAdded, tid); // changed to spec
          }
          DPRINTF(Decoder, "Updating microop in the microop cache: %#x tag:%#x idx:%#x way:%#x uop:%d size:%d.\n", addr, tag, idx, way, uop, emi.instSize);
          // DPRINTF(ConstProp, "1Updating microop in the microop cache: %#x tag:%#x idx:%#x way:%#x uop:%d size:%d.\n", addr, tag, idx, way, uop, emi.instSize);
        }
        updateLRUBits(idx, way);
        uopCacheUpdates += numUops;
        return true;
      }
    }

    if (numFullWays == 3) {
      DPRINTF(Decoder, "Could not accomodate 32 byte region: Could not update microop in the microop cache: %#x tag:%#x idx:%#x. Affected PCs:\n", addr, tag, idx);
      for (int way = 0; way < 8; way++) {
        if (uopValidArray[idx][way] && uopTagArray[idx][way] == tag) {
          for (int uop = 0; uop < uopCountArray[idx][way]; uop++) {
            DPRINTF(Decoder, "%#x\n", uopAddrArray[idx][way][uop], true);
            DPRINTF(ConstProp, "Decoder is invalidating way %i, so removing uop[%i][%i][%i]\n", way, idx, way, uop);
            depTracker->removeAtIndex(idx, way, uop); // changed to spec
            depTracker->microopAddrArray[idx][way][uop] = ArrayDependencyTracker::FullUopAddr(0,0);
          }
          uopValidArray[idx][way] = false;
          uopCountArray[idx][way] = 0;
          uopHotnessArray[idx][way] = BigSatCounter(4);
          uopCacheWayInvalidations++;
        }
      }
      return false;
    }

    for (int way = 0; way < 8; way++) {
      if (!uopValidArray[idx][way]) {
        uopCountArray[idx][way] = numUops;
        uopValidArray[idx][way] = true;
        uopTagArray[idx][way] = tag;
        DPRINTF(ConstProp, "Set uopTagArray[%i][%i] to %x\n", idx, way, tag);
        for (int uop = 0; uop < numUops; uop++) {
          uopAddrArray[idx][way][uop] = addr;
          depTracker->microopAddrArray[idx][way][uop] = ArrayDependencyTracker::FullUopAddr(addr,uop);
          DPRINTF(ConstProp, "Set microopAddrArray[%i][%i][%i] to %x.%i\n", idx, way, uop, addr, uop);
          emi.instSize = size;
          uopCache[idx][way][uop] = emi;
          StaticInstPtr inst = decodeInst(emi);
          if (inst->isMacroop()) {
              StaticInstPtr new_inst = inst->fetchMicroop(uop);
              new_inst->macroOp = inst;
              inst = new_inst;
              depTracker->addToGraph(inst, addr, uop, cycleAdded, tid); // changed to spec
              // uopAddr++;
              // if (inst->isLastMicroop()) {
              //     uopAddr = 0;
              // }
              inst->macroOp->deleteMicroOps();
          } else {
              depTracker->addToGraph(inst, addr, uop, cycleAdded, tid); // changed to spec
          }
          DPRINTF(Decoder, "Updating microop in the microop cache: %#x tag:%#x idx:%#x way:%#x uop:%d size:%d.\n", addr, tag, idx, way, uop, emi.instSize);
          // DPRINTF(ConstProp, "2Updating microop in the microop cache: %#x tag:%#x idx:%#x way:%#x uop:%d size:%d.\n", addr, tag, idx, way, uop, emi.instSize);
        }
        updateLRUBits(idx, way);
        uopCacheUpdates += numUops;
        return true;
      }
    }

    for (int way = 0; way < 8; way++) {
      if (uopLRUArray[idx][way] == 0) {
        DPRINTF(Decoder, "Evicting microop in the microop cache: tag:%#x idx:%#x way:%#x.\n Affected PCs:\n", tag, idx, way);
        for (int w = 0; w < 8; w++) {
          if (uopValidArray[idx][w] && uopTagArray[idx][w] == uopTagArray[idx][way]) {
            for (int uop = 0; uop < uopCountArray[idx][w]; uop++) {
              DPRINTF(Decoder, "%#x\n", uopAddrArray[idx][w][uop]);
              uopConflictMisses++;
              depTracker->removeAtIndex(idx, w, uop); // changed to spec
              depTracker->microopAddrArray[idx][w][uop] = ArrayDependencyTracker::FullUopAddr(0,0);
            }
            uopValidArray[idx][w] = false;
            uopCountArray[idx][w] = 0;
            uopHotnessArray[idx][w] = BigSatCounter(4);
            uopCacheWayInvalidations++;
          }
        }
        uopCountArray[idx][way] = numUops;
        uopValidArray[idx][way] = true;
        uopTagArray[idx][way] = tag;
        DPRINTF(ConstProp, "Set uopTagArray[%i][%i] to %x\n", idx, way, tag);
        for (int uop = 0; uop < numUops; uop++) {
          uopAddrArray[idx][way][uop] = addr;
          depTracker->microopAddrArray[idx][way][uop] = ArrayDependencyTracker::FullUopAddr(addr, uop);
          DPRINTF(ConstProp, "Set microopAddrArray[%i][%i][%i] to %x.%i\n", idx, way, uop, addr, uop);
          emi.instSize = size;
          uopCache[idx][way][uop] = emi;
          StaticInstPtr inst = decodeInst(emi);

          if (inst->isMacroop()) {
            StaticInstPtr new_inst = inst->fetchMicroop(uop);
            new_inst->macroOp = inst;
            inst = new_inst;
            depTracker->addToGraph(inst, addr, uop, cycleAdded, tid); // changed to spec
            // uopAddr++;
            // if (inst->isLastMicroop()) {
            //   uopAddr = 0;
            // }
            inst->macroOp->deleteMicroOps();
          } else {
            depTracker->addToGraph(inst, addr, uop, cycleAdded, tid); // changed to spec
          }
          DPRINTF(Decoder, "Updating microop in the microop cache: %#x tag:%#x idx:%#x way:%#x uop:%d size:%d.\n", addr, tag, idx, way, uop, emi.instSize);
          // DPRINTF(ConstProp, "3Updating microop in the microop cache: %#x tag:%#x idx:%#x way:%#x uop:%d size:%d.\n", addr, tag, idx, way, uop, emi.instSize);
        }
        updateLRUBits(idx, way);
        uopCacheUpdates += numUops;
        return true;
      }
    }

    DPRINTF(Decoder, "Eviction failed: Could not update microop in the microop cache :%#x tag:%#x.\n", addr, tag);
    return false;
}

bool
Decoder::updateUopInSpeculativeCache(ExtMachInst emi, Addr addr, int numUops, int size, unsigned cycleAdded)
{
    // Speculative cache is back, but no longer has entries in depend graph
    // This is because spec cache entries come from depend graph
    // They already have a parallel in the uop cache
    // Will also be indexed independently now
    // And the size of this cache may shrink eventually (i.e., fewer ways per set, fewer uops per way)

    if (numUops > 6) {
      DPRINTF(Decoder, "More than 6 microops: Could not update microop in the speculative cache: %#x.\n", addr);
      return false;
    }

    int idx = (addr >> 5) & 0x1f;
    uint64_t tag = (addr >> 10);
    int numFullWays = 0;
    for (int way = 0; way < 8 && numFullWays < 3; way++) {
      if (speculativeValidArray[idx][way] && speculativeTagArray[idx][way] == tag) {
        int waySize = speculativeCountArray[idx][way];
        if ((waySize + numUops) > 6) {
          speculativeCountArray[idx][way] = 6;
          numFullWays++;
          continue;
        }
        speculativeCountArray[idx][way] += numUops;
        unsigned uopAddr = 0;
        for (int uop = waySize; uop < (waySize + numUops); uop++) {
          //speculativeAddrArray[idx][way][uop] = addr;
         // depTracker->speculativeAddrArray[idx][way][uop] = ArrayDependencyTracker::FullUopAddr(addr, uopAddr);
          //DPRINTF(ConstProp, "Set speculativeAddrArray[%i][%i][%i] to %x.%i\n", idx, way, uop, addr, uopAddr);
          emi.instSize = size;
          assert(uopAddr == uop - waySize);
          StaticInstPtr decodedEMI = decodeInst(emi);
          if (decodedEMI->isMacroop()) {
                speculativeCache[idx][way][uop] = decodedEMI->fetchMicroop(uopAddr);
                speculativeCache[idx][way][uop]->macroOp = decodedEMI;
                // depTracker->addToGraph(speculativeCache[idx][way][uop], addr, uopAddr, cycleAdded);
                uopAddr++;
                if (speculativeCache[idx][way][uop]->isLastMicroop()) {
                        uopAddr = 0;
                }
          } else {
                speculativeCache[idx][way][uop] = decodedEMI;
                // depTracker->addToGraph(speculativeCache[idx][way][uop], addr, uopAddr, cycleAdded);
          }
          DPRINTF(Decoder, "Updating microop in the speculative cache: %#x tag:%#x idx:%#x way:%#x uop:%d size:%d.\n", addr, tag, idx, way, uop, emi.instSize);
          // DPRINTF(ConstProp, "1Updating microop in the speculative cache: %#x tag:%#x idx:%#x way:%#x uop:%d size:%d.\n", addr, tag, idx, way, uop, emi.instSize);
        }
        updateLRUBitsSpeculative(idx, way);
        return true;
      }
    }

    if (numFullWays == 3) {
      DPRINTF(Decoder, "Could not accomodate 32 byte region: Could not update microop in the speculative cache: %#x tag:%#x idx:%#x. Affected PCs:\n", addr, tag, idx);
      for (int way = 0; way < 8; way++) {
        if (speculativeValidArray[idx][way] && speculativeTagArray[idx][way] == tag) {
          for (int uop = 0; uop < speculativeCountArray[idx][way]; uop++) {
            // DPRINTF(Decoder, "%#x\n", speculativeAddrArray[idx][way][uop]);

            DPRINTF(ConstProp, "Decoder is invalidating way %i, so removing spec[%i][%i][%i]\n", way, idx, way, uop);
            depTracker->removeAtIndex(idx, way, uop);
            speculativeTraceSources[idx][way][uop] = 0;
          }
          speculativeValidArray[idx][way] = false;
          speculativeCountArray[idx][way] = 0;
          specHotnessArray[idx][way] = BigSatCounter(4);
          speculativePrevWayArray[idx][way] = 10;
          speculativeNextWayArray[idx][way] = 10;
        }
      }
      return false;
    }

    for (int way = 0; way < 8; way++) {
      if (!speculativeValidArray[idx][way]) {
        speculativeCountArray[idx][way] = numUops;
        speculativeValidArray[idx][way] = true;
        speculativeTagArray[idx][way] = tag;
        DPRINTF(ConstProp, "Set speculativeTagArray[%i][%i] to %x\n", idx, way, tag);
        for (int uop = 0; uop < numUops; uop++) {
          // speculativeAddrArray[idx][way][uop] = addr;
          // depTracker->speculativeAddrArray[idx][way][uop] = ArrayDependencyTracker::FullUopAddr(addr, uop);
          // DPRINTF(ConstProp, "Set speculativeAddrArray[%i][%i][%i] to %x.%i\n", idx, way, uop, addr, uop);
          emi.instSize = size;
          StaticInstPtr decodedEMI = decodeInst(emi);
          if (decodedEMI->isMacroop()) {
                speculativeCache[idx][way][uop] = decodedEMI->fetchMicroop(uop);
                speculativeCache[idx][way][uop]->macroOp = decodedEMI;
                // depTracker->addToGraph(speculativeCache[idx][way][uop], addr, uop, cycleAdded);
                // uopAddr++;
                // if (speculativeCache[idx][way][uop]->isLastMicroop()) {
                // 	uopAddr = 0;
                // }
          } else {
                speculativeCache[idx][way][uop] = decodedEMI;
                // depTracker->addToGraph(speculativeCache[idx][way][uop], addr, uop, cycleAdded);
          }
          DPRINTF(Decoder, "Updating microop in the speculative cache: %#x tag:%#x idx:%#x way:%#x uop:%d size:%d.\n", addr, tag, idx, way, uop, emi.instSize);
          // DPRINTF(ConstProp, "2Updating microop in the speculative cache at [%i][%i][%i]: %#x tag:%#x idx:%#x way:%#x uop:%d size:%d.\n", idx, way, uop, addr, tag, idx, way, uop, emi.instSize);
        }
        updateLRUBitsSpeculative(idx, way);
        return true;
      }
    }

    for (int way = 0; way < 8; way++) {
      if (speculativeLRUArray[idx][way] == 0) {
        DPRINTF(Decoder, "Evicting microop in the speculative cache: tag:%#x idx:%#x way:%#x.\n Affected PCs:\n", tag, idx, way);
        for (int w = 0; w < 8; w++) {
          if (speculativeValidArray[idx][w] && speculativeTagArray[idx][w] == speculativeTagArray[idx][way]) {
            for (int uop = 0; uop < speculativeCountArray[idx][w]; uop++) {
              //DPRINTF(Decoder, "%#x\n", speculativeAddrArray[idx][w][uop]);
              depTracker->removeAtIndex(idx, w, uop);
              speculativeTraceSources[idx][w][uop] = 0;
              speculativeCache[idx][way][uop]->macroOp->deleteMicroOps();
            }
            speculativeValidArray[idx][w] = false;
            speculativeCountArray[idx][w] = 0;
            specHotnessArray[idx][w] = BigSatCounter(4);
            speculativePrevWayArray[idx][w] = 10;
            speculativeNextWayArray[idx][w] = 10;
          }
        }
        speculativeCountArray[idx][way] = numUops;
        speculativeValidArray[idx][way] = true;
        speculativeTagArray[idx][way] = tag;
        DPRINTF(ConstProp, "Set speculativeTagArray[%i][%i] to %x\n", idx, way, tag);
        for (int uop = 0; uop < numUops; uop++) {
          // speculativeAddrArray[idx][way][uop] = addr;
          // depTracker->speculativeAddrArray[idx][way][uop] = ArrayDependencyTracker::FullUopAddr(addr, uop);
          // DPRINTF(ConstProp, "Set speculativeAddrArray[%i][%i][%i] to %x.%i\n", idx, way, uop, addr, uop);
          emi.instSize = size;
          StaticInstPtr decodedEMI = decodeInst(emi);
          if (decodedEMI->isMacroop()) {
                speculativeCache[idx][way][uop] = decodedEMI->fetchMicroop(uop);
                speculativeCache[idx][way][uop]->macroOp = decodedEMI;
                // depTracker->addToGraph(speculativeCache[idx][way][uop], addr, uop, cycleAdded);
                // uopAddr++;
                // if (speculativeCache[idx][way][uop]->isLastMicroop()) {
                // 	uopAddr = 0;
                // }
          } else {
                speculativeCache[idx][way][uop] = decodedEMI;
                // depTracker->addToGraph(speculativeCache[idx][way][uop], addr, uop, cycleAdded);
          }
          DPRINTF(Decoder, "Updating microop in the speculative cache: %#x tag:%#x idx:%#x way:%#x uop:%d size:%d.\n", addr, tag, idx, way, uop, emi.instSize);
          // DPRINTF(ConstProp, "3Updating microop in the speculative cache: %#x tag:%#x idx:%#x way:%#x uop:%d size:%d.\n", addr, tag, idx, way, uop, emi.instSize);
        }
        updateLRUBitsSpeculative(idx, way);
        return true;
      }
    }

    DPRINTF(Decoder, "Eviction failed: Could not update microop in the speculative cache :%#x tag:%#x.\n", addr, tag);
    return false;
}

ArrayDependencyTracker::FullCacheIdx
Decoder::addUopToSpeculativeCache(StaticInstPtr inst, Addr addr, unsigned uop) {
	int idx = (addr >> 5) & 0x1f;
	uint64_t tag = (addr >> 10);
	int numFullWays = 0;

	for (int way = 0; way < 8 && numFullWays < 3; way++) {
		if (speculativeValidArray[idx][way] && speculativeTagArray[idx][way] == tag) {
			int waySize = speculativeCountArray[idx][way];
			if (waySize == 6) {
				numFullWays++;
				continue;
			}
			speculativeCountArray[idx][way]++;
			speculativeCache[idx][way][waySize] = inst;
			speculativeAddrArray[idx][way][waySize] = ArrayDependencyTracker::FullUopAddr(addr, uop);
			updateLRUBitsSpeculative(idx, way);
			return ArrayDependencyTracker::FullCacheIdx(idx, way, waySize);
		}
	}

	if (numFullWays == 3) {
		// Replace this section
		DPRINTF(ConstProp, "Already 3 full ways so couldn't add optimized inst\n");
		return ArrayDependencyTracker::FullCacheIdx();
	}

	// If we make it here, then we need to add a way for this tag
	for (int way = 0; way < 8; way++) {
		if (!speculativeValidArray[idx][way]) {
			if (numFullWays > 0) {
				for (int w = 0; w < 8; w++) {
					if (speculativeValidArray[idx][w] && speculativeTagArray[idx][w] == tag && speculativeNextWayArray[idx][w] == 10) {
						speculativeNextWayArray[idx][w] = way;
						speculativePrevWayArray[idx][way] = w;
					}
				}
			}
			speculativeCountArray[idx][way] = 1;
			speculativeValidArray[idx][way] = true;
			speculativeTagArray[idx][way] = tag;
			speculativeCache[idx][way][0] = inst;
			speculativeAddrArray[idx][way][0] = ArrayDependencyTracker::FullUopAddr(addr, uop);
			updateLRUBitsSpeculative(idx, way);
			return ArrayDependencyTracker::FullCacheIdx(idx, way, 0);
		}
	}

	// If we make it here, we need to evict a way to make space
	for (int way = 0; way < 8; way++) {
		if (speculativeLRUArray[idx][way] == 0) {
			for (int w = 0; w < 8; w++) {
				if (speculativeValidArray[idx][w] && speculativeTagArray[idx][w] == speculativeTagArray[idx][way]) {
					for (int u = 0; u < speculativeCountArray[idx][w]; u++) {
						depTracker->invalidateTraceInst(idx, w, u);
					}
					speculativeValidArray[idx][w] = false;
					speculativeCountArray[idx][w] = 0;
					if (speculativePrevWayArray[idx][w] != 10) {
						speculativeNextWayArray[idx][speculativePrevWayArray[idx][w]] = 10;
						speculativePrevWayArray[idx][w] = 10; // default invalid
					}
					if (speculativeNextWayArray[idx][w] != 10) {
						speculativePrevWayArray[idx][speculativeNextWayArray[idx][w]] = 10;
						speculativeNextWayArray[idx][w] = 10; // default invalid
					}
				}
			}
			if (numFullWays > 0) {
				for (int w = 0; w < 8; w++) {
					if (speculativeValidArray[idx][w] && speculativeTagArray[idx][w] == tag && speculativeNextWayArray[idx][w] == 10) {
						speculativeNextWayArray[idx][w] = way;
						speculativePrevWayArray[idx][way] = w;
					}
				}
			}
			speculativeCountArray[idx][way] = 1;
			speculativeValidArray[idx][way] = true;
			speculativeTagArray[idx][way] = tag;
			speculativeCache[idx][way][0] = inst;
			speculativeAddrArray[idx][way][0] = ArrayDependencyTracker::FullUopAddr(addr, uop);
			updateLRUBitsSpeculative(idx, way);	
			return ArrayDependencyTracker::FullCacheIdx(idx, way, 0);
		}
	}
	DPRINTF(ConstProp, "Optimized trace could not be loaded into speculative cache because no tag has LRU bits equal to 0\n");
	return ArrayDependencyTracker::FullCacheIdx();
}

bool
Decoder::addToSpeculativeCacheIffTagExists(StaticInstPtr inst, Addr addr, unsigned uop) {
	int idx = (addr >> 5) & 0x1f;
	uint64_t tag = (addr >> 10);
	int numFullWays = 0;

	for (int way = 0; way < 8 && numFullWays < 3; way++) {
		if (speculativeValidArray[idx][way] && speculativeTagArray[idx][way] == tag) {
			int waySize = speculativeCountArray[idx][way];
			if (waySize == 6) {
				numFullWays++;
				continue;
			}
			speculativeCountArray[idx][way]++;
			speculativeCache[idx][way][waySize] = inst;
			speculativeAddrArray[idx][way][waySize] = ArrayDependencyTracker::FullUopAddr(addr, uop);
			updateLRUBitsSpeculative(idx, way);
			return true;
		}
	}

	// If we make it here, then we need to add a way for this tag
	for (int way = 0; way < 8; way++) {
		if (!speculativeValidArray[idx][way]) {
			speculativeCountArray[idx][way] = 1;
			speculativeValidArray[idx][way] = true;
			speculativeTagArray[idx][way] = tag;
			speculativeCache[idx][way][0] = inst;
			speculativeAddrArray[idx][way][0] = ArrayDependencyTracker::FullUopAddr(addr, uop);
			updateLRUBitsSpeculative(idx, way);
			return true;
		}
	}
	DPRINTF(SuperOp, "Tag not found in cache, so not adding inst\n");
	return false;
}

ArrayDependencyTracker::FullCacheIdx
Decoder::updateTagInSpeculativeCacheWithoutAdding(Addr addr, unsigned uop) {
	int idx = (addr >> 5) & 0x1f;
	uint64_t tag = (addr >> 10);
	int numFullWays = 0;

	for (int way = 0; way < 8 && numFullWays < 3; way++) {
		if (speculativeValidArray[idx][way] && speculativeTagArray[idx][way] == tag) {
			int waySize = speculativeCountArray[idx][way];
			if (waySize == 6) {
				numFullWays++;
				continue;
			}
			// No need to add a new way, there's space in this one
			updateLRUBits(idx, way);
			return ArrayDependencyTracker::FullCacheIdx(idx, way, waySize+1);
		}
	}

	if (numFullWays == 3) {
		// Replace this section
		return ArrayDependencyTracker::FullCacheIdx();
	}

	// If we make it here, then we need to add a way for this tag
	for (int way = 0; way < 8; way++) {
		if (!speculativeValidArray[idx][way]) {
			if (numFullWays > 0) {
				for (int w = 0; w < 8; w++) {
					if (speculativeValidArray[idx][w] && speculativeTagArray[idx][w] == tag && speculativeNextWayArray[idx][w] == 10) {
						speculativeNextWayArray[idx][w] = way;
						speculativePrevWayArray[idx][way] = w;
					}
				}
			}
			speculativeValidArray[idx][way] = true;
			speculativeTagArray[idx][way] = tag;
			updateLRUBitsSpeculative(idx, way);
			return ArrayDependencyTracker::FullCacheIdx(idx, way, 0);
		}
	}

	// If we make it here, we need to evict a way to make space
	for (int way = 0; way < 8; way++) {
		if (speculativeLRUArray[idx][way] == 0) {
			for (int w = 0; w < 8; w++) {
				if (speculativeValidArray[idx][w] && speculativeTagArray[idx][w] == speculativeTagArray[idx][way]) {
					for (int u = 0; u < speculativeCountArray[idx][w]; u++) {
						depTracker->invalidateTraceInst(idx, w, u);
					}
					speculativeValidArray[idx][w] = false;
					speculativeCountArray[idx][w] = 0;
				}
			}
			if (numFullWays > 0) {
				for (int w = 0; w < 8; w++) {
					if (speculativeValidArray[idx][w] && speculativeTagArray[idx][w] == tag && speculativeNextWayArray[idx][w] == 10) {
						speculativeNextWayArray[idx][w] = way;
						speculativePrevWayArray[idx][way] = w;
					}
				}
			}
			speculativeValidArray[idx][way] = true;
			speculativeTagArray[idx][way] = tag;
			updateLRUBitsSpeculative(idx, way);
			return ArrayDependencyTracker::FullCacheIdx(idx, way, 0);
		}
	}
	DPRINTF(SuperOp, "Optimized trace could not be loaded into speculative cache because no tag has LRU bits equal to 0\n");
	return ArrayDependencyTracker::FullCacheIdx();
}

bool
Decoder::isHitInUopCache(Addr addr)
{
    int idx = (addr >> 5) & 0x1f;
    uint64_t tag = (addr >> 10);
    for (int way = 0; way < 8; way++) {
      if (uopValidArray[idx][way] && uopTagArray[idx][way] == tag) {
        for (int uop = 0; uop < uopCountArray[idx][way]; uop++) {
          if (uopAddrArray[idx][way][uop] == addr) {
            DPRINTF(Decoder, "Is hit in the microop cache? true: %#x tag:%#x idx:%#x way:%#x uop:%x size:%d.\n", addr, tag, idx, way, uop, uopCache[idx][way][uop].instSize);
            return true;
          }
        }
      }
    }
    return false;
}

bool
Decoder::isHitInSpeculativeCache(Addr addr, unsigned uop)
{
    int idx = (addr >> 5) & 0x1f;
    uint64_t tag = (addr >> 10);
    for (int way = 0; way < 8; way++) {
        if (speculativeValidArray[idx][way] && speculativeTagArray[idx][way] == tag) {
            for (int u = 0; u < speculativeCountArray[idx][way]; u++) {
                if (speculativeAddrArray[idx][way][u] == ArrayDependencyTracker::FullUopAddr(addr, uop)) {
                    return true;
                }
            }
        }
    }
    return false;
}

StaticInstPtr
Decoder::fetchUopFromUopCache(Addr addr, PCState &nextPC)
{
    int idx = (addr >> 5) & 0x1f;
    uint64_t tag = (addr >> 10);
    for (int way = 0; way < 8; way++) {
      if (uopValidArray[idx][way] && uopTagArray[idx][way] == tag) {
        for (int uop = 0; uop < uopCountArray[idx][way]; uop++) {
          if (uopAddrArray[idx][way][uop] == addr) {
            updateLRUBits(idx, way);
			if (uopHotnessArray[idx][way].read() > 7) {
				hotnessGreaterThanSeven++;
			} else {
				hotnessLessThanSeven++;
			}
	    	uopHotnessArray[idx][way].increment();
            ExtMachInst emi = uopCache[idx][way][uop];
            nextPC.size(emi.instSize);
            nextPC.npc(nextPC.pc() + emi.instSize);
            return decode(emi, addr);
          }
        }
      }
    }
    panic("microop cache hit, but couldn't fetch from the cache.");
    return NULL;
}

StaticInstPtr
Decoder::fetchUopFromSpeculativeCache(Addr addr, PCState &nextPC)
{
	// modeled on uop cache fetch, may not translate well; shouldn't need to use
    int idx = (addr >> 5) & 0x1f;
    uint64_t tag = (addr >> 10);
    for (int way = 0; way < 8; way++) {
        if (speculativeValidArray[idx][way] && speculativeTagArray[idx][way] == tag) {
            for (int uop = 0; uop < speculativeCountArray[idx][way]; uop++) {
                // if (speculativeAddrArray[idx][way][uop] == addr) {
                updateLRUBitsSpeculative(idx, way);
				if (specHotnessArray[idx][way].read() > 7) {
					hotnessGreaterThanSeven++;
				} else {
					hotnessLessThanSeven++;
				}
		    	specHotnessArray[idx][way].increment();
                StaticInstPtr emi = speculativeCache[idx][way][uop];
                nextPC.size(emi->machInst.instSize);
                nextPC.npc(nextPC.pc() + emi->machInst.instSize);
                return emi;
                // return decode(emi, addr);
            // }
            }
        }
    }
    panic("speculative cache hit, but couldn't fetch from the cache.");
    return NULL;
}

bool
Decoder::isTraceAvailable(const X86ISA::PCState thisPC) {
	// Two questions: does a translation exist, and is it the start of its trace
	int idx = (thisPC.instAddr() >> 5) & 0x1f;
	uint64_t tag = (thisPC.instAddr() >> 10);
	for (int way = 0; way < 8; way++) {
		if (uopValidArray[idx][way] && uopTagArray[idx][way] == tag) {
			for (int u = 0; u < uopCountArray[idx][way]; u++) {
				if (depTracker->microopAddrArray[idx][way][u] == ArrayDependencyTracker::FullUopAddr(thisPC.instAddr(), thisPC.microPC())) {
					// Found a match, should have a dependency graph entry
					assert(depTracker->speculativeDependencyGraph[idx][way][u]);
					if (depTracker->speculativeDependencyGraph[idx][way][u]->specIdx.valid || depTracker->speculativeDependencyGraph[idx][way][u]->deadCode) {
						ArrayDependencyTracker::FullCacheIdx specIdx = depTracker->speculativeDependencyGraph[idx][way][u]->specIdx;
						if ((specIdx.uop == 0) && speculativePrevWayArray[specIdx.idx][specIdx.way] == 10) {
							return true;
						}
					}
				}
			}
		}
	}
	// if not found
	return false;
}



bool
Decoder::doSquash(const StaticInstPtr si, X86ISA::PCState pc) {
       
  // if (depTracker->isPredictionSource(pc.instAddr(), pc.microPC()))
  // {
    //std::cout << "Instrucion is source of prediction with PC: " << pc << " " << si->disassemble(pc.pc()) << "\n"; 
    depTracker->flushMisprediction(pc.instAddr(), pc.microPC());
    return true;
 // }

    //   return false;
}



// sends back the microop from the active trace
// In case of a folded branch, nextPC and predict_taken should be set by the function
StaticInstPtr 
Decoder::getSuperoptimizedMicroop (const X86ISA::PCState thisPC, X86ISA::PCState &nextPC, bool &predict_taken)
{
    return StaticInst::nullStaticInstPtr;

}

bool 
Decoder::isTraceAvailable(const X86ISA::PCState thisPC)
{

  return false;

}


bool
Decoder::isDeadCode(Addr addr, unsigned uop, StaticInstPtr& nodeStaticInst) {
	// Should only be called if superoptimizedTraceAvailable returned true
  nodeStaticInst = StaticInst::nullStaticInstPtr;
	int idx = (addr >> 5) & 0x1f;
	uint64_t tag = (addr >> 10);
	for (int way = 0; way < 8; way++) {
		if (uopValidArray[idx][way] && uopTagArray[idx][way] == tag) {
			for (int u = 0; u < uopCountArray[idx][way]; u++) {
				if (depTracker->microopAddrArray[idx][way][u] == ArrayDependencyTracker::FullUopAddr(addr, uop)) {
          nodeStaticInst = depTracker->speculativeDependencyGraph[idx][way][u]->nodeStaticInst;
					return depTracker->speculativeDependencyGraph[idx][way][u]->deadCode;
				}
			}
		}
	}
	// panic("isDeadCode was called on an addr w/o a dependency graph entry");
	return false;
}

bool
Decoder::isSourceOfPrediction(Addr addr, unsigned uop) {
	return depTracker->isPredictionSource(addr, uop);
}

void
Decoder::invalidateSpecTrace(Addr addr, unsigned uop) {
	/*
	 * 1. Find tag match in spec cache
	 * 2. Clear out tag
	 * 3. If has a previous line, clear that one too
	 * 4. If has a next line, clear that one too
	 * Put the check for prev and next in the tag function to call recursively
	 */
	int idx = (addr >> 5) & 0x1f;
	uint64_t tag = (addr >> 10);
	for (int way = 0; way < 8; way++) {
		if (speculativeValidArray[idx][way] && speculativeTagArray[idx][way] == tag) {
			invalidateSpecCacheLine(idx, way);
		}
	}
}

void
Decoder::invalidateSpecCacheLine(int idx, int way) {
	for (int u = 0; u < 6; u++) {
		depTracker->registerRemovalOfTraceInst(idx, way, u);
		speculativeAddrArray[idx][way][u] = ArrayDependencyTracker::FullUopAddr(0,0);
	}
	if (speculativePrevWayArray[idx][way] != 10) {
		invalidateSpecCacheLine(idx, speculativePrevWayArray[idx][way]);
		speculativePrevWayArray[idx][way] = 10;
	}
	if (speculativeNextWayArray[idx][way] != 10) {
		invalidateSpecCacheLine(idx, speculativeNextWayArray[idx][way]);
		speculativeNextWayArray[idx][way] = 10;
	}
}

unsigned
Decoder::getSpecTraceLength(Addr addr) {
	int idx = (addr >> 5) & 0x1f;
	uint64_t tag = (addr >> 10);
	unsigned traceLength = 0;
	for (int way = 0; way < 8; way++) {
		if (speculativeValidArray[idx][way] && speculativeTagArray[idx][way] == tag) {
			traceLength += speculativeCountArray[idx][way];
		}
	}
	return traceLength;
}

unsigned
Decoder::getHotnessOfTrace(Addr addr) {
	int idx = (addr >> 5) & 0x1f;
	uint64_t tag = (addr >> 10);
	unsigned spec = 0;
	unsigned uop = 0;
	for (int way = 0; way < 8; way++) {
		if (speculativeValidArray[idx][way] && speculativeTagArray[idx][way] == tag) {
			spec = specHotnessArray[idx][way].read();
		}
		if (uopValidArray[idx][way] && uopTagArray[idx][way] == tag) {
			uop = uopHotnessArray[idx][way].read();
		}
	}
	if (spec > uop) { return spec; }
	return uop;
}

Decoder::TraceMetaData
Decoder::getTraceMetaData(Addr addr) {
	int idx = (addr >> 5) & 0x1f;
	uint64_t tag = (addr >> 10);
	for (int way = 0; way < 8; way++) {
		if (speculativeValidArray[idx][way] && speculativeTagArray[idx][way] == tag) {
			// will return in this if, so can use way without worrying about the loop
			while (speculativePrevWayArray[idx][way] != 10) {
				way = speculativePrevWayArray[idx][way]; // traverse to start of trace
			}
			return TraceMetaData(specHotnessArray[idx][way].read(), minConfidence(idx, way), maxLatency(idx, way));
		}
	}
	panic("Requested metadata for a trace which does not exist\n");
}

unsigned
Decoder::minConfidence(unsigned idx, unsigned way) {
	unsigned minConf = 50;
	for (int source = 0; source < 6; source++) {
		if (speculativeTraceSources[idx][way][source] != 0) {
			assert(depTracker->predictionSourceValid[speculativeTraceSources[idx][way][source]]);
			unsigned sourceConf = depTracker->predictionConfidence[speculativeTraceSources[idx][way][source]];
			if (sourceConf < minConf) { minConf = sourceConf; }
		}
	}
	assert(minConf != 50);
	return minConf;
}

unsigned
Decoder::maxLatency(unsigned idx, unsigned way) {
	unsigned maxLat = 0;
	for (int source = 0; source < 6; source++) {
		if (speculativeTraceSources[idx][way][source] != 0) {
			assert(depTracker->predictionSourceValid[speculativeTraceSources[idx][way][source]]);
			unsigned sourceLat = depTracker->predictionResolutionLatency[speculativeTraceSources[idx][way][source]];
			if (sourceLat > maxLat) { maxLat = sourceLat; }
		}
	}
	return maxLat;
}

void
Decoder::addSourceToCacheLine(unsigned predID, int idx, uint64_t tag) {
	for (int way = 0; way < 8; way++) {
		if (speculativeValidArray[idx][way] && speculativeTagArray[idx][way] == tag) {
			bool added = false;
			for (int source = 0; source < 6 && !added; source++) {
				if (speculativeTraceSources[idx][way][source] == 0) {
					added = true;
					speculativeTraceSources[idx][way][source] = predID;
				} else if (speculativeTraceSources[idx][way][source] == predID) {
					added = true;
				}
			}
		}
	}
}

StaticInstPtr Decoder::getSuperOptimizedMicroop(const X86ISA::PCState thisPc, X86ISA::PCState &nextPc, bool &predict_taken) {
	int idx = (thisPc.instAddr() >> 5) & 0x1f;
	uint64_t tag = (thisPc.instAddr() >> 10);
	for (int way = 0; way < 8; way++) {
		if (uopValidArray[idx][way] && uopTagArray[idx][way] == tag) {
			for (int uop = 0; uop < uopCountArray[idx][way]; uop++) {
				if (depTracker->microopAddrArray[idx][way][uop].pcAddr == thisPc.instAddr() && depTracker->microopAddrArray[idx][way][uop].uopAddr == thisPc.microPC()) {
					if (depTracker->speculativeDependencyGraph[idx][way][uop]->specIdx.valid) {
						ArrayDependencyTracker::FullCacheIdx specIdx = depTracker->speculativeDependencyGraph[idx][way][uop]->specIdx;

						// should be start of trace
						assert(specIdx.uop == 0);
						assert(speculativePrevWayArray[specIdx.idx][specIdx.way] == 10);

						// update nextPc and predict_taken
						depTracker->incrementPC(specIdx, nextPc, predict_taken);
						
						// return optimized inst
						return speculativeCache[specIdx.idx][specIdx.way][specIdx.uop];
					}
				}
			}
		}
	}

	return StaticInst::nullStaticInstPtr;
}

StaticInstPtr
Decoder::decode(PCState &nextPC, unsigned cycleAdded, ThreadID tid)
{
    // if using speculative superoptimizer and optimization available, more complex decode stage
    // if (isSpeculativeCachePresent && isSpeculativeCacheActive && isHitInSpeculativeCache(nextPC.instAddr())) {
    //	return fetchUopFromSpeculativeCache(nextPC.instAddr(), nextPC);
    // } else

    if (isUopCachePresent && isUopCacheActive && isHitInUopCache(nextPC.instAddr())) {
      DPRINTF(Decoder, "Fetching microop from the microop cache: %s.\n", nextPC);
      return fetchUopFromUopCache(nextPC.instAddr(), nextPC);
    }

    if (!instDone)
        return NULL;
    instDone = false;
    emi.instSize = basePC + offset - origPC;
    updateNPC(nextPC);

/**
    StaticInstPtr &si = instBytes->si;
    if (si) {
        int numFusedMicroops = 1;
        if (si->isMacroop() && isMicroFusionPresent) {
            for (int i = 1; i < si->getNumMicroops(); i++) {
                StaticInstPtr cur = si->fetchMicroop(i);
                StaticInstPtr prev = si->fetchMicroop(i-1);
                if ((cur->isInteger() || cur->isNop() || cur->isControl() || cur->isMicroBranch()) && prev->isLoad() && !prev->isRipRel()) {
                    i++;
                }
                numFusedMicroops++;
            }
            DPRINTF(Decoder, "Inserting fused microops: (%d/%d)\n", numFusedMicroops, si->getNumMicroops());
        } else if (si->isMacroop()) {
            numFusedMicroops = si->getNumMicroops();
        }
        if (isUopCachePresent) {
            updateUopInUopCache(si->machInst, nextPC.instAddr(), numFusedMicroops, emi.instSize, cycleAdded);
        }
        if (isSuperOptimizationPresent) {
            //updateUopInSpeculativeCache(si->machInst, nextPC.instAddr(), numFusedMicroops, emi.instSize, cycleAdded);
        } // This populates the uop cache from fetch, don't want for speculative
        return si;
    }
**/

    // We didn't match in the AddrMap, but we still populated an entry. Fix
    // up its byte masks.
    const int chunkSize = sizeof(MachInst);

    instBytes->lastOffset = offset;

    Addr firstBasePC = basePC - (instBytes->chunks.size() - 1) * chunkSize;
    Addr firstOffset = origPC - firstBasePC;
    Addr totalSize = instBytes->lastOffset - firstOffset +
        (instBytes->chunks.size() - 1) * chunkSize;
    int start = firstOffset;
    instBytes->masks.clear();

    while (totalSize) {
        int end = start + totalSize;
        end = (chunkSize < end) ? chunkSize : end;
        int size = end - start;
        int idx = instBytes->masks.size();

        MachInst maskVal = mask(size * 8) << (start * 8);
        assert(maskVal);

        instBytes->masks.push_back(maskVal);
        instBytes->chunks[idx] &= instBytes->masks[idx];
        totalSize -= size;
        start = 0;
    }

    StaticInstPtr si = decode(emi, nextPC.instAddr());

    if (si->isMacroop()) {
        switch (si->getNumMicroops()) {
          case 1:   macroTo1MicroEncoding++;
                    break;
          case 2:   macroTo2MicroEncoding++;
                    break;
          case 3:   macroTo3MicroEncoding++;
                    break;
          case 4:   macroTo4MicroEncoding++;
                    break;
          default:  macroToROMMicroEncoding++;
                    break;
        }
    }

    if (isUopCachePresent) {
        int numFusedMicroops = 1;
        if (si->isMacroop() && isMicroFusionPresent) {
            for (int i = 1; i < si->getNumMicroops(); i++) {
                StaticInstPtr cur = si->fetchMicroop(i);
                StaticInstPtr prev = si->fetchMicroop(i-1);
                if ((cur->isInteger() || cur->isNop() || cur->isControl() || cur->isMicroBranch()) && prev->isLoad() && !prev->isRipRel()) {
                    i++;
                }
                numFusedMicroops++;
            }
            DPRINTF(Decoder, "Inserting fused microops: (%d/%d)\n", numFusedMicroops, si->getNumMicroops());
        } else if (si->isMacroop()) {
            numFusedMicroops = si->getNumMicroops();
        }
        updateUopInUopCache(si->machInst, nextPC.instAddr(), numFusedMicroops, emi.instSize, cycleAdded, tid);
        if (isSuperOptimizationPresent) {
            // updateUopInSpeculativeCache(si->machInst, nextPC.instAddr(), numFusedMicroops, emi.instSize, cycleAdded);
        }
    }

    return si;
}

void
Decoder::dumpMicroopCache()
{
    // DPRINTF(SuperOp, "Contents of Micro-op Cache: \n");
    for (int i=0; i<32; i++) {
        for (int j=0; j<8; j++) {
            // DPRINTF(SuperOp, "Tag:%x, Valid:%i, Count:%i, LRU:%i\n", uopTagArray[i][j], uopValidArray[i][j], uopCountArray[i][j], uopLRUArray[i][j]);
            for (int k=0; k<6; k++) {
                // DPRINTF(SuperOp, "Addr:%x has inst: %s\n", uopAddrArray[i][j][k], decodeInst(uopCache[i][j][k])->disassemble(uopAddrArray[i][j][k]));
            }
        }
    }
}

void
Decoder::regStats()
{
    uopCacheUpdates
        .name("system.switch_cpus.decode.uopCacheOpUpdates")
        .desc("Number of micro-ops written to the micro-op cache");

    uopCacheLRUUpdates
        .name("system.switch_cpus.decode.uopCacheLRUUpdates")
        .desc("Number of times the micro-op cache LRU bits were updated");

    uopCacheWayInvalidations
        .name("system.switch_cpus.decode.uopCacheWayInvalidations")
        .desc("Number of times the micro-op cache ways were invalidated");

    macroTo1MicroEncoding
        .name("system.switch_cpus.decode.oneMicroOp")
        .desc("Number of times we decoded a macro-op into one micro-op");

    macroTo2MicroEncoding
        .name("system.switch_cpus.decode.twoMicroOps")
        .desc("Number of times we decoded a macro-op into two micro-ops");

    macroTo3MicroEncoding
        .name("system.switch_cpus.decode.threeMicroOps")
        .desc("Number of times we decoded a macro-op into three micro-ops");

    macroTo4MicroEncoding
        .name("system.switch_cpus.decode.fourMicroOps")
        .desc("Number of tiems we decoded a macro-op into four micro-ops");

    macroToROMMicroEncoding
        .name("system.switch_cpus.decode.MSROMAccess")
        .desc("Number of times we decoded a macro-op usingMSROM");
	hotnessLessThanSeven
		.name("system.switch_cpus.decode.hotnessLessThenSeven")
		.desc("Number of cache lines accessed with a low hotness score");
	hotnessGreaterThanSeven
		.name("system.switch_cpus.decode.hotnessGreaterThanSeven")
		.desc("Number of cache lines accessed with a high hotness score");
}

void
Decoder::setCPU(BaseCPU * newCPU, ThreadID tid)
{
    assert(!cpu);
    assert(newCPU);
    cpu = newCPU;
}
}
