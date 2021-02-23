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
#include "cpu/o3/cpu.hh"
#include "debug/Decoder.hh"
#include "debug/ConstProp.hh"
#include "debug/SuperOp.hh"

#include <iostream>
#include <algorithm>


namespace X86ISA
{

Decoder::Decoder(ISA* isa, DerivO3CPUParams* params) : basePC(0), origPC(0), offset(0), cpu(NULL), outOfBytes(true), instDone(false), state(ResetState)
{
    emi.reset();
    mode = LongMode;
    submode = SixtyFourBitMode;
    emi.mode.mode = mode;
    emi.mode.submode = submode;
    altOp = 0;
    defOp = 0;
    altAddr = 0;
    defAddr = 0;
    stack = 0;
    instBytes = &dummy;
    decodePages = NULL;
    // instMap = NULL;
    isUopCachePresent = false;
    isMicroFusionPresent = false;
    isSpeculativeCachePresent = false;
    speculativeCacheActive = false;
    currentActiveTraceID = 0;
    redirectDueToLVPSquashing = false;
    for (int idx=0; idx<32; idx++) {
        for (int way=0; way<8; way++) {
            uopPrevWayArray[idx][way] = 10;
            uopNextWayArray[idx][way] = 10;
            uopValidArray[idx][way] = false;
            uopFullArray[idx][way] = false;
            uopCountArray[idx][way] = 0;
            uopLRUArray[idx][way] = way;
            uopHotnessArray[idx][way] = BigSatCounter(4);
            for (int uop = 0; uop < 6; uop++) {
                uopAddrArray[idx][way][uop] = FullUopAddr();
            }
        }
    }

    
    if (params != nullptr){

        if (params->traceConstructor != nullptr) {
            traceConstructor = params->traceConstructor;
            traceConstructor->decoder = this;
            traceConstructor->branchPred = params->branchPred;
            traceConstructor->loadPred = params->loadPred;
        } else {
            // CPUO3 without depTracker?!
            assert(0);
        }
    }

    if (params != nullptr)
    {
        // allocate spec cache
        SPEC_CACHE_NUM_WAYS = traceConstructor->specCacheNumWays;
        SPEC_CACHE_NUM_SETS = traceConstructor->specCacheNumSets;
        SPEC_CACHE_WAY_MAGIC_NUM = 2 + SPEC_CACHE_NUM_WAYS; // this is used to find invalid ways (it was 10 before)

        assert((SPEC_CACHE_NUM_WAYS & (SPEC_CACHE_NUM_WAYS - 1)) == 0);
        assert((SPEC_CACHE_NUM_SETS & (SPEC_CACHE_NUM_SETS - 1)) == 0);

        speculativeCache = new StaticInstPtr ** [SPEC_CACHE_NUM_SETS];
        for (size_t set = 0; set < SPEC_CACHE_NUM_SETS; set++)
        {
            speculativeCache[set] = new StaticInstPtr * [SPEC_CACHE_NUM_WAYS];
            for (size_t way = 0; way < SPEC_CACHE_NUM_WAYS; way++)
            {
                speculativeCache[set][way] = new StaticInstPtr [6];
            }
        }

        speculativeAddrArray = new FullUopAddr ** [SPEC_CACHE_NUM_SETS];
        for (size_t set = 0; set < SPEC_CACHE_NUM_SETS; set++)
        {
            speculativeAddrArray[set] = new FullUopAddr * [SPEC_CACHE_NUM_WAYS];
            for (size_t way = 0; way < SPEC_CACHE_NUM_WAYS; way++)
            {
                speculativeAddrArray[set][way] = new FullUopAddr [6];
            }
        }

        speculativeTagArray = new uint64_t * [SPEC_CACHE_NUM_SETS];
        for (size_t set = 0; set < SPEC_CACHE_NUM_SETS; set++)
        {
            speculativeTagArray[set] = new uint64_t [SPEC_CACHE_NUM_WAYS];
        
        }

        speculativeEvictionStat = new uint64_t * [SPEC_CACHE_NUM_SETS];
        for (size_t set = 0; set < SPEC_CACHE_NUM_SETS; set++)
        {
            speculativeEvictionStat[set] = new uint64_t [SPEC_CACHE_NUM_WAYS];
        
        }


        speculativePrevWayArray = new int * [SPEC_CACHE_NUM_SETS];
        for (size_t set = 0; set < SPEC_CACHE_NUM_SETS; set++)
        {
            speculativePrevWayArray[set] = new int [SPEC_CACHE_NUM_WAYS];
        
        }
    
        speculativeNextWayArray = new int * [SPEC_CACHE_NUM_SETS];
        for (size_t set = 0; set < SPEC_CACHE_NUM_SETS; set++)
        {
            speculativeNextWayArray[set] = new int [SPEC_CACHE_NUM_WAYS];
        
        }

        speculativeValidArray = new bool * [SPEC_CACHE_NUM_SETS];
        for (size_t set = 0; set < SPEC_CACHE_NUM_SETS; set++)
        {
            speculativeValidArray[set] = new bool [SPEC_CACHE_NUM_WAYS];
        
        }

        speculativeCountArray = new int * [SPEC_CACHE_NUM_SETS];
        for (size_t set = 0; set < SPEC_CACHE_NUM_SETS; set++)
        {
            speculativeCountArray[set] = new int [SPEC_CACHE_NUM_WAYS];
        
        }

        speculativeLRUArray = new int * [SPEC_CACHE_NUM_SETS];
        for (size_t set = 0; set < SPEC_CACHE_NUM_SETS; set++)
        {
            speculativeLRUArray[set] = new int [SPEC_CACHE_NUM_WAYS];
        
        }

        speculativeTraceIDArray = new uint64_t * [SPEC_CACHE_NUM_SETS];
        for (size_t set = 0; set < SPEC_CACHE_NUM_SETS; set++)
        {
            speculativeTraceIDArray[set] = new uint64_t [SPEC_CACHE_NUM_WAYS];
        
        }

        specHotnessArray = new BigSatCounter * [SPEC_CACHE_NUM_SETS];
        for (size_t set = 0; set < SPEC_CACHE_NUM_SETS; set++)
        {
            specHotnessArray[set] = new BigSatCounter [SPEC_CACHE_NUM_WAYS];
        
        }

        SPEC_INDEX_MASK = (SPEC_CACHE_NUM_SETS-1);
        SPEC_NUM_INDEX_BITS = std::log2(SPEC_CACHE_NUM_SETS);


        for (int idx=0; idx< SPEC_CACHE_NUM_SETS; idx++) {
            for (int way=0; way< SPEC_CACHE_NUM_WAYS; way++) {
                
                // Parallel cache for optimized micro-ops
                speculativeValidArray[idx][way] = false;
                speculativeCountArray[idx][way] = 0;
                speculativeLRUArray[idx][way] = way;
                speculativeTagArray[idx][way] = 0;
                speculativePrevWayArray[idx][way] = SPEC_CACHE_WAY_MAGIC_NUM;
                speculativeNextWayArray[idx][way] = SPEC_CACHE_WAY_MAGIC_NUM;
                specHotnessArray[idx][way] = BigSatCounter(4);
                speculativeTraceIDArray[idx][way] = 0;
                speculativeEvictionStat[idx][way] = 0;
                for (int uop = 0; uop < 6; uop++) {
                    speculativeAddrArray[idx][way][uop] = FullUopAddr();
                    speculativeCache[idx][way][uop] = NULL;
                }
            }
        }
    }
}

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
    for (int lru = 0; lru < SPEC_CACHE_NUM_WAYS; lru++) {
        if (speculativeLRUArray[idx][lru] > speculativeLRUArray[idx][way]) {
            speculativeLRUArray[idx][lru]--;
        }
    }
    speculativeLRUArray[idx][way] = SPEC_CACHE_NUM_WAYS-1;
}

bool
Decoder::updateUopInUopCache(ExtMachInst emi, Addr addr, int numUops, int size, unsigned cycleAdded, ThreadID tid)
{
    DPRINTF(Decoder, "Trying to update microop in the microop cache for %#x with %i uops.\n", addr, numUops);
    if (numUops > 6) {
        DPRINTF(Decoder, "More than 6 microops: Could not update microop in the microop cache: %#x.\n", addr);
        return false;
    }

    int idx = (addr >> 5) & 0x1f;
    uint64_t tag = (addr >> 10);
    int numFullWays = 0;
    int lastWay = -1;

    int baseAddr = 0;
    int baseWay = 0;
    int waysVisited = 0;
    for (int way = 0; way < 8; way++) {
        if ((uopValidArray[idx][way] && uopTagArray[idx][way] == tag) &&
             (!baseAddr || uopAddrArray[idx][way][0].pcAddr <= baseAddr)) {
            baseAddr = uopAddrArray[idx][way][0].pcAddr;
            baseWay = way;
        }
    }
    /* Trace traversal is circular, rather than linear. */
    for (int way = baseWay; waysVisited < 8 && numFullWays < 3; way = (way + 1) % 8) {
        /*if (uopFullArray[idx][way]) {
            DPRINTF(Decoder, "uop[[%i][%i] is full\n", idx, way);
            numFullWays++;
        } else*/ if (uopValidArray[idx][way] && uopTagArray[idx][way] == tag) {
            /* Check if this way can accommodate the uops that correspond
                 to this instruction. */
            int waySize = uopCountArray[idx][way];
            DPRINTF(Decoder, "uop[[%i][%i] has %i uops\n", idx, way, waySize);
            if ((waySize + numUops) > 6) {
                lastWay = way;
                //uopFullArray[idx][way] = true;
                numFullWays++;
                waysVisited++;
                continue;
            }

            uopCountArray[idx][way] += numUops;
            unsigned uopAddr = 0;
            for (int uop = waySize; uop < (waySize + numUops); uop++) {
                uopAddrArray[idx][way][uop] = FullUopAddr(addr, uopAddr + uop - waySize);
                DPRINTF(ConstProp, "Set microopAddrArray[%i][%i][%i] to %x.%i\n", idx, way, uop, addr, uopAddr + uop - waySize);
                emi.instSize = size;
                uopCache[idx][way][uop] = emi;
                DPRINTF(Decoder, "Updating microop in the microop cache: %#x tag:%#x idx:%#x way:%#x uop:%d size:%d count:%d.\n", addr, tag, idx, way, uop, emi.instSize, uopCountArray[idx][way]);
            }
            updateLRUBits(idx, way);
            uopCacheUpdates += numUops;
            return true;
        }
        DPRINTF(Decoder, "uop[[%i][%i] has valid bit %i and has a tag %x that does not match with %x\n", idx, way, uopValidArray[idx][way], uopTagArray[idx][way], tag);
        waysVisited++;
    }

    if (numFullWays == 3) {
        /* We've used up 3 ways for a 32 byte region and we're still not
             able to accomodate all uops.    Invalidate all 3 ways. */
        DPRINTF(Decoder, "Could not accomodate 32 byte region: Could not update microop in the microop cache: %#x tag:%#x idx:%#x. Affected PCs:\n", addr, tag, idx);
        for (int way = 0; way < 8; way++) {
            if (uopValidArray[idx][way] && uopTagArray[idx][way] == tag) {
                if (traceConstructor->currentTrace.state == SpecTrace::OptimizationInProcess &&
                    (traceConstructor->currentTrace.head.way == way || 
                     way == uopNextWayArray[idx][traceConstructor->currentTrace.head.way] ||
                     way == uopPrevWayArray[idx][traceConstructor->currentTrace.head.way] ||
                     traceConstructor->currentTrace.head.way == uopNextWayArray[idx][way] ||
                     traceConstructor->currentTrace.head.way == uopPrevWayArray[idx][way])) {
                    return false;
                }
                if (traceConstructor->currentTrace.state == SpecTrace::OptimizationInProcess &&
                    traceConstructor->currentTrace.addr.valid &&
                    (traceConstructor->currentTrace.addr.way == way || 
                     way == uopNextWayArray[idx][traceConstructor->currentTrace.addr.way] ||
                     way == uopPrevWayArray[idx][traceConstructor->currentTrace.addr.way] ||
                     traceConstructor->currentTrace.addr.way == uopNextWayArray[idx][way] ||
                     traceConstructor->currentTrace.addr.way == uopPrevWayArray[idx][way])) {
                    return false;
                }
                for (int uop = 0; uop < uopCountArray[idx][way]; uop++) {
                    // DPRINTF(Decoder, "%#x\n", uopAddrArray[idx][way][uop], true);
                    DPRINTF(ConstProp, "Decoder is invalidating way %i, so removing uop[%i][%i][%i]\n", way, idx, way, uop);
                    // depTracker->removeAtIndex(idx, way, uop); // changed to spec
                    // depTracker->microopAddrArray[idx][way][uop] = FullUopAddr(0,0);
                }
                uopValidArray[idx][way] = false;
                uopCountArray[idx][way] = 0;
                uopHotnessArray[idx][way] = BigSatCounter(4);
                uopPrevWayArray[idx][way] = 10;
                uopNextWayArray[idx][way] = 10;
                uopCacheWayInvalidations++;
            }
        }
        return false;
    }

    /* If we're here, there were either no unused ways or we found no
         empty slots in an already used way -- let's try unused ways first. */
    for (int way = 0; way < 8; way++) {
        if (!uopValidArray[idx][way]) {
            if (lastWay != -1) {
                /* Multi-way region. */
                uopNextWayArray[idx][lastWay] = way;
                uopPrevWayArray[idx][way] = lastWay;
                DPRINTF(ConstProp, "way %i --> way %i\n", lastWay, way);
            }
            uopCountArray[idx][way] = numUops;
            uopValidArray[idx][way] = true;
            //uopFullArray[idx][way] = false;
            uopTagArray[idx][way] = tag;
            DPRINTF(ConstProp, "Set uopTagArray[%i][%i] to %x\n", idx, way, tag);
            for (int uop = 0; uop < numUops; uop++) {
                uopAddrArray[idx][way][uop] = FullUopAddr(addr,uop);
                DPRINTF(ConstProp, "Set microopAddrArray[%i][%i][%i] to %x.%i\n", idx, way, uop, addr, uop);
                emi.instSize = size;
                uopCache[idx][way][uop] = emi;
                DPRINTF(Decoder, "Updating microop in the microop cache: %#x tag:%#x idx:%#x way:%#x uop:%d size:%d.\n", addr, tag, idx, way, uop, emi.instSize);
            }
            updateLRUBits(idx, way);
            uopCacheUpdates += numUops;
            return true;
        }
    }

    /* There aren't any unused ways. Evict the LRU way. */
    unsigned lruWay = 8;
    unsigned evictWay = 8;

    if (traceConstructor->currentTrace.state == SpecTrace::OptimizationInProcess) {
        DPRINTF(Decoder, "Trace being superoptimized has its head at uop[%i][%i][%i] and is currently optimizing uop[%i][%i][%i]\n", traceConstructor->currentTrace.head.idx, traceConstructor->currentTrace.head.way, traceConstructor->currentTrace.head.uop, traceConstructor->currentTrace.addr.idx, traceConstructor->currentTrace.addr.way, traceConstructor->currentTrace.addr.uop);
    }
    for (int way = 0; way < 8; way++) {
        // check if we are processing the trace for first time optimization -- read from way in progress
        if (traceConstructor->currentTrace.state == SpecTrace::OptimizationInProcess &&
            (traceConstructor->currentTrace.head.way == way || 
             way == uopNextWayArray[idx][traceConstructor->currentTrace.head.way] ||
             way == uopPrevWayArray[idx][traceConstructor->currentTrace.head.way] ||
             traceConstructor->currentTrace.head.way == uopNextWayArray[idx][way] ||
             traceConstructor->currentTrace.head.way == uopPrevWayArray[idx][way])) {
            continue;
        }
        if (traceConstructor->currentTrace.state == SpecTrace::OptimizationInProcess &&
            traceConstructor->currentTrace.addr.valid &&
            (traceConstructor->currentTrace.addr.way == way || 
             way == uopNextWayArray[idx][traceConstructor->currentTrace.addr.way] ||
             way == uopPrevWayArray[idx][traceConstructor->currentTrace.addr.way] ||
             traceConstructor->currentTrace.addr.way == uopNextWayArray[idx][way] ||
             traceConstructor->currentTrace.addr.way == uopPrevWayArray[idx][way])) {
            continue;
        }
        if (uopLRUArray[idx][way] < lruWay) {
            lruWay = uopLRUArray[idx][way];
            evictWay = way;
        }
    }
    if (evictWay != 8) {
        /* Invalidate all prior content. */
        DPRINTF(Decoder, "Evicting microop in the microop cache: tag:%#x idx:%d way:%d.\n Affected PCs:\n", tag, idx, evictWay);
        for (int w = 0; w < 8; w++) {
            if (uopValidArray[idx][w] && uopTagArray[idx][w] == uopTagArray[idx][evictWay]) {
                for (int uop = 0; uop < uopCountArray[idx][w]; uop++) {
                    DPRINTF(Decoder, "uop[%i][%i][%i] -- %#x:%i\n", idx, w, uop, uopAddrArray[idx][w][uop].pcAddr, uopAddrArray[idx][w][uop].uopAddr);
                    uopConflictMisses++;
                }
                uopValidArray[idx][w] = false;
                uopCountArray[idx][w] = 0;
                uopHotnessArray[idx][w] = BigSatCounter(4);
                uopPrevWayArray[idx][w] = 10;
                uopNextWayArray[idx][w] = 10;
                uopCacheWayInvalidations++;
            }
        }
        if (lastWay != -1) {
            /* Multi-way region. */
            uopNextWayArray[idx][lastWay] = evictWay;
            uopPrevWayArray[idx][evictWay] = lastWay;
            DPRINTF(ConstProp, "way %i --> way %i\n", lastWay, evictWay);
        }
        uopCountArray[idx][evictWay] = numUops;
        uopValidArray[idx][evictWay] = true;
        //uopFullArray[idx][evictWay] = false;
        uopTagArray[idx][evictWay] = tag;
        DPRINTF(ConstProp, "Set uopTagArray[%i][%i] to %x\n", idx, evictWay, tag);
        for (int uop = 0; uop < numUops; uop++) {
            uopAddrArray[idx][evictWay][uop] = FullUopAddr(addr, uop);
            DPRINTF(ConstProp, "Set microopAddrArray[%i][%i][%i] to %x.%i\n", idx, evictWay, uop, addr, uop);
            emi.instSize = size;
            uopCache[idx][evictWay][uop] = emi;
            DPRINTF(Decoder, "Updating microop in the microop cache: %#x tag:%#x idx:%#x way:%#x uop:%d size:%d.\n", addr, tag, idx, evictWay, uop, emi.instSize);
        }
        updateLRUBits(idx, evictWay);
        uopCacheUpdates += numUops;
        return true;
    }

    DPRINTF(Decoder, "Eviction failed: Could not update microop in the microop cache :%#x tag:%#x.\n", addr, tag);
    return false;
}

void
Decoder::updateStreamTrace(uint64_t traceID, X86ISA::PCState &thisPC) {
    traceConstructor->streamTrace = traceConstructor->traceMap[traceID];
    int idx = traceConstructor->streamTrace.getOptimizedHead().idx;
    int baseWay = traceConstructor->streamTrace.getOptimizedHead().way;
    FullUopAddr addr = FullUopAddr(thisPC.pc(), thisPC.upc());

    for (int way = baseWay; way != SPEC_CACHE_WAY_MAGIC_NUM; way = speculativeNextWayArray[idx][way]) {
        if (speculativeValidArray[idx][way] && speculativeTraceIDArray[idx][way] == traceID) {
            for (int uop = 0; uop < speculativeCountArray[idx][way]; uop++) {
                if (speculativeAddrArray[idx][way][uop] == addr) {
                    traceConstructor->streamTrace.addr = FullCacheIdx(idx, way, uop);
                    DPRINTF(Decoder, "updateStreamTrace: stream trace %d to begin at SPEC[%d][%d][%d]\n", traceID, idx, way, uop);
                    return;
                }
            }
        }
    }
    DPRINTF(Decoder, "updateStreamTrace: couldn't find instruction in stream trace %d\n", traceID);
}

bool
Decoder::addUopToSpeculativeCache(SpecTrace &trace, SuperOptimizedMicroop superoptimized_microop) {

   
    StaticInstPtr inst =  superoptimized_microop.inst;
    assert(inst);
    Addr addr = superoptimized_microop.instAddr.pcAddr;
    uint16_t uop = superoptimized_microop.instAddr.uopAddr; 
    uint64_t traceID = trace.id;

    uint64_t spec_cache_idx = (addr >> SPEC_NUM_INDEX_BITS) & SPEC_INDEX_MASK;
    uint64_t tag = (addr >> ( SPEC_NUM_INDEX_BITS + 5 ));
    int numFullWays = 0;
    int lastWay = -1;

    uint64_t idx;
    int baseWay = 0;
    int waysVisited = 0;
    if (trace.getOptimizedHead().valid) {
        idx = trace.getOptimizedHead().idx;
        baseWay = trace.getOptimizedHead().way;
        DPRINTF(Decoder, "addUopToSpeculativeCache: Trace %d optimized head is (%d,%d)!\n", traceID, idx, baseWay);
    }
    else 
    {
        idx = spec_cache_idx;
        DPRINTF(Decoder, "addUopToSpeculativeCache: Trace %d optimized head is not valid!\n", traceID);
    }


    /* Link list traversal traversal. */
    for (int way = baseWay; way != SPEC_CACHE_WAY_MAGIC_NUM  && numFullWays < 3 && waysVisited < SPEC_CACHE_NUM_WAYS; way = speculativeNextWayArray[idx][way]) {
        if (speculativeValidArray[idx][way] && speculativeTraceIDArray[idx][way] == traceID) {
            /* Check if this way can accommodate the uops that correspond
                 to this instruction. */
            int waySize = speculativeCountArray[idx][way];
            if (waySize == 6) {
                lastWay = way;
                waysVisited++;
                continue;
            }
            speculativeCountArray[idx][way]++;
            speculativeValidArray[idx][way] = true;
            speculativeTraceIDArray[idx][way] = traceID;
            speculativeCache[idx][way][waySize] = inst;
            speculativeAddrArray[idx][way][waySize] = FullUopAddr(addr, uop);



            DPRINTF(ConstProp, "Set speculativeAddrArray[%i][%i][%i] to %x.%i\n", idx, way, waySize, addr, uop);
            updateLRUBitsSpeculative(idx, way);
            DPRINTF(Decoder, "Adding microop in the speculative cache: %#x tag:%#x idx:%d way:%d uop:%d nextway:%d prevway:%d.\n", addr, tag, idx, way, waySize, speculativeNextWayArray[idx][way], speculativePrevWayArray[idx][way]);
            if (speculativeCountArray[idx][way] == 6) {
                numFullWays++;
            }

            // update optimized head if it's not valid yet!
            if (!trace.getOptimizedHead().valid) {
                // when the trace head is not valid, it means this is always the frist microop in the trace
                // therefore waySize should be equal to zero
                assert(waySize == 0);
                DPRINTF(SuperOp, "updateSpecTrace: Trace %d optimized head is not valid!\n", trace.id);
                trace.setOptimizedTraceHead(FullCacheIdx(idx, way, waySize));
                DPRINTF(SuperOp, "updateSpecTrace: Trace %d optimized head is updated to (%d, %d)!\n", trace.id, idx, way);
                // after the for lopp trace.optimizedHead.valid should always be true otherwise something is wrong!
                assert(trace.getOptimizedHead().valid);
            }

            return true;
        }
        waysVisited++;
    }

    if (numFullWays >= 3) {
        // Replace this section
        panic("Already 3 full ways so couldn't add optimized inst");
    }

    // If we make it here, then we need to add a way for this tag
    for (int way = 0; way < SPEC_CACHE_NUM_WAYS; way++) {
        if (!speculativeValidArray[idx][way]) {
            if (lastWay != -1) {
                /* Multi-way region. */
                speculativeNextWayArray[idx][lastWay] = way;
                speculativePrevWayArray[idx][way] = lastWay;
            }
            int u = speculativeCountArray[idx][way]++;
            speculativeValidArray[idx][way] = true;
            speculativeTagArray[idx][way] = tag;
            speculativeTraceIDArray[idx][way] = traceID;
            speculativeCache[idx][way][u] = inst;
            speculativeAddrArray[idx][way][u] = FullUopAddr(addr, uop);
            updateLRUBitsSpeculative(idx, way);


            DPRINTF(Decoder, "Allocating a previous invalid way for trace %d and adding microop in the speculative cache: %#x tag:%#x idx:%d way:%d uop:%d nextWay:%d prevway:%d.\n", traceID, addr, tag, idx, way, u, speculativeNextWayArray[idx][way], speculativePrevWayArray[idx][way]);
            DPRINTF(ConstProp, "Set speculativeAddrArray[%i][%i][%i] to %x.%i\n", idx, way, u, addr, uop);

            // update optimized head if it's not valid yet!
            if (!trace.getOptimizedHead().valid) {
                // when the trace head is not valid, it means this is always the frist microop in the trace
                // therefore waySize should be equal to zero
                assert(u == 0);
                DPRINTF(SuperOp, "updateSpecTrace: Trace %d optimized head is not valid!\n", trace.id);
                trace.setOptimizedTraceHead(FullCacheIdx(idx, way, u));
                DPRINTF(SuperOp, "updateSpecTrace: Trace %d optimized head is updated to (%d, %d)!\n", trace.id, idx, way);
                // after the for lopp trace.optimizedHead.valid should always be true otherwise something is wrong!
                assert(trace.getOptimizedHead().valid);
            }

            return true;
        }
    }

    // If we make it here, we need to evict a way to make space -- evicted region shouldn't be currently in use for optimization
    unsigned lruWay = SPEC_CACHE_NUM_WAYS;
    unsigned evictWay = SPEC_CACHE_NUM_WAYS;
    for (int way = 0; way < SPEC_CACHE_NUM_WAYS; way++) {
        // check if we are processing the trace for first time optimization -- write to way in progress
        // if (traceConstructor->currentTrace.state == SpecTrace::OptimizationInProcess) {
            
        //     // a trace should never be added to spec cache when optimization is in process
        //     assert(0);
        //     DPRINTF(Decoder, "Can't evict becuase OptimizationInProcess: tag:%#x idx:%d way:%d. currentTrace.id: %d\n", tag, idx, way, traceConstructor->currentTrace.id);
        //     continue;
        // }
       
        // check if we are streaming the trace -- read from way in progress
        if ((traceConstructor->streamTrace.id != 0) && 
            (traceConstructor->streamTrace.id == speculativeTraceIDArray[idx][way] ||
            traceConstructor->streamTrace.id == speculativeTraceIDArray[idx][speculativeNextWayArray[idx][way]] ||
            traceConstructor->streamTrace.id == speculativeTraceIDArray[idx][speculativePrevWayArray[idx][way]])) 
        {
            DPRINTF(Decoder, "Can't evict becuase we are streaming from this trace: tag:%#x idx:%d way:%d. streamTrace.id: %d\n", tag, idx, way, traceConstructor->streamTrace.id);
            continue;
        }

        if (isSpeculativeCacheActive()) assert(currentActiveTraceID);
        // Check of we are going to stream this trace starting in the next cycle (this was a bug!)
        if ((isSpeculativeCacheActive() && currentActiveTraceID != 0) && 
            (currentActiveTraceID == speculativeTraceIDArray[idx][way] ||
            currentActiveTraceID == speculativeTraceIDArray[idx][speculativeNextWayArray[idx][way]] ||
            currentActiveTraceID == speculativeTraceIDArray[idx][speculativePrevWayArray[idx][way]])) 
        {
            DPRINTF(Decoder, "Can't evict becuase we just activated streaming from this trace: tag:%#x idx:%d way:%d. currentActiveTraceID: %d\n", tag, idx, way, currentActiveTraceID);
            continue;
        }

        if (speculativeLRUArray[idx][way] < lruWay) {
            DPRINTF(Decoder, "lruWay = %d,  speculativeLRUArray[%d][%d] = %d\n", lruWay, idx, way, speculativeLRUArray[idx][way]);
            lruWay = speculativeLRUArray[idx][way];
            evictWay = way;
        }
        else 
        {
            DPRINTF(Decoder, "lruWay = %d,  speculativeLRUArray[%d][%d] = %d\n", lruWay, idx, way, speculativeLRUArray[idx][way]);
        }
    }
    if (evictWay != SPEC_CACHE_NUM_WAYS) {
        DPRINTF(Decoder, "Evicting microop in the speculative cache: tag:%#x idx:%d way:%d.\n Affected PCs:\n", tag, idx, evictWay);
        /* Invalidate all prior content. */
        unsigned int evictedTraceID = speculativeTraceIDArray[idx][evictWay];
        uint64_t     evcitedTag = speculativeTagArray[idx][evictWay];
        assert(evictedTraceID); // never should be zero
        assert(evcitedTag);
        for (int w = 0; w < SPEC_CACHE_NUM_WAYS; w++) {
            if (speculativeValidArray[idx][w] &&
                speculativeTraceIDArray[idx][w] == evictedTraceID) 
            {
                for (int uop = 0; uop < speculativeCountArray[idx][w]; uop++) {
                    DPRINTF(Decoder, "Trace %i: spec[%i][%i][%i] -- %#x:%i\n", speculativeTraceIDArray[idx][w], idx, w, uop, speculativeAddrArray[idx][w][uop].pcAddr, speculativeAddrArray[idx][w][uop].uopAddr);
                }
                speculativeValidArray[idx][w] = false;
                speculativeCountArray[idx][w] = 0;
                speculativePrevWayArray[idx][w] = SPEC_CACHE_WAY_MAGIC_NUM;
                speculativeNextWayArray[idx][w] = SPEC_CACHE_WAY_MAGIC_NUM;
                speculativeTraceIDArray[idx][w] = 0;
                speculativeTagArray[idx][w] = 0;
                speculativeEvictionStat[idx][w]++; // stat to see how many times each way is getting evicted
                StaticInstPtr macroOp = speculativeCache[idx][w][0]->macroOp;
                if (macroOp) {
                    macroOp->deleteMicroOps();
                    macroOp = NULL;
                }
            }
        }
        // trace map should always hold it
        DPRINTF(Decoder, "Removing trace %d from Trace Map.  Tag :%#x\n", evictedTraceID, evcitedTag);
        assert(traceConstructor->traceMap.find(evictedTraceID) != traceConstructor->traceMap.end());
        traceConstructor->traceMap.erase(evictedTraceID);
        if (lastWay != -1) {
            /* Multi-way region. */
            speculativeNextWayArray[idx][lastWay] = evictWay;
            speculativePrevWayArray[idx][evictWay] = lastWay;
        }
        int u = speculativeCountArray[idx][evictWay]++;
        speculativeValidArray[idx][evictWay] = true;
        speculativeTagArray[idx][evictWay] = tag;
        speculativeTraceIDArray[idx][evictWay] = traceID;
        speculativeCache[idx][evictWay][u] = inst;
        speculativeAddrArray[idx][evictWay][u] = FullUopAddr(addr, uop);

        DPRINTF(ConstProp, "Set speculativeAddrArray[%i][%i][%i] to %x.%i\n", idx, evictWay, u, addr, uop);
        updateLRUBitsSpeculative(idx, evictWay);        
        DPRINTF(Decoder, "Evicting and allocating a way and  adding microop in the speculative cache: %#x tag:%#x idx:%d way:%d uop:%d.\n", addr, tag, idx, evictWay, u);

        // update optimized head if it's not valid yet!
        if (!trace.getOptimizedHead().valid) {
            // when the trace head is not valid, it means this is always the frist microop in the trace
            // therefore waySize should be equal to zero
            assert(u == 0);
            DPRINTF(SuperOp, "updateSpecTrace: Trace %d optimized head is not valid!\n", trace.id);
            trace.setOptimizedTraceHead(FullCacheIdx(idx, evictWay, 0));
            DPRINTF(SuperOp, "updateSpecTrace: Trace %d optimized head is updated to (%d, %d)!\n", trace.id, idx, evictWay);
            // after the for lopp trace.optimizedHead.valid should always be true otherwise something is wrong!
            assert(trace.getOptimizedHead().valid);
        }

        return true;
    }
    else 
    {
        /// This is normal to happen but the problem is that we don't have the necessary logic to cause a stall
        panic("We always should be able to find a way to evict!");
    }
    DPRINTF(ConstProp, "Optimized trace could not be loaded into speculative cache because eviction failed\n");

    // if we can't evict a way, we need to put the spec cache into a consistent state before inserting this trace
    // Just invalid all the way's that this trace with TraceID has

    // should we remove the trace from the trace queue too? 

    /// This is normal to happen but the problem is that we don't have the necessary logic to cause a stall
    // what is the policy in case of an eviction failure in general?
    // if this happened, we need to add logic to make sure that we stall super optimizer
    // otherwise it's gonna cause hard to find bugs!
    panic("Couldn't write to spec cache!");
    return false;
}

bool
Decoder::isHitInUopCache(Addr addr)
{
  int idx = (addr >> 5) & 0x1f;
  uint64_t tag = (addr >> 10);
  for (int way = 0; way < 8; way++) {
    if (uopValidArray[idx][way] && uopTagArray[idx][way] == tag) {
      for (int uop = 0; uop < uopCountArray[idx][way]; uop++) {
        if (uopAddrArray[idx][way][uop].pcAddr == addr) {
          DPRINTF(Decoder, "Is hit in the microop cache? true: %#x tag:%#x idx:%#x way:%#x uop:%x size:%d.\n", addr, tag, idx, way, uop, uopCache[idx][way][uop].instSize);
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
                if (uopAddrArray[idx][way][uop].pcAddr == addr) {
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

                    StaticInstPtr si = decode(emi, addr);

                    if (uopHotnessArray[idx][way].read() > 7)
                    {
                        if (si->isMacroop())
                        {
                            si->setUOpCacheHotTrace(true);
                            // set all the microops in this macro as fetched from uop cache
                            for (uint16_t idx = 0; idx < si->getNumMicroops(); idx++)
                            {
                                StaticInstPtr micro = si->fetchMicroop((MicroPC)idx);
                                micro->setUOpCacheHotTrace(true);
                            }
                        }
                        else 
                        {
                            si->setUOpCacheHotTrace(true);
                        }
                    }

                    return si;


                }
            }
        }
    }
    panic("microop cache hit, but couldn't fetch from the cache.");
    return NULL;
}

// LVPredictor return int8_t confidence, if this confidence if less than zero then just return
unsigned
Decoder::isTraceAvailable(Addr addr, uint64_t value, uint64_t confidence) {
    unsigned maxScore = 0;
    unsigned maxTraceID = 0;

    //if (confidence < 0) return 0;

    for (auto it = traceConstructor->traceMap.begin(); it != traceConstructor->traceMap.end(); it++) {
        SpecTrace trace = it->second;
        if (trace.getTraceHeadAddr().pcAddr == addr) {
            DPRINTF(Decoder, "Checking Trace %i for at addr = %#x\n", trace.id, addr);

            if (trace.state == SpecTrace::OptimizationInProcess  ||
                trace.state == SpecTrace::QueuedForFirstTimeOptimization) {
                DPRINTF(Decoder, "Trace %i is still being processed (state:%d)\n", trace.id, trace.state);
                continue;
            }

            assert(trace.state == SpecTrace::Complete);

            int numValidPredSources = 0; int numNotFoundPredSources = 0; int numMatchedPredSources = 0;
            for (int i = 0; i < 4; i++)
            {
                if (trace.source[i].valid)
                {
                    numValidPredSources++;
                    LVPredUnit::lvpReturnValues ret;
                    if (traceConstructor->loadPred->makePredictionForTraceGenStage(trace.source[i].addr.pcAddr, trace.source[i].addr.uopAddr, 0 , ret))
                    {
                        if (trace.source[i].value == ret.predictedValue && ret.confidence >= 5)
                        {
                            DPRINTF(Decoder, "Found the prediction source with address of %#x:%d in the predictor and the values match! Confidence is %d! trace.source[i].value = %#x ret.predictedValue = %#x\n", 
                                              trace.source[i].addr.pcAddr, trace.source[i].addr.uopAddr, ret.confidence, trace.source[i].value, ret.predictedValue );
                            numMatchedPredSources++;
                        }
                        else 
                        {
                            DPRINTF(Decoder, "Found the prediction source with address of %#x:%d in the predictor but the values dont match or the confidence is low! trace.source[i].value = %#x ret.predictedValue = %#x ret.confidence = %d\n", 
                                              trace.source[i].addr.pcAddr, trace.source[i].addr.uopAddr, trace.source[i].value, ret.predictedValue, ret.confidence);
                        }
                    }
                    else 
                    {
                        DPRINTF(Decoder, "Can't find prediction source with address of %#x:%d in the predictor!\n", trace.source[i].addr.pcAddr, trace.source[i].addr.uopAddr);
                        numNotFoundPredSources++;
                    }
                    
                }
            }


            if (numMatchedPredSources != numValidPredSources) { // trace with incorrect value, move on to a different trace
                DPRINTF(Decoder, "The predicted value of some of the prediction sources in Trace %d are not consistent with the current state of predictor! numValidPredSources = %d numMatchedPredSources = %d\n", trace.id, numValidPredSources, numMatchedPredSources);
                continue;
            }

            if ((trace.controlSources[0].valid && trace.controlSources[0].confidence < 5) || 
                (trace.controlSources[1].valid && trace.controlSources[1].confidence < 5 )) {
                DPRINTF(Decoder, "Control sources have low confidence\n");
                continue;
            }

            unsigned traceConfidence = minConfidence(trace.id);
            unsigned latency = maxLatency(trace.id);
            unsigned shrinkage = trace.length - trace.shrunkLength;
            DPRINTF(Decoder, "confidence=%i, latency=%i, shrinkage=%i\n", traceConfidence, latency, shrinkage);
            if (traceConfidence < 5) { // low confidence, move on the the next trace at this index
                continue;
            }

            // taking product because we want the highest confidence, latency, and shrinkage
            // we don't include hotness here as that is considered while generating a trace
            unsigned score = traceConfidence * shrinkage * (latency + 1);
            if (score > maxScore) {
                maxScore = score; // Select this trace
                maxTraceID = trace.id; 
            }
        }
    }

    if (maxTraceID) {
        DPRINTF(Decoder, "isTraceAvailable returning trace %i\n", maxTraceID);
    } else {
        DPRINTF(Decoder, "isTraceAvailable: no traces found\n");
    }
    return maxTraceID;
}

void
Decoder::doSquash(Addr addr) {
    traceConstructor->streamTrace.addr.valid = false;
    traceConstructor->streamTrace.id = 0;

    int idx = (addr >> SPEC_NUM_INDEX_BITS) & SPEC_INDEX_MASK;
    for (int way = 0; way < SPEC_CACHE_NUM_WAYS; way++) {
        if (speculativeValidArray[idx][way] && speculativeAddrArray[idx][way][0].pcAddr == addr && speculativePrevWayArray[idx][way] == 10) {

            assert(traceConstructor->traceMap.find(speculativeTraceIDArray[idx][way]) != traceConstructor->traceMap.end());

            //SpecTrace trace = traceConstructor->traceMap[speculativeTraceIDArray[idx][way]];
            for (int i=0; i<4; i++) 
            {
                if (traceConstructor->traceMap[speculativeTraceIDArray[idx][way]].source[i].valid && 
                    traceConstructor->traceMap[speculativeTraceIDArray[idx][way]].source[i].addr.pcAddr == addr) 
                {
                    
                    // Is there a limit on confidence? I'm assuming 0.
                    if (traceConstructor->traceMap[speculativeTraceIDArray[idx][way]].source[i].confidence > 0) 
                    {
                        traceConstructor->traceMap[speculativeTraceIDArray[idx][way]].source[i].confidence--;
                        return;
                    }
                }
                // we might have gotten squashed at the middle of the trace
                //traceConstructor->traceMap[speculativeTraceIDArray[idx][way]].source[0].confidence--;
            }
        }
    }
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
    int idx = (addr >> SPEC_NUM_INDEX_BITS) & SPEC_INDEX_MASK;
    uint64_t tag = (addr >> (SPEC_NUM_INDEX_BITS + 5));
    for (int way = 0; way < SPEC_CACHE_NUM_WAYS; way++) {
        if (speculativeValidArray[idx][way] && speculativeTagArray[idx][way] == tag) {
            invalidateSpecCacheLine(idx, way);
        }
    }
}

void
Decoder::invalidateSpecTrace(FullCacheIdx addr, uint64_t trace_id) 
{
    
    assert(trace_id);
    
    /*
     * 1. Find tag match in spec cache
     * 2. Clear out tag
     * 3. If has a previous line, clear that one too
     * 4. If has a next line, clear that one too
     * Put the check for prev and next in the tag function to call recursively
     */
    int idx = addr.idx;
    int evict_way = addr.way;
    uint64_t evictTag = speculativeTagArray[idx][evict_way];
    
    assert(evictTag);
    assert(speculativeTraceIDArray[idx][evict_way] == trace_id);

    for (int w = 0; w < SPEC_CACHE_NUM_WAYS; w++) 
    {

        if (speculativeValidArray[idx][w] && 
            speculativeTraceIDArray[idx][w] == trace_id) 
        {
                //assert(speculativeTagArray[idx][w] == evictTag);
                for (int uop = 0; uop < speculativeCountArray[idx][w]; uop++) {
                        DPRINTF(Decoder, "Trace %i: spec[%i][%i][%i] -- %#x:%i\n", speculativeTraceIDArray[idx][w], idx, w, uop, speculativeAddrArray[idx][w][uop].pcAddr, speculativeAddrArray[idx][w][uop].uopAddr);
                }
                speculativeValidArray[idx][w] = false;
                speculativeCountArray[idx][w] = 0;
                speculativePrevWayArray[idx][w] = SPEC_CACHE_WAY_MAGIC_NUM;
                speculativeNextWayArray[idx][w] = SPEC_CACHE_WAY_MAGIC_NUM;
                speculativeTraceIDArray[idx][w] = 0;
                speculativeTagArray[idx][w] = 0;
                speculativeEvictionStat[idx][w]++; // stat to see how many times each way is getting evicted
                StaticInstPtr macroOp = speculativeCache[idx][w][0]->macroOp;
                if (macroOp) {
                    macroOp->deleteMicroOps();
                    macroOp = NULL;
                }
        }
    
    }
    
}



void
Decoder::invalidateSpecCacheLine(int idx, int way) {
    for (int u = 0; u < 6; u++) {
        // depTracker->registerRemovalOfTraceInst(idx, way, u);
        speculativeAddrArray[idx][way][u] = FullUopAddr(0,0);
    }
    if (speculativePrevWayArray[idx][way] != SPEC_CACHE_WAY_MAGIC_NUM) {
        invalidateSpecCacheLine(idx, speculativePrevWayArray[idx][way]);
        speculativePrevWayArray[idx][way] = SPEC_CACHE_WAY_MAGIC_NUM;
    }
    if (speculativeNextWayArray[idx][way] != SPEC_CACHE_WAY_MAGIC_NUM) {
        invalidateSpecCacheLine(idx, speculativeNextWayArray[idx][way]);
        speculativeNextWayArray[idx][way] = SPEC_CACHE_WAY_MAGIC_NUM;
    }
}

unsigned
Decoder::minConfidence(uint64_t traceId) {
    unsigned minConf = traceConstructor->traceMap[traceId].source[0].confidence;
    for (int i = 0; i < 4; i++) {
        assert(traceConstructor->traceMap.find(traceId) != traceConstructor->traceMap.end());
        if (traceConstructor->traceMap[traceId].source[i].valid) {
            unsigned sourceConf = traceConstructor->traceMap[traceId].source[i].confidence;
            if (sourceConf < minConf) { minConf = sourceConf; }
        }
    }
    return minConf;
}

unsigned
Decoder::maxLatency(uint64_t traceId) {
    unsigned maxLat = 0;
    for (int i = 0; i < 4; i++) {
        if (traceConstructor->traceMap[traceId].source[i].valid) {
            assert(traceConstructor->traceMap.find(traceId) != traceConstructor->traceMap.end());
            unsigned sourceLat = traceConstructor->traceMap[traceId].source[i].latency;
            if (sourceLat > maxLat) { maxLat = sourceLat; }
        }
    }
    return maxLat;
}

// sends back the microop from the active trace
// In case of a folded branch, nextPC and predict_taken should be set by the function
StaticInstPtr 
Decoder::getSuperOptimizedMicroop(uint64_t traceID, X86ISA::PCState &thisPC, X86ISA::PCState &nextPC, bool &predict_taken) {
    int idx = traceConstructor->traceMap[traceID].addr.idx;
    int way = traceConstructor->traceMap[traceID].addr.way;
    int uop = traceConstructor->traceMap[traceID].addr.uop;

    
    if ((traceConstructor->streamTrace.id != traceID || !traceConstructor->streamTrace.addr.valid) ) { 

        DPRINTF(Decoder, "traceConstructor->streamTrace.id  = %d\n", traceConstructor->streamTrace.id);
        DPRINTF(Decoder, "traceConstructor->streamTrace.addr.valid = %d\n", traceConstructor->streamTrace.addr.valid);
        DPRINTF(Decoder, "thisPC._pc = %x speculativeAddrArray[%d][%d][%d].pcAddr = %x:\n", thisPC._pc, idx, way, uop , speculativeAddrArray[idx][way][uop].pcAddr); 
    
        

        traceConstructor->streamTrace = traceConstructor->traceMap[traceID];
        DPRINTF(Decoder, "Trace %d ought to be triggered with shrinkage %d:\n", traceConstructor->streamTrace.id, (traceConstructor->streamTrace.length - traceConstructor->streamTrace.shrunkLength));
        traceConstructor->dumpTrace(traceConstructor->streamTrace);

        int _idx = traceConstructor->streamTrace.addr.idx;
        int _way = traceConstructor->streamTrace.addr.way;
        int _uop = traceConstructor->streamTrace.addr.uop;

        DPRINTF(Decoder, "thisPC._pc = %x speculativeAddrArray[%d][%d][%d].pcAddr = %x\n",thisPC._pc, _idx, _way, _uop , speculativeAddrArray[_idx][_way][_uop].pcAddr);
        DPRINTF(Decoder,"speculativeValidArray[%d][%d] = %d\n", _idx, _way , speculativeValidArray[_idx][_way]);
        DPRINTF(Decoder,"speculativeTraceIDArray[%d][%d] = %d, traceID = %d\n", _idx, _way ,speculativeTraceIDArray[_idx][_way], traceID); 
                        
        // assert(thisPC._pc == speculativeAddrArray[_idx][_way][_uop].pcAddr);
        // assert(traceConstructor->streamTrace.addr.valid);
        assert(speculativeTraceIDArray[_idx][_way] == traceID);
    }
    // if the traceID is correct and addr is valid, then thisPC._pc should be equal to speculativeAddrArray[idx][way][uop].pcAddr
    // if this is not true, then there is a bug?

        DPRINTF(Decoder, "thisPC._pc = %x speculativeAddrArray[%d][%d][%d].pcAddr = %x\n",thisPC._pc, idx, way, uop , speculativeAddrArray[idx][way][uop].pcAddr);
        DPRINTF(Decoder,"speculativeValidArray[%d][%d] = %d\n", idx, way , speculativeValidArray[idx][way]);
        DPRINTF(Decoder,"speculativeTraceIDArray[%d][%d] = %d, traceID = %d\n", idx, way ,speculativeTraceIDArray[idx][way], traceID); 
        	

    idx = traceConstructor->streamTrace.addr.idx;
    way = traceConstructor->streamTrace.addr.way;
    uop = traceConstructor->streamTrace.addr.uop;

    thisPC._pc = speculativeAddrArray[idx][way][uop].pcAddr;

    StaticInstPtr curInst = speculativeCache[idx][way][uop];
    /// dump all the micropps in the way and then assert!
    if (!curInst)
    {
        for (size_t i = 0; i < 6; i++)
        {
            if (speculativeCache[idx][way][i])
                DPRINTF(Decoder, "speculativeCache[%d][%d][%d] = %s!\n", idx, way, i, speculativeCache[idx][way][i]->getName());
            else 
                DPRINTF(Decoder, "speculativeCache[%d][%d][%d] = NULL!\n", idx, way, i);
        }
        
    }
    assert(curInst);
    FullUopAddr instAddr = speculativeAddrArray[idx][way][uop];
    predict_taken = false;

    if (curInst->isControl()) {
        predict_taken = traceConstructor->branchPred->lookupWithoutUpdate(0, instAddr.pcAddr);
    }

    assert(curInst->macroOp);
    thisPC.size(curInst->macroOp->getMacroopSize());
    thisPC._npc = thisPC._pc + curInst->macroOp->getMacroopSize();
    thisPC._nupc = thisPC._upc + 1;

    traceConstructor->advanceTrace(traceConstructor->streamTrace);
    if (traceConstructor->streamTrace.addr.valid) {
        idx = traceConstructor->streamTrace.addr.idx;
        way = traceConstructor->streamTrace.addr.way;
        uop = traceConstructor->streamTrace.addr.uop;
        StaticInstPtr nextInst = speculativeCache[idx][way][uop];
        FullUopAddr instAddr = speculativeAddrArray[idx][way][uop];
        nextPC._pc = instAddr.pcAddr;
        nextPC._upc = instAddr.uopAddr;

        /* Leverage advanceTrace's control tracking feature to set these. */
        SpecTrace trace = traceConstructor->streamTrace;
        traceConstructor->advanceTrace(trace);
        nextPC._npc = speculativeAddrArray[trace.addr.idx][trace.addr.way][trace.addr.uop].pcAddr;
        nextPC._nupc = speculativeAddrArray[trace.addr.idx][trace.addr.way][trace.addr.uop].uopAddr;
        nextPC.valid = true;
    } else {
        /* Assuming a trace always ends at the last micro-op of a macro-op. */
        nextPC._pc += curInst->macroOp->getMacroopSize();
        nextPC._npc = nextPC._pc + 1;
        nextPC.size(0);
        nextPC._upc = 0;
        nextPC._nupc = 1;
        nextPC.valid = false;
        //curInst->setEndOfTrace();
    }

    if (curInst->isEndOfTrace()) {
        traceConstructor->streamTrace.id = 0;
    }

    // set the trace ID for this instruction
    curInst->traceID = traceID; 

    return curInst;
}

StaticInstPtr
Decoder::decode(PCState &nextPC, unsigned cycleAdded, ThreadID tid)
{
    if (isUopCachePresent && isUopCacheActive() && isHitInUopCache(nextPC.instAddr())) 
    {
        DPRINTF(Decoder, "Fetching microop from the microop cache: %s.\n", nextPC);
      
        StaticInstPtr si = fetchUopFromUopCache(nextPC.instAddr(), nextPC);

        if (si->isMacroop())
        {
            si->setStreamedFromUOpCache(true);
            // set all the microops in this macro as fetched from uop cache
            for (uint16_t idx = 0; idx < si->getNumMicroops(); idx++)
            {
                StaticInstPtr micro = si->fetchMicroop((MicroPC)idx);
                micro->setStreamedFromUOpCache(true);
            }
        }
        else 
        {
            si->setStreamedFromUOpCache(true);
        }
             

        return si;
    }

    if (!instDone)
        return NULL;
    instDone = false;
    emi.instSize = basePC + offset - origPC;
    updateNPC(nextPC);


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
                cur->macroOp = si;
                StaticInstPtr prev = si->fetchMicroop(i-1);
                prev->macroOp = si;
                if ((cur->isInteger() || cur->isNop() || cur->isControl() || cur->isMicroBranch()) && prev->isLoad() && !prev->isRipRel()) {
                    i++;
                }
                numFusedMicroops++;
            }
            DPRINTF(Decoder, "Inserting fused microops: (%d/%d)\n", numFusedMicroops, si->getNumMicroops());
        } else if (si->isMacroop()) {
            numFusedMicroops = si->getNumMicroops();
        }
        if (!isHitInUopCache(nextPC.instAddr())) {
            updateUopInUopCache(si->machInst, nextPC.instAddr(), numFusedMicroops, emi.instSize, cycleAdded, tid);
        }
    }

    return si;
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
