/*
 * Copyright (c) 2012 Google
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

#ifndef __ARCH_X86_DECODER_HH__
#define __ARCH_X86_DECODER_HH__

#include <cassert>
#include <unordered_map>
#include <vector>

#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "arch/x86/decoder_structs.hh"
#include "arch/x86/regs/misc.hh"
#include "arch/x86/superop/trace_based_graph.hh"
#include "arch/x86/types.hh"
#include "base/bitfield.hh"
#include "base/logging.hh"
#include "base/statistics.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "cpu/decode_cache.hh"
#include "cpu/pred/big_sat_counter.hh"
#include "cpu/static_inst.hh"
#include "debug/Decoder.hh"
#include "params/DerivO3CPU.hh"

namespace X86ISA
{

class ISA;



class Decoder
{
  public :


  private:
    //These are defined and documented in decoder_tables.cc
    static const uint8_t SizeTypeToSize[3][10];
    typedef const uint8_t ByteTable[256];
    static ByteTable Prefixes;

    static ByteTable UsesModRMOneByte;
    static ByteTable UsesModRMTwoByte;
    static ByteTable UsesModRMThreeByte0F38;
    static ByteTable UsesModRMThreeByte0F3A;

    static ByteTable ImmediateTypeOneByte;
    static ByteTable ImmediateTypeTwoByte;
    static ByteTable ImmediateTypeThreeByte0F38;
    static ByteTable ImmediateTypeThreeByte0F3A;
    static ByteTable ImmediateTypeVex[10];

  protected:
    struct InstBytes
    {
        StaticInstPtr si;
        std::vector<MachInst> chunks;
        std::vector<MachInst> masks;
        int lastOffset;

        InstBytes() : lastOffset(0)
        {}
    };

    static InstBytes dummy;

    //The bytes to be predecoded
    MachInst fetchChunk;
    InstBytes *instBytes;
    int chunkIdx;
    //The pc of the start of fetchChunk
    Addr basePC;
    //The pc the current instruction started at
    Addr origPC;
    //The offset into fetchChunk of current processing
    int offset;
    //The extended machine instruction being generated
    ExtMachInst emi;
    //Predecoding state
    X86Mode mode;
    X86SubMode submode;
    uint8_t altOp;
    uint8_t defOp;
    uint8_t altAddr;
    uint8_t defAddr;
    uint8_t stack;

  public:
    bool isUopCachePresent;
    bool isMicroFusionPresent;
    bool uopCacheActive;
    bool isSuperOptimizationPresent;
    ExtMachInst *** uopCache;
    FullUopAddr *** uopAddrArray;
    uint64_t ** uopTagArray;
    int ** uopPrevWayArray;
    int ** uopNextWayArray;
    bool ** uopValidArray;
    int ** uopCountArray;
    int ** uopLRUArray;
    BigSatCounter ** uopHotnessArray;
    bool ** uopProfitableTrace;
    // Parallel cache for optimized micro-ops
    bool redirectDueToLVPSquashing;
    bool isSpeculativeCachePresent;
    bool speculativeCacheActive;
    uint64_t currentActiveTraceID;
    StaticInstPtr *** speculativeCache;
    FullUopAddr *** speculativeAddrArray;
    uint64_t ** speculativeTagArray ;
    uint64_t ** speculativeEvictionStat;
    int ** speculativePrevWayArray;
    int ** speculativeNextWayArray;
    bool ** speculativeValidArray;
    int ** speculativeCountArray;
    int ** speculativeLRUArray;
	BigSatCounter ** specHotnessArray;
	uint64_t ** speculativeTraceIDArray;
    std::deque<SuperOptimizedMicroop> specCacheWriteQueue;
    std::deque<SpecCacheHistory>  specCacheHistoryBuffer;


    //uint64_t SPEC_INDEX_MASK;
    //uint64_t SPEC_NUM_INDEX_BITS;
    uint64_t SPEC_CACHE_NUM_WAYS;
    uint64_t SPEC_CACHE_NUM_SETS;
    uint64_t SPEC_CACHE_WAY_MAGIC_NUM;

    uint64_t UOP_CACHE_NUM_WAYS;
    uint64_t UOP_CACHE_NUM_SETS;
    uint64_t UOP_CACHE_WAY_MAGIC_NUM;

    bool lvpLookupAtFetch;

	void tickAllHotnessCounters() {
		for (int i=0; i<UOP_CACHE_NUM_SETS; i++) {
			for (int j=0; j<UOP_CACHE_NUM_WAYS; j++) {
				uopHotnessArray[i][j].decrement();
			}
		}
	}

    void increaseSpecWayHotness(int idx, int way) {specHotnessArray[idx][way].increment();}
    void decreaseSpecWayHotness(int idx, int way) {specHotnessArray[idx][way].decrement();}

    void setUopTraceProfitableForSuperOptimization(Addr addr, bool state)
    {
        uint64_t uop_cache_idx = (addr >> 5) % UOP_CACHE_NUM_SETS;
        uint64_t tag = (addr >> 5) / UOP_CACHE_NUM_SETS;
        // find the the base way for this 32B code region
        for (int way = 0; way < UOP_CACHE_NUM_WAYS; way++) {
            if ((uopValidArray[uop_cache_idx][way] && uopTagArray[uop_cache_idx][way] == tag)) {
                
                uopProfitableTrace[uop_cache_idx][way] = state;
            
            }
        }
    }

    bool isUopTraceProfitableForSuperOptimization(Addr addr)
    {
        uint64_t uop_cache_idx = (addr >> 5) % UOP_CACHE_NUM_SETS;;
        uint64_t tag = (addr >> 5) / UOP_CACHE_NUM_SETS;
        // find the the base way for this 32B code region
        for (int way = 0; way < UOP_CACHE_NUM_WAYS; way++) {
            if ((uopValidArray[uop_cache_idx][way] && 
                uopTagArray[uop_cache_idx][way] == tag &&
                uopProfitableTrace[uop_cache_idx][way] == false)) 
            {
                
                return false;
            
            }
        }

        return true;
    }

    BaseCPU *cpu;
    void setCPU(BaseCPU * newCPU, ThreadID tid=0);
   // vector<ArrayDependencyTracker::DependGraphEntry> victimCache;
   // vector<ExtMachInst> victimEMIs;

protected:
    Stats::Scalar uopCacheWayInvalidations;
    Stats::Scalar uopCacheUpdates;
    Stats::Scalar uopConflictMisses;
    Stats::Scalar uopCacheLRUUpdates;
    Stats::Scalar macroTo1MicroEncoding;
    Stats::Scalar macroTo2MicroEncoding;
    Stats::Scalar macroTo3MicroEncoding;
    Stats::Scalar macroTo4MicroEncoding;
    Stats::Scalar macroToROMMicroEncoding;
    Stats::Scalar hotnessLessThanSeven;
    Stats::Scalar hotnessGreaterThanSeven;


    uint8_t getNextByte()
    {
        return ((uint8_t *)&fetchChunk)[offset];
    }

    void getImmediate(int &collected, uint64_t &current, int size)
    {
        //Figure out how many bytes we still need to get for the
        //immediate.
        int toGet = size - collected;
        //Figure out how many bytes are left in our "buffer"
        int remaining = sizeof(MachInst) - offset;
        //Get as much as we need, up to the amount available.
        toGet = toGet > remaining ? remaining : toGet;

        //Shift the bytes we want to be all the way to the right
        uint64_t partialImm = fetchChunk >> (offset * 8);
        //Mask off what we don't want
        partialImm &= mask(toGet * 8);
        //Shift it over to overlay with our displacement.
        partialImm <<= (immediateCollected * 8);
        //Put it into our displacement
        current |= partialImm;
        //Update how many bytes we've collected.
        collected += toGet;
        consumeBytes(toGet);
    }

    void updateOffsetState()
    {
        assert(offset <= sizeof(MachInst));
        if (offset == sizeof(MachInst)) {
            DPRINTF(Decoder, "At the end of a chunk, idx = %d, chunks = %d.\n",
                    chunkIdx, instBytes->chunks.size());
            chunkIdx++;
            if (chunkIdx == instBytes->chunks.size()) {
                outOfBytes = true;
            } else {
                offset = 0;
                fetchChunk = instBytes->chunks[chunkIdx];
                basePC += sizeof(MachInst);
            }
        }
    }

    void consumeByte()
    {
        offset++;
        updateOffsetState();
    }

    void consumeBytes(int numBytes)
    {
        offset += numBytes;
        updateOffsetState();
    }

    //State machine state
  protected:
    //Whether or not we're out of bytes
    bool outOfBytes;
    //Whether we've completed generating an ExtMachInst
    bool instDone;
    //The size of the displacement value
    int displacementSize;
    //The size of the immediate value
    int immediateSize;
    //This is how much of any immediate value we've gotten. This is used
    //for both the actual immediate and the displacement.
    int immediateCollected;

    enum State {
        ResetState,
        FromCacheState,
        PrefixState,
        Vex2Of2State,
        Vex2Of3State,
        Vex3Of3State,
        VexOpcodeState,
        OneByteOpcodeState,
        TwoByteOpcodeState,
        ThreeByte0F38OpcodeState,
        ThreeByte0F3AOpcodeState,
        ModRMState,
        SIBState,
        DisplacementState,
        ImmediateState,
        //We should never get to this state. Getting here is an error.
        ErrorState
    };

    State state;

    //Functions to handle each of the states
    State doResetState();
    State doFromCacheState();
    State doPrefixState(uint8_t);
    State doVex2Of2State(uint8_t);
    State doVex2Of3State(uint8_t);
    State doVex3Of3State(uint8_t);
    State doVexOpcodeState(uint8_t);
    State doOneByteOpcodeState(uint8_t);
    State doTwoByteOpcodeState(uint8_t);
    State doThreeByte0F38OpcodeState(uint8_t);
    State doThreeByte0F3AOpcodeState(uint8_t);
    State doModRMState(uint8_t);
    State doSIBState(uint8_t);
    State doDisplacementState();
    State doImmediateState();

    //Process the actual opcode found earlier, using the supplied tables.
    State processOpcode(ByteTable &immTable, ByteTable &modrmTable,
                        bool addrSizedImm = false);
    // Process the opcode found with VEX / XOP prefix.
    State processExtendedOpcode(ByteTable &immTable);

  protected:
    /// Caching for decoded instruction objects.

    typedef MiscReg CacheKey;

    typedef DecodeCache::AddrMap<Decoder::InstBytes> DecodePages;
    DecodePages *decodePages;
    typedef std::unordered_map<CacheKey, DecodePages *> AddrCacheMap;
    AddrCacheMap addrCacheMap;

    // DecodeCache::InstMap<ExtMachInst> *instMap;
    // typedef std::unordered_map<
    //        CacheKey, DecodeCache::InstMap<ExtMachInst> *> InstCacheMap;
    // static InstCacheMap instCacheMap;

  public:
    Decoder(ISA* isa = nullptr, DerivO3CPUParams* params = nullptr);

    void setM5Reg(HandyM5Reg m5Reg)
    {
        mode = (X86Mode)(uint64_t)m5Reg.mode;
        submode = (X86SubMode)(uint64_t)m5Reg.submode;
        emi.mode.mode = mode;
        emi.mode.submode = submode;
        altOp = m5Reg.altOp;
        defOp = m5Reg.defOp;
        altAddr = m5Reg.altAddr;
        defAddr = m5Reg.defAddr;
        stack = m5Reg.stack;

        AddrCacheMap::iterator amIter = addrCacheMap.find(m5Reg);
        if (amIter != addrCacheMap.end()) {
            decodePages = amIter->second;
        } else {
            decodePages = new DecodePages;
            addrCacheMap[m5Reg] = decodePages;
        }

        // InstCacheMap::iterator imIter = instCacheMap.find(m5Reg);
        // if (imIter != instCacheMap.end()) {
            // instMap = imIter->second;
        // } else {
            // instMap = new DecodeCache::InstMap<ExtMachInst>;
            // instCacheMap[m5Reg] = instMap;
        // }
    }

    void takeOverFrom(Decoder *old)
    {
        mode = old->mode;
        submode = old->submode;
        emi.mode.mode = mode;
        emi.mode.submode = submode;
        altOp = old->altOp;
        defOp = old->defOp;
        altAddr = old->altAddr;
        defAddr = old->defAddr;
        stack = old->stack;
    }

    void reset()
    {
        state = ResetState;
    }

    void process();

    //Use this to give data to the decoder. This should be used
    //when there is control flow.
    void moreBytes(const PCState &pc, Addr fetchPC, MachInst data)
    {
        DPRINTF(Decoder, "Getting more bytes.\n");
        basePC = fetchPC;
        offset = (fetchPC >= pc.instAddr()) ? 0 : pc.instAddr() - fetchPC;
        fetchChunk = data;
        outOfBytes = false;
        process();
    }

    bool needMoreBytes()
    {
        return outOfBytes;
    }

    bool instReady()
    {
        return instDone;
    }

    void
    updateNPC(X86ISA::PCState &nextPC)
    {
        if (!nextPC.size()) {
            int size = basePC + offset - origPC;
            DPRINTF(Decoder,
                    "Calculating the instruction size: "
                    "basePC: %#x offset: %#x origPC: %#x size: %d\n",
                    basePC, offset, origPC, size);
            nextPC.size(size);
            nextPC.npc(nextPC.pc() + size);
        }
    }

  public:

    // Dependency Tracking Unit for Speculative Superoptimization
    // ArrayDependencyTracker* depTracker;
    TraceBasedGraph* traceConstructor;

    StaticInstPtr decodeInst(ExtMachInst mach_inst);

    /// Decode a machine instruction.
    /// @param mach_inst The binary instruction to decode.
    /// @retval A pointer to the corresponding StaticInst object.
    StaticInstPtr decode(ExtMachInst mach_inst, Addr addr);
    StaticInstPtr decode(X86ISA::PCState &nextPC, unsigned cycleAdded, ThreadID tid);
    bool isHitInUopCache(Addr addr);
    StaticInstPtr fetchUopFromUopCache(Addr addr, X86ISA::PCState &nextPC);
    bool updateUopInUopCache(ExtMachInst emi, Addr addr, int numUops, int size, unsigned cycleAdded, ThreadID tid);
    void updateLRUBits(int idx, int way);
    void setUopCacheActive(bool active)
    {
        // before deactiviting the uop cache, the spec cache should always have been deactivated,
        // otherwise there is a bug
        if (active == false) assert(speculativeCacheActive == false);

        uopCacheActive = active;
    }
    void resetDecoder() {instDone = false; state = ResetState;}
    void setUopCachePresent(bool present)
    {
        isUopCachePresent = present;
    }
    void setMicroFusionPresent(bool present)
    {
        isMicroFusionPresent = present;
    }
    void setSuperOptimizationPresent(bool present)
    {
        isSuperOptimizationPresent = present;
    }

    // Parallel cache for optimized micro-ops
    void updateLRUBitsSpeculative(int idx, int way);

    //*****CHANGE START**********
    void setSpeculativeCacheActive(bool active, uint64_t _currentTraceID = 0)
    {
        assert((active && _currentTraceID != 0) || (!active && _currentTraceID == 0));
        speculativeCacheActive = active;
        currentActiveTraceID = _currentTraceID;
    }
    //*****CHANGE END**********
    void setSpeculativeCachePresent(bool present)
    {
        isSpeculativeCachePresent = present;
    }

	// Interface for superoptimization, interacts with cache differently in gem5
	// In hardware wouldn't need a different interface, as both are streaming caches

	bool addUopToSpeculativeCache( SpecTrace &trace, SuperOptimizedMicroop superoptimized_microop);

    bool isSpeculativeCacheActive()
    {
      return speculativeCacheActive;
    }

    bool isUopCacheActive()
    {
      return uopCacheActive;
    }
    void doSquash(Addr addr);

	StaticInstPtr getSuperoptimizedInst(Addr addr, unsigned uop);

	void invalidateSpecTrace(Addr addr, unsigned uop);
    void invalidateSpecTrace(FullCacheIdx addr, uint64_t trace_id);

	void invalidateSpecCacheLine(int idx, int way);

	unsigned getHotnessOfTrace(Addr addr);

	unsigned minConfidence(uint64_t traceId);

	unsigned maxLatency(uint64_t traceId);

	// Interface for fetch!
    // tells fetch stage that if a speculative trace is availble for this PC
    // LVPredictor return int8_t confidence, if this confidence if less than zero then just return
	uint64_t isTraceAvailable(FullUopAddr addr);

    StaticInstPtr getSuperOptimizedMicroop(uint64_t traceID, X86ISA::PCState &thisPC, X86ISA::PCState &nextPC, bool &predict_taken);
    void updateStreamTrace(uint64_t traceID, X86ISA::PCState &thisPC);

    void regStats();

	struct TraceMetaData {
		/*
		 * "hotness" used as hotness in dependgraph implementation
		 * "hotness" used as trace ID in tracegeneration implementation
		 */ 
		unsigned hotness;
		unsigned minConfidence;
		unsigned maxLatency;

		TraceMetaData(unsigned h, unsigned c, unsigned m) {
			hotness = h;
			minConfidence = c;
			maxLatency = m;
		}
	};

//	TraceMetaData getTraceMetaData(Addr addr);
	TraceMetaData getTraceMetaData(FullCacheIdx specIdx, FullCacheIdx uopIdx);
	TraceMetaData getTraceMetaData(FullCacheIdx specIdx);
};

} // namespace X86ISA

#endif // __ARCH_X86_DECODER_HH__
