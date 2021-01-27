#ifndef __ARCH_X86_DECODER_STRUCTS__
#define __ARCH_X86_DECODER_STRUCTS__

#include "arch/x86/types.hh"
#include "base/types.hh"
#include "cpu/static_inst.hh"
#include "cpu/pred/big_sat_counter.hh"
#include "base/intmath.hh"
#include <cmath>

struct FullUopAddr {
	Addr pcAddr;
	uint16_t uopAddr;

	FullUopAddr(Addr p, uint16_t u) {
		pcAddr = p;
		uopAddr = u;
	}

	FullUopAddr() {
		pcAddr = 0;
		uopAddr = 0;
	}

	bool operator==(const FullUopAddr& rhs) {
		return (pcAddr == rhs.pcAddr) && (uopAddr == rhs.uopAddr);
	}

	bool operator!=(const FullUopAddr& rhs) {
		return (pcAddr != rhs.pcAddr) || (uopAddr != rhs.uopAddr);
	}
};

struct FullCacheIdx {
	int idx;
	int way;
	int uop;
	bool valid;

	FullCacheIdx(int i, int w, int u) {
		idx = i;
		way = w;
		uop = u;
		valid = true;
	}

	FullCacheIdx() {
		idx = 0;
		way = 0;
		uop = 0;
		valid = false;
	}

	bool operator==(const FullCacheIdx& rhs) {
		return (idx == rhs.idx) && (way == rhs.way) && (uop == rhs.uop);
	}

	bool operator!=(const FullCacheIdx& rhs) {
		return (idx != rhs.idx) || (way != rhs.way) || (uop != rhs.uop);
	}
};


// tracks instructions with confident value predictions
struct PredictionSource
{
    FullUopAddr addr;
    bool valid;
    bool isBranch;
    uint64_t value;
    uint64_t confidence;
    uint64_t latency;

    PredictionSource() {valid = false; isBranch = false;}
};

// Register context block for liveness analysis
struct RegisterContext {
    uint64_t value;
    bool valid;
    bool source;

    RegisterContext() {value = 0; valid = false; source = false;}
};

struct OriginalMicroop {
    StaticInstPtr microop; 
    StaticInstPtr macroop;
    Addr tag;
    Addr idx;
    Addr way;

    OriginalMicroop ()
    {
        this->microop = NULL;
        this->macroop = NULL;
        this->tag = 0;
        this->idx = 0;
        this->way = 0;
       
    };

    OriginalMicroop (StaticInstPtr _microop, StaticInstPtr _macroop, Addr _tag, Addr _idx, Addr _way)
    {
        this->microop = _microop;
        this->macroop = _macroop;
        this->tag = _tag;
        this->idx = _idx;
        this->way = _way;
    
    };
};

struct SuperOptimizedMicroop {
    StaticInstPtr microop; 
    StaticInstPtr macroop;
    Addr tag;
    Addr idx;
    Addr way;
    bool compacted;

    SuperOptimizedMicroop ()
    {
        this->microop = NULL;
        this->macroop = NULL;
        this->tag = 0;
        this->idx = 0;
        this->way = 0;
        this->compacted = false; // is the microop is compacted?
    };

    SuperOptimizedMicroop (StaticInstPtr _microop, StaticInstPtr _macroop, Addr _tag, Addr _idx, Addr _way, bool _compacted)
    {
        this->microop = _microop;
        this->macroop = _macroop;
        this->tag = _tag;
        this->idx = _idx;
        this->way = _way;
        this->compacted = _compacted;
    };
};

// Speculative Trace
struct SpecTrace
{

    typedef std::map<Addr, std::map<uint16_t, OriginalMicroop>>         OriginalTrace;
    typedef std::map<Addr, std::map<uint16_t, SuperOptimizedMicroop>>   SuperOptimizedTrace;
    // Trace ID
    uint64_t id;

    // address of the head of the original trace
    FullUopAddr headAddr;
    // address of the last instruction in the original trace
    FullUopAddr endAddr;

    // address of the head of the original trace
    FullUopAddr superHeadAddr;
    // address of the last instruction in the original trace
    FullUopAddr superEndAddr;

    // head address of trace 
    Addr traceHeadAddr;

    // end address of trace
    Addr traceEndAddr;

    // head address of trace 
    Addr superTraceHeadAddr;

    // end address of trace
    Addr superTraceEndAddr;

    // (idx, way, uop) of head of the trace
    FullCacheIdx head;

    // (idx, way, uop) of head of the optimized trace
    FullCacheIdx optimizedHead;

    // (idx, way, uop) of the instruction being optimized
    FullCacheIdx addr;

    // address of the instruction being optimized
    FullUopAddr instAddr;

    // address of the last instruction processed
    FullUopAddr lastAddr;

    // instruction being optimized
    StaticInstPtr inst;

    // previous non-eliminated instruction
    StaticInstPtr prevNonEliminatedInst;



    // Number of branches folded
    unsigned branchesFolded;

    // Control Prediction Sources (at most 2)
    PredictionSource controlSources[2];

    // Originial Trace in Uop/Spec cache
    OriginalTrace originalTrace;

    // Ways in Uop/Spec cache that holds the original trace
    //std::vector<uint64_t> originalTraceCacheWays;

    // SuperOptimized Trace
    SuperOptimizedTrace superOptimizedTrace;

    // Ways in Uop/Spec cache that holds the super optimized trace
    //std::vector<uint64_t> superOptmizedTraceCacheWays;

    enum State {
        Invalid,
        
        QueuedForFirstTimeOptimization,
        QueuedForReoptimization,

        // first time optimization
        OptimizationInProcess,

        // re-optimization (e.g., due to a new pred source)
        ReoptimizationInProcess,

        // evicted before we could process it
        Evicted,

        Complete
    };

    // Trace Satte
    State state;

    // Prediction Sources (at most 8)
    PredictionSource source[8];

    // Trace Length
    uint64_t length;

    // Trace Hotness
    uint64_t hotness;

    // Shrunk length
    uint64_t shrunkLength;

    // ID of the trace being re-optimized in case this is a re-optimization
    unsigned int reoptId;

    // Counter to assign trace IDs
    static uint64_t traceIDCounter;

    SpecTrace() {

        id = 0;
        reoptId = 0;
        state = Invalid;
        length = 0;
        shrunkLength = 0;
        head = FullCacheIdx();
        addr = FullCacheIdx();
        inst = NULL;
        prevNonEliminatedInst = NULL;
        branchesFolded = 0;
        originalTrace.clear();
        hotness = 0;
        traceHeadAddr = 0;
        traceEndAddr = 0;
        superOptimizedTrace.clear();


    }

    ~SpecTrace() {

        id = 0;
        reoptId = 0;
        state = Invalid;
        length = 0;
        shrunkLength = 0;
        head = FullCacheIdx();
        addr = FullCacheIdx();
        inst = NULL;
        prevNonEliminatedInst = NULL;
        branchesFolded = 0;
        
        hotness = 0;
        traceHeadAddr = 0;
        traceEndAddr = 0;

        superOptimizedTrace.clear();
        originalTrace.clear();
    }

    void reset()
    {
        id = 0;
        reoptId = 0;
        state = Invalid;
        length = 0;
        shrunkLength = 0;
        head = FullCacheIdx();
        addr = FullCacheIdx();
        inst = NULL;
        prevNonEliminatedInst = NULL;
        branchesFolded = 0;
        hotness = 0;
        traceHeadAddr = 0;
        traceEndAddr = 0;        
        originalTrace.clear();
        superOptimizedTrace.clear();
    }
};



class SpeculativeUopCache 
{
    private:

        struct SpecCacheEntity
        {
            uint64_t LRU;
            uint64_t traceHeadAddr;

            SpecCacheEntity()
            {
                this->LRU = 0;
                this->traceHeadAddr = 0;
            }

            SpecCacheEntity(uint64_t _traceHeadAddr, uint64_t _lru)
            {
                this->LRU = _lru;
                this->traceHeadAddr = _traceHeadAddr;
            }
        };
                          // TraceID             NumWaysOccupied     LRU, TraceHeadAddr
        typedef std::map <uint64_t,     std::pair<uint64_t,          SpecCacheEntity       >> SpecCacheSets;

        


        uint64_t NumSets;
        uint64_t NumWays;
        uint64_t SetBitsMask;
        SpecCacheSets * specCacheSets;

    public:

        SpeculativeUopCache(uint64_t _sets, uint64_t _ways )
        {
            assert(isPowerOf2(_sets));
            assert(isPowerOf2(_ways));
            NumSets = _sets;
            NumWays = _ways;
            SetBitsMask = _sets - 1;

            specCacheSets = new SpecCacheSets[_sets];
            for (size_t i = 0; i < _sets; i++)
            {
                specCacheSets[i].clear();
            }
            

        }

        void findAllTracesInSpeculativeCache(Addr _traceHeadAddr, std::vector<uint64_t>& _traces)
        {

            uint64_t setIdx = (_traceHeadAddr >> 5) & (SetBitsMask);
            assert(setIdx < NumSets);
            
            for (auto const& s : specCacheSets[setIdx])
            {
                if (s.second.second.traceHeadAddr == _traceHeadAddr)
                {
                    _traces.push_back(s.first);
                    s.second.second.LRU++;
                }
            }

        }

        bool addToSpeculativeCache(Addr _traceHeadAddr, uint64_t _traceID, uint64_t _numOfMicroops, std::vector<uint64_t>& _evictedTraces)
        {
            assert(_numOfMicroops <= 18 && _numOfMicroops > 0);
            uint64_t numWaysToAccomodate = (_numOfMicroops <= 6) ? 1 : (_numOfMicroops <= 12 ? 2 : 3);

            uint64_t setIdx = (_traceHeadAddr >> 5) & (SetBitsMask);
            assert(setIdx < NumSets);

            // check whether there is a free space for this trace

            uint64_t numOccupiedWays = 0;
            for (auto const &s : specCacheSets[setIdx])
            {
                assert(_traceID != s.first);
                numOccupiedWays += s.second.first; // this is the NumWaysOccupied for the trace
            }

            // if we have enough space to insert the trace, just do it!
            if (numWaysToAccomodate + numOccupiedWays <= NumWays)
            {
                specCacheSets[setIdx][_traceID] = std::make_pair(numWaysToAccomodate, SpecCacheEntity(_traceHeadAddr, 0));
                return true;
            }
            else 
            {
                bool replaced = false;
                do {
                    assert(!specCacheSets[setIdx].empty());
                    // there is not enoght space for this trace
                    // try to remove the trace with least accesses and check to see if removing it can help to accomodate the new trace
                    uint64_t _trace_id = specCacheSets[setIdx].begin()->first;
                    uint64_t _waysOccupied = specCacheSets[setIdx].begin()->second.first;
                    uint64_t _lru = specCacheSets[setIdx].begin()->second.second.LRU;
                    for (auto const & s: specCacheSets[setIdx])
                    {
                        if (s.second.second.LRU < _lru)
                        {
                            _trace_id = s.first;
                            _waysOccupied = s.second.first;
                        }
                    }

                    // remove the LRU trace
                    specCacheSets[setIdx].erase(_trace_id);
                    _evictedTraces.push_back(_trace_id);
                    numOccupiedWays -= _waysOccupied;
                    assert(numOccupiedWays >= 1);
                    if (numWaysToAccomodate + numOccupiedWays <= NumWays)
                    {
                        specCacheSets[setIdx][_traceID] = std::make_pair(numWaysToAccomodate, SpecCacheEntity(_traceHeadAddr,0));
                        replaced = true; /* this is for future replcement algorithms */
                        return true;
                    }

                } while(!replaced);

                assert(0);
                return replaced;

            }

        }

};
#endif // __ARCH_X86_DECODER_STRUCTS__
