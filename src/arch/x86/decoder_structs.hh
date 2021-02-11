#ifndef __ARCH_X86_DECODER_STRUCTS__
#define __ARCH_X86_DECODER_STRUCTS__

#include "arch/x86/types.hh"
#include "base/types.hh"
#include "cpu/static_inst.hh"
#include "cpu/pred/big_sat_counter.hh"
#include "base/intmath.hh"
#include "debug/SpecCache.hh"
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
    X86ISA::PCState thisPC;
    X86ISA::PCState nextPC;

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
    X86ISA::PCState thisPC;
    X86ISA::PCState nextPC;

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
    public:
        typedef std::map<Addr, std::map<uint16_t, OriginalMicroop>>         OriginalTrace;
        typedef std::map<Addr, std::map<uint16_t, SuperOptimizedMicroop>>   SuperOptimizedTrace;
    
    private:
        // address of the head of the original trace
        FullUopAddr origHeadAddr;
        // address of the last instruction in the original trace
        FullUopAddr origEndAddr;
    
        // Trace ID
        uint64_t traceID;

        // address of the head of the original trace
        FullUopAddr superHeadAddr;
        // address of the last instruction in the original trace
        FullUopAddr superEndAddr;



    public:
        void setOrigHeadAndEndAddr(FullUopAddr _origHeadAddr, FullUopAddr _origEndAddr) {origHeadAddr = _origHeadAddr; origEndAddr = _origEndAddr;}
        void setSuperHeadAndEndAddr(FullUopAddr _superHeadAddr, FullUopAddr _superEndAddr) {superHeadAddr = _superHeadAddr; superEndAddr = _superEndAddr;}
        FullUopAddr getOrigHeadAddr() {return origHeadAddr;}
        FullUopAddr getOrigEndAddr() {return origEndAddr;}
        FullUopAddr getSuperHeadAddr() {return superHeadAddr;}
        FullUopAddr getSuperEndAddr() {return superEndAddr;}

        void setTraceID (uint64_t _id) {traceID = _id;}
        uint64_t getTraceID() {return traceID;}

        

    public:

        const static int NUM_PREDICTION_SOURCES  = 8;

        // address of the instruction being optimized
        FullUopAddr instGettingOptimizedAddr;

        // address of the instruction being optimized
        FullUopAddr instGettingStreamedAddr;
        
        // previous non-eliminated instruction
        StaticInstPtr prevNonEliminatedInst;



        // (idx, way, uop) of head of the trace
        FullCacheIdx head;

        // (idx, way, uop) of head of the optimized trace
        FullCacheIdx optimizedHead;

        // (idx, way, uop) of the instruction being optimized
        FullCacheIdx addr;


        // address of the last instruction processed
        FullUopAddr lastAddr;

        // instruction being optimized
        StaticInstPtr inst;





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
            Rejected,
            Complete
        };

        // Trace Satte
        State state;

        // Prediction Sources (at most 4)
        PredictionSource source[NUM_PREDICTION_SOURCES];

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
            instGettingOptimizedAddr = FullUopAddr(0,0);
            instGettingStreamedAddr  = FullUopAddr(0,0);
            traceID = 0;
            //id = 0;
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
            superOptimizedTrace.clear();
        }

        ~SpecTrace() {
            instGettingOptimizedAddr = FullUopAddr(0,0);
            instGettingStreamedAddr  = FullUopAddr(0,0);
            traceID = 0;
            //id = 0;
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

            superOptimizedTrace.clear();
            originalTrace.clear();
        }

        void reset()
        {
            instGettingOptimizedAddr = FullUopAddr(0,0);
            instGettingStreamedAddr  = FullUopAddr(0,0);
            traceID = 0;
            //id = 0;
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
            originalTrace.clear();
            superOptimizedTrace.clear();
        }
};



class SpeculativeUopCache 
{
    private:
        
        struct SpecCacheEntity
        {
            public:
                uint64_t hotness;

                SpecCacheEntity()
                {
                    this->hotness = 0;
                    this->traceHeadAddr = 0;
                    this->numOfMicroopsInTrace = 0;

                }

                SpecCacheEntity(uint64_t _traceHeadAddr, uint64_t _hotness, uint64_t _numOfMicroopsInTrace)
                {
                    this->hotness = _hotness;
                    this->traceHeadAddr = _traceHeadAddr;
                    this->numOfMicroopsInTrace = _numOfMicroopsInTrace;
                }

            public:
                uint64_t traceHeadAddr;
                uint64_t numOfMicroopsInTrace;
        };
                          // TraceID             NumWaysOccupied     hotness, TraceHeadAddr, numOfMicroopsInTrace
        typedef std::map <uint64_t,     std::pair<uint64_t,          SpecCacheEntity       >> SpecCacheSets;

        


        uint64_t NumSets;
        uint64_t NumWays;
        uint64_t SetBitsMask;
        SpecCacheSets * specCacheSets;
        uint64_t NumOfValidTracesInSpecCache;
        uint64_t NumOfEvictedTraces;
        uint64_t currentActiveTraceID;
        bool speculativeCacheActive;

    public:

        SpeculativeUopCache(uint64_t _sets, uint64_t _ways )
        {
            assert(isPowerOf2(_sets));
            assert(isPowerOf2(_ways));
            NumSets = _sets;
            NumWays = _ways;
            SetBitsMask = _sets - 1;
            NumOfValidTracesInSpecCache = 0;
            NumOfEvictedTraces = 0;

            DPRINTF(SpecCache, "Creating a Spec Cache! Number of Sets: %d, Number of Ways: %d\n", NumSets, NumWays);
            specCacheSets = new SpecCacheSets[_sets];
            for (size_t i = 0; i < _sets; i++)
            {
                specCacheSets[i].clear();
            }

            currentActiveTraceID = 0;
            speculativeCacheActive = false;
            

        }

        bool isSpeculativeCacheActive()
        {
            return speculativeCacheActive;
        }

        void setSpeculativeCacheActive(bool _active, uint64_t _currentTraceID = 0)
        {

            speculativeCacheActive = _active;
            currentActiveTraceID = _currentTraceID;
        }

        uint64_t getCurrentActiveTraceID() {return currentActiveTraceID; }

        void isHitInSpecCache(Addr _traceHeadAddr, std::vector<uint64_t>& _traces)
        {

            
            uint64_t setIdx = (_traceHeadAddr >> 5) & (SetBitsMask);
            assert(setIdx < NumSets);
            
            for (auto & s : specCacheSets[setIdx])
            {
                if (s.second.second.traceHeadAddr == _traceHeadAddr)
                {
                    _traces.push_back(s.first);
                    s.second.second.hotness++;
                }
            }

        }

        bool AddToSpeculativeCache(Addr _traceHeadAddr, uint64_t _traceID, uint64_t _numOfMicroops, std::vector<uint64_t>& _evictedTraces)
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
                specCacheSets[setIdx][_traceID] = std::make_pair(numWaysToAccomodate, SpecCacheEntity(_traceHeadAddr, 0, _numOfMicroops));
                assert (NumOfValidTracesInSpecCache <= (NumSets * NumWays));
                NumOfValidTracesInSpecCache++;
                DPRINTF(SpecCache, "Adding trace %d!  Set#[%d] NumOfValidTracesInSpecCache[%d] NumWaysToAccomodate[%d] NumOccupiedWays[%d].\n", _traceID, setIdx, NumOfValidTracesInSpecCache,  numWaysToAccomodate, numOccupiedWays);
                return true;
            }
            else 
            {
                bool replaced = false;
                do {
                    assert(!specCacheSets[setIdx].empty());
                    // there is not enoght space for this trace
                    // try to remove the trace with least accesses and check to see if removing it can help to accomodate the new trace
                    uint64_t _trace_id = 0;
                    uint64_t _waysOccupied = 0;
                    uint64_t _hotness = UINT64_MAX;
                    for (auto const & s: specCacheSets[setIdx])
                    {
                        // do not remove a streaming trace 
                        if (s.first == currentActiveTraceID) 
                        {
                            assert(currentActiveTraceID);
                            continue;
                        }    

                        if (s.second.second.hotness < _hotness)
                        {
                            _trace_id = s.first;
                            _waysOccupied = s.second.first;
                        }
                    }

                    assert(_trace_id);
                    assert(_waysOccupied);
                    // remove the LRU trace
                    
                    specCacheSets[setIdx].erase(_trace_id);
                    assert(NumOfValidTracesInSpecCache >= 1);
                    NumOfValidTracesInSpecCache--; 
                    NumOfEvictedTraces++;
                    _evictedTraces.push_back(_trace_id);
                    numOccupiedWays -= _waysOccupied;
                    assert(numOccupiedWays >= 1);
                    DPRINTF(SpecCache, "Removing trace %d! Set#[%d] NumOfValidTracesInSpecCache[%d] WaysOccupied[%d] NumOccupiedWays[%d]\n", _trace_id, setIdx, NumOfValidTracesInSpecCache,  _waysOccupied, numOccupiedWays);
                    if (numWaysToAccomodate + numOccupiedWays <= NumWays)
                    {
                        specCacheSets[setIdx][_traceID] = std::make_pair(numWaysToAccomodate, SpecCacheEntity(_traceHeadAddr, 0, _numOfMicroops));
                        assert (NumOfValidTracesInSpecCache <= (NumSets * NumWays));
                        NumOfValidTracesInSpecCache++;
                        replaced = true; /* this is for future replcement algorithms */
                        DPRINTF(SpecCache, "Adding trace %d!  Set#[%d] NumOfValidTracesInSpecCache[%d] NumWaysToAccomodate[%d] NumOccupiedWays[%d].\n", _traceID, setIdx, NumOfValidTracesInSpecCache,  numWaysToAccomodate, numOccupiedWays);
                        return true;
                    }

                } while(!replaced);

                assert(0);
                return replaced;

            }

        }

        void DumpSpecCache()
        {
            for (size_t setIdx = 0; setIdx < NumSets; setIdx++)
            {
                if (!specCacheSets[setIdx].empty())
                {
                    
                    std::string set_string = "";
                    for (auto const& s : specCacheSets[setIdx])
                    {
                        std::stringstream stream;
                        stream << "{" << s.first << "," << s.second.first << "," << std::hex << s.second.second.traceHeadAddr << std::dec << "," << s.second.second.hotness << "} - ";
                        set_string += stream.str();
                    }
                 
                    DPRINTF(SpecCache, "SET[%d]: %s\n", setIdx, set_string);
                }
                
            }
        }
        void DumpSpecCacheToStdOut()
        {
            for (size_t setIdx = 0; setIdx < NumSets; setIdx++)
            {
                if (!specCacheSets[setIdx].empty())
                {
                    
                    std::string set_string = "";
                    for (auto const& s : specCacheSets[setIdx])
                    {
                        std::stringstream stream;
                        stream << "{" << s.first << "," << s.second.first << "," << std::hex << s.second.second.traceHeadAddr << std::dec << "," << s.second.second.hotness << "} - ";
                        set_string += stream.str();
                    }
                 
                    
                    std::cout << std::dec << "SET[" << setIdx << "]: " << set_string << "\n";
                }
                
            }
        }

        uint64_t GetNumOfValidTracesInSpecCache() {return NumOfValidTracesInSpecCache;}
        uint64_t GetNumOfEvictedTraces() {return NumOfEvictedTraces;}

};
#endif // __ARCH_X86_DECODER_STRUCTS__
