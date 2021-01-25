#ifndef __ARCH_X86_DECODER_STRUCTS__
#define __ARCH_X86_DECODER_STRUCTS__

#include "arch/x86/types.hh"
#include "base/types.hh"
#include "cpu/static_inst.hh"

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
    Addr tag;
    Addr idx;
    Addr way;

    OriginalMicroop ()
    {
        this->microop = NULL;
        tag = 0;
        idx = 0;
        way = 0;
       
    };

    OriginalMicroop (StaticInstPtr _microop, Addr _tag, Addr _idx, Addr _way)
    {
        this->microop = _microop;
        tag = _tag;
        idx = _idx;
        way = _way;
    
    };
};

struct SuperOptimizedMicroop {
    StaticInstPtr microop; 
    Addr tag;
    Addr idx;
    Addr way;
    bool compacted;

    SuperOptimizedMicroop ()
    {
        this->microop = NULL;
        tag = 0;
        idx = 0;
        way = 0;
        compacted = false; // is the microop is compacted?
    };

    SuperOptimizedMicroop (StaticInstPtr _microop, Addr _tag, Addr _idx, Addr _way, bool _compacted)
    {
        this->microop = _microop;
        tag = _tag;
        idx = _idx;
        way = _way;
        compacted = _compacted;
    };
};

// Speculative Trace
struct SpecTrace
{

    typedef std::map<Addr, std::map<uint16_t, OriginalMicroop>>         OriginalTrace;
    typedef std::map<Addr, std::map<uint16_t, SuperOptimizedMicroop>>   SuperOptimizedTrace;
    // Trace ID
    uint64_t id;

    // address of the head of the trace
    FullUopAddr headAddr;
    // address of the last instruction in the trace
    FullUopAddr endAddr;
    
    // head address of trace 
    Addr traceHeadAddr;

    // end address of trace
    Addr traceEndAddr;


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
        originalTrace.clear();
        hotness = 0;
        traceHeadAddr = 0;
        traceEndAddr = 0;
        superOptimizedTrace.clear();
    }
};
#endif // __ARCH_X86_DECODER_STRUCTS__
