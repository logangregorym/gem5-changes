#ifndef __ARCH_X86_DECODER_STRUCTS__
#define __ARCH_X86_DECODER_STRUCTS__

#include "arch/x86/types.hh"
#include "base/types.hh"
#include "cpu/static_inst.hh"





struct FullUopAddr {
	Addr pcAddr;
	unsigned uopAddr;

	FullUopAddr(Addr p, unsigned u) {
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

struct SuperOptimizedMicroop
{
    StaticInstPtr inst;
    FullUopAddr   instAddr;

    SuperOptimizedMicroop(StaticInstPtr _inst, FullUopAddr   _instAddr)
    {
        this->inst = _inst;
        this->instAddr = _instAddr;
    }

};

struct SpecCacheHistory
{
    uint64_t seqNum;
    uint64_t traceID;

    SpecCacheHistory(uint64_t _seqNum, uint64_t _traceID)
    {
        this->seqNum = _seqNum;
        this->traceID = _traceID; 
    }

};
// tracks instructions with confident value predictions
struct PredictionSource
{
    FullUopAddr addr;
    bool valid;
    bool isBranch;
    uint64_t value;
    unsigned confidence;
    unsigned latency;
    uint64_t numOfTimesMisspredicted;

    PredictionSource() {valid = false; isBranch = false; numOfTimesMisspredicted = 0;}
};

// Register context block for liveness analysis
struct RegisterContext {
    uint64_t value;
    bool valid;
    bool source;

    RegisterContext() {value = 0; valid = false; source = false;}
};

// Speculative Trace
struct SpecTrace
{
    private:
        // address of the head of the trace
        FullUopAddr headAddr;
        
        // (idx, way, uop) of head of the optimized trace
        FullCacheIdx optimizedHead;

        // whether an inst from this trace is in any stage of cpu
        bool inTransit;
    
    public:

    void setOptimizedTraceHead(FullCacheIdx _optimizedHead) {optimizedHead = _optimizedHead;}
    void setTraceHeadAddress(FullUopAddr _headAddr){ headAddr = _headAddr;}

    FullUopAddr getTraceHeadAddr() {return headAddr;}
    FullCacheIdx getOptimizedHead() {return optimizedHead;}

    bool isInTransit() {return inTransit;}
    void setInTransit(bool state) {inTransit = state;}
    


    // Trace ID
    uint64_t id;

    uint64_t hotness;

    // the tick at which this trace is inserted into the spec cache
    uint64_t insertion_tick;

    // the tick at which the trace is evicted from the spec cache
    uint64_t eviction_tick;

    // (idx, way, uop) of head of the trace
    FullCacheIdx head;


    // (idx, way, uop) of the instruction being optimized
    FullCacheIdx addr;

    // address of the instruction being optimized
    FullUopAddr instAddr;

    // address of the last instruction processed
    FullUopAddr lastAddr;

    // instruction being optimized
    StaticInstPtr inst;

    // previous non-eliminated and eliminated instructions
    StaticInstPtr prevNonEliminatedInst;
    StaticInstPtr prevEliminatedInst;

    // nextPC of the last instruction in the trace
    FullUopAddr end;

    // Number of branches folded
    unsigned branchesFolded;

    // Control Prediction Sources (at most 2)
    PredictionSource controlSources[2];
    // Total number of times trace is misspredicted (controlSources[2])
    uint64_t totalNumOfTimesControlSourcesAreMisspredicted;

    // Total number of microops commited from this trace
    uint64_t totalNumOfMicroopsCommitedFromTrace;

    // Total number of microops fetched from this trace
    uint64_t totalNumOfMicroopsFetchedFromTrace;

    enum State {
        Invalid,
        
        QueuedForFirstTimeOptimization,
        //QueuedForReoptimization,

        // first time optimization
        OptimizationInProcess,

        // re-optimization (e.g., due to a new pred source)
        //ReoptimizationInProcess,

        // evicted before we could process it
        Evicted,

        Complete
    };

    // Trace Satte
    State state;

    // number of valid prediction sources
    unsigned validPredSources ;

    // Prediction Sources (at most 4)
    PredictionSource source[4];
    // Total number of times trace is misspredicted (source[4])
    uint64_t totalNumOfTimesPredictionSourcesAreMisspredicted;
    // Trace Length
    unsigned length;

    // Shrunk length
    unsigned shrunkLength;

    // Intervening dead instructions
    unsigned interveningDeadInsts;

    // Counter to assign trace IDs
    static uint64_t traceIDCounter;

    SpecTrace() {
        id = 0;
        state = Invalid;
        length = 0;
        shrunkLength = 0;
        interveningDeadInsts = 0;
        head = FullCacheIdx();
        addr = FullCacheIdx();
        inst = NULL;
        prevNonEliminatedInst = NULL;
        prevEliminatedInst = NULL;
        branchesFolded = 0;
        totalNumOfTimesControlSourcesAreMisspredicted = 0;
        totalNumOfTimesPredictionSourcesAreMisspredicted = 0;
        hotness = 0;
        insertion_tick = 0;
        eviction_tick = 0;
        validPredSources = 0;
        inTransit= false;
        totalNumOfMicroopsCommitedFromTrace = 0;
        totalNumOfMicroopsFetchedFromTrace = 0;
    }
};
#endif // __ARCH_X86_DECODER_STRUCTS__
