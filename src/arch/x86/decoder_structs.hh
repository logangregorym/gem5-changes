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
// tracks instructions with confident value predictions
struct PredictionSource
{
    FullUopAddr addr;
    bool valid;
    bool isBranch;
    uint64_t value;
    unsigned confidence;
    unsigned latency;

    PredictionSource() {valid = false; isBranch = false;}
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
    
    public:

    void setOptimizedTraceHead(FullCacheIdx _optimizedHead) {optimizedHead = _optimizedHead;}
    void setTraceHeadAddress(FullUopAddr _headAddr){ headAddr = _headAddr;}

    FullUopAddr getTraceHeadAddr() {return headAddr;}
    FullCacheIdx getOptimizedHead() {return optimizedHead;}
    


    // Trace ID
    uint64_t id;

    

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

    // previous non-eliminated instruction
    StaticInstPtr prevNonEliminatedInst;

    // address of the last instruction in the trace
    FullUopAddr end;
    FullUopAddr prevNonEliminatedEnd;

    // Number of branches folded
    unsigned branchesFolded;

    // Control Prediction Sources (at most 2)
    PredictionSource controlSources[2];

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

    // Prediction Sources (at most 4)
    PredictionSource source[4];

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
        branchesFolded = 0;
    }
};
#endif // __ARCH_X86_DECODER_STRUCTS__
