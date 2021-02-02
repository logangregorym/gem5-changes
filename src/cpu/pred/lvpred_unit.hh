/*
 * Will have to ask what I'm supposed to  put here
 */

#ifndef __CPU_PRED_LVPRED_UNIT_HH__
#define __CPU_PRED_LVPRED_UNIT_HH__

#include <deque> // is this the best structure?
#include <list>

#include "base/statistics.hh"
#include "base/types.hh"
// #include "cpu/base_dyn_inst.hh"
//#include "cpu/base_dyn_inst.hh"
//#include "cpu/o3/dyn_inst.hh"
//#include "cpu/o3/cpu.hh"
#include "cpu/base.hh"
#include "cpu/pred/big_sat_counter.hh"
#include "cpu/static_inst.hh"
#include "mem/packet.hh"
#include "params/LoadValuePredictor.hh"
#include "sim/sim_object.hh"

/**
 * Base of load value predictor, specific implementations inherit from this
 */
class LVPredUnit : public SimObject
{
  public:
    typedef LoadValuePredictorParams Params;

    std::string lvpredType = "";
    
    /**
     * Constructor for LVPredUnit base
     * @param params The params object with size information
     */
    LVPredUnit(const Params *p);


    /** Registers statistics. */
    void regStats();

    virtual unsigned getConfidence(TheISA::PCState pc) { return 0; }

    virtual unsigned getDelay(TheISA::PCState pc) { return 0; }

    virtual void updateGtables(Addr pc, Addr next_pc, bool branches) { };

    struct lvpReturnValues {
        lvpReturnValues(int64_t v, int8_t status, uint8_t predSource) {
            this->predictedValue = v;
            this->confidence = status;
            this->latency = 0;
            this->predSource = predSource;
        }

        lvpReturnValues(int64_t v, int8_t status) {
            this->predictedValue = v;
            this->confidence = status;
            this->latency = 0;
            this->predSource = 0;
        }

        lvpReturnValues() {
            this->predictedValue = 0;
            this->confidence = 0;
            this->latency = 0;
            this->predSource = 0;
        }

        uint64_t predictedValue;
        int8_t confidence;
        unsigned latency;
        uint8_t predSource;

	// eves specific stuff ahead
        bool predVtage = false;
        bool predStride = false;
        bool prediction_result;  // true iff the prediction was correct
	uint64_t GTAG[9] = {0};
        uint64_t GI[9] = {0};  // these are snapshots of the VTAGE components
	int TAGSTR[3] = {0};  // TODO: allow multiple sizes, but for now this is fine
        int B[3] = {0};
	int STHIT;
        int HitBank;
    };

    uint64_t stored_seq_no;
    int stored_inflight;  // these are calculated externally and passed in immediately before use

    /**
     * Does lookUpLVPT and lookUpLCT and stores results in instruction itself
     */
    virtual lvpReturnValues makePrediction(TheISA::PCState pc, ThreadID tid, unsigned currentCycle) = 0;
    virtual bool makePredictionForTraceGenStage(Addr addr, uint16_t upc, ThreadID tid , lvpReturnValues& ret){ assert(0);}
    // overload for eves
    //virtual lvpReturnValues makePrediction(TheISA::PCState pc, ThreadID tid, unsigned currentcycle, std::list<DynInstPtr>* cpuInsts) = 0;


    virtual uint64_t getValuePredicted(TheISA::PCState pc) { panic("getValuePredicted not implemented for this type of value predictor\n"); }

    /**
     * Called when a value is returned from memory.
     * cyclesElapsed = memoryAccessStartCycle - memoryAccessEndCycle
     */
    virtual bool processPacketRecieved(TheISA::PCState pc, StaticInstPtr inst, uint64_t value, ThreadID tid, uint64_t predictedValue, int8_t confidence, unsigned cyclesElapsed, unsigned currentCycle) = 0;
    int8_t firstConst;

    bool dynamicThreshold;

    unsigned decrementBy;

    unsigned resetTo;

    unsigned missThreshold;

    unsigned hitThreshold;

    unsigned lastMisprediction = 0;

    unsigned resetDelay;

    uint8_t predictStage;

    bool predictingArithmetic;

  protected:

    /**
     * Called by constructor
     * Will differ depending on predictor type
     */
    virtual void setUpTables(const Params *p) = 0;

    struct ValuePredWithConstStatus {
        ValuePredWithConstStatus(uint64_t value, BigSatCounter status) {
            this->value = value;
            this->status = status;
        }

        ValuePredWithConstStatus() {
            this->value = 0;
            this->status = BigSatCounter(2, 0);
        }

        uint64_t value;
        BigSatCounter status;
    };

    uint32_t numThreads;

    public:
    /** Stat for number of load value lookups. */
    Stats::Scalar lvLookups;
    
    protected:
    /** Stat for number of correct predictions used. */
    Stats::Scalar correctUsed;
    /** Stat for number of incorrect predictions used. */
    Stats::Scalar incorrectUsed;
    /** Stat for number of predictable values not predicted. */
    Stats::Scalar correctNotUsed;
    /** Stat for number of unpredictable values not predicted. */
    Stats::Scalar incorrectNotUsed;
    /** Stat for total predictable values. */
    Stats::Formula totalCorrect;
    /** Stat for total unpredictable values. */
    Stats::Formula totalIncorrect;
    /** Stat for total predictions used. */
    Stats::Formula totalUsed;
    /** Stat for total predictions not used. */
    Stats::Formula totalNotUsed;
    /** Stat for prediction accuracy. */
    Stats::Formula accuracy;
    /** Stat for prediction coverage. */
    Stats::Formula coverage;
    /** Stat for number of memory access cycles saved. */
    Stats::Scalar cyclesSaved;
    /** Stat for number of rip relative loads. */
    Stats::Scalar ripRelNum;
    /** Stat for percentage rip relative loads. */
    Stats::Formula ripRelPercent;
    /** Stat for number of load values larger than 64 bits (8 bytes). */
    Stats::Scalar largeValueLoads;
    /** Stat for number of hybrid predictions that come from the basic predictor. */
    Stats::Scalar basicChosen;
    /** Stat for number of hybrid predictions that come from the strideHist predictor. */
    Stats::Scalar strideHistChosen;
    /** Stat for number of hybrid predictions that come from the 3periodic predictor. */
    Stats::Scalar periodicChosen;
    /** Stat for number of times firstConst is incremented. */
    Stats::Scalar constIncrement;
    /** Stat for number of times firstConst is decremented. */
    Stats::Scalar constDecrement;
    /** Stat for number of times a load address indexed to a different tag. */
    Stats::Scalar tagMismatch;
    /** Stat for the percentage of loads that collide with another address. */
    Stats::Formula aliasPercent;
    /** Stat for the percentage of loads that we make a prediction for. */
    Stats::Formula predictedPercent;
    /** Stat for predictions made. */
    Stats::Scalar predictionsMade;
	
	// Stats for comparing isLoad() and isInteger() insts
	Stats::Scalar finishedLoads;
	Stats::Scalar totalLoadLatency;
	Stats::Formula averageLoadLatency;
	Stats::Scalar finishedArithmetic;
	Stats::Scalar totalArithmeticLatency;
	Stats::Formula averageArithmeticLatency;
};

#endif //__CPU_PRED_LVPRED_UNIT_HH__
