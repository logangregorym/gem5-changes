/**
 * Will have to ask what I'm supposed to put here
 */

#ifndef __CPU_PRED_HYBRID_LVPRED_HH__
#define __CPU_PRED_HYBRID_LVPRED_HH__

#include <vector>

using namespace std;

#include "base/types.hh"
#include "cpu/pred/big_sat_counter.hh"
#include "cpu/pred/history.hh"
#include "cpu/pred/lvpred_unit.hh"
#include "cpu/pred/sat_counter.hh"
#include "mem/packet.hh"
#include "params/LoadValuePredictor.hh"

// #include "basic_lvpred.hh"
// #include "stride_hist.hh"
// #include "three_period.hh"

class HybridLVP : public LVPredUnit
{
  public:
    typedef LoadValuePredictorParams Params;

    HybridLVP(Params* params);

    virtual lvpReturnValues makePrediction(TheISA::PCState pc, ThreadID tid, unsigned currentCycle);
    //virtual bool makePredictionForTraceGenStage(Addr loadAddr, ThreadID tid , lvpReturnValues& ret);

    virtual bool processPacketRecieved(TheISA::PCState pc, StaticInstPtr inst, uint64_t value, ThreadID tid, uint64_t predictedValue, int8_t confidence, unsigned cyclesElapsed, unsigned currentCycle);

  private:
    virtual void setUpTables(const Params* p);

    void setUpTablesBasic(const Params* p);
    void setUpTablesSH(const Params* p);
    void setUpTables3P(const Params* p);

    lvpReturnValues makePredictionBasic(TheISA::PCState pc, ThreadID tid);
    lvpReturnValues makePredictionSH(TheISA::PCState pc, ThreadID tid);
    lvpReturnValues makePrediction3P(TheISA::PCState pc, ThreadID tid);

    bool processPacketRecievedBasic(TheISA::PCState pc, StaticInstPtr inst, uint64_t value, ThreadID tid, uint64_t predictedValue, int8_t confidence, unsigned cyclesElapsed);
    bool processPacketRecievedSH(TheISA::PCState pc, StaticInstPtr inst, uint64_t value, ThreadID tid, uint64_t predictedValue, int8_t confidence, unsigned cyclesElapsed);
    bool processPacketRecieved3P(TheISA::PCState pc, StaticInstPtr inst, uint64_t value, ThreadID tid, uint64_t predictedValue, int8_t confidence, unsigned cyclesElapsed);

//    BasicLVP basicPred;
//    StrideHistLVP strideHistPred;
//    ThreePeriodLVP threePeriodPred;

    // Parameters
    unsigned tableEntries;
    unsigned scBits;
    unsigned initialPred;
    unsigned historyLength;
    unsigned historyEntryBits;

    // Universal Dynamic Threshold Tracker
//    typedef vector<uint8_t> penaltyTable;
//    vector<penaltyTable> threadPTs;

    // Basic Variables
    typedef vector<ValuePredWithConstStatus> LVPCT;
    vector<LVPCT> threadPredictorsBasic;

    // Stride History Variables
    struct SHTEntry {
        SHTEntry(uint64_t base, History history) {
            this->base = base;
            this->history = history;
        }
        SHTEntry() {
            this->base = 0;
            this->history = History();
        }
        uint64_t base;
        History history;
    };
    struct SPTEntry {
        SPTEntry(unsigned stride, BigSatCounter confidence) {
            this->stride = stride;
            this->confidence = confidence;
        }
        SPTEntry() {
            this->stride = 0;
            this->confidence = 0;
        }
        unsigned stride;
        BigSatCounter confidence;
    };
    struct predictorSH {
        predictorSH(vector<SHTEntry> SHT, vector<SPTEntry> SPT, vector<unsigned> inFlightCount) {
            this->SHT = SHT;
            this->SPT = SPT;
            this->inFlightCount = inFlightCount;
        }
        predictorSH() {
            this->SHT = vector<SHTEntry>();
            this->SPT = vector<SPTEntry>();
            this->inFlightCount = vector<unsigned>();
        }
        vector<SHTEntry> SHT;
        vector<SPTEntry> SPT;
        vector<unsigned> inFlightCount;
    };
    vector<predictorSH> threadPredictorsSH;

    // Three Periodic Variables
    struct ValueWithCount {
        ValueWithCount(uint64_t value, SatCounter count) {
            this->value = value;
            this->count = count;
        }
        ValueWithCount() {
            this->value = 0;
            this->count = SatCounter(4);
        }
        uint64_t value;
        SatCounter count;
    };
    struct LVTEntry {
        LVTEntry(ValueWithCount val1, ValueWithCount val2, ValueWithCount val3, History history, BigSatCounter confidence) {
            this->val1 = val1;
            this->val2 = val2;
            this->val3 = val3;
            this->history = history;
            this->confidence = confidence;
        }
        LVTEntry() {
            this->val1 = ValueWithCount();
            this->val2 = ValueWithCount();
            this->val3 = ValueWithCount();
            this->history = History();
            this->confidence = BigSatCounter();
        }
        ValueWithCount val1;
        ValueWithCount val2;
        ValueWithCount val3;
        History history;
        BigSatCounter confidence;
    };
    struct predictor3P {
        predictor3P(vector<LVTEntry> LVT, vector<uint8_t> choice) {
            this->LVT = LVT;
            this->choice = choice;
        }
        predictor3P() {
            this->LVT = vector<LVTEntry>();
            this->choice = vector<uint8_t>();
        }
        vector<LVTEntry> LVT;
        vector<uint8_t> choice;
    };
    vector<predictor3P> threadPredictors3P;

    uint8_t missCountBasic = 0;
    uint8_t missCountSH = 0;
    uint8_t missCount3P = 0;

    uint8_t hitCountBasic = 0;
    uint8_t hitCountSH = 0;
    uint8_t hitCount3P = 0;

    uint8_t firstConstBasic;
    uint8_t firstConstSH;
    uint8_t firstConst3P;
};

#endif
