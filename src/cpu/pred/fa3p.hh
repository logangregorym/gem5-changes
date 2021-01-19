/**
 * Will have to ask what I'm supposed to put here
 */

#ifndef __CPU_PRED_FA3P_HH__
#define __CPU_PRED_FA3P_HH__

#include <vector>

using namespace std;

#include "base/types.hh"
#include "cpu/pred/big_sat_counter.hh"
#include "cpu/pred/history.hh"
#include "cpu/pred/lvpred_unit.hh"
#include "cpu/pred/sat_counter.hh"
#include "mem/packet.hh"
#include "params/LoadValuePredictor.hh"

class FA3P : public LVPredUnit
{
  public:
    typedef LoadValuePredictorParams Params;

    FA3P(Params* params);

    virtual unsigned getConfidence(TheISA::PCState pc);

    virtual unsigned getDelay(TheISA::PCState pc);

    virtual lvpReturnValues makePrediction(TheISA::PCState pc, ThreadID tid, unsigned currentCycle);
    virtual bool makePredictionForTraceGenStage(Addr addr, uint16_t upc, ThreadID tid , lvpReturnValues& ret);

    virtual uint64_t getValuePredicted(TheISA::PCState pc);

    virtual bool processPacketRecieved(TheISA::PCState pc, StaticInstPtr inst, uint64_t value, ThreadID tid, uint64_t predictedValue, int8_t confidence, unsigned cyclesElapsed, unsigned currentCycle);

  private:
    virtual void setUpTables(const Params *p);

    struct ValueWithCount {
        ValueWithCount(uint64_t value, SatCounter count) {
            this->value = value;
            this->count = count;
            this->valid = true;
        }

        ValueWithCount() {
            this->value = 0;
            this->count = SatCounter(4);
            this->valid = false;
        }

        void reset()
        {
            this->value = 0;
            this->count.reset();
            this->valid = false;
        }

        uint64_t value;
        SatCounter count;
        bool valid;
    };

    struct LVTEntry {
        LVTEntry(ValueWithCount val1, ValueWithCount val2, ValueWithCount val3, History history, BigSatCounter confidence, int16_t upc, bool _valid) {
            this->val1 = val1;
            this->val2 = val2;
            this->val3 = val3;
            this->history = history;
            this->confidence = confidence;
            this->tag = 0;
            this->lastUsed = 0;
            this->micropc = upc;
            this->valid = _valid;
        }

        LVTEntry() {
            this->val1 = ValueWithCount();
            this->val2 = ValueWithCount();
            this->val3 = ValueWithCount();
            this->history = History();
            this->confidence = BigSatCounter();
            this->tag = 0;
            this->lastUsed = 0;
            this->micropc = -1;
            this->valid = false;
        }

        ValueWithCount val1;
        ValueWithCount val2;
        ValueWithCount val3;
        History history;
        BigSatCounter confidence;
        uint64_t tag;
        int16_t micropc = -1;
        bool valid = false;
        unsigned lastUsed;
	    unsigned numUses = 0;
	    unsigned averageCycles = 0;
    };

    struct Predictor {
        Predictor(vector<LVTEntry> LVT, vector<uint8_t> choice) {
            this->LVT = LVT;
            this->choice = choice;
        }

        Predictor() {
            this->LVT = vector<LVTEntry>();
            this->choice = vector<uint8_t>();
        }

        vector<LVTEntry> LVT;
        vector<uint8_t> choice; // Indexed by history, determines which value to use
        // only using 2 bits, but uint8_t is smallest type available
    };

    Predictor predictor;

    unsigned tableEntries;

    unsigned scBits;

    unsigned historyLength;

    unsigned initialPred;

    uint8_t missCount = 0;

    uint8_t hitCount = 0;

    friend class HybridLVP;
};

#endif //__CPU_PRED_FA3P_HH__
