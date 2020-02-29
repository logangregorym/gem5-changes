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

    virtual lvpReturnValues makePrediction(TheISA::PCState pc, ThreadID tid, unsigned currentCycle);

    virtual bool processPacketRecieved(TheISA::PCState pc, StaticInstPtr inst, PacketPtr pkt, ThreadID tid, uint64_t predictedValue, int8_t confidence, unsigned cyclesElapsed, unsigned currentCycle);

  private:
    virtual void setUpTables(const Params *p);

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
            this->tag = 0;
            this->lastUsed = 0;
        }

        LVTEntry() {
            this->val1 = ValueWithCount();
            this->val2 = ValueWithCount();
            this->val3 = ValueWithCount();
            this->history = History();
            this->confidence = BigSatCounter();
            this->tag = 0;
            this->lastUsed = 0;
        }

        ValueWithCount val1;
        ValueWithCount val2;
        ValueWithCount val3;
        History history;
        BigSatCounter confidence;
        uint64_t tag;
        unsigned lastUsed;
    };

    struct predictor {
        predictor(vector<LVTEntry> LVT, vector<uint8_t> choice) {
            this->LVT = LVT;
            this->choice = choice;
        }

        predictor() {
            this->LVT = vector<LVTEntry>();
            this->choice = vector<uint8_t>();
        }

        vector<LVTEntry> LVT;
        vector<uint8_t> choice; // Indexed by history, determines which value to use
        // only using 2 bits, but uint8_t is smallest type available
    };

    vector<predictor> threadPredictors;

    unsigned tableEntries;

    unsigned scBits;

    unsigned historyLength;

    unsigned initialPred;

    uint8_t missCount = 0;

    uint8_t hitCount = 0;

    friend class HybridLVP;
};

#endif //__CPU_PRED_FA3P_HH__
