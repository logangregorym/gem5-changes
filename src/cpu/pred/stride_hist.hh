/**
 * Will have to ask what I'm supposed to put here
 */

#ifndef __CPU_PRED_STRIDE_HIST_HH__
#define __CPU_PRED_STRIDE_HIST_HH__

#include <vector>

using namespace std;

#include "base/types.hh"
#include "cpu/pred/big_sat_counter.hh"
#include "cpu/pred/history.hh"
#include "cpu/pred/lvpred_unit.hh"
#include "mem/packet.hh"
#include "params/LoadValuePredictor.hh"

class StrideHistLVP : public LVPredUnit
{
  public:
    typedef LoadValuePredictorParams Params;

    StrideHistLVP(Params* params);

    virtual lvpReturnValues makePrediction(TheISA::PCState pc, ThreadID tid, unsigned currentCycle);
    //virtual bool makePredictionForTraceGenStage(Addr loadAddr, ThreadID tid , lvpReturnValues& ret);

    virtual bool processPacketRecieved(TheISA::PCState pc, StaticInstPtr inst, uint64_t value, ThreadID tid, uint64_t predictedValue, int8_t confidence, unsigned cyclesElapsed, unsigned currentCycle);

  private:
    virtual void setUpTables(const Params *p);

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
            this->confidence = BigSatCounter(2, 0);
        }

        unsigned stride;
        BigSatCounter confidence;
    };

    struct predictor {
        predictor(vector<SHTEntry> SHT, vector<SPTEntry> SPT, vector<unsigned> inFlightCount) {
            this->SHT = SHT;
            this->SPT = SPT;
            this->inFlightCount = inFlightCount;
        }

        predictor() {
            this->SHT = vector<SHTEntry>();
            this->SPT = vector<SPTEntry>();
            this->inFlightCount = vector<unsigned>();
        }

        vector<SHTEntry> SHT;

        vector<SPTEntry> SPT;

        vector<unsigned> inFlightCount; // TODO: find a better way to do this
    };

    vector<predictor> threadPredictors;

    unsigned tableEntries;

    unsigned scBits;

    unsigned historyLength;

    unsigned historyEntryBits;

    unsigned initialPred;

    uint8_t missCount = 0;

    uint8_t hitCount = 0;

    friend class HybridLVP;
};

#endif //__CPU_PRED_STRIDE_HIST_HH__
