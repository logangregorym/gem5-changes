/**
 * Will have to ask what I'm supposed to put here
 */

#ifndef __CPU_PRED_BASIC_LVPRED_HH__
#define __CPU_PRED_BASIC_LVPRED_HH__

#include <vector>

using namespace std;

#include "base/types.hh"
#include "cpu/pred/big_sat_counter.hh"
#include "cpu/pred/lvpred_unit.hh"
#include "mem/packet.hh"
#include "params/LoadValuePredictor.hh"

class BasicLVP : public LVPredUnit
{
  public:
    typedef LoadValuePredictorParams Params;

    BasicLVP(Params* params);

    virtual lvpReturnValues makePrediction(TheISA::PCState pc, ThreadID tid, unsigned currentCycle);

    virtual bool processPacketRecieved(TheISA::PCState pc, StaticInstPtr inst, PacketPtr pkt, ThreadID tid, uint64_t predictedValue, int8_t confidence, unsigned cyclesElapsed, unsigned currentCycle);

  private:
    virtual void setUpTables(const Params *p);

    typedef vector<ValuePredWithConstStatus> LVPCT;

    vector<LVPCT> threadPredictors;

    unsigned tableEntries;

    unsigned scBits;

    unsigned initialPred;

    uint8_t missCount = 0;

    uint8_t hitCount = 0;

    friend class HybridLVP;
};

#endif
