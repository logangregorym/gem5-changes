/**
 * Will have to ask what I'm supposed to put here
 */

#ifndef __CPU_PRED_EMPTY_LVP_HH__
#define __CPU_PRED_EMPTY_LVP_HH__

#include "cpu/pred/lvpred_unit.hh"
#include "mem/packet.hh"
#include "params/LoadValuePredictor.hh"

class EmptyLVP : public LVPredUnit
{
  public:
    typedef LoadValuePredictorParams Params;
    EmptyLVP(Params * params);
    virtual void setUpTables(const Params * params);
    virtual lvpReturnValues makePrediction(TheISA::PCState pc, ThreadID tid, unsigned currentCycle);
    virtual bool makePredictionForTraceGenStage(Addr loadAddr, ThreadID tid , lvpReturnValues& ret);
    virtual bool processPacketRecieved(TheISA::PCState pc, StaticInstPtr inst, uint64_t value, ThreadID tid, uint64_t predictedValue, int8_t confidence, unsigned cyclesElapsed, unsigned currentCycle);
};

#endif
