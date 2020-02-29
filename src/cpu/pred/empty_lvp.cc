/**
 * Will have to ask what I'm supposed to put here
 */

#include "cpu/pred/empty_lvp.hh"
#include "debug/LVP.hh"
#include "mem/packet_access.hh"

typedef LoadValuePredictorParams Params;

EmptyLVP::EmptyLVP(Params * params)
    : LVPredUnit(params)
{
    DPRINTF(LVP, "lvpredType=none, doing nothing\n");
}

void EmptyLVP::setUpTables(const Params * params)
{
    DPRINTF(LVP, "lvpredType=none, doing nothing\n");
}

LVPredUnit::lvpReturnValues EmptyLVP::makePrediction(TheISA::PCState pc, ThreadID tid, unsigned currentCycle)
{
    DPRINTF(LVP, "lvpredType=none, doing nothing\n");
    return LVPredUnit::lvpReturnValues(0, -1);
}

bool EmptyLVP::processPacketRecieved(TheISA::PCState pc, StaticInstPtr inst, PacketPtr pkt, ThreadID tid, uint64_t prediction, int8_t confidence, unsigned cyclesElapsed, unsigned currentCycle)
{
    DPRINTF(LVP, "lvpredType=none, doing nothing\n");
    return true;
}
