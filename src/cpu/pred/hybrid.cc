/**
 * Will have to ask what I'm supposed to put here
 */

#define HYBRID_ON

#include "base/intmath.hh"

// #include "base/misc.hh"
#include "cpu/pred/hybrid.hh"
#include "debug/LVP.hh"
#include "mem/packet_access.hh"

typedef LoadValuePredictorParams Params;

HybridLVP::HybridLVP(Params *params)
    : LVPredUnit(params),
      tableEntries(params->tableEntries),
      scBits(params->satCounterBits),
      initialPred(params->initialPredictionQuality),
      historyLength(params->historyLength),
      historyEntryBits(params->historyEntryBits)
{
    DPRINTF(LVP, "Setting up tables (hybrid)\n");
    firstConstBasic = firstConst;
    firstConstSH = firstConst;
    firstConst3P = firstConst;
    setUpTables(params);
    assert(!threadPredictorsBasic.empty());
    assert(!threadPredictorsSH.empty());
    assert(!threadPredictors3P.empty());
    DPRINTF(LVP, "Tables are set up (hybrid)\n");
}

void HybridLVP::setUpTables(const Params *params)
{
    if (!isPowerOf2(tableEntries))
        fatal("Invalid table size! Must be power of 2\n");
//    for (int i=0; i < numThreads; i++) {
//	vector<uint8_t> newPT = vector<uint8_t>(tableEntries, 0);
//	threadPTs.push_back(newPT);
//    }
    setUpTablesBasic(params);
    setUpTablesSH(params);
    setUpTables3P(params);
}

void HybridLVP::setUpTablesBasic(const Params *params)
{
    for (int i=0; i < numThreads; i++) {
        vector<ValuePredWithConstStatus> newLVPCT = vector<ValuePredWithConstStatus>(tableEntries, ValuePredWithConstStatus(0, BigSatCounter(scBits, initialPred)));
        threadPredictorsBasic.push_back(newLVPCT);
    }
}

void HybridLVP::setUpTablesSH(const Params *params)
{
    for (int i=0; i < numThreads; i++) {
        vector<SHTEntry> newSHT = vector<SHTEntry>(tableEntries, SHTEntry(0, History(historyEntryBits, historyLength)));
        vector<unsigned> newinFlightCount = vector<unsigned>(tableEntries, 0);
        uint64_t SPTSize = power(2, historyEntryBits * historyLength);
        vector<SPTEntry> newSPT = vector<SPTEntry>(SPTSize, SPTEntry(0, BigSatCounter(scBits, initialPred)));
        threadPredictorsSH.push_back(predictorSH(newSHT, newSPT, newinFlightCount));
    }
}

void HybridLVP::setUpTables3P(const Params *params)
{
    for (int i=0; i < numThreads; i++) {
        vector<LVTEntry> newLVT = vector<LVTEntry>(tableEntries, LVTEntry(ValueWithCount(0, SatCounter(4)), ValueWithCount(0, SatCounter(4)), ValueWithCount(0, SatCounter(4)), History(2, historyLength), BigSatCounter(scBits, initialPred)));
        uint64_t choiceSize = power(2, historyLength*2);
        vector<uint8_t> newChoice = vector<uint8_t>(choiceSize,0);
        threadPredictors3P.push_back(predictor3P(newLVT, newChoice));
    }
}

LVPredUnit::lvpReturnValues HybridLVP::makePrediction(TheISA::PCState pc, ThreadID tid, unsigned currentCycle)
{
//    DPRINTF(LVP, "Inst %s called LVP makePrediction\n", inst->disassemble(pc.instAddr()));
    ++lvLookups;
//    if (inst->isRipRel()) { ++ripRelNum; }
    LVPredUnit::lvpReturnValues bpResp = makePredictionBasic(pc, tid);
    LVPredUnit::lvpReturnValues shpResp = makePredictionSH(pc, tid);
    LVPredUnit::lvpReturnValues tppResp = makePrediction3P(pc, tid);
    bpResp.confidence -= firstConstBasic;
    shpResp.confidence -= firstConstSH;
    tppResp.confidence -= firstConst3P;
    if ((bpResp.confidence >= shpResp.confidence) && (bpResp.confidence >= tppResp.confidence)) {
        DPRINTF(LVP, "Basic prediction used, value %x predicted with confidence %i and penalty %i\n", bpResp.predictedValue, bpResp.confidence); //, threadPTs[tid][idx]);
        basicChosen++;
//	bpResp.confidence -= threadPTs[tid][idx];
        return bpResp;
    } else if ((shpResp.confidence >= bpResp.confidence) && (shpResp.confidence >= tppResp.confidence)) {
        DPRINTF(LVP, "StrideHist prediction used, value %x predicted with confidence %i and penalty %i\n", shpResp.predictedValue, shpResp.confidence); //, threadPTs[tid][idx]);
        strideHistChosen++;
//	shpResp.confidence -= threadPTs[tid][idx];
        return shpResp;
    } else {
        DPRINTF(LVP, "3Period prediction used, value %x predicted with confidence %i and penalty %i\n", tppResp.predictedValue, tppResp.confidence); //, threadPTs[tid][idx]);
        periodicChosen++;
//	tppResp.confidence -= threadPTs[tid][idx];
        return tppResp;
    }
}

LVPredUnit::lvpReturnValues HybridLVP::makePredictionBasic(TheISA::PCState pc, ThreadID tid)
{
    Addr loadAddr = pc.instAddr();
    unsigned idx = loadAddr & (tableEntries - 1);
    vector<ValuePredWithConstStatus> threadLVPCT = threadPredictorsBasic[tid];
    uint64_t value = threadLVPCT[idx].value;
    int8_t status = threadLVPCT[idx].status.read();
    return LVPredUnit::lvpReturnValues(value, status, 1);
}

LVPredUnit::lvpReturnValues HybridLVP::makePredictionSH(TheISA::PCState pc, ThreadID tid)
{
    Addr loadAddr = pc.instAddr();
    unsigned idx = loadAddr & (tableEntries - 1);
    predictorSH p = threadPredictorsSH[tid];
    vector<SHTEntry> SHT = p.SHT;
    vector<SPTEntry> SPT = p.SPT;
    vector<unsigned> inFlightCount = p.inFlightCount;
    uint64_t base = SHT[idx].base;
    uint64_t hist = SHT[idx].history.read();
    unsigned stride = SPT[hist].stride;
    int8_t status = SPT[hist].confidence.read();
    unsigned inFlight = inFlightCount[idx];
    uint64_t value = base + (stride * (inFlight + 1));
    return LVPredUnit::lvpReturnValues(value, status, 2);
}

LVPredUnit::lvpReturnValues HybridLVP::makePrediction3P(TheISA::PCState pc, ThreadID tid)
{
    Addr loadAddr = pc.instAddr();
    unsigned idx = loadAddr & (tableEntries - 1);
    predictor3P threadPred = threadPredictors3P[tid];
    LVTEntry addressInfo = threadPred.LVT[idx];
    uint8_t choice = threadPred.choice[addressInfo.history.read()];
    uint64_t value;
    int8_t status;
    if (choice == 1) {
        value = addressInfo.val1.value;
        status = addressInfo.confidence.read();
    } else if (choice == 2) {
        value = addressInfo.val2.value;
        status = addressInfo.confidence.read();
    } else if (choice == 3) {
        value = addressInfo.val3.value;
        status = addressInfo.confidence.read();
    } else {
        value = 0;
        status = 0;
    }
    return LVPredUnit::lvpReturnValues(value, status, 3);
}

bool HybridLVP::processPacketRecieved(TheISA::PCState pc, StaticInstPtr inst, uint64_t value, ThreadID tid, uint64_t prediction, int8_t confidence, unsigned cyclesElapsed, uint8_t predSource, unsigned currentCycle)
{
    DPRINTF(LVP, "Inst %s calles processPacketRecieved\n", inst->disassemble(pc.instAddr()));
    DPRINTF(LVP, "Value %x predicted for address %x with confidence %i\n", prediction, pc.instAddr(), confidence);

    uint64_t responseVal = value;
/**
    if (pkt->isResponse()) {
        processPacketRecievedBasic(pc, inst, pkt, tid, prediction, confidence, cyclesElapsed);
        processPacketRecievedSH(pc, inst, pkt, tid, prediction, confidence, cyclesElapsed);
        processPacketRecieved3P(pc, inst, pkt, tid, prediction, confidence, cyclesElapsed);
        unsigned size = pkt->getSize();
        DPRINTF(LVP, "Packet contains %i bytes of data\n", size);
        assert(pkt->hasData());
        uint64_t responseVal;
        if (size == 8) {
            responseVal = pkt->get<uint64_t>();
        } else if (size == 4) {
            responseVal = pkt->get<uint32_t>();
        } else if (size == 2) {
            responseVal = pkt->get<uint16_t>();
        } else if (size == 1) {
            responseVal = pkt->get<uint8_t>();
        } else {
            ++largeValueLoads;
            bool noMisPred;
            if (predSource == 1) {
                noMisPred = confidence < 0;
            } else if (predSource == 2) {
                noMisPred = confidence < 0;
            } else {
                noMisPred = confidence < 0;
            }
            if (!noMisPred) {
                DPRINTF(LVP, "MISPREDICTION DETECTED for address %x\n", pc.instAddr());
                ++incorrectUsed;
//		threadPTs[tid][idx]++;
                if (predSource == 1) {
                    missCountBasic++;
                    if (missCountBasic >= missThreshold) {
                        firstConstBasic++;
                        ++constIncrement;
                        missCountBasic = 0;
                    }
                } else if (predSource == 2) {
                    missCountSH++;
                    if (missCountSH >= missThreshold) {
                        firstConstSH++;
                        ++constIncrement;
                        missCountSH = 0;
                    }
                } else if (predSource == 3) {
                    missCount3P++;
                    if (missCount3P >= missThreshold) {
                        firstConst3P++;
                        ++constIncrement;
                        missCount3P = 0;
                    }
                }
            } else { ++incorrectNotUsed; }
            return noMisPred;
        }
**/
        DPRINTF(LVP, "Value %x received for address %x\n", responseVal, pc.instAddr());

        if (confidence >= 0) {
            if (prediction == responseVal) {
                ++correctUsed;
                cyclesSaved += cyclesElapsed;
            } else {
                ++incorrectUsed;
            }
        } else {
            if (prediction == responseVal) {
                ++correctNotUsed;
                if (predSource == 1) {
                    hitCountBasic++;
                    if (hitCountBasic >= hitThreshold) {
                        firstConstBasic--;
                        ++constDecrement;
                        hitCountBasic = 0;
                    }
                } else if (predSource == 2) {
                    hitCountSH++;
                    if (hitCountSH >= hitThreshold) {
                        firstConstSH--;
                        ++constDecrement;
                        hitCountSH = 0;
                    }
                } else if (predSource == 3) {
                    hitCount3P++;
                    if (hitCount3P >= hitThreshold) {
                        firstConst3P--;
                        ++constDecrement;
                        hitCount3P = 0;
                    }
                }
            } else {
                ++incorrectNotUsed;
            }
        }
        bool misPred = (confidence >= 0) && (prediction != responseVal);
        if (misPred) {
            DPRINTF(LVP, "MISPREDICTION DETECTED for address %x\n", pc.instAddr());
//	    threadPTs[tid][idx]++;
            if (predSource == 1) {
                missCountBasic++;
                if (missCountBasic >= missThreshold) {
                    firstConstBasic++;
                    ++constIncrement;
                    missCountBasic = 0;
                }
            } else if (predSource == 2) {
                missCountSH++;
                if (missCountSH >= missThreshold) {
                    firstConstSH++;
                    ++constIncrement;
                    missCountSH = 0;
                }
            } else if (predSource == 3) {
                missCount3P++;
                if (missCount3P >= missThreshold) {
                    firstConst3P++;
                    ++constIncrement;
                    missCount3P = 0;
                }
            }
        }
        return !misPred;
    }
    return true;
}

bool HybridLVP::processPacketRecievedBasic(TheISA::PCState pc, StaticInstPtr inst, uint64_t value, ThreadID tid, uint64_t prediction, int8_t confidence, unsigned cyclesElapsed)
{
    uint64_t responseVal = value;
    if (pkt->isResponse()) {
        Addr loadAddr = pc.instAddr();
        unsigned idx = loadAddr & (tableEntries - 1);
        vector<ValuePredWithConstStatus> &threadLVPCT = threadPredictorsBasic[tid];
/**
        unsigned size = pkt->getSize();
        assert(pkt->hasData());
        uint64_t responseVal;
        if (size == 8) {
            responseVal = pkt->get<uint64_t>();
        } else if (size == 4) {
            responseVal = pkt->get<uint32_t>();
        } else if (size == 2) {
            responseVal = pkt->get<uint16_t>();
        } else if (size == 1) {
            responseVal = pkt->get<uint8_t>();
        } else {
            bool noMisPred = confidence < 0;
            if (!noMisPred) {
                threadLVPCT[idx].status.reset();
                while (threadLVPCT[idx].status.read() < resetTo) {
                    threadLVPCT[idx].status.increment();
                }
            }
            return noMisPred;
        }
**/
        bool misPred = (confidence >= 0) && (prediction != responseVal);
        if (prediction == responseVal) {
            threadLVPCT[idx].status.increment();
        } else {
            threadLVPCT[idx].value = responseVal;
            for (int i=0; i < decrementBy; i++) {
                threadLVPCT[idx].status.decrement();
            }
        }
        if (misPred) {
            threadLVPCT[idx].status.reset();
            while (threadLVPCT[idx].status.read() < resetTo) {
                threadLVPCT[idx].status.increment();
            }
        }
        return !misPred;
    }
    return true;
}

bool HybridLVP::processPacketRecievedSH(TheISA::PCState pc, StaticInstPtr inst, uint64_t value, ThreadID tid, uint64_t prediction, int8_t confidence, unsigned cyclesElapsed)
{
    if (pkt->isResponse()) {
        Addr loadAddr = pc.instAddr();
        unsigned idx = loadAddr & (tableEntries - 1);
        predictorSH &p = threadPredictorsSH[tid];
        vector<SHTEntry> &SHT = p.SHT;
        vector<SPTEntry> &SPT = p.SPT;
        vector<unsigned> &inFlightCount = p.inFlightCount;
        assert(inFlightCount[idx] > 0);
        inFlightCount[idx]--;
        uint64_t responseVal = value;
/**
        unsigned size = pkt->getSize();
        assert(pkt->hasData());
        uint64_t responseVal;
        if (size == 8) {
            responseVal = pkt->get<uint64_t>();
        } else if (size == 4) {
            responseVal = pkt->get<uint32_t>();
        } else if (size == 2) {
            responseVal = pkt->get<uint16_t>();
        } else if (size == 1) {
            responseVal = pkt->get<uint8_t>();
        } else {
            bool noMisPred = confidence < 0;
            if (!noMisPred) {
                SPT[SHT[idx].history.read()].confidence.reset();
                while (SPT[SHT[idx].history.read()].confidence.read() < resetTo) {
                    SPT[SHT[idx].history.read()].confidence.increment();
                }
            }
            return noMisPred;
        }
**/
        bool misPred = (confidence >= 0) && (prediction != responseVal);
        uint64_t oldBase = SHT[idx].base;
        uint64_t oldHist = SHT[idx].history.read();
        if (prediction == responseVal) {
            SPT[oldHist].confidence.increment();
        } else {
            for (int i = 0; i < decrementBy; i++) {
                SPT[oldHist].confidence.decrement();
            }
        }
        if (misPred) {
            SPT[oldHist].confidence.reset();
            while (SPT[oldHist].confidence.read() < resetTo) {
                SPT[oldHist].confidence.increment();
            }
        }
        unsigned stride = (responseVal - oldBase);
        SHT[idx].base = responseVal;
        SHT[idx].history.update(stride);
        return !misPred;
    }
    return true;
}

bool HybridLVP::processPacketRecieved3P(TheISA::PCState pc, StaticInstPtr inst, uint64_t, ThreadID tid, uint64_t prediction, int8_t confidence, unsigned cyclesElapsed)
{
    if (pkt->isResponse()) {
        Addr loadAddr = pc.instAddr();
        unsigned idx = loadAddr & (tableEntries - 1);
        predictor3P &threadPred = threadPredictors3P[tid];
        LVTEntry &addressInfo = threadPred.LVT[idx];
        uint64_t responseVal = value;
/**
        unsigned size = pkt->getSize();
        assert(pkt->hasData());
        uint64_t responseVal;
        if (size == 8) {
            responseVal = pkt->get<uint64_t>();
        } else if (size == 4) {
            responseVal = pkt->get<uint32_t>();
        } else if (size == 2) {
            responseVal = pkt->get<uint16_t>();
        } else if (size == 1) {
            responseVal = pkt->get<uint8_t>();
        } else {
            addressInfo.history.update(0);
            bool noMisPred = confidence < 0;
            if (!noMisPred) {
                addressInfo.confidence.reset();
                while (addressInfo.confidence.read() < resetTo) {
                    addressInfo.confidence.increment();
                }
            }
            return noMisPred;
        }
**/
        bool misPred = (confidence >= 0) && (prediction != responseVal);

        if (prediction == responseVal) {
            addressInfo.confidence.increment();
        } else {
            for (int i = 0; i < decrementBy; i++) {
                addressInfo.confidence.decrement();
            }
        }
        if (responseVal == addressInfo.val1.value) {
            threadPred.choice[addressInfo.history.read()] = 1;
            addressInfo.history.update(1);
            addressInfo.val1.count.increment();
        } else if (responseVal == addressInfo.val2.value) {
            threadPred.choice[addressInfo.history.read()] = 2;
            addressInfo.history.update(2);
            addressInfo.val2.count.increment();
        } else if (responseVal == addressInfo.val3.value) {
            threadPred.choice[addressInfo.history.read()] = 3;
            addressInfo.history.update(3);
            addressInfo.val3.count.increment();
        } else {
            threadPred.choice[addressInfo.history.read()] = 0;
            unsigned c1 = addressInfo.val1.count.read();
            unsigned c2 = addressInfo.val2.count.read();
            unsigned c3 = addressInfo.val3.count.read();
            if ((c1 <= 0) && (c1 <= c2) && (c1 <= c3)) {
                addressInfo.val1.value = responseVal;
                addressInfo.val1.count.reset();
                addressInfo.val1.count.increment();
                addressInfo.history.update(1);
            } else if ((c2 <= 0) && (c2 <= c1) && (c2 <= c3)) {
                addressInfo.val2.value = responseVal;
                addressInfo.val2.count.reset();
                addressInfo.val2.count.increment();
                addressInfo.history.update(2);
            } else if ((c3 <= 0) && (c3 <= c1) && (c3 <= c2)) {
                addressInfo.val3.value = responseVal;
                addressInfo.val3.count.reset();
                addressInfo.val3.count.increment();
                addressInfo.history.update(3);
            } else {
                addressInfo.val1.count.decrement();
                addressInfo.val2.count.decrement();
                addressInfo.val3.count.decrement();
                addressInfo.history.update(0);
            }
        }
        if (misPred) {
            addressInfo.confidence.reset();
            while (addressInfo.confidence.read() < resetTo) {
                addressInfo.confidence.increment();
            }
        }
        return !misPred;
    }
    return true;
}
