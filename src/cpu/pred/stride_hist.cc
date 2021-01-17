/**
 * Will have to ask what I'm supposed to put here
 */

#include "base/intmath.hh"

// #include "base/misc.hh"
#include "cpu/pred/stride_hist.hh"
#include "debug/LVP.hh"
#include "mem/packet_access.hh"

typedef LoadValuePredictorParams Params;

StrideHistLVP::StrideHistLVP(Params *params)
    : LVPredUnit(params),
      tableEntries(params->tableEntries),
      scBits(params->satCounterBits),
      historyLength(params->historyLength),
      historyEntryBits(params->historyEntryBits),
      initialPred(params->initialPredictionQuality)
{
    DPRINTF(LVP, "Setting up tables (strideHist)\n");
    setUpTables(params);
    assert(!threadPredictors.empty());
    DPRINTF(LVP, "Tables are set up (strideHist)\n");
}

void StrideHistLVP::setUpTables(const Params *params)
{
    if (!isPowerOf2(tableEntries))
        fatal("Invalid table size! Must be power of 2\n");
    for (int i = 0; i < numThreads; i++) {
        DPRINTF(LVP, "Building SHT with %i entries for thread %i\n", tableEntries, i);
        vector<SHTEntry> newSHT = vector<SHTEntry>(tableEntries, SHTEntry(0, History(historyEntryBits, historyLength)));
        vector<unsigned> newinFlightCount = vector<unsigned>(tableEntries, 0);
        uint64_t SPTSize = power(2, historyEntryBits * historyLength);
        DPRINTF(LVP, "Building SPT with %i entries for thread %i\n", SPTSize, i);
        vector<SPTEntry> newSPT = vector<SPTEntry>(SPTSize, SPTEntry(0, BigSatCounter(scBits, initialPred)));
        threadPredictors.push_back(predictor(newSHT, newSPT, newinFlightCount));
    }
}


// bool StrideHistLVP::makePredictionForTraceGenStage(Addr loadAddr, ThreadID tid , LVPredUnit::lvpReturnValues& ret)
// {
//     return false;
// }


LVPredUnit::lvpReturnValues StrideHistLVP::makePrediction(TheISA::PCState pc, ThreadID tid, unsigned currentCycle)
{
//    DPRINTF(LVP, "Inst %s called LVP makePrediction\n", inst->disassemble(pc.instAddr()));
    ++lvLookups;
//    if (inst->isRipRel()) { ++ripRelNum; }
    Addr loadAddr = pc.instAddr();
    unsigned idx = loadAddr & (tableEntries - 1);
    predictor p = threadPredictors[tid];
    vector<SHTEntry> SHT = p.SHT;
    vector<SPTEntry> SPT = p.SPT;
    vector<unsigned> inFlightCount = p.inFlightCount;
    DPRINTF(LVP, "Accessing index %x (%i) in SHT of size %i\n", idx, idx, SHT.size());
    SHTEntry entry = SHT[idx];
    uint64_t base = entry.base;
    uint64_t hist = entry.history.read();
    DPRINTF(LVP, "Accessing index %x (%i) in SPT of size %i\n", hist, hist, SPT.size());
    SPTEntry entry2 = SPT[hist];
    unsigned stride = entry2.stride;
    int8_t status = entry2.confidence.read();
    unsigned inFlight = inFlightCount[idx];
    uint64_t value = base + (stride * (inFlight + 1));
    DPRINTF(LVP, "Value for address %x is %x\n", loadAddr, value);
    DPRINTF(LVP, "Status for address %x is %i\n", loadAddr, status - firstConst);
    return LVPredUnit::lvpReturnValues(value, status - firstConst);
}

bool StrideHistLVP::processPacketRecieved(TheISA::PCState pc, StaticInstPtr inst, uint64_t value, ThreadID tid, uint64_t prediction, int8_t confidence, unsigned cyclesElapsed, unsigned currentCycle)
{
    DPRINTF(LVP, "Inst %s called processPacketRecieved\n", inst->disassemble(pc.instAddr()));
    Addr loadAddr = pc.instAddr();
    unsigned idx = loadAddr & (tableEntries - 1);
    // uint64_t prediction = inst->predictedValue;
    // uint8_t confidence = inst->confidence;
    DPRINTF(LVP, "Value %x predicted for address %x with confidence %i\n", prediction, loadAddr, confidence);
    predictor &p = threadPredictors[tid];
    vector<SHTEntry> &SHT = p.SHT;
    vector<SPTEntry> &SPT = p.SPT;
    vector<unsigned> &inFlightCount = p.inFlightCount;
    SHTEntry shtEntry = SHT[idx];
    if (value) {
// 	Moved to lsq_unit_impl.hh
//	assert(inst->pendingPredictions > 0);
//	inst->pendingPredictions--;
        assert(inFlightCount[idx] > 0);
        inFlightCount[idx]--;
        uint64_t responseVal = value;
/**
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
            bool noMisPred = confidence < 0;
            if (!noMisPred) {
                DPRINTF(LVP, "MISPREDICTION DETECTED for address %x\n", loadAddr);
                SPTEntry &entry = SPT[shtEntry.history.read()];
                entry.confidence.reset();
                while (entry.confidence.read() < resetTo) {
                    entry.confidence.increment();
                }
                ++incorrectUsed;
                missCount++;
                if (missCount >= missThreshold) {
                    firstConst++;
                    ++constIncrement;
                    missCount = 0;
                }
                // lastMisprediction = inst->memoryAccessEndCycle;
            } else {
                ++incorrectNotUsed;
            }
            return noMisPred;
        }
**/
        DPRINTF(LVP, "Value %x received for address %x\n", responseVal, loadAddr);

        // Stats time!
        if (confidence >= 0) {
            // Used
            if (prediction == responseVal) {
                // Correct
                ++correctUsed;
                cyclesSaved += cyclesElapsed;
            } else {
                // Incorrect
                ++incorrectUsed;
            }
        } else {
            // Unused
            if (prediction == responseVal) {
                // Correct
                ++correctNotUsed;
                hitCount++;
                if (hitCount >= hitThreshold) {
                    firstConst--;
                    ++constDecrement;
                    hitCount = 0;
                }
            } else {
                // Incorrect
                ++incorrectNotUsed;
            }
        }

        bool misPred = (confidence >= 0) && (prediction != responseVal);
        uint64_t oldBase = shtEntry.base;
        uint64_t oldHist = shtEntry.history.read();
        SPTEntry &sptEntry = SPT[oldHist];
        if (prediction == responseVal) {
            // increase confidence
            sptEntry.confidence.increment();
        } else {
            // decrease confidence
            for (int i = 0; i < decrementBy; i++) {
                sptEntry.confidence.decrement();
            }
        }
        // Stride == change in base
        unsigned stride = (responseVal - oldBase);
        // Update base
        shtEntry.base = responseVal;
        // Add stride to history
        shtEntry.history.update(stride);
        if (misPred) {
            DPRINTF(LVP, "MISPREDICTION DETECTED for address %x\n", loadAddr);
            sptEntry.confidence.reset();
            while (sptEntry.confidence.read() < resetTo) {
                sptEntry.confidence.increment();
            }
            missCount++;
            if (missCount >= missThreshold) {
                firstConst++;
                ++constIncrement;
                missCount = 0;
            }
        }
        return !misPred;
    }
    return true;
}
