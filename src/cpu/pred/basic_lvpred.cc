/**
 * Will have to ask what I'm supposed to put here
 */

#include "base/intmath.hh"

// #include "base/misc.hh"
#include "cpu/pred/basic_lvpred.hh"
#include "debug/LVP.hh"
#include "mem/packet_access.hh"

typedef LoadValuePredictorParams Params;

BasicLVP::BasicLVP(Params *params)
    : LVPredUnit(params),
      tableEntries(params->tableEntries),
      scBits(params->satCounterBits),
      initialPred(params->initialPredictionQuality)
{
    DPRINTF(LVP, "Setting up tables (basic)\n");
    setUpTables(params);
    assert(!threadPredictors.empty());
    DPRINTF(LVP, "Tables are set up (basic)\n");
}

void BasicLVP::setUpTables(const Params *params)
{
    if (!isPowerOf2(tableEntries))
        fatal("Invalid table size! Must be power of 2\n");

    for (int i = 0; i < numThreads; i++) {
        DPRINTF(LVP, "Building table with %i entries for thread %i\n", tableEntries, i);
        vector<ValuePredWithConstStatus> newLVPCT = vector<ValuePredWithConstStatus>(tableEntries, ValuePredWithConstStatus(0, BigSatCounter(scBits, initialPred)));
        threadPredictors.push_back(newLVPCT);
    }
}

LVPredUnit::lvpReturnValues BasicLVP::makePrediction(TheISA::PCState pc, ThreadID tid, unsigned currentCycle)
{
//    DPRINTF(LVP, "Inst %s called LVP makePrediction\n", inst->disassemble(pc.instAddr()));
    ++lvLookups;
//    if (inst->isRipRel()) { ++ripRelNum; }
    Addr loadAddr = pc.instAddr();
    unsigned idx = loadAddr & (tableEntries - 1);
    vector<ValuePredWithConstStatus> threadLVPCT = threadPredictors[tid];
    uint64_t value = threadLVPCT[idx].value;
    DPRINTF(LVP, "Value for address %x is %x\n", loadAddr, value);
    int8_t status = threadLVPCT[idx].status.read();
    DPRINTF(LVP, "Read status %d then subtracted firstConst %d to get status %d\n", status, firstConst, status-firstConst);
    DPRINTF(LVP, "Status for address %x is %i\n", loadAddr, status - firstConst);
    return LVPredUnit::lvpReturnValues(value, status - firstConst);
}

bool BasicLVP::processPacketRecieved(TheISA::PCState pc, StaticInstPtr inst, uint64_t value, ThreadID tid, uint64_t prediction, int8_t confidence, unsigned cyclesElapsed, unsigned currentCycle)
{
    DPRINTF(LVP, "Inst %s called processPacketRecieved\n", inst->disassemble(pc.instAddr()));
    Addr loadAddr = pc.instAddr();
    unsigned idx = loadAddr & (tableEntries - 1);
    DPRINTF(LVP, "Value %x predicted for address %x with confidence %i\n", prediction, loadAddr, confidence);
    vector<ValuePredWithConstStatus> &threadLVPCT = threadPredictors[tid];

    uint64_t responseVal = value;
/**
    if (pkt->isResponse()) {
        // Moved to lsq_unit_impl.hh
        // assert(inst->pendingPredictions > 0);
        // inst->pendingPredictions--;
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
                threadLVPCT[idx].status.reset();
                while (threadLVPCT[idx].status.read() < resetTo) {
                    threadLVPCT[idx].status.increment();
                }
                ++incorrectUsed;
            } else {
                ++incorrectNotUsed;
            }
            if (!noMisPred) {
                missCount++;
                if (missCount >= missThreshold) {
                    firstConst++;
                    ++constIncrement;
                    missCount = 0;
                }
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
        if (prediction == responseVal) {
            threadLVPCT[idx].status.increment();
            DPRINTF(LVP, "Updated status of index %x to %i\n", idx, threadLVPCT[idx].status.read());
        } else {
            threadLVPCT[idx].value = responseVal;
            for (int i=0; i < decrementBy; i++) {
                threadLVPCT[idx].status.decrement(); // ensure we're now not using prediction
            }
            DPRINTF(LVP, "Updated status of index %x to %i and value to %x\n", idx, threadLVPCT[idx].status.read(), threadLVPCT[idx].value);
        }
        if (misPred) {
            DPRINTF(LVP, "MISPREDICTION DETECTED for address %x\n", loadAddr);
            threadLVPCT[idx].status.reset();
            while (threadLVPCT[idx].status.read() < resetTo) {
                threadLVPCT[idx].status.increment(); // ensure we're now not using prediction
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
