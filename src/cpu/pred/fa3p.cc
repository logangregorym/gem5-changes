/**
 * Will have to ask what I'm supposed to put here
 */

#include "base/intmath.hh"

// #include "base/misc.hh"
#include "cpu/pred/fa3p.hh"
#include "debug/LVP.hh"
#include "mem/packet_access.hh"

typedef LoadValuePredictorParams Params;

FA3P::FA3P(Params *params)
    : LVPredUnit(params),
      tableEntries(params->tableEntries),
      scBits(params->satCounterBits),
      historyLength(params->historyLength),
      initialPred(params->initialPredictionQuality)
{
    DPRINTF(LVP, "Setting up tables (fa3period)\n");
    setUpTables(params);
    assert(!threadPredictors.empty());
    DPRINTF(LVP, "Tables are set up (fa3period)\n");
}

void FA3P::setUpTables(const Params *params)
{
    if (!isPowerOf2(tableEntries))
        fatal("Invalid table size! Must be power of 2\n");
    for (int i = 0; i < numThreads; i++) {
        DPRINTF(LVP, "Building LVT with %i entries for thread %i\n", tableEntries, i);
        vector<LVTEntry> newLVT = vector<LVTEntry>(tableEntries, LVTEntry(ValueWithCount(), ValueWithCount(), ValueWithCount(), History(2, historyLength), BigSatCounter(scBits, initialPred)));
        uint64_t choiceSize = power(2, (historyLength * 2)); // 2 bits each
        DPRINTF(LVP, "Building choice table with %i entries for thread %i\n", choiceSize, i);
        vector<uint8_t> newChoice = vector<uint8_t>(choiceSize, 0);
        threadPredictors.push_back(predictor(newLVT, newChoice));
        // cout << tableEntries << " " << newLVT.size() << endl;
    }
}

unsigned FA3P::getConfidence(Addr addr) {
    for (int i=0; i < tableEntries; i++) {
        if (threadPredictors[0].LVT[i].tag == addr) {
            return threadPredictors[0].LVT[i].confidence.read() - firstConst;
        }
    }
    return 0;
}

unsigned FA3P::getDelay(Addr addr) {
    for (int i=0; i < tableEntries; i++) {
        if (threadPredictors[0].LVT[i].tag == addr) {
            return threadPredictors[0].LVT[i].averageCycles;
        }
    }
    return 0;
}

LVPredUnit::lvpReturnValues FA3P::makePrediction(TheISA::PCState pc, ThreadID tid, unsigned currentCycle)
{
//    DPRINTF(LVP, "Inst %s called LVP makePrediction\n", inst->disassemble(pc.instAddr()));
    ++lvLookups;
//    if (inst->isRipRel()) { ++ripRelNum; }
    Addr loadAddr = pc.instAddr();
    predictor &threadPred = threadPredictors[tid];

    LVTEntry *addressInfo = NULL;
    bool foundAddress = false;
    for (int i=0; i < tableEntries; i++) {
        if (threadPred.LVT[i].tag == loadAddr) {
            addressInfo = &threadPred.LVT[i];
            foundAddress = true;
        }
        else if (threadPred.LVT[i].tag == 0) {
            threadPred.LVT[i].tag = loadAddr;
            addressInfo = &threadPred.LVT[i];
            foundAddress = true;
        }
    }

    if (!foundAddress) {
        tagMismatch++;
        LVTEntry *LRU = &threadPred.LVT[0];
        for (int i=0; i < tableEntries; i++) {
            if (threadPred.LVT[i].lastUsed < LRU->lastUsed) {
                LRU = &threadPred.LVT[i];
            }
        }
        LRU->history.reset();
        LRU->confidence.reset();
        LRU->tag = loadAddr;
        addressInfo = LRU;
    }

    addressInfo->lastUsed = currentCycle;

    uint8_t choice = threadPred.choice[addressInfo->history.read()];
    DPRINTF(LVP, "Choice %i selected for address %x based on history %x\n", choice, loadAddr, addressInfo->history.read());
    uint64_t value;
    int8_t status;
    if (choice == 1) {
        value = addressInfo->val1.value;
        status = addressInfo->confidence.read();
    } else if (choice == 2) {
        value = addressInfo->val2.value;
        status = addressInfo->confidence.read();
    } else if (choice == 3) {
        value = addressInfo->val3.value;
        status = addressInfo->confidence.read();
    } else {
        value = 0;
        status = -1;
    }
    DPRINTF(LVP, "Value for address %x is %llx\n", loadAddr, value);
    DPRINTF(LVP, "Status for address %x is %i\n", loadAddr, status - firstConst);
    ++predictionsMade;
    return LVPredUnit::lvpReturnValues(value, status - firstConst);
}

uint64_t FA3P::getValuePredicted(Addr loadAddr) 
{
    predictor &threadPred = threadPredictors[0]; // Assuming single-threaded for now

    LVTEntry *addressInfo = NULL;
    bool foundAddress = false;
    for (int i=0; i < tableEntries; i++) {
        if (threadPred.LVT[i].tag == loadAddr) {
            addressInfo = &threadPred.LVT[i];
            foundAddress = true;
        }
        else if (threadPred.LVT[i].tag == 0) {
            threadPred.LVT[i].tag = loadAddr;
            addressInfo = &threadPred.LVT[i];
            foundAddress = true;
        }
    }

    if (!foundAddress) {
        panic("Asked for predicted value, but none exists\n");
    }

    uint8_t choice = threadPred.choice[addressInfo->history.read()];
    DPRINTF(LVP, "Choice %i selected for address %x based on history %x\n", choice, loadAddr, addressInfo->history.read());
    uint64_t value;
    int8_t status;
    if (choice == 1) {
        value = addressInfo->val1.value;
        status = addressInfo->confidence.read();
    } else if (choice == 2) {
        value = addressInfo->val2.value;
        status = addressInfo->confidence.read();
    } else if (choice == 3) {
        value = addressInfo->val3.value;
        status = addressInfo->confidence.read();
    } else {
        value = 0;
        status = -1;
    }
    DPRINTF(LVP, "Value for address %x is %llx\n", loadAddr, value);
    DPRINTF(LVP, "Status for address %x is %i\n", loadAddr, status - firstConst);
    ++predictionsMade;
    return value;
}

bool FA3P::processPacketRecieved(TheISA::PCState pc, StaticInstPtr inst, uint64_t value, ThreadID tid, uint64_t prediction, int8_t confidence, unsigned cyclesElapsed, unsigned currentCycle)
{
	// New stats time
	if (inst->isLoad()) {
		finishedLoads++;
		totalLoadLatency += cyclesElapsed;
	} else if (inst->isInteger()) {
		finishedArithmetic++;
		totalArithmeticLatency += cyclesElapsed;
	} else {
		panic("unrecognized inst returned value to LVP\n");
	}
	

    DPRINTF(LVP, "Inst %s called processPacketRecieved\n", inst->disassemble(pc.instAddr()));
    Addr loadAddr = pc.instAddr();
    DPRINTF(LVP, "Value %llx predicted for address %x with confidence %i\n", prediction, loadAddr, confidence);
    predictor &threadPred = threadPredictors[tid];

    LVTEntry *addressInfo = NULL;
    bool foundAddress = false;
    for (int i=0; i < tableEntries; i++) {
        if (threadPred.LVT[i].tag == loadAddr) {
            addressInfo = &threadPred.LVT[i];
            foundAddress = true;
	    addressInfo->averageCycles = ((addressInfo->averageCycles * addressInfo->numUses) + cyclesElapsed + 1)/(addressInfo->numUses + 1);
	    addressInfo->numUses++;
        }
    }

    uint64_t responseVal = value;
/**
    if (pkt->isResponse()) {
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
            if (foundAddress) {
                addressInfo->history.update(0);
            }
            bool noMisPred = confidence < 0;
            if (!noMisPred) {
                DPRINTF(LVP, "MISPREDICTION DETECTED for address %x\n", loadAddr);
                if (foundAddress) {
                    addressInfo->confidence.reset();
                    while (addressInfo->confidence.read() < resetTo) {
                        addressInfo->confidence.increment();
                    }
                }
                ++incorrectUsed;
                missCount++;
                if (missCount >= missThreshold) {
                    if (dynamicThreshold) {
                        firstConst++;
                        ++constIncrement;
                    }
                    missCount = 0;
                }
            } else {
                ++incorrectNotUsed;
            }
            return noMisPred;
        }
**/

        DPRINTF(LVP, "Value %llx recieved for address %x\n", responseVal, loadAddr);

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
                    if (dynamicThreshold) {
                        firstConst--;
                        ++constDecrement;
                    }
                    hitCount = 0;
                }
            } else {
                // Incorrect
                ++incorrectNotUsed;
            }
        }

        bool misPred = (confidence >= 0) && (prediction != responseVal);

        if (foundAddress) {
            //if ((prediction != responseVal) && ((addressInfo.replaceTag.read() > 2) || (addressInfo.tag == 0)) && (addressInfo.tag != tag)) {
            //	addressInfo.val1.value = responseVal;
            //	addressInfo.val1.count.reset();
            //	addressInfo.val1.count.increment();
            //	addressInfo.val2.value = 0;
            //	addressInfo.val2.count.reset();
            //	addressInfo.val3.value = 0;
            //	addressInfo.val3.count.reset();
            //	addressInfo.tag = tag;
            //	addressInfo.replaceTag.reset();
            //	addressInfo.history.reset();
            //	addressInfo.history.update(1);
            //} else {
                // UPDATE THE PREDICTOR
                if (prediction == responseVal) {
                    DPRINTF(LVP, "Correct prediction :)\n");
                    addressInfo->confidence.increment();
                } else {
                    for (int i = 0; i < decrementBy; i++) {
                        addressInfo->confidence.decrement();
                    }
                }
                if (responseVal == addressInfo->val1.value) {
                    threadPred.choice[addressInfo->history.read()] = 1;
                    addressInfo->history.update(1);
                    addressInfo->val1.count.increment();
                } else if (responseVal == addressInfo->val2.value) {
                    threadPred.choice[addressInfo->history.read()] = 2;
                    addressInfo->history.update(2);
                    addressInfo->val2.count.increment();
                } else if (responseVal == addressInfo->val3.value) {
                    threadPred.choice[addressInfo->history.read()] = 3;
                    addressInfo->history.update(3);
                    addressInfo->val3.count.increment();
                } else {
                    threadPred.choice[addressInfo->history.read()] = 0;
                    // Any candidates for eviction?
                    // Value can be evicted iff it's count is < 1
                    // Evict the smallest value if possible
                    unsigned c1 = addressInfo->val1.count.read();
                    unsigned c2 = addressInfo->val2.count.read();
                    unsigned c3 = addressInfo->val3.count.read();
                    if ((c1 <= 0) && (c1 <= c2) && (c1 <= c3)) {
                        // Evicting val1
                        addressInfo->val1.value = responseVal;
                        addressInfo->val1.count.reset();
                        addressInfo->val1.count.increment();
                        // addressInfo->val2.count.decrement();
                        // addressInfo->val3.count.decrement();
                        // threadPred.choice[addressInfo.history.read()] = 1;
                        addressInfo->history.update(1);
                    } else if ((c2 <= 0) && (c2 <= c1) && (c2 <= c3)) {
                        // Evicting val2
                        addressInfo->val2.value = responseVal;
                        addressInfo->val2.count.reset();
                        addressInfo->val2.count.increment();
                        // addressInfo->val1.count.decrement();
                        // addressInfo->val3.count.decrement();
                        // threadPred.choice[addressInfo.history.read()] = 2;
                        addressInfo->history.update(2);
                    } else if ((c3 <= 0) && (c3 <= c1) && (c3 <= c2)) {
                        // Evicting val3
                        addressInfo->val3.value = responseVal;
                        addressInfo->val3.count.reset();
                        addressInfo->val3.count.increment();
                        // addressInfo->val1.count.decrement();
                        // addressInfo->val2.count.decrement();
                        // threadPred.choice[addressInfo.history.read()] = 3;
                        addressInfo->history.update(3);
                    } else {
                        // If no candidates for eviction, decrease all counts slightly
                        // Keeps predictor from getting stuck with old values
                        addressInfo->val1.count.decrement();
                        addressInfo->val2.count.decrement();
                        addressInfo->val3.count.decrement();
                        addressInfo->history.update(0);
                    }
                }
            //}
        }

        if (misPred) {
            DPRINTF(LVP, "MISPREDICTION DETECTED for address %x\n", loadAddr);
            if (foundAddress) {
                addressInfo->confidence.reset();
                while (addressInfo->confidence.read() < resetTo) {
                    addressInfo->confidence.increment();
                }
            }
            missCount++;
            if (missCount >= missThreshold) {
                if (dynamicThreshold) {
                    firstConst++;
                    ++constIncrement;
                }
                missCount = 0;
            }
        }
        return !misPred;
}
