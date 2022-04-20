/**
 * Will have to ask what I'm supposed to put here
 */

#include "base/intmath.hh"

// #include "base/misc.hh"
#include "cpu/pred/fa3p.hh"
#include "debug/LVP.hh"
#include "debug/FA3P.hh"
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
    //assert(!threadPredictors.empty());
    DPRINTF(LVP, "Tables are set up (fa3period)\n");
}

void FA3P::setUpTables(const Params *params)
{
    if (!isPowerOf2(tableEntries))
        fatal("Invalid table size! Must be power of 2\n");
    //for (int i = 0; i < numThreads; i++) {
    DPRINTF(LVP, "Building LVT with %i entries for thread 0\n", tableEntries);
    vector<LVTEntry> newLVT = vector<LVTEntry>(tableEntries, LVTEntry(ValueWithCount(), ValueWithCount(), ValueWithCount(), History(2, historyLength), BigSatCounter(scBits, initialPred), -1, false));
    uint64_t choiceSize = power(2, (historyLength * 2)); // 2 bits each
    DPRINTF(LVP, "Building choice table with %i entries for thread 0\n", choiceSize);
    vector<uint8_t> newChoice = vector<uint8_t>(choiceSize, 0);
    predictor = Predictor(newLVT, newChoice);
        // cout << tableEntries << " " << newLVT.size() << endl;
    //}
}

unsigned FA3P::getConfidence(TheISA::PCState pc) {
    assert(0);
    int16_t upc = (int16_t)pc.microPC();
    Addr addr = pc.instAddr();
    for (int idx=0; idx < tableEntries; idx++) {
        if (predictor.LVT[idx].tag == addr && predictor.LVT[idx].tag == addr && predictor.LVT[idx].micropc == upc) {
            return predictor.LVT[idx].confidence.read();// - firstConst;
        }
    }
    return 0;
}

unsigned FA3P::getDelay(TheISA::PCState pc) {
    //assert(0);
    int16_t upc = (int16_t)pc.microPC();
    Addr addr = pc.instAddr();
    for (int idx=0; idx < tableEntries; idx++) {
        if (predictor.LVT[idx].tag == addr && predictor.LVT[idx].tag == addr && predictor.LVT[idx].micropc == upc) {
            return predictor.LVT[idx].averageCycles;
        }
    }
    return 0;
}


uint64_t FA3P::getValuePredicted(TheISA::PCState pc) 
{

    assert(0);
    Addr addr = pc.instAddr();
    int16_t upc = (int16_t)pc.microPC(); 


    bool foundAddress = false;
    int idx;
    for (idx=0; idx < tableEntries; idx++) {
        if (predictor.LVT[idx].valid && predictor.LVT[idx].tag == addr && predictor.LVT[idx].micropc == upc) {
            foundAddress = true;
            break;
        }    
    }

    if (!foundAddress) {
        panic("Asked for predicted value, but none exists\n");
    }

    uint8_t choice = predictor.choice[predictor.LVT[idx].history.read()];
    DPRINTF(LVP, "Choice %i selected for address %x based on history %x\n", choice, addr, predictor.LVT[idx].history.read());
    uint64_t value;
    int8_t status;
    if (choice == 1 && predictor.LVT[idx].val1.valid) {
        value = predictor.LVT[idx].val1.value;
        status = predictor.LVT[idx].confidence.read();
    } else if (choice == 2 && predictor.LVT[idx].val2.valid) {
        value = predictor.LVT[idx].val2.value;
        status = predictor.LVT[idx].confidence.read();
    } else if (choice == 3 && predictor.LVT[idx].val3.valid) {
        value = predictor.LVT[idx].val3.value;
        status = predictor.LVT[idx].confidence.read();
    } else {
        value = 0;
        status = -1;
    }
    DPRINTF(LVP, "Value for address %x is %llx\n", addr, value);
    DPRINTF(LVP, "Status for address %x is %i\n", addr, status);// - firstConst);

    return value;
}

LVPredUnit::lvpReturnValues FA3P::makePrediction(TheISA::PCState pc, ThreadID tid, unsigned currentCycle)
{

    ++lvLookups;

    Addr addr = pc.instAddr();
    int16_t upc = (int16_t)pc.microPC(); 
   


    bool foundAddress = false;
    int idx;
    for (idx=0; idx < tableEntries; idx++) {
        if (predictor.LVT[idx].valid && predictor.LVT[idx].tag == addr && predictor.LVT[idx].micropc == upc) {
            foundAddress = true;
            DPRINTF(FA3P, "makePrediction: Entry %d is found for address %#x:%d!\n", idx, addr, upc);
            break;
        }    
    }

    bool foundEmptySpace = false;
    if (!foundAddress)
    {
        for (idx=0; idx < tableEntries; idx++) {
            if (!predictor.LVT[idx].valid) {
                predictor.LVT[idx].history.reset();
                predictor.LVT[idx].confidence.reset();
                predictor.LVT[idx].tag = addr;
                predictor.LVT[idx].micropc = upc;
                predictor.LVT[idx].val1.reset();
                predictor.LVT[idx].val2.reset();
                predictor.LVT[idx].val3.reset();
                predictor.LVT[idx].valid = true;
                foundEmptySpace = true;
                DPRINTF(FA3P, "makePrediction: Entry %d is allocated for address %#x:%d!\n", idx, addr, upc);
                break;
            }    
        }
    }


    int replacementIdx = 0;
    if (!foundAddress && !foundEmptySpace) {
        tagMismatch++;
        // find a replacement based on LRU policy
        for (int i = 0; i < tableEntries; i++) {
            if (predictor.LVT[i].lastUsed < predictor.LVT[replacementIdx].lastUsed) {
                //LRU = &threadPredictors[tid].LVT[i];
                replacementIdx = i;
            }
        }
        DPRINTF(FA3P, "makePrediction: Entry %d is evicted with address %#x:%d!\n", replacementIdx, predictor.LVT[replacementIdx].tag, predictor.LVT[replacementIdx].micropc);
        predictor.LVT[replacementIdx].history.reset();
        predictor.LVT[replacementIdx].confidence.reset();
        predictor.LVT[replacementIdx].tag = addr;
        predictor.LVT[replacementIdx].micropc = upc;
        predictor.LVT[replacementIdx].val1.reset();
        predictor.LVT[replacementIdx].val2.reset();
        predictor.LVT[replacementIdx].val3.reset();
        predictor.LVT[replacementIdx].valid = true;
        DPRINTF(FA3P, "makePrediction: Entry %d is replaced for address %#x:%d!\n", replacementIdx, addr, upc);
        //addressInfo = LRU;
    }

    if (foundAddress || foundEmptySpace)    idx = idx;
    else                                    idx = replacementIdx;

    predictor.LVT[idx].lastUsed = currentCycle;

    uint8_t choice = predictor.choice[predictor.LVT[idx].history.read()];
    DPRINTF(FA3P, "makePrediction: Choice %i selected for address %#x:%d based on history %d\n", choice, addr, upc, predictor.LVT[idx].history.read());

    // if the entry is allocated now or is evicted, then base on the choice select a val (1,2, or 3)
    // if (!foundAddress || !foundEmptySpace)
    // {

    // }

    uint64_t value;
    int8_t confidence;
    if (choice == 1 && predictor.LVT[idx].val1.valid) {
        value = predictor.LVT[idx].val1.value;
        confidence = predictor.LVT[idx].confidence.read();
    } else if (choice == 2 && predictor.LVT[idx].val2.valid) {
        value = predictor.LVT[idx].val2.value;
        confidence = predictor.LVT[idx].confidence.read();
    } else if (choice == 3 && predictor.LVT[idx].val3.valid) {
        value = predictor.LVT[idx].val3.value;
        confidence = predictor.LVT[idx].confidence.read();
    } else {
        //if the entry is allocated now or is evicted, then just send a random value (i.e., 0) with lowest confidence. Later we will find a spot for this prediction
        value = 0;
        confidence = 0;
    }

    DPRINTF(FA3P, "makePrediction: Address: %#x Value: %llx Confidence: %d\n", addr, value, confidence /*- firstConst*/);
    ++predictionsMade;

    return LVPredUnit::lvpReturnValues(value, confidence/* - firstConst*/, getDelay(pc));
}

// this function should only be called for instructions coming from uop cache
bool FA3P::makePredictionForTraceGenStage(Addr addr, uint16_t upc, ThreadID tid , LVPredUnit::lvpReturnValues& ret)
{


    //LVTEntry *addressInfo = NULL;
    bool foundAddress = false;
    int idx;
    for (idx = 0; idx  < tableEntries; idx++) {
        if (predictor.LVT[idx].valid && predictor.LVT[idx].tag == addr && predictor.LVT[idx].micropc == upc) {
            foundAddress = true;
            break;
        }
    }

    if (!foundAddress)
    {
        return false;
    }




    uint8_t choice = predictor.choice[predictor.LVT[idx].history.read()];
    DPRINTF(LVP, "Choice %i selected for address %x based on history %x\n", choice, addr, predictor.LVT[idx].history.read());
    uint64_t value;
    int8_t confidence;
    if (choice == 1 && predictor.LVT[idx].val1.valid) {
        value = predictor.LVT[idx].val1.value;
        confidence = predictor.LVT[idx].confidence.read();
    } else if (choice == 2 && predictor.LVT[idx].val2.valid) {
        value = predictor.LVT[idx].val2.value;
        confidence = predictor.LVT[idx].confidence.read();
    } else if (choice == 3 && predictor.LVT[idx].val3.valid) {
        value = predictor.LVT[idx].val3.value;
        confidence = predictor.LVT[idx].confidence.read();
    } else {
        return false;
    }
    DPRINTF(LVP, "Value for address %x is %llx\n", addr, value);
    DPRINTF(LVP, "confidence for address %x is %i\n", addr, confidence /*- firstConst*/);

    ret = LVPredUnit::lvpReturnValues(value, confidence /*- firstConst*/, predictor.LVT[idx].averageCycles);

    return true;


}

// To not mess around with this logic, we will only update the confidence in Load Value Predictor in this fucntion. 
// The trace confidence is updated in IEW::squashDueToLoad function
bool FA3P::processPacketRecieved(TheISA::PCState pc, StaticInstPtr inst, uint64_t value, ThreadID tid, uint64_t prediction, int8_t confidence, unsigned cyclesElapsed, unsigned currentCycle)
{
    assert(inst);
    
    Addr addr = pc.instAddr();
    int16_t upc = (int16_t)pc.microPC(); 
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

    // don't predict for these types TODO: add more types gradually
    //assert(!inst->isStore());

    
    DPRINTF(FA3P, "processPacketRecieved: Inst %s called processPacketRecieved\n", inst->disassemble(pc.instAddr()));

    //LVTEntry *addressInfo = NULL;
    bool foundAddress = false;
    int idx = 0;
    for (idx = 0; idx < tableEntries; idx++) {
        if (predictor.LVT[idx].tag == addr && predictor.LVT[idx].tag == addr && predictor.LVT[idx].micropc == upc) {
            foundAddress = true;
	        predictor.LVT[idx].averageCycles = ((predictor.LVT[idx].averageCycles * predictor.LVT[idx].numUses) + cyclesElapsed + 1)/(predictor.LVT[idx].numUses + 1);
	        predictor.LVT[idx].numUses++;
            DPRINTF(FA3P, "processPacketRecieved: Entry %d is found for address %#x:%d!\n", idx, addr, upc);
            break;
        }
    }



    uint64_t responseVal = value;

    DPRINTF(FA3P, "processPacketRecieved: Address: %#x:%d Predicted Value %#llx Actual Value: %#llx Confidence %d\n", addr, upc, prediction, responseVal ,confidence);


    // Stats time!,
    if (confidence >= 5) {
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
                //if (dynamicThreshold) {
                    //firstConst--;
                //    ++constDecrement;
                //}
                hitCount = 0;
            }
        } else {
            // Incorrect
            ++incorrectNotUsed;
        }
    }



    bool misPred = (prediction != responseVal);

    if (foundAddress) {

        if (prediction == responseVal) {
            
            predictor.LVT[idx].confidence.increment();
            DPRINTF(FA3P, "processPacketRecieved: Correct prediction for entry %d at address %#x:%d! New confidence: %d\n", idx, addr, upc, predictor.LVT[idx].confidence.read());
        } 
        else 
        {
            //predictor.LVT[idx].confidence.decrement();
            //while (predictor.LVT[idx].confidence.read() >= 5) {
           //     predictor.LVT[idx].confidence.decrement();
            //}
            predictor.LVT[idx].confidence.decrement();
            predictor.LVT[idx].confidence.decrement();
            DPRINTF(FA3P, "processPacketRecieved: Missprediction for entry %d at address %#x:%d! New confidence: %d\n", idx, addr, upc, predictor.LVT[idx].confidence.read());
        }


        if (predictor.LVT[idx].val1.valid && (responseVal == predictor.LVT[idx].val1.value)) 
        {
            DPRINTF(FA3P, "processPacketRecieved: Value predicted for entry %d is found at VAL1!\n", idx);
            predictor.choice[predictor.LVT[idx].history.read()] = 1;
            predictor.LVT[idx].history.update(1);
            predictor.LVT[idx].val1.count.increment();
        } 
        else if (predictor.LVT[idx].val2.valid && (responseVal == predictor.LVT[idx].val2.value)) 
        {
            DPRINTF(FA3P, "processPacketRecieved: Value predicted for entry %d is found at VAL2!\n", idx);
            predictor.choice[predictor.LVT[idx].history.read()] = 2;
            predictor.LVT[idx].history.update(2);
            predictor.LVT[idx].val2.count.increment();
            
        } 
        else if (predictor.LVT[idx].val3.valid && (responseVal == predictor.LVT[idx].val3.value)) 
        {
            DPRINTF(FA3P, "processPacketRecieved: Value predicted for entry %d is found at VAL3!\n", idx);
            predictor.choice[predictor.LVT[idx].history.read()] = 3;
            predictor.LVT[idx].history.update(3);
            predictor.LVT[idx].val3.count.increment();
        } 
        else 
        {
            DPRINTF(FA3P, "processPacketRecieved: Can't find a valid value for entry %d! Predictor history is %d\n", idx, predictor.LVT[idx].history.read());
            predictor.choice[predictor.LVT[idx].history.read()] = 0;
            // Any candidates for eviction?
            // Value can be evicted iff it's count is < 1
            // Evict the smallest value if possible
            unsigned c1 = predictor.LVT[idx].val1.count.read();
            unsigned c2 = predictor.LVT[idx].val2.count.read();
            unsigned c3 = predictor.LVT[idx].val3.count.read();
            if ((c1 == 0) && (c1 <= c2) && (c1 <= c3)) {
                // Evicting val1
                DPRINTF(FA3P, "processPacketRecieved: Evicting VAL1 for entry %d!\n", idx);
                predictor.LVT[idx].val1.value = responseVal;
                predictor.LVT[idx].val1.count.reset();
                predictor.LVT[idx].val1.count.increment();
                predictor.LVT[idx].val1.valid = true;
                predictor.LVT[idx].history.update(1);
            } else if ((c2 == 0) && (c2 <= c1) && (c2 <= c3)) {
                // Evicting val2
                DPRINTF(FA3P, "processPacketRecieved: Evicting VAL2 for entry %d!\n", idx);
                predictor.LVT[idx].val2.value = responseVal;
                predictor.LVT[idx].val2.count.reset();
                predictor.LVT[idx].val2.count.increment();
                predictor.LVT[idx].val2.valid = true;
                predictor.LVT[idx].history.update(2);
            } else if ((c3 == 0) && (c3 <= c1) && (c3 <= c2)) {
                // Evicting val3
                DPRINTF(FA3P, "processPacketRecieved: Evicting VAL3 for entry %d!\n", idx);
                predictor.LVT[idx].val3.value = responseVal;
                predictor.LVT[idx].val3.count.reset();
                predictor.LVT[idx].val3.count.increment();
                predictor.LVT[idx].val3.valid = true;
                predictor.LVT[idx].history.update(3);
            } else {
                // If no candidates for eviction, decrease all counts slightly
                // Keeps predictor from getting stuck with old values
                DPRINTF(FA3P, "processPacketRecieved: Can't find a candidate for eviction for entry %d!\n", idx);
                predictor.LVT[idx].val1.count.decrement();
                predictor.LVT[idx].val2.count.decrement();
                predictor.LVT[idx].val3.count.decrement();
                predictor.LVT[idx].history.update(0);
            }
        }
            //}
    }


    
    return !misPred;
}
