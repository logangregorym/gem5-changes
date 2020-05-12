/*
 * Will have to ask what I'm supposed to  put here
 */

#ifndef __CPU_PRED_LVPRED_UNIT_IMPL_HH__
#define __CPU_PRED_LVPRED_UNIT_IMPL_HH__

#include "cpu/pred/lvpred_unit.hh"
#include "debug/LVP.hh"

LVPredUnit::LVPredUnit(const Params *params)
    : SimObject(params),
      firstConst(params->constantThreshold),
      dynamicThreshold(params->dynamicThreshold),
      decrementBy(params->decrementBy),
      resetTo(params->resetTo),
      missThreshold(params->missThreshold),
      hitThreshold(params->hitThreshold),
      resetDelay(params->resetDelay),
      predictStage(params->predictStage),
      predictingArithmetic(params->predictingArithmetic),
      numThreads(params->numThreads)
{}

void LVPredUnit::regStats()
{
    lvLookups
        .name(name() + ".lvLookups")
        .desc("Number of LVP Unit lookups")
        ;
    correctUsed
        .name(name() + ".correctUsed")
        .desc("Number of correct predictions used")
        ;
    incorrectUsed
        .name(name() + ".incorrectUsed")
        .desc("Number of incorrect predictions used")
        ;
    correctNotUsed
        .name(name() + ".correctNotUsed")
        .desc("Number of correct predictions not used")
        ;
    incorrectNotUsed
        .name(name() + ".incorrectNotUsed")
        .desc("Number of incorrect predictions not used")
        ;
    totalCorrect
        .name(name() + ".totalCorrect")
        .desc("Total predictable load values")
        ;
    totalCorrect = correctUsed + correctNotUsed;
    totalIncorrect
        .name(name() + ".totalIncorrect")
        .desc("Total predictable load values")
        ;
    totalIncorrect = incorrectUsed + incorrectNotUsed;
    totalUsed
        .name(name() + ".totalUsed")
        .desc("Total value predictions used")
        ;
    totalUsed = correctUsed + incorrectUsed;
    totalNotUsed
        .name(name() + ".totalNotUsed")
        .desc("Total value predictions not used")
        ;
    totalNotUsed = correctNotUsed + incorrectNotUsed;
    accuracy
        .name(name() + ".accuracy")
        .desc("Percentage of used predictions that were accurate")
        .precision(6);
    accuracy = (correctUsed / totalUsed) * 100;
    coverage
        .name(name() + ".coverage")
        .desc("Percentage of accurate predictions that were used")
        .precision(6);
    coverage = (correctUsed / totalCorrect) * 100;
    cyclesSaved
        .name(name() + ".cyclesSaved")
        .desc("Number of memory access cycles skipped")
        ;
    ripRelNum
        .name(name() + ".ripRelNum")
        .desc("Number of rip relative loads")
        ;
    ripRelPercent
        .name(name() + ".ripRelPercent")
        .desc("Percent of loads which are rip relative")
        .precision(6);
    ripRelPercent = (ripRelNum / lvLookups) * 100;
    largeValueLoads
        .name(name() + ".largeValueLoads")
        .desc("Number of load values > 8 bytes")
        ;
    basicChosen
        .name(name() + ".basicChosen")
        .desc("Times hybrid chose basic")
        ;
    strideHistChosen
        .name(name() + ".strideHistChosen")
        .desc("Times hybrid chose strideHist")
        ;
    periodicChosen
        .name(name() + ".periodicChosen")
        .desc("Times hybrid chose periodic")
        ;
    constIncrement
        .name(name() + ".constIncrement")
        .desc("Times firstCont was incremented")
        ;
    constDecrement
        .name(name() + ".constDecrement")
        .desc("Times firstConst was decremented")
        ;
    tagMismatch
        .name(name() + ".tagMismatch")
        .desc("Times an address indexed to a different tag")
        ;
    aliasPercent
        .name(name() + ".aliasPercent")
        .desc("Percent of loads whose tags don't match")
        .precision(6);
    aliasPercent = (tagMismatch / lvLookups) * 100;
    predictedPercent
        .name(name() + ".predictedPercent")
        .desc("Percent of loads we forward a prediction for")
        .precision(6);
    predictedPercent = (totalUsed / lvLookups) * 100;
    predictionsMade
        .name(name() + ".predictionsMade")
        .desc("Number of predictions made")
        ;
	finishedLoads
		.name(name() + ".finishedLoads")
		.desc("Number of loads completed")
		;
	finishedArithmetic
		.name(name() + ".finishedArithmetic")
		.desc("Number of arithmetic insts completed")
		;
	totalLoadLatency
		.name(name() + ".totalLoadLatency")
		.desc("Total cycles between a load being fetched and finishing")
		;
	totalArithmeticLatency
		.name(name() + ".totalArithmeticLatency")
		.desc("Total cycles between an arithmetic inst being fetched and finishing")
		;
	averageLoadLatency
		.name(name() + ".averageLoadLatency")
		.desc("Average cycles between a load being fetched and finishing")
		.precision(6);
	averageLoadLatency = (totalLoadLatency / finishedLoads);
	averageArithmeticLatency
		.name(name() + ".averageArithmeticLatency")
		.desc("Average cycles between an arithmetic inst being fetched and finishing")
		.precision(6);
	averageArithmeticLatency = (totalArithmeticLatency / finishedArithmetic);
}

#endif

