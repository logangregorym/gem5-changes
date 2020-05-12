# Will have to ask what I'm supposed to put here

from m5.SimObject import SimObject
from m5.params import *

class LoadValuePredictor(SimObject):
    type = 'LoadValuePredictor'
    cxx_class = 'LVPredUnit'
    cxx_header = "cpu/pred/lvpred_unit.hh"

    lvpredType = Param.String("basic", "Load Value predictor type ('basic', 'strideHist', '3period', 'hybrid', 'fa3p', 'none')")
    tableEntries = Param.Unsigned(256, "Number of entries in the lookup table")
    constantThreshold = Param.Int(10, "First usable prediction")
    dynamicThreshold = Param.Bool(1, "Learn the constant threshold dynamically?")
    satCounterBits = Param.Unsigned(5, "Bits in the saturating counters")
    decrementBy = Param.Unsigned(10, "Confidence decrement on misprediction")
    resetTo = Param.Unsigned(0, "Confidence reset on misprediction")
    initialPredictionQuality = Param.Unsigned(0, "Default value of saturating counters")
    resetDelay = Param.Unsigned(128, "Cycles to remain inactive after a misprediction")
    historyLength = Param.Unsigned(4, "Number of past strides to track")
    historyEntryBits = Param.Unsigned(4, "Bits per stride in history")
    numThreads = Param.Unsigned(1, "Number of threads")
    missThreshold = Param.Unsigned(150, "Misses allowed before confidence threshold increments")
    hitThreshold = Param.Unsigned(300, "Unused hits allowed before confidence threshold decrements")
    predictingArithmetic = Param.Bool(0, "Whether to predict arithmetic insts as well as loads")
    predictStage = Param.Unsigned(3, "When to make a prediction (1-fetch, 2-iew, 3-both)")
