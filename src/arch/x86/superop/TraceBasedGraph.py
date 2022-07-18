from m5.SimObject import SimObject
from m5.params import *

class TraceBasedGraph(SimObject):
        type = 'TraceBasedGraph'
        cxx_class = 'TraceBasedGraph'
        cxx_header = 'arch/x86/superop/trace_based_graph.hh'
        usingControlTracking = Param.Bool(0, "Tracking control dependencies to optimize across?");
        usingCCTracking = Param.Bool(0, "Tracking condition codes?");
        connectionCount = Param.Unsigned(4096, "Number of connections to track at a time")
        predictionConfidenceThreshold = Param.Unsigned(5, "Confidence threshold to make a prediction during superoptmization")
        controlPredictionConfidenceThreshold = Param.Unsigned(5, "Confidence threshold to make a prediction during superoptmization")
        specCacheNumWays = Param.Unsigned(8, "Number of ways in the spec cache")
        specCacheNumSets = Param.Unsigned(32, "Number of sets in the spec cache")
        specCacheNumUops = Param.Unsigned(6, "Number of uops per way in the spec cache")
        specCacheNumTicks = Param.Unsigned(10, "Number of cycles between ticking all hotness counters")
        numOfTracePredictionSources = Param.Unsigned(4, "Confidence threshold to make a prediction during superoptmization")
        debugTraceGen = Param.Unsigned(0, "Trace id to generate logs for")
        disableSuperProp = Param.Bool(0, "Disables all SCC propogation, prediction, and folding for all instructions except mov, movi, and limm")
        disableSuperSimple = Param.Bool(0, "Disables all SCC propogation, prediction, and folding for mov, movi, and limm ")
        constantWidth = Param.Unsigned(64, "Constant width for live out inlining")

