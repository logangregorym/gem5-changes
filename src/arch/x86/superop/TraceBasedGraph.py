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
        specCacheNumWays = Param.Unsigned(8, "Confidence threshold to make a prediction during superoptmization")
        specCacheNumSets = Param.Unsigned(32, "Confidence threshold to make a prediction during superoptmization")
        specCacheNumUops = Param.Unsigned(6, "Confidence threshold to make a prediction during superoptmization")
        numOfTracePredictionSources = Param.Unsigned(4, "Confidence threshold to make a prediction during superoptmization")
        debugTraceGen = Param.Unsigned(0, "Trace id to generate logs for")

