from m5.SimObject import SimObject
from m5.params import *

class TraceBasedGraph(SimObject):
        type = 'TraceBasedGraph'
        cxx_class = 'TraceBasedGraph'
        cxx_header = 'arch/x86/superop/trace_based_graph.hh'
        usingControlTracking = Param.Bool(0, "Tracking control dependencies to optimize across?");
	connectionCount = Param.Unsigned(4096, "Number of connections to track at a time")
