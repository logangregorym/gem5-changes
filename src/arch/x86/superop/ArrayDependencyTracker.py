from m5.SimObject import SimObject
from m5.params import *

class ArrayDependencyTracker(SimObject):
        type = 'ArrayDependencyTracker'
        cxx_class = 'ArrayDependencyTracker'
        cxx_header = 'arch/x86/superop/array_dependency_tracker.hh'
	maxRecursiveDepth = Param.Unsigned(8, "Maximum depth to recurse when measuring dependency chains")
        usingControlTracking = Param.Bool(0, "Tracking control dependencies to optimize across?");
