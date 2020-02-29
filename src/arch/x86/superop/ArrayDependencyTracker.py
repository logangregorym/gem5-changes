from m5.SimObject import SimObject
from m5.params import *

class ArrayDependencyTracker(SimObject):
        type = 'ArrayDependencyTracker'
        cxx_class = 'ArrayDependencyTracker'
        cxx_header = 'arch/x86/superop/array_dependency_tracker.hh'
