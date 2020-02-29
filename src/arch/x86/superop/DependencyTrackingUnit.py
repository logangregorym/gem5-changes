# Will have to ask what I'm supposed to put here

from m5.SimObject import SimObject
from m5.params import *

class DependencyTrackingUnit(SimObject):
    type = 'DependencyTrackingUnit'
    cxx_class = 'DependencyTrackingUnit'
    cxx_header = "arch/x86/superop/dependency_tracking_unit.hh"
