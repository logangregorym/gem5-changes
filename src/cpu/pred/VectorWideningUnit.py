
from m5.SimObject import SimObject
from m5.params import *

class VectorWideningUnit(SimObject):
    type = 'VectorWideningUnit'
    cxx_class = 'VWUnit'
    cxx_header = "cpu/pred/vw_unit.hh"