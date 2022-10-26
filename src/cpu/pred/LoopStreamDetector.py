
from m5.SimObject import SimObject
from m5.params import *

class LoopStreamDetector(SimObject):
    type = 'LoopStreamDetector'
    cxx_class = 'LSDUnit'
    cxx_header = "cpu/pred/lsd_unit.hh"

    lsdLength = Param.Unsigned(64, "Number of PCs to track")
