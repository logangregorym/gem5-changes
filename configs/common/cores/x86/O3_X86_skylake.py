# Copyright (c) 2012 The Regents of The University of Michigan
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Ashish Venkat


from m5.objects import *

# Simple ALU Instructions have a latency of 1
class O3_X86_skylake_Simple_Int(FUDesc):
    opList = [ OpDesc(opClass='IntAlu', opLat=1) ]
    count = 6

# Complex ALU instructions have a variable latencies
class O3_X86_skylake_Complex_Int(FUDesc):
    opList = [ OpDesc(opClass='IntMult', opLat=4, pipelined=True),
               OpDesc(opClass='IntDiv', opLat=5, pipelined=False) ]
    count = 1


# Floating point and SIMD instructions
class O3_X86_skylake_FP(FUDesc):
    opList = [ OpDesc(opClass='SimdAdd', opLat=1),
               OpDesc(opClass='SimdAddAcc', opLat=1),
               OpDesc(opClass='SimdAlu', opLat=1),
               OpDesc(opClass='SimdCmp', opLat=1),
               OpDesc(opClass='SimdCvt', opLat=1),
               OpDesc(opClass='SimdMisc', opLat=3),
               OpDesc(opClass='SimdMult',opLat=5),
               OpDesc(opClass='SimdMultAcc',opLat=5),
               OpDesc(opClass='SimdShift',opLat=4),
               OpDesc(opClass='SimdShiftAcc', opLat=1),
               OpDesc(opClass='SimdSqrt', opLat=5),
               OpDesc(opClass='SimdFloatAdd',opLat=1),
               OpDesc(opClass='SimdFloatAlu',opLat=1),
               OpDesc(opClass='SimdFloatCmp', opLat=1),
               OpDesc(opClass='SimdFloatCvt', opLat=3),
               OpDesc(opClass='SimdFloatDiv', opLat=3),
               OpDesc(opClass='SimdFloatMisc', opLat=3),
               OpDesc(opClass='SimdFloatMult', opLat=3),
               OpDesc(opClass='SimdFloatMultAcc',opLat=1),
               OpDesc(opClass='SimdFloatSqrt', opLat=9),
               OpDesc(opClass='FloatAdd', opLat=3),
               OpDesc(opClass='FloatCmp', opLat=3),
               OpDesc(opClass='FloatCvt', opLat=3),
               OpDesc(opClass='FloatDiv', opLat=9, pipelined=False),
               OpDesc(opClass='FloatSqrt', opLat=33, pipelined=False),
               OpDesc(opClass='FloatMult', opLat=4),
               OpDesc(opClass='FloatMultAcc', opLat=5),
               OpDesc(opClass='FloatMisc', opLat=3) ]
    count = 3


# Load/Store Units
class O3_X86_skylake_Load(FUDesc):
    opList = [ OpDesc(opClass='MemRead',opLat=3),
               OpDesc(opClass='FloatMemRead',opLat=3) ]
    count = 2

class O3_X86_skylake_Store(FUDesc):
    opList = [ OpDesc(opClass='MemWrite',opLat=2),
               OpDesc(opClass='FloatMemWrite',opLat=2) ]
    count = 2

# Functional Units for this CPU
class O3_X86_skylake_FUP(FUPool):
    FUList = [O3_X86_skylake_Simple_Int(), O3_X86_skylake_Complex_Int(),
              O3_X86_skylake_Load(), O3_X86_skylake_Store(), O3_X86_skylake_FP()]

# Bi-Mode Branch Predictor
class O3_X86_skylake_BP(LTAGE):
    BTBEntries = 4096
    BTBTagSize = 18
    RASSize = 64
    instShiftAmt = 2

class O3_X86_skylake_1(DerivO3CPU):
    LQEntries = 72
    SQEntries = 56
    LSQDepCheckShift = 0
    LFSTSize = 1024
    SSITSize = 1024
    decodeToFetchDelay = 1
    renameToFetchDelay = 1
    iewToFetchDelay = 1
    commitToFetchDelay = 1
    renameToDecodeDelay = 1
    iewToDecodeDelay = 1
    commitToDecodeDelay = 1
    iewToRenameDelay = 1
    commitToRenameDelay = 1
    commitToIEWDelay = 1
    fetchWidth = 8
    fetchBufferSize = 16
    fetchToDecodeDelay = 3
    decodeWidth = 8
    decodeToRenameDelay = 2
    renameWidth = 8
    renameToIEWDelay = 1
    issueToExecuteDelay = 1
    dispatchWidth = 6
    issueWidth = 6
    wbWidth = 6
    fuPool = O3_X86_skylake_FUP()
    iewToCommitDelay = 1
    renameToROBDelay = 1
    commitWidth = 6
    squashWidth = 6
    trapLatency = 13
    backComSize = 6
    forwardComSize = 6
    numPhysIntRegs =260
    numPhysFloatRegs = 168
    numPhysVecRegs = 168
    numIQEntries = 64
    numROBEntries = 224

    switched_out = False
    branchPred = O3_X86_skylake_BP()

# Instruction Cache
class O3_X86_skylake_ICache(Cache):
    response_latency = 1
    tag_latency = 1
    data_latency = 1
    response_latency = 1
    mshrs = 2
    tgts_per_mshr = 8
    size = '32kB'
    assoc = 8
    is_read_only = True
    writeback_clean = True

# Data Cache
class O3_X86_skylake_DCache(Cache):
    tag_latency = 4
    data_latency = 4
    response_latency = 4
    mshrs = 6
    tgts_per_mshr = 8
    size = '32kB'
    assoc = 8
    write_buffers = 10
    writeback_clean = True

# TLB Cache
# Use a cache as a L2 TLB
class O3_X86_skylakeWalkCache(Cache):
    data_latency = 2
    tag_latency = 2
    response_latency = 1
    mshrs = 6
    tgts_per_mshr = 8
    size = '1kB'
    assoc = 4
    write_buffers = 16
    is_read_only = True
    # Writeback clean lines as well
    writeback_clean = True


# L2 Cache
class O3_X86_skylakeL2(Cache):
    tag_latency = 42
    data_latency = 42
    response_latency =42
    mshrs = 16
    tgts_per_mshr = 8
    size = '8MB'
    assoc = 16
    write_buffers = 16
    prefetch_on_access = True
    clusivity = 'mostly_excl'
    # Simple stride prefetcher
    prefetcher = StridePrefetcher(degree=8, latency = 1)
    tags = BaseSetAssoc()
    repl_policy = RandomRP()
