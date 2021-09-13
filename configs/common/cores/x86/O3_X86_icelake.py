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
class O3_X86_icelake_Simple_Int(FUDesc):
    opList = [ OpDesc(opClass='IntAlu', opLat=1) ]
    count = 4

# Complex ALU instructions have a variable latencies
class O3_X86_icelake_Complex_Int(FUDesc):
    opList = [ OpDesc(opClass='IntMult', opLat=4, pipelined=True),
               OpDesc(opClass='IntDiv', opLat=5, pipelined=False) ]
    count = 2


# Floating point and SIMD instructions
class O3_X86_icelake_FP(FUDesc):
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
class O3_X86_icelake_Load(FUDesc):
    opList = [ OpDesc(opClass='MemRead',opLat=3),
               OpDesc(opClass='FloatMemRead',opLat=3) ]
    count = 2

class O3_X86_icelake_Store(FUDesc):
    opList = [ OpDesc(opClass='MemWrite',opLat=2),
               OpDesc(opClass='FloatMemWrite',opLat=2) ]
    count = 4

# Functional Units for this CPU
class O3_X86_icelake_FUP(FUPool):
    FUList = [O3_X86_icelake_Simple_Int(), O3_X86_icelake_Complex_Int(),
              O3_X86_icelake_Load(), O3_X86_icelake_Store(), O3_X86_icelake_FP()]

# Bi-Mode Branch Predictor
class O3_X86_icelake_BP(LTAGE):
    BTBEntries = 8192
    BTBTagSize = 18
    RASSize = 64
    instShiftAmt = 2

class O3_X86_icelake_1(DerivO3CPU):
    LQEntries = 128
    SQEntries = 72
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
    fetchWidth = 6
    fetchBufferSize = 32
    fetchToDecodeDelay = 3
    decodeWidth = 6
    decodeToRenameDelay = 2
    renameWidth = 6
    renameToIEWDelay = 1
    issueToExecuteDelay = 1
    dispatchWidth = 10
    issueWidth = 10
    wbWidth = 10
    fuPool = O3_X86_icelake_FUP()
    iewToCommitDelay = 1
    renameToROBDelay = 1
    commitWidth = 10
    squashWidth = 10
    trapLatency = 13
    backComSize = 10
    forwardComSize = 10
    numPhysIntRegs = 988
    numPhysFloatRegs = 288
    numPhysVecRegs = 288
    numIQEntries = 160
    numROBEntries = 352

    switched_out = False
    branchPred = O3_X86_icelake_BP()

# Instruction Cache
class O3_X86_icelake_ICache(Cache):
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
class O3_X86_icelake_DCache(Cache):
    tag_latency = 1
    data_latency = 1
    response_latency = 1
    mshrs = 6
    tgts_per_mshr = 8
    size = '48kB'
    assoc = 12
    write_buffers = 10
    writeback_clean = True

# TLB Cache
# Use a cache as a L2 TLB
class O3_X86_icelakeWalkCache(Cache):
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
class O3_X86_icelakeL2(Cache):
    tag_latency = 8
    data_latency = 8
    response_latency = 8
    mshrs = 16
    tgts_per_mshr = 8
    size = '512kB'
    assoc = 8
    write_buffers = 16
    prefetch_on_access = True
    clusivity = 'mostly_excl'
    # Simple stride prefetcher
    prefetcher = StridePrefetcher(degree=8, latency = 1)
    tags = BaseSetAssoc()
    repl_policy = RandomRP()

    # L3 Cache
class O3_X86_icelakeL3(Cache):
    tag_latency = 8
    data_latency = 8
    response_latency = 8
    mshrs = 512
    tgts_per_mshr = 20
    size = '2MB'
    assoc = 16
    write_buffers = 256
    prefetch_on_access = True
    clusivity = 'mostly_excl'
    # Simple stride prefetcher
    #prefetcher = StridePrefetcher(degree=8, latency = 1)
    tags = BaseSetAssoc()
    repl_policy = RandomRP()
