# Copyright (c) 2012 The Regents of The University of Michigan
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notalder, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notalder, this list of conditions and the following disclaimer in the
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
class O3_X86_alderlake_Simple_Int(FUDesc):
    opList = [ OpDesc(opClass='IntAlu', opLat=1) ]
    count = 5

# Complex ALU instructions have a variable latencies
class O3_X86_alderlake_Complex_Int(FUDesc):
    opList = [ OpDesc(opClass='IntMult', opLat=4, pipelined=True),
               OpDesc(opClass='IntDiv', opLat=5, pipelined=False) ]
    count = 2


# Floating point and SIMD instructions
class O3_X86_alderlake_FP(FUDesc):
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
class O3_X86_alderlake_Load(FUDesc):
    opList = [ OpDesc(opClass='MemRead',opLat=4),
               OpDesc(opClass='FloatMemRead',opLat=4) ]
    count = 3

class O3_X86_alderlake_Store(FUDesc):
    opList = [ OpDesc(opClass='MemWrite',opLat=2),
               OpDesc(opClass='FloatMemWrite',opLat=2) ]
    count = 4

# Functional Units for this CPU
class O3_X86_alderlake_FUP(FUPool):
    FUList = [O3_X86_alderlake_Simple_Int(), O3_X86_alderlake_Complex_Int(),
              O3_X86_alderlake_Load(), O3_X86_alderlake_Store(), O3_X86_alderlake_FP()]

# Bi-Mode Branch Predictor
class O3_X86_alderlake_BP(LTAGE):
    BTBEntries = 8192
    BTBTagSize = 18
    RASSize = 64
    instShiftAmt = 2

class O3_X86_alderlake_1(DerivO3CPU):
    LQEntries = 192
    SQEntries = 128
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
    fetchBufferSize = 32
    fetchToDecodeDelay = 3
    decodeWidth = 8
    decodeToRenameDelay = 2
    renameWidth = 8
    renameToIEWDelay = 1
    issueToExecuteDelay = 1
    dispatchWidth = 12
    issueWidth = 12
    wbWidth = 12
    fuPool = O3_X86_alderlake_FUP()
    iewToCommitDelay = 1
    renameToROBDelay = 1
    commitWidth = 12
    squashWidth = 12
    trapLatency = 13
    backComSize = 12
    forwardComSize = 12
    numPhysIntRegs = 988
    numPhysFloatRegs = 488
    numPhysVecRegs = 488
    numIQEntries = 224
    numROBEntries = 512

    switched_out = False
    branchPred = O3_X86_alderlake_BP()

# Instruction Cache
class O3_X86_alderlake_ICache(Cache):
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
class O3_X86_alderlake_DCache(Cache):
    tag_latency = 1
    data_latency = 1
    response_latency = 1
    mshrs = 6
    tgts_per_mshr = 8
    size = '48kB'
    assoc = 12
    write_buffers = 16
    writeback_clean = True

# TLB Cache
# Use a cache as a L2 TLB
class O3_X86_alderlakeWalkCache(Cache):
    data_latency = 2
    tag_latency = 2
    response_latency = 1
    mshrs = 6
    tgts_per_mshr = 8
    size = '2kB'
    assoc = 4
    write_buffers = 16
    is_read_only = True
    # Writeback clean lines as well
    writeback_clean = True


# L2 Cache
class O3_X86_alderlakeL2(Cache):
    tag_latency = 8
    data_latency = 8
    response_latency = 8
    mshrs = 16
    tgts_per_mshr = 8
    size = '1280kB'
    assoc = 20
    write_buffers = 16
    prefetch_on_access = True
    clusivity = 'mostly_excl'
    # Simple stride prefetcher
    prefetcher = StridePrefetcher(degree=16, latency = 1)
    tags = BaseSetAssoc()
    repl_policy = RandomRP()

    # L3 Cache
class O3_X86_alderlakeL3(Cache):
    tag_latency = 29
    data_latency = 29
    response_latency = 29
    mshrs = 512
    tgts_per_mshr = 20
    size = '3MB'
    assoc = 12
    write_buffers = 256
    prefetch_on_access = True
    clusivity = 'mostly_excl'
    # Simple stride prefetcher
    #prefetcher = StridePrefetcher(degree=8, latency = 1)
    tags = BaseSetAssoc()
    repl_policy = RandomRP()