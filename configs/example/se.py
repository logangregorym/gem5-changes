# Copyright (c) 2012-2013 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) 2006-2008 The Regents of The University of Michigan
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
# Authors: Steve Reinhardt

# Simple test script
#
# "m5 test.py"

from __future__ import print_function

import optparse
import sys
import os

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath, fatal, warn

addToPath('../')

from ruby import Ruby

from common import Options
from common import Simulation
from common import CacheConfig
from common import CpuConfig
from common import MemConfig
from common.Caches import *
from common.cpu2000 import *
from common.cores.x86 import O3_X86_skylake
from common.cores.x86 import O3_X86_icelake
from common.cores.x86 import O3_X86_alderlake

def get_processes(options):
    """Interprets provided options and returns a list of processes"""

    multiprocesses = []
    inputs = []
    outputs = []
    errouts = []
    pargs = []

    workloads = options.cmd.split(';')
    if options.input != "":
        inputs = options.input.split(';')
    if options.output != "":
        outputs = options.output.split(';')
    if options.errout != "":
        errouts = options.errout.split(';')
    if options.options != "":
        pargs = options.options.split(';')

    idx = 0
    for wrkld in workloads:
        process = Process(pid = 100 + idx)
        process.executable = wrkld
        process.cwd = os.getcwd()

        if options.env:
            with open(options.env, 'r') as f:
                process.env = [line.rstrip() for line in f]

        if len(pargs) > idx:
            process.cmd = [wrkld] + pargs[idx].split()
        else:
            process.cmd = [wrkld]

        if len(inputs) > idx:
            process.input = inputs[idx]
        if len(outputs) > idx:
            process.output = outputs[idx]
        if len(errouts) > idx:
            process.errout = errouts[idx]

        multiprocesses.append(process)
        idx += 1

    if options.smt:
        assert(options.cpu_type == "DerivO3CPU" or options.cpu_type == "O3_X86_sb_1" or options.cpu_type == "O3_X86_skylake_1" or options.cpu_type == "O3_X86_icelake_1" or options.cpu_type == "O3_X86_alderlake_1")
        return multiprocesses, idx
    else:
        return multiprocesses, 1


parser = optparse.OptionParser()
Options.addCommonOptions(parser)
Options.addSEOptions(parser)

parser.add_option("--enable-microop-cache", action="store_true", help="""Enable micro-op cache.""")
parser.add_option("--enable-micro-fusion", action="store_true", help="""Enable micro-fusion.""")
parser.add_option("--enable-superoptimization", action="store_true", help="""Enable speculative superoptimization.""")
parser.add_option("--superoptimization-warmup-cycles", default=0, type="int", help="""Warmup cycles for speculative superoptimization.""")
parser.add_option("--constantBufferSize", default=256, type="int", action="store", help="Size of the buffer storing constant load addresses.")
parser.add_option("--dumpFrequency", default=10000, type="int", action="store", help="Number of cycles between dumps.")
parser.add_option("--lvpredType", default="basic", help="Load Value Predictor Type: basic/strideHist/3period/hybrid/eves.");
parser.add_option("--tableEntries", default=4096, type="int", action="store", help="Number of entries in the lookup table.")
parser.add_option("--constantThreshold", default=10, type="int", action="store", help="First usable prediction.")
parser.add_option("--dynamicThreshold", default=1, type="int", action="store", help="Learn the constant threshold dynamically?")
parser.add_option("--satCounterBits", default=5, type="int", action="store", help="Bits in the saturating counters.")
parser.add_option("--initialPredictionQuality", default=0, type="int", action="store", help="Default value of saturating counters.")
parser.add_option("--resetDelay", default=128, type="int", action="store", help="Cycles to remain inactive after a misprediction.")
parser.add_option("--historyLength", default=4, type="int", action="store", help="Number of past values to track.")
parser.add_option("--historyEntryBits", default=4, type="int", action="store", help="Bits per value in the history.")
parser.add_option("--decrementBy", default=10, type="int", action="store", help="Amount to decrement confidence in response to a misprediction.")
parser.add_option("--resetTo", default=0, type="int", action="store", help="Value to reset confidence to in response to a misprediction.")
parser.add_option("--missThreshold", default=150, type="int", action="store", help="Misses allowed before confidence threshold increments.")
parser.add_option("--hitThreshold", default=300, type="int", action="store", help="Unused hits allowed before confidence threshold decrements.")
parser.add_option("--predictingArithmetic", default=0, type="int", action="store", help="Whether to predict arithmetic insts as well as loads.");
parser.add_option("--predictStage", default=3, type="int", action="store", help="Prediction Stage: fetch/iew/both.");
parser.add_option("--maxDependencyRecursion", default=15, type="int", action="store", help="How deep to recurse when counting dependencies.");
parser.add_option("--usingControlTracking", default=0, type="int", action="store", help="Track control dependencies to optimize across?");
parser.add_option("--usingCCTracking", default=0, type="int", action="store", help="Track condition codes?");
parser.add_option("--predictionConfidenceThreshold", default=5, type="int", action="store", help="Confidence threshold to make a prediction during superoptmization");
parser.add_option("--debugTraceGen", default=0, type="int", action="store", help="Trace id which we want to generate logs for");

parser.add_option("--specCacheNumWays", default=8, type="int", action="store", help="Number of ways for speculative cache");
parser.add_option("--specCacheNumSets", default=32, type="int", action="store", help="Number of sets for speculative cache");
parser.add_option("--specCacheNumUops", default=6, type="int", action="store", help="Number of uops for speculative cache");
parser.add_option("--uopCacheNumWays", default=8, type="int", action="store", help="Number of ways for micro-op cache");
parser.add_option("--uopCacheNumSets", default=32, type="int", action="store", help="Number of sets for micro-op cache");
parser.add_option("--uopCacheNumUops", default=6, type="int", action="store", help="Number of uops for micro-op cache");
parser.add_option("--numOfTracePredictionSources", default=4, type="int", action="store", help="Number of prediction sources in a super optimized trace");

parser.add_option("--maxRecursiveDepth", default=8, type="int", action="store", help="Maximum depth to recurse to when measuring dependency chains")
parser.add_option("--usingTrace", default=0, type="int", action="store", help="Whether to stream the optimized trace")
parser.add_option("--connectionCount", default=4096, type="int", action="store", help="Number of connections to track at any given time");
parser.add_option("--branchConfidenceCounterSize", default=2, type="int", action="store", help="Size of the branch confidence counters in bits.")
parser.add_option("--branchConfidenceThreshold", default=3, type="int", action="store", help="Minimum confidence needed to do LVP across a branch.")
parser.add_option("--doStoragelessBranchConf", action="store_true", help="Whether to use storageless TAGE confidence (Seznec 2010).")
parser.add_option("--checkpoint_at_instr", default=0, type="int", action="store", help="");
parser.add_option("--after_exec_cnt", default=0, type="int", action="store", help="");
parser.add_option("--lvpLookupAtFetch", action="store_true", help="Enables an LVP lookup at every fetch cycle to detect stale traces, enabling will incur a 1 cycle penalty");
parser.add_option("--enableValuePredForwarding", action="store_true", help="Enables Load Value Prediction for Raw execution");
parser.add_option("--enableDynamicThreshold", action="store_true", help="Enables the use of a dynamic threshold for LVP");

if '--ruby' in sys.argv:
    Ruby.define_options(parser)

(options, args) = parser.parse_args()

if args:
    print("Error: script doesn't take any positional arguments")
    sys.exit(1)

multiprocesses = []
numThreads = 1

if options.bench:
    apps = options.bench.split("-")
    if len(apps) != options.num_cpus:
        print("number of benchmarks not equal to set num_cpus!")
        sys.exit(1)

    for app in apps:
        try:
            if buildEnv['TARGET_ISA'] == 'alpha':
                exec("workload = %s('alpha', 'tru64', '%s')" % (
                        app, options.spec_input))
            elif buildEnv['TARGET_ISA'] == 'arm':
                exec("workload = %s('arm_%s', 'linux', '%s')" % (
                        app, options.arm_iset, options.spec_input))
            else:
                exec("workload = %s(buildEnv['TARGET_ISA', 'linux', '%s')" % (
                        app, options.spec_input))
            multiprocesses.append(workload.makeProcess())
        except:
            print("Unable to find workload for %s: %s" %
                  (buildEnv['TARGET_ISA'], app),
                  file=sys.stderr)
            sys.exit(1)
elif options.cmd:
    multiprocesses, numThreads = get_processes(options)
else:
    print("No workload specified. Exiting!\n", file=sys.stderr)
    sys.exit(1)


(CPUClass, test_mem_mode, FutureClass) = Simulation.setCPUClass(options)




if CPUClass.__name__ != "AtomicSimpleCPU":
    CPUClass.enable_microop_cache = options.enable_microop_cache
    CPUClass.enable_micro_fusion = options.enable_micro_fusion
    CPUClass.enable_superoptimization = options.enable_superoptimization
    CPUClass.superoptimization_warmup_cycles = options.superoptimization_warmup_cycles
    CPUClass.loadPred.lvpredType = options.lvpredType
    CPUClass.loadPred.tableEntries = options.tableEntries
    CPUClass.loadPred.constantThreshold = options.constantThreshold
    CPUClass.loadPred.dynamicThreshold = options.dynamicThreshold
    CPUClass.loadPred.satCounterBits = options.satCounterBits
    CPUClass.loadPred.initialPredictionQuality = options.initialPredictionQuality
    CPUClass.loadPred.resetDelay = options.resetDelay
    CPUClass.loadPred.historyLength = options.historyLength
    CPUClass.loadPred.historyEntryBits = options.historyEntryBits
    CPUClass.loadPred.decrementBy = options.decrementBy
    CPUClass.loadPred.resetTo = options.resetTo
    CPUClass.loadPred.missThreshold = options.missThreshold
    CPUClass.loadPred.hitThreshold = options.hitThreshold
    CPUClass.loadPred.predictingArithmetic = options.predictingArithmetic
    CPUClass.loadPred.predictStage = options.predictStage
    CPUClass.maxDependencyRecursion = options.maxDependencyRecursion
    CPUClass.usingTrace = options.usingTrace
    CPUClass.traceConstructor.usingControlTracking = options.usingControlTracking
    CPUClass.traceConstructor.usingCCTracking = options.usingCCTracking
    CPUClass.traceConstructor.predictionConfidenceThreshold = options.predictionConfidenceThreshold
    CPUClass.traceConstructor.specCacheNumWays = options.specCacheNumWays
    CPUClass.traceConstructor.specCacheNumSets = options.specCacheNumSets
    CPUClass.traceConstructor.specCacheNumUops = options.specCacheNumUops
    CPUClass.traceConstructor.numOfTracePredictionSources = options.numOfTracePredictionSources
    CPUClass.traceConstructor.debugTraceGen = options.debugTraceGen
    CPUClass.checkpoint_at_instr = options.checkpoint_at_instr
    CPUClass.after_exec_cnt = options.after_exec_cnt
    CPUClass.uopCacheNumWays = options.uopCacheNumWays
    CPUClass.uopCacheNumSets = options.uopCacheNumSets
    CPUClass.uopCacheNumUops = options.uopCacheNumUops
    CPUClass.lvpLookupAtFetch = options.lvpLookupAtFetch
    CPUClass.enableValuePredForwarding = options.enableValuePredForwarding
    CPUClass.enableDynamicThreshold = options.enableDynamicThreshold

CPUClass.numThreads = numThreads
CPUClass.branchPred.numThreads = numThreads
CPUClass.branchPred.branchConfidenceCounterSize = options.branchConfidenceCounterSize
CPUClass.branchPred.branchConfidenceThreshold = options.branchConfidenceThreshold
CPUClass.branchPred.doStoragelessBranchConf = options.doStoragelessBranchConf


if FutureClass and FutureClass.__name__ != "AtomicSimpleCPU":
    FutureClass.enable_microop_cache = options.enable_microop_cache
    FutureClass.enable_micro_fusion = options.enable_micro_fusion
    FutureClass.enable_superoptimization = options.enable_superoptimization
    FutureClass.superoptimization_warmup_cycles = options.superoptimization_warmup_cycles
    FutureClass.loadPred.lvpredType = options.lvpredType
    FutureClass.loadPred.tableEntries = options.tableEntries
    FutureClass.loadPred.constantThreshold = options.constantThreshold
    FutureClass.loadPred.dynamicThreshold = options.dynamicThreshold
    FutureClass.loadPred.satCounterBits = options.satCounterBits
    FutureClass.loadPred.initialPredictionQuality = options.initialPredictionQuality
    FutureClass.loadPred.resetDelay = options.resetDelay
    FutureClass.loadPred.historyLength = options.historyLength
    FutureClass.loadPred.historyEntryBits = options.historyEntryBits
    FutureClass.loadPred.decrementBy = options.decrementBy
    FutureClass.loadPred.resetTo = options.resetTo
    FutureClass.loadPred.missThreshold = options.missThreshold
    FutureClass.loadPred.hitThreshold = options.hitThreshold
    FutureClass.loadPred.predictingArithmetic = options.predictingArithmetic
    FutureClass.loadPred.predictStage = options.predictStage
    FutureClass.maxDependencyRecursion = options.maxDependencyRecursion
    FutureClass.usingTrace = options.usingTrace
    FutureClass.traceConstructor.usingControlTracking = options.usingControlTracking
    FutureClass.traceConstructor.usingCCTracking = options.usingCCTracking
    FutureClass.traceConstructor.predictionConfidenceThreshold = options.predictionConfidenceThreshold
    FutureClass.traceConstructor.specCacheNumWays = options.specCacheNumWays
    FutureClass.traceConstructor.specCacheNumSets = options.specCacheNumSets
    FutureClass.traceConstructor.specCacheNumUops = options.specCacheNumUops
    FutureClass.traceConstructor.numOfTracePredictionSources = options.numOfTracePredictionSources
    FutureClass.traceConstructor.debugTraceGen = options.debugTraceGen
    FutureClass.numThreads = numThreads
    FutureClass.branchPred.numThreads = numThreads
    FutureClass.branchPred.branchConfidenceCounterSize = options.branchConfidenceCounterSize
    FutureClass.branchPred.branchConfidenceThreshold = options.branchConfidenceThreshold
    FutureClass.branchPred.doStoragelessBranchConf = options.doStoragelessBranchConf
    FutureClass.after_exec_cnt = options.after_exec_cnt
    FutureClass.checkpoint_at_instr = options.checkpoint_at_instr
    FutureClass.uopCacheNumWays = options.uopCacheNumWays
    FutureClass.uopCacheNumSets = options.uopCacheNumSets
    FutureClass.uopCacheNumUops = options.uopCacheNumUops
    FutureClass.lvpLookupAtFetch = options.lvpLookupAtFetch
    FutureClass.enableValuePredForwarding = options.enableValuePredForwarding
    FutureClass.enableDynamicThreshold = options.enableDynamicThreshold

# Check -- do not allow SMT with multiple CPUs
if options.smt and options.num_cpus > 1:
    fatal("You cannot use SMT with multiple CPUs!")

np = options.num_cpus
system = System(cpu = [CPUClass(cpu_id=i) for i in xrange(np)],
                mem_mode = test_mem_mode,
                mem_ranges = [AddrRange(options.mem_size)],
                cache_line_size = options.cacheline_size)

if numThreads > 1:
    system.multi_thread = True
    CPUClass._uncached_slave_ports += ["interrupts[1].pio",
                              "interrupts[1].int_slave"]
    CPUClass._uncached_master_ports += ["interrupts[1].int_master"]

# Create a top-level voltage domain
system.voltage_domain = VoltageDomain(voltage = options.sys_voltage)

# Create a source clock for the system and set the clock period
system.clk_domain = SrcClockDomain(clock =  options.sys_clock,
                                   voltage_domain = system.voltage_domain)

# Create a CPU voltage domain
system.cpu_voltage_domain = VoltageDomain()

# Create a separate clock domain for the CPUs
system.cpu_clk_domain = SrcClockDomain(clock = options.cpu_clock,
                                       voltage_domain =
                                       system.cpu_voltage_domain)

# If elastic tracing is enabled, then configure the cpu and attach the elastic
# trace probe
if options.elastic_trace_en:
    CpuConfig.config_etrace(CPUClass, system.cpu, options)

# All cpus belong to a common cpu_clk_domain, therefore running at a common
# frequency.
for cpu in system.cpu:
    cpu.clk_domain = system.cpu_clk_domain

if CpuConfig.is_kvm_cpu(CPUClass) or CpuConfig.is_kvm_cpu(FutureClass):
    if buildEnv['TARGET_ISA'] == 'x86':
        system.kvm_vm = KvmVM()
        for process in multiprocesses:
            process.useArchPT = True
            process.kvmInSE = True
    else:
        fatal("KvmCPU can only be used in SE mode with x86")

# Enable 1 cycle penalty for LVP lookup
if options.lvpLookupAtFetch:
    if not options.enable_superoptimization:
        fatal("No LVP Lookup needed if SuperOptimization not enabled")
    found = False
    for cpu in system.cpu:
        if (cpu is O3_X86_skylake) or (cpu is O3_X86_icelake) or (cpu is O3_X86_alderlake):
            cpu.fetchToDecodeDelay = int(system.cpu[0].fetchToDecodeDelay) + 1
            found = True
elif options.enableValuePredForwarding:
    if not (options.lvpredType and options.predictionConfidenceThreshold):
        fatal("Must define lvpredType and predictionConfidenceThreshold to enable forwarding")
    if options.enable_superoptimization:
        fatal("Value forwarding not yet supported with superoptimization")
    found = False
    for cpu in system.cpu:
        if (cpu is O3_X86_skylake) or (cpu is O3_X86_icelake) or (cpu is O3_X86_alderlake):
            cpu.fetchToDecodeDelay = int(system.cpu[0].fetchToDecodeDelay) + 1
            found = True
#    if not found:
#        print(type(cpu))
#        fatal("Unable to add 1 cycle penalty for enableValuePredForwarding")


# Sanity check
if options.simpoint_profile:
    if not CpuConfig.is_atomic_cpu(TestCPUClass):
        fatal("SimPoint/BPProbe should be done with an atomic cpu")
    if np > 1:
        fatal("SimPoint generation not supported with more than one CPUs")

for i in xrange(np):
    if options.smt:
        system.cpu[i].workload = multiprocesses
    elif len(multiprocesses) == 1:
        system.cpu[i].workload = multiprocesses[0]
    else:
        system.cpu[i].workload = multiprocesses[i]

    if options.simpoint_profile:
        system.cpu[i].addSimPointProbe(options.simpoint_interval)

    if options.checker:
        system.cpu[i].addCheckerCpu()

    system.cpu[i].createThreads()

if options.ruby:
    Ruby.create_system(options, False, system)
    assert(options.num_cpus == len(system.ruby._cpu_ports))

    system.ruby.clk_domain = SrcClockDomain(clock = options.ruby_clock,
                                        voltage_domain = system.voltage_domain)
    for i in xrange(np):
        ruby_port = system.ruby._cpu_ports[i]

        # Create the interrupt controller and connect its ports to Ruby
        # Note that the interrupt controller is always present but only
        # in x86 does it have message ports that need to be connected
        system.cpu[i].createInterruptController()

        # Connect the cpu's cache ports to Ruby
        system.cpu[i].icache_port = ruby_port.slave
        system.cpu[i].dcache_port = ruby_port.slave
        if buildEnv['TARGET_ISA'] == 'x86':
            system.cpu[i].interrupts[0].pio = ruby_port.master
            system.cpu[i].interrupts[0].int_master = ruby_port.slave
            system.cpu[i].interrupts[0].int_slave = ruby_port.master
            system.cpu[i].itb.walker.port = ruby_port.slave
            system.cpu[i].dtb.walker.port = ruby_port.slave
else:
    MemClass = Simulation.setMemClass(options)
    system.membus = SystemXBar()
    system.system_port = system.membus.slave
    CacheConfig.config_cache(options, system)
    MemConfig.config_mem(options, system)

root = Root(full_system = False, system = system)
Simulation.run(options, root, system, FutureClass)
