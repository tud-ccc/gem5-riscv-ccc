# Copyright (c) 2018 TU Dresden
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
# Authors: Robert Scheffel

'''
System definition.
'''

import m5
from m5.objects import *
m5.util.addToPath('../../')
m5.util.addToPath('../../../ext/riscv-custom-extension/build/python')
from common.Caches import *

try:
    from minor_custom_timings import custom_timings
    timings = custom_timings
except ImportError:
    timings = []
    print('module not found')
    pass


class HiFive1IntFU(MinorFU):
    opClasses = minorMakeOpClassSet(['IntAlu'])
    timings = [MinorFUTiming(description="Int",
                             srcRegsRelativeLats=[2])]
    opLat = 3


class HiFive1IntMulFU(MinorFU):
    opClasses = minorMakeOpClassSet(['IntMult'])
    timings = [MinorFUTiming(description='Mul',
                             srcRegsRelativeLats=[0])]
    opLat = 7
    issueLat = 7


class HiFive1IntDivFU(MinorFU):
    opClasses = minorMakeOpClassSet(['IntDiv'])
    issueLat = 9
    opLat = 9


class HiFive1MemFU(MinorFU):
    opClasses = minorMakeOpClassSet(['MemRead', 'MemWrite', 'FloatMemRead',
                                     'FloatMemWrite'])
    timings = [MinorFUTiming(description='Mem',
                             srcRegsRelativeLats=[1], extraAssumedLat=1)]
    opLat = 1


class MinorCustomIntFU(MinorFU):
    opClasses = minorMakeOpClassSet(['IntCustom'])

    timings = timings
    opLat = 1


class HiFive1FUPool(MinorFUPool):
    funcUnits = [HiFive1IntFU(),
                 HiFive1IntMulFU(),
                 HiFive1IntDivFU(),
                 MinorCustomIntFU(),
                 MinorCustomIntFU(),
                 MinorDefaultFloatSimdFU(),
                 HiFive1MemFU(),
                 HiFive1MemFU(),
                 MinorDefaultMiscFU()]


class MemBus(SystemXBar):
    badaddr_responder = BadAddr(warn_access="warn", pio_latency='1ns')
    default = Self.badaddr_responder.pio

    frontend_latency = 0
    forward_latency = 0
    response_latency = 0
    snoop_response_latency = 0
    snoop_filter = SnoopFilter(lookup_latency=1)


class L1I(Cache):
    is_read_only = True
    tag_latency = 1
    data_latency = 1
    response_latency = 1
    size = '16kB'
    assoc = 2

    mshrs = 12
    tgts_per_mshr = 8


class HiFive1(BareMetalRiscvSystem):
    """
    Simple system containing just CPU, bus and memory
    """

    def __init__(self, cpu_class, wfgdb, **kwargs):
        super(HiFive1, self).__init__(**kwargs)

        # create clock and voltage domain
        # set to 100MHz clock frequency (like real hw board)
        self.clk_domain = SrcClockDomain(clock='256MHz')
        self.clk_domain.voltage_domain = VoltageDomain()

        # set up our board
        self.board = SimpleBoard()
        # appearently we need a dedicated interrupt controller
        self.intrctrl = IntrControl()

        # specify functional unit
        self.execFU = HiFive1FUPool()

        # create cpu and thread
        # the interruptcontroller needs to be created as well
        # tell the cpu if it shall wait until a gdb gets
        # connected remotely or not
        self.cpu = cpu_class()
        self.cpu.createThreads()
        self.cpu.createInterruptController()
        self.cpu.wait_for_remote_gdb = wfgdb

        # if cpu_class is MinorCPU:
        self.cpu.fetch1LineSnapWidth = 4
        self.cpu.fetch1LineWidth = 4
        self.cpu.fetch1ToFetch2BackwardDelay = 0

        # branch predictor
        branchpred = LocalBP()
        branchpred.BTBEntries = 64
        branchpred.RASSize = 2
        branchpred.localPredictorSize = 128
        self.cpu.branchPred = branchpred

        self.cache_line_size = 32

        # add caches
        # self.cpu.addPrivateSplitL1Caches(L1I(), L1D())
        l1i = L1I()
        self.cpu.icache = l1i
        self.cpu._cached_ports = ['icache.mem_side']

        # system memory bus
        self.membus = MemBus()

        # create mem_range
        # for now start at 0x0
        # take 3GB as size as it is a bit lower than 0xffffffff bytes
        iflash_start = Addr(0x20400000)
        iflash_size = '512MB'
        dram_start = Addr(0x80000000)
        dram_size = '32kB'
        self.mem_ranges = [AddrRange(start=iflash_start, size=iflash_size),
                           AddrRange(start=dram_start, size=dram_size)]

        self.iflash = SimpleMemory(latency='18us')
        self.iflash.range = self.mem_ranges[0]

        self.dmem = SimpleMemory(latency='1ns')
        self.dmem.range = self.mem_ranges[1]

        # define bit mode
        # self.rv32 = True

    def connect(self):

        self.cpu.dcache_port = self.membus.slave
        self.cpu.icache_port = self.cpu.icache.cpu_side
        self.cpu.icache.mem_side = self.membus.slave

        self.system_port = self.membus.slave

        self.dmem.port = self.membus.master
        self.iflash.port = self.membus.master

        self.board.attachIO(self.membus)
