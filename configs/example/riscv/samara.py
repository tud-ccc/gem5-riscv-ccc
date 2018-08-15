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

from m5.objects import *
from minor_custom_fu import MinorCustomFUPool


class SamaraIntFU(MinorFU):
    opClasses = minorMakeOpClassSet(['IntAlu'])
    timings = [MinorFUTiming(description="Int",
                             srcRegsRelativeLats=[2])]
    opLat = 5
    issueLat = 5


class SamaraIntMulFU(MinorFU):
    opClasses = minorMakeOpClassSet(['IntMult'])
    timings = [MinorFUTiming(description='Mul',
                             srcRegsRelativeLats=[0])]
    opLat = 6
    issueLat = 6


class SamaratIntDivFU(MinorFU):
    opClasses = minorMakeOpClassSet(['IntDiv'])
    issueLat = 40
    opLat = 40


class SamaraMemFU(MinorFU):
    opClasses = minorMakeOpClassSet(['MemRead', 'MemWrite', 'FloatMemRead',
                                     'FloatMemWrite'])
    # timings = [MinorFUTiming(description='Mem',
    #    srcRegsRelativeLats=[1], extraAssumedLat=2)]
    opLat = 8
    issueLat = 8


class SamaraFUPool(MinorFUPool):
    funcUnits = [SamaraIntFU(),
                 SamaraIntMulFU(),
                 SamaratIntDivFU(),
                 MinorDefaultFloatSimdFU(),
                 SamaraMemFU(),
                 MinorDefaultMiscFU()]


class MemBus(SystemXBar):
    badaddr_responder = BadAddr(warn_access="warn")
    default = Self.badaddr_responder.pio

    frontend_latency = 0
    forward_latency = 0
    response_latency = 0
    snoop_response_latency = 0
    snoop_filter = SnoopFilter(lookup_latency=1)


class TimingMemBus(SystemXBar):
    badaddr_responder = BadAddr(warn_access="warn")
    default = Self.badaddr_responder.pio

    frontend_latency = 0
    forward_latency = 0
    response_latency = 0
    snoop_response_latency = 0
    snoop_filter = SnoopFilter(lookup_latency=1)


class Samara(BareMetalRiscvSystem):
    """
    Simple system containing just CPU, bus and memory
    """

    def __init__(self, cpu_class, wfgdb, **kwargs):
        super(Samara, self).__init__(**kwargs)

        # create clock and voltage domain
        # set to 100MHz clock frequency (like real hw board)
        self.clk_domain = SrcClockDomain(clock='64MHz')
        self.clk_domain.voltage_domain = VoltageDomain()

        # set up our board
        self.board = SimpleBoard()
        # appearently we need a dedicated interrupt controller
        self.intrctrl = IntrControl()

        # specify functional unit
        self.execFU = SamaraFUPool()

        # create cpu and thread
        # the interruptcontroller needs to be created as well
        # tell the cpu if it shall wait until a gdb gets
        # connected remotely or not
        self.cpu = cpu_class()
        self.cpu.createThreads()
        self.cpu.createInterruptController()
        self.cpu.wait_for_remote_gdb = wfgdb

        # configure cpu parameter
        # self.cpu.fetch1FetchLimit = 2
        self.cpu.fetch1LineSnapWidth = 4
        self.cpu.fetch1LineWidth = 4
        # self.cpu.executeAllowEarlyMemoryIssue = False
        # self.cpu.executeMemoryWidth = 4

        # system memory bus
        self.membus = MemBus()

        # create mem_range
        # for now start at 0x0
        # take 3GB as size as it is a bit lower than 0xffffffff bytes
        mem_start = Addr(0x00000000)
        mem_size = '2GB'
        self.mem_ranges = [AddrRange(start=mem_start, size=mem_size)]

        self.ram = SimpleMemory(latency='62ns')
        self.ram.range = self.mem_ranges[0]

        # define bit mode
        # self.rv32 = True

    def connect(self):
        # connect cache ports of cpu to membus
        # no caches -> connect directly to mem bus
        self.cpu.connectAllPorts(self.membus)
        self.system_port = self.membus.slave

        self.ram.port = self.membus.master

        self.board.attachIO(self.membus)
