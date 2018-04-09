"""
System definition.
"""

import m5
from m5.objects import *


class MemBus(SystemXBar):
    badaddr_responder = BadAddr(warn_access="warn")
    default = Self.badaddr_responder.pio


class SimpleSystem(BareMetalRiscvSystem):
    """
    Simple system containing just CPU, bus and memory
    """

    def __init__(self, cpu_class, wfgdb, **kwargs):
        super(SimpleSystem, self).__init__(**kwargs)

        # create clock and voltage domain
        # set to 100MHz clock frequency (like real hw board)
        self.clk_domain = SrcClockDomain(clock='100MHz')
        self.clk_domain.voltage_domain = VoltageDomain()

        # set up our board
        self.board = SimpleBoard()
        # appearently we need a dedicated interrupt controller
        self.intrctrl = IntrControl()

        # create cpu and thread
        # the interruptcontroller needs to be created as well
        # tell the cpu if it shall wait until a gdb gets
        # connected remotely or not
        self.cpu = cpu_class()
        self.cpu.createThreads()
        self.cpu.createInterruptController()
        self.cpu.wait_for_remote_gdb = wfgdb

        # system memory bus
        self.membus = MemBus()

        # create mem_range
        # for now start at 0x0
        # take 3GB as size as it is a bit lower than 0xffffffff bytes
        mem_start = Addr(0x00000000)
        mem_size = '2GB'
        self.mem_ranges = [AddrRange(start=mem_start, size=mem_size)]

    def connect(self):
        # connect cache ports of cpu to membus
        # no caches -> connect directly to mem bus
        self.cpu.connectAllPorts(self.membus)
        self.system_port = self.membus.slave

        self.board.attachIO(self.membus)
