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

from m5.params import *
from m5.proxy import *

from Device import BasicPioDevice
from Platform import Platform
from Terminal import Terminal
from Uart import Uart8250


class CustomRegs(BasicPioDevice):
    type = 'CustomRegs'
    cxx_header = 'dev/riscv/custom_regs.hh'

    regs = VectorParam.UInt32("Addresses of the custom registers.")


class TimerCpu(BasicPioDevice):
    type = 'TimerCpu'
    cxx_header = 'dev/riscv/timer_cpu.hh'

    cpu = Param.BaseCPU(Parent.any, "Cpu this device is part of.")


class SimpleBoard(Platform):
    type = 'SimpleBoard'
    cxx_header = 'dev/riscv/simpleboard.hh'
    system = Param.System(Parent.any, 'system')

    timer_cpu = TimerCpu(pio_addr=0x80000100)

    term = Terminal()
    uart = Uart8250(pio_addr=0x80000000)

    cust = CustomRegs()
    cust.regs = [0x0, 0x4, 0x8]

    # attach I/O devices to bus
    # call this method after bus is defined at system level
    def attachIO(self, bus):
        self.timer_cpu.pio = bus.master
        self.uart.device = self.term
        self.uart.pio = bus.master
