/*
 * Copyright (c) 2018 TU Dresden
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Robert Scheffel
 */

#include "dev/riscv/timer_cpu.hh"

#include "cpu/base.hh"
#include "debug/Timer.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

TimerCpu::TimerCpu(Params *p)
    : BasicPioDevice(p, 0x10),
      cpu(p->cpu),
      timecmp(0x0),
      timerAlarmEvent([this]{ timerAlarm(); }, name())
{}

Tick
TimerCpu::read(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    assert(pkt->getSize() == 8);

    /** which register do we want to read? */
    Addr addr = pkt->getAddr() - pioAddr;
    switch (addr) {
        case Time:
            /**
             * this register does not exist explicitly in code
             * instead calculate value
             */
            break;
        case TimeCmp:
            pkt->set<uint64_t>(timecmp);
            break;
        default:
            panic("Tried to read TimerCpu at offset %#x\n", addr);
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
TimerCpu::write(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
    assert(pkt->getSize() == 8);

    Addr addr = pkt->getAddr() - pioAddr;
    switch (addr) {
        case Time:
            /** TODO: should not be writable, right? */
            DPRINTF(Timer, "Ignore write on TimerCpu r-o time register.\n");
            break;
        case TimeCmp:
            timecmp = pkt->get<uint64_t>();
            // clear mtip bit
            cpu->clearInterrupt(0, 7, 0);
            startTimer(timecmp);
            break;
        default:
            panic("Tried to read TimerCpu at offset %#x\n", addr);
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}

void
TimerCpu::startTimer(uint64_t val)
{
    DPRINTF(Timer, "Start timer with value %#lx\n", val);

    Tick time = clockPeriod();
    time *= val;

    if (timerAlarmEvent.scheduled()) {
        DPRINTF(Timer, "-- AlarmEvent was already scheduled, de-scheduling\n");
        deschedule(timerAlarmEvent);
    }

    schedule(timerAlarmEvent, curTick() + time);
    DPRINTF(Timer, "-- Scheduling new event for: %d\n", curTick() + time);
}

void
TimerCpu::timerAlarm()
{
    DPRINTF(Timer, "Timer Alarm\n");
    cpu->postInterrupt(0, 7, 0);
}

TimerCpu *
TimerCpuParams::create()
{
    return new TimerCpu(this);
}
