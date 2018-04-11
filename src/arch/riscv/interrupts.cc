/*
 * Copyright (c) 2011 Google
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
 * Authors: Gabe Black
 *          Robert Scheffel
 */

#include "arch/riscv/interrupts.hh"

#include "cpu/thread_context.hh"
#include "debug/Interrupt.hh"

namespace RiscvISA {

Interrupts::Interrupts(Params * p)
    : SimObject(p), cpu(nullptr),
      ip(0x0), update(false)
{
    clearAll();
}

void
Interrupts::post(int int_num, int index)
{
    DPRINTF(Interrupt, "Interrupt %d posted\n", int_num);
    if (int_num < 0 || int_num >= NumInterruptTypes)
        panic("int_num out of bounds\n");
    if (index < 0 || index >= (int)sizeof(uint64_t) * 8)
        panic("int_num out of bounds\n");

    interrupts[int_num] |= 1 << index;
    intstatus |= (ULL(1) << int_num);
}

void
Interrupts::clear(int int_num, int index)
{
    DPRINTF(Interrupt, "Interrupt %d cleared\n", int_num, index);

    if (int_num < 0 || int_num >= NumInterruptTypes)
        panic("int_num out of bounds\n");

    if (index < 0 || index >= (int)sizeof(uint64_t) * 8)
        panic("int_num out of bounds\n");

    interrupts[int_num] &= ~(1 << index);
    if (interrupts[int_num] == 0)
        intstatus &= ~(ULL(1) << int_num);
}

void
Interrupts::clearAll()
{
    DPRINTF(Interrupt, "Interrupts all cleared\n");
    intstatus = 0;
    memset(interrupts, 0, sizeof(interrupts));
}

bool
Interrupts::checkInterrupts(ThreadContext *tc) const
{
    // get the CSR mstatus to check the interrupt enable bits
    MSTATUS status = tc->readMiscRegNoEffect(MISCREG_MSTATUS);

    // assume we are in machine mode at this point of time
    if (!(intstatus && status.mie)) {
        return false;
    }

    MIE mie = tc->readMiscRegNoEffect(MISCREG_MIE);

    // TODO: consider delegation register

    return ((interrupts[INT_MSI] && mie.msie) ||
            (interrupts[INT_MTI] && mie.mtie) ||
            (interrupts[INT_MEI] && mie.meie)
           );
}

Fault
Interrupts::getInterrupt(ThreadContext *tc)
{
    assert(checkInterrupts(tc));

    MSTATUS status = tc->readMiscRegNoEffect(MISCREG_MSTATUS);
    assert(status.mie);

    uint64_t ints;        // all pending interrupts, that are enabled
    ExceptionCode intr;   // stores the interrupt, that will be taken
    InterruptCode icode;  // find out if it is a timer or sw interrupt
    uint64_t mie;         // interrupt enable bits
    ints = 0;
    intr = UNSPECIFIED;
    mie = tc->readMiscRegNoEffect(MISCREG_MIE);

    // and the interrupt status with the interrupt pending bits
    // now we know which interrupt we can take and which not
    ints = mie & intstatus;

    // per default, interrupts trap to machine mode
    // machine level interrupts
    for (int i = 3; i < NumInterruptTypes; i += 4) {
        if (interrupts[i])
            intr = static_cast<ExceptionCode>(i);
    }

    DPRINTF(Interrupt, "Interrupt!  %#lx \n", intr);

    if (intr & 0x8)
        icode = EXTERNAL;
    else if (intr & 0x4)
        icode = TIMER;
    else
        icode = SOFTWARE;
    ip = ints;
    update = true;

    assert(intr != UNSPECIFIED);

    return std::make_shared<InterruptFault>(intr, icode);
}

void
Interrupts::updateIntrInfo(ThreadContext *tc)
{
    assert(update);
    uint64_t mip = tc->readMiscRegNoEffect(MISCREG_MIP);
    mip |= ip;
    DPRINTF(Interrupt, "MIP register: %#lx \n", mip);
    tc->setMiscRegNoEffect(MISCREG_MIP, mip);
    update = 0;
}

} // RiscvISA

RiscvISA::Interrupts *
RiscvInterruptsParams::create()
{
    return new RiscvISA::Interrupts(this);
}
