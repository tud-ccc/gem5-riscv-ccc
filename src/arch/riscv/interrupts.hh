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
 */

#ifndef __ARCH_RISCV_INTERRUPT_HH__
#define __ARCH_RISCV_INTERRUPT_HH__

#include "arch/riscv/faults.hh"
#include "arch/riscv/isa_traits.hh"
#include "arch/riscv/registers.hh"
#include "base/logging.hh"
#include "params/RiscvInterrupts.hh"
#include "sim/sim_object.hh"

class BaseCPU;
class ThreadContext;

namespace RiscvISA {

class Interrupts : public SimObject
{
  private:
    BaseCPU * cpu;

    bool interrupts[NumInterruptTypes];
    uint64_t intstatus;

    ExceptionCode recentInt;  // stores the most recent interrupt
    uint64_t ip;              // summary of pending interrupts
    bool update;              // indicates a change in pending interrupts

  public:
    typedef RiscvInterruptsParams Params;

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    Interrupts(Params * p);

    void setCPU(BaseCPU * _cpu) { cpu = _cpu; }

    /**
     * post(int int_num, int index) is responsible for
     * posting an interrupt.
     */
    void post(int int_num, int index);
    // void post(int int_num, ThreadContext *tc);

    /**
     * clear(int int_num, int index) is responsible
     * for clearing an interrupt.
     */
    void clear(int int_num, int index);

    /**
     * clearAll() is responsible for clearing all interrupts.
     * It sets intstatus to 0 and clears every entry in
     * the interrupts array.
     */
    void clearAll();
    // void clearAll(ThreadContext *tc);

    /**
     * getInterrupt(ThreadContext * tc) checks if an interrupt
     * should be returned. It ands the interrupt mask and
     * and interrupt pending bits to see if one exists. It
     * also makes sure interrupts are enabled globally (mstatus.xie)
     * and locally (mie)
     */
    Fault getInterrupt(ThreadContext *tc);

    bool checkInterrupts(ThreadContext *tc) const;

    void updateIntrInfo(ThreadContext *tc);
};

} // namespace RiscvISA

#endif // __ARCH_RISCV_INTERRUPT_HH__

