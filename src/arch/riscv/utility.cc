#include "arch/riscv/utility.hh"

#include "arch/riscv/faults.hh"
#include "arch/riscv/registers.hh"

namespace RiscvISA
{

void
initCPU(ThreadContext *tc, int cpuId)
{
    static Fault reset = std::make_shared<Reset>();
    reset->invoke(tc);
}

bool
isRv32(ThreadContext *tc)
{
    MISA misa = tc->readMiscRegNoEffect(MISCREG_MISA);
    if (misa.mxl32)
        return true;
    return false;
}

}