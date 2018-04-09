#include "arch/riscv/utility.hh"

#include "arch/riscv/faults.hh"

namespace RiscvISA
{

void initCPU(ThreadContext *tc, int cpuId)
{
    static Fault reset = std::make_shared<Reset>();
    reset->invoke(tc);
}

}