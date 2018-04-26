/*
 * Copyright (c) 2016 The University of Virginia
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
 * Authors: Alec Roelke
 *          Robert Scheffel
 */

#pragma once

#include <cstdint>
#include <iostream>

#include "insttest.h"

namespace I
{

inline uint32_t
lui(const uint32_t imm)
{
    int32_t rd = -1;
    asm volatile("lui %0,%1" : "=r" (rd) : "i" (imm));
    return rd;
}

inline bool
auipc(const uint32_t imm)
{
    int32_t rd = -1;
    asm volatile("auipc %0,%1" : "=r" (rd) : "i" (imm));
    std::cout << "auipc: 0x" << std::hex << std::uppercase << rd <<
        std::nouppercase << std::dec << std::endl;
    return rd >= imm;
}

inline bool
jal()
{
    asm volatile goto("jal zero,%l[jallabel]" : : : : jallabel);
    return false;
  jallabel:
    return true;
}

inline bool
jalr()
{
    int a = 0;
    asm volatile("auipc %0,0;"
                 "jalr t0,%0,12;"
                 "addi %0,zero,0;"
                 "sub %0,t0,%0;"
                 : "+r" (a)
                 :
                 : "t0");
    return a == 8;
}

inline bool
beq(int32_t a, int32_t b)
{
    asm volatile goto("beq %0,%1,%l[beqlabel]"
            :
            : "r" (a), "r" (b)
            :
            : beqlabel);
    return false;
  beqlabel:
    return true;
}

inline bool
bne(int32_t a, int32_t b)
{
    asm volatile goto("bne %0,%1,%l[bnelabel]"
            :
            : "r" (a), "r" (b)
            :
            : bnelabel);
    return false;
  bnelabel:
    return true;
}

inline bool
blt(int32_t a, int32_t b)
{
    asm volatile goto("blt %0,%1,%l[bltlabel]"
            :
            : "r" (a), "r" (b)
            :
            : bltlabel);
    return false;
  bltlabel:
    return true;
}

inline bool
bge(int32_t a, int32_t b)
{
    asm volatile goto("bge %0,%1,%l[bgelabel]"
            :
            : "r" (a), "r" (b)
            :
            : bgelabel);
    return false;
  bgelabel:
    return true;
}

inline bool
bltu(uint32_t a, uint32_t b)
{
    asm volatile goto("bltu %0,%1,%l[bltulabel]"
            :
            : "r" (a), "r" (b)
            :
            : bltulabel);
    return false;
  bltulabel:
    return true;
}

inline bool
bgeu(uint32_t a, uint32_t b)
{
    asm volatile goto("bgeu %0,%1,%l[bgeulabel]"
            :
            : "r" (a), "r" (b)
            :
            : bgeulabel);
    return false;
  bgeulabel:
    return true;
}

template<typename M, typename R> inline R
load(const M& b)
{
    R a = 0;
    switch(sizeof(M))
    {
      case 1:
        if (std::is_signed<M>::value) {
            asm volatile("lb %0,%1" : "=r" (a) : "m" (b));
        } else {
            asm volatile("lbu %0,%1" : "=r" (a) : "m" (b));
        }
        break;
      case 2:
        if (std::is_signed<M>::value) {
            asm volatile("lh %0,%1" : "=r" (a) : "m" (b));
        } else {
            asm volatile("lhu %0,%1" : "=r" (a) : "m" (b));
        }
        break;
      case 4:
        asm volatile("lw %0,%1" : "=r" (a) : "m" (b));
        break;
    }
    return a;
}

template<typename M> inline M
store(const M& rs2)
{
    M mem = 0;
    switch (sizeof(M))
    {
      case 1:
        asm volatile("sb %1,%0" : "=m" (mem) : "r" (rs2));
        break;
      case 2:
        asm volatile("sh %1,%0" : "=m" (mem) : "r" (rs2));
        break;
      case 4:
        asm volatile("sw %1,%0" : "=m" (mem) : "r" (rs2));
        break;
    }
    return mem;
}

inline int32_t
addi(int32_t rs1, const int16_t imm)
{
    int32_t rd = 0;
    IOP("addi", rd, rs1, imm);
    return rd;
}

inline bool
slti(int32_t rs1, const int16_t imm)
{
    bool rd = false;
    IOP("slti", rd, rs1, imm);
    return rd;
}

inline bool
sltiu(uint32_t rs1, const uint16_t imm)
{
    bool rd = false;
    IOP("sltiu", rd, rs1, imm);
    return rd;
}

inline uint32_t
xori(uint32_t rs1, const uint16_t imm)
{
    uint32_t rd = 0;
    IOP("xori", rd, rs1, imm);
    return rd;
}

inline uint32_t
ori(uint32_t rs1, const uint16_t imm)
{
    uint32_t rd = 0;
    IOP("ori", rd, rs1, imm);
    return rd;
}

inline uint32_t
andi(uint32_t rs1, const uint16_t imm)
{
    uint32_t rd = 0;
    IOP("andi", rd, rs1, imm);
    return rd;
}

inline int32_t
slli(int32_t rs1, const uint16_t imm)
{
    int32_t rd = 0;
    IOP("slli", rd, rs1, imm);
    return rd;
}

inline uint32_t
srli(uint32_t rs1, const uint16_t imm)
{
    uint32_t rd = 0;
    IOP("srli", rd, rs1, imm);
    return rd;
}

inline int32_t
srai(int32_t rs1, const uint16_t imm)
{
    int32_t rd = 0;
    IOP("srai", rd, rs1, imm);
    return rd;
}

inline int32_t
add(int32_t rs1, int32_t rs2)
{
    int32_t rd = 0;
    ROP("add", rd, rs1, rs2);
    return rd;
}

inline int32_t
sub(int32_t rs1, int32_t rs2)
{
    int32_t rd = 0;
    ROP("sub", rd, rs1, rs2);
    return rd;
}

inline int32_t
sll(int32_t rs1, int32_t rs2)
{
    int32_t rd = 0;
    ROP("sll", rd, rs1, rs2);
    return rd;
}

inline bool
slt(int32_t rs1, int32_t rs2)
{
    bool rd = false;
    ROP("slt", rd, rs1, rs2);
    return rd;
}

inline bool
sltu(uint32_t rs1, uint32_t rs2)
{
    bool rd = false;
    ROP("sltu", rd, rs1, rs2);
    return rd;
}

inline uint32_t
xor_inst(uint32_t rs1, uint32_t rs2)
{
    uint32_t rd = 0;
    ROP("xor", rd, rs1, rs2);
    return rd;
}

inline uint32_t
srl(uint32_t rs1, uint32_t rs2)
{
    uint32_t rd = 0;
    ROP("srl", rd, rs1, rs2);
    return rd;
}

inline int32_t
sra(int32_t rs1, int32_t rs2)
{
    int32_t rd = 0;
    ROP("sra", rd, rs1, rs2);
    return rd;
}

inline uint32_t
or_inst(uint32_t rs1, uint32_t rs2)
{
    uint32_t rd = 0;
    ROP("or", rd, rs1, rs2);
    return rd;
}

inline uint32_t
and_inst(uint32_t rs1, uint32_t rs2)
{
    uint32_t rd = 0;
    ROP("and", rd, rs1, rs2);
    return rd;
}

}
