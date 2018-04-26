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
 * Authors: Robert Scheffel
 */

#include "rv32i.h"

int main()
{
    // lui
    expect<int32_t>(4096, []{return I::lui(1);}, "lui");
    expect<int32_t>(numeric_limits<int32_t>::min(),
                    []{return I::lui(0x80000);}, "lui, -");

    // auipc
    expect<bool>(true, []{return I::auipc(3);}, "auipc");

    // jal, jalr
    expect<bool>(true, []{return I::jal();}, "jal");
    expect<bool>(true, []{return I::jalr();}, "jalr");

    // beq
    expect<bool>(true, []{return I::beq(5, 5);}, "beq, =");
    expect<bool>(false,[]{return I::beq(numeric_limits<int32_t>::max(),
        numeric_limits<int32_t>::min());},"beq, !=");

    // bne
    expect<bool>(false, []{return I::bne(5, 5);}, "bne, =");
    expect<bool>(true,[]{return I::bne(numeric_limits<int32_t>::max(),
        numeric_limits<int32_t>::min());},"bne, !=");

    // blt
    expect<bool>(true,[]{return I::blt(numeric_limits<int32_t>::min(),
        numeric_limits<int32_t>::max());},"blt, <");
    expect<bool>(false,[]{return I::blt(numeric_limits<int32_t>::max(),
        numeric_limits<int32_t>::max());},"blt, =");
    expect<bool>(false,[]{return I::blt(numeric_limits<int32_t>::max(),
        numeric_limits<int32_t>::min());},"blt, >");

    // bge
    expect<bool>(true,[]{return I::bge(numeric_limits<int32_t>::max(),
        numeric_limits<int32_t>::min());},"bge, >");
    expect<bool>(true,[]{return I::bge(numeric_limits<int32_t>::min(),
        numeric_limits<int32_t>::min());},"bge, =");
    expect<bool>(true,[]{return I::bge(numeric_limits<int32_t>::min(),
        numeric_limits<int32_t>::max());},"bge, <");

    // bltu
    expect<bool>(true,[]{return I::bltu(numeric_limits<uint32_t>::min(),
        numeric_limits<uint32_t>::max());},"bltu, <");
    expect<bool>(false,[]{return I::bltu(numeric_limits<uint32_t>::max(),
        numeric_limits<uint32_t>::max());},"bltu, =");
    expect<bool>(false,[]{return I::bltu(numeric_limits<uint32_t>::max(),
        numeric_limits<uint32_t>::min());},"bltu, >");

    // bgeu
    expect<bool>(true,[]{return I::bgeu(numeric_limits<uint32_t>::max(),
        numeric_limits<uint32_t>::min());},"bgeu, >");
    expect<bool>(true,[]{return I::bgeu(numeric_limits<uint32_t>::min(),
        numeric_limits<uint32_t>::min());},"bgeu, =");
    expect<bool>(false,[]{return I::bgeu(numeric_limits<uint32_t>::min(),
        numeric_limits<uint32_t>::max());},"bgeu, <");

    // lb
    expect<int32_t>(numeric_limits<int8_t>::max(),
            []{return I::load<int8_t, int32_t>(0x07);}, "lb, +");
    expect<int32_t>(numeric_limits<int8_t>::min(),
            []{return I::load<int8_t, int32_t>(0x80);}, "lb, -");

    // lh
    expect<int32_t>(numeric_limits<int16_t>::max(),
            []{return I::load<int16_t, int32_t>(0x7fff);}, "lh, +");
    expect<int32_t>(numeric_limits<int16_t>::min(),
            []{return I::load<int16_t, int32_t>(0x8000);}, "lh, -");

    // lw
    expect<int32_t>(0x70000, []{return I::load<int32_t, int32_t>(458752);},
        "lw");

    // lbu
    expect<int32_t>(0xff, []{return I::load<uint8_t, int32_t>(255)}, "lbu");

    // lhu
    expect<int32_t>(0xffff, []{return I::load<uint16_t, int32_t>(65535)},
        "lhu");

    // sb
    expect<uint8_t>(0xFF, []{return I::store<int8_t>(-1);}, "sb");

    // sh
    expect<uint16_t>(0xFFFF, []{return I::store<int16_t>(-1);}, "sh");

    // sw
    expect<uint32_t>(0xFFFFFFFF, []{return I::store<int32_t>(-1);}, "sw");

    // addi
    expect<int32_t>(16638, []{return I::addi(0x3fff, 0xff);}, "addi");
    expect<int32_t>(1, []{return I::addi(-1, 2);}, "addi, overflow");

    // slti
    expect<bool>(true, []{return I::slti(-1, 0);}, "slti, true");
    expect<bool>(false, []{return I::slti(0, -1);}, "slti, false");

    // sltiu
    expect<bool>(false, []{return I::sltiu(-1, 0);}, "sltiu, false");
    expect<bool>(true, []{return I::sltiu(0, -1);}, "sltiu, true");
    expect<bool>(true, []{return I::sltiu(0xFFFF, -1);}, "sltiu, sext");

    // xori
    expect<uint32_t>(0xff, []{return I::xori(0xaa, 0x55);}, "xori (1)");
    expect<uint32_t>(0x00, []{return I::xori(0xaa, 0xaa);}, "xori (0)");

    // ori
    expect<uint32_t>(0xff, []{return I::ori(0xaa, 0x55);}, "ori (1)");
    expect<uint32_t>(0xaa, []{return I::ori(0xaa, 0xaa);}, "ori (A)");

    // andi
    expect<uint32_t>(0, []{return I::andi(-1, 0);}, "andi (0)");
    expect<uint32_t>(0x12345678UL, []{return I::andi(0x12345678UL, -1);},
        "andi (1)");

    // slli
    expect<int32_t>(65280, []{return I::slli(255, 8);}, "slli, general");
    expect<int32_t>(numeric_limits<int32_t>::min(),
            []{return I::slli(255, 31);}, "slli, erase");

    // srli
    expect<int32_t>(255, []{return I::srli(65280, 8);}, "srli, general");
    expect<int32_t>(0, []{return I::srli(255, 8);}, "srli, erase");
    expect<int32_t>(1, []{return I::srli(numeric_limits<int32_t>::min(), 31);},
            "srli, negative");

    // srai

    // add
    expect<int32_t>(16638, []{return I::add(0x3fff, 0xff);}, "add");
    expect<int32_t>(-1,
            []{return I::add(0x7fffffff, 0x80000000);},
            "add, overflow");
}
