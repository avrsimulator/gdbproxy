/**
 * \file
 *
 * \brief Register definitions
 *
 * Copyright (c) 2016 Atmel Corporation. All rights reserved.
 *
 * \avrmodelapi_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \avrmodelapi_license_stop
 *
 */


// In addition to CPU register file, Model::peekReg and Model::pokeReg
// are used for accessing various special registers like PC, SP, status
// register/CCR, UC3 system registers, cycle counters, etc. (PC and SP
// are in regfile on UC3/ARM but not on AVR).
//
// The lowest 256 addresses are reserved for physical regfile and
// anything else standardized by architecture.
// Above 0x100 is used for mapping (pseudo-)registers that don't fit 
// anywhere else.

#ifndef _MODEL_REGISTERS_H_
#define _MODEL_REGISTERS_H_

enum ModelRegister
{
    // These won't be used much in real code. ARM/UC3 may use R16-R32 for
    // other purposes.
    R0 = 0, R1, R2, R3, R4, R5, R6, R7, R8, R9, R10, R11, R12, R13, R14, R15,
    R16, R17, R18, R19, R20, R21, R22, R23,
    R24, R25, R26, R27, R28, R29, R30, R31,
    // Reserved 0x20..0x100
    R_PC = 0x100, R_INSTR, R_SP, R_STATUS, R_CYCLECOUNT, R_LIFETIMECOUNT,
    // Reserved ..0x400
    R_SYSREG = 0x400,  // UC3 sysregs, or similar
    // Reserved ..0x800
    R_RESERVED = 0x800
};

// Map sysreg number to pseudo-address and vice-versa, see above
#define SYSREGADDR(num) (R_SYSREG + (num))
#define SYSREGNUM(addr) ((addr) - R_SYSREG)


#endif // _MODEL_REGISTERS_H_
