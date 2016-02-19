/**
 * \file
 *
 * \brief AVR32 register definitions
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
/**
 * This file contains definitions that may be needed when using AVR32
 * (currently uc3) models.
 */

/*********************************************************************
AVR32 register file shadowing.

The Model API presents the software view of the AVR32 register file (i.e.
according to current CPU mode) at base address 0 (RF_CURRENT_BASE)

Higher base addresses can be used to force a specific mode view.
Example:

 sp = model->peekReg(RF_SP);    // get the current stack pointer

 sp_sys = model->peekReg(RF_PRIVMODE_BASE + RF_SP);
                                   // get the prvileged
                                   // mode stack ptr (SP_SYS),
                                   // regardless of current mode.

Registers that are not shadowed will be the same regardless of base
address used in API call. The amount of shadowing depends on HW
implementation, uc3 devices currently only shadow SP.

**********************************************************************/

#ifndef MODEL_AVR32_H_
#define MODEL_AVR32_H_


// Offset of special registers in regfile
// Base register addresses are in the current context (app/priv/sec)
#define RF_SP 0x0d        // Context dependent stack pointer (=R13)
#define RF_LR 0x0e        // Link register (=R14)
#define RF_PC 0x0f        // Program counter (not really in regfile) (=R15)

#define RF_SIZE 0x10      // Size of user view of regfile
#define RF_MASK 0x0f

// Force view in alternative modes. In UC3 this only affects stack pointer.
// This concept is however extensible to more register shadowing (ala AP7)
#define RF_CURRENT_BASE  0x00  // Base address for current mode
#define RF_APPMODE_BASE  0x10  // Base address for application mode
#define RF_PRIVMODE_BASE 0x20  // Base address for privileged mode
#define RF_SECMODE_BASE  0x30  // Base address for secure state

#endif // MODEL_AVR32_H_

