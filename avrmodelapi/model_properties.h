/**
 * \file
 *
 * \brief Model proprety definitions
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

#ifndef _MODEL_PROPERTIES_H_
#define _MODEL_PROPERTIES_H_


// @@TODO: Add stuff related to multi-core models.
enum ModelProperty
{
    //---------------------------------------------------------------------/
    // Device global properties, can be accessed by both Model:: and
    // Core:: methods
    //---------------------------------------------------------------------/
    P_DeviceName = 0,      // String: Device name
    P_Signature,           // Int: Chip signature.
    P_Revision,            // Revision (representation TBD)
    P_JTAGID,              // Int: JTAG ID if present (?)

    P_ExtClkFreq,          // Int: External clock frequency, only used when
                           // device is externally clocked.

    // Main memories description.
    P_FlashSize,           // Flash size (bytes)
    P_FlashStart,          // Flash start (byte) address
    P_RamSize,             // Internal RAM size (bytes)
    P_RamStart,            // Internal RAM start address
    P_EESize,              // EEPROM size
    P_EEStart,             // EEPROM start address

    P_HasFlashVault,       // FlashVault supported (0: false, 1: true)

    // Simulator properties
    P_SimType,             // Simulator type, see below

    // Fuse/lockbit properties. Fuses/lockbits are mapped into a
    // separate memory range described by these properties.
    P_FuseOffset,
    P_NumFuseBytes,
    P_LockbitOffset,
    P_NumLockbitBytes,

    // Multi-core
    P_NumCores,

    // API subrevision
    P_ApiRevision, 

    // Verilog path (debug models only), named integer property.
    P_VerilogPath,

    //---------------------------------------------------------------------/
    // Per-core properties, can be accessed by Core:: methods only
    //---------------------------------------------------------------------/

    // Memory sizes and start addresses. @@@FIXME: Consider changing names etc
    // eg Flash -> Code memory, etc
    P_CpuClass = 0x400,    // Int: CPU class, see below
    P_CpuClkFreq,          // Int: CPU Clock frequency, read only


    P_IOSize,              // I/O range size
    P_IOStart,             // I/O range start address
    P_RegFileSize,         // # of registers in register file
    P_RegFileStart,        // Start of register file (AVR8L has 16-31). (?)

    // Properties specific for AVR8.
    P_RegFileMemMapped,    // Register file memory mapped.
    // @TODO: P_RegFileAddress // Register file address in memory if mem-mapped

    P_HasFloatingPoint,    // FP instructions supported (0: false, 1: true)

    // Special flash sections present in some devices. These properties
    // are set if they map to user-accessible addresses in the flash segment.
    P_UserPageAddress,
    P_UserPageSize,
    P_FactoryPageAddress,
    P_FactoryPageSize,
    
    // Endianness.
    P_BigEndian,

    // Number of "System Registers" (AVR32), or similar. See model_registers.h
    P_NumSysRegs,

    // Core number. 0 is the primary core and always present.
    P_CoreNum
};


//  CPU architecture main groups
enum CpuArch
{
    // NOTE: Keep at least 0x100 separation between each group, must
    // be able to determine CpuArch = CpuClass & ~0xff (see below)
    // 
    // 
    A_AVR   = 0,
    A_AVR32 = 0x100,
    A_ARM   = 0x200,
    A_MXT   = 0x300,         // maxTouch co-processors
    // new archs go in here
    A_UNK   = ~0xff
};

// Make CpuArch from CpuClass.
#define CPUARCH(CLASS) (CpuArch((CLASS) & ~0xff))

// CPU class
enum CpuClass
{
    AVR8  = A_AVR,           // traditional mega/tiny: mem-mapped regfile[32]
    AVR8L = (A_AVR + 1),     // non-mem-mapped regfile[16], mem-mapped flash
    AVR8_XMEGA = (A_AVR + 2),// non-mem-mapped regfile[32], mem-mapped eeprom
    AVR16 = (A_AVR + 3),     // AVR16 co-processor
    // new 8/16 bit families go in here

    // AVR32
    AVR32_UC3 = A_AVR32,  // AVR32 range, make room for more 8-bit types above
    AVR32_AP7 = (A_AVR32 + 0x10), // Will probably never simulate this...

    // ARM
    ARM_CM0   = A_ARM, // Cortex-M0 based devices (@FIXME: Atmel name TBD)
    ARM_CM3   = (A_ARM + 0x30), // Cortex-M3 based
    ARM_CM4   = (A_ARM + 0x40), // Cortex-M4 based

    // maXtouch co-processors (in the CTE)
    MXT_US     = A_MXT,
    MXT_MAXDSP = A_MXT + 0x10,

    // New 32-bit families go in here

    CPUCLASS_UNK = A_UNK
};


enum SimType
{
    SyntVTOC,
    SyntCarbon,
    InstrSet,
    InstrSetCycleAccurate,
    SyntVerilator
};

#endif // _MODEL_PROPERTIES_H_
