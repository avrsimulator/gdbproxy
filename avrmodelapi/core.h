/**
 * \file
 *
 * \brief Common 8/32-bit core public API base class
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

#ifndef CORE_H_
#define CORE_H_

#ifdef WIN32
#define __STDC_LIMIT_MACROS
#endif

#include <stdio.h>
#include <stdint.h>
//#include <cstdint>  //@@@TODO: Check which one of these MSVC likes.
#include <limits>
#include "breakpoint.h"
#include "model_properties.h"
#include "model_registers.h"
#include "model_memory.h"


// --------------------------------------------------------------------------/
// Forward declarations
// --------------------------------------------------------------------------/
class Model;
class Core;

// --------------------------------------------------------------------------/
// TBD declarations
// --------------------------------------------------------------------------/
class Test;   // For test() method, i.e. testing new features without
              // changing API
typedef int TestCallback(Model *m);  // Callback function for test()


// --------------------------------------------------------------------------/
// function pointers
// --------------------------------------------------------------------------/
// Step callback function type.
typedef void StepCallback(Core*, void*);


// User-specified reset type. @@@TODO: Global vs per-core reset?
enum ResetType  { 
    Reset_por, 
    Reset_ext, 
    Reset_bod, 
    Reset_spike,   // xmega only
};


// --------------------------------------------------------------------------/
// interface to simulator model core, which represent a CPU core contained
// in a physical chip (model).
// --------------------------------------------------------------------------/
class Core
{
public:

    //    virtual ~Core() {} NO! causes MSVC-mingw incompatibilty!!!

    // For methods removing some object by id/handle being a positive int,
    // this value means remove all such objects.
    static const int REMOVE_ALL = 0;

public: // runcontrol functions

    // ----------------------------------------------------------------------/
    // Reset the chip, applying specified reset type.
    // @return a breakpoint pointer if breakpoint was hit, otherwise NULL
    // ----------------------------------------------------------------------/
    virtual Breakpoint* reset(ResetType type = Reset_por) = 0;

    // ----------------------------------------------------------------------/
    // Execute current instruction and stop
    // @return a breakpoint pointer if breakpoint was hit, otherwise NULL
    // ----------------------------------------------------------------------/
    virtual Breakpoint* step(uint64_t num = 1) = 0;

    // ----------------------------------------------------------------------/
    // Run until either a breakpoint is hit or reached the address
    // @param the byte address of intended stop instruction, default value 
    //        means run without stopping
    // @return the breakpoint pointer once a breakpoint is hit otherwise NULL
    // ----------------------------------------------------------------------/
    virtual Breakpoint* run(uint64_t addr
                            = std::numeric_limits<uint64_t>::max()) = 0;

    // ----------------------------------------------------------------------/
    // Stop after current instruction has completed. Should be called
    // from runcallback function.
    // ----------------------------------------------------------------------/
    virtual void stop() = 0;


public: // callback functions

    // ----------------------------------------------------------------------/
    // Add a callback function which will be called in every step
    // @param callback function and user data
    // @return handle of added callback if successful and -1 if failed
    // ----------------------------------------------------------------------/
    virtual int addStepCallback(StepCallback *cb, void *p) = 0;

    // ----------------------------------------------------------------------/
    // Remove a callback function. Use REMOVE_ALL to remove all.
    // @return # items removed or negative if error.
    // ----------------------------------------------------------------------/
    virtual int removeStepCallback(int handle) = 0;


public: // register access

    // ----------------------------------------------------------------------/
    // Get the speical register value
    // @param the index of register as in the enum
    // @return -1 if not exist and size of register if reading sucessfully
    // ----------------------------------------------------------------------/
    virtual int peekReg(int index, uint64_t* val) = 0;

    // ----------------------------------------------------------------------/
    // Set the speical register value
    // @param the index of register as in the enum
    // @return -1 if not exist and size of register if writing sucessfully
    // ----------------------------------------------------------------------/
    virtual int pokeReg(int index, uint64_t val) = 0;


public: // memory access
    // ----------------------------------------------------------------------/
    // Return memory map as seen from this core.
    // @return Pointer to memory map or NULL if not implemented.
    // ----------------------------------------------------------------------/
    virtual MemoryMap *getMemoryMap() = 0;

    // ----------------------------------------------------------------------/
    // Read memory content
    // Notice these functions do not wrap and can return a short count
    // @return the number of bytes read
    // ----------------------------------------------------------------------/
    virtual int readMemory(uint64_t address, uint64_t size, uint8_t *data,
            Segment seg) = 0;

    // ----------------------------------------------------------------------/
    // Write memory content
    // @return the number of bytes written
    // ----------------------------------------------------------------------/
    virtual int writeMemory(uint64_t address, uint64_t size,
                            const uint8_t *data,  Segment seg) = 0;


public: // property access

    // ----------------------------------------------------------------------/
    // Get integer property. `name' may be specified to get a named
    // prpoerty value from a set (eg. named Verilog location)
    // @return -1 if property not exist and positive value if sucessfull
    // ----------------------------------------------------------------------/
    virtual int getIntProperty(int propId, uint64_t *value,
                               const char *name = 0) = 0;

    // ----------------------------------------------------------------------/
    // Set named property value- `name' can be used to set a named property
    // value in a set (eg. named Verilog location)
    // ----------------------------------------------------------------------/
    virtual int setIntProperty(int propId,  uint64_t value,
                               const char *name = 0) = 0;

    // ----------------------------------------------------------------------/
    // Get a string property. `value' can be used to get a string
    // property associated with a value from a set (reverse of the above)
    // ----------------------------------------------------------------------/
    virtual int getStringProperty(int propId, size_t size, char* buf,
                                  const uint64_t *value = 0) = 0;
 
    // ----------------------------------------------------------------------/
    // Set a string property. `value' can be used to set a string
    // property associated with a value in a set (reverse of the above)
    // ----------------------------------------------------------------------/
    virtual int setStringProperty(int propId, char* buf,
                                  const uint64_t *value = 0) = 0;


public: // upper-level device accessor

    // ----------------------------------------------------------------------/
    // Get containing Model object
    // ----------------------------------------------------------------------/
    virtual Model* getModel() = 0;


public: // breakpoint / watchpoint / tracepoint

    // ----------------------------------------------------------------------/
    // Add a break/watchpoint.
    // For breakpoint construction, only address is specified. See
    // breakpoint.h for how to construct a watchpoint.
    // @return Positive breakpoint id on success, 0 if already exists, or
    // negative number if error.
    // ----------------------------------------------------------------------/
    virtual int addBreakpoint(Breakpoint *b) = 0;

    // ----------------------------------------------------------------------/
    // Remove break/watchpoint specified by id. Use REMOVE_ALL to
    // remove all.
    // @returns: Number of breakpoints removed, negative number if error.
    // ----------------------------------------------------------------------/
    virtual int removeBreakpoint(int id) = 0;

    // ----------------------------------------------------------------------/
    // Get all break/watchpoints of specified types (default = all)
    // WARNING: returned array is not updated when breakpoints added/removed. 
    // WARNING: returned array overwritten by successive calls.
    // @return  NULL-terminated array
    // ----------------------------------------------------------------------/
    virtual Breakpoint **getBreakpoints(BPtype type 
                                 = BPtype(BP_Exec|WP_Access|WP_Change)) = 0;


public: // test

    // ----------------------------------------------------------------------/
    // Used for internal test, should not be used by end-user
    // ----------------------------------------------------------------------/
    virtual int test(int command, Test* tbd, TestCallback* func = NULL) = 0;
};

#endif // CORE_H_
