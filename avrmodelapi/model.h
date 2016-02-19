/**
 * \file
 *
 * \brief Common 8/32-bit simulator model public API base class
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

#ifndef MODEL_H_
#define MODEL_H_

#include <stdio.h>
#include <stdint.h>
#include "core.h"

// --------------------------------------------------------------------------/
// API version number
// --------------------------------------------------------------------------/
#define MODEL_API_MAJOR   07
#define MODEL_API_MINOR   00
#define MODEL_API_PATCH   00

#define CONCAT_(a, b) a ## b
#define CONCAT(a, b) CONCAT_(a, b)
#define MODEL_API_VERSION CONCAT(0x00, CONCAT(MODEL_API_MAJOR, CONCAT(MODEL_API_MINOR, MODEL_API_PATCH)))

// --------------------------------------------------------------------------/
// Forward declarations
// --------------------------------------------------------------------------/
class Model;
//clase Core;

// --------------------------------------------------------------------------/
// TBD declarations
// --------------------------------------------------------------------------/
class SimState;  // For savesim/loadsim arg. @@TODO: Contents is TBD!

// --------------------------------------------------------------------------/
// function pointers
// --------------------------------------------------------------------------/
// Debug report  callback function type (printf compatible)
typedef int Rp_func(const char*, ...);

// Cycle callback function type.
typedef void CycleCallback(Model*, void*);

// --------------------------------------------------------------------------/
// interface to simulator model, which represent a physical chip which may 
// contain one or multiple cores
// --------------------------------------------------------------------------/
class Model
{
public:

    //    virtual ~Model() {}  NO! causes MSVC-mingw incompatibilty!!!

public: // core access

    // ----------------------------------------------------------------------/
    // Get Core
    // @return pointer to Core object or NULL if non-existent
    // ----------------------------------------------------------------------/
    virtual Core* getCore(unsigned id = 0) = 0;


public: // runcontrol functions

    // ----------------------------------------------------------------------/
    // Reset the chip, applying specified reset type.
    // @return a breakpoint pointer if breakpoint was hit, otherwise NULL
    // ----------------------------------------------------------------------/
    virtual Breakpoint* reset(ResetType type = Reset_por) = 0;

    // ----------------------------------------------------------------------/
    // Run #ncycles highest-speed clock cycle. 
    // @param number of clock cycles, default 1
    // @return a breakpoint pointer if breakpoint was hit, otherwise NULL
    // ----------------------------------------------------------------------/
    virtual Breakpoint* cycle(unsigned ncycles = 1) = 0;

    // ----------------------------------------------------------------------/
    // Stop after current instruction has completed. Should be called
    // from callback function.
    // ----------------------------------------------------------------------/
    virtual void stop() = 0;


public: // callback functions

    // ----------------------------------------------------------------------/
    // Add a callback function which will be called in every cycle
    // @param callback function and user data
    // @return handle of added callback if successful and -1 if failed
    // ----------------------------------------------------------------------/
    virtual int addCycleCallback(CycleCallback *cb, void *p) = 0;

    // ----------------------------------------------------------------------/
    // Remove a callback function. Use REMOVE_ALL to remove all.
    // @return # items removed or negative if error.
    // ----------------------------------------------------------------------/
    virtual int removeCycleCallback(int handle) = 0;


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


public: // misc
    virtual int saveSim(class SimState*) = 0;
    virtual int loadSim(class SimState*) = 0;

    // ----------------------------------------------------------------------/
    // Debug hook. Only enabled when model DLL compiled for debugging.
    // When enabled this function will execute a debug command,
    // otherwise it does nothing and returns 0.
    // For internal use only, will always be disabled in production models.
    // ----------------------------------------------------------------------/
    virtual int debug(const char *cmd, Rp_func* func = printf) = 0;

public: // test

    // ----------------------------------------------------------------------/
    // Used for internal test, should not be used by end-user
    // ----------------------------------------------------------------------/
    virtual int test(int command, Test* tbd, TestCallback* func = NULL) = 0;
};

#endif // MODEL_H_
