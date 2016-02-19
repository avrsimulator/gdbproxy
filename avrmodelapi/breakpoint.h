/**
 * \file
 *
 * \brief Model breakpoint API base class
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

#ifndef _BREAKPOINT_H_
#define _BREAKPOINT_H_

#include <stdint.h>
#include <string>
#include <cstring>
#include <cstdio>
#include "model_memory.h"

// Breakpoint types.
// BP_exec is an ordinary breakpoint. We don't distiguish between hardware
// and memory breakpoints.
// WP_Mod(ified) differs from WP_Write in that it also triggers
// on a location being modified by hardware (WP_Write watches the
// bus vhile WP_Mod watches the value). WP_Mod is not required by
// AVR Studio and not yet implemented. Use TtVariable->RegisterBreak() to
// implement this!
// BP_none is used internally to indicate user initiated stop.
// BP_Break means BREAK instrcution, a "built_in" breakpoint.
// Notice: combinatorial definitions.
enum BPtype { BP_None=0,BP_Exec=1,WP_Write=2, WP_Read=4, 
              WP_Access = WP_Write | WP_Read,
              WP_Change=8, BP_Break = 16};

class Model;
class Breakpoint;
class Core;


enum BPaction
{
    BP_Continue = 0,
    BP_BreakStep = 1,    // Complete instruction, then break
    BP_BreakCycle = 2    // Break immediately
};

#define BP_MAXVPATH 256  // Max length of a Verilog path.


/*
 * Breakpoint/tracepoint callback function.
 * @return: 0 to continue execution, 1 to break, other values undefined.
 */
typedef int BreakCallback(Core *c, Breakpoint *bp);

// At this level, a breakpoint always stops, unless a callback
// function is specified, in which case it is referred to as a tracepoint.
// Enabling/disabling,  alternative actions,
// expressions, etc, must be implemented by user.
// 
class Breakpoint
{
 public:
    // Default constructor. Required for populating std::map.
    // Not sure it actually requres initialization, but better
    // safe than sorry. This breakpoint will never hit.
    Breakpoint():
        m_core(0), m_address((uint64_t)(-1)),
        m_busaddr((uint64_t)(-1)), m_size(1), m_segment(Seg_Code),
        m_type(BP_None), m_actaddr((uint64_t)(-1)), m_actacc(BP_None),
        m_pc((uint64_t)(-1)), m_hitcount(0), m_name(),
        m_cb(0), m_userdata(0)
    {};

    // Construct a code breakpoint. This only requires an address.
    Breakpoint(uint64_t address, unsigned core = 0):
        m_core(core), m_address(address),
        m_busaddr(address), m_size(1), m_segment(Seg_Code),
        m_type(BP_Exec), m_actaddr(address), m_actacc(BP_Exec),
        m_pc((uint64_t)(-1)), m_hitcount(0), m_name(),
        m_cb(0), m_userdata(0)
    {};

    // Construct a code tracepoint. This only requires an address and callback.
    Breakpoint(uint64_t address, BreakCallback *cb, void *userdata = 0, 
               unsigned core = 0):
        m_core(core), m_address(address),
        m_busaddr(address), m_size(1), m_segment(Seg_Code),
        m_type(BP_Exec), m_actaddr(address), m_actacc(BP_Exec),
        m_pc((uint64_t)(-1)), m_hitcount(0), m_name(),
        m_cb(cb), m_userdata(userdata)
    {};

    // Construct a watchpoint (aka data breakpoint).
    Breakpoint(BPtype type, Segment seg, uint64_t address, uint64_t size = 1,
               unsigned core = 0):
        m_core(core), m_address(address),
        m_busaddr(address), m_size(size), m_segment(seg),
        m_type(type), m_actaddr(address), m_actacc(BP_None),
        m_pc((uint64_t)(-1)), m_hitcount(0), m_name(),
        m_cb(0), m_userdata(0)
    {};


    // Construct a data tracepoint
    Breakpoint(BPtype type, Segment seg, uint64_t address, uint64_t size,
               BreakCallback *cb, void *userdata = 0, unsigned core = 0):
        m_core(core), m_address(address),
        m_busaddr(address), m_size(size), m_segment(seg),
        m_type(type), m_actaddr(address), m_actacc(BP_None),
        m_pc((uint64_t)(-1)), m_hitcount(0), m_name(),
        m_cb(cb), m_userdata(userdata)
    {};


    // Construct a Verilog breakpoint. This only requires a Verilog path.
    Breakpoint(const char *vpath, BreakCallback *cb = 0, void *userdata = 0):
        m_core(0), m_address(0),
        m_busaddr(0), m_size(1), m_segment(Seg_Code),
        m_type(WP_Change), m_actaddr(0), m_actacc(BP_Exec),
        m_pc(0), m_hitcount(0), /*m_name(name),*/ 
        m_cb(cb), m_userdata(userdata)
    {
        unsigned len = strlen(vpath);
        // see coments in breakpoint.h
        if (len < BP_MAXVPATH)
            std::strcpy(m_name, vpath);
        else
        {
            std::strncpy(m_name, vpath, BP_MAXVPATH - 1);
            m_name[BP_MAXVPATH- 1] = '\0'; // truncate & ensure 0-termination.
            std::fprintf(stderr,
                         "Breakpoint: Verilog path too long (%u), max = %u\n",
                         len, BP_MAXVPATH);
        }
    };



    bool operator==(const Breakpoint& rv) const {
        return (m_address == rv.m_address && m_type == rv.m_type
                && m_segment == rv.m_segment && m_size == rv.m_size
                && m_core == rv.m_core && m_cb == rv.m_cb
                && m_userdata == rv.m_userdata);
    };

    int      m_handle;    // Breakpoint handle, unique number.

    // request part of breakpoint
    unsigned m_core;      // Core number
    uint64_t m_address;   // memory address
    uint64_t m_busaddr;   // bus address
    uint64_t   m_size;
    Segment  m_segment;
    BPtype   m_type;
    // report part of breakpoint
    uint64_t m_actaddr;   // actual address when WP is hit (size != 1)
    BPtype   m_actacc;    // actual access mode when WP is hit (type == Access)
    // unsigned m_data;      // data read/written NOT IMPLEMENTED
    uint64_t m_pc;        // program address when last hit
    unsigned m_hitcount;
    // @@FIXME: the below is wasteful on storage and can truncate long
    // verilog paths. Cannot use std::string here, and managing dynamic
    // char* storage is complicated.
    char     m_name[BP_MAXVPATH];
    BreakCallback *m_cb;
    void      *m_userdata;
};


#endif // _BREAKPOINT_H_
