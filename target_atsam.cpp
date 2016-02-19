/* Copyright (C) 2002 Chris Liechti and Steve Underwood
   Copyright (C) 2016 Atmel, Inc.
 
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
 
     1. Redistributions of source code must retain the above copyright notice,
        this list of conditions and the following disclaimer.
     2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
     3. The name of the author may not be used to endorse or promote products
        derived from this software without specific prior written permission.
 
   THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
   WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
   EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
 
   Implementation of a Atmel SAM target for the GDB proxy server.
   
   Exported Data:
     atsam_target             - target descriptor of the `atsam' target 
  
   Imported Data:
     None     
  
   Static Data:
     atsam_XXXX               - static data representing status and 
                                   parameters of the target 
  
   Global Functions:  
     None
  
   Static Functions:  
     atsam_XXXX              - methods comprising the `atsam' target.
                                  A description is in file gdbproxy.h
 
     atsam_                  - local finctions
     atsam_command
 
   $Id: arm.c,v 0.1 2013/08/15 11:41:51 JieXu Exp $ */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <stdarg.h>

#include "gdbproxy.h"
#if WIN32
#include "getopt.h"
#endif

#if defined(__FreeBSD__)
#include "getopt.h"
#endif

/* Included for use with callback */
#include "rpmisc.h"

/* Include Atmel simulator model API */
#include "model.h"
#include "modelmgr/modelmgr.h"

#ifdef __unix__                                                                                          
const char dll[] = "so";                                                                                 
#else                                                                                                    
const char dll[] = "dll";
#endif

/* Note: we are using prefix 'atsam' for static stuff in
   order to simplify debugging of the target code itself */

/* TODO: Put the correct values for the real target in these macros */
#define RP_ATSAM_MIN_ADDRESS             0x0U
#define RP_ATSAM_MAX_ADDRESS             0xFFFFFFFFULL
#define RP_ATSAM_NUM_REGS                16
#define RP_ATSAM_REG_DATATYPE            uint32_t
#define RP_ATSAM_REG_BYTES               (RP_ATSAM_NUM_REGS*sizeof(RP_ATSAM_REG_DATATYPE))
#define RP_ATSAM_REGNUM_PC               15  /* Program counter reg. number */
#define RP_ATSAM_REGNUM_SP               13  /* Stack pointer reg. number */
#define RP_ATSAM_REGNUM_FP               7  /* Frame pointer reg. number */
/* According to EABI, FP is r7 for THUMB and r11 otherwise. */

#define RP_ATSAM_NUM_XREGS               66
/* Redefining register locations according to ../verilator/cm0p/arm_regdefs.vh */
#define XREG_APSR                        21

#define RP_ATSAM_MAX_BREAKPOINTS         512 /* This is a simulator, in theory unlimited :-) */

/* Some example states a real target might support. */
#define RP_ATSAM_TARGET_STATE_RUNNING                    0
#define RP_ATSAM_TARGET_STATE_STOPPED                    1
#define RP_ATSAM_TARGET_STATE_SINGLE_STEP_COMPLETE       2
#define RP_ATSAM_TARGET_STATE_RUN_TO_ADDR_COMPLETE       3
#define RP_ATSAM_TARGET_STATE_BREAKPOINT_HIT             4

/*
 * Target methods, static
 */
static void  atsam_help(const char *prog_name);
static int   atsam_open(int argc,
                           char * const argv[],
                           const char *prog_name,
                           log_func log_fn);
static void  atsam_close(void);
static int   atsam_connect(char *status_string,
                              size_t status_string_size,
                              int *can_restart);
static int   atsam_disconnect(void);
static void  atsam_kill(void);
static int   atsam_restart(void);
static void  atsam_stop(void);
static int   atsam_set_gen_thread(rp_thread_ref *thread);
static int   atsam_set_ctrl_thread(rp_thread_ref *thread);
static int   atsam_is_thread_alive(rp_thread_ref *thread, int *alive);
static int   atsam_read_registers(uint8_t *data_buf,
                                     uint8_t *avail_buf,
                                     size_t buf_size,
                                     size_t *read_size);
static int   atsam_write_registers(uint8_t *data_buf, size_t write_size);
static int   atsam_read_single_register(unsigned int reg_no,
                                           uint8_t *data_buf,
                                           uint8_t *avail_buf,
                                           size_t buf_size,
                                           size_t *read_size);
static int   atsam_write_single_register(unsigned int reg_no,
                                            uint8_t *data_buf,
                                            size_t write_size);
static int   atsam_read_mem(uint64_t addr,
                               uint8_t *data_buf,
                               size_t req_size,
                               size_t *actual_size);
static int   atsam_write_mem(uint64_t addr,
                                uint8_t *data_buf,
                                size_t req_sise);
static int   atsam_resume_from_current(int step, int sig);
static int   atsam_resume_from_addr(int step,
                                       int sig,
                                       uint64_t addr);
static int   atsam_go_waiting(int sig);
static int   atsam_wait_partial(int first,
                                   char *status_string,
                                   size_t status_string_len,
                                   out_func out,
                                   int *implemented,
                                   int *more,
                                   rp_target* t);
static int   atsam_wait(char *status_string,
                           size_t status_string_len,
                           out_func out,
                           int *implemented,
                           rp_target* t);
static int   atsam_process_query(unsigned int *mask,
                                    rp_thread_ref *arg,
                                    rp_thread_info *info);
static int   atsam_list_query(int first,
                                 rp_thread_ref *arg,
                                 rp_thread_ref *result,
                                 size_t max_num,
                                 size_t *num,
                                 int *done);
static int   atsam_current_thread_query(rp_thread_ref *thread);
static int   atsam_offsets_query(uint64_t *text,
                                    uint64_t *data,
                                    uint64_t *bss);
static int   atsam_crc_query(uint64_t addr,
                                size_t len,
                                uint32_t *val);
static int   atsam_raw_query(char *in_buf,
                                char *out_buf,
                                size_t out_buf_size);
static int   atsam_add_break(int type, uint64_t addr, unsigned int len);
static int   atsam_remove_break(int type, uint64_t addr, unsigned int len);

static uint32_t crc32(uint8_t *buf, size_t len, uint32_t crc);

#define RCMD(name, hlp) {#name, atsam_rcmd_##name, hlp}  //table entry generation

/* Prototyping of remote commands */
static int atsam_rcmd_erase(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atsam_rcmd_setlib(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atsam_rcmd_getlib(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atsam_rcmd_setdevice(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atsam_rcmd_getdevice(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atsam_rcmd_reset(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atsam_rcmd_getcyclecnt(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atsam_rcmd_getlifecnt(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atsam_rcmd_debug(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atsam_rcmd_info(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atsam_rcmd_lastbreak(int argc, char *argv[], out_func of, data_func df, rp_target* t);

/* Table of remote commands */
static const RCMD_TABLE atsam_remote_commands[] =
{
    RCMD(erase,         "Erase target flash memory"),
    RCMD(setlib,        "Sets the name of the library file to load. \"setlib lib [device]\""),
    RCMD(getlib,        "Printss the name of the selected library file"),
    RCMD(setdevice,     "Sets the name of the device to load"),
    RCMD(getdevice,     "Prints the name of the selected device"),
    RCMD(reset,         "Reset target. Optional reset types: por bod ext spike"),
    RCMD(getcyclecnt,   "Get the cycle counter"),
    RCMD(getlifecnt,    "Get the life time counter"),
    RCMD(debug,         "Command for extended debug features. Requires model built for debug."),
    RCMD(info,          "Extended target information"),
    RCMD(lastbreak,     "Extended information about the last breakpoint that was hit."),
    {0,0,0}     //sentinel, end of table marker
};

/* Target XML description */
const char atsam_target_xml[] = "<?xml version=\"1.0\"?>\n<!-- Copyright (C) 2008 Free Software Foundation, Inc.\n\n     Copying and distribution of this file, with or without modification,\n     are permitted in any medium without royalty provided the copyright\n     notice and this notice are preserved.  -->\n\n<!DOCTYPE feature SYSTEM \"gdb-target.dtd\">\n<target>\n  <architecture>arm</architecture>\n<feature name=\"org.gnu.gdb.arm.m-profile\">\n    <reg name=\"r0\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"r1\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"r2\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"r3\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"r4\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"r5\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"r6\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"r7\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"r8\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"r9\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"r10\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"r11\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"r12\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"sp\" bitsize=\"32\" type=\"data_ptr\" group=\"general\"/>\n    <reg name=\"lr\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"pc\" bitsize=\"32\" type=\"code_ptr\" group=\"general\"/>\n    <reg name=\"xpsr\" bitsize=\"32\" regnum=\"25\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"MSP\" bitsize=\"32\" regnum=\"26\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"PSP\" bitsize=\"32\" regnum=\"27\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"PRIMASK\" bitsize=\"32\" regnum=\"28\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"BASEPRI\" bitsize=\"32\" regnum=\"29\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"FAULTMASK\" bitsize=\"32\" regnum=\"30\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"CONTROL\" bitsize=\"32\" regnum=\"31\" type=\"uint32\" group=\"general\"/>\n  </feature>\n</target>\n\n";

// String arrays, must be kept in sync with model.h/breakpoint.h enums.
static const char *bptypes[] =
{ "none", "exec", "write", "N/A=3", "read", "N/A=5", "access", "N/A=7",
  "modify"};
static const char *segtypes[] =
{ "code", "data", "eeprom", "reg", "i/o", "sysreg", "none" };

/*
 * Global target descriptor 
 */
rp_target atsam_target =
{
    NULL,      /* next */
    -1,
    -1,
    0,
    0,
    0,
    0,
    "atsam",
    "atsam target to use GDB proxy server with Atmel SAM simulator models",
    atsam_remote_commands,
    atsam_help,
    atsam_open,
    atsam_close,
    atsam_connect,
    atsam_disconnect,
    atsam_kill,
    atsam_restart,
    atsam_stop,
    atsam_set_gen_thread,
    atsam_set_ctrl_thread,
    atsam_is_thread_alive,
    atsam_read_registers,
    atsam_write_registers,
    atsam_read_single_register,
    atsam_write_single_register,
    atsam_read_mem,
    atsam_write_mem,
    atsam_resume_from_current,
    atsam_resume_from_addr,
    atsam_go_waiting,
    atsam_wait_partial,
    atsam_wait,
    atsam_process_query,
    atsam_list_query,
    atsam_current_thread_query,
    atsam_offsets_query,
    atsam_crc_query,
    atsam_raw_query,
    atsam_add_break,
    atsam_remove_break
};

struct atsam_status_s
{
    /* Start up parameters, set by atsam_open */
    log_func    log;
    int         is_open;

    /* Tell wait_xxx method the notion of whether or not
       previously called resume is supported */
    int         target_running;
    int         target_interrupted;
    int         state;
    RP_ATSAM_REG_DATATYPE    registers[RP_ATSAM_NUM_REGS];
    uint64_t                    breakpoints[RP_ATSAM_MAX_BREAKPOINTS];
    bool        deferUnload;
    char       *lib;
    char       *device;
    unsigned    selectedCore;
};

static struct atsam_status_s atsam_status =
{
    NULL,
    FALSE,
    FALSE,
    FALSE,
    RP_ATSAM_TARGET_STATE_STOPPED,
    {0},
    {0},
    TRUE,
    '\0',
    '\0',
    0
};

/* Local functions */
static char *atsam_out_treg(char *in, unsigned int reg_no);
// static int refresh_registers(void); // Not used. Access model directly.
static void printErr(const ModelError &err);
int atsam_load_model(out_func of, rp_target* t);

/* Local variables */
static Core *mC;
static Model *mM;
static ModelMgr *mMgr;
static Breakpoint *mBp;

/* Run callback function to catch intterupt from gdb client */
static void stepcallback(Core *c, void *p)
{
    /* Use timeout == 0 to make just one poll. Break on Ctrl-C  */
//jie    if(dbg_sock_readchar(0) == '\3')
//jie        atsam_stop();
}

uint64_t atsam_getIntProp(Core *c, int propId, const char *name)
{
    uint64_t  val = 0;
    if (c->getIntProperty(propId, &val, name))
        return val;
    else if (propId == P_Signature)
        return atsam_getIntProp(c, P_JTAGID, name);
    else
        return 0;
}


uint64_t atsam_getIntProp(Model *m, int propId, const char *name)
{
    return atsam_getIntProp(m->getCore(atsam_status.selectedCore), propId, name);
}


/* Target method */

static void atsam_help(const char *prog_name)
{
    printf("This is the atsam target for the GDB proxy server. Usage:\n\n");
    printf("  %s [options] %s [atsam-options]\n",
           prog_name,
           atsam_target.name);
    printf("\nOptions:\n\n");
    printf("  --debug              run %s in debug mode\n", prog_name);
    printf("  --help               `%s --help %s'  prints this message\n",
           prog_name,
           atsam_target.name);
    printf("  --port=PORT          use the specified TCP port\n");
    printf("\natsam-options:\n\n");
    printf("  --model=MODEL        use the specified .%s file\n", dll);
    printf("  --device=DEVICE      use the specified device\n");

    printf("\n");

    return;
}

/* Target method */
static int atsam_open(int argc,
                         char * const argv[],
                         const char *prog_name,
                         log_func log_fn)
{
   /* Option descriptors */
    static struct option long_options[] =
    {
        /* Options setting flag */
        {"model",  required_argument, 0, 1},
        {"device", required_argument, 0, 2},
        {NULL, no_argument, 0, 0}
    };

    assert(!atsam_status.is_open);
    assert(prog_name != NULL);
    assert(log_fn != NULL);

    /* Set log */
    atsam_status.log = log_fn;

    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_open()",
                        atsam_target.name);

    /* Reset optind to scan all arguments */
    optind = 1;

    /* Process options */
    for (;;)
    {
        int c;
        int option_index;

        c = getopt_long(argc, argv, "+", long_options, &option_index);
        if (c == EOF)
            break;
        switch (c)
        {
        case 1:
            atsam_status.lib = strcpy((char*)malloc(strlen(optarg) + 1), (optarg));
            atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: lib option: %s, lib: %s",
                            atsam_target.name, optarg, atsam_status.lib);
            break;
        case 2:
            atsam_status.device = strcpy((char*)malloc(strlen(optarg) + 1), (optarg));
            atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: device option: %s, device: %s",
                            atsam_target.name, optarg, atsam_status.device);
            break;
        case 0:
            /* Long option which just sets a flag */
            break;
        default:
            atsam_status.log(RP_VAL_LOGLEVEL_NOTICE,
                                "%s: Use `%s --help %s' to see a complete list of options",
                                atsam_target.name,
                                prog_name,
                                atsam_target.name);
            return RP_VAL_TARGETRET_ERR;
        }
    }


    if (!atsam_status.is_open)
    {
        /* TODO: Verify precense of model file, but load() in modelmgr.cpp
         * still fails unless we specify path (i.e. prepend with "./" */
        if(access(atsam_status.lib, F_OK) == -1)
        {
            atsam_status.log(RP_VAL_LOGLEVEL_ERR,
                                "%s: Can not find file specified!",
                                atsam_target.name);
            return RP_VAL_TARGETRET_ERR;
        }
    }

    /* Set up initial default values */
    atsam_status.target_running = FALSE;
    atsam_status.target_interrupted = FALSE;
    atsam_status.state = RP_ATSAM_TARGET_STATE_STOPPED;
    memset (atsam_status.registers, 0, sizeof(atsam_status.registers));
    memset (atsam_status.breakpoints, 0, sizeof(atsam_status.breakpoints));

    atsam_status.is_open = TRUE;

    return RP_VAL_TARGETRET_OK;
}


/* Target method */
static void atsam_close(void)
{
    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_close()",
                        atsam_target.name);

    assert(atsam_status.is_open);

    /* TODO: Tidy up things and shut down. */

    atsam_status.is_open = FALSE;

    free(atsam_status.lib);
    atsam_status.lib = NULL;
    free(atsam_status.device);
    atsam_status.device = NULL;
    free(mBp);
    mBp = NULL;
    delete mMgr;
}

/* Target method */
static int atsam_connect(char *status_string,
                            size_t status_string_len,
                            int *can_restart)
{
    char *cp;
    static ModelMgr mgr;

    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_connect()",
                        atsam_target.name);

    if(!atsam_status.lib)
    {
        atsam_status.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: No simulator model specified!",
                            atsam_target.name);
        return RP_VAL_TARGETRET_ERR;
    }

    assert(atsam_status.is_open);

    assert(status_string != NULL);
    assert(status_string_len >= 34);
    assert(can_restart != NULL);

    *can_restart = TRUE;

    mgr.setDeferUnload(atsam_status.deferUnload);
    mM = mgr.load(atsam_status.lib, atsam_status.device);
    if(mM)
    {
        atsam_status.log(RP_VAL_LOGLEVEL_NOTICE,
                           "%s: Simulator model loaded",
                           atsam_target.name);
        mMgr = &mgr;
        int numcores = atsam_getIntProp(mM, P_NumCores, 0);
        for (int i = 0; i < numcores; i++)
        {
            mC = mM->getCore(i);
            if (!mC || mC->addStepCallback(stepcallback, NULL) <= 0)
                printf("addStepCallback(core %d) failed."
                        " Ctrl-C WILL NOT WORK\n", i);
        }
        mC = mM->getCore(atsam_status.selectedCore);

        char name_buf[256];
        if(mM->getStringProperty(P_DeviceName, sizeof(name_buf), name_buf))
            atsam_status.device = strcpy((char*)malloc(strlen(name_buf) + 1), (name_buf));
    }
    else
    {
        atsam_status.log(RP_VAL_LOGLEVEL_NOTICE,
                           "%s: Loading of simulator model failed",
                           atsam_target.name);
        return RP_VAL_TARGETRET_ERR;
    }

    /* Fill out the the status string */
    sprintf(status_string, "T%02d", RP_SIGNAL_ABORTED);

    
    cp = atsam_out_treg(&status_string[3], RP_ATSAM_REGNUM_PC);
    cp = atsam_out_treg(cp, RP_ATSAM_REGNUM_FP);

    return (cp != NULL)  ?  RP_VAL_TARGETRET_OK  :  RP_VAL_TARGETRET_ERR;
}

/* Target method */
static int atsam_disconnect(void)
{
    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_disconnect()",
                        atsam_target.name);

    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static void atsam_kill(void)
{
    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_kill()",
                        atsam_target.name);

    /* Kill the target debug session. */
    atsam_stop();
}

static int atsam_restart(void)
{
    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_restart()",
                        atsam_target.name);

    /* Just stop it. The actual restart will be done
       when connect is called again */
    atsam_stop();

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static void atsam_stop(void)
{
    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_stop()",
                        atsam_target.name);

    assert(atsam_status.is_open);

    /* Stop (i,e, break) the target program. */
    mC->stop();

    atsam_status.target_interrupted = TRUE;
    atsam_status.state = RP_ATSAM_TARGET_STATE_STOPPED;
}

static int atsam_set_gen_thread(rp_thread_ref *thread)
{
    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_set_gen_thread()",
                        atsam_target.name);

    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int atsam_set_ctrl_thread(rp_thread_ref *thread)
{
    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_set_ctrl_thread()",
                        atsam_target.name);

    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int atsam_is_thread_alive(rp_thread_ref *thread, int *alive)
{
    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_is_thread_alive()",
                        atsam_target.name);

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atsam_read_registers(uint8_t *data_buf,
                                 uint8_t *avail_buf,
                                 size_t buf_size,
                                 size_t *read_size)
{
    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_read_registers()",
                        atsam_target.name);

    assert(atsam_status.is_open);

    assert(data_buf != NULL);
    assert(avail_buf != NULL);
    assert(buf_size >= RP_ATSAM_REG_BYTES);
    assert(read_size != NULL);

//    *read_size = c->readMemory(0,
//                                RP_ATSAM_REG_BYTES,
//                                data_buf, Seg_Reg);
//    assert (read_size == RP_ATSAM_REG_BYTES);  // @@@FIXME

    for(uint8_t index = 0; index < RP_ATSAM_NUM_REGS; index++)
    {
        uint64_t regVal;
        mC->peekReg(index, &regVal);
        *(data_buf+(index*sizeof(RP_ATSAM_REG_DATATYPE))) = (RP_ATSAM_REG_DATATYPE)regVal;
    }
    *read_size = RP_ATSAM_REG_BYTES;

    memset(avail_buf, 1, RP_ATSAM_REG_BYTES);
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atsam_write_registers(uint8_t *buf, size_t write_size)
{
    RP_ATSAM_REG_DATATYPE *data_buf = (RP_ATSAM_REG_DATATYPE*)buf;

    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_write_registers()",
                        atsam_target.name);

    assert(atsam_status.is_open);

    assert(buf != NULL);
    assert(write_size > 0);
    assert(write_size <= RP_ATSAM_REG_BYTES);

//    write_size = mC->writeMemory(0,
//                                RP_ATSAM_REG_BYTES,
//                                buf, Seg_Reg);
//    assert (write_size == RP_ATSAM_REG_BYTES);  // @@@FIXME

    for(uint8_t index = 0; (index*sizeof(RP_ATSAM_REG_DATATYPE)) < write_size; index++)
    {
        //uint64_t regVal = *(data_buf+(index*sizeof(RP_ATSAM_REG_DATATYPE)));
        uint64_t regVal = *(data_buf+index);
        mC->pokeReg(index, regVal);
    }
    write_size = RP_ATSAM_REG_BYTES;
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atsam_read_single_register(unsigned int reg_no,
                                         uint8_t *data_buf,
                                         uint8_t *avail_buf,
                                         size_t buf_size,
                                         size_t *read_size)
{
    uint64_t regVal;

    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_read_single_register(%d)",
                        atsam_target.name, reg_no);

    assert(atsam_status.is_open);

    assert(data_buf != NULL);
    assert(avail_buf != NULL);
    assert(buf_size >= sizeof(RP_ATSAM_REG_DATATYPE));
    assert(read_size != NULL);

    if (reg_no < 0  ||  reg_no > RP_ATSAM_NUM_XREGS)
        return RP_VAL_TARGETRET_ERR;

    /* There probably is need for a more complete register translation of
     * registers beyond R15.
    */
    /* Asking for status register */
    if(reg_no == 25)
    {
        reg_no = R_STATUS;   // XREG_APSR;
    }
    mC->peekReg(reg_no, &regVal);
    *read_size = mC->readMemory(reg_no * sizeof(RP_ATSAM_REG_DATATYPE),
                                sizeof(RP_ATSAM_REG_DATATYPE),
                                data_buf, Seg_Reg);

    assert (*read_size == sizeof(RP_ATSAM_REG_DATATYPE));  // @@@FIXME

    memset(avail_buf, 1, sizeof(RP_ATSAM_REG_DATATYPE));
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atsam_write_single_register(unsigned int reg_no,
                                          uint8_t *buf,
                                          size_t write_size)
{
    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_write_single_register(%d, 0x%X)",
                        atsam_target.name,
                        reg_no,
                        ((RP_ATSAM_REG_DATATYPE *) buf)[0]);

    assert(atsam_status.is_open);

    assert(buf != NULL);
    assert(write_size == sizeof(RP_ATSAM_REG_DATATYPE));

    if (reg_no < 0  ||  reg_no > RP_ATSAM_NUM_XREGS)
        return RP_VAL_TARGETRET_ERR;

    /* Accessing for status register */
    if(reg_no == 25)
    {
        reg_no = XREG_APSR;
    }

    // Most convenient to use writeMemory() instead of pokeReg() here
    // because it copies data directly from buffer.
    write_size = mC->writeMemory(reg_no * sizeof(RP_ATSAM_REG_DATATYPE),
                                  sizeof(RP_ATSAM_REG_DATATYPE),
                                  buf, Seg_Reg);

    assert (write_size == sizeof(RP_ATSAM_REG_DATATYPE));  // @@@FIXME

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atsam_read_mem(uint64_t addr,
                             uint8_t *buf,
                             size_t req_size,
                             size_t *actual_size)
{
    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_read_mem(0x%llX, ptr, %d, ptr)",
                        atsam_target.name,
                        addr,
                        req_size);

    assert(atsam_status.is_open);

    assert(buf != NULL);
    assert(req_size > 0);
    assert(actual_size != NULL);

    if (addr > RP_ATSAM_MAX_ADDRESS)
    {
        atsam_status.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: bad address 0x%llx",
                            atsam_target.name,
                            addr);

        return RP_VAL_TARGETRET_ERR;
    }

    if (addr + req_size > RP_ATSAM_MAX_ADDRESS + 1)
        *actual_size = RP_ATSAM_MAX_ADDRESS + 1 - addr;
    else
        *actual_size = req_size;

    if(!(mC->readMemory(addr, *actual_size, buf, Seg_Data)))
    {
        atsam_status.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: Read mem address 0x%llx size 0x%llx failed!",
                            atsam_target.name, addr, *actual_size);
        return RP_VAL_TARGETRET_ERR;
    }
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atsam_write_mem(uint64_t addr,
                              uint8_t *buf,
                              size_t write_size)
{
    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_write_mem(0x%llX, ptr, %d)",
                        atsam_target.name,
                        addr,
                        write_size);

    assert(atsam_status.is_open);
    assert(buf != NULL);

    /* GDB does zero length writes for some reason. Treat them harmlessly. */
    if (write_size == 0)
        return RP_VAL_TARGETRET_OK;

    if (addr > RP_ATSAM_MAX_ADDRESS)
    {
        atsam_status.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: bad address 0x%llx",
                            atsam_target.name,
                            addr);
        return RP_VAL_TARGETRET_ERR;
    }

    if ((addr + write_size - 1) > RP_ATSAM_MAX_ADDRESS)
    {
        atsam_status.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: bad address/write_size 0x%llx/0x%x",
                            atsam_target.name,
                            addr,
                            write_size);
        return RP_VAL_TARGETRET_ERR;
    }

    if(!(write_size = mC->writeMemory(addr, write_size, buf, Seg_Data)))
    {
        atsam_status.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: Write mem address 0x%llx size 0x%llx failed!",
                            atsam_target.name, addr, write_size);
        return RP_VAL_TARGETRET_ERR;
    }

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atsam_resume_from_current(int step, int sig)
{
    Breakpoint* bp;
    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_resume_from_current(%s, %d)",
                        atsam_target.name,
                        (step)  ?  "step"  :  "run",
                        sig);

    assert(atsam_status.is_open);

    if (step)
    {
        /* Single step the target */
        atsam_status.state = RP_ATSAM_TARGET_STATE_SINGLE_STEP_COMPLETE;
        bp = mC->step();
    }
    else
    {
        /* Run the target to a breakpoint, or until we stop it. */
        atsam_status.state = RP_ATSAM_TARGET_STATE_RUN_TO_ADDR_COMPLETE;
        bp = mC->run();
    }

    if(mBp)
        free(mBp);

    atsam_status.target_running = FALSE;
    if(bp)
    {
        atsam_status.target_interrupted = TRUE;
        atsam_status.state = RP_ATSAM_TARGET_STATE_BREAKPOINT_HIT;
        /* Copy breakpoint to support extended breakpoint info */
        mBp = (Breakpoint*)malloc(sizeof(Breakpoint));
        *mBp = *bp;
    }
    else
    {
        mBp = NULL;
        atsam_status.target_interrupted = FALSE;
    }
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atsam_resume_from_addr(int step, int sig, uint64_t addr)
{
    Breakpoint* bp;
    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_resume_from_addr(%s, %d, 0x%llX)",
                        atsam_target.name,
                        (step)  ?  "step"  :  "run",
                        sig,
                        addr);

    assert(atsam_status.is_open);

    atsam_status.registers[RP_ATSAM_REGNUM_PC] = addr;

    /* Update the PC register in the target */
    mC->pokeReg(R_PC, addr);

    /* Run the target from the new PC address. */
    if (step)
    {
        /* Single step the target */
        atsam_status.state = RP_ATSAM_TARGET_STATE_SINGLE_STEP_COMPLETE;
        bp = mC->step();
    }
    else
    {
        /* Run the target to a breakpoint, or until we stop it. */
        atsam_status.state = RP_ATSAM_TARGET_STATE_RUN_TO_ADDR_COMPLETE;
        bp = mC->run();
    }

    if(mBp)
        free(mBp);

    atsam_status.target_running = FALSE;
    if(bp)
    {
        atsam_status.target_interrupted = TRUE;
        atsam_status.state = RP_ATSAM_TARGET_STATE_BREAKPOINT_HIT;
        /* Copy breakpoint to support extended breakpoint info */
        mBp = (Breakpoint*)malloc(sizeof(Breakpoint));
        *mBp = *bp;
    }
    else
    {
        mBp = NULL;
        atsam_status.target_interrupted = FALSE;
    }
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atsam_go_waiting(int sig)
{
    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_go_waiting()",
                        atsam_target.name);
    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int atsam_wait_partial(int first,
                                 char *status_string,
                                 size_t status_string_len,
                                 out_func of,
                                 int *implemented,
                                 int *more,
                                 rp_target* t)
{
    int state;
    char *cp;
    int sig;

    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_wait_partial()",
                        atsam_target.name);

    assert(atsam_status.is_open);

    assert(status_string != NULL);
    assert(status_string_len >= 34);
    assert(of != NULL);
    assert(implemented != NULL);
    assert(more != NULL);

    *implemented = TRUE;

//    if (!atsam_status.target_running)
//        return RP_VAL_TARGETRET_NOSUPP;
    atsam_status.target_running = FALSE;

//#ifdef WIN32
//    sleep((first)  ?  500  :  100);
//#else
//    usleep((first)  ?  500000  :  100000);
//#endif
    /* TODO: Test the target state (i.e. running/stopped) without blocking */
    /* If the target only supports a blocking form of test return no support,
       and the blocking version of this test will be called instead. That is
       not so nice, as the system is less interactive using a blocking test. */
    state = atsam_status.state;

    if (state == RP_ATSAM_TARGET_STATE_RUNNING)
    {
        *more = TRUE;
        return RP_VAL_TARGETRET_OK;
    }

    switch (state)
    {
    case RP_ATSAM_TARGET_STATE_STOPPED:
        if (atsam_status.target_interrupted)
            sig = RP_SIGNAL_INTERRUPT;
        else
            sig = RP_SIGNAL_ABORTED;
        break;
    case RP_ATSAM_TARGET_STATE_RUNNING:
        *more = TRUE;
        return RP_VAL_TARGETRET_OK;
    case RP_ATSAM_TARGET_STATE_SINGLE_STEP_COMPLETE:
        sig = RP_SIGNAL_TRAP;
        break;
    case RP_ATSAM_TARGET_STATE_BREAKPOINT_HIT:
        sig = RP_SIGNAL_TRAP;
        break;
    default:
        atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: unexpected state %d for the atsam",
                            atsam_target.name,
                            state);
        sig = RP_SIGNAL_ABORTED;
        break;
    }
    /* Fill out the status string */
    sprintf(status_string, "T%02d", sig);

    //if (!refresh_registers())
    //    return RP_VAL_TARGETRET_ERR;
    
    cp = atsam_out_treg(&status_string[3], RP_ATSAM_REGNUM_PC);
    cp = atsam_out_treg(cp, RP_ATSAM_REGNUM_FP);

    *more = FALSE;

    return (cp != NULL)  ?  RP_VAL_TARGETRET_OK  :  RP_VAL_TARGETRET_ERR;
}

/* Target method */
static int atsam_wait(char *status_string,
                         size_t status_string_len,
                         out_func of,
                         int *implemented,
                         rp_target* t)
{
    int state;
    char *cp;
    int sig;

    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_wait()",
                        atsam_target.name);

    assert(atsam_status.is_open);

    assert(status_string != NULL);
    assert(status_string_len >= 34);
    assert(of != NULL);
    assert(implemented != NULL);

    *implemented = TRUE;

//    if (!atsam_status.target_running)
//        return RP_VAL_TARGETRET_NOSUPP;
    atsam_status.target_running = FALSE;

    /* TODO: Wait for the target to stop */
    state = atsam_status.state;

    switch (state)
    {
    case RP_ATSAM_TARGET_STATE_STOPPED:
        sig = RP_SIGNAL_ABORTED;
        break;
    case RP_ATSAM_TARGET_STATE_SINGLE_STEP_COMPLETE:
        sig = RP_SIGNAL_TRAP;
        break;
    case RP_ATSAM_TARGET_STATE_BREAKPOINT_HIT:
        sig = RP_SIGNAL_TRAP;
        break;
    default:
        atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: unexpected state %d for the atsam",
                            atsam_target.name,
                            state);
        sig = RP_SIGNAL_ABORTED;
        break;
    }
    /* Fill out the status string */
    sprintf(status_string, "T%02d", sig);

    //    if (!refresh_registers())
    //    return RP_VAL_TARGETRET_ERR;
    
    cp = atsam_out_treg(&status_string[3], RP_ATSAM_REGNUM_PC);
    cp = atsam_out_treg(cp, RP_ATSAM_REGNUM_FP);

    return (cp != NULL)  ?  RP_VAL_TARGETRET_OK  :  RP_VAL_TARGETRET_ERR;
}

/* Target method */
static int atsam_process_query(unsigned int *mask,
                                  rp_thread_ref *arg,
                                  rp_thread_info *info)
{
    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_process_query()",
                        atsam_target.name);
    /* TODO: Does your target support threads? Is so, implement this function.
       Otherwise just return no support. */
    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int atsam_list_query(int first,
                               rp_thread_ref *arg,
                               rp_thread_ref *result,
                               size_t max_num,
                               size_t *num,
                               int *done)
{
    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_list_query()",
                        atsam_target.name);
    /* TODO: Does your target support threads? Is so, implement this function.
       Otherwise just return no support. */
    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int atsam_current_thread_query(rp_thread_ref *thread)
{
    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_current_thread_query()",
                        atsam_target.name);
    /* TODO: Does your target support threads? Is so, implement this function.
       Otherwise just return no support. */
    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int atsam_offsets_query(uint64_t *text, uint64_t *data, uint64_t *bss)
{
    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_offsets_query()",
                        atsam_target.name);

    assert(atsam_status.is_open);

    assert(text != NULL);
    assert(data != NULL);
    assert(bss != NULL);

    /* TODO: Is this what *your* target really needs? */
    *text = 0;
    *data = 0;
    *bss = 0;
    return RP_VAL_TARGETRET_NOSUPP;
//    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atsam_crc_query(uint64_t addr, size_t len, uint32_t *val)
{
    uint8_t buf[1];

    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_crc_query()",
                        atsam_target.name);

    assert(atsam_status.is_open);

    if (addr > RP_ATSAM_MAX_ADDRESS  ||  addr + len > RP_ATSAM_MAX_ADDRESS + 1)
    {
        atsam_status.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: bad address 0x%llx",
                            atsam_target.name,
                            addr);

        return RP_VAL_TARGETRET_ERR;
    }

    /* TODO: Read the target memory,and use the crc32 routine to calculate
       the CRC value to be returned. */
    /* Note: The CRC can be calculated in chunks. The first call to crc32
       should set the current CRC value to all 1's, as this is the priming
       value for CRC32. Subsequent calls should set the current CRC to the
       value returned by the previous call, until all the data has been
       processed. */

    *val = crc32(buf, sizeof(buf), 0xFFFFFFFF);

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atsam_raw_query(char *in_buf, char *out_buf, size_t out_buf_size)
{
    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_raw_query() %s",
                        atsam_target.name, in_buf);

    return RP_VAL_TARGETRET_NOSUPP;
}

/* command: erase flash */
static int atsam_rcmd_erase(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    char buf[1000 + 1];

    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_rcmd_erase()",
                        atsam_target.name);
    rp_encode_string("Erasing target flash - ", buf, size_t(1000));
    of(t, buf);

    /* TODO: perform the erase. */

    rp_encode_string(" Erased OK\n", buf, size_t(1000));
    of(t, buf);
    return RP_VAL_TARGETRET_OK;
}

/* command: Specify the model to use */
static int atsam_rcmd_setlib(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    char buf[1000 + 1];
    char reply_buf[256];

    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_rcmd_setlib()",
                        atsam_target.name);
    if(!argv[1])
    {
        rp_encode_string("No new lib file specified.\n", buf, size_t(1000));
        of(t, buf);
        return RP_VAL_TARGETRET_OK;
    }
    if(atsam_status.lib)
    {
        if(strcmp(atsam_status.lib, argv[1]) == 0)
        {
            sprintf(reply_buf, "%s already selected!\n", atsam_status.lib);
            rp_encode_string(reply_buf, buf, size_t(1000));
            of(t, buf);
            return RP_VAL_TARGETRET_OK;
        }
        sprintf(reply_buf, "Current lib file: %s\n", atsam_status.lib);
        rp_encode_string(reply_buf, buf, size_t(1000));
        of(t, buf);
        free(atsam_status.lib);
    }

    atsam_status.lib = strcpy((char*)malloc(strlen(argv[1]) + 1), (argv[1]));
    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: argv=%s",
                        atsam_target.name, argv[1]);

    sprintf(reply_buf, "New lib file: %s\n", atsam_status.lib);
    rp_encode_string(reply_buf, buf, size_t(1000));
    of(t, buf);

    /* Clear selected device */
    if(atsam_status.device)
        free(atsam_status.device);

    /* Check if second argument is present, and it starts with 'A' or 'a' */
    if(argv[2] && ((*argv[2] & 0xdf) == 'A'))
    {
        atsam_status.device = strcpy((char*)malloc(strlen(argv[2]) + 1), (argv[2]));
        atsam_load_model(of, t);
    }
    else
        atsam_status.device = NULL;

    return RP_VAL_TARGETRET_OK;
}

/* command: Print the selected model */
static int atsam_rcmd_getlib(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    char buf[1000 + 1];
    char reply_buf[256];

    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_rcmd_getlib()",
                        atsam_target.name);

    if(atsam_status.lib)
    {
        sprintf(reply_buf, "%s\n", atsam_status.lib);
        rp_encode_string(reply_buf, buf, size_t(1000));
        of(t, buf);
    }
    else
    {
        rp_encode_string("No current lib file.\n", buf, size_t(1000));
        of(t, buf);
    }
    return RP_VAL_TARGETRET_OK;
}

/* command: Specify the device to use */
static int atsam_rcmd_setdevice(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    char buf[1000 + 1];
    char reply_buf[256];

    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_rcmd_setdevice()",
                        atsam_target.name);
    if(!argv[1])
    {
        rp_encode_string("No new device specified.\n", buf, size_t(1000));
        of(t, buf);
        return RP_VAL_TARGETRET_OK;
    }
    if(atsam_status.device)
    {
        if(strcmp(atsam_status.device, argv[1]) == 0)
        {
            sprintf(reply_buf, "%s already selected!\n", atsam_status.device);
            rp_encode_string(reply_buf, buf, size_t(1000));
            of(t, buf);
            return RP_VAL_TARGETRET_OK;
        }
    }

    if(atsam_status.device)
        free(atsam_status.device);
    atsam_status.device = strcpy((char*)malloc(strlen(argv[1]) + 1), (argv[1]));

    atsam_load_model(of, t);

    return RP_VAL_TARGETRET_OK;
}

/* command: Print the selected device */
static int atsam_rcmd_getdevice(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    char buf[1000 + 1];
    char reply_buf[256];

    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_rcmd_setdevice()",
                        atsam_target.name);
    if(atsam_status.device)
    {
        sprintf(reply_buf, "%s\n", atsam_status.device);
        rp_encode_string(reply_buf, buf, size_t(1000));
        of(t, buf);
    }
    else
    {
        rp_encode_string("No current device.\n", buf, size_t(1000));
        of(t, buf);
    }
    return RP_VAL_TARGETRET_OK;
}

/* command: erase flash */
static int atsam_rcmd_reset(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    char buf[1000 + 1];
    char reply_buf[256];
    ResetType rst;

    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_rcmd_reset()",
                        atsam_target.name);

    /* Default to POR reset */
    rst = Reset_por;
    sprintf(reply_buf, "Performing Power-on reset - ", atsam_status.device);

    if(argc > 1)
    {
        if(strcmp("por", argv[1]) == 0);
        else if(strcmp("bod", argv[1]) == 0)
        {
            sprintf(reply_buf, "Performing Brown-out reset - ", atsam_status.device);
            rst = Reset_bod;
        }
        else if(strcmp("ext", argv[1]) == 0)
        {
            sprintf(reply_buf, "Performing External reset - ", atsam_status.device);
            rst = Reset_ext;
        }
        else if(strcmp("spike", argv[1]) == 0)
        {
            sprintf(reply_buf, "Performing Spike reset - ", atsam_status.device);
            rst = Reset_spike;
        }
        else
        {
            rp_encode_string("Reset type not recognized. Reset not performed.\n", buf, size_t(1000));
            of(t, buf);
            return RP_VAL_TARGETRET_OK;
        }
    }

    rp_encode_string(reply_buf, buf, size_t(1000));
    of(t, buf);

    /* Perform the reset. */
    mC->reset(rst);

    rp_encode_string(" Reset OK\n", buf, size_t(1000));
    of(t, buf);
    return RP_VAL_TARGETRET_OK;
}

/* command: Get Cycle Counter */
static int atsam_rcmd_getcyclecnt(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    uint64_t counter;
    char buf[1000 + 1];
    char reply_buf[256];

    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_rcmd_getcyclecnt()",
                        atsam_target.name);
    mC->peekReg(R_CYCLECOUNT, &counter);
    sprintf(reply_buf, "Cycle counter - %d\n", counter);
    rp_encode_string(reply_buf, buf, size_t(1000));
    of(t, buf);

    return RP_VAL_TARGETRET_OK;
}

/* command: Get Lifetime Counter */
static int atsam_rcmd_getlifecnt(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    uint64_t counter;
    char buf[1000 + 1];
    char reply_buf[256];

    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_rcmd_getlifecnt()",
                        atsam_target.name);
    mC->peekReg(R_LIFETIMECOUNT, &counter);
    sprintf(reply_buf, "Lifetime counter - %d\n", counter);
    rp_encode_string(reply_buf, buf, size_t(1000));
    of(t, buf);

    return RP_VAL_TARGETRET_OK;
}

/* command: debug - Extended debug features for models built with debug option */
static int atsam_rcmd_debug(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    char buf[1000 + 1] = {""};

    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_rcmd_debug()",
                        atsam_target.name);

    int i = 1;
    while(i < argc)
    {
        atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: atsam_rcmd_debug(): argv[%d]=%s",
                            atsam_target.name, i, argv[i]);
        if(i > 1)
            strcat(buf, " ");
        strcat(buf, argv[i]);
        i++;
    }
    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_rcmd_debug(): buf=%s",
                        atsam_target.name, buf);
    mM->debug(buf);

    return RP_VAL_TARGETRET_OK;
}

/* command: info - Extended target information */
static int atsam_rcmd_info(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    char buf[1000 + 1];
    char reply_buf[500 + 1];

    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_rcmd_info()",
                        atsam_target.name);

    if(argc == 1)
    {
        /* No arguments. Print help text. */
    }
    int i = 1;
    while(i < argc)
    {
        atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: atsam_rcmd_debug(): argv[%d]=%s",
                            atsam_target.name, i, argv[i]);
        if(i > 1)
            strcat(buf, " ");
        strcat(buf, argv[i]);
        i++;
    }
    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_rcmd_debug(): buf=%s",
                        atsam_target.name, buf);

    return RP_VAL_TARGETRET_OK;
}

/* command: lastbreak - Extended information about the last breakpoint hit */
static int atsam_rcmd_lastbreak(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    char buf[1000 + 1];
    char reply_buf[500 + 1];
    int index;

    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_rcmd_lastbreak()",
                        atsam_target.name);
    if(mBp)
    {
        index = sprintf(reply_buf, "Stopped at ");
        strcat(reply_buf, "");
        if (mBp->m_type & (BP_Exec | BP_Break))
        {
            index += sprintf((reply_buf+index),"breakpoint at 0x%06x\n", mBp->m_pc);
        }
        else
        {
            index += sprintf((reply_buf+index), "watchpoint at 0x%06x: ", mBp->m_pc);
            switch (mBp->m_segment)
            {
              case Seg_Code:
                index += sprintf((reply_buf+index), "FLASH %s at address 0x%04x\n",
                        bptypes[mBp->m_actacc], mBp->m_actaddr);
                break;
              case Seg_Data:
                index += sprintf((reply_buf+index), "SRAM %s at address 0x%04x\n",
                        bptypes[mBp->m_actacc], mBp->m_actaddr);
                break;
              case Seg_Eeprom:
                index += sprintf((reply_buf+index), "EEPROM %s at address 0x%04x\n",
                        bptypes[mBp->m_actacc], mBp->m_actaddr);
                break;
              case Seg_IO:
                {
//                    const char *sym = symtab.lookup(m->mem2ioAddr(mBp->m_actaddr));
//                    if (sym)
//                        index += sprintf((reply_buf+index), "I/O %s at %s (0x%04x)\n",
//                               bptypes[mBp->m_actacc], sym, mBp->m_actaddr);
//                    else
                        index += sprintf((reply_buf+index), "I/O %s at address 0x%04x\n",
                               bptypes[mBp->m_actacc], mBp->m_actaddr);
                }
                break;
            case Seg_Reg:
                index += sprintf((reply_buf+index), "R%d %s\n", mBp->m_actaddr, bptypes[mBp->m_actacc]);
                break;
            default:
                index += sprintf((reply_buf+index), "Unknown %s at address 0x%04x\n",
                        bptypes[mBp->m_actacc], mBp->m_actaddr);
                break;
            }
        }
        rp_encode_string(reply_buf, buf, size_t(1000));
    }
    else
    {
        rp_encode_string("No breakpoint recorded.\n", buf, size_t(1000));
    }
    of(t, buf);
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atsam_add_break(int type, uint64_t addr, unsigned int len)
{
    int retVal;
    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_add_break(%d, 0x%llx, %d)",
                        atsam_target.name,
                        type,
                        addr,
                        len);
    /* TODO: Handle whichever types of breakpoint the target can support, and
       report no support for the others. */
    if(type == 0 || type == 1)
    {
        Breakpoint b(addr);
        retVal = mC->addBreakpoint(&b);
    }
    else if(type == 2)
    {
        Breakpoint b(WP_Write, Seg_Data, addr, len);
        retVal = mC->addBreakpoint(&b);
    }
    else if(type == 3)
    {
        Breakpoint b(WP_Read, Seg_Data, addr, len);
        retVal = mC->addBreakpoint(&b);
    }
    else if(type == 4)
    {
        Breakpoint b(WP_Access, Seg_Data, addr, len);
        retVal = mC->addBreakpoint(&b);
    }

    if(retVal > 0)
        return RP_VAL_TARGETRET_OK;
    else if(retVal == 0)
        return RP_VAL_TARGETRET_NOSUPP;
    else
        return RP_VAL_TARGETRET_ERR;
}

/* Target method */
static int atsam_remove_break(int type, uint64_t addr, unsigned int len)
{
    int retVal;
    BPtype t;
    Segment s;
    int id = -1;
    uint64_t l = len;

    atsam_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_remove_break(%d, 0x%llx, %d)",
                        atsam_target.name,
                        type,
                        addr,
                        len);
    /* TODO: Handle whichever types of breakpoint the target can support, and
       report no support for the others. */
    if(type == 0 || type == 1)
    {
        t = BP_Exec;
        s = Seg_Code;
    }
    else if(type == 2)
    {
        t = WP_Write;
        s = Seg_Data;
    }
    else if(type == 3)
    {
        t = WP_Read;
        s = Seg_Data;
    }
    else if(type == 4)
    {
        t = WP_Access;
        s = Seg_Data;
    }

    Breakpoint **bpp = mC->getBreakpoints();
    for (; *bpp; bpp++)
    {
        Breakpoint *bp = *bpp;
        if (bp->m_address == addr &&
                bp->m_type == t &&
                bp->m_size == l &&
                bp->m_segment == s)
        {
            id = bp->m_handle;
            break;
        }
    }
    if(id == -1)
        return RP_VAL_TARGETRET_ERR;

    retVal = mC->removeBreakpoint(id);

    if(retVal > 0)
        return RP_VAL_TARGETRET_OK;
    else if(retVal == 0)
        return RP_VAL_TARGETRET_NOSUPP;
    else
        return RP_VAL_TARGETRET_ERR;
}

/* Output registers in the format suitable
   for TAAn:r...;n:r...;  format */
static char *atsam_out_treg(char *in, unsigned int reg_no)
{
    static const char hex[] = "0123456789abcdef";
    uint8_t data_buf[sizeof(RP_ATSAM_REG_DATATYPE)];
    uint8_t avail_buf[sizeof(RP_ATSAM_REG_DATATYPE)];
    size_t read_size;

    if (in == NULL)
        return NULL;

    assert(reg_no < RP_ATSAM_NUM_REGS);

    *in++ = hex[(reg_no >> 4) & 0x0f];
    *in++ = hex[reg_no & 0x0f];
    *in++ = ':';

    atsam_read_single_register(reg_no, data_buf, avail_buf, sizeof(data_buf), &read_size);

    assert (read_size == sizeof(RP_ATSAM_REG_DATATYPE));  // @@@FIXME
    /* The register goes into the buffer in little-endian order */
    *in++ = hex[(data_buf[0] >> 4) & 0x0f];
    *in++ = hex[data_buf[0] & 0x0f];
    *in++ = hex[(data_buf[1] >> 4) & 0x0f];
    *in++ = hex[data_buf[1] & 0x0f];
    *in++ = hex[(data_buf[2] >> 4) & 0x0f];
    *in++ = hex[data_buf[2]  & 0x0f];
    *in++ = hex[(data_buf[3] >> 4) & 0x0f];
    *in++ = hex[data_buf[3] & 0x0f];
    *in++ = ';';
    *in   = '\0';

    return in;
}

/* Table used by the crc32 function to calcuate the checksum. */
static uint32_t crc32_table[256] =
{
    0,
    0
};

static uint32_t crc32(uint8_t *buf, size_t len, uint32_t crc)
{
    if (!crc32_table[1])
    {
        /* Initialize the CRC table and the decoding table. */
        int i;
        int j;
        unsigned int c;

        for (i = 0; i < 256; i++)
	{
	    for (c = i << 24, j = 8; j > 0; --j)
	        c = c & 0x80000000 ? (c << 1) ^ 0x04c11db7 : (c << 1);
	    crc32_table[i] = c;
	}
    }

    while (len--)
    {
        crc = (crc << 8) ^ crc32_table[((crc >> 24) ^ *buf) & 255];
        buf++;
    }
    return crc;
}

// static int refresh_registers(void)
// {
//     unsigned i;

//     /* TODO: Grab the real register values from the target */
//     for (i = 0;  i < RP_ATSAM_NUM_REGS;  i++)
//     {
// //        atsam_status.registers[i] = mM->peekMemoryWord32(i, Seg_Reg);
//         atsam_status.registers[i] = mM->peekReg(i);
//     }
//     return  TRUE;
// }

int atsam_load_model(out_func of, rp_target* t)
{
    char buf[1000 + 1];
    char reply_buf[256];

    if(atsam_status.device)
        sprintf(reply_buf, "Loading %s... ", atsam_status.device);
    else
        sprintf(reply_buf, "Loading... ");
    rp_encode_string(reply_buf, buf, size_t(1000));
    of(t, buf);

    mM = mMgr->load(atsam_status.lib, atsam_status.device);

    if(mM)
    {
        mC = mM->getCore(atsam_status.selectedCore);
        mC->addStepCallback(stepcallback, NULL);
        if(atsam_status.device)
        {
            char name_buf[256];
            if(mM->getStringProperty(P_DeviceName, sizeof(name_buf), name_buf))
                atsam_status.device = strcpy((char*)malloc(strlen(name_buf) + 1), (name_buf));
        }
        sprintf(reply_buf, "Successfully loaded %s\n", atsam_status.device);
        rp_encode_string(reply_buf, buf, size_t(1000));
        of(t, buf);
    }
    else
    {
        rp_encode_string("FAILED! No device loaded.\n", buf, size_t(1000));
        of(t, buf);
    }
}
