/* Copyright (C) 2002 Chris Liechti and Steve Underwood
   Copyright (C) 2013 Atmel, Inc.
 
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
 
 
   Implementation of a Atmel AVR32 target for the GDB proxy server.
   
   Exported Data:
     atuc3_target             - target descriptor of the `atuc3' target 
  
   Imported Data:
     None     
  
   Static Data:
     atuc3_XXXX               - static data representing status and 
                                   parameters of the target 
  
   Global Functions:  
     None
  
   Static Functions:  
     atuc3_XXXX              - methods comprising the `atuc3' target.
                                  A description is in file gdbproxy.h
 
     atuc3_                  - local finctions
     atuc3_command
 
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

/* Note: we are using prefix 'atuc3' for static stuff in
   order to simplify debugging of the target code itself */

/* TODO: Put the correct values for the real target in these macros */
#define RP_ATUC3_MIN_ADDRESS             0x0U
#define RP_ATUC3_MAX_ADDRESS             0xFFFFFFFFULL
#define RP_ATUC3_NUM_REGS                16
#define RP_ATUC3_REG_DATATYPE            uint32_t
#define RP_ATUC3_REG_BYTES               (RP_ATUC3_NUM_REGS*sizeof(RP_ATUC3_REG_DATATYPE))
#define RP_ATUC3_REGNUM_PC               15  /* Program counter reg. number */
#define RP_ATUC3_REGNUM_SP               13  /* Stack pointer reg. number */
#define RP_ATUC3_REGNUM_FP               7  /* Frame pointer reg. number */
/* According to EABI, FP is r7 for THUMB and r11 otherwise. */

#define RP_ATUC3_NUM_XREGS               66
/* Redefining register locations according to ../verilator/cm0p/arm_regdefs.vh */
#define XREG_APSR                        21 // @@@@FIXME!

#define RP_ATUC3_MAX_BREAKPOINTS         512 /* This is a simulator, in theory unlimited :-) */

/* Some example states a real target might support. */
#define RP_ATUC3_TARGET_STATE_RUNNING                    0
#define RP_ATUC3_TARGET_STATE_STOPPED                    1
#define RP_ATUC3_TARGET_STATE_SINGLE_STEP_COMPLETE       2
#define RP_ATUC3_TARGET_STATE_RUN_TO_ADDR_COMPLETE       3
#define RP_ATUC3_TARGET_STATE_BREAKPOINT_HIT             4

/*
 * Target methods, static
 */
static void  atuc3_help(const char *prog_name);
static int   atuc3_open(int argc,
                           char * const argv[],
                           const char *prog_name,
                           log_func log_fn);
static void  atuc3_close(void);
static int   atuc3_connect(char *status_string,
                              size_t status_string_size,
                              int *can_restart);
static int   atuc3_disconnect(void);
static void  atuc3_kill(void);
static int   atuc3_restart(void);
static void  atuc3_stop(void);
static int   atuc3_set_gen_thread(rp_thread_ref *thread);
static int   atuc3_set_ctrl_thread(rp_thread_ref *thread);
static int   atuc3_is_thread_alive(rp_thread_ref *thread, int *alive);
static int   atuc3_read_registers(uint8_t *data_buf,
                                     uint8_t *avail_buf,
                                     size_t buf_size,
                                     size_t *read_size);
static int   atuc3_write_registers(uint8_t *data_buf, size_t write_size);
static int   atuc3_read_single_register(unsigned int reg_no,
                                           uint8_t *data_buf,
                                           uint8_t *avail_buf,
                                           size_t buf_size,
                                           size_t *read_size);
static int   atuc3_write_single_register(unsigned int reg_no,
                                            uint8_t *data_buf,
                                            size_t write_size);
static int   atuc3_read_mem(uint64_t addr,
                               uint8_t *data_buf,
                               size_t req_size,
                               size_t *actual_size);
static int   atuc3_write_mem(uint64_t addr,
                                uint8_t *data_buf,
                                size_t req_sise);
static int   atuc3_resume_from_current(int step, int sig);
static int   atuc3_resume_from_addr(int step,
                                       int sig,
                                       uint64_t addr);
static int   atuc3_go_waiting(int sig);
static int   atuc3_wait_partial(int first,
                                   char *status_string,
                                   size_t status_string_len,
                                   out_func out,
                                   int *implemented,
                                   int *more,
                                   rp_target* t);
static int   atuc3_wait(char *status_string,
                           size_t status_string_len,
                           out_func out,
                           int *implemented,
                           rp_target* t);
static int   atuc3_process_query(unsigned int *mask,
                                    rp_thread_ref *arg,
                                    rp_thread_info *info);
static int   atuc3_list_query(int first,
                                 rp_thread_ref *arg,
                                 rp_thread_ref *result,
                                 size_t max_num,
                                 size_t *num,
                                 int *done);
static int   atuc3_current_thread_query(rp_thread_ref *thread);
static int   atuc3_offsets_query(uint64_t *text,
                                    uint64_t *data,
                                    uint64_t *bss);
static int   atuc3_crc_query(uint64_t addr,
                                size_t len,
                                uint32_t *val);
static int   atuc3_raw_query(char *in_buf,
                                char *out_buf,
                                size_t out_buf_size);
static int   atuc3_add_break(int type, uint64_t addr, unsigned int len);
static int   atuc3_remove_break(int type, uint64_t addr, unsigned int len);

static uint32_t crc32(uint8_t *buf, size_t len, uint32_t crc);

#define RCMD(name, hlp) {#name, atuc3_rcmd_##name, hlp}  //table entry generation

/* Prototyping of remote commands */
static int atuc3_rcmd_erase(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atuc3_rcmd_setlib(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atuc3_rcmd_getlib(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atuc3_rcmd_setdevice(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atuc3_rcmd_getdevice(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atuc3_rcmd_reset(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atuc3_rcmd_getcyclecnt(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atuc3_rcmd_getlifecnt(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atuc3_rcmd_debug(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atuc3_rcmd_info(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atuc3_rcmd_lastbreak(int argc, char *argv[], out_func of, data_func df, rp_target* t);

/* Table of remote commands */
static const RCMD_TABLE atuc3_remote_commands[] =
{
    RCMD(erase,         "Erase target flash memory"),
    RCMD(setlib,        "Sets the name of the library file to load. \"setlib lib [device]\""),
    RCMD(getlib,        "Printss the name of the selected library file"),
    RCMD(setdevice,     "Sets the name of the device to load"),
    RCMD(getdevice,     "Prints the name of the selected device"),
    RCMD(reset,         "Reset target"),
    RCMD(getcyclecnt,   "Get the cycle counter"),
    RCMD(getlifecnt,    "Get the life time counter"),
    RCMD(debug,         "Command for extended debug features. Requires model built for debug."),
    RCMD(info,          "Extended target information"),
    RCMD(lastbreak,     "Extended information about the last breakpoint that was hit."),
    {0,0,0}     //sentinel, end of table marker
};

/* Target XML description */
const char atuc3_target_xml[] = "<?xml version=\"1.0\"?>\n<!-- Copyright (C) 2008 Free Software Foundation, Inc.\n\n     Copying and distribution of this file, with or without modification,\n     are permitted in any medium without royalty provided the copyright\n     notice and this notice are preserved.  -->\n\n<!DOCTYPE feature SYSTEM \"gdb-target.dtd\">\n<target>\n  <architecture>arm</architecture>\n<feature name=\"org.gnu.gdb.arm.m-profile\">\n    <reg name=\"r0\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"r1\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"r2\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"r3\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"r4\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"r5\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"r6\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"r7\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"r8\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"r9\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"r10\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"r11\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"r12\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"sp\" bitsize=\"32\" type=\"data_ptr\" group=\"general\"/>\n    <reg name=\"lr\" bitsize=\"32\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"pc\" bitsize=\"32\" type=\"code_ptr\" group=\"general\"/>\n    <reg name=\"xpsr\" bitsize=\"32\" regnum=\"25\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"MSP\" bitsize=\"32\" regnum=\"26\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"PSP\" bitsize=\"32\" regnum=\"27\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"PRIMASK\" bitsize=\"32\" regnum=\"28\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"BASEPRI\" bitsize=\"32\" regnum=\"29\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"FAULTMASK\" bitsize=\"32\" regnum=\"30\" type=\"uint32\" group=\"general\"/>\n    <reg name=\"CONTROL\" bitsize=\"32\" regnum=\"31\" type=\"uint32\" group=\"general\"/>\n  </feature>\n</target>\n\n";

// String arrays, must be kept in sync with model.h/breakpoint.h enums.
static const char *bptypes[] =
{ "none", "exec", "write", "N/A=3", "read", "N/A=5", "access", "N/A=7",
  "modify"};
static const char *segtypes[] =
{ "code", "data", "eeprom", "reg", "i/o", "sysreg", "none" };

/*
 * Global target descriptor 
 */
rp_target atuc3_target =
{
    NULL,      /* next */
    -1,
    -1,
    0,
    0,
    0,
    0,
    "atuc3",
    "atuc3 target to use GDB proxy server with Atmel AVR32 simulator models",
    atuc3_remote_commands,
    atuc3_help,
    atuc3_open,
    atuc3_close,
    atuc3_connect,
    atuc3_disconnect,
    atuc3_kill,
    atuc3_restart,
    atuc3_stop,
    atuc3_set_gen_thread,
    atuc3_set_ctrl_thread,
    atuc3_is_thread_alive,
    atuc3_read_registers,
    atuc3_write_registers,
    atuc3_read_single_register,
    atuc3_write_single_register,
    atuc3_read_mem,
    atuc3_write_mem,
    atuc3_resume_from_current,
    atuc3_resume_from_addr,
    atuc3_go_waiting,
    atuc3_wait_partial,
    atuc3_wait,
    atuc3_process_query,
    atuc3_list_query,
    atuc3_current_thread_query,
    atuc3_offsets_query,
    atuc3_crc_query,
    atuc3_raw_query,
    atuc3_add_break,
    atuc3_remove_break
};

struct atuc3_status_s
{
    /* Start up parameters, set by atuc3_open */
    log_func    log;
    int         is_open;

    /* Tell wait_xxx method the notion of whether or not
       previously called resume is supported */
    int         target_running;
    int         target_interrupted;
    int         state;
    RP_ATUC3_REG_DATATYPE    registers[RP_ATUC3_NUM_REGS];
    uint64_t                    breakpoints[RP_ATUC3_MAX_BREAKPOINTS];
    bool        deferUnload;
    char       *lib;
    char       *device;
    unsigned    selectedCore;
};

static struct atuc3_status_s atuc3_status =
{
    NULL,
    FALSE,
    FALSE,
    FALSE,
    RP_ATUC3_TARGET_STATE_STOPPED,
    {0},
    {0},
    TRUE,
    '\0',
    '\0',
    0
};

/* Local functions */
static char *atuc3_out_treg(char *in, unsigned int reg_no);
// static int refresh_registers(void); // Not used. Access model directly.
int atuc3_load_model(out_func of, rp_target* t);

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
//jie        atuc3_stop();
}

/* Target method */

uint64_t atuc3_getIntProp(Core *c, int propId, const char *name)
{
    uint64_t  val = 0;
    if (c->getIntProperty(propId, &val, name))
        return val;
    else if (propId == P_Signature)
        return atuc3_getIntProp(c, P_JTAGID, name);
    else
        return 0;
}


uint64_t atuc3_getIntProp(Model *m, int propId, const char *name)
{
    return atuc3_getIntProp(m->getCore(atuc3_status.selectedCore), propId, name);
}


static void atuc3_help(const char *prog_name)
{
    printf("This is the atuc3 target for the GDB proxy server. Usage:\n\n");
    printf("  %s [options] %s [atuc3-options]\n",
           prog_name,
           atuc3_target.name);
    printf("\nOptions:\n\n");
    printf("  --debug              run %s in debug mode\n", prog_name);
    printf("  --help               `%s --help %s'  prints this message\n",
           prog_name,
           atuc3_target.name);
    printf("  --port=PORT          use the specified TCP port\n");
    printf("\natuc3-options:\n\n");
    printf("  --model=MODEL        use the specified .%s file\n", dll);
    printf("  --device=DEVICE      use the specified device\n");

    printf("\n");

    return;
}

/* Target method */
static int atuc3_open(int argc,
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

    assert(!atuc3_status.is_open);
    assert(prog_name != NULL);
    assert(log_fn != NULL);

    /* Set log */
    atuc3_status.log = log_fn;

    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_open()",
                        atuc3_target.name);

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
            atuc3_status.lib = strcpy((char*)malloc(strlen(optarg) + 1), (optarg));
            atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: lib option: %s, lib: %s",
                            atuc3_target.name, optarg, atuc3_status.lib);
            break;
        case 2:
            atuc3_status.device = strcpy((char*)malloc(strlen(optarg) + 1), (optarg));
            atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: device option: %s, device: %s",
                            atuc3_target.name, optarg, atuc3_status.device);
            break;
        case 0:
            /* Long option which just sets a flag */
            break;
        default:
            atuc3_status.log(RP_VAL_LOGLEVEL_NOTICE,
                                "%s: Use `%s --help %s' to see a complete list of options",
                                atuc3_target.name,
                                prog_name,
                                atuc3_target.name);
            return RP_VAL_TARGETRET_ERR;
        }
    }

    if (!atuc3_status.is_open)
    {
        /* TODO: Verify precense of model file, but load() in modelmgr.cpp
         * still fails unless we specify path (i.e. prepend with "./" */
        if(access(atuc3_status.lib, F_OK) == -1)
        {
            atuc3_status.log(RP_VAL_LOGLEVEL_ERR,
                                "%s: Can not find file specified!",
                                atuc3_target.name);
            return RP_VAL_TARGETRET_ERR;
        }
    }

    /* Set up initial default values */
    atuc3_status.target_running = FALSE;
    atuc3_status.target_interrupted = FALSE;
    atuc3_status.state = RP_ATUC3_TARGET_STATE_STOPPED;
    memset (atuc3_status.registers, 0, sizeof(atuc3_status.registers));
    memset (atuc3_status.breakpoints, 0, sizeof(atuc3_status.breakpoints));

    atuc3_status.is_open = TRUE;

    return RP_VAL_TARGETRET_OK;
}


/* Target method */
static void atuc3_close(void)
{
    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_close()",
                        atuc3_target.name);

    assert(atuc3_status.is_open);

    /* TODO: Tidy up things and shut down. */

    atuc3_status.is_open = FALSE;

    free(atuc3_status.lib);
    atuc3_status.lib = NULL;
    free(atuc3_status.device);
    atuc3_status.device = NULL;
    free(mBp);
    mBp = NULL;
    delete mMgr;
}

/* Target method */
static int atuc3_connect(char *status_string,
                            size_t status_string_len,
                            int *can_restart)
{
    char *cp;
    static ModelMgr mgr;

    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_connect()",
                        atuc3_target.name);

    if(!atuc3_status.lib)
    {
        atuc3_status.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: No simulator model specified!",
                            atuc3_target.name);
        return RP_VAL_TARGETRET_ERR;
    }

    assert(atuc3_status.is_open);

    assert(status_string != NULL);
    assert(status_string_len >= 34);
    assert(can_restart != NULL);

    *can_restart = TRUE;

    mgr.setDeferUnload(atuc3_status.deferUnload);
    mM = mgr.load(atuc3_status.lib, atuc3_status.device);
    if(mM)
    {
       atuc3_status.log(RP_VAL_LOGLEVEL_NOTICE,
                           "%s: Simulator model loaded",
                           atuc3_target.name);
       mMgr = &mgr;
       int numcores = atuc3_getIntProp(mM, P_NumCores, 0);
       for (int i = 0; i < numcores; i++)
       {
           mC = mM->getCore(i);
           if (!mC || mC->addStepCallback(stepcallback, NULL) <= 0)
               printf("addStepCallback(core %d) failed."
                       " Ctrl-C WILL NOT WORK\n", i);
       }
       mC = mM->getCore(atuc3_status.selectedCore);

       char name_buf[256];
       if(mM->getStringProperty(P_DeviceName, sizeof(name_buf), name_buf))
            atuc3_status.device = strcpy((char*)malloc(strlen(name_buf) + 1), (name_buf));
    }
    else
    {
        atuc3_status.log(RP_VAL_LOGLEVEL_NOTICE,
                           "%s: Loading of simulator model failed",
                           atuc3_target.name);
        return RP_VAL_TARGETRET_ERR;
    }

    /* Fill out the the status string */
    sprintf(status_string, "T%02d", RP_SIGNAL_ABORTED);

    
    cp = atuc3_out_treg(&status_string[3], RP_ATUC3_REGNUM_PC);
    cp = atuc3_out_treg(cp, RP_ATUC3_REGNUM_FP);

    return (cp != NULL)  ?  RP_VAL_TARGETRET_OK  :  RP_VAL_TARGETRET_ERR;
}

/* Target method */
static int atuc3_disconnect(void)
{
    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_disconnect()",
                        atuc3_target.name);

    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static void atuc3_kill(void)
{
    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_kill()",
                        atuc3_target.name);

    /* Kill the target debug session. */
    atuc3_stop();
}

static int atuc3_restart(void)
{
    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_restart()",
                        atuc3_target.name);

    /* Just stop it. The actual restart will be done
       when connect is called again */
    atuc3_stop();

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static void atuc3_stop(void)
{
    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_stop()",
                        atuc3_target.name);

    assert(atuc3_status.is_open);

    /* Stop (i,e, break) the target program. */
    mC->stop();

    atuc3_status.target_interrupted = TRUE;
    atuc3_status.state = RP_ATUC3_TARGET_STATE_STOPPED;
}

static int atuc3_set_gen_thread(rp_thread_ref *thread)
{
    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_set_gen_thread()",
                        atuc3_target.name);

    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int atuc3_set_ctrl_thread(rp_thread_ref *thread)
{
    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_set_ctrl_thread()",
                        atuc3_target.name);

    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int atuc3_is_thread_alive(rp_thread_ref *thread, int *alive)
{
    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_is_thread_alive()",
                        atuc3_target.name);

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atuc3_read_registers(uint8_t *data_buf,
                                 uint8_t *avail_buf,
                                 size_t buf_size,
                                 size_t *read_size)
{
    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_read_registers()",
                        atuc3_target.name);

    assert(atuc3_status.is_open);

    assert(data_buf != NULL);
    assert(avail_buf != NULL);
    assert(buf_size >= RP_ATUC3_REG_BYTES);
    assert(read_size != NULL);

    *read_size = mC->readMemory(0,
                                 RP_ATUC3_REG_BYTES,
                                 data_buf, Seg_Reg);
    assert (*read_size == RP_ATUC3_REG_BYTES);

    memset(avail_buf, 1, RP_ATUC3_REG_BYTES);
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atuc3_write_registers(uint8_t *data_buf, size_t write_size)
{
    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_write_registers()",
                        atuc3_target.name);

    assert(atuc3_status.is_open);

    assert(data_buf != NULL);
    assert(write_size > 0);
    assert(write_size <= RP_ATUC3_REG_BYTES);

    size_t written = mC->writeMemory(0,
                                 RP_ATUC3_REG_BYTES,
                                 data_buf, Seg_Reg);
    if(write_size != written)
    {
        atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: atuc3_write_registers(%d of %d bytes written)",
                            atuc3_target.name,
                            written, write_size);
    }

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atuc3_read_single_register(unsigned int reg_no,
                                         uint8_t *data_buf,
                                         uint8_t *avail_buf,
                                         size_t buf_size,
                                         size_t *read_size)
{
    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_read_single_register(%d)",
                        atuc3_target.name, reg_no);

    assert(atuc3_status.is_open);

    assert(data_buf != NULL);
    assert(avail_buf != NULL);
    assert(buf_size >= sizeof(RP_ATUC3_REG_DATATYPE));
    assert(read_size != NULL);

    if (reg_no < 0  ||  reg_no > RP_ATUC3_NUM_XREGS)
        return RP_VAL_TARGETRET_ERR;

    /* Asking for status register */
    if(reg_no == 25)
    {
        reg_no = XREG_APSR;
    }

    // Most convenient to use readMemory() instead of peekReg() here
    // because it copies data directly to buffer.
    *read_size = mC->readMemory(reg_no * sizeof(RP_ATUC3_REG_DATATYPE),
                                sizeof(RP_ATUC3_REG_DATATYPE),
                                data_buf, Seg_Reg);
    assert (*read_size == sizeof(RP_ATUC3_REG_DATATYPE));

    memset(avail_buf, 1, sizeof(RP_ATUC3_REG_DATATYPE));
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atuc3_write_single_register(unsigned int reg_no,
                                          uint8_t *data_buf,
                                          size_t write_size)
{
    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_write_single_register(%d, 0x%X)",
                        atuc3_target.name,
                        reg_no,
                        ((RP_ATUC3_REG_DATATYPE *) data_buf)[0]);

    assert(atuc3_status.is_open);

    assert(data_buf != NULL);
    assert(write_size == sizeof(RP_ATUC3_REG_DATATYPE));

    if (reg_no < 0  ||  reg_no > RP_ATUC3_NUM_XREGS)
        return RP_VAL_TARGETRET_ERR;

    /* Accessing for status register */
    if(reg_no == 25)
    {
        reg_no = XREG_APSR;
    }

    // Most convenient to use writeMemory() instead of pokeReg() here
    // because it copies data directly from buffer.
    size_t written = mC->writeMemory(reg_no * sizeof(RP_ATUC3_REG_DATATYPE),
                                  sizeof(RP_ATUC3_REG_DATATYPE),
                                  data_buf, Seg_Reg);

    if(write_size != written)
    {
        atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: atuc3_write_single_register(%d of %d bytes written)",
                            atuc3_target.name,
                            written, write_size);
    }
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atuc3_read_mem(uint64_t addr,
                             uint8_t *data_buf,
                             size_t req_size,
                             size_t *actual_size)
{
    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_read_mem(0x%llX, ptr, %d, ptr)",
                        atuc3_target.name,
                        addr,
                        req_size);

    assert(atuc3_status.is_open);

    assert(data_buf != NULL);
    assert(req_size > 0);
    assert(actual_size != NULL);

    if (addr > RP_ATUC3_MAX_ADDRESS)
    {
        atuc3_status.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: bad address 0x%llx",
                            atuc3_target.name,
                            addr);

        return RP_VAL_TARGETRET_ERR;
    }

    if (addr + req_size > RP_ATUC3_MAX_ADDRESS + 1)
        *actual_size = RP_ATUC3_MAX_ADDRESS + 1 - addr;
    else
        *actual_size = req_size;

    if(!(mC->readMemory(addr, *actual_size, data_buf, Seg_Data)))
    {
        atuc3_status.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: Read mem address 0x%llx size 0x%llx failed!",
                            atuc3_target.name, addr, *actual_size);
        return RP_VAL_TARGETRET_ERR;
    }
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atuc3_write_mem(uint64_t addr,
                              uint8_t *data_buf,
                              size_t write_size)
{
    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_write_mem(0x%llX, ptr, %d)",
                        atuc3_target.name,
                        addr,
                        write_size);

    assert(atuc3_status.is_open);
    assert(data_buf != NULL);

    /* GDB does zero length writes for some reason. Treat them harmlessly. */
    if (write_size == 0)
        return RP_VAL_TARGETRET_OK;

    if (addr > RP_ATUC3_MAX_ADDRESS)
    {
        atuc3_status.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: bad address 0x%llx",
                            atuc3_target.name,
                            addr);
        return RP_VAL_TARGETRET_ERR;
    }

    if ((addr + write_size - 1) > RP_ATUC3_MAX_ADDRESS)
    {
        atuc3_status.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: bad address/write_size 0x%llx/0x%x",
                            atuc3_target.name,
                            addr,
                            write_size);
        return RP_VAL_TARGETRET_ERR;
    }

    if(!(write_size = mC->writeMemory(addr, write_size, data_buf, Seg_Data)))
    {
        atuc3_status.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: Write mem address 0x%llx size 0x%llx failed!",
                            atuc3_target.name, addr, write_size);
        return RP_VAL_TARGETRET_ERR;
    }

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atuc3_resume_from_current(int step, int sig)
{
    Breakpoint* bp;
    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_resume_from_current(%s, %d)",
                        atuc3_target.name,
                        (step)  ?  "step"  :  "run",
                        sig);

    assert(atuc3_status.is_open);

    if (step)
    {
        /* Single step the target */
        atuc3_status.state = RP_ATUC3_TARGET_STATE_SINGLE_STEP_COMPLETE;
        bp = mC->step();
    }
    else
    {
        /* Run the target to a breakpoint, or until we stop it. */
        atuc3_status.state = RP_ATUC3_TARGET_STATE_RUN_TO_ADDR_COMPLETE;
        bp = mC->run();
    }

    if(mBp)
        free(mBp);

    atuc3_status.target_running = FALSE;
    if(bp)
    {
        atuc3_status.target_interrupted = TRUE;
        atuc3_status.state = RP_ATUC3_TARGET_STATE_BREAKPOINT_HIT;
        /* Copy breakpoint to support extended breakpoint info */
        mBp = (Breakpoint*)malloc(sizeof(Breakpoint));
        *mBp = *bp;
    }
    else
    {
        mBp = NULL;
        atuc3_status.target_interrupted = FALSE;
    }
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atuc3_resume_from_addr(int step, int sig, uint64_t addr)
{
    Breakpoint* bp;
    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_resume_from_addr(%s, %d, 0x%llX)",
                        atuc3_target.name,
                        (step)  ?  "step"  :  "run",
                        sig,
                        addr);

    assert(atuc3_status.is_open);

    atuc3_status.registers[RP_ATUC3_REGNUM_PC] = addr;

    /* Update the PC register in the target */
    mC->pokeReg(R_PC, addr);

    /* Run the target from the new PC address. */
    if (step)
    {
        /* Single step the target */
        atuc3_status.state = RP_ATUC3_TARGET_STATE_SINGLE_STEP_COMPLETE;
        bp = mC->step();
    }
    else
    {
        /* Run the target to a breakpoint, or until we stop it. */
        atuc3_status.state = RP_ATUC3_TARGET_STATE_RUN_TO_ADDR_COMPLETE;
        bp = mC->run();
    }

    if(mBp)
        free(mBp);

    atuc3_status.target_running = FALSE;
    if(bp)
    {
        atuc3_status.target_interrupted = TRUE;
        atuc3_status.state = RP_ATUC3_TARGET_STATE_BREAKPOINT_HIT;
        /* Copy breakpoint to support extended breakpoint info */
        mBp = (Breakpoint*)malloc(sizeof(Breakpoint));
        *mBp = *bp;
    }
    else
    {
        mBp = NULL;
        atuc3_status.target_interrupted = FALSE;
    }
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atuc3_go_waiting(int sig)
{
    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_go_waiting()",
                        atuc3_target.name);
    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int atuc3_wait_partial(int first,
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

    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_wait_partial()",
                        atuc3_target.name);

    assert(atuc3_status.is_open);

    assert(status_string != NULL);
    assert(status_string_len >= 34);
    assert(of != NULL);
    assert(implemented != NULL);
    assert(more != NULL);

    *implemented = TRUE;

//    if (!atuc3_status.target_running)
//        return RP_VAL_TARGETRET_NOSUPP;
    atuc3_status.target_running = FALSE;

//#ifdef WIN32
//    sleep((first)  ?  500  :  100);
//#else
//    usleep((first)  ?  500000  :  100000);
//#endif
    /* TODO: Test the target state (i.e. running/stopped) without blocking */
    /* If the target only supports a blocking form of test return no support,
       and the blocking version of this test will be called instead. That is
       not so nice, as the system is less interactive using a blocking test. */
    state = atuc3_status.state;

    if (state == RP_ATUC3_TARGET_STATE_RUNNING)
    {
        *more = TRUE;
        return RP_VAL_TARGETRET_OK;
    }

    switch (state)
    {
    case RP_ATUC3_TARGET_STATE_STOPPED:
        if (atuc3_status.target_interrupted)
            sig = RP_SIGNAL_INTERRUPT;
        else
            sig = RP_SIGNAL_ABORTED;
        break;
    case RP_ATUC3_TARGET_STATE_RUNNING:
        *more = TRUE;
        return RP_VAL_TARGETRET_OK;
    case RP_ATUC3_TARGET_STATE_SINGLE_STEP_COMPLETE:
        sig = RP_SIGNAL_TRAP;
        break;
    case RP_ATUC3_TARGET_STATE_BREAKPOINT_HIT:
        sig = RP_SIGNAL_TRAP;
        break;
    default:
        atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: unexpected state %d for the atuc3",
                            atuc3_target.name,
                            state);
        sig = RP_SIGNAL_ABORTED;
        break;
    }
    /* Fill out the status string */
    sprintf(status_string, "T%02d", sig);

    //if (!refresh_registers())
    //    return RP_VAL_TARGETRET_ERR;
    
    cp = atuc3_out_treg(&status_string[3], RP_ATUC3_REGNUM_PC);
    cp = atuc3_out_treg(cp, RP_ATUC3_REGNUM_FP);

    *more = FALSE;

    return (cp != NULL)  ?  RP_VAL_TARGETRET_OK  :  RP_VAL_TARGETRET_ERR;
}

/* Target method */
static int atuc3_wait(char *status_string,
                         size_t status_string_len,
                         out_func of,
                         int *implemented,
                         rp_target* t)
{
    int state;
    char *cp;
    int sig;

    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_wait()",
                        atuc3_target.name);

    assert(atuc3_status.is_open);

    assert(status_string != NULL);
    assert(status_string_len >= 34);
    assert(of != NULL);
    assert(implemented != NULL);

    *implemented = TRUE;

//    if (!atuc3_status.target_running)
//        return RP_VAL_TARGETRET_NOSUPP;
    atuc3_status.target_running = FALSE;

    /* TODO: Wait for the target to stop */
    state = atuc3_status.state;

    switch (state)
    {
    case RP_ATUC3_TARGET_STATE_STOPPED:
        sig = RP_SIGNAL_ABORTED;
        break;
    case RP_ATUC3_TARGET_STATE_SINGLE_STEP_COMPLETE:
        sig = RP_SIGNAL_TRAP;
        break;
    case RP_ATUC3_TARGET_STATE_BREAKPOINT_HIT:
        sig = RP_SIGNAL_TRAP;
        break;
    default:
        atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: unexpected state %d for the atuc3",
                            atuc3_target.name,
                            state);
        sig = RP_SIGNAL_ABORTED;
        break;
    }
    /* Fill out the status string */
    sprintf(status_string, "T%02d", sig);

    //    if (!refresh_registers())
    //    return RP_VAL_TARGETRET_ERR;
    
    cp = atuc3_out_treg(&status_string[3], RP_ATUC3_REGNUM_PC);
    cp = atuc3_out_treg(cp, RP_ATUC3_REGNUM_FP);

    return (cp != NULL)  ?  RP_VAL_TARGETRET_OK  :  RP_VAL_TARGETRET_ERR;
}

/* Target method */
static int atuc3_process_query(unsigned int *mask,
                                  rp_thread_ref *arg,
                                  rp_thread_info *info)
{
    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_process_query()",
                        atuc3_target.name);
    /* TODO: Does your target support threads? Is so, implement this function.
       Otherwise just return no support. */
    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int atuc3_list_query(int first,
                               rp_thread_ref *arg,
                               rp_thread_ref *result,
                               size_t max_num,
                               size_t *num,
                               int *done)
{
    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_list_query()",
                        atuc3_target.name);
    /* TODO: Does your target support threads? Is so, implement this function.
       Otherwise just return no support. */
    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int atuc3_current_thread_query(rp_thread_ref *thread)
{
    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_current_thread_query()",
                        atuc3_target.name);
    /* TODO: Does your target support threads? Is so, implement this function.
       Otherwise just return no support. */
    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int atuc3_offsets_query(uint64_t *text, uint64_t *data, uint64_t *bss)
{
    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_offsets_query()",
                        atuc3_target.name);

    assert(atuc3_status.is_open);

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
static int atuc3_crc_query(uint64_t addr, size_t len, uint32_t *val)
{
    uint8_t buf[1];

    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_crc_query()",
                        atuc3_target.name);

    assert(atuc3_status.is_open);

    if (addr > RP_ATUC3_MAX_ADDRESS  ||  addr + len > RP_ATUC3_MAX_ADDRESS + 1)
    {
        atuc3_status.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: bad address 0x%llx",
                            atuc3_target.name,
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
static int atuc3_raw_query(char *in_buf, char *out_buf, size_t out_buf_size)
{
    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_raw_query() %s",
                        atuc3_target.name, in_buf);

    if (strncmp(in_buf + 1, "Xfer:sysreg:", 12) == 0)
    {
        /* Access system registers */
        atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: atuc3_sysreg_query()",
                            atuc3_target.name);
        //atuc3_sysreg_query(inbuf+13, outbuf, out_buf_size);
    }
    if (strncmp(in_buf + 1, "Xfer:pin:", 9) == 0)
    {
        /* Access device pins */
        atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: atuc3_pin_query()",
                            atuc3_target.name);
    }
    return RP_VAL_TARGETRET_NOSUPP;
}

/* command: erase flash */
static int atuc3_rcmd_erase(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    char buf[1000 + 1];

    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_rcmd_erase()",
                        atuc3_target.name);
    rp_encode_string("Erasing target flash - ", buf, size_t(1000));
    of(t, buf);

    /* TODO: perform the erase. */

    rp_encode_string(" Erased OK\n", buf, size_t(1000));
    of(t, buf);
    return RP_VAL_TARGETRET_OK;
}

/* command: Specify the model to use */
static int atuc3_rcmd_setlib(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    char buf[1000 + 1];
    char reply_buf[256];

    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_rcmd_setlib()",
                        atuc3_target.name);
    if(!argv[1])
    {
        rp_encode_string("No new lib file specified.\n", buf, size_t(1000));
        of(t, buf);
        return RP_VAL_TARGETRET_OK;
    }
    if(atuc3_status.lib)
    {
        if(strcmp(atuc3_status.lib, argv[1]) == 0)
        {
            sprintf(reply_buf, "%s already selected!\n", atuc3_status.lib);
            rp_encode_string(reply_buf, buf, size_t(1000));
            of(t, buf);
            return RP_VAL_TARGETRET_OK;
        }
        sprintf(reply_buf, "Current lib file: %s\n", atuc3_status.lib);
        rp_encode_string(reply_buf, buf, size_t(1000));
        of(t, buf);
        free(atuc3_status.lib);
    }

    atuc3_status.lib = strcpy((char*)malloc(strlen(argv[1]) + 1), (argv[1]));
    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: argv=%s",
                        atuc3_target.name, argv[1]);

    sprintf(reply_buf, "New lib file: %s\n", atuc3_status.lib);
    rp_encode_string(reply_buf, buf, size_t(1000));
    of(t, buf);

    /* Clear selected device */
    if(atuc3_status.device)
        free(atuc3_status.device);

    /* Check if second argument is present, and it starts with 'A' or 'a' */
    if(argv[2] && ((*argv[2] & 0xdf) == 'A'))
    {
        atuc3_status.device = strcpy((char*)malloc(strlen(argv[2]) + 1), (argv[2]));
        atuc3_load_model(of, t);
    }
    else
        atuc3_status.device = NULL;

    return RP_VAL_TARGETRET_OK;
}

/* command: Print the selected model */
static int atuc3_rcmd_getlib(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    char buf[1000 + 1];
    char reply_buf[256];

    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_rcmd_getlib()",
                        atuc3_target.name);

    if(atuc3_status.lib)
    {
        sprintf(reply_buf, "%s\n", atuc3_status.lib);
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
static int atuc3_rcmd_setdevice(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    char buf[1000 + 1];
    char reply_buf[256];

    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_rcmd_setdevice()",
                        atuc3_target.name);
    if(!argv[1])
    {
        rp_encode_string("No new device specified.\n", buf, size_t(1000));
        of(t, buf);
        return RP_VAL_TARGETRET_OK;
    }
    if(atuc3_status.device)
    {
        if(strcmp(atuc3_status.device, argv[1]) == 0)
        {
            sprintf(reply_buf, "%s already selected!\n", atuc3_status.device);
            rp_encode_string(reply_buf, buf, size_t(1000));
            of(t, buf);
            return RP_VAL_TARGETRET_OK;
        }
    }

    if(atuc3_status.device)
        free(atuc3_status.device);
    atuc3_status.device = strcpy((char*)malloc(strlen(argv[1]) + 1), (argv[1]));

    atuc3_load_model(of, t);

    return RP_VAL_TARGETRET_OK;
}

/* command: Print the selected device */
static int atuc3_rcmd_getdevice(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    char buf[1000 + 1];
    char reply_buf[256];

    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_rcmd_setdevice()",
                        atuc3_target.name);
    if(atuc3_status.device)
    {
        sprintf(reply_buf, "%s\n", atuc3_status.device);
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
static int atuc3_rcmd_reset(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{ 
    char buf[1000 + 1];
    char reply_buf[256];
    ResetType rst;

    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_rcmd_reset()",
                        atuc3_target.name);

    /* Default to POR reset */
    rst = Reset_por;
    sprintf(reply_buf, "Performing Power-on reset - ", atuc3_status.device);

    if(argc > 1)
    {
        if(strcmp("por", argv[1]) == 0);
        else if(strcmp("bod", argv[1]) == 0)
        {
            sprintf(reply_buf, "Performing Brown-out reset - ", atuc3_status.device);
            rst = Reset_bod;
        }
        else if(strcmp("ext", argv[1]) == 0)
        {
            sprintf(reply_buf, "Performing External reset - ", atuc3_status.device);
            rst = Reset_ext;
        }
        else if(strcmp("spike", argv[1]) == 0)
        {
            sprintf(reply_buf, "Performing Spike reset - ", atuc3_status.device);
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
static int atuc3_rcmd_getcyclecnt(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    uint64_t counter;
    char buf[1000 + 1];
    char reply_buf[256];

    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_rcmd_getcyclecnt()",
                        atuc3_target.name);
    mC->peekReg(R_CYCLECOUNT, &counter);
    sprintf(reply_buf, "Cycle counter - %d\n", counter);
    rp_encode_string(reply_buf, buf, size_t(1000));
    of(t, buf);

    return RP_VAL_TARGETRET_OK;
}

/* command: Get Lifetime Counter */
static int atuc3_rcmd_getlifecnt(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    uint64_t counter;
    char buf[1000 + 1];
    char reply_buf[256];

    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_rcmd_getlifecnt()",
                        atuc3_target.name);
    mC->peekReg(R_LIFETIMECOUNT, &counter);
    sprintf(reply_buf, "Lifetime counter - %d\n", counter);
    rp_encode_string(reply_buf, buf, size_t(1000));
    of(t, buf);

    return RP_VAL_TARGETRET_OK;
}

/* command: debug - Extended debug features for models built with debug option */
static int atuc3_rcmd_debug(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    char buf[1000 + 1] = {""};

    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_rcmd_debug()",
                        atuc3_target.name);

    int i = 1;
    while(i < argc)
    {
        atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: atuc3_rcmd_debug(): argv[%d]=%s",
                            atuc3_target.name, i, argv[i]);
        if(i > 1)
            strcat(buf, " ");
        strcat(buf, argv[i]);
        i++;
    }
    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_rcmd_debug(): buf=%s",
                        atuc3_target.name, buf);
    mM->debug(buf);

    return RP_VAL_TARGETRET_OK;
}

/* command: info - Extended target information */
static int atuc3_rcmd_info(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    char buf[1000 + 1];
    char reply_buf[500 + 1];

    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_rcmd_info()",
                        atuc3_target.name);

    if(argc == 1)
    {
        /* No arguments. Print help text. */
    }
    int i = 1;
    while(i < argc)
    {
        atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: atuc3_rcmd_debug(): argv[%d]=%s",
                            atuc3_target.name, i, argv[i]);
        if(i > 1)
            strcat(buf, " ");
        strcat(buf, argv[i]);
        i++;
    }
    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_rcmd_debug(): buf=%s",
                        atuc3_target.name, buf);

    return RP_VAL_TARGETRET_OK;
}

/* command: lastbreak - Extended information about the last breakpoint hit */
static int atuc3_rcmd_lastbreak(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    char buf[1000 + 1];
    char reply_buf[500 + 1];
    int index;

    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_rcmd_lastbreak()",
                        atuc3_target.name);
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
static int atuc3_add_break(int type, uint64_t addr, unsigned int len)
{
    int retVal;
    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_add_break(%d, 0x%llx, %d)",
                        atuc3_target.name,
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
static int atuc3_remove_break(int type, uint64_t addr, unsigned int len)
{
    int retVal;
    BPtype t;
    Segment s;
    int id = -1;
    uint64_t l = len;

    atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atuc3_remove_break(%d, 0x%llx, %d)",
                        atuc3_target.name,
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
static char *atuc3_out_treg(char *in, unsigned int reg_no)
{
    static const char hex[] = "0123456789abcdef";
    uint8_t data_buf[sizeof(RP_ATUC3_REG_DATATYPE)];
    size_t read_size;

    if (in == NULL)
        return NULL;

    assert(reg_no < RP_ATUC3_NUM_REGS);

    *in++ = hex[(reg_no >> 4) & 0x0f];
    *in++ = hex[reg_no & 0x0f];
    *in++ = ':';

    read_size = mC->readMemory(reg_no * sizeof(RP_ATUC3_REG_DATATYPE),
                                sizeof(RP_ATUC3_REG_DATATYPE),
                                data_buf, Seg_Reg);

    assert (read_size == sizeof(RP_ATUC3_REG_DATATYPE));  // @@@FIXME
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
//     for (i = 0;  i < RP_ATUC3_NUM_REGS;  i++)
//     {
// //        atuc3_status.registers[i] = mM->peekMemoryWord32(i, Seg_Reg);
//         atuc3_status.registers[i] = mM->peekReg(i);
//     }
//     return  TRUE;
// }

int atuc3_load_model(out_func of, rp_target* t)
{
    char buf[1000 + 1];
    char reply_buf[256];

    if(atuc3_status.device)
        sprintf(reply_buf, "Loading %s... ", atuc3_status.device);
    else
        sprintf(reply_buf, "Loading... ");
    rp_encode_string(reply_buf, buf, size_t(1000));
    of(t, buf);

    mM = mMgr->load(atuc3_status.lib, atuc3_status.device);

    if(mM)
    {
        if(atuc3_status.device)
        {
            char name_buf[256];
            if(mM->getStringProperty(P_DeviceName, sizeof(name_buf), name_buf))
                atuc3_status.device = strcpy((char*)malloc(strlen(name_buf) + 1), (name_buf));
        }
        sprintf(reply_buf, "Successfully loaded %s\n", atuc3_status.device);
        rp_encode_string(reply_buf, buf, size_t(1000));
        of(t, buf);
    }
    else
    {
        rp_encode_string("FAILED! No device loaded.\n", buf, size_t(1000));
        of(t, buf);
    }
}
