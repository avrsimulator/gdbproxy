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
 
 
   Implementation of a Atmel AVR 8-bit target for the GDB proxy server.
   
   Exported Data:
     atavr8_target             - target descriptor of the `atavr8' target 
  
   Imported Data:
     None     
  
   Static Data:
     atavr8_XXXX               - static data representing status and 
                                   parameters of the target 
  
   Global Functions:  
     None
  
   Static Functions:  
     atavr8_XXXX              - methods comprising the `atavr8' target.
                                  A description is in file gdbproxy.h
 
     atavr8_                  - local finctions
     atavr8_command
 
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

/* Note: we are using prefix 'atavr8' for static stuff in
   order to simplify debugging of the target code itself */

/* TODO: Put the correct values for the real target in these macros */
#define RP_ATAVR8_MIN_ADDRESS             0x0U
#define RP_ATAVR8_MAX_ADDRESS             0xFFFFFFFFULL
#define RP_ATAVR8_REG_DATATYPE            uint8_t
//#define RP_ATAVR8_REG_BYTES               (AVR_NUM_REGS*sizeof(RP_ATAVR8_REG_DATATYPE))
#define AVR_NUM_GEN_REGS                  32

enum
{
  AVR_REG_W = 24,
  AVR_REG_X = 26,
  AVR_REG_Y = 28,
  AVR_FP_REGNUM = 28,
  AVR_REG_Z = 30,

  AVR_SREG_REGNUM = 32,
  AVR_SP_REGNUM = 33,
  AVR_PC_REGNUM = 34,

  AVR_NUM_REGS = 32 + 1 /*SREG*/ + 1 /*SP*/ + 1 /*PC*/,
  AVR_NUM_REG_BYTES = 32 + 1 /*SREG*/ + 2 /*SP*/ + 4 /*PC*/,

  /* Pseudo registers.  */
  AVR_PSEUDO_PC_REGNUM = 35,
  AVR_NUM_PSEUDO_REGS = 1,

  AVR_PC_REG_INDEX = 35,    /* index into array of registers */

  AVR_MAX_PROLOGUE_SIZE = 64,   /* bytes */

  /* Count of pushed registers.  From r2 to r17 (inclusively), r28, r29 */
  AVR_MAX_PUSHES = 18,

  /* Number of the last pushed register.  r17 for current avr-gcc */
  AVR_LAST_PUSHED_REGNUM = 17,

  AVR_ARG1_REGNUM = 24,         /* Single byte argument */
  AVR_ARGN_REGNUM = 25,         /* Multi byte argments */

  AVR_RET1_REGNUM = 24,         /* Single byte return value */
  AVR_RETN_REGNUM = 25,         /* Multi byte return value */

  AVR_IMEM_START = 0x00000000,  /* INSN memory */
  AVR_SMEM_START = 0x00800000,  /* SRAM memory */
#if 1
  /* No eeprom mask defined */
  AVR_MEM_MASK = 0x00f00000,    /* mask to determine memory space */
#else
  AVR_EMEM_START = 0x00810000,  /* EEPROM memory */
  AVR_MEM_MASK = 0x00ff0000,    /* mask to determine memory space */
#endif
};

//#define RP_ATAVR8_NUM_XREGS               66

#define RP_ATAVR8_MAX_BREAKPOINTS         512 /* This is a simulator, in theory unlimited :-) */

/* Some example states a real target might support. */
#define RP_ATAVR8_TARGET_STATE_RUNNING                    0
#define RP_ATAVR8_TARGET_STATE_STOPPED                    1
#define RP_ATAVR8_TARGET_STATE_SINGLE_STEP_COMPLETE       2
#define RP_ATAVR8_TARGET_STATE_RUN_TO_ADDR_COMPLETE       3
#define RP_ATAVR8_TARGET_STATE_BREAKPOINT_HIT             4

/*
 * Target methods, static
 */
static void  atavr8_help(const char *prog_name);
static int   atavr8_open(int argc,
                           char * const argv[],
                           const char *prog_name,
                           log_func log_fn);
static void  atavr8_close(void);
static int   atavr8_connect(char *status_string,
                              size_t status_string_size,
                              int *can_restart);
static int   atavr8_disconnect(void);
static void  atavr8_kill(void);
static int   atavr8_restart(void);
static void  atavr8_stop(void);
static int   atavr8_set_gen_thread(rp_thread_ref *thread);
static int   atavr8_set_ctrl_thread(rp_thread_ref *thread);
static int   atavr8_is_thread_alive(rp_thread_ref *thread, int *alive);
static int   atavr8_read_registers(uint8_t *data_buf,
                                     uint8_t *avail_buf,
                                     size_t buf_size,
                                     size_t *read_size);
static int   atavr8_write_registers(uint8_t *data_buf, size_t write_size);
static int   atavr8_read_single_register(unsigned int reg_no,
                                           uint8_t *data_buf,
                                           uint8_t *avail_buf,
                                           size_t buf_size,
                                           size_t *read_size);
static int   atavr8_write_single_register(unsigned int reg_no,
                                            uint8_t *data_buf,
                                            size_t write_size);
static int   atavr8_read_mem(uint64_t addr,
                               uint8_t *data_buf,
                               size_t req_size,
                               size_t *actual_size);
static int   atavr8_write_mem(uint64_t addr,
                                uint8_t *data_buf,
                                size_t req_sise);
static int   atavr8_resume_from_current(int step, int sig);
static int   atavr8_resume_from_addr(int step,
                                       int sig,
                                       uint64_t addr);
static int   atavr8_go_waiting(int sig);
static int   atavr8_wait_partial(int first,
                                   char *status_string,
                                   size_t status_string_len,
                                   out_func out,
                                   int *implemented,
                                   int *more,
                                   rp_target* t);
static int   atavr8_wait(char *status_string,
                           size_t status_string_len,
                           out_func out,
                           int *implemented,
                           rp_target* t);
static int   atavr8_process_query(unsigned int *mask,
                                    rp_thread_ref *arg,
                                    rp_thread_info *info);
static int   atavr8_list_query(int first,
                                 rp_thread_ref *arg,
                                 rp_thread_ref *result,
                                 size_t max_num,
                                 size_t *num,
                                 int *done);
static int   atavr8_current_thread_query(rp_thread_ref *thread);
static int   atavr8_offsets_query(uint64_t *text,
                                    uint64_t *data,
                                    uint64_t *bss);
static int   atavr8_crc_query(uint64_t addr,
                                size_t len,
                                uint32_t *val);
static int   atavr8_raw_query(char *in_buf,
                                char *out_buf,
                                size_t out_buf_size);
static int   atavr8_add_break(int type, uint64_t addr, unsigned int len);
static int   atavr8_remove_break(int type, uint64_t addr, unsigned int len);

static uint32_t crc32(uint8_t *buf, size_t len, uint32_t crc);

#define RCMD(name, hlp) {#name, atavr8_rcmd_##name, hlp}  //table entry generation

/* Prototyping of remote commands */
static int atavr8_rcmd_erase(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atavr8_rcmd_setlib(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atavr8_rcmd_getlib(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atavr8_rcmd_setdevice(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atavr8_rcmd_getdevice(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atavr8_rcmd_reset(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atavr8_rcmd_getcyclecnt(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atavr8_rcmd_getlifecnt(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atavr8_rcmd_debug(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atavr8_rcmd_info(int argc, char *argv[], out_func of, data_func df, rp_target* t);
static int atavr8_rcmd_lastbreak(int argc, char *argv[], out_func of, data_func df, rp_target* t);

/* Table of remote commands */
static const RCMD_TABLE atavr8_remote_commands[] =
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

// String arrays, must be kept in sync with model.h/breakpoint.h enums.
static const char *bptypes[] =
{ "none", "exec", "write", "N/A=3", "read", "N/A=5", "access", "N/A=7",
  "modify"};
static const char *segtypes[] =
{ "code", "data", "eeprom", "reg", "i/o", "sysreg", "none" };

/*
 * Global target descriptor 
 */
rp_target atavr8_target =
{
    NULL,      /* next */
    -1,
    -1,
    0,
    0,
    0,
    0,
    "atavr8",
    "atavr8 target to use GDB proxy server with Atmel AVR 8-bit simulator models",
    atavr8_remote_commands,
    atavr8_help,
    atavr8_open,
    atavr8_close,
    atavr8_connect,
    atavr8_disconnect,
    atavr8_kill,
    atavr8_restart,
    atavr8_stop,
    atavr8_set_gen_thread,
    atavr8_set_ctrl_thread,
    atavr8_is_thread_alive,
    atavr8_read_registers,
    atavr8_write_registers,
    atavr8_read_single_register,
    atavr8_write_single_register,
    atavr8_read_mem,
    atavr8_write_mem,
    atavr8_resume_from_current,
    atavr8_resume_from_addr,
    atavr8_go_waiting,
    atavr8_wait_partial,
    atavr8_wait,
    atavr8_process_query,
    atavr8_list_query,
    atavr8_current_thread_query,
    atavr8_offsets_query,
    atavr8_crc_query,
    atavr8_raw_query,
    atavr8_add_break,
    atavr8_remove_break
};

struct atavr8_status_s
{
    /* Start up parameters, set by atavr8_open */
    log_func    log;
    int         is_open;

    /* Tell wait_xxx method the notion of whether or not
       previously called resume is supported */
    int         target_running;
    int         target_interrupted;
    int         state;
    RP_ATAVR8_REG_DATATYPE    registers[AVR_NUM_REGS];
    uint64_t                    breakpoints[RP_ATAVR8_MAX_BREAKPOINTS];
    bool        deferUnload;
    char       *lib;
    char       *device;
    unsigned    selectedCore;
};

static struct atavr8_status_s atavr8_status =
{
    NULL,
    FALSE,
    FALSE,
    FALSE,
    RP_ATAVR8_TARGET_STATE_STOPPED,
    {0},
    {0},
    TRUE,
    '\0',
    '\0',
    0
};

/* Local functions */
static char *atavr8_out_treg(char *in, unsigned int reg_no);
// static int refresh_registers(void); // Not used. Access model directly.
static void printErr(const ModelError &err);
int atavr8_load_model(out_func of, rp_target* t);

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
//jie        atavr8_stop();
}

uint64_t atavr8_getIntProp(Core *c, int propId, const char *name)
{
    uint64_t  val = 0;
    if (c->getIntProperty(propId, &val, name))
        return val;
    else if (propId == P_Signature)
        return atavr8_getIntProp(c, P_JTAGID, name);
    else
        return 0;
}


uint64_t atavr8_getIntProp(Model *m, int propId, const char *name)
{
    return atavr8_getIntProp(m->getCore(atavr8_status.selectedCore), propId, name);
}


/* Target method */

static void atavr8_help(const char *prog_name)
{
    printf("This is the atavr8 target for the GDB proxy server. Usage:\n\n");
    printf("  %s [options] %s [atavr8-options]\n",
           prog_name,
           atavr8_target.name);
    printf("\nOptions:\n\n");
    printf("  --debug              run %s in debug mode\n", prog_name);
    printf("  --help               `%s --help %s'  prints this message\n",
           prog_name,
           atavr8_target.name);
    printf("  --port=PORT          use the specified TCP port\n");
    printf("\natavr8-options:\n\n");
    printf("  --model=MODEL        use the specified .%s file\n", dll);
    printf("  --device=DEVICE      use the specified device\n");

    printf("\n");

    return;
}

/* Target method */
static int atavr8_open(int argc,
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

    assert(!atavr8_status.is_open);
    assert(prog_name != NULL);
    assert(log_fn != NULL);

    /* Set log */
    atavr8_status.log = log_fn;

    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_open()",
                        atavr8_target.name);

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
            /* If no '/' in string, then assume current dir and prepend with "./" */
            if(!(strchr(optarg, '/')))
            {
                atavr8_status.lib = strcpy((char*)malloc(strlen(optarg) + 3), "./");
                strcat(atavr8_status.lib, optarg);
            }
            else
            {
                atavr8_status.lib = strcpy((char*)malloc(strlen(optarg) + 1), (optarg));
            }
            atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: lib option: %s, lib: %s",
                            atavr8_target.name, optarg, atavr8_status.lib);
            break;
        case 2:
            atavr8_status.device = strcpy((char*)malloc(strlen(optarg) + 1), (optarg));
            atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: device option: %s, device: %s",
                            atavr8_target.name, optarg, atavr8_status.device);
            break;
       case 0:
            /* Long option which just sets a flag */
            break;
        default:
            atavr8_status.log(RP_VAL_LOGLEVEL_NOTICE,
                                "%s: Use `%s --help %s' to see a complete list of options",
                                atavr8_target.name,
                                prog_name,
                                atavr8_target.name);
            return RP_VAL_TARGETRET_ERR;
        }
    }


    if (!atavr8_status.is_open)
    {
        atavr8_status.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: Checking for file specified!",
                            atavr8_target.name);
        /* TODO: Verify precense of model file, but load() in modelmgr.cpp
         * still fails unless we specify path (i.e. prepend with "./" */
        if(access(atavr8_status.lib, F_OK) == -1)
        {
            atavr8_status.log(RP_VAL_LOGLEVEL_ERR,
                                "%s: Can not find file specified!",
                                atavr8_target.name);
            return RP_VAL_TARGETRET_ERR;
        }
    }

    /* Set up initial default values */
    atavr8_status.target_running = FALSE;
    atavr8_status.target_interrupted = FALSE;
    atavr8_status.state = RP_ATAVR8_TARGET_STATE_STOPPED;
    memset (atavr8_status.registers, 0, sizeof(atavr8_status.registers));
    memset (atavr8_status.breakpoints, 0, sizeof(atavr8_status.breakpoints));

    atavr8_status.is_open = TRUE;

    return RP_VAL_TARGETRET_OK;
}


/* Target method */
static void atavr8_close(void)
{
    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_close()",
                        atavr8_target.name);

    assert(atavr8_status.is_open);

    /* TODO: Tidy up things and shut down. */

    atavr8_status.is_open = FALSE;

    free(atavr8_status.lib);
    atavr8_status.lib = NULL;
    free(atavr8_status.device);
    atavr8_status.device = NULL;
    free(mBp);
    mBp = NULL;
    delete mMgr;
}

/* Target method */
static int atavr8_connect(char *status_string,
                            size_t status_string_len,
                            int *can_restart)
{
    char *cp;
    static ModelMgr mgr;

    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_connect()",
                        atavr8_target.name);

    if(!atavr8_status.lib)
    {
        atavr8_status.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: No simulator model specified!",
                            atavr8_target.name);
        return RP_VAL_TARGETRET_ERR;
    }

    assert(atavr8_status.is_open);

    assert(status_string != NULL);
    assert(status_string_len >= 34);
    assert(can_restart != NULL);

    *can_restart = TRUE;

    mgr.setDeferUnload(atavr8_status.deferUnload);
    mM = mgr.load(atavr8_status.lib, atavr8_status.device);
    if(mM)
    {
       atavr8_status.log(RP_VAL_LOGLEVEL_NOTICE,
                           "%s: Simulator model loaded",
                           atavr8_target.name);
       mMgr = &mgr;
       int numcores = atavr8_getIntProp(mM, P_NumCores, 0);
       for (int i = 0; i < numcores; i++)
       {
           mC = mM->getCore(i);
           if (!mC || mC->addStepCallback(stepcallback, NULL) <= 0)
               printf("addStepCallback(core %d) failed."
                       " Ctrl-C WILL NOT WORK\n", i);
       }
       mC = mM->getCore(atavr8_status.selectedCore);

       char name_buf[256];
       if(mM->getStringProperty(P_DeviceName, sizeof(name_buf), name_buf))
            atavr8_status.device = strcpy((char*)malloc(strlen(name_buf) + 1), (name_buf));
    }
    else
    {
        atavr8_status.log(RP_VAL_LOGLEVEL_NOTICE,
                           "%s: Loading of simulator model failed",
                           atavr8_target.name);
        printErr(mgr.getError());
        return RP_VAL_TARGETRET_ERR;
    }

    /* Fill out the the status string */
    sprintf(status_string, "T%02d", RP_SIGNAL_ABORTED);

    
    cp = atavr8_out_treg(&status_string[3], AVR_PC_REGNUM);
    cp = atavr8_out_treg(cp, AVR_FP_REGNUM);

    return (cp != NULL)  ?  RP_VAL_TARGETRET_OK  :  RP_VAL_TARGETRET_ERR;
}

/* Target method */
static int atavr8_disconnect(void)
{
    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_disconnect()",
                        atavr8_target.name);

    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static void atavr8_kill(void)
{
    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_kill()",
                        atavr8_target.name);

    /* Kill the target debug session. */
    atavr8_stop();
}

static int atavr8_restart(void)
{
    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_restart()",
                        atavr8_target.name);

    /* Just stop it. The actual restart will be done
       when connect is called again */
    atavr8_stop();

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static void atavr8_stop(void)
{
    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_stop()",
                        atavr8_target.name);

    assert(atavr8_status.is_open);

    /* Stop (i,e, break) the target program. */
    mC->stop();

    atavr8_status.target_interrupted = TRUE;
    atavr8_status.state = RP_ATAVR8_TARGET_STATE_STOPPED;
}

static int atavr8_set_gen_thread(rp_thread_ref *thread)
{
    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_set_gen_thread()",
                        atavr8_target.name);

    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int atavr8_set_ctrl_thread(rp_thread_ref *thread)
{
    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_set_ctrl_thread()",
                        atavr8_target.name);

    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int atavr8_is_thread_alive(rp_thread_ref *thread, int *alive)
{
    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_is_thread_alive()",
                        atavr8_target.name);

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atavr8_read_registers(uint8_t *data_buf,
                                 uint8_t *avail_buf,
                                 size_t buf_size,
                                 size_t *read_size)
{
    size_t tmp_size;

    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_read_registers()",
                        atavr8_target.name);

    assert(atavr8_status.is_open);

    assert(data_buf != NULL);
    assert(avail_buf != NULL);
    assert(buf_size >= AVR_NUM_REG_BYTES);
    assert(read_size != NULL);

    /* Read general registers */
    tmp_size = mC->readMemory(0,
                              32,
                              data_buf, Seg_Reg);
    atavr8_read_single_register(AVR_SREG_REGNUM,
                                data_buf+32,
                                avail_buf+32,
                                buf_size-32,
                                read_size);
    tmp_size += *read_size;
    atavr8_read_single_register(AVR_SP_REGNUM,
                                data_buf+33,
                                avail_buf+33,
                                buf_size-33,
                                read_size);
    tmp_size += *read_size;
    atavr8_read_single_register(AVR_PC_REGNUM,
                                data_buf+35,
                                avail_buf+35,
                                buf_size-35,
                                read_size);
    *read_size += tmp_size;
    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: read_size = %d",
                        atavr8_target.name, *read_size);

    assert (*read_size == AVR_NUM_REG_BYTES);

    memset(avail_buf, 1, AVR_NUM_REG_BYTES);
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atavr8_write_registers(uint8_t *buf, size_t write_size)
{
    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_write_registers()",
                        atavr8_target.name);

    assert(atavr8_status.is_open);

    assert(buf != NULL);
    assert(write_size > 0);
    assert(write_size <= AVR_NUM_REG_BYTES);

    write_size = mC->writeMemory(0,
                                 AVR_NUM_REG_BYTES,
                                 buf, Seg_Reg);
    assert (write_size == AVR_NUM_REG_BYTES);
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atavr8_read_single_register(unsigned int reg_no,
                                         uint8_t *data_buf,
                                         uint8_t *avail_buf,
                                         size_t buf_size,
                                         size_t *read_size)
{
    uint64_t tmp_reg;

    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_read_single_register(%d)",
                        atavr8_target.name, reg_no);

    assert(atavr8_status.is_open);

    assert(data_buf != NULL);
    assert(avail_buf != NULL);
    assert(buf_size >= (sizeof(RP_ATAVR8_REG_DATATYPE) * 4));
    assert(read_size != NULL);

    if (reg_no < 0  ||  reg_no >= AVR_NUM_REGS)
        return RP_VAL_TARGETRET_ERR;

    /* Asking for status register */
    if(reg_no == AVR_SREG_REGNUM)
    {
        mC->peekReg(R_STATUS, &tmp_reg);
        atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: SREG = %x",
                            atavr8_target.name, tmp_reg);
        *data_buf = (uint8_t)tmp_reg;
        *read_size = 1;
    }
    else if(reg_no == AVR_SP_REGNUM)
    {
        mC->peekReg(R_SP, &tmp_reg);
        atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: SP = %x",
                            atavr8_target.name, tmp_reg);
        *data_buf = (uint8_t)tmp_reg;
        *(data_buf+1) = (uint8_t)(tmp_reg>>8);
        *read_size = 2;
    }
    else if(reg_no == AVR_PC_REGNUM)
    {
        mC->peekReg(R_PC, &tmp_reg);
        atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: PC = %x",
                            atavr8_target.name, tmp_reg);
        *data_buf = (uint8_t)tmp_reg;
        *(data_buf+1) = (uint8_t)(tmp_reg>>8);
        *(data_buf+2) = (uint8_t)(tmp_reg>>16);
        *(data_buf+3) = (uint8_t)(tmp_reg>>24);
        *read_size = 4;
    }
    else
    {
        *read_size = mC->peekReg(reg_no, &tmp_reg);
        for(int i=0; i<*read_size; i++)
        {
            *(data_buf+i) = (uint8_t)(tmp_reg>>(i*8));
        }
        assert (*read_size == sizeof(RP_ATAVR8_REG_DATATYPE));  // @@@FIXME
    }

    memset(avail_buf, 1, *read_size);
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atavr8_write_single_register(unsigned int reg_no,
                                          uint8_t *buf,
                                          size_t write_size)
{
    uint64_t tmp_reg = 0;
    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_write_single_register(%d, 0x%X)",
                        atavr8_target.name,
                        reg_no,
                        ((RP_ATAVR8_REG_DATATYPE *) buf)[0]);

    assert(atavr8_status.is_open);

    assert(buf != NULL);

    if (reg_no < 0  ||  reg_no >= AVR_NUM_REGS)
        return RP_VAL_TARGETRET_ERR;

    // Copy data in buffer to tmp_reg
    for(int i=0; i<write_size; i++)
    {
        tmp_reg += *(buf+i)<<(i*8);
    }

    /* Accessing for status register */
    if(reg_no == AVR_SREG_REGNUM)
    {
        assert (write_size == sizeof(RP_ATAVR8_REG_DATATYPE));
        write_size = mC->pokeReg(R_STATUS, tmp_reg);
    }
    else if(reg_no == AVR_SP_REGNUM)
    {
        assert (write_size == (sizeof(RP_ATAVR8_REG_DATATYPE)*2));
        write_size = mC->pokeReg(R_SP, tmp_reg);
    }
    else if(reg_no == AVR_PC_REGNUM)
    {
        assert (write_size == (sizeof(RP_ATAVR8_REG_DATATYPE)*4));
        write_size = mC->pokeReg(R_PC, tmp_reg);
    }
    else
    {
        assert (write_size == sizeof(RP_ATAVR8_REG_DATATYPE));
        write_size = mC->pokeReg(reg_no, tmp_reg);
    }

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atavr8_read_mem(uint64_t addr,
                             uint8_t *buf,
                             size_t req_size,
                             size_t *actual_size)
{
    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_read_mem(0x%llX, ptr, %d, ptr)",
                        atavr8_target.name,
                        addr,
                        req_size);

    assert(atavr8_status.is_open);

    assert(buf != NULL);
    assert(req_size > 0);
    assert(actual_size != NULL);

    Segment seg;
    if(addr >= AVR_SMEM_START)
    {
        seg = Seg_Data;
        addr -= AVR_SMEM_START;
    }
    else
    {
        seg = Seg_Code;
    }

    *actual_size = mC->readMemory(addr, req_size, buf, seg);

    if(!(*actual_size))
    {
        atavr8_status.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: Read mem address 0x%llx size 0x%llx failed!",
                            atavr8_target.name, addr, req_size);
        return RP_VAL_TARGETRET_ERR;
    }
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atavr8_write_mem(uint64_t addr,
                              uint8_t *buf,
                              size_t write_size)
{
    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_write_mem(0x%llX, ptr, %d)",
                        atavr8_target.name,
                        addr,
                        write_size);

    assert(atavr8_status.is_open);
    assert(buf != NULL);

    /* GDB does zero length writes for some reason. Treat them harmlessly. */
    if (write_size == 0)
        return RP_VAL_TARGETRET_OK;

    Segment seg;
    if(addr >= AVR_SMEM_START)
    {
        seg = Seg_Data;
        addr -= AVR_SMEM_START;
    }
    else
    {
        seg = Seg_Code;
    }

    size_t actual_size;
    actual_size = mC->writeMemory(addr, write_size, buf, seg);
    if(actual_size != write_size)
    {
        atavr8_status.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: Write mem address 0x%llx size 0x%llx failed! Actually written 0x%llx bytes",
                            atavr8_target.name, addr, write_size, actual_size);
        return RP_VAL_TARGETRET_ERR;
    }

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atavr8_resume_from_current(int step, int sig)
{
    Breakpoint* bp;
    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_resume_from_current(%s, %d)",
                        atavr8_target.name,
                        (step)  ?  "step"  :  "run",
                        sig);

    assert(atavr8_status.is_open);

    if (step)
    {
        /* Single step the target */
        atavr8_status.state = RP_ATAVR8_TARGET_STATE_SINGLE_STEP_COMPLETE;
        bp = mC->step();
    }
    else
    {
        /* Run the target to a breakpoint, or until we stop it. */
        atavr8_status.state = RP_ATAVR8_TARGET_STATE_RUN_TO_ADDR_COMPLETE;
        bp = mC->run();
    }

    if(mBp)
        free(mBp);

    atavr8_status.target_running = FALSE;
    if(bp)
    {
        atavr8_status.target_interrupted = TRUE;
        atavr8_status.state = RP_ATAVR8_TARGET_STATE_BREAKPOINT_HIT;
        /* Copy breakpoint to support extended breakpoint info */
        mBp = (Breakpoint*)malloc(sizeof(Breakpoint));
        *mBp = *bp;
    }
    else
    {
        mBp = NULL;
        atavr8_status.target_interrupted = FALSE;
    }
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atavr8_resume_from_addr(int step, int sig, uint64_t addr)
{
    Breakpoint* bp;
    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_resume_from_addr(%s, %d, 0x%llX)",
                        atavr8_target.name,
                        (step)  ?  "step"  :  "run",
                        sig,
                        addr);

    assert(atavr8_status.is_open);

    atavr8_status.registers[AVR_PC_REGNUM] = addr;

    /* Update the PC register in the target */
    mC->pokeReg(R_PC, addr);

    /* Run the target from the new PC address. */
    if (step)
    {
        /* Single step the target */
        atavr8_status.state = RP_ATAVR8_TARGET_STATE_SINGLE_STEP_COMPLETE;
        bp = mC->step();
    }
    else
    {
        /* Run the target to a breakpoint, or until we stop it. */
        atavr8_status.state = RP_ATAVR8_TARGET_STATE_RUN_TO_ADDR_COMPLETE;
        bp = mC->run();
    }

    if(mBp)
        free(mBp);

    atavr8_status.target_running = FALSE;
    if(bp)
    {
        atavr8_status.target_interrupted = TRUE;
        atavr8_status.state = RP_ATAVR8_TARGET_STATE_BREAKPOINT_HIT;
        /* Copy breakpoint to support extended breakpoint info */
        mBp = (Breakpoint*)malloc(sizeof(Breakpoint));
        *mBp = *bp;
    }
    else
    {
        mBp = NULL;
        atavr8_status.target_interrupted = FALSE;
    }
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int atavr8_go_waiting(int sig)
{
    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_go_waiting()",
                        atavr8_target.name);
    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int atavr8_wait_partial(int first,
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

    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_wait_partial()",
                        atavr8_target.name);

    assert(atavr8_status.is_open);

    assert(status_string != NULL);
    assert(status_string_len >= 34);
    assert(of != NULL);
    assert(implemented != NULL);
    assert(more != NULL);

    *implemented = TRUE;

//    if (!atavr8_status.target_running)
//        return RP_VAL_TARGETRET_NOSUPP;
    atavr8_status.target_running = FALSE;

//#ifdef WIN32
//    sleep((first)  ?  500  :  100);
//#else
//    usleep((first)  ?  500000  :  100000);
//#endif
    /* TODO: Test the target state (i.e. running/stopped) without blocking */
    /* If the target only supports a blocking form of test return no support,
       and the blocking version of this test will be called instead. That is
       not so nice, as the system is less interactive using a blocking test. */
    state = atavr8_status.state;

    if (state == RP_ATAVR8_TARGET_STATE_RUNNING)
    {
        *more = TRUE;
        return RP_VAL_TARGETRET_OK;
    }

    switch (state)
    {
    case RP_ATAVR8_TARGET_STATE_STOPPED:
        if (atavr8_status.target_interrupted)
            sig = RP_SIGNAL_INTERRUPT;
        else
            sig = RP_SIGNAL_ABORTED;
        break;
    case RP_ATAVR8_TARGET_STATE_RUNNING:
        *more = TRUE;
        return RP_VAL_TARGETRET_OK;
    case RP_ATAVR8_TARGET_STATE_SINGLE_STEP_COMPLETE:
        sig = RP_SIGNAL_TRAP;
        break;
    case RP_ATAVR8_TARGET_STATE_BREAKPOINT_HIT:
        sig = RP_SIGNAL_TRAP;
        break;
    default:
        atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: unexpected state %d for the atavr8",
                            atavr8_target.name,
                            state);
        sig = RP_SIGNAL_ABORTED;
        break;
    }
    /* Fill out the status string */
    sprintf(status_string, "T%02d", sig);

    //if (!refresh_registers())
    //    return RP_VAL_TARGETRET_ERR;
    
    cp = atavr8_out_treg(&status_string[3], AVR_PC_REGNUM);
    cp = atavr8_out_treg(cp, AVR_FP_REGNUM);

    *more = FALSE;

    return (cp != NULL)  ?  RP_VAL_TARGETRET_OK  :  RP_VAL_TARGETRET_ERR;
}

/* Target method */
static int atavr8_wait(char *status_string,
                         size_t status_string_len,
                         out_func of,
                         int *implemented,
                         rp_target* t)
{
    int state;
    char *cp;
    int sig;

    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_wait()",
                        atavr8_target.name);

    assert(atavr8_status.is_open);

    assert(status_string != NULL);
    assert(status_string_len >= 34);
    assert(of != NULL);
    assert(implemented != NULL);

    *implemented = TRUE;

//    if (!atavr8_status.target_running)
//        return RP_VAL_TARGETRET_NOSUPP;
    atavr8_status.target_running = FALSE;

    /* TODO: Wait for the target to stop */
    state = atavr8_status.state;

    switch (state)
    {
    case RP_ATAVR8_TARGET_STATE_STOPPED:
        sig = RP_SIGNAL_ABORTED;
        break;
    case RP_ATAVR8_TARGET_STATE_SINGLE_STEP_COMPLETE:
        sig = RP_SIGNAL_TRAP;
        break;
    case RP_ATAVR8_TARGET_STATE_BREAKPOINT_HIT:
        sig = RP_SIGNAL_TRAP;
        break;
    default:
        atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: unexpected state %d for the atavr8",
                            atavr8_target.name,
                            state);
        sig = RP_SIGNAL_ABORTED;
        break;
    }
    /* Fill out the status string */
    sprintf(status_string, "T%02d", sig);

    //    if (!refresh_registers())
    //    return RP_VAL_TARGETRET_ERR;
    
    cp = atavr8_out_treg(&status_string[3], AVR_PC_REGNUM);
    cp = atavr8_out_treg(cp, AVR_FP_REGNUM);

    return (cp != NULL)  ?  RP_VAL_TARGETRET_OK  :  RP_VAL_TARGETRET_ERR;
}

/* Target method */
static int atavr8_process_query(unsigned int *mask,
                                  rp_thread_ref *arg,
                                  rp_thread_info *info)
{
    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_process_query()",
                        atavr8_target.name);
    /* TODO: Does your target support threads? Is so, implement this function.
       Otherwise just return no support. */
    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int atavr8_list_query(int first,
                               rp_thread_ref *arg,
                               rp_thread_ref *result,
                               size_t max_num,
                               size_t *num,
                               int *done)
{
    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_list_query()",
                        atavr8_target.name);
    /* TODO: Does your target support threads? Is so, implement this function.
       Otherwise just return no support. */
    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int atavr8_current_thread_query(rp_thread_ref *thread)
{
    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_current_thread_query()",
                        atavr8_target.name);
    /* TODO: Does your target support threads? Is so, implement this function.
       Otherwise just return no support. */
    return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int atavr8_offsets_query(uint64_t *text, uint64_t *data, uint64_t *bss)
{
    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_offsets_query()",
                        atavr8_target.name);

    assert(atavr8_status.is_open);

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
static int atavr8_crc_query(uint64_t addr, size_t len, uint32_t *val)
{
    uint8_t buf[1];

    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_crc_query()",
                        atavr8_target.name);

    assert(atavr8_status.is_open);

    if (addr > RP_ATAVR8_MAX_ADDRESS  ||  addr + len > RP_ATAVR8_MAX_ADDRESS + 1)
    {
        atavr8_status.log(RP_VAL_LOGLEVEL_ERR,
                            "%s: bad address 0x%llx",
                            atavr8_target.name,
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
static int atavr8_raw_query(char *in_buf, char *out_buf, size_t out_buf_size)
{
    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_raw_query() %s",
                        atavr8_target.name, in_buf);

    return RP_VAL_TARGETRET_NOSUPP;
}

/* command: erase flash */
static int atavr8_rcmd_erase(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    char buf[1000 + 1];

    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_rcmd_erase()",
                        atavr8_target.name);
    rp_encode_string("Erasing target flash - ", buf, size_t(1000));
    of(t, buf);

    /* TODO: perform the erase. */

    rp_encode_string(" Erased OK\n", buf, size_t(1000));
    of(t, buf);
    return RP_VAL_TARGETRET_OK;
}

/* command: Specify the model to use */
static int atavr8_rcmd_setlib(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    char buf[1000 + 1];
    char reply_buf[256];

    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_rcmd_setlib()",
                        atavr8_target.name);
    if(!argv[1])
    {
        rp_encode_string("No new lib file specified.\n", buf, size_t(1000));
        of(t, buf);
        return RP_VAL_TARGETRET_OK;
    }
    if(atavr8_status.lib)
    {
        if(strcmp(atavr8_status.lib, argv[1]) == 0)
        {
            sprintf(reply_buf, "%s already selected!\n", atavr8_status.lib);
            rp_encode_string(reply_buf, buf, size_t(1000));
            of(t, buf);
            return RP_VAL_TARGETRET_OK;
        }
        sprintf(reply_buf, "Current lib file: %s\n", atavr8_status.lib);
        rp_encode_string(reply_buf, buf, size_t(1000));
        of(t, buf);
        free(atavr8_status.lib);
    }

    atavr8_status.lib = strcpy((char*)malloc(strlen(argv[1]) + 1), (argv[1]));
    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: argv=%s",
                        atavr8_target.name, argv[1]);

    sprintf(reply_buf, "New lib file: %s\n", atavr8_status.lib);
    rp_encode_string(reply_buf, buf, size_t(1000));
    of(t, buf);

    /* Clear selected device */
    if(atavr8_status.device)
        free(atavr8_status.device);

    /* Check if second argument is present, and it starts with 'A' or 'a' */
    if(argv[2] && ((*argv[2] & 0xdf) == 'A'))
    {
        atavr8_status.device = strcpy((char*)malloc(strlen(argv[2]) + 1), (argv[2]));
        atavr8_load_model(of, t);
    }
    else
        atavr8_status.device = NULL;

    return RP_VAL_TARGETRET_OK;
}

/* command: Print the selected model */
static int atavr8_rcmd_getlib(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    char buf[1000 + 1];
    char reply_buf[256];

    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_rcmd_getlib()",
                        atavr8_target.name);

    if(atavr8_status.lib)
    {
        sprintf(reply_buf, "%s\n", atavr8_status.lib);
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
static int atavr8_rcmd_setdevice(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    char buf[1000 + 1];
    char reply_buf[256];

    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_rcmd_setdevice()",
                        atavr8_target.name);
    if(!argv[1])
    {
        rp_encode_string("No new device specified.\n", buf, size_t(1000));
        of(t, buf);
        return RP_VAL_TARGETRET_OK;
    }
    if(atavr8_status.device)
    {
        if(strcmp(atavr8_status.device, argv[1]) == 0)
        {
            sprintf(reply_buf, "%s already selected!\n", atavr8_status.device);
            rp_encode_string(reply_buf, buf, size_t(1000));
            of(t, buf);
            return RP_VAL_TARGETRET_OK;
        }
    }

    if(atavr8_status.device)
        free(atavr8_status.device);
    atavr8_status.device = strcpy((char*)malloc(strlen(argv[1]) + 1), (argv[1]));

    atavr8_load_model(of, t);

    return RP_VAL_TARGETRET_OK;
}

/* command: Print the selected device */
static int atavr8_rcmd_getdevice(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    char buf[1000 + 1];
    char reply_buf[256];

    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_rcmd_setdevice()",
                        atavr8_target.name);
    if(atavr8_status.device)
    {
        sprintf(reply_buf, "%s\n", atavr8_status.device);
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
static int atavr8_rcmd_reset(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    char buf[1000 + 1];
    char reply_buf[256];
    ResetType rst;

    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atsam_rcmd_reset()",
                        atavr8_target.name);

    /* Default to POR reset */
    rst = Reset_por;
    sprintf(reply_buf, "Performing Power-on reset - ", atavr8_status.device);

    if(argc > 1)
    {
        if(strcmp("por", argv[1]) == 0);
        else if(strcmp("bod", argv[1]) == 0)
        {
            sprintf(reply_buf, "Performing Brown-out reset - ", atavr8_status.device);
            rst = Reset_bod;
        }
        else if(strcmp("ext", argv[1]) == 0)
        {
            sprintf(reply_buf, "Performing External reset - ", atavr8_status.device);
            rst = Reset_ext;
        }
        else if(strcmp("spike", argv[1]) == 0)
        {
            sprintf(reply_buf, "Performing Spike reset - ", atavr8_status.device);
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
static int atavr8_rcmd_getcyclecnt(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    uint64_t counter;
    char buf[1000 + 1];
    char reply_buf[256];

    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_rcmd_getcyclecnt()",
                        atavr8_target.name);
    mC->peekReg(R_CYCLECOUNT, &counter);
    sprintf(reply_buf, "Cycle counter - %d\n", counter);
    rp_encode_string(reply_buf, buf, size_t(1000));
    of(t, buf);

    return RP_VAL_TARGETRET_OK;
}

/* command: Get Lifetime Counter */
static int atavr8_rcmd_getlifecnt(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    uint64_t counter;
    char buf[1000 + 1];
    char reply_buf[256];

    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_rcmd_getlifecnt()",
                        atavr8_target.name);
    mC->peekReg(R_LIFETIMECOUNT, &counter);
    sprintf(reply_buf, "Lifetime counter - %d\n", counter);
    rp_encode_string(reply_buf, buf, size_t(1000));
    of(t, buf);

    return RP_VAL_TARGETRET_OK;
}

/* command: debug - Extended debug features for models built with debug option */
static int atavr8_rcmd_debug(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    char buf[1000 + 1] = {""};

    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_rcmd_debug()",
                        atavr8_target.name);

    int i = 1;
    while(i < argc)
    {
        atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: atavr8_rcmd_debug(): argv[%d]=%s",
                            atavr8_target.name, i, argv[i]);
        if(i > 1)
            strcat(buf, " ");
        strcat(buf, argv[i]);
        i++;
    }
    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_rcmd_debug(): buf=%s",
                        atavr8_target.name, buf);
    // @@@@TODO: Pass a print function that prints in client console (of).
    mM->debug(buf);

    return RP_VAL_TARGETRET_OK;
}

/* command: info - Extended target information */
static int atavr8_rcmd_info(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    char buf[1000 + 1];
    char reply_buf[500 + 1];

    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_rcmd_info()",
                        atavr8_target.name);

    if(argc == 1)
    {
        /* No arguments. Print help text. */
    }
    int i = 1;
    while(i < argc)
    {
        atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: atavr8_rcmd_debug(): argv[%d]=%s",
                            atavr8_target.name, i, argv[i]);
        if(i > 1)
            strcat(buf, " ");
        strcat(buf, argv[i]);
        i++;
    }
    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_rcmd_debug(): buf=%s",
                        atavr8_target.name, buf);

    return RP_VAL_TARGETRET_OK;
}

/* command: lastbreak - Extended information about the last breakpoint hit */
static int atavr8_rcmd_lastbreak(int argc, char *argv[], out_func of, data_func df, rp_target* t)
{
    char buf[1000 + 1];
    char reply_buf[500 + 1];
    int index;

    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_rcmd_lastbreak()",
                        atavr8_target.name);
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
static int atavr8_add_break(int type, uint64_t addr, unsigned int len)
{
    int retVal;
    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_add_break(%d, 0x%llx, %d)",
                        atavr8_target.name,
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
static int atavr8_remove_break(int type, uint64_t addr, unsigned int len)
{
    int retVal;
    BPtype t;
    Segment s;
    int id = -1;
    uint64_t l = len;

    atavr8_status.log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: atavr8_remove_break(%d, 0x%llx, %d)",
                        atavr8_target.name,
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
static char *atavr8_out_treg(char *in, unsigned int reg_no)
{
    static const char hex[] = "0123456789abcdef";
    uint8_t data_buf[sizeof(RP_ATAVR8_REG_DATATYPE) * 4];
    uint8_t avail_buf[sizeof(RP_ATAVR8_REG_DATATYPE) * 4];
    size_t read_size;

    if (in == NULL)
        return NULL;

    assert(reg_no < AVR_NUM_REGS);

    *in++ = hex[(reg_no >> 4) & 0x0f];
    *in++ = hex[reg_no & 0x0f];
    *in++ = ':';

//    read_size = mM->readMemory(reg_no * sizeof(RP_ATAVR8_REG_DATATYPE),
//                                sizeof(RP_ATAVR8_REG_DATATYPE),
//                                data_buf, Seg_Reg);

    atavr8_read_single_register(reg_no, data_buf, avail_buf, sizeof(data_buf), &read_size);


    assert (read_size <= sizeof(RP_ATAVR8_REG_DATATYPE) * 4);  // @@@FIXME
    /* The register goes into the buffer in little-endian order */
    for(int i=0; i < read_size; i++)
    {
        *in++ = hex[(data_buf[i] >> 4) & 0x0f];
        *in++ = hex[data_buf[i] & 0x0f];
    }
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
//     for (i = 0;  i < AVR_NUM_REGS;  i++)
//     {
// //        atavr8_status.registers[i] = mM->peekMemoryWord32(i, Seg_Reg);
//         atavr8_status.registers[i] = mM->peekReg(i);
//     }
//     return  TRUE;
// }

static void printErr(const ModelError &err)
{
    struct Conv
    {
        int   c;
        const char *s;
    };
    static struct Conv convtab[] =
    {
        { MODEL_NO_ERROR,             "No error" },
        { MODEL_LOAD_ERROR,           "Model load error" },
        { MODEL_UNLOAD_ERROR,         "Model unload error" },
        { MODEL_MALFORMED,            "Model DLL malformed" },
        { MODEL_VERSION_ERROR,        "Incompatible model version" },
        { MODEL_UNSUPPORTED_DEVICE,   "Device not supported by this model" },
        { MODEL_LICENSE_ERROR,        "Model licensing error" },
        { MODEL_UNKNOWN_ERROR,        "Unknown error" },
    };
    struct Conv *p;
    for (p = convtab; p->c != MODEL_UNKNOWN_ERROR; p++)
        if (p->c == err.error_code) break;
    printf("Error: %s (%d)\n", p->s, err.error_code);
    if (err.error_code == MODEL_NO_ERROR) return;
    printf("Device: %s  Model: %s Product ID: %s Version: %s\n",
           err.device? err.device : "Unknown",
           err.dllname? err.dllname : "Unknown",
           err.product? err.product : "Unknown",
           err.version? err.version : "Unknown");
    printf("%s\n", err.errstring? err.errstring : "");
    if (err.error_code == MODEL_LICENSE_ERROR)
    {
        printf("License Path: %s\nLicense manager: %s\n",
               err.licpath? err.licpath : "Unknown",
               err.licmgr? err.licmgr : "Unknown");
    }
}

int atavr8_load_model(out_func of, rp_target* t)
{
    char buf[1000 + 1];
    char reply_buf[256];

    if(atavr8_status.device)
        sprintf(reply_buf, "Loading %s... ", atavr8_status.device);
    else
        sprintf(reply_buf, "Loading... ");
    rp_encode_string(reply_buf, buf, size_t(1000));
    of(t, buf);

    mM = mMgr->load(atavr8_status.lib, atavr8_status.device);

    if(mM)
    {
        mC = mM->getCore(atavr8_status.selectedCore);
        mC->addStepCallback(stepcallback, NULL);
        if(atavr8_status.device)
        {
            char name_buf[256];
            if(mM->getStringProperty(P_DeviceName, sizeof(name_buf), name_buf))
                atavr8_status.device = strcpy((char*)malloc(strlen(name_buf) + 1), (name_buf));
        }
        sprintf(reply_buf, "Successfully loaded %s\n", atavr8_status.device);
        rp_encode_string(reply_buf, buf, size_t(1000));
        of(t, buf);
    }
    else
    {
        rp_encode_string("FAILED! No device loaded.\n", buf, size_t(1000));
        of(t, buf);
    }
}
