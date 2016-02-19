/* Copyright (C) 1999-2001 Quality Quorum, Inc.
   Copyright (C) 2002 Chris Liechti and Steve Underwood
   2005 Martin Strubel (fixed pNN packet bug)

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

   QQI can be contacted as qqi@world.std.com


   Main remote proxy unit.

   Exported Data:
   None

   Imported Data:
   None

   Static Data:
   rp_t_list            - list of targets
   rp_debug_level       - debug flag
   rp_target_out_valid  - to help catch uunappropriate output 
   from target 
   rp_log               - pointer to a current log function

   Global Functions:  
   main        - main 

   Static Functions:
   rp_putpkt          - send packet to debugger
   rp_getpkt          - get packet from debugger
   rp_console_output  - send output to debugger console
   rp_data_output     - send data to debugger (used remcmd)
   rp_decode_xxxxx    - various decode functions
   rp_encode_xxxxx    - various encode functions
   rp_usage           - usage/help
   rp_write_xxxxx     - encode result of operation


   $Id: gdbproxy.c,v 1.12 2010/02/10 12:45:50 vapier Exp $ */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#if defined(WIN32)
#include <windows.h>
#endif

#include <pthread.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <stdarg.h>
#include <inttypes.h>

#ifdef  HAVE_GETOPT_LONG_ONLY
#ifndef HAVE_GETOPT_H
#error "configuration error: unexpected combination"
#endif /* HAVE_GETOPT_H */
#include <getopt.h>
#else
#include "getopt.h"
#endif /* HAVE_GETOPT_LONG_ONLY */

#include "gdbproxy.h"
#include "rpmisc.h"

/* Target list */
static rp_target *rp_t_list;

/* Debug flag */
int rp_debug_level = 0;
int optionIndex = 0;

/* Flag to catch unexpected output from target */


/* Current log */
static log_func rp_log = NULL;

/* Connection to debugger */
static int rp_putpkt(rp_target* t, const char *buf);
static int rp_getpkt(rp_target* t, char *buf, size_t buf_len, size_t *in_len, int timeout);
static void rp_console_output(rp_target* t, const char *buf);
static void rp_data_output(rp_target* t, const char *buf);

/* Decode/encode functions */
static int rp_decode_data(const char *in,
        unsigned char *out,
        size_t out_size,
        size_t *len);

static int rp_decode_reg(const char *in, unsigned int *reg_no);

static int rp_decode_reg_assignment(const char *in,
        unsigned int *reg_no,
        unsigned char *out,
        size_t out_size,
        size_t *len);
static int rp_decode_mem(const char *in,
        uint64_t *addr,
        size_t *len);
static int rp_decode_process_query(const char *in,
        unsigned int *mask,
        rp_thread_ref *ref);
static int rp_decode_break(const char *in,
        int *type,
        uint64_t *addr,
        unsigned int *len);
static int rp_decode_list_query(const char *in,
        int *first,
        size_t *max,
        rp_thread_ref *arg);
static int rp_encode_regs(const unsigned char *data,
        const unsigned char *avail,
        size_t data_len,
        char *out,
        size_t out_size);
static int rp_encode_data(const unsigned char *data,
        size_t data_len,
        char *out,
        size_t out_size);
static int rp_encode_process_query_response(unsigned int mask,
        const rp_thread_ref *ref,
        const rp_thread_info *info,
        char *out,
        size_t out_size);
static int rp_encode_list_query_response(size_t count,
        int done,
        const rp_thread_ref *arg,
        const rp_thread_ref *found,
        char *out,
        size_t out_size);
static int rp_decode_nibble(const char *in, unsigned int *nibble);
static int rp_decode_byte(const char *in, unsigned int *byte_ptr);
static int rp_decode_4bytes(const char *in, uint32_t *val);
static int rp_decode_8bytes(const char *in, uint64_t *val);
static int rp_decode_uint32(char const **in, uint32_t *val, char break_char);
static int rp_decode_uint64(char const **in, uint64_t *val, char break_char);
static void rp_encode_byte(unsigned int val, char *out);

/* Usage */
static void rp_usage(void);

/* Funcions to stuff output value */
static void rp_write_retval(int ret, char *buf);

static int can_restart;
static int doing_daemon;
static int extended_protocol;
static char *name;

static void handle_search_memory_command(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t);
static void handle_thread_commands(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t);
static void handle_read_registers_command(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t);
static void handle_write_registers_command(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t);
static void handle_read_single_register_command(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t);
static void handle_write_single_register_command(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t);
static void handle_read_memory_command(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t);
static void handle_write_memory_command(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t);
static void handle_running_commands(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        char *status_string,
        int status_string_len,
        rp_target *t);
static int handle_kill_command(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t);
static int handle_thread_alive_command(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t);
static int handle_restart_target_command(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t);
static void handle_detach_command(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t);
static void handle_query_command(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t);
static void handle_breakpoint_command(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t);
static int handle_rcmd_command(char *in_buf,
        out_func of,
        data_func df,
        rp_target *t);
static int rp_rcmd_help(int argc,
        char *argv[],
        out_func of,
        data_func df,
        rp_target *t);
static int rp_rcmd_set(int argc,
        char *argv[],
        out_func of,
        data_func df,
        rp_target *t);

/* Remote command */
#define RP_RCMD(name, hlp) {#name, rp_rcmd_##name, hlp}

/* Table entry definition */
typedef struct
{
    /* command name */
    const char *name;
    /* command function */
    int (*function) (int, char **, out_func, data_func, rp_target *);
    /* one line of help text */
    const char *help;
} RP_RCMD_TABLE;

static void handle_search_memory_command(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t)
{
    uint64_t addr;
    uint32_t pattern;
    uint32_t mask;
    const char *in;

    /* Format: taddr:PP,MM
       Search backwards starting at address addr for a match with the
       supplied pattern PP and mask MM. PP and MM are 4 bytes. addr
       must be at least 3 digits. */

    in = &in_buf[1];
    if (!rp_decode_uint64(&in, &addr, ':'))
    {
        rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
        return;
    }
    if (!rp_decode_uint32(&in, &pattern, ','))
    {
        rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
        return;
    }
    if (!rp_decode_uint32(&in, &mask, '\0'))
    {
        rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
        return;
    }
    rp_write_retval(RP_VAL_TARGETRET_NOSUPP, out_buf);
}

static void handle_thread_commands(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t)
{
    int ret;
    rp_thread_ref ref;
    const char *in;

    if (in_len == 1  ||  in_buf[2] == '-')
    {
        /* Either short or an obsolete form */
        return;
    }

    /* Set thread */
    switch (in_buf[1])
    {
        case 'c':
            in = &in_buf[2];
            if (!rp_decode_uint64(&in, &ref.val, '\0'))
            {
                rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
                break;
            }

            ret = t->set_ctrl_thread(&ref);
            rp_write_retval(ret, out_buf);
            break;
        case 'g':
            in = &in_buf[2];
            if (!rp_decode_uint64(&in, &ref.val, '\0'))
            {
                rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
                break;
            }

            ret = t->set_gen_thread(&ref);
            rp_write_retval(ret, out_buf);
            break;
        default:
            rp_log(RP_VAL_LOGLEVEL_ERR, "%s: Bad H command", name);
            break;
    }
}

static void handle_read_registers_command(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t)
{
    int ret;
    size_t len;
    unsigned char data_buf[RP_PARAM_DATABYTES_MAX];
    unsigned char avail_buf[RP_PARAM_DATABYTES_MAX];

    /* Get all registers. Format: 'g'. Note we do not do any
       data caching - all caching is done by the debugger */
    ret = t->read_registers(data_buf,
            avail_buf,
            sizeof(data_buf),
            &len);
    switch (ret)
    {
        case RP_VAL_TARGETRET_OK:
            assert(len <= RP_PARAM_DATABYTES_MAX);
            rp_encode_regs(data_buf,
                    avail_buf,
                    len,
                    out_buf,
                    out_buf_len);
            break;
        case RP_VAL_TARGETRET_ERR:
            rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
            break;
        default:
            /* This should not happen */
            assert(0);
            break;
    }
}

static void handle_write_registers_command(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t)
{
    int ret;
    size_t len;
    unsigned char data_buf[RP_PARAM_DATABYTES_MAX];

    /* Write all registers. Format: 'GXXXXXXXXXX' */
    ret = rp_decode_data(&in_buf[1], data_buf, sizeof(data_buf), &len);
    if (!ret)
    {
        rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
        return;
    }

    ret = t->write_registers(data_buf, len);
    rp_write_retval(ret, out_buf);
}

static void handle_read_single_register_command(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t)
{
    int ret;
    unsigned int reg_no;
    size_t len;
    unsigned char data_buf[RP_PARAM_DATABYTES_MAX];
    unsigned char avail_buf[RP_PARAM_DATABYTES_MAX];

    /* Get a single register. Format 'pNN' */
    ret = rp_decode_reg(&in_buf[1], &reg_no);
    if (!ret) {
        rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
        return;
    }

    ret = t->read_single_register(reg_no,
            data_buf,
            avail_buf,
            sizeof(data_buf),
            &len);
    switch (ret)
    {
        case RP_VAL_TARGETRET_OK:
            assert(len <= RP_PARAM_DATABYTES_MAX);
            rp_encode_regs(data_buf,
                    avail_buf,
                    len,
                    out_buf,
                    out_buf_len);
            break;
        case RP_VAL_TARGETRET_ERR:
            rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
            break;
            // handle targets non supporting single register read
        case RP_VAL_TARGETRET_NOSUPP:
            break;
        default:
            /* This should not happen */
            assert(0);
            break;
    }
}

static void handle_write_single_register_command(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t)
{
    int ret;
    unsigned int reg_no;
    size_t len;
    unsigned char data_buf[RP_PARAM_DATABYTES_MAX];

    /* Write a single register. Format: 'PNN=XXXXX' */
    ret = rp_decode_reg_assignment(&in_buf[1],
            &reg_no,
            data_buf,
            sizeof(data_buf),
            &len);
    if (!ret)
    {
        rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
        return;
    }
    //assert(len == 4);
    assert(len < RP_PARAM_DATABYTES_MAX);

    ret = t->write_single_register(reg_no, data_buf, len);
    rp_write_retval(ret, out_buf);
}

static void handle_read_memory_command(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t)
{
    int ret;
    size_t len;
    uint64_t addr;
    unsigned char data_buf[RP_PARAM_DATABYTES_MAX];

    /* Read memory format: 'mAA..A,LL..LL' */
    if (!(ret = rp_decode_mem(&in_buf[1], &addr, &len)))
    {
        rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
        return;
    }

    /* Limit it so buggy gdbs will not complain */
    if (len > ((RP_VAL_DBG_PBUFSIZ - 32)/2))
        len = (RP_VAL_DBG_PBUFSIZ - 32)/2;

    ret = t->read_mem(addr, data_buf, len, &len);
    switch (ret)
    {
        case RP_VAL_TARGETRET_OK:
            assert(len <= RP_PARAM_DATABYTES_MAX);
            rp_encode_data(data_buf, len, out_buf, out_buf_len);
            break;
        case RP_VAL_TARGETRET_ERR:
            rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
            break;
        default:
            /* This should not happen */
            assert(0);
            break;
    }
}

static void handle_write_memory_command(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t)
{
    int ret;
    char *cp;
    size_t len;
    size_t len1;
    uint64_t addr;
    unsigned char data_buf[RP_PARAM_DATABYTES_MAX];

    /* Write memory format: 'mAA..A,LL..LL:XX..XX' */
    if ((cp = strchr(&in_buf[1], ':')) == NULL)
    {
        rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
        return;
    }
    *cp = '\0';

    ret = rp_decode_mem(&in_buf[1], &addr, &len);
    if (!ret  ||  len > RP_PARAM_DATABYTES_MAX)
    {
        rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
        return;
    }

    ret = rp_decode_data(cp + 1, data_buf, sizeof(data_buf), &len1);
    if (!ret  ||  len != len1)
    {
        rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
        return;
    }

    ret = t->write_mem(addr, data_buf, len);
    rp_write_retval(ret, out_buf);
}

static void handle_running_commands(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        char *status_string,
        int status_string_len,
        rp_target *t)
{
    int step;
    uint32_t sig;
    int go;
    int more;
    int implemented;
    const char *addr_ptr;
    uint64_t addr;
    int ret;
    const char *in;

    /* 'w' go from address
     * 'W' go from address with signal
     * 's' step from address
     * 'S' step from address with signal
     * 'c' continue from address
     * 'C' continue from address with signal
     */

    step = (in_buf[0] == 'S'  ||  in_buf[0] == 's');
    go = (in_buf[0] == 'W'  ||  in_buf[0] == 'w');

    addr_ptr = NULL;

    if (in_buf[0] == 'C'  ||  in_buf[0] == 'S'  ||  in_buf[0] == 'W')
    {
        /*
         * Resume with signal.
         * Format Csig[;AA..AA], Ssig[;AA..AA], or Wsig[;AA..AA]
         */

        in = &in_buf[1];
        if (strchr(in, ';'))
        {
            if (!rp_decode_uint32(&in, &sig, ';'))
            {
                rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
                return;
            }
            addr_ptr = in;
        }
        else
        {
            if (!rp_decode_uint32(&in, &sig, '\0'))
            {
                rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
                return;
            }
        }
    }
    else
    {
        sig = RP_VAL_TARGETSIG_0;
        if (in_buf[1] != '\0')
            addr_ptr = &in_buf[1];
    }

    if (go)
    {
        ret = t->go_waiting(sig);
    }
    else if (addr_ptr)
    {
        if (!rp_decode_uint64(&addr_ptr, &addr, '\0'))
        {
            rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
            return;
        }
        ret = t->resume_from_addr(step, sig, addr);
    }
    else
    {
        ret = t->resume_from_current(step, sig);
    }


    if (ret != RP_VAL_TARGETRET_OK)
    {
        rp_write_retval(ret, out_buf);
        return;
    }

    /* Now we have to wait for the target */
    t->rp_target_running = TRUE;
    t->rp_target_out_valid = TRUE;

    /* Try a partial wait first */
    ret = t->wait_partial(TRUE,
            status_string,
            status_string_len,
            rp_console_output,
            &implemented,
            &more,
            t);
    if (ret != RP_VAL_TARGETRET_OK)
    {
        rp_write_retval(ret, out_buf);
        t->rp_target_out_valid = FALSE;
        t->rp_target_running = FALSE;
        return;
    }
    if (!implemented)
    {
        /* There is no pertial wait facility for this target, so use a
           blocking wait */
        ret = t->wait(status_string,
                status_string_len,
                rp_console_output,
                &implemented,
                t);

        assert(implemented);

        if (ret == RP_VAL_TARGETRET_OK)
        {
            assert(strlen(status_string) < status_string_len);
            strcpy(out_buf, status_string);
        }
        else
        {
            rp_write_retval(ret, out_buf);
        }
        t->rp_target_out_valid = FALSE;
        t->rp_target_running = FALSE;
        return;
    }
    if (!more)
    {
        /* We are done. The program has already stopped */
        assert(strlen(status_string) < status_string_len);
        strcpy(out_buf, status_string);
        t->rp_target_out_valid = FALSE;
        t->rp_target_running = FALSE;
    }
}

static int handle_kill_command(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t)
{
    int ret;

    t->kill();

    if (!extended_protocol)
    {
        dbg_sock_close(&(t->dbg_sock));

        if (!can_restart)
        {
            /* If the current target cannot restart, we have little choice but
               to exit right now. */
            rp_log(RP_VAL_LOGLEVEL_INFO,
                    "%s: session killed. Exiting",
                    name);
            dbg_sock_cleanup(&(t->dbg_sock));
            exit(0);
        }

        rp_log(RP_VAL_LOGLEVEL_INFO,
                "%s: session killed. Will wait for a new connection",
                name);
        return  FALSE;
    }

    rp_log(RP_VAL_LOGLEVEL_INFO,
            "%s: remote proxy restarting",
            name);

    /* Let us do our best while starting system */
    if (!can_restart)
    {
        /* Even if restart is not supported it is still worth calling connect */
        return  -1;
    }

    ret = t->restart();

    assert(ret != RP_VAL_TARGETRET_NOSUPP);

    if (ret != RP_VAL_TARGETRET_OK)
    {
        /* There is no point in continuing */
        rp_log(RP_VAL_LOGLEVEL_ERR,
                "%s: unable to restart target %s",
                name,
                t->name);
        rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
        rp_putpkt(t, out_buf);
        dbg_sock_close(&(t->dbg_sock));

        if (!can_restart)
        {
            dbg_sock_cleanup(&(t->dbg_sock));
            exit(1);
        }

        rp_log(RP_VAL_LOGLEVEL_INFO,
                "%s: will wait for a new connection",
                name);
        return  FALSE;
    }
    return  TRUE;
}

static int handle_thread_alive_command(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t)
{
    int ret;
    int alive;
    rp_thread_ref ref;
    const char *in;

    /* Is thread alive? */
    /* This is a deprecated feature of the remote debug protocol */
    in = &in_buf[1];
    if (!(ret = rp_decode_uint64(&in, &ref.val, '\0')))
    {
        rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
        return  TRUE;
    }

    ret = t->is_thread_alive(&ref, &alive);
    if (ret != RP_VAL_TARGETRET_OK)
    {
        rp_write_retval(ret, out_buf);
    }
    else
    {
        if (alive)
            rp_write_retval(RP_VAL_TARGETRET_OK, out_buf);
        else
            rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
    }
    return  TRUE;
}

static int handle_restart_target_command(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t)
{
    int ret;

    /* Restarting the target is only supported in the extended protocol. */
    if (!extended_protocol)
        return  FALSE;

    assert(can_restart);

    /* Let us do our best to restart the system */
    if ((ret = t->restart()) != RP_VAL_TARGETRET_OK)
    {
        /* There is no point to continuing */
        rp_log(RP_VAL_LOGLEVEL_ERR,
                "%s: unable to restart target %s",
                name,
                t->name);
        rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
        rp_putpkt(t, out_buf);
        dbg_sock_close(&(t->dbg_sock));

        if (!can_restart)
        {
            /* If the current target cannot restart, we have little choice but
               to exit right now. */
            rp_log(RP_VAL_LOGLEVEL_INFO,
                    "%s: target is not restartable. Exiting",
                    name);
            dbg_sock_cleanup(&(t->dbg_sock));
            exit(1);
        }

        rp_log(RP_VAL_LOGLEVEL_INFO,
                "%s: will wait for a new connection",
                name);
        return  -1;
    }
    return  TRUE;
}

static void handle_detach_command(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t)
{
    int  ret;

    ret = t->disconnect();

    /* Note: The current GDB does not expect a reply */
    rp_putpkt(t, out_buf);
    dbg_sock_close(&(t->dbg_sock));

    rp_log(RP_VAL_LOGLEVEL_INFO, "%s: debugger detached", name);

    if (!can_restart)
    {
        /* If the current target cannot restart, we have little choice but
           to exit right now. */
        rp_log(RP_VAL_LOGLEVEL_INFO,
                "%s: target is not restartable. Exiting",
                name);
        dbg_sock_cleanup(&(t->dbg_sock));
        exit(0);
    }

    rp_log(RP_VAL_LOGLEVEL_INFO,
            "%s: will wait for a new connection",
            name);
}

static void handle_query_command(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t)
{
    int  ret;
    rp_thread_ref ref;
    rp_thread_info info;
    unsigned int mask;
    rp_thread_ref arg;
    rp_thread_ref *found;
    size_t max_found;
    size_t count;
    int done;
    int first;
    unsigned int len;
    uint32_t val;
    uint64_t addr;
    const char *cp;

    if (in_len == 1)
    {
        rp_log(RP_VAL_LOGLEVEL_ERR,
                "%s: bad 'q' command received",
                name);
        return;
    }
    if (strncmp(in_buf + 1, "Offsets", 7) == 0)
    {
        uint64_t text;
        uint64_t data;
        uint64_t bss;

        /* Get the program segment offsets */
        ret = t->offsets_query(&text, &data, &bss);
        if (ret == RP_VAL_TARGETRET_OK)
        {
            sprintf(out_buf,
                    "Text=%"PRIu64"x;Data=%"PRIu64"x;Bss=%"PRIu64"x",
                    text,
                    data,
                    bss);
        }
        else
        {
            rp_write_retval(ret, out_buf);
        }
        return;
    }

    if (strncmp(in_buf + 1, "CRC:", 4) == 0)
    {
        /* Find the CRC32 value of the specified memory area */
        cp = &in_buf[5];
        if (!rp_decode_uint64(&cp, &addr, ','))
        {
            rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
            return;
        }

        if (!rp_decode_uint32(&cp, &len, '\0'))
        {
            rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
            return;
        }
        ret = t->crc_query(addr, len, &val);
        if (ret == RP_VAL_TARGETRET_OK)
            sprintf(out_buf, "C%x", val);
        else
            rp_write_retval(ret, out_buf);
        return;
    }

    if (strncmp(in_buf + 1, "Symbol:", 7) == 0)
    {
        rp_write_retval(RP_VAL_TARGETRET_NOSUPP, out_buf);
        return;
    }

    if (strncmp(in_buf + 1, "TStatus:", 8) == 0)
    {
        rp_write_retval(RP_VAL_TARGETRET_NOSUPP, out_buf);
        return;
    }

    if (strncmp(in_buf + 1, "ThreadExtraInfo,", 16) == 0)
    {
        char data_buf[RP_PARAM_DATABYTES_MAX];
        const char *in;

        if (t->threadextrainfo_query == NULL)
        {
            rp_write_retval(RP_VAL_TARGETRET_NOSUPP, out_buf);
            return;
        }

        in = &in_buf[17];
        if (!(ret = rp_decode_uint64(&in, &ref.val, '\0')))
        {
            rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
            return;
        }

        ret = t->threadextrainfo_query (&ref, data_buf, RP_PARAM_DATABYTES_MAX);
        switch (ret)
        {
            case RP_VAL_TARGETRET_OK:
                rp_encode_data ((unsigned char *)data_buf, strlen (data_buf),
                        out_buf, out_buf_len);
                break;
            case RP_VAL_TARGETRET_ERR:
            case RP_VAL_TARGETRET_NOSUPP:
                rp_write_retval (ret, out_buf);
                break;
            default:
                assert(0);
                break;
        }
        return;
    }

    if (strncmp(in_buf + 1, "fThreadInfo", 11) == 0)
    {
        if (t->threadinfo_query == NULL)
        {
            rp_write_retval (RP_VAL_TARGETRET_NOSUPP, out_buf);
            return;
        }

        ret = t->threadinfo_query (1, out_buf, out_buf_len);
        switch (ret)
        {
            case RP_VAL_TARGETRET_OK:
                break;
            case RP_VAL_TARGETRET_NOSUPP:
            case RP_VAL_TARGETRET_ERR:
                rp_write_retval(ret, out_buf);
                break;
            default:
                /* This should not happen */
                assert(0);
        }
        return;
    }

    if (strncmp(in_buf + 1, "sThreadInfo", 11) == 0)
    {
        if (t->threadinfo_query == NULL)
        {
            rp_write_retval(RP_VAL_TARGETRET_NOSUPP, out_buf);
            return;
        }

        ret = t->threadinfo_query(0, out_buf, out_buf_len);
        switch (ret)
        {
            case RP_VAL_TARGETRET_OK:
                break;
            case RP_VAL_TARGETRET_NOSUPP:
            case RP_VAL_TARGETRET_ERR:
                rp_write_retval(ret, out_buf);
                break;
            default:
                /* This should not happen */
                assert(0);
        }
        return;
    }

    if (strncmp(in_buf + 1, "fProcessInfo", 12) == 0)
    {
        /* Get first string of process info */
        rp_write_retval(RP_VAL_TARGETRET_NOSUPP, out_buf);
        return;
    }

    if (strncmp(in_buf + 1, "sProcessInfo", 12) == 0)
    {
        /* Get subsequent string of process info */
        rp_write_retval(RP_VAL_TARGETRET_NOSUPP, out_buf);
        return;
    }

    if (strncmp(in_buf + 1, "Rcmd,", 5) == 0)
    {
        /* Remote command */
        t->rp_target_out_valid = TRUE;
        ret = handle_rcmd_command(&in_buf[6],
                rp_console_output,
                rp_data_output,
                t);
        t->rp_target_out_valid = FALSE;
        rp_write_retval(ret, out_buf);
        return;
    }

    if (strncmp(in_buf + 1, "Supported", 9) == 0
            && (in_buf[10] == ':' || in_buf[10] == '\0'))
    {
        /* Features supported */
        if (t->packetsize_query == NULL)
        {
            sprintf(out_buf, "PacketSize=%x", RP_PARAM_INOUTBUF_SIZE);
            return;
        }

        ret = t->packetsize_query(out_buf, out_buf_len);
        switch (ret)
        {
            case RP_VAL_TARGETRET_OK:
                break;
            case RP_VAL_TARGETRET_NOSUPP:
            case RP_VAL_TARGETRET_ERR:
                rp_write_retval(ret, out_buf);
                break;
            default:
                /* This should not happen */
                assert(0);
        }
        return;
    }

    switch (in_buf[1])
    {
        case 'C':
            /* Current thread query */
            ret = t->current_thread_query(&ref);

            if (ret == RP_VAL_TARGETRET_OK)
                sprintf(out_buf, "QC%"PRIu64"x", ref.val);
            else
                rp_write_retval(ret, out_buf);
            break;
        case 'L':
            /* Thread list query */
            ret = rp_decode_list_query(&in_buf[2],
                    &first,
                    &max_found,
                    &arg);
            if (!ret  ||  max_found > 255)
            {
                rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
                break;
            }

            if ((found = malloc(max_found*sizeof(rp_thread_ref))) == NULL)
            {
                rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
                break;
            }

            ret = t->list_query(first,
                    &arg,
                    found,
                    max_found,
                    &count,
                    &done);
            if (ret != RP_VAL_TARGETRET_OK  ||  count > max_found)
            {
                free(found);
                rp_write_retval(ret, out_buf);
                break;
            }

            ret = rp_encode_list_query_response(count,
                    done,
                    &arg,
                    found,
                    out_buf,
                    out_buf_len);

            free(found);

            if (!ret)
                rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
            break;
        case 'P':
            /* Thread info query */
            if (!(ret = rp_decode_process_query(&in_buf[2], &mask, &ref)))
            {
                rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
                break;
            }

            info.thread_id.val = 0;
            info.display[0] = 0;
            info.thread_name[0] = 0;
            info.more_display[0] = 0;

            if ((ret = t->process_query(&mask, &ref, &info)) != RP_VAL_TARGETRET_OK)
            {
                rp_write_retval(ret, out_buf);
                break;
            }

            ret = rp_encode_process_query_response(mask,
                    &ref,
                    &info,
                    out_buf,
                    out_buf_len);
            if (!ret)
                rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
            break;
        default:
            /* Raw Query is a universal fallback */
            ret = t->raw_query(in_buf, out_buf, out_buf_len);
            if (ret != RP_VAL_TARGETRET_OK)
                rp_write_retval(ret, out_buf);
            break;
    }
}

static void handle_breakpoint_command(char * const in_buf,
        int in_len,
        char *out_buf,
        int out_buf_len,
        rp_target *t)
{
    uint64_t addr;
    unsigned int len;
    int type;
    int ret;

    if (!(ret = rp_decode_break(in_buf, &type, &addr, &len)))
    {
        rp_write_retval(RP_VAL_TARGETRET_ERR, out_buf);
        return;
    }

    if (in_buf[0] == 'Z')
        ret = t->add_break(type, addr, len);
    else
        ret = t->remove_break(type, addr, len);

    rp_write_retval(ret, out_buf);
}




static int target_thread_run(rp_target* t)
{

    // variables
    int input_error;
    int ret;
    int do_reinitialize;
    int do_connect;
    int more;
    int implemented;
    size_t in_len;


    /* Various buffers used by the system */
    static char in_buf[RP_PARAM_INOUTBUF_SIZE];
    static char out_buf[RP_PARAM_INOUTBUF_SIZE];
    static char status_string[RP_PARAM_INOUTBUF_SIZE];

    input_error = FALSE;
    do_reinitialize = TRUE;
    do_connect = TRUE;
    status_string[0] = '\0';
   

    // 
    t->rp_target_running = FALSE;
    t->rp_target_out_valid = FALSE;

    // the main loop, receive from and sent to gdb front-end
    for (;;)
    {
        if (input_error)
        {
            input_error = FALSE;
            /* We come here when an input error is discovered */
            rp_log(RP_VAL_LOGLEVEL_INFO,
                    "%s: debugger has terminated connection",
                    name);

            // t->close();
            dbg_sock_close(&(t->dbg_sock));

            /* Close connection and start again */
            rp_log(RP_VAL_LOGLEVEL_INFO, "%s: will reopen the connection", name);
            do_reinitialize = TRUE;
        }

        // Connect to the gdb front-end
        if (do_reinitialize)
        {
            do_reinitialize = FALSE;

            /* Initialize target */
            extended_protocol = FALSE;

            if (!(ret = dbg_sock_accept(&(t->dbg_sock), &(t->dbg_listen_sock))))
            {
                rp_log(RP_VAL_LOGLEVEL_ERR,
                        "%s: error while waiting for debugger connection. Will restart.",
                        name);
                dbg_sock_close(&(t->dbg_sock));
                do_reinitialize = TRUE;
            }
            else
            {
                rp_log(RP_VAL_LOGLEVEL_NOTICE, "%s: connected", name);
                do_connect = TRUE;
            }
        }

        // Connect to the gdb front-end, cont... 
        if (do_connect)
        {
            do_connect = FALSE;
            ret = t->connect(status_string, sizeof(status_string), &can_restart);
            if (ret != RP_VAL_TARGETRET_OK)
            {
                dbg_sock_close(&(t->dbg_sock));
                rp_log(RP_VAL_LOGLEVEL_ERR,
                        "%s: unable to connect to target %s. Will restart.",
                        name,
                        t->name);
                do_reinitialize = TRUE;
                continue;
            }
        }

        // check if the target is already running
        if (t->rp_target_running)
            ret = rp_getpkt(t, in_buf, sizeof(in_buf), &in_len, 100);
        else
            ret = rp_getpkt(t, in_buf, sizeof(in_buf), &in_len, -1);
        if (ret == -1)
        {
            /* Debugger closed connection, or connection is bad */
            input_error = TRUE;
            continue;
        }

        // STOP
        if (ret == '\3')
        {
            /* We got a control-C. */
            /* Only when the target has responded to the following stop command,
               will GDB receive a response. */
            if (t->rp_target_running)
                t->stop();
            continue;
        }

        // 
        if (ret == 1)
        {
            /* Timeout: if the target is running, check for input from it */
            if (t->rp_target_running)
            {
                ret = t->wait_partial(FALSE,
                        status_string,
                        sizeof(status_string),
                        rp_console_output,
                        &implemented,
                        &more,
                        t);
                assert(implemented);

                if (ret != RP_VAL_TARGETRET_OK  ||  !more)
                {
                    t->rp_target_running = FALSE;
                    if (ret == RP_VAL_TARGETRET_OK)
                    {
                        assert(strlen(status_string) < sizeof(status_string));
                        strcpy(out_buf, status_string);
                    }
                    else
                    {
                        rp_write_retval(ret, out_buf);
                    }
                    t->rp_target_out_valid = FALSE;
                    rp_putpkt(t, out_buf);
                }
            }
            continue;
        }

        if (ret != 0)
        {
            /* We got an ACK or something else that is not a packet. */
            continue;
        }

        assert(in_len == strlen(in_buf));

        /* If we cannot process this command, it is not supported */
        //jie Why do we need this here?
        rp_write_retval(RP_VAL_TARGETRET_NOSUPP, out_buf);
        t->rp_target_out_valid = FALSE;


        /* the main switch for different packet from gdb front-end */
        // here the packets are processed based their type, therefore
        // an additional layer handle_**** is implemented.
        //
        switch (in_buf[0])
        {
            case '!':
                /* Set extended operation */
                rp_log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: switching to extended protocol mode\n",
                        name);
                if (can_restart)
                {
                    extended_protocol = TRUE;
                    rp_write_retval(RP_VAL_TARGETRET_OK, out_buf);
                }
                else
                {
                    /* Some GDBs will accept any response as a good one. Let us
                       bark in the log at least */
                    rp_log(RP_VAL_LOGLEVEL_ERR,
                            "%s: extended operations required, but not supported",
                            name);
                }
                break;
            case '?':
                /* Report the last signal status */
                strcpy(out_buf, status_string);
                break;
            case 'A':
                /* Set the argv[] array of the target */
                //TODO: implement this!
                // This could be useful, so that gdb front-end could set
                // the device itself, instead of only one device is supported 
                // within one session.
                break;
            case 'C':
            case 'S':
            case 'W':
            case 'c':
            case 's':
            case 'w':
                handle_running_commands(in_buf,
                        in_len,
                        out_buf,
                        sizeof(out_buf),
                        status_string,
                        sizeof(status_string),
                        t);
                if (t->rp_target_running)
                    continue;
                break;
            case 'D':
                handle_detach_command(in_buf,
                        in_len,
                        out_buf,
                        sizeof(out_buf),
                        t);
                do_reinitialize = TRUE;
                continue;
            case 'g':
                handle_read_registers_command(in_buf,
                        in_len,
                        out_buf,
                        sizeof(out_buf),
                        t);
                break;
            case 'G':
                handle_write_registers_command(in_buf,
                        in_len,
                        out_buf,
                        sizeof(out_buf),
                        t);
                break;
            case 'H':
                handle_thread_commands(in_buf,
                        in_len,
                        out_buf,
                        sizeof(out_buf),
                        t);
                break;
            case 'k':
                do_connect = handle_kill_command(in_buf,
                        in_len,
                        out_buf,
                        sizeof(out_buf),
                        t);
                if (do_connect == -1)
                    input_error = TRUE;
                if (!do_connect)
                    do_reinitialize = TRUE;
                continue;
            case 'm':
                handle_read_memory_command(in_buf,
                        in_len,
                        out_buf,
                        sizeof(out_buf),
                        t);
                break;
            case 'M':
                handle_write_memory_command(in_buf,
                        in_len,
                        out_buf,
                        sizeof(out_buf),
                        t);
                break;
            case 'p':
                handle_read_single_register_command(in_buf,
                        in_len,
                        out_buf,
                        sizeof(out_buf),
                        t);
                break;
            case 'P':
                handle_write_single_register_command(in_buf,
                        in_len,
                        out_buf,
                        sizeof(out_buf),
                        t);
                break;
            case 'q':
                handle_query_command(in_buf,
                        in_len,
                        out_buf,
                        sizeof(out_buf),
                        t);
                break;
            case 'R':
                do_connect = handle_restart_target_command(in_buf,
                        in_len,
                        out_buf,
                        sizeof(out_buf),
                        t);
                if (do_connect == -1)
                    do_reinitialize = TRUE;
                if (do_connect)
                    continue;
                break;
            case 't':
                handle_search_memory_command(in_buf,
                        in_len,
                        out_buf,
                        sizeof(out_buf),
                        t);
                break;
            case 'T':
                handle_thread_alive_command(in_buf,
                        in_len,
                        out_buf,
                        sizeof(out_buf),
                        t);
                break;
            case 'Z':
            case 'z':
                handle_breakpoint_command(in_buf,
                        in_len,
                        out_buf,
                        sizeof(out_buf),
                        t);
                break;
            default:
                break;
        }

        if (!input_error)
            rp_putpkt(t, out_buf);
    }

}




int main (int argc, char **argv)
{
    rp_target *t;
    int ret;
    int port;
    int doing_help;
    int t_argc;
    char **t_argv;
    int quiet;

    // threading
    pthread_t thread;
    void* rval;

    /* Option descriptors */
    static struct option long_options[] =
    {
        {"help",     0, 0, 1},
#ifndef WIN32
        {"daemon",   0, 0, 2},
#endif /* WIN32 */
        {"debug",    0, 0, 3},
        {"port",     1, 0, 4},
        {"warranty", 0, 0, 5},
        {"copying",  0, 0, 6},
        {"version",  0, 0, 7},
        {"quiet",    0, 0, 8},
        {NULL,  0, 0, 0}
    };


#ifdef WIN32
#define PATHSEP '\\'
#else
#define PATHSEP '/'
#endif
    /* Chop path off argv[0] */
    name = argv[0];
    for (;;)
    {
        char *cp;

        cp = strchr(name, PATHSEP);
        if (cp == NULL)
            break;
        name = cp + 1;
    }

    /* Initialize everything */
    rp_t_list = rp_init();
    port = 0;
    quiet = 0;
    doing_help   = FALSE;
    doing_daemon = FALSE;
    rp_debug_level = 0;

    /* Process options */
    for (;;)
    {
        int c;

        c = getopt_long_only(argc, argv, "+", long_options, &optionIndex);

        if (c == EOF)
            break;

        switch (c)
        {
            case 1:
                /* help */
                doing_help = TRUE;
                break;
#ifndef WIN32
            case 2:
                /* daemon */
                doing_daemon = TRUE;
                break;
#endif /* WIN32 */
            case 3:
                /* debug */
                rp_debug_level++;
                break;
            case 4:
                /* port */
                port = atoi(optarg);
                if (port == 0  ||  port > 0xFFFF)
                {
                    printf("%s: bad port %s\n", name, optarg);
                    exit(1);
                }
                break;
            case 5:
                /* warranty */
                rp_show_warranty();
                return 0;
            case 6:
                /* copying */
                rp_show_copying();
                return 0;
            case 7:
                /* version */
                printf("Remote proxy for GDB, version %s\n\n", VERSION);
                return 0;
            case 8:
                /* quiet */
                ++quiet;
                break;
            default:
                printf("Use `%s --help' to see a complete list of options\n",
                        argv[0]);
                return 1;
        }
    }

    if (optind >= argc)
    {
        rp_usage();
        exit(1);
    }

    if (!quiet)
    {
        printf("\nRemote proxy for GDB, v%s, Copyright (C) 1999 Quality Quorum Inc.\n",
                VERSION);
        printf("MSP430 adaption Copyright (C) 2002 Chris Liechti and Steve Underwood\n");
        printf("Blackfin adaption Copyright (C) 2008-2010 Analog Devices, Inc.\n");
        printf("Atmel simulator adaption Copyright (C) 2016 Atmel, Inc.\n\n");
        printf("GDBproxy comes with ABSOLUTELY NO WARRANTY; for details\n");
        printf("use `--warranty' option. This is Open Source software. You are\n");
        printf("welcome to redistribute it under certain conditions. Use the\n");
        printf("'--copying' option for details.\n\n");
    }


    t_argc = argc - optind; 
    t_argv = argv;

    // loops until all the options are processed
    while (t_argc > 0)
    {
        /* Save target argc and argv. We have to do this because optind may
           be reset in t->open */
        t_argv = t_argv + optind;

        printf("t_argv --> %s\n", *t_argv);


        /* Find the target */
        for (t = rp_t_list;  t;  t = t->next)
        {
            assert(t->name != NULL);
            if (strcmp(t->name, t_argv[0]) == 0)
                break;
        }
        if (t == NULL)
        {
            printf("Target %s not present,\n", argv[optind]);
            printf("use '%s --help' to see a complete list of targets\n", name);
            exit(1);
        }


        dbg_sock_init(&(t->dbg_listen_sock));
        /* Ensure the target is valid */
        assert(t->help != NULL);
        assert(t->open != NULL);
        assert(t->close != NULL);
        assert(t->connect != NULL);
        assert(t->disconnect != NULL);
        assert(t->kill != NULL);
        assert(t->restart != NULL);
        assert(t->stop != NULL);
        assert(t->set_gen_thread != NULL);
        assert(t->set_ctrl_thread != NULL);
        assert(t->is_thread_alive != NULL);
        assert(t->read_registers != NULL);
        assert(t->write_registers != NULL);
        assert(t->write_single_register != NULL);
        assert(t->read_mem != NULL);
        assert(t->write_mem != NULL);
        assert(t->resume_from_addr != NULL);
        assert(t->resume_from_current != NULL);
        assert(t->go_waiting != NULL);
        assert(t->wait_partial != NULL);
        assert(t->wait != NULL);
        assert(t->process_query != NULL);
        assert(t->list_query != NULL);
        assert(t->current_thread_query != NULL);
        assert(t->offsets_query != NULL);
        assert(t->raw_query != NULL);
        assert(t->add_break != NULL);
        assert(t->remove_break != NULL);

        if (doing_help)
        {
            assert(t->help != NULL);
            t->help(name);
            exit(0);
        }

                if ((rp_log = rp_env_init(name, doing_daemon)) == NULL)
        {
            printf("%s: fatal: unable to initialize environment\n", name);
            exit(1);
        }

        can_restart = TRUE;

        // Connect to the target
        ret = t->open(t_argc, t_argv, name, rp_log);
        t_argc = t_argc - optind;

        // process based on the connect result
        switch (ret)
        {
            case RP_VAL_TARGETRET_OK:
                break;
            case RP_VAL_TARGETRET_NOSUPP:
                /* This target does not support repeated opens. */
                exit(0);
            case RP_VAL_TARGETRET_ERR:
                rp_log(RP_VAL_LOGLEVEL_ERR,
                        "%s: unable to open target %s",
                        name,
                        t->name);
                exit(1);
            default:
                assert(0);
                exit(1);
        }

        // setup the socket
        if (port != 0) port += 1;  // increase port number
        if (!(ret = dbg_listen_sock_open(&(t->dbg_sock), &(t->dbg_listen_sock), &port)))
        {
            rp_log(RP_VAL_LOGLEVEL_ERR,
                    "%s: unable to open debugger connection.",
                    name);
            exit(1);
        }
        else
            rp_log(RP_VAL_LOGLEVEL_NOTICE,
                    "%s: waiting on TCP port %d",
                    name,
                    port);

        // create thread for each target
        if ((ret = pthread_create(&thread, NULL, target_thread_run, (void*)t)))
        {
            rp_log(RP_VAL_LOGLEVEL_ERR,
                    "%s: unable to start thread for target.",
                    "ARM");
            exit(1);

        }
    }

    if ((ret = pthread_join(thread, &rval)))
    {
        rp_log(RP_VAL_LOGLEVEL_ERR,
                "%s: unable to join thread for target.",
                "ARM");
        exit(1);

    }

    return 0;
}





/* Send packet to debugger */
static int rp_putpkt(rp_target* t, const char *buf)
{
    int i;
    int ret;
    size_t len;
    size_t dummy_len;
    uint8_t csum;
    uint8_t *d;
    const char *s;
    uint8_t buf2[RP_PARAM_INOUTBUF_SIZE + 4];
    char in_buf[RP_PARAM_INOUTBUF_SIZE];
    static const char hex[] = "0123456789abcdef";

    assert(buf != NULL);

    /* Copy the packet into buf2, encapsulate it, and give
       it a checksum. */

    d = buf2;
    *d++ = '$';

    csum = 0;
    for (s = buf, i = 0;   *s  &&  i < RP_PARAM_INOUTBUF_SIZE;  i++)
    {
        csum += *s;
        *d++ = *s++;
    }

    assert(*s == '\0');

    /* Add the sumcheck to the end of the message */
    *d++ = '#';
    *d++ = hex[(csum >> 4) & 0xf];
    *d++ = hex[(csum & 0xf)];

    *d = '\0';

    /* Send it over and over until we get a positive ack. */
    len = d - buf2;

    /* Bodge alert: flush any packets in the incoming buffer */
    do
        ret = rp_getpkt(t, in_buf, sizeof(in_buf), &dummy_len, 0);
    while (ret != 1 && ret != -1);

    if (ret == -1)
        return FALSE;

    for (;;)
    {
        rp_log(RP_VAL_LOGLEVEL_DEBUG2,
                "%s: sending packet: %d bytes: %s...",
                name,
                len,
                buf2);

        if ((ret = dbg_sock_write(&(t->dbg_sock), &(t->dbg_listen_sock), buf2, len)) == 0)
        {
            /* Something went wrong */
            rp_log(RP_VAL_LOGLEVEL_DEBUG, "%s: write failed", name);
            return 0;
        }

        /* Now look for an ACK from GDB */
        if ((ret = rp_getpkt(t, in_buf, sizeof(in_buf), &dummy_len, -1)) == -1)
        {
            rp_log(RP_VAL_LOGLEVEL_DEBUG, "%s: read of ACK failed", name);
            return  FALSE;
        }
        if (ret == ACK)
        {
            rp_log(RP_VAL_LOGLEVEL_DEBUG2, "%s: got ACK", name);
            return  TRUE;
        }
    }
    /* We can't actually reach this point */
    return  FALSE;
}

/* Read a packet from the remote machine, with error checking,
   and store it in buf. */
static int rp_getpkt(rp_target* t, char *buf, size_t buf_len, size_t *len, int timeout)
{
    char seq[2];
    char seq_valid;
    unsigned char rx_csum;
    unsigned char calc_csum;
    int c;
    int nib;
    size_t pkt_len;
    int esc_found;
    int state;
    static const char hex[] = "0123456789abcdef";

    assert(buf != NULL);
    assert(buf_len > 6);
    assert(len != NULL);

    seq[0] = 0;
    seq[1] = 0;
    seq_valid = FALSE;
    rx_csum = 0;
    calc_csum = 0;
    pkt_len = 0;
    state = 0;
    esc_found = FALSE;
    for (;;)
    {
        if (state == 0)
            c = dbg_sock_readchar(&(t->dbg_sock), &(t->dbg_listen_sock), timeout);
        else
            c = dbg_sock_readchar(&(t->dbg_sock), &(t->dbg_listen_sock), -1);
        if (c == RP_VAL_MISCREADCHARRET_ERR)
        {
            rp_log(RP_VAL_LOGLEVEL_DEBUG, "%s: error while reading from GDB", name);
            return  -1;
        }
        if (c == RP_VAL_MISCREADCHARRET_TMOUT)
        {
            rp_log(RP_VAL_LOGLEVEL_DEBUG2, "%s: timeout while reading from GDB", name);
            return  1;
        }

        if (c == '$'  &&  state != 0)
        {
            /* Unexpected start of packet marker in mid-packet. */
            rp_log(RP_VAL_LOGLEVEL_DEBUG, "%s: unexpected new packet", name);
            seq[0] = 0;
            seq[1] = 0;
            seq_valid = FALSE;
            rx_csum = 0;
            calc_csum = 0;
            pkt_len = 0;
            state = 1;
        }

        switch (state)
        {
            case 0:
                /* Waiting for a start of packet marker */
                if (c == '$')
                {
                    /* Start of packet */
                    seq[0] = 0;
                    seq[1] = 0;
                    seq_valid = FALSE;
                    rx_csum = 0;
                    calc_csum = 0;
                    pkt_len = 0;
                    state = 1;
                }
                else if (c == '\3')
                {
                    /* A control C */
                    rp_log(RP_VAL_LOGLEVEL_DEBUG, "%s: Control-C received", name);
                    return  '\3';
                }
                else if (c == '+')
                {
                    /* An ACK to one of our packets */
                    /* We don't use sequence numbers, so we shouldn't expect a
                       sequence number after this character. */
                    rp_log(RP_VAL_LOGLEVEL_DEBUG2, "%s: ACK received", name);
                    return  ACK;
                }
                else if (c == '-')
                {
                    /* A NAK to one of our packets */
                    /* We don't use sequence numbers, so we shouldn't expect a
                       sequence number after this character. */
                    rp_log(RP_VAL_LOGLEVEL_DEBUG2, "%s: NAK received", name);
                    return  NAK;
                }
                else
                {
                    rp_log(RP_VAL_LOGLEVEL_DEBUG, "%s: we got junk - 0x%X", name, c & 0xFF);
                }
                break;
            case 1:
            case 2:
                /* We might be in the two character sequence number
                   preceeding a ':'. Then again, we might not! */
                if (c == '#')
                {
                    state = 8;
                    break;
                }
                buf[pkt_len++] = c;
                rx_csum += c;
                state++;
                break;
            case 3:
                if (c == '#')
                {
                    state = 8;
                    break;
                }
                if (c == ':')
                {
                    /* A ':' at this position means the previous 2
                       characters form a sequence number for the
                       packet. This must be saved, and used when
                       ack'ing the packet */
                    seq[0] = buf[0];
                    seq[1] = buf[1];
                    seq_valid = TRUE;
                    pkt_len = 0;
                }
                else
                {
                    buf[pkt_len++] = c;
                    rx_csum += c;
                }
                state = 4;
                break;
            case 4:
                if (c == '#')
                {
                    state = 8;
                    break;
                }
                buf[pkt_len++] = c;
                rx_csum += c;
                if (buf[0] == 'X')
                {
                    /* Special case: binary data. Format X<addr>,<len>:<data>.
Note: we have not reached the ':' yet. */
                    /* Translate this packet, so it looks like a non-binary
                       format memory write command. */
                    buf[0] = 'M';
                    esc_found = FALSE;
                    buf_len--; /* We have to save extra space */
                    state = 6;
                }
                else
                {
                    state = 5;
                }
                break;
            case 5:
                /* Normal, non-binary mode */
                if (c == '#')
                {
                    state = 8;
                    break;
                }
                if (pkt_len >= buf_len)
                {
                    rp_log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: received excessive length packet",
                            name);
                    continue;
                }
                buf[pkt_len++] = c;
                rx_csum += c;
                break;
            case 6:
                /* Escaped binary data mode - pre ':' */
                buf[pkt_len++] = c;
                rx_csum += c;
                if (c == ':')
                {
                    /* From now on the packet will be in escaped binary. */
                    state = 7;
                    continue;
                }
                break;
            case 7:
                /* Escaped binary data mode - post ':' */
                if (pkt_len >= buf_len)
                {
                    rp_log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: received a packet that is too long",
                            name);
                    continue;
                }
                if (esc_found)
                {
                    rx_csum += c;
                    esc_found = FALSE;
                    c ^= 0x20;
                    buf[pkt_len++] = hex[(c >> 4) & 0xf];
                    buf[pkt_len++] = hex[c & 0xf];
                    continue;
                }

                if (c == 0x7D)
                {
                    rx_csum += c;
                    esc_found = TRUE;
                    continue;
                }

                if (c == '#')
                {
                    /* Unescaped '#' means end of packet */
                    state = 8;
                    break;
                }

                rx_csum += c;
                buf[pkt_len++] = hex[(c >> 4) & 0xf];
                buf[pkt_len++] = hex[c & 0xf];
                break;
            case 8:
                /* Now get the first byte of the two byte checksum */
                if ((nib = rp_hex_nibble(c)) < 0)
                {
                    rp_log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: bad checksum character %c",
                            name,
                            c);
                    state = 0;
                    break;
                }
                calc_csum = (calc_csum << 4) | nib;
                state = 9;
                break;
            case 9:
                /* Now get the second byte of the checksum, and
                   check it. */
                if ((nib = rp_hex_nibble(c)) < 0)
                {
                    rp_log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: bad checksum character %c",
                            name,
                            c);
                    state = 0;
                    break;
                }
                calc_csum = (calc_csum << 4) | nib;
                if (rx_csum == calc_csum)
                {
                    buf[pkt_len] = '\0';
                    *len = pkt_len;

                    /* Acknowledge this good packet */
                    dbg_sock_putchar(&(t->dbg_sock), &(t->dbg_listen_sock), '+');
                    if (seq_valid)
                    {
                        dbg_sock_putchar(&(t->dbg_sock), &(t->dbg_listen_sock), seq[0]);
                        dbg_sock_putchar(&(t->dbg_sock), &(t->dbg_listen_sock), seq[1]);
                    }

                    rp_log(RP_VAL_LOGLEVEL_DEBUG2, "%s: packet received: %s", name, buf);
                    return  0;
                }
                rp_log(RP_VAL_LOGLEVEL_DEBUG,
                        "%s: bad checksum calculated=0x%x received=0x%x",
                        name,
                        rx_csum,
                        calc_csum);
                state = 0;
                continue;
        }
    }
    /* We can't actually get here */
    return  -1;
}

/* Send an 'O' packet (console output) to GDB */
static void rp_console_output(rp_target* t, const char *s)
{
    int ret;
    char *d;
    size_t count;
    size_t lim;
    static char buf[RP_VAL_DBG_PBUFSIZ - 6];

#if RP_VAL_DBG_PBUFSIZ < 10
#error "Unexpected value of RP_VAL_DBG_PBUFSIZ"
#endif /* RP_VAL_DBG_PBUFSIZ < 10 */

    if (!t->rp_target_out_valid)
    {
        rp_log(RP_VAL_LOGLEVEL_DEBUG,
                "%s: unexpected output from target: %s",
                name,
                s);
        return;
    }

    lim = sizeof(buf) - 1;
    if ((lim & 1) == 0)
    {
        /* We can split on any byte boundary */
        lim--;
    }

    do
    {
        d = buf;
        *d++ = 'O';
        for (count = 1;  *s  &&  count < lim;  s++, d++, count++)
            *d = *s;
        *d = '\0';
        ret = rp_putpkt(t, buf);
    }
    while (*s  &&  ret);
}

/* Send hex data to GDB */
static void rp_data_output(rp_target* t, const char *s)
{
    int ret;
    char *d;
    size_t count;
    size_t lim;
    static char buf[RP_VAL_DBG_PBUFSIZ - 6];

#if RP_VAL_DBG_PBUFSIZ < 10
#error "Unexpected value for RP_VAL_DBG_PBUFSIZ"
#endif /* RP_VAL_DBG_PBUFSIZ < 10 */

    if (!t->rp_target_out_valid)
    {
        rp_log(RP_VAL_LOGLEVEL_DEBUG,
                "%s: unexpected output from target: %s",
                name,
                s);
        return;
    }

    lim = sizeof(buf) - 1;

    if ((lim & 1))
    {
        /* We can split on any byte boundary */
        lim--;
    }

    do
    {
        for (d = buf, count = 0;  *s != 0  &&  count < lim;  s++, d++, count++)
            *d = *s;
        *d = '\0';
        ret = rp_putpkt(t, buf);
    }
    while (*s  &&  ret);
}

/* Convert stream of chars into data */
static int rp_decode_data(const char *in,
        unsigned char *out,
        size_t out_size,
        size_t *len)
{
    size_t count;
    unsigned int bytex;

    assert(in != NULL);
    assert(out != NULL);
    assert(out_size > 0);
    assert(len != NULL);

    for (count = 0;  *in  &&  count < out_size;  count++, in += 2, out++)
    {
        if (*(in + 1) == '\0')
        {
            /* Odd number of nibbles. Discard the last one */
            rp_log(RP_VAL_LOGLEVEL_WARNING, "%s: odd number of nibbles", name);
            if (count == 0)
                return  FALSE;
            *len = count;
            return  TRUE;
        }

        if (!rp_decode_byte(in, &bytex))
            return  FALSE;

        *out = bytex & 0xff;
    }

    if (*in)
    {
        /* Input too long */
        return  FALSE;
    }

    *len = count;

    return  TRUE;
}

static int rp_decode_reg(const char *in, unsigned int *reg_no)
{
    if (!rp_decode_uint32(&in, reg_no, '\0'))
        return  FALSE;

    return  TRUE;
}

/* Decode reg_no=XXXXXX */
static int rp_decode_reg_assignment(const char *in,
        unsigned int *reg_no,
        unsigned char *out,
        size_t out_size,
        size_t *out_len)
{
    assert(in != NULL);
    assert(reg_no != NULL);
    assert(out != NULL);
    assert(out_size > 0);
    assert(out_len != NULL);

    if (!rp_decode_uint32(&in, reg_no, '='))
        return  FALSE;

    out_size -= 8;

    return rp_decode_data(in, out, out_size - 1, out_len);
}

/* Decode memory transfer parameter in the form of AA..A,LL..L */
static int rp_decode_mem(const char *in, uint64_t *addr, size_t *len)
{
    assert(in != NULL);
    assert(addr != NULL);
    assert(len != 0);
    if (!rp_decode_uint64(&in, addr, ','))
        return  FALSE;

    *len = 0;
    return  rp_decode_uint32(&in, (uint32_t*) len, '\0');
}

/* Decode process query. Format: 'MMMMMMMMRRRRRRRRRRRRRRRR'
where: 
M represents mask
R represents thread reference */
static int rp_decode_process_query(const char *in,
        unsigned int *mask,
        rp_thread_ref *ref)
{
    unsigned int tmp_mask;
    uint64_t tmp_val;

    assert(in != NULL);
    assert(mask != NULL);
    assert(ref != NULL);

    if (!rp_decode_4bytes(in, &tmp_mask))
        return  FALSE;
    in += 8;

    if (!rp_decode_8bytes(in, &tmp_val))
        return  FALSE;

    *mask = tmp_mask;
    ref->val = tmp_val;

    return  TRUE;
}

/* Decode thread list list query. Format 'FMMAAAAAAAAAAAAAAAA'
where:
F represents first flag
M represents max count
A represents argument thread reference */
static int rp_decode_list_query(const char *in,
        int *first,
        size_t *max,
        rp_thread_ref *arg)
{
    unsigned int first_flag;
    unsigned int tmp_max;
    uint64_t tmp_val;

    assert(in != NULL);
    assert(first != NULL);
    assert(max != NULL);
    assert(arg != NULL);

    if (!rp_decode_nibble(in, &first_flag))
        return  FALSE;
    in++;

    if (!rp_decode_byte(in, &tmp_max))
        return  FALSE;
    in += 2;

    if (!rp_decode_8bytes(in, &tmp_val))
        return  FALSE;

    *first = (first_flag)  ?  TRUE  :  FALSE;
    *max = (size_t) tmp_max;
    arg->val = tmp_val;

    return  TRUE;
}

/* Decode a breakpoint (z or Z) packet */
static int rp_decode_break(const char *in,
        int *type,
        uint64_t *addr,
        unsigned int *len)
{
    unsigned int val;

    assert(in != NULL);
    assert(*in != '\0');
    assert(type != NULL);
    assert(addr != NULL);
    assert(len != NULL);

    in++;
    if (!rp_decode_nibble(in, &val))
        return  FALSE;
    in++;

    if (val < RP_VAL_BREAKTYPE_MIN  ||  val > RP_VAL_BREAKTYPE_MAX)
        return  FALSE;

    if (*in++ != ',')
        return  FALSE;

    *type = val;

    if (!rp_decode_uint64(&in, addr, ','))
        return  FALSE;

    if (!rp_decode_uint32(&in, len, '\0'))
        return  FALSE;

    return  TRUE;
}


/* If a byte of avail is 0 then the corresponding data byte is
   encoded as 'xx', otherwise it is encoded in normal way */
static int rp_encode_regs(const unsigned char *data,
        const unsigned char *avail,
        size_t data_len,
        char *out,
        size_t out_size)
{
    size_t i;

    assert(data != NULL);
    assert(avail != NULL);
    assert(data_len > 0);
    assert(out != NULL);
    assert(out_size > 0);

    if ((data_len*2) >= out_size)
    {
        /* We do not have enough space to encode the data */
        return  FALSE;
    }

    for (i = 0;  i < data_len;  i++, data++, avail++, out += 2)
    {
        if (*avail)
        {
            rp_encode_byte(*data, out);
        }
        else
        {
            *out = 'x';
            *(out + 1) = 'x';
        }
    }

    *out = 0;

    return  TRUE;
}

/* Convert an array of bytes into an array of characters */
static int rp_encode_data(const unsigned char *data,
        size_t data_len,
        char *out,
        size_t out_size)
{
    size_t i;

    assert(data != NULL);
    assert(data_len > 0);
    assert(out != NULL);
    assert(out_size > 0);

    if ((data_len*2) >= out_size)
    {
        /* We do not have enough space to encode the data */
        return  FALSE;
    }

    for (i = 0;  i < data_len;  i++, data++, out += 2)
        rp_encode_byte(*data, out);

    *out = 0;

    return  TRUE;
}

/* Encode string into an array of characters */
int rp_encode_string(const char *s, char *out, size_t out_size)
{
    int i;
    static const char hex[] = "0123456789abcdef";

    assert(s != NULL);
    assert(out != NULL);
    assert(out_size > 0);

    if (strlen (s) * 2 >= out_size)
    {
        /* We do not have enough space to encode the data */
        return  FALSE;
    }

    i = 0;
    while (*s)
    {
        *out++ = hex[(*s >> 4) & 0x0f];
        *out++ = hex[*s & 0x0f];
        s++;
        i++;
    }
    *out = '\0';
    return i;
}

/* Encode result of process query:
   qQMMMMMMMMRRRRRRRRRRRRRRRR(TTTTTTTTLLVV..V)*,
   where 
   M   represents mask
   R   represents ref
   T   represents tag
   L   represents length
   V   represents value */
static int rp_encode_process_query_response(unsigned int mask,
        const rp_thread_ref *ref,
        const rp_thread_info *info,
        char *out,
        size_t out_size)
{
    size_t len;
    unsigned int tag;
    int i;

    assert(ref != NULL);
    assert(info != NULL);
    assert(out != NULL);
    assert(out_size > 0);

    /* In all cases we will have at least mask and reference thread */
    if (out_size <= 26)
        return 0;

    /* Encode header */
    *out++ = 'q';
    *out++ = 'Q';
    out_size -= 2;

    /* Encode mask */
    sprintf(out, "%08x", mask);
    out += 8;
    out_size -= 8;

    /* Encode reference thread */
    sprintf(out, "%016"PRIu64"x", ref->val);

    out += 16;
    out_size -= 16;

    for (i = 0, tag = 0;  i < 32;  i++, tag <<= 1)
    {
        if ((mask & tag) == 0)
            continue;

        if (out_size <= 10)
        {
            /* We have no place to put even tag and length */
            return 0;
        }

        /* Encode tag */
        sprintf(out, "%08x", tag);
        out += 8;
        out_size -= 8;

        switch (tag)
        {
            case RP_BIT_PROCQMASK_THREADID:
                if (out_size <= 18)
                    return 0;

                /* Encode length - it is 16 */
                rp_encode_byte(16, out);
                out += 2;
                out_size -= 2;

                /* Encode value */
                sprintf(out, "%016"PRIu64"x", info->thread_id.val);

                out += 16;
                out_size -= 16;
                break;
            case RP_BIT_PROCQMASK_EXISTS:
                /* One nibble is enough */
                if (out_size <= 3)
                    return 0;

                /* Encode Length */
                rp_encode_byte(1, out);
                out += 2;
                out_size -= 2;

                /* Encode value */
                *out++    = (info->exists) ? '1' : '0';
                out_size-- ;
                *out      = 0;
                break;
            case RP_BIT_PROCQMASK_DISPLAY:
                /* Encode length */
                len = strlen(info->display);
                assert(len <= 255);

                if (out_size <= (len + 2))
                    return 0;

                rp_encode_byte(len, out);
                out += 2;
                out_size -= 2;

                /* Encode value */
                strcpy(out, info->display);
                out      += len;
                out_size -= len;
                break;
            case RP_BIT_PROCQMASK_THREADNAME:
                /* Encode length */
                len = strlen(info->thread_name);
                assert(len <= 255);

                if (out_size <= (len + 2))
                    return 0;

                rp_encode_byte(len, out);
                out += 2;
                out_size -= 2;

                /* Encode value */
                strcpy(out, info->thread_name);
                out      += len;
                out_size -= len;
                break;
            case RP_BIT_PROCQMASK_MOREDISPLAY:
                /* Encode length */
                len = strlen(info->more_display);
                assert(len <= 255);

                if (out_size <= (len + 2))
                    return 0;

                rp_encode_byte(len, out);
                out += 2;
                out_size -= 2;

                /* Encode value */
                strcpy(out, info->more_display);
                out += len;
                out_size -= len;
                break;
            default:
                /* Unexpected tag value */
                assert(0);
                return 0;
        }
    }

    return 1;
}

/* Encode result of list query:
   qMCCDAAAAAAAAAAAAAAAA(FFFFFFFFFFFFFFFF)*,
   where 
   C   reprsents  count
   D   represents done
   A   represents arg thread reference
   F   represents found thread reference(s) */
static int rp_encode_list_query_response(size_t count,
        int done,
        const rp_thread_ref *arg,
        const rp_thread_ref *found,
        char *out,
        size_t out_size)
{
    size_t i;

    assert(arg != NULL);
    assert(found != NULL  ||  count == 0);
    assert(count <= 255);

    /* Encode header, count, done and arg */
    if (out_size <= 21)
        return  FALSE;

    *out++ = 'q';
    *out++ = 'M';
    out_size -= 2;

    rp_encode_byte(count, out);
    out += 2;
    out_size -= 2;

    *out++ = (done)  ?  '1'  :  '0';
    out_size--;

    sprintf(out, "%016"PRIu64"x", arg->val);

    out += 16;
    out_size -= 16;

    /* Encode found */
    for (i = 0;  i < count;  i++, found++)
    {
        if (out_size <= 16)
            return  FALSE;

        sprintf(out, "%016"PRIu64"x", found->val);

        out += 16;
        out_size -= 16;
    }

    return  TRUE;
}

int rp_hex_nibble(char in)
{
    int c;

    c = in & 0xff;

    if (c >= '0'  &&  c <= '9')
        return  c - '0';

    if (c >= 'A'  &&  c <= 'F')
        return  c - 'A' + 10;

    if (c >= 'a'  &&  c <= 'f')
        return  c - 'a' + 10;

    return  -1;
}

/* Decode a single nibble */
static int rp_decode_nibble(const char *in, unsigned int *nibble)
{
    int nib;

    if ((nib = rp_hex_nibble(*in)) >= 0)
    {
        *nibble = nib;
        return  TRUE;
    }

    return  FALSE;
}

/* Decode byte */
static int rp_decode_byte(const char *in, unsigned int *byte_ptr)
{
    unsigned int ls_nibble;
    unsigned int ms_nibble;

    if (!rp_decode_nibble(in, &ms_nibble))
        return  FALSE;

    if (!rp_decode_nibble(in + 1, &ls_nibble))
        return  FALSE;

    *byte_ptr = (ms_nibble << 4) + ls_nibble;
    return  TRUE;
}

/* Decode exactly 4 bytes of hex from a longer string, and return the result
   as an unsigned 32-bit value */
static int rp_decode_4bytes(const char *in, uint32_t *val)
{
    unsigned int nibble;
    uint32_t tmp;
    int count;

    for (tmp = 0, count = 0;  count < 8;  count++, in++)
    {
        if (!rp_decode_nibble(in, &nibble))
            break;
        tmp = (tmp << 4) + nibble;
    }
    *val = tmp;
    return  TRUE;
}

/* Decode exactly 8 bytes of hex from a longer string, and return the result
   as an unsigned 64-bit value */
static int rp_decode_8bytes(const char *in, uint64_t *val)
{
    unsigned int nibble;
    uint64_t tmp;
    int count;

    for (tmp = 0, count = 0;  count < 16;  count++, in++)
    {
        if (!rp_decode_nibble(in, &nibble))
            break;
        tmp = (tmp << 4) + nibble;
    }
    *val = tmp;
    return  TRUE;
}

/* Decode a hex string to an unsigned 32-bit value */
static int rp_decode_uint32(char const **in, uint32_t *val, char break_char)
{
    unsigned int nibble;
    uint32_t tmp;
    int count;

    assert(in != NULL);
    assert(val != NULL);

    if (**in == '\0')
    {
        /* We are expecting at least one character */
        return  FALSE;
    }

    for (tmp = 0, count = 0;  **in  &&  count < 8;  count++, (*in)++)
    {
        if (!rp_decode_nibble(*in, &nibble))
            break;
        tmp = (tmp << 4) + nibble;
    }

    if (**in != break_char)
    {
        /* Wrong terminating character */
        return  FALSE;
    }
    if (**in)
        (*in)++;
    *val = tmp;
    return  TRUE;
}

/* Decode a hex string to an unsigned 64-bit value */
static int rp_decode_uint64(char const **in, uint64_t *val, char break_char)
{
    unsigned int nibble;
    uint64_t tmp;
    int count;

    assert(in != NULL);
    assert(val != NULL);

    if (**in == '\0')
    {
        /* We are expecting at least one character */
        return  FALSE;
    }

    for (tmp = 0, count = 0;  **in  &&  count < 16;  count++, (*in)++)
    {
        if (!rp_decode_nibble(*in, &nibble))
            break;
        tmp = (tmp << 4) + nibble;
    }

    if (**in != break_char)
    {
        /* Wrong terminating character */
        return  FALSE;
    }
    if (**in)
        (*in)++;
    *val = tmp;
    return  TRUE;
}

/* Encode byte */
static void rp_encode_byte(unsigned int val, char *out)
{
    static const char hex[] = "0123456789abcdef";

    assert(val <= 0xff);
    assert(out != NULL);

    *out = hex[(val >> 4) & 0xf];
    *(out + 1) = hex[val & 0xf];
}

/* Print usage */
static void rp_usage(void)
{
    rp_target *t;

    printf("This is GDBproxy - a remote proxy for the GNU debugger, GDB.\n\n");
    printf("Usage:\n");
    printf("  %s [options] [target [target-options] [target-args]]\n", name);
    printf("\nOptions:\n");
    printf("  --quiet              do not print help on startup\n");
    printf("  --copying            print copying information\n");
#ifndef WIN32
    printf("  --daemon             run %s as daemon\n", name);
#endif /* WIN32 */
    printf("  --debug              run %s in debug mode\n", name);
    printf("  --help               `%s --help' prints this message\n", name);


    printf("                       `%s --help target' prints target's help\n",
            name);
    printf("  --port=PORT          use the specified TCP port\n");
    printf("  --version            print version\n");
    printf("  --warranty           print warranty information\n");

    printf("\nSupported targets:\n");

    for (t = rp_t_list;  t != NULL;  t = t->next)
    {
        assert(t->name != NULL);
        assert(t->desc != NULL);

        printf("  %-20s %s\n", t->name, t->desc);
    }

    printf("\n");
}

/* Encode return value */
static void rp_write_retval(int ret, char *b)
{
    switch (ret)
    {
        case RP_VAL_TARGETRET_OK:
            strcpy(b, "OK");
            break;
        case RP_VAL_TARGETRET_ERR:
            strcpy(b, "E00");
            break;
        case RP_VAL_TARGETRET_NOSUPP:
            /* Write empty string into buffer */
            *b = '\0';
            break;
        default:
            assert(0);
            break;
    }
}

/* Table of commands */
static const RP_RCMD_TABLE rp_remote_commands[] =
{
    RP_RCMD(help, "This help text"),
    RP_RCMD(set, "Set debug level"),
    {0,0,0}     //sentinel, end of table marker
};

/* Help function, generate help text from command table */
static int rp_rcmd_help(int argc, char *argv[], out_func of, data_func df, rp_target *t)
{
    char buf[1000 + 1];
    char buf2[1000 + 1];
    int i = 0;

    rp_encode_string("Remote command help:\n", buf, 1000);
    of(t, buf);
    for (i = 0;  rp_remote_commands[i].name;  i++)
    {
#ifdef WIN32
        sprintf(buf2, "%-10s %s\n", rp_remote_commands[i].name, rp_remote_commands[i].help);
#else
        snprintf(buf2, 1000, "%-10s %s\n", rp_remote_commands[i].name, rp_remote_commands[i].help);
#endif
        rp_encode_string(buf2, buf, 1000);
        of(t, buf);
    }
    if (t->remote_commands)
        for (i = 0;  t->remote_commands[i].name;  i++)
        {
#ifdef WIN32
            sprintf(buf2, "%-10s %s\n", t->remote_commands[i].name, t->remote_commands[i].help);
#else
            snprintf(buf2, 1000, "%-10s %s\n", t->remote_commands[i].name, t->remote_commands[i].help);
#endif
            rp_encode_string(buf2, buf, 1000);
            of(t, buf);
        }
    return RP_VAL_TARGETRET_OK;
}

/* Set function, set debug level */
static int rp_rcmd_set(int argc, char *argv[], out_func of, data_func df, rp_target *t)
{
    char buf[1000 + 1];
    char buf2[1000 + 1];

    if (argc == 1)
    {
        sprintf (buf2, "Missing argument to set command.\n");
        rp_encode_string(buf2, buf, 1000);
        of(t, buf);
        return RP_VAL_TARGETRET_OK;
    }

    if (strcmp ("debug", argv[1]) != 0)
    {
        sprintf (buf2, "Undefined set command: \"%s\"\n", argv[1]);
        rp_encode_string(buf2, buf, 1000);
        of(t, buf);
        return RP_VAL_TARGETRET_OK;
    }

    if (argc != 3)
    {
        sprintf (buf2, "Wrong arguments for debug command.\n");
        rp_encode_string(buf2, buf, 1000);
        of(t, buf);
        return RP_VAL_TARGETRET_OK;
    }

    if (strcmp ("0", argv[2]) == 0)
        rp_debug_level = 0;
    else if (strcmp ("1", argv[2]) == 0)
        rp_debug_level = 1;
    else if (strcmp ("2", argv[2]) == 0)
        rp_debug_level = 2;
    else if (strcmp ("3", argv[2]) == 0)
        rp_debug_level = 3;
    else
    {
        sprintf (buf2, "Invalid debug level: \"%s\"\n", argv[2]);
        rp_encode_string(buf2, buf, 1000);
        of(t, buf);
        return RP_VAL_TARGETRET_OK;
    }

    return RP_VAL_TARGETRET_OK;
}

#ifdef NDEBUG
#define DEBUG_OUT(...)
#else
static void DEBUG_OUT(const char *string,...)
{
    va_list args;
    va_start (args, string);
    fprintf (stderr, "debug: ");
    vfprintf (stderr, string, args);
    fprintf (stderr, "\n");
    va_end (args);
}
#endif

/* Target method */
#define MAXARGS 4
static int handle_rcmd_command(char *in_buf, out_func of, data_func df, rp_target *t)
{
    int count = 0;
    int i;
    char *args[MAXARGS];
    char *ptr;
    unsigned int ch;
    char buf[1000 + 1];
    char *s;

    rp_log(RP_VAL_LOGLEVEL_DEBUG,
            "%s: handle_rcmd_command()",
            name);
    DEBUG_OUT("command '%s'", in_buf);

    if (strlen(in_buf))
    {
        /* There is something to process */
        /* TODO: Handle target specific commands, such as flash erase, JTAG
           control, etc. */
        /* A single example "flash erase" command is partially implemented
           here as an example. */

        /* Turn the hex into ASCII */
        ptr = in_buf;
        s = buf;
        while (*ptr)
        {
            if (rp_decode_byte(ptr, &ch) == 0)
                return RP_VAL_TARGETRET_ERR;
            *s++ = ch;
            ptr += 2;
        }
        *s = '\0';
        DEBUG_OUT("command '%s'", buf);

        /* Split string into separate arguments */
        ptr = buf;
        args[count++] = ptr;
        while (*ptr)
        {
            /* Search to the end of the string */
            if (*ptr == ' ')
            {
                /* Space is the delimiter */
                *ptr = 0;
                if (count >= MAXARGS)
                    return RP_VAL_TARGETRET_ERR;
                args[count++] = ptr + 1;
            }
            ptr++;
        }
        /* Search the command table, and execute the function if found */
        DEBUG_OUT("executing target dependant command '%s'", args[0]);

        /* Search the target command table first, such that we allow target
           to override the general command.  */
        if (t->remote_commands)
            for (i = 0;  t->remote_commands[i].name;  i++)
            {
                if (strcmp(args[0], t->remote_commands[i].name) == 0)
                    return t->remote_commands[i].function(count, args, of, df, t);
            }

        for (i = 0;  rp_remote_commands[i].name;  i++)
        {
            if (strcmp(args[0], rp_remote_commands[i].name) == 0)
                return rp_remote_commands[i].function(count, args, of, df, t);
        }
        return RP_VAL_TARGETRET_NOSUPP;
    }
    return RP_VAL_TARGETRET_ERR;
}

