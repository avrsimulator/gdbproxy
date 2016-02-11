/* Copyright (C) 1999-2001 Quality Quorum, Inc.
   Copyright (C) 2002 Chris Liechti and Steve Underwood

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
   
  
   Implementation of `remote' and `extended-remote' targets. This
   targets share most of the methods.
   
   Exported Data:
     remote_target                - target descriptor of `remote' target 
  
   Imported Data:
     None     
  
   Static Data:
     remote_XXX                  - static data representing status and 
                                   parameters of remote target 
  
   Global Functions:  
     None
  
   Static Functions:  
     remote_XXXX                 - methods comprising `remote' target,
                                   descrition in file proxy.h
     extended_remote_XXX         - the only extended-remote specifi method
     remote_connect_worker       - local function implementing connect
                                   method for both targets
     remote_decode_XXXX          - decode functions
     remote_encode_XXXX          - encode functions 
     remote_putpkt               - send packet to remote
     remote_getpkt               - get packet from remote
      
   
   $Id: remote.c,v 1.5 2010/02/10 12:33:31 vapier Exp $ */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
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
#include "serial.h"

/* Note: we are using prefix 'remote' for static stuff in
   order to simplify debugging of the target code itself */
#define RPREMOTE_VAL_TIMEOUT_DEFAULT   (2)
#define RPREMOTE_VAL_BAUDRATE_DEFAULT  (38400)

/*
 * Target methods, static
 */
static void  remote_help(const char *prog_name);
static void  extended_remote_help(const char *prog_name);
static int   remote_open(int argc,
                         char * const argv[],
                         const char *prog_name,
                         log_func log_fn);
static void  remote_close(void);
static int   remote_connect(char *status_string, size_t status_string_size,
                            int *can_restart);
static int   extended_remote_connect(char *status_string,
                                     size_t status_string_size,
                                     int *can_restart);
static int   remote_disconnect(void);
static void  remote_kill(void);
static int   remote_restart(void);
static int   extended_remote_restart(void);
static void  remote_stop(void);
static int   remote_set_gen_thread(rp_thread_ref *thread);
static int   remote_set_ctrl_thread(rp_thread_ref *thread);
static int   remote_is_thread_alive(rp_thread_ref *thread, int *alive);
static int   remote_read_registers(unsigned char *data_buf,
                                   unsigned char *avail_buf,
                                   size_t buf_size,
                                   size_t *read_size);
static int   remote_write_registers(unsigned char *data_buf, size_t write_size);
static int   remote_read_single_register(unsigned int reg_no,
                                         unsigned char *data_buf,
                                         unsigned char *avail_buf,
                                         size_t buf_size,
                                         size_t *read_size);
static int   remote_write_single_register(unsigned int reg_no,
                                          unsigned char *data_buf,
                                          size_t write_size);
static int   remote_read_mem(uint64_t addr, unsigned char *data_buf,
                             size_t req_size, size_t *actual_size);
static int   remote_write_mem(uint64_t addr, unsigned char *data_buf,
                              size_t req_sise);
static int   remote_resume_from_current(int step, int sig);
static int   remote_resume_from_addr(int step, int sig,
                                     uint64_t addr);
static int   remote_go_waiting(int sig);
static int   remote_wait_partial(int first, char *status_string,
                                 size_t status_string_len, out_func out,
                                 int *implemented, int *more);
static int   remote_wait(char *status_string, size_t status_string_len,
                         out_func out, int *implemented);
static int   remote_process_query(unsigned int *mask, rp_thread_ref *arg,
                                  rp_thread_info *info);
static int   remote_list_query(int first, rp_thread_ref *arg,
                               rp_thread_ref *result,
                               size_t max_num, size_t *num, int *done);
static int   remote_current_thread_query(rp_thread_ref *thread);
static int   remote_offsets_query(uint64_t *text,
                                  uint64_t *data,
                                  uint64_t *bss);
static int   remote_crc_query(uint64_t addr,
                              size_t len,
                              uint32_t *val);
static int   remote_raw_query(char *in_buf,
                              char *out_buf,
                              size_t out_buf_size);
static int   remote_remcmd(char *in_buf, out_func of, data_func df);
static int   remote_add_break(int type, uint64_t adddr, unsigned int len);
static int   remote_remove_break(int type, uint64_t, unsigned int len);


/*
 * Global target descriptor for target `remote' 
 */
rp_target remote_target =
{
    NULL,      /* next */
    "remote",
    "proxy target implementing gdb serial protocol",
    NULL,
    remote_help,
    remote_open,
    remote_close,
    remote_connect,
    remote_disconnect,
    remote_kill,
    remote_restart,
    remote_stop,
    remote_set_gen_thread,
    remote_set_ctrl_thread,
    remote_is_thread_alive,
    remote_read_registers,
    remote_write_registers,
    remote_read_single_register,
    remote_write_single_register,
    remote_read_mem,
    remote_write_mem,
    remote_resume_from_current,
    remote_resume_from_addr,
    remote_go_waiting,
    remote_wait_partial,
    remote_wait,
    remote_process_query,
    remote_list_query,
    remote_current_thread_query,
    remote_offsets_query,
    remote_crc_query,
    remote_raw_query,
    remote_add_break,
    remote_remove_break
};

/*
 * Global target descriptor for target `extended-remote' 
 */
rp_target extended_remote_target =
{
    NULL,      /* next */
    "extended-remote",
    "proxy target implementing extended gdb serial protocol",
    NULL,
    extended_remote_help,
    remote_open,
    remote_close,
    extended_remote_connect,
    remote_disconnect,
    remote_kill,
    extended_remote_restart,
    remote_stop,
    remote_set_gen_thread,
    remote_set_ctrl_thread,
    remote_is_thread_alive,
    remote_read_registers,
    remote_write_registers,
    remote_read_single_register,
    remote_write_single_register,
    remote_read_mem,
    remote_write_mem,
    remote_resume_from_current,
    remote_resume_from_addr,
    remote_go_waiting,
    remote_wait_partial,
    remote_wait,
    remote_process_query,
    remote_list_query,
    remote_current_thread_query,
    remote_offsets_query,
    remote_crc_query,
    remote_raw_query,
    remote_add_break,
    remote_remove_break
};


/* Internal status */
static int      remote_is_open      = 0;
static int      remote_connected    = 0;
static int      remote_extended_ops = 0;

/* Start up parameters, set by open */
static const char *remote_name      = NULL;
static log_func   remote_log        = NULL;
static int        remote_break      = FALSE; /* Use ^C to break in */
static int        remote_baud       = RPREMOTE_VAL_BAUDRATE_DEFAULT;
static int        remote_timeout    = RPREMOTE_VAL_TIMEOUT_DEFAULT;

/* Buffers */
static char remote_out_buf[RP_PARAM_INOUTBUF_SIZE];

/* Local functions */
static int  remote_connect_worker(int extended,
                                  char *status_string,
                                  size_t status_string_size,
                                  int *can_restart);
static int  remote_decode_data(const char *in,
                               unsigned char *out,
                               size_t out_size,
                               size_t *len);
static int  remote_encode_data(const unsigned char *data,
                               size_t data_len,
                               char *out,
                               size_t out_size);
static int  remote_decode_process_query_response(const char *in,
                                                 unsigned int *mask,
                                                 rp_thread_ref *ref,
                                                 rp_thread_info *info);
static int  remote_decode_list_query_response(const char *in,
                                              size_t max_num,
                                              size_t *num, int *done,
                                              rp_thread_ref *arg,
                                              rp_thread_ref *result);
static int  remote_decode_byte(const char *in, unsigned int *byte);
static int  remote_decode_nibble(const char *in, unsigned int *nibble);
static int  remote_decode_4bytes(const char *in, unsigned int *val);
static int  remote_decode_8bytes(const char *in, uint64_t *val);
static void remote_encode_byte(unsigned int val, char *out);

static int  remote_putpkt(const char *buf);
static int  remote_getpkt(char *buf, size_t buf_len, size_t *len, int timeout);

/* Target method */
static void remote_help(const char *prog_name)
{
    printf("This is proxy target `%s'. Usage:\n\n", remote_target.name);
    printf("  %s [options] %s [%s-options] DEVICE\n",
           prog_name,
           remote_target.name,
           remote_target.name);
    printf("\nOptions:\n\n");
    printf("  --copying         print copying information\n");
    printf("  --debug           run %s in debug mode\n", prog_name);
    printf("  --help            `%s --help %s'  prints this message\n",
           prog_name,
           remote_target.name);
    printf("  --port=PORT       use the specified TCP port\n");
    printf("  --version         print version\n");
    printf("  --warranty        print warranty information\n");
    printf("\nRemote-options:\n\n");
    printf("  --debug           run %s in debug mode\n", prog_name);
    printf("  --use-break       instructs to send BREAK to stop running\n");
    printf("                    target (default is to send CTRL-C)\n");
    printf("  --baud-rate=BAUD  sets baud rate (default is %d)\n",
           RPREMOTE_VAL_BAUDRATE_DEFAULT);
    printf("  --timeout=TIMEOUT sets timeout in seconds (default is %d,\n",
           RPREMOTE_VAL_TIMEOUT_DEFAULT);
    printf("                    min value is 1, max value is 3600)\n");
    printf("\nDevice:\n\n");
    printf(" Device specifies serial device, e.g. `/dev/ttyS0'\n");

    printf("\n");

    return;
}


/* Target method */
static void extended_remote_help(const char *prog_name)
{
    printf("This is proxy target `%s'. Usage:\n\n", extended_remote_target.name);
    printf("  %s [options] %s [%s-options] DEVICE\n",
           prog_name,
           extended_remote_target.name,
           extended_remote_target.name);
    printf("\nOptions:\n\n");
    printf("  --copying         print copying information\n");
    printf("  --debug           run %s in debug mode\n", prog_name);
    printf("  --help            `%s --help %s'  prints this message\n",
           prog_name,
           extended_remote_target.name);
    printf("  --port=PORT       use the specified TCP port\n");
    printf("  --version         print version\n");
    printf("  --warranty        print warranty information\n");
    printf("\nRemote-options:\n\n");
    printf("  --debug           enables proxy target debugging.\n");
    printf("  --use-break       instructs to send BREAK to stop running\n");
    printf("                    target. Default is to send CTRL-C\n");
    printf("  --baud-rate=BAUD  sets baud rate (default is %d)\n",
           RPREMOTE_VAL_BAUDRATE_DEFAULT);
    printf("  --timeout=TIMEOUT sets timeout in seconds (default is %d,\n",
           RPREMOTE_VAL_TIMEOUT_DEFAULT);
    printf("                    min value is 1, max value is 3600)\n");
    printf("\nDevice:\n\n");
    printf(" Device specifies serial device, e.g. `/dev/ttyS0'\n");
    printf("\nNote:\n\n");
    printf(" If the stub your communicating with does not support\n");
    printf(" extended operations, use target `remote' instead\n");

    printf("\n");

    return;
}

/* Target method */
static int remote_open(int argc,
                       char * const argv[],
                       const char *prog_name,
                       log_func log_fn)
{
    int ret;

    /* Option descriptors */
    static struct option long_options[] =
    {
        /* Options setting flag */
        {"use-break",   0, &remote_break, 1},

        /* Options with parameter */
        {"baud-rate",   1, 0, 1},
        {"timeout",     1, 0, 2},
        {NULL, 0, 0, 0}
    };

    assert(argc > 0);
    assert(prog_name != NULL);
    assert(log_fn != NULL);

    assert(!remote_is_open);
    assert(!remote_connected);

    /* Reset default values */
    remote_extended_ops   = FALSE;
    remote_break          = FALSE;
    remote_baud           = RPREMOTE_VAL_BAUDRATE_DEFAULT;
    remote_timeout        = RPREMOTE_VAL_TIMEOUT_DEFAULT;

    /* Set name and log */
    if (strcmp(argv[0], remote_target.name) == 0)
    {
        remote_name = remote_target.name;
    }
    else
    {
        assert(strcmp(argv[0], extended_remote_target.name) == 0);
        remote_name = extended_remote_target.name;
    }

    remote_log = log_fn;

    /* Process options */
    for (;;)
    {
        int c;
        int option_index;

        c = getopt_long_only(argc, argv, "", long_options, &option_index);
        if (c == EOF)
            break;

        switch (c)
        {
        case 0:
            /* Long option which just sets a flag */
            break;
        case 1:
            /* Baud rate */
            remote_baud = atoi(optarg);

            if (!serial_check_baud(remote_baud))
            {
                remote_log(RP_VAL_LOGLEVEL_ERR,
                           "%s: bad baud value %s",
                           remote_name,
                           optarg);
                return RP_VAL_TARGETRET_ERR;
            }

            break;
        case 2:
            /* Timeout */
            remote_timeout = atoi(optarg);

            if (remote_timeout < 1  ||  remote_timeout > 3600)
            {
                remote_log(RP_VAL_LOGLEVEL_ERR,
                           "%s: bad timeout value %s",
                           remote_name,
                           optarg);
                return RP_VAL_TARGETRET_ERR;
            }

            break;
        default:
            remote_log(RP_VAL_LOGLEVEL_NOTICE,
                       "%s: Use `%s --help %s' to see a complete list of options",
                       remote_name,
                       prog_name,
                       remote_name);
            return RP_VAL_TARGETRET_ERR;
        }
    }

    /* Convert remote timeout into milliseconds */
    assert(remote_timeout >= 1  &&  remote_timeout <= 3600);

    remote_timeout = remote_timeout * 1000;

    if (argc != (optind + 1))
    {
        /* Bad number of arguments */
        remote_log(RP_VAL_LOGLEVEL_ERR,
                   "%s: bad number of arguments",
                   remote_name);
        remote_target.help(prog_name);

        return RP_VAL_TARGETRET_ERR;
    }


    if (!(ret = serial_open(argv[optind], remote_baud)))
    {
        remote_log(RP_VAL_LOGLEVEL_ERR,
                   "%s: unable to opend device %s at %d baud",
                   remote_name,
                   argv[optind],
                   remote_baud);

        return RP_VAL_TARGETRET_ERR;
    }

    remote_is_open = 1;

    return RP_VAL_TARGETRET_OK;
}


/* Target method */
static void remote_close(void)
{
    assert(remote_is_open);

    if (remote_connected)
    {
        /* Disconnect and leave it running if connected */
        remote_target.disconnect();
    }

    remote_is_open = 0;
    remote_connected = 0;

    serial_close();
}

/* Target method */
static int remote_connect(char *status_string,
                          size_t status_string_len,
                          int *can_restart)
{
    return remote_connect_worker(FALSE,
                                 status_string,
                                 status_string_len,
                                 can_restart);
}

/* Target method */
static int extended_remote_connect(char *status_string,
                                   size_t status_string_len,
                                   int *can_restart)
{
    return remote_connect_worker(TRUE,
                                 status_string,
                                 status_string_len,
                                 can_restart);
}

/* Target method */
static int remote_disconnect(void)
{
    int ret;
    char in_buf[RP_PARAM_INOUTBUF_SIZE];
    unsigned int in_len;

    assert(remote_is_open);
    assert(remote_connected);

    if (!(ret = remote_putpkt("D")))
        return RP_VAL_TARGETRET_ERR;

    /* If stub does not support the function, it
       will reply, so we still have to try sucking in the 
       buffer */

    ret = remote_getpkt(in_buf, sizeof(in_buf), &in_len, 500);
    if (ret == 0  &&  in_len < 1)
    {
        /* It is not supported */
        return RP_VAL_TARGETRET_NOSUPP;
    }

    /* Otherwise consider it disconnected */
    remote_connected = FALSE;
    remote_extended_ops = FALSE;

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static void remote_kill(void)
{
    char in_buf[RP_PARAM_INOUTBUF_SIZE];
    unsigned int in_len;

    assert(remote_is_open);

    remote_putpkt("k");

    /* It is quite likely there will be nothing back, so don't wait very long */
    remote_getpkt(in_buf, sizeof(in_buf), &in_len, 100);
    remote_connected = FALSE;
}


/* Target method */
static int remote_restart(void)
{
    assert(remote_is_open);
    assert(remote_connected);
    assert(remote_extended_ops);

    /* It should not happen */
    assert(0);

    return RP_VAL_TARGETRET_ERR;
}

/* Target method */
static int extended_remote_restart(void)
{
    int ret;

    assert(remote_is_open);
    assert(remote_connected);
    assert(remote_extended_ops);

    /* We can use the 'R' command. It has to be
       followed by a number for reasons unknown
      
       It is quite possible we are restarting 
       a hung up system. For example, this could happen
       after a kill. */

    if (!(ret = remote_putpkt("R00")))
        return RP_VAL_TARGETRET_ERR;

    /* We do not expect a response here */
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static void remote_stop(void)
{
    assert(remote_is_open);
    assert(remote_connected);

    if (remote_break)
        serial_sendbreak();
    else
        serial_putchar(0x03);
}

static int remote_set_gen_thread(rp_thread_ref *thread)
{
    int  ret;
    char buf[40];
    char in_buf[RP_PARAM_INOUTBUF_SIZE];
    unsigned int in_len;

    assert(thread != NULL);

    sprintf(buf, "Hg%"PRIu64"x", thread->val);

    if (!(ret = remote_putpkt(buf)))
        return RP_VAL_TARGETRET_ERR;

    if ((ret = remote_getpkt(in_buf, sizeof(in_buf), &in_len, -1)) != 0)
        return RP_VAL_TARGETRET_ERR;
    if (in_len < 1)
        return RP_VAL_TARGETRET_NOSUPP;
    if (in_buf[0] != 'O'  ||  in_buf[1] != 'K')
        return RP_VAL_TARGETRET_ERR;

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int remote_set_ctrl_thread(rp_thread_ref *thread)
{
    char in_buf[RP_PARAM_INOUTBUF_SIZE];
    unsigned int in_len;
    int  ret;
    char buf[40];

    sprintf(buf, "Hc%"PRIu64"x", thread->val);

    if (!(ret = remote_putpkt(buf)))
        return RP_VAL_TARGETRET_ERR;

    if ((ret = remote_getpkt(in_buf, sizeof(in_buf), &in_len, -1)) != 0)
        return RP_VAL_TARGETRET_ERR;
    if (in_len < 1)
        return RP_VAL_TARGETRET_NOSUPP;
    if (in_buf[0] != 'O'  ||  in_buf[1] != 'K')
        return RP_VAL_TARGETRET_ERR;

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int remote_is_thread_alive(rp_thread_ref *thread, int *alive)
{
    char in_buf[RP_PARAM_INOUTBUF_SIZE];
    unsigned int in_len;
    int  ret;
    char buf[40];

    assert(thread != NULL);
    assert(alive != NULL);

    sprintf(buf, "T%"PRIu64"x", thread->val);

    if (!(ret = remote_putpkt(buf)))
        return RP_VAL_TARGETRET_ERR;

    /* This one is unusual 'ENN' means - not alive */
    if ((ret = remote_getpkt(in_buf, sizeof(in_buf), &in_len, -1)) != 0)
        return RP_VAL_TARGETRET_ERR;
    if (in_len < 1)
        return RP_VAL_TARGETRET_NOSUPP;
    if ((in_buf[0] != 'O'  ||  in_buf[1] != 'K')
        &&
        in_buf[0] != 'E')
    {
        return RP_VAL_TARGETRET_ERR;
    }

    if (in_buf[0] == 'O'  &&  in_buf[1] == 'K')
        *alive = TRUE;
    else
        *alive = FALSE;

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int remote_read_registers(unsigned char *data_buf,
                                 unsigned char *avail_buf,
                                 size_t buf_size,
                                 size_t *read_size)
{
    char in_buf[RP_PARAM_INOUTBUF_SIZE];
    unsigned int in_len;
    int ret;
    size_t count;
    char *in;
    unsigned int val;

    assert(data_buf != NULL);
    assert(avail_buf != NULL);
    assert(buf_size > 0);
    assert(read_size != NULL);

    if ((ret = remote_putpkt("g")))
        return RP_VAL_TARGETRET_ERR;

    for (;;)
    {
        if ((ret = remote_getpkt(in_buf, sizeof(in_buf), &in_len, -1)) != 0)
            return RP_VAL_TARGETRET_ERR;
        if (in_len < 1)
            return RP_VAL_TARGETRET_NOSUPP;
        if (rp_hex_nibble(in_buf[0]) >= 0  ||  in_buf[0] != 'x')
            break;
    }

    for (count = 0, in = in_buf;
         *in != 0  &&  count < buf_size;
         count++, in += 2, data_buf++, avail_buf++)
    {
        if (*(in + 1) == '\0')
        {
            /* Odd number of nibbles, discard the last one */
            remote_log(RP_VAL_LOGLEVEL_WARNING,
                       "%s: odd number of nibbles",
                       remote_name);

            if (count == 0)
                return RP_VAL_TARGETRET_ERR;

            *read_size = count;
            return RP_VAL_TARGETRET_OK;
        }

        if (*in == 'x'  &&  *(in + 1) == 'x')
        {
            *avail_buf = 0;
            *data_buf = 0;
        }
        else if (*in == 'x'  ||  *(in + 1) == 'x')
        {
            /* Unavailable nibble ???? */
            return RP_VAL_TARGETRET_ERR;
        }
        else
        {
            if (!remote_decode_byte(in, &val))
                return RP_VAL_TARGETRET_ERR;

            *avail_buf = 1;
            *data_buf = val & 0xff;
        }
    }

    if (*in != '\0')
    {
        /* Input too long */
        return RP_VAL_TARGETRET_ERR;
    }

    *read_size = count;

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int remote_write_registers(unsigned char *buf, size_t write_size)
{
    char in_buf[RP_PARAM_INOUTBUF_SIZE];
    unsigned int in_len;
    int ret;

    assert(buf != NULL);
    assert(write_size > 0);

    remote_out_buf[0] = 'G';

    ret = remote_encode_data(buf,
                             write_size,
                             &remote_out_buf[1],
                             sizeof(remote_out_buf) - 1);
    if (!ret)
    {
        assert(0);
        return RP_VAL_TARGETRET_ERR;
    }

    if (!(ret = remote_putpkt(remote_out_buf)))
        return RP_VAL_TARGETRET_ERR;

    if ((ret = remote_getpkt(in_buf, sizeof(in_buf), &in_len, -1)) != 0)
        return RP_VAL_TARGETRET_ERR;
    if (in_len < 1)
        return RP_VAL_TARGETRET_NOSUPP;
    if (in_buf[0] != 'O'  ||  in_buf[1] != 'K')
        return RP_VAL_TARGETRET_ERR;

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int remote_read_single_register(unsigned int reg_no,
                                       unsigned char *data_buf,
                                       unsigned char *avail_buf,
                                       size_t buf_size,
                                       size_t *read_size)
{
    char in_buf[RP_PARAM_INOUTBUF_SIZE];
    unsigned int in_len;
    int ret;
    size_t count;
    char  *in;
    unsigned int val;
    size_t len;

    assert(data_buf != NULL);
    assert(avail_buf != NULL);
    assert(buf_size > 0);
    assert(read_size != NULL);

    len = sprintf(remote_out_buf, "p%x", reg_no);
    if (!(ret = remote_putpkt(remote_out_buf)))
        return RP_VAL_TARGETRET_ERR;

    for (;;)
    {
        if ((ret = remote_getpkt(in_buf, sizeof(in_buf), &in_len, -1)) != 0)
            return RP_VAL_TARGETRET_ERR;
        if (in_len < 1)
            return RP_VAL_TARGETRET_NOSUPP;
        if (rp_hex_nibble(in_buf[0]) >= 0  ||  in_buf[0] != 'x')
            break;
    }

    for (count = 0, in = in_buf;
         *in != 0  &&  count < buf_size;
         count++, in += 2, data_buf++, avail_buf++)
    {
        if (*(in + 1) == 0)
        {
            /* Odd number of nibbles, discard the last one */
            remote_log(RP_VAL_LOGLEVEL_WARNING,
                       "%s: odd number of nibbles",
                       remote_name);

            if (count == 0)
                return RP_VAL_TARGETRET_ERR;

            *read_size = count;
            return RP_VAL_TARGETRET_OK;
        }

        if (*in == 'x'  &&  *(in + 1) == 'x')
        {
            *avail_buf = 0;
            *data_buf  = 0;
        }
        else if (*in == 'x'  ||  *(in + 1) == 'x')
        {
            /* Unavailable nibble ???? */
            return RP_VAL_TARGETRET_ERR;
        }
        else
        {
            if (!remote_decode_byte(in, &val))
                return RP_VAL_TARGETRET_ERR;

            *avail_buf = 1;
            *data_buf  = val & 0xff;
        }
    }

    if (*in)
    {
        /* Input too long */
        return RP_VAL_TARGETRET_ERR;
    }

    *read_size = count;

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int remote_write_single_register(unsigned int reg_no,
                                        unsigned char *buf,
                                        size_t write_size)
{
    char in_buf[RP_PARAM_INOUTBUF_SIZE];
    unsigned int in_len;
    int ret;
    size_t len;

    assert(buf != NULL);
    assert(write_size > 0);

    len = sprintf(remote_out_buf, "P%x=", reg_no);

    ret = remote_encode_data(buf,
                             write_size,
                             &remote_out_buf[len],
                             sizeof(remote_out_buf) - len);
    if (!ret)
    {
        assert(0);
        return RP_VAL_TARGETRET_ERR;
    }

    if (!(ret = remote_putpkt(remote_out_buf)))
        return RP_VAL_TARGETRET_ERR;

    if ((ret = remote_getpkt(in_buf, sizeof(in_buf), &in_len, -1)) != 0)
        return RP_VAL_TARGETRET_ERR;
    if (in_len < 1)
        return RP_VAL_TARGETRET_NOSUPP;
    if (in_buf[0] != 'O'  ||  in_buf[1] != 'K')
        return RP_VAL_TARGETRET_ERR;

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int remote_read_mem (uint64_t addr,
                            unsigned char *buf,
                            size_t req_size,
                            size_t *actual_size)
{
    char in_buf[RP_PARAM_INOUTBUF_SIZE];
    unsigned int in_len;
    int ret;

    assert(buf != NULL);
    assert(actual_size != NULL);

    sprintf(remote_out_buf,"m%"PRIu64"x,%x", addr, req_size);

    if (!(ret = remote_putpkt(remote_out_buf)))
        return RP_VAL_TARGETRET_ERR;

    if ((ret = remote_getpkt(in_buf, sizeof(in_buf), &in_len, -1)) != 0)
        return RP_VAL_TARGETRET_ERR;
    if (in_len < 1)
        return (req_size == 0)  ?  RP_VAL_TARGETRET_OK  :  RP_VAL_TARGETRET_NOSUPP;
    if (!(ret = remote_decode_data(in_buf, buf, req_size, actual_size)))
        return RP_VAL_TARGETRET_ERR;

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int remote_write_mem(uint64_t addr,
                            unsigned char *buf,
                            size_t write_size)
{
    char in_buf[RP_PARAM_INOUTBUF_SIZE];
    unsigned int in_len;
    int ret;
    size_t off;

    assert(buf != NULL);

    off = sprintf(remote_out_buf, "M%"PRIu64"x,%x:", addr, write_size);

    if (write_size > 0)
    {
        ret = remote_encode_data(buf,
                                 write_size,
                                 &remote_out_buf[off],
                                 sizeof(remote_out_buf) - off);
        if (!ret)
        {
            assert(0);
            return RP_VAL_TARGETRET_ERR;
        }
    }

    if (!(ret = remote_putpkt(remote_out_buf)))
        return RP_VAL_TARGETRET_ERR;

    if ((ret = remote_getpkt(in_buf, sizeof(in_buf), &in_len, -1)) != 0)
        return RP_VAL_TARGETRET_ERR;
    if (in_len < 1)
        return RP_VAL_TARGETRET_NOSUPP;
    if (in_buf[0] != 'O'  ||  in_buf[1] != 'K')
        return RP_VAL_TARGETRET_ERR;

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int remote_resume_from_current(int step, int sig)
{
    int ret;

    assert(remote_is_open);
    assert(remote_connected);

    if (sig == RP_VAL_TARGETSIG_0)
    {
        ret = remote_putpkt(step  ?  "s"  :  "c");
    }
    else
    {
        sprintf(remote_out_buf, "%c%02x", step  ?  'S'  :  'C', sig);
        ret = remote_putpkt(remote_out_buf);
    }

    return (ret)  ?  RP_VAL_TARGETRET_OK  :  RP_VAL_TARGETRET_ERR;
}

/* Target method */
static int remote_resume_from_addr(int step, int sig, uint64_t addr)
{
    int ret;

    assert(remote_is_open);
    assert(remote_connected);

    if (sig == RP_VAL_TARGETSIG_0)
        sprintf(remote_out_buf, "%c%"PRIu64"x", step ? 's' : 'c', addr);
    else
        sprintf(remote_out_buf, "%c%02x;%"PRIu64"x", step ? 'S' : 'C', sig, addr);

    ret = remote_putpkt(remote_out_buf);

    return (ret)  ?  RP_VAL_TARGETRET_OK  :  RP_VAL_TARGETRET_ERR;
}

/* Target method */
static int remote_go_waiting(int sig)
{
    int ret;

    if (sig == RP_VAL_TARGETSIG_0)
    {
        ret = remote_putpkt("w");
    }
    else
    {
        sprintf(remote_out_buf, "W%02x", sig);
        ret = remote_putpkt(remote_out_buf);
    }

    return (ret)  ?  RP_VAL_TARGETRET_OK  :  RP_VAL_TARGETRET_ERR;
}

/* Target method */
static int remote_wait_partial(int first,
                               char *status_string,
                               size_t status_string_len,
                               out_func of,
                               int *implemented,
                               int *more)
{
    char in_buf[RP_PARAM_INOUTBUF_SIZE];
    unsigned int in_len;
    int ret;

    assert(status_string != NULL);
    assert(status_string_len > 0);
    assert(of != NULL);
    assert(implemented != NULL);
    assert(more != NULL);

    assert(remote_is_open);
    assert(remote_connected);

    *implemented = 1;

    for (;;)
    {
        /* We will use bigger timeout on the first try and
           smaller one on the second in order to improve 
           responsivness of the thing to user interrupts
           
           In any case timeout has to be high enough so there
           will be no artificial timeouts in the middle of the buffer.  */
        ret = remote_getpkt(in_buf,
                            sizeof(in_buf),
                            &in_len, 
                            first  ?  500u  :  100u);
        if (ret == -1)
            return RP_VAL_TARGETRET_ERR;

        if (ret == 1)
        {
            /* Timeout */
            *more = 1;
            return RP_VAL_TARGETRET_OK;
        }

        if (in_len < 1)
        {
            /* This means the remote_resume_xXXX we just used
               is not supported */
            return RP_VAL_TARGETRET_NOSUPP;
        }

        switch (in_buf[0])
        {
        case 'E':
            return RP_VAL_TARGETRET_ERR;
        case 'T':
        case 'S':
        case 'W':
        case 'X':
            assert(strlen(in_buf) < status_string_len);
            strcpy(status_string, in_buf);
            *more = 0;
            return RP_VAL_TARGETRET_OK;
        case 'O':
        {
            char *in;
            char *out;
            unsigned int val;

            for (in = &in_buf[1], out = remote_out_buf;  *in != 0;  in += 2, out++)
            {
                if (*(in + 1) == 0)
                {
                    /* Odd number of nibbles, ignore the last one */
                    *out = 0;
                    break;
                }

                if (!remote_decode_byte(in, &val))
                {
                    /* Corrupted string */
                    *out = '.';
                }
                else
                {
                    *out = val & 0xff;
                }
            }

            *out = 0;

            of(remote_out_buf);
            break;
        }
        default:
            break;
        }
    }
}

/* Target method */
static int remote_wait(char *status_string,
                       size_t status_string_len,
                       out_func of,
                       int *implemented)
{
    char in_buf[RP_PARAM_INOUTBUF_SIZE];
    unsigned int in_len;
    int ret;

    assert(status_string != NULL);
    assert(status_string_len > 0);
    assert(of != NULL);
    assert(implemented != NULL);

    assert(remote_is_open);
    assert(remote_connected);

    *implemented = 1;

    for (;;)
    {
        if ((ret = remote_getpkt(in_buf, sizeof(in_buf), &in_len, 2000)) == 1)
        {
            /* Timeout */
            continue;
        }

        if (ret != 0)
            return RP_VAL_TARGETRET_ERR;

        if (in_len < 1)
        {
            /*
             * This means that remote_resume_xXXX we just used
             * is not supported
             */
            return RP_VAL_TARGETRET_NOSUPP;
        }

        switch (in_buf[0])
        {
        case 'E':
            return RP_VAL_TARGETRET_ERR;
        case 'T':
        case 'S':
        case 'W':
        case 'X':
            assert(strlen(in_buf) < status_string_len);
            strcpy(status_string, in_buf);
            return RP_VAL_TARGETRET_OK;
        case 'O':
            {
                char *in;
                char *out;
                unsigned int val;

                for (in = &in_buf[1], out = remote_out_buf;  *in != 0;  in += 2, out++)
                {
                    if (*(in + 1) == 0)
                    {
                        /* Odd number of nibbles, ignore the last one */
                        *out = 0;
                        break;
                    }

                    if (!remote_decode_byte(in, &val))
                    {
                        /* Corrupted string */
                        *out = '.';
                    }
                    else
                    {
                        *out = val & 0xff;
                    }
                }

                *out = 0;

                of(out);
                break;
            }
        default:
            break;
        }
    }
    /* NOTREACHED */
}

/* Target method */
static int remote_process_query(unsigned int *mask,
                                rp_thread_ref *arg,
                                rp_thread_info *info)
{
    char in_buf[RP_PARAM_INOUTBUF_SIZE];
    unsigned int in_len;
    int off;
    int ret;

    assert(mask != NULL);
    assert(arg != NULL);
    assert(info != NULL);

    off = sprintf(remote_out_buf, "qP%08x%016"PRIu64"x", *mask, arg->val);

    assert(off < sizeof(remote_out_buf));

    if (!(ret = remote_putpkt(remote_out_buf)))
        return RP_VAL_TARGETRET_ERR;

    if ((ret = remote_getpkt(in_buf, sizeof(in_buf), &in_len, -1)) != 0)
        return RP_VAL_TARGETRET_ERR;
    if (in_len < 1)
        return RP_VAL_TARGETRET_NOSUPP;

    ret = remote_decode_process_query_response(in_buf,
                                               mask,
                                               arg,
                                               info);
    if (!ret)
        return RP_VAL_TARGETRET_ERR;

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int remote_list_query(int first,
                             rp_thread_ref *arg,
                             rp_thread_ref *result,
                             size_t max_num,
                             size_t *num,
                             int *done)
{
    char in_buf[RP_PARAM_INOUTBUF_SIZE];
    unsigned int in_len;
    int off;
    int ret;

    off = sprintf(remote_out_buf,
                  "qL%c%02x%016"PRIu64"x",
                  first  ?  '1'  :  '0',
                  max_num,
                  arg->val);

    assert(off < sizeof(remote_out_buf));

    if (!(ret = remote_putpkt(remote_out_buf)))
        return RP_VAL_TARGETRET_ERR;

    if ((ret = remote_getpkt(in_buf, sizeof(in_buf), &in_len, -1)) != 0)
        return RP_VAL_TARGETRET_ERR;
    if (in_len < 1)
        return RP_VAL_TARGETRET_NOSUPP;

    ret = remote_decode_list_query_response(in_buf,
                                            max_num,
                                            num,
                                            done,
                                            arg,
                                            result);
    if (!ret)
        return RP_VAL_TARGETRET_ERR;

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int remote_current_thread_query(rp_thread_ref *thread)
{
    char in_buf[RP_PARAM_INOUTBUF_SIZE];
    unsigned int in_len;
    int  i;
    int ret;
    char *in;
    unsigned int nibble;
    uint64_t tmp;


    assert(thread != NULL);

    if (!(ret = remote_putpkt("qC")))
        return RP_VAL_TARGETRET_ERR;

    if ((ret = remote_getpkt(in_buf, sizeof(in_buf), &in_len, -1)) != 0)
        return RP_VAL_TARGETRET_ERR;
    if (in_len < 1)
        return RP_VAL_TARGETRET_NOSUPP;

    if (in_buf[0] != 'Q'  ||  in_buf[1] != 'C')
        return RP_VAL_TARGETRET_ERR;

    tmp = 0;
    in  = &in_buf[2];

    for (i = 0;  i < 16;  i++, in++)
    {
        if (!remote_decode_nibble(in, &nibble))
            return RP_VAL_TARGETRET_ERR;

        tmp = (tmp << 4) + nibble;
    }

    thread->val = tmp;

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int remote_offsets_query(uint64_t *text,
                                uint64_t *data,
                                uint64_t *bss)
{
    char in_buf[RP_PARAM_INOUTBUF_SIZE];
    unsigned int in_len;
    uint64_t addr;
    unsigned int val;
    unsigned int i;
    int  ret;
    char *cp;

    assert(text != NULL);
    assert(data != NULL);
    assert(bss != NULL);

    if (!(ret = remote_putpkt("qOffsets")))
        return RP_VAL_TARGETRET_ERR;

    if ((ret = remote_getpkt(in_buf, sizeof(in_buf), &in_len, -1)) != 0)
        return RP_VAL_TARGETRET_ERR;
    if (in_len < 1)
        return RP_VAL_TARGETRET_NOSUPP;

    cp = in_buf;

    if (strncmp(cp, "Text=", 5) != 0)
        return RP_VAL_TARGETRET_ERR;

    for (addr = 0, i = 0, cp += 5;  i < 16;  i++, cp++)
    {
        if (!remote_decode_nibble(cp, &val))
            break;

        addr = (addr << 4) + val;
    }

    if (*cp != ';')
        return RP_VAL_TARGETRET_ERR;

    cp++;
    *text = addr;

    if (strncmp(cp, "Data=", 5) != 0)
        return RP_VAL_TARGETRET_ERR;

    for (addr = 0, i = 0, cp += 5;  i < 16;  i++, cp++)
    {
        if (!remote_decode_nibble(cp, &val))
            break;

        addr = (addr << 4) + val;
    }

    if (*cp != ';')
        return RP_VAL_TARGETRET_ERR;

    cp++;
    *data = addr;

    if (strncmp(cp, "Bss=", 4) != 0)
        return RP_VAL_TARGETRET_ERR;

    for (addr = 0, i = 0, cp += 4;  i < 16;  i++, cp++)
    {
        if (!remote_decode_nibble(cp, &val))
            break;

        addr = (addr << 4) + val;
    }

    if (*cp != ';')
        return RP_VAL_TARGETRET_ERR;

    cp++;
    *bss = addr;

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int remote_crc_query(uint64_t addr,
                            size_t len,
                            uint32_t *val)
{
    char in_buf[RP_PARAM_INOUTBUF_SIZE];
    unsigned int in_len;
    int32_t crc;
    unsigned int tmp;
    unsigned int i;
    char *cp;
    int ret;

    assert(val != NULL);

    sprintf(remote_out_buf, "qCRC:%"PRIu64"x,%x", addr, len);

    if (!(ret = remote_putpkt(remote_out_buf)))
        return RP_VAL_TARGETRET_ERR;

    if ((ret = remote_getpkt(in_buf, sizeof(in_buf), &in_len, -1)) != 0)
        return RP_VAL_TARGETRET_ERR;
    if (in_len < 1)
        return RP_VAL_TARGETRET_NOSUPP;

    cp = in_buf;

    if (*cp != 'C')
        return RP_VAL_TARGETRET_ERR;

    for (crc = 0, i = 0, cp++;  i < 8;  i++, cp++)
    {
        if (!remote_decode_nibble(cp, &tmp))
            break;

        crc = (crc << 4) + tmp;
    }

    if (*cp != 0)
        return RP_VAL_TARGETRET_ERR;

    *val = crc;

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int remote_raw_query(char *in_buf, char *out_buf, size_t out_buf_size)
{
    unsigned int out_len;
    int ret;

    assert(strlen(in_buf) < sizeof(remote_out_buf));
    assert(out_buf != 0);
    assert(out_buf_size > 4);

    if (!(ret = remote_putpkt(in_buf)))
        return RP_VAL_TARGETRET_ERR;

    if ((ret = remote_getpkt(out_buf, out_buf_size, &out_len, -1)) != 0)
        return RP_VAL_TARGETRET_ERR;
    if (out_len < 1)
        return RP_VAL_TARGETRET_NOSUPP;

    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int remote_remcmd(char *in_buf, out_func of, data_func df)
{
    unsigned int in_len;
    int ret;

    assert(strlen(in_buf) < (sizeof(remote_out_buf) - 6));

    sprintf(remote_out_buf, "qRcmd,%s", in_buf);

    if (!(ret = remote_putpkt(remote_out_buf)))
        return RP_VAL_TARGETRET_ERR;

    for (;;)
    {
        if ((ret = remote_getpkt(in_buf, sizeof(in_buf), &in_len, -1)) != 0)
            return RP_VAL_TARGETRET_ERR;
        if (in_len < 1)
            return RP_VAL_TARGETRET_NOSUPP;
        switch (in_buf[0])
        {
        case 'E':
            return RP_VAL_TARGETRET_ERR;
        case 'O':
            if (in_buf[1] == 'K'  &&  in_buf[2] == 0)
                return RP_VAL_TARGETRET_OK;

            {
                char *in;
                char *out;
                unsigned int val;

                for (in = &in_buf[1], out = remote_out_buf;  *in != 0;  in += 2, out++)
                {
                    if (*(in + 1) == 0)
                    {
                        /* Odd number of nibbles, ignore the last one */
                        *out = 0;
                        break;
                    }

                    if (!remote_decode_byte(in, &val))
                    {
                        /* Corrupted string */
                        *out = '.';
                    }
                    else
                    {
                        *out = val & 0xff;
                    }
                }

                *out = 0;
                of(remote_out_buf);
                break;
            }
        default:
            df(in_buf);
            break;
        }
    }
    /* NOTREACHED */
}

static int remote_add_break(int type, uint64_t addr, unsigned int len)
{
    char in_buf[RP_PARAM_INOUTBUF_SIZE];
    unsigned int in_len;
    int ret;

    sprintf(remote_out_buf, "Z%d,%"PRIu64"x,%x", type, addr, len);

    if (!(ret = remote_putpkt(remote_out_buf)))
        return RP_VAL_TARGETRET_ERR;

    if ((ret = remote_getpkt(in_buf, sizeof(in_buf), &in_len, -1)) != 0)
        return RP_VAL_TARGETRET_ERR;
    if (in_len < 1)
        return RP_VAL_TARGETRET_NOSUPP;

    if (in_buf[0] != 'O'  ||  in_buf[1] != 'K')
        return RP_VAL_TARGETRET_ERR;

    return RP_VAL_TARGETRET_OK;
}

static int remote_remove_break(int type, uint64_t addr, unsigned int len)
{
    char in_buf[RP_PARAM_INOUTBUF_SIZE];
    unsigned int in_len;
    int ret;

    sprintf(remote_out_buf, "z%d,%"PRIu64"x,%x", type, addr, len);

    if (!(ret = remote_putpkt(remote_out_buf)))
        return RP_VAL_TARGETRET_ERR;

    if ((ret = remote_getpkt(in_buf, sizeof(in_buf), &in_len, -1)) != 0)
        return RP_VAL_TARGETRET_ERR;
    if (in_len < 1)
        return RP_VAL_TARGETRET_NOSUPP;

    if (in_buf[0] != 'O'  ||  in_buf[1] != 'K')
        return RP_VAL_TARGETRET_ERR;

    return RP_VAL_TARGETRET_OK;
}

/* Implementation of target method */
static int remote_connect_worker(int extended,
                                 char *status_string,
                                 size_t status_string_len,
                                 int *can_restart)
{
    char in_buf[RP_PARAM_INOUTBUF_SIZE];
    unsigned int in_len;
    int ret;

    assert(status_string != NULL);
    assert(status_string_len >= 34);
    assert(can_restart != NULL);

    assert(remote_is_open);
    assert(!remote_connected);

    /* If there is something sitting in the buffer we might
       take it as a response to a command, which would be bad. */
    serial_flushinput();

    /* Let us break into remote gdb stub - if remote already in
       gdb stub, it won't harm anything */
    if (remote_break)
        serial_sendbreak();
    else
        serial_putchar(0x03);

    /* If remote is busy sending us something acknowledge it */
    serial_putchar('+');

    if (extended)
    {
        /* Tell remote that we would prefer extended operations */
        if (!(ret = remote_putpkt("!")))
            return RP_VAL_TARGETRET_ERR;

        ret = remote_getpkt(in_buf, sizeof(in_buf), &in_len, -1);
        if (ret != 0  ||  in_buf[0] == 'E')
            return RP_VAL_TARGETRET_ERR;

        /* Any reply is good - XXX */
        remote_extended_ops = TRUE;
    }
    else
    {
        remote_extended_ops = FALSE;
    }

    /* Query latest status */
    if (!(ret = remote_putpkt("?")))
        return RP_VAL_TARGETRET_ERR;

    if ((ret = remote_getpkt(in_buf, sizeof(in_buf), &in_len, -1)) != 0)
        return RP_VAL_TARGETRET_ERR;

    assert(strlen(in_buf) < status_string_len);

    strcpy(status_string, in_buf);

    *can_restart = remote_extended_ops;

    remote_connected = TRUE;

    return RP_VAL_TARGETRET_OK;
}


/* Encode data bin -> ascii */
static int remote_encode_data(const unsigned char *data,
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
        /* We do not have enough space to encode data */
        return  FALSE;
    }

    for (i = 0;  i < data_len;  i++, data++, out += 2)
        remote_encode_byte(*data, out);

    *out = '\0';

    return  TRUE;
}

/* Decode data ascii -> bin */
static int remote_decode_data(const char *in,
                              unsigned char *out,
                              size_t out_size,
                              size_t *len)
{
    size_t count;
    unsigned int byte;

    assert(in != NULL);
    assert(out != NULL);
    assert(out_size > 0);
    assert(len != NULL);

    for (count = 0;  *in != 0  &&  count < out_size;  count++, in += 2, out++)
    {
        if (*(in + 1) == 0)
        {
            /* Odd number of nibbles, discard the last one */
            remote_log(RP_VAL_LOGLEVEL_WARNING,
                       "%s: odd number of nibbles",
                       remote_name);

            if (count == 0)
                return  FALSE;

            *len = count;
            return  TRUE;
        }

        if (!remote_decode_byte(in, &byte))
            return  FALSE;

        *out = byte & 0xff;
    }


    if (*in != '\0')
    {
        /* Input too long */
        return  FALSE;
    }

    *len = count;

    return  TRUE;
}

/* Decode result of process query:
   MMMMMMMMRRRRRRRRRRRRRRRR(TTTTTTTTLLVV..V)*
   where 
      M   reprsents mask
      R   represents ref
      T   represents tag
      L   represents length
      V   represents value */
static int remote_decode_process_query_response(const char *in,
                                                unsigned int *mask,
                                                rp_thread_ref *ref,
                                                rp_thread_info *info)
{
    unsigned int val, len;

    assert(in != NULL);
    assert(mask != NULL);
    assert(ref != NULL);
    assert(info != NULL);

    if (*in++ != 'q')
        return  FALSE;

    if (*in++ != 'Q')
        return  FALSE;

    /* Decode mask */
    if (!remote_decode_4bytes(in, mask))
        return  FALSE;
    in += 8;

    /* Decode reference thread */
    if (!remote_decode_8bytes(in, &ref->val))
        return  FALSE;
    in += 16;

    memset(info, 0, sizeof(*info));

    while (*in)
    {
        /* Decode tag */
        if (!remote_decode_4bytes(in, &val))
            return  FALSE;
        in += 8;

        if (!remote_decode_byte(in, &len))
            return  FALSE;
        in += 2;

        switch (val)
        {
        case RP_BIT_PROCQMASK_THREADID:
            if (!remote_decode_8bytes(in, &info->thread_id.val))
                return  FALSE;
            in += 16;
            break;
        case RP_BIT_PROCQMASK_EXISTS:
            if (len != 1)
                return RP_VAL_TARGETRET_ERR;

            if (*in == '0')
                info->exists = FALSE;
            else
                info->exists = TRUE;

            in++;
            break;
        case RP_BIT_PROCQMASK_DISPLAY:
            assert(len < 255);

            memcpy(info->display, in, len);
            info->display[len] = 0;
            break;
        case RP_BIT_PROCQMASK_THREADNAME:
            assert(len < 255);

            memcpy(info->thread_name, in, len);
            info->thread_name[len] = 0;
            break;
        case RP_BIT_PROCQMASK_MOREDISPLAY:
            assert(len < 255);

            memcpy(info->more_display, in, len);
            info->more_display[len] = 0;
            break;
        default:
            return  FALSE;
        }
    }

    return  TRUE;
}

/* Decode result of list query:
   CCDAAAAAAAAAAAAAAAA(FFFFFFFFFFFFFFFF)*
   where 
      C   reprsents  count
      D   represents done
      A   represents arg thread reference
      F   represents found thread reference(s) */
static int remote_decode_list_query_response(const char *in,
                                             size_t max_num,
                                             size_t *num,
                                             int *done,
                                             rp_thread_ref *arg,
                                             rp_thread_ref *found)
{
    int ret;
    int unsigned val;
    size_t count, i;

    assert(num != NULL);
    assert(done != NULL);
    assert(arg != NULL);
    assert(found != NULL);

    if (*in++ != 'q')
        return  FALSE;

    if (*in++ != 'M')
        return  FALSE;

    /* Decode num */
    ret = remote_decode_byte(in, &count);
    if (!ret  ||  count > max_num)
        return  FALSE;
    in += 2;

    *num = count;

    /* Decode done */
    if (!remote_decode_nibble(in, &val))
        return  FALSE;
    in += 1;

    *done = (val == 0) ? 0 : 1;

    /* Decode argument thread */
    if (!remote_decode_8bytes(in, &arg->val))
        return  FALSE;
    in += 16;

    for (i = 0;  i < count;  i++, found++, in += 16)
    {
        if (!remote_decode_8bytes(in, &found->val))
            return  FALSE;
    }

    /* If there are some bytes after the data,
       we will ignore it. */
    return  TRUE;
}

/* Decode nibble */
static int remote_decode_nibble(const char *in, unsigned int *nibble)
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
static int remote_decode_byte(const char *in, unsigned int *byte)
{
    unsigned int ls_nibble;
    unsigned int ms_nibble;

    if (!remote_decode_nibble(in, &ms_nibble))
        return  FALSE;

    if (!remote_decode_nibble(in + 1, &ls_nibble))
        return  FALSE;

    *byte = (ms_nibble << 4) + ls_nibble;

    return  TRUE;
}

/* Decode 4 bytes */
static int remote_decode_4bytes(const char *in, unsigned int *val)
{
    unsigned int tmp;
    unsigned int byte;
    int i;

    tmp = 0;

    for (i = 0;  i < 4;  i++, in += 2)
    {
        if (!remote_decode_byte(in, &byte))
            return  FALSE;

        tmp = (tmp << 8) + byte;
    }

    *val = tmp;

    return  TRUE;
}

/* Decode 8 bytes */
static int remote_decode_8bytes(const char *in, uint64_t *val)
{
    unsigned int byte;
    uint64_t tmp;
    int i;

    tmp = 0;

    for (i = 0;  i < 8;  i++, in += 2)
    {
        if (!remote_decode_byte(in, &byte))
            return  FALSE;
        tmp = (tmp << 8) + byte;
    }

    *val = tmp;

    return  TRUE;
}

/* Encode byte */
static void remote_encode_byte(unsigned int val, char *out)
{
    static char hex[] = "0123456789abcdef";

    assert(val <= 0xff);
    assert(out != NULL);

    *out = hex[(val >> 4) & 0xf];
    *(out + 1) = hex[val & 0xf];
}

/* Send packet */
static int remote_putpkt(const char *buf)
{
    int i;
    int ret;
    size_t len;
    size_t in_len;
    unsigned char csum;
    char *d;
    const char *s;
    char buf2[RP_PARAM_INOUTBUF_SIZE + 4];
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
        ret = remote_getpkt(in_buf, sizeof(in_buf), &in_len, 0);
    while (ret != 1);

    for (i = 0;  i < 3;  i++)
    {
        remote_log(RP_VAL_LOGLEVEL_DEBUG2,
                   "%s: sending packet: %d bytes: %s...",
                   remote_name,
                   len,
                   buf2);

        ret = serial_write(buf2, len);

        if (!ret)
        {
            /* Something went wrong */
            remote_log(RP_VAL_LOGLEVEL_DEBUG, "%s: write failed", remote_name);
            return 0;
        }

        /* Now look for an ACK from GDB */
        ret = remote_getpkt(in_buf, sizeof(in_buf), &in_len, -1);
        if (ret == -1)
        {
            remote_log(RP_VAL_LOGLEVEL_DEBUG, "%s: read of ACK failed", remote_name);
            return  FALSE;
        }
        if (ret == ACK)
        {
            remote_log(RP_VAL_LOGLEVEL_DEBUG2, "%s: got ACK", remote_name);
            return  TRUE;
        }
    }
    /* We tried three times and did not get anything back */
    remote_log(RP_VAL_LOGLEVEL_DEBUG,
               "%s: putpkt: too many errors on the current packet. Giving up",
               remote_name);
    return  FALSE;
}

/* Read a packet from the remote machine, with error checking,
   and store it in buf. */
static int remote_getpkt(char *buf, size_t buf_len, size_t *len, int timeout)
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
    if (timeout < 0)
        timeout = remote_timeout;
    for (;;)
    {
        if (state == 0)
            c = serial_getchar(timeout);
        else
            c = serial_getchar(1000);
        if (c == RP_VAL_SERIALGETCHARRET_ERR)
        {
            remote_log(RP_VAL_LOGLEVEL_DEBUG,
                       "%s: error while reading from remote device",
                       remote_name);
            return  -1;
        }
        if (c == RP_VAL_SERIALGETCHARRET_TMOUT)
        {
            remote_log(RP_VAL_LOGLEVEL_DEBUG,
                       "%s: timeout while reading from remote device",
                       remote_name);
            return  1;
        }

        if (c == '$'  &&  state != 0)
        {
            /* Unexpected start of packet marker in mid-packet. */
            remote_log(RP_VAL_LOGLEVEL_DEBUG,
                       "%s: unexpected new packet",
                       remote_name);
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
                remote_log(RP_VAL_LOGLEVEL_DEBUG, "%s: Control-C received", remote_name);
                return  '\3';
            }
            else if (c == '+')
            {
                /* An ACK to one of our packets */
                /* We don't use sequence numbers, so we shouldn't expect a
                   sequence number after this character. */
                remote_log(RP_VAL_LOGLEVEL_DEBUG, "%s: ACK received", remote_name);
                return  ACK;
            }
            else if (c == '-')
            {
                /* A NAK to one of our packets */
                /* We don't use sequence numbers, so we shouldn't expect a
                   sequence number after this character. */
                remote_log(RP_VAL_LOGLEVEL_DEBUG, "%s: NAK received", remote_name);
                return  NAK;
            }
            else
            {
                remote_log(RP_VAL_LOGLEVEL_DEBUG, "%s: we got junk - 0x%X", remote_name, c & 0xFF);
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
                remote_log(RP_VAL_LOGLEVEL_DEBUG,
                           "%s: received excessive length packet",
                           remote_name);
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
                remote_log(RP_VAL_LOGLEVEL_DEBUG,
                           "%s: received a packet that is too long",
                           remote_name);
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
                remote_log(RP_VAL_LOGLEVEL_DEBUG,
                           "%s: bad checksum character %c",
                           remote_name,
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
                remote_log(RP_VAL_LOGLEVEL_DEBUG,
                           "%s: bad checksum character %c",
                           remote_name,
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
                serial_putchar('+');
                if (seq_valid)
                {
                    serial_putchar(seq[0]);
                    serial_putchar(seq[1]);
                }

                remote_log(RP_VAL_LOGLEVEL_DEBUG, "%s: packet received: %s", remote_name, buf);
                return  0;
            }
            remote_log(RP_VAL_LOGLEVEL_DEBUG,
                       "%s: bad checksum calculated=0x%x received=0x%x",
                       remote_name,
                       rx_csum,
                       calc_csum);
            state = 0;
            continue;
        }
    }
    /* We can't actually get here */
    return  -1;
}
