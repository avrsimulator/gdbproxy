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
   
   Misc definitions used by gdbproxy.
   
   $Id: rpmisc.h,v 1.2 2010/02/10 11:33:54 vapier Exp $ */


void dbg_sock_init(void);
void dbg_sock_cleanup(void);
int  dbg_sock_open(unsigned int *port);
void dbg_sock_close(void);
int  dbg_sock_accept(void);
int  dbg_sock_readchar(int ms);
int  dbg_listen_sock_open(unsigned int *port);

/* Return values for readchar: either character
   code or one of the following*/
#define RP_VAL_MISCREADCHARRET_TMOUT (-2)
#define RP_VAL_MISCREADCHARRET_ERR   (-1)

void dbg_sock_putchar(int c);
int  dbg_sock_write(unsigned char *buf, size_t len);

log_func rp_env_init(char *name, int do_daemon);
