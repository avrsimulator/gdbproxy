#
#
# Copyright (C) 1999-2001 Quality Quorum, Inc.
# Copyright (C) 2002 Chris Liechti and Steve Underwood
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#  3. The name of the author may not be used to endorse or promote products
#     derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
# EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# QQI can be contacted as qqi@world.std.com
# 
#
# This is a part of gdbproxy - a remote proxy server for gdb 
#

AUTOMAKE_OPTIONS = gnu

bin_PROGRAMS = gdbproxy

gdbproxy_SOURCES = \
    copying.c \
    init.c \
    remote.c \
    gdbproxy.c \
    rpmisc.c \
    serial.c \
    target_skeleton.c \
    gdbproxy.h \
    rpmisc.h \
    serial.h

EXTRA_gdbproxy_SOURCES = getopt.h

gdbproxy_LDADD = @LIBOBJS@
gdbproxy_DEPENDENCIES = @LIBOBJS@

info_TEXINFOS = gdbproxy.texi

man_MANS = gdbproxy.1

EXTRA_DIST = gdbproxy.1 Makefile.am bootstrap getopt.c getopt1.c

CLEANFILES = configure.scan res tmp.tmp

MAINTAINERCLEANFILES = aclocal.m4 configure config.h.in stamp-h.in Makefile.in

