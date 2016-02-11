/* Copyright (C) 1999-2001 Quality Quorum, Inc.
   Copyright (C) 2002 Chris Liechti and Steve Underwood.
 
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
   
   Implementation of serial connection used on target side.
  
   Exported Data:
     None
  
   Imported Data:
     None     
  
   Static Data:
     serial_xXXX    - static data representing status of serial 
  
   Global Functions:  
     serial_xXXX    - see definitions in serial_xXXX
  
   Static Functions:  
     None
  
   
   $Id: serial.c,v 1.1.1.1 2005/09/04 08:57:15 coppice Exp $ */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <assert.h>

#if defined(WIN32)
#include <windows.h>
#include <stdint.h>
#else
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <errno.h>

#ifdef HAVE_FCNTL_H
#include <fcntl.h>
#endif /* HAVE_FCNTL_H */

#ifdef HAVE_UNISTD_H
#include <unistd.h>
#include <sys/types.h>
#endif /* HAVE_UNISTD_H */

#ifdef HAVE_SYS_TIME_H
#include <sys/time.h>
#else
#include <time.h>
#endif /* HAVE_SYS_TIME_H */

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#endif

#include "serial.h"

#if !defined(FALSE)
#define FALSE 0
#endif
#if !defined(TRUE)
#define TRUE (!FALSE)
#endif

#define SERIAL_BUFF_SIZE    8192

/* Serial device file descriptor */
#if defined(WIN32)
static HANDLE serial_hand = INVALID_HANDLE_VALUE;
#else
static int serial_fd = -1;
#endif

/* Input buffer */
static char serial_in_buf[SERIAL_BUFF_SIZE];

/* Pointer to the current char in buffer */
static char *serial_in_ptr = serial_in_buf;

/* Amount of characters in buffer */
static int serial_in_count = 0;

/* Check check whether baud is acceptable rate */
int serial_check_baud(int baud)
{
    switch (baud)
    {
    case 50:
    case 75:
    case 110:
    case 134:
    case 150:
    case 1800:
    case 2400:
    case 4800:
    case 9600:
    case 19200:
    case 38400:
    case 57600:
    case 115200:
#if defined(B230400)
    case 230400:
#endif
#if defined(B460800)
    case 460800:
#endif
#if defined(B921600)
    case 921600:
#endif
        return  TRUE;
    }

    return  FALSE;
}

/* Open serial connection */
int serial_open(char *name, int baud)
{
#if defined(WIN32)
    DCB dcb;
    BOOL res;
    COMMTIMEOUTS ctm;

    assert(name != NULL);
    assert(serial_hand == INVALID_HANDLE_VALUE);
    /* Open port with exclusive access and no security attributes */
    serial_hand = CreateFile(name,
    			     GENERIC_READ | GENERIC_WRITE,
			     0,
			     NULL,
                             OPEN_EXISTING,
			     FILE_ATTRIBUTE_NORMAL,
			     NULL);
    if (serial_hand == INVALID_HANDLE_VALUE)
        return  FALSE;

    /* Set raw mode */
    if (!(res = GetCommState(serial_hand, &dcb)))
    {
        CloseHandle(serial_hand);
        serial_hand = INVALID_HANDLE_VALUE;
        return  FALSE;
    }

    dcb.BaudRate        = baud;
    dcb.fBinary         = TRUE;
    dcb.fParity         = FALSE;
    dcb.fOutxCtsFlow    = FALSE;
    dcb.fOutxDsrFlow    = FALSE;
    dcb.fDtrControl     = DTR_CONTROL_ENABLE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fOutX           = FALSE;
    dcb.fInX            = FALSE;
    dcb.fErrorChar      = FALSE;
    dcb.fNull           = FALSE;
    dcb.fRtsControl     = RTS_CONTROL_ENABLE;
    dcb.fAbortOnError   = FALSE;
    dcb.ByteSize        = 8;
    dcb.Parity          = NOPARITY;
    dcb.StopBits        = ONESTOPBIT;

    if (!(res = SetCommState(serial_hand, &dcb)))
    {
        CloseHandle(serial_hand);
        serial_hand = INVALID_HANDLE_VALUE;
        return  FALSE;
    }

    /* Reset timeouts */
    ctm.ReadIntervalTimeout         = 20; /* Allow 20ms between characters */
    ctm.ReadTotalTimeoutConstant    = 0;  /* This one will be changed later */
    ctm.ReadTotalTimeoutMultiplier  = 0;
    ctm.WriteTotalTimeoutConstant   = 0;
    ctm.WriteTotalTimeoutMultiplier = 0;

    if (!(res = SetCommTimeouts(serial_hand, &ctm)))
    {
        assert(0);
        CloseHandle(serial_hand);
        serial_hand = INVALID_HANDLE_VALUE;
        return  FALSE;
    }
#else
    int ret;
    struct termios termios;

    assert(serial_fd == -1);

    if (strchr (name, ':') != NULL)
    {
        /* We do not support TCP connections right now */
        return  FALSE;
    }

    if ((serial_fd = open(name, O_RDWR)) < 0)
    {
        serial_fd = -1;
        return  FALSE;
    }

    /* Set raw mode */
    do
    {
        ret = tcgetattr(serial_fd, &termios);
    }
    while (ret < 0  &&  errno == EINTR);

    if (ret < 0)
    {
        close(serial_fd);
        serial_fd = -1;
        return  FALSE;
    }

    termios.c_iflag     = 0;
    termios.c_oflag     = 0;
    termios.c_lflag     = 0;
    termios.c_cflag     = CLOCAL | CREAD | CS8;
    termios.c_cc[VMIN]  = 0;
    termios.c_cc[VTIME] = 0;

    switch (baud)
    {
    case 50:
        ret = cfsetospeed(&termios, B50);
        break;
    case 75:
        ret = cfsetospeed(&termios, B75);
        break;
    case 110:
        ret = cfsetospeed(&termios, B110);
        break;
    case 134:
        ret = cfsetospeed(&termios, B134);
        break;
    case 150:
        ret = cfsetospeed(&termios, B150);
        break;
    case 200:
        ret = cfsetospeed(&termios, B200);
        break;
    case 1800:
        ret = cfsetospeed(&termios, B1800);
        break;
    case 2400:
        ret = cfsetospeed(&termios, B2400);
        break;
    case 4800:
        ret = cfsetospeed(&termios, B4800);
        break;
    case 9600:
        ret = cfsetospeed(&termios, B9600);
        break;
    case 19200:
        ret = cfsetospeed(&termios, B19200);
        break;
    case 38400:
        ret = cfsetospeed(&termios, B38400);
        break;
    case 57600:
        ret = cfsetospeed(&termios, B57600);
        break;
    case 115200:
        ret = cfsetospeed(&termios, B115200);
        break;
#if defined(B230400)
    case 230400:
        ret = cfsetospeed(&termios, B230400);
        break;
#endif
#if defined(B460800)
    case 460800:
        ret = cfsetospeed(&termios, B460800);
        break;
#endif
#if defined(B921600)
    case 921600:
        ret = cfsetospeed(&termios, B921600);
        break;
#endif
    default:
        ret = -1;
        break;
    }

    if (ret < 0)
    {
        close(serial_fd);
        serial_fd = -1;
        return  FALSE;
    }

    do
    {
        ret = tcsetattr(serial_fd, TCSANOW, &termios);
    }
    while (ret < 0  &&  errno == EINTR);

    if (ret < 0)
    {
        close(serial_fd);
        serial_fd = -1;
        return  FALSE;
    }
#endif

    serial_in_ptr = serial_in_buf;
    serial_in_count = 0;
    return  TRUE;
}

/* Close serial connection */
void serial_close(void)
{
#if defined(WIN32)
    if (serial_hand != INVALID_HANDLE_VALUE)
    {
        CloseHandle(serial_hand);
        serial_hand = INVALID_HANDLE_VALUE;
    }
#else
    if (serial_fd >= 0)
    {
        close(serial_fd);
        serial_fd = -1;
    }
#endif
}

/* Receive character, timeout is in in milliseconds, -1 means wait
   forever.  Returns error code if timeout or error, and charcter code 
   othervise */
int serial_getchar(int timeout)
{
    assert(timeout == -1  ||  timeout >= 0);

    if (serial_in_count > 0)
    {
        serial_in_count--;
        return (*serial_in_ptr++ & 0xff);  /* 8-bit transparent */
    }

    serial_in_count = serial_read(serial_in_buf, sizeof(serial_in_buf), timeout);
    if (serial_in_count < 0)
        return  RP_VAL_SERIALGETCHARRET_ERR;
    if (serial_in_count == 0)
        return  RP_VAL_SERIALGETCHARRET_TMOUT;

    serial_in_ptr = serial_in_buf;
    serial_in_count--;

    return (*serial_in_ptr++ & 0xff);
}

/* Receive character, timeout is in in milliseconds, -1 means wait
   forever.  Returns error code if timeout or error, and charcter code 
   othervise */
int serial_read(char *buf, size_t len, int timeout)
{
    int ret;
#if defined(WIN32)
    COMMTIMEOUTS ctm;
    DWORD dtmp;
    BOOL res;

    assert(serial_hand != INVALID_HANDLE_VALUE);
    assert(timeout == -1  ||  timeout >= 0);

    if (!(res = GetCommTimeouts(serial_hand, &ctm)))
    {
        assert(0);
        return  -1;
    }

    assert(ctm.ReadIntervalTimeout == 20);
    assert(ctm.ReadTotalTimeoutMultiplier == 0);
    assert(ctm.WriteTotalTimeoutConstant == 0);
    assert(ctm.WriteTotalTimeoutMultiplier == 0);

    ctm.ReadTotalTimeoutConstant = timeout;

    if (!(res = SetCommTimeouts(serial_hand, &ctm)))
    {
        assert(0);
        return  -1;
    }

    if (!(res = ReadFile(serial_hand, buf, len, &dtmp, NULL)))
        return  -1;

    ret = dtmp;
#else
    struct timeval tv;
    struct timeval end;
    struct timeval cur;
    struct timezone tz;
    fd_set rset;

    assert(serial_fd >= 0);
    assert(timeout == -1  ||  timeout >= 0);

    if (timeout != -1)
    {
        /* It is quite possible that targets would install
           some signal handlers, so we have to ignore EINTR
           in all cases */

        /* Let us calculate the timeout */
        tv.tv_sec = timeout/1000;
        tv.tv_usec = (timeout%1000)*1000;

        /* Let us get current time */
        if ((ret = gettimeofday(&cur, &tz)) < 0)
            return -1;

        /* Let us figure out end time */
        end.tv_sec  = cur.tv_sec + tv.tv_sec;
        end.tv_usec = cur.tv_usec + tv.tv_usec;
        if (end.tv_usec >= 1000000)
        {
            end.tv_sec++;
            end.tv_usec -= 1000000;
        }

        assert(end.tv_usec < 1000000);

        for (;;)
        {
            FD_ZERO(&rset);
            FD_SET(serial_fd, &rset);

            if ((ret = select(serial_fd + 1, &rset, NULL, NULL, &tv)) > 0)
            {
                assert(FD_ISSET(serial_fd, &rset));
                break;
            }

            if (ret == 0)
            {
                /* Timeout */
                return  0;
            }

            if (ret < 0  &&  errno != EINTR)
                return  -1;

            /* We have been interrupted by a signal */

            /* We have to recalculate the timeout */
            if ((ret = gettimeofday(&cur, &tz)) < 0)
                return  -1;

            if (cur.tv_sec > end.tv_sec
	    	||
                (cur.tv_sec == end.tv_sec  &&  cur.tv_usec >= end.tv_usec))
            {
                tv.tv_sec  = 0;
                tv.tv_usec = 0;

                continue;
            }

            tv.tv_sec = end.tv_sec - cur.tv_sec;

            if (cur.tv_usec <= end.tv_usec)
            {
                tv.tv_usec = end.tv_usec - cur.tv_usec;
            }
            else
            {
                assert(tv.tv_sec > 0);

                tv.tv_sec--;
                tv.tv_usec = end.tv_usec + 1000000 - cur.tv_usec;
            }
        }
    }
    else
    {
        /* Wait forever (if necessary) */
        for (;;)
        {
            FD_ZERO(&rset);
            FD_SET(serial_fd, &rset);

            if ((ret = select(serial_fd + 1, &rset, NULL, NULL, NULL)) > 0)
            {
                assert(FD_ISSET(serial_fd, &rset));
                break;
            }

            if (ret < 0  &&  errno != EINTR)
                return  -1;

            assert(ret != 0);
            assert(errno == EINTR);
        }
    }

    /* We have to ignore EINTR here too */
    do
    {
        ret = read(serial_fd, buf, len);
    }
    while (ret < 0  &&  errno == EINTR);
#endif

    return ret;
}

/* Send character */
void serial_putchar(int c)
{
    char b[4];
#if defined(WIN32)
    DWORD dtmp;

    assert(serial_hand != INVALID_HANDLE_VALUE);

    b[0] = c & 0xff;

    WriteFile(serial_hand, b, 1, &dtmp, NULL);
#else
    int ret;

    assert(serial_fd >= 0);

    b[0] = c & 0xff;

    do
    {
        ret = write(serial_fd, b, 1);
    }
    while (ret < 0  &&  errno == EINTR);
#endif
}

/* Send buffer */
int serial_write(char *buf, size_t len)
{
#if defined(WIN32)
    BOOL res;
    DWORD dtmp;

    assert(serial_hand != INVALID_HANDLE_VALUE);
    assert(buf != NULL);
    assert(len > 0);

    res = WriteFile(serial_hand, buf, len, &dtmp, NULL);
    if (!res  ||  dtmp != len)
        return  FALSE;
#else
    int ret;

    assert(buf != NULL);
    assert(len > 0);
    assert(serial_fd >= 0);

    do
    {
        ret = write(serial_fd, buf, len);
    }
    while (ret < 0  &&  errno == EINTR);

    if (ret != (int) len)
        return  FALSE;
#endif
    return  TRUE;
}

/* Discard input */
void serial_flushinput(void)
{
#if defined(WIN32)
    assert(serial_hand != INVALID_HANDLE_VALUE);

    PurgeComm(serial_hand, PURGE_RXCLEAR);
#else
    assert(serial_fd >= 0);

    tcflush(serial_fd, TCIFLUSH);
#endif
    serial_in_count = 0;
}

/* Send break */
void serial_sendbreak(void)
{
#if defined(WIN32)
    assert(serial_hand != INVALID_HANDLE_VALUE);

    SetCommBreak(serial_hand);
    Sleep(500);
    ClearCommBreak(serial_hand);
#else
    assert(serial_fd >= 0);

    tcsendbreak(serial_fd, 0);
#endif
}
