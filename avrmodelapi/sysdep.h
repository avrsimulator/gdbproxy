/**
 * \file
 *
 * \brief System-dependent definitions (Unix/Windows)
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

/**
 * We try to hide as much as possible of ugly platform-specific #ifdef's
 * etc  here.
 *
 * Platforms tested: 
 * Windows/MSVSC++ 16, Windows/Cygwin/gcc-3.3.1, Ubuntu 12.10/gcc-4.6.3
 * Windows / mingw32 gcc-4.7.1
 */

#define  __STDC_FORMAT_MACROS  // Needed for PRI?64 macros.

#ifndef SYSDEP_H_
#define SYSDEP_H_

#if defined (__unix__)

#define SLASH '/'

/* Not sure the below holds for all unixes, but at least for RH7.2 */
#ifndef stricmp
#define stricmp(S1,S2) strcasecmp((S1), (S2))
#endif
#ifndef strnicmp
#define strnicmp(S1,S2,N) strncasecmp((S1), (S2),(N))
#endif

#include <unistd.h>   /* Unix only */

#ifndef __CYGWIN__
#ifndef sun
#include <libgen.h>   /* basename (glibc at least), not present in cygwin */
#endif
#endif

/* alloca() differs, see WIN32 branch */
#include <alloca.h>

#else // assume windows.
#ifndef WIN32
#define WIN32
#endif

#define SLASH '\\'

#ifdef _MSC_VER
// The following is for MSVC compiler. if _MSC_VER is not defined,
// assume gcc (mingw32).

/* The below suppresses a long useless STL warning *if* sysdep.h is included
 * _first_.
 */
#pragma warning(disable: 4786)

/* alloca() is sometimes _alloca() in MSVC++ and uses different header. */
#include <malloc.h>
#ifndef alloca
#define alloca(SZ) _alloca(SZ)
#endif /*alloca*/

#if _MSC_VER < 1800  // Not sure about the version. 1900 is VS2015
/* MSVC++ uses _snprintf() rather than snprintf() */
#ifndef snprintf
/*#define snprintf(BUF, SZ, FMT, ...) _snprintf(BUF, SZ, FMT, __VA_ARGS__) */
#define snprintf _snprintf  // MSVC++ doesn't grok the above, sigh...
#endif /*snprintf*/
#endif    

# if _MSC_VER < 1500
/* MSVC++ < 15 (VS2008) uses _vsnprintf() rather than vsnprintf() */
#ifndef vnsprintf
#define vsnprintf(BUF, SZ, FMT, VA) _vsnprintf(BUF, SZ, FMT, VA)
#endif /*_vnsprintf */
#endif // MSC_VER

#if 0  // *sigh* microsoft not consistent with their own documentation :(
#ifndef stricmp
#define stricmp(S1,S2) _stricmp((S1), (S2))
#endif
#ifndef strnicmp
#define strnicmp(S1,S2,N) _strnicmp((S1), (S2),(N))
#endif
#endif

#define strerror_r(errnum, bf, sz) strerror_s(bf, sz, errnum)
#if _MSC_VER < 1800  // Not sure about the version. 1900 is VS2015
#define strtoull(s, ptr, base) _strtoui64(s, ptr, base)
#endif

#else  // gcc assumed. 4.7.1 uses malloc.h instead of alloca.h
#include <malloc.h>

#endif // _MSC_VER


#ifndef popen
#define popen(F,M) _popen((F),(M))
#define pclose(F) _pclose((F))
#endif


#endif // __unix__

#endif /* SYSDEP_H_ */
