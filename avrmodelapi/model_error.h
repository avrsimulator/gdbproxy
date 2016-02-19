/**
 * \file
 *
 * \brief Model error information
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

#ifndef _MODEL_ERROR_H
#define _MODEL_ERROR_H

// Notice: this structure is passed through a C interface and
// we therefore cannot do any clever C++ tricks.
//
// This structure is partially filled in by model DLL and partially by
// ModelMgr, depending on where error is detected.

// Error codes.
#define MODEL_NO_ERROR             0
#define MODEL_UNKNOWN_ERROR        (-1)
// Error conditions detected by ModelMgr
#define MODEL_LOAD_ERROR           (-2)  // DLL load failure.
#define MODEL_MALFORMED            (-3)  // Model lacks required symbols
#define MODEL_VERSION_ERROR        (-4)  // Incompatible model API version.
#define MODEL_UNLOAD_ERROR         (-5)  // DLL unload failure.
#define MODEL_NOT_LOADED           (-6)  // Unload request when not loaded.
#define MODEL_NONEXISTENT          (-7)  // Model does not exist
#define MODEL_VTOC_INIT_ERROR      (-8)  // Occurs when loading multiple VTOC
                                         // models in Windows.
// Error conditions detected by model DLL. 
#define MODEL_LICENSE_ERROR        (-100)
#define MODEL_UNSUPPORTED_DEVICE   (-101) // Device not supported by this model


// Buffer sizes.
#define M_STORESIZE 1024
#define M_PATHSIZE  512

typedef struct ModelErrorStruct
{
    int   error_code;          // Numeric error code, see above.
    int   lic_errno;           // Error mumber from license manager, if relevant
    // DLL name filled in by modelmgr. Model does not have this.
    char  *dllname;            // File name of offending DLL, if available.
    // Model fills in these using the 'store' buffer.
    // ModelMgr is free to use separate storage.
    char  *errstring;          // Error string
    char  *device;             // Device name requested
    char  *product;            // Product name, if relevant
    char  *version;            // Product version, if relevant
    char  *licpath;            // License file path, if relevant
    char  *licmgr;             // License manager information
    char  store[M_STORESIZE+1];// Storage buffer for all the strings
                               // except dllname.
} ModelError;


#endif // _MODEL_ERROR_H
