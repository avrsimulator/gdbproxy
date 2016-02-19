/**
 * \file
 *
 * \brief Model dll manager class
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

#ifndef MODELMGR_H_
#define MODELMGR_H_

#ifdef __unix__
#include <dlfcn.h>
#else // assume windows
#include <windows.h>
#endif // __unix__
#include "pinmodel.h"
#include "model.h"
#include "model_error.h"

// Function pointer types
typedef Model* ModelConstructor(const char*, ModelError *err);
typedef void   ModelDestructor(Model*);
typedef unsigned ModelApiVersion();
typedef PinModel* PinModel_f(Model*);
typedef void SetDllName(const char*);          // AVRSIM-370

#ifdef __unix__
typedef void*    Handle;
// this thing was originally written for win32, Linux semantic is basically
// the same, translate names.
#define LoadLibrary(name)           dlopen(name, RTLD_NOW)
#define GetProcAddress(handle, sym) dlsym(handle, sym)
#define FreeLibrary(handle)         (dlclose(handle) == 0)
#else
typedef HMODULE  Handle; 
#endif

//
// Model dll manager class. Used to load and unload Model dll's.  This
// class enforces only one model be loaded at any time (muliple models
// may be loaded using multiple manager instances, not tested).
//
// NOTICE: the returned model ptr can ONLY be safely destroyed by means of
// calling the unload() method, or destroying the manager object.
//
class ModelMgr
{
 public:
    ModelMgr();
    
    // constructor loading named dll.
    ModelMgr(const char *dllname, const char *device = 0,
             bool deferUnload = true); // bug 12425
    ~ModelMgr();                       // destructor implicitly unloads model

    // load new dll (implicitly unload prev)
    // 'device' arg can be used to select device in model dll's supporting
    // multiple configurations run-time (using OTP fuses or similar)
    Model *load(const char *dllname, const char *device = 0);
    const ModelError &getError() const { return m_error; }
    bool unload(); 
    Model *model() {return m_pModel;};  // return current Model ptr
    PinModel *pinmodel() {return m_pPinModel;}; // return current pin model or NULL
    char *errorText();
    unsigned getModelApiVersion() {return m_fVer? m_fVer() : 0;};

    void setDeferUnload(bool deferUnload) {m_deferUnload = deferUnload;};

    void* getsym(const char *sym);    // For test programs only; dont use.

 private:
    void init();
    bool _unload();   // internal, unconditional unload
    void err(int ercode, const char *errstring);
    void cleanup(Model *m);
    Model *m_pModel;
    PinModel *m_pPinModel;
    Handle m_handle;
    ModelConstructor *m_fCtor;
    ModelDestructor *m_fDtor;
    ModelApiVersion *m_fVer;
    PinModel_f  *m_fPin;
    char m_errmsg[120];
    char m_device[64];
    bool m_deferUnload;
    char m_dllname[512];
    ModelError m_error;
};

#endif //MODELMGR_H_
