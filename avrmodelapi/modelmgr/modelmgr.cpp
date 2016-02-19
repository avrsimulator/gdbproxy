/**
 * \file
 *
 * \brief Model dll manager class implementation
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

#include "sysdep.h"

#include "modelmgr.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef __unix__
// assume win32
#define snprintf _snprintf
#endif

#ifdef DEBUG
#define DBG(X) X
#else
#define DBG(X)
#endif

#ifdef _MSC_VER
// The following is for MSVC compiler. if _MSC_VER is not defined,
// assume gcc (mingw32).

#pragma warning (push)
// warning C4996: 'getenv': This function or variable may be unsafe.
// warning C4996: 'putenv': The POSIX name for this item is deprecated. Instead, use the ISO C++ conformant name: _putenv.
#pragma warning (disable: 4996)
#endif

ModelMgr::ModelMgr()
{
    m_deferUnload = true;
    init();
}

ModelMgr::ModelMgr(const char *dllname, const char *device,
                   bool deferUnload)
{
    m_deferUnload = deferUnload;
    init();
    load(dllname, device);
}

ModelMgr::~ModelMgr()
{
    _unload();  // Unconditionally unload here!!!
}

void ModelMgr::init()
{
    m_handle = NULL;
    m_pModel = NULL;
    m_pPinModel = NULL;
    m_fCtor = NULL;
    m_fDtor = NULL;
    m_fVer = NULL;
    m_fPin = NULL;
    *m_errmsg = '\0';
    *m_dllname = '\0';
    *m_device = '\0';
    memset(&m_error, 0, sizeof(m_error));

    // Carbon bug ID 13218 workaround. Disable Carbon's internal
    // (buggy, leaking like a sieve) memory manager.  In AVR Studio 4 it
    // doesn't even help to set it here (probably because the CRT dll
    // used by carbon runtime is alredy loaded at this point and has
    // its own copu of the enviroment). Solved by AVR Studio installer
    // setting it in registry.
    if (!getenv("CARBON_MEM_DISABLE")) putenv((char*)"CARBON_MEM_DISABLE=1");
}


void ModelMgr::cleanup(Model *m)
{
    uint64_t ncores;
    Core *c;

    // Make sure model does not run and does not call
    // back, this duplicated from unload() because
    // we can not be sure unload was called first.
    // Bug 12835: Clear any callbacks and
    // breakpoints,
    m->stop();                           // Bug 12835
    m->removeCycleCallback(Core::REMOVE_ALL);   
    if (m->getIntProperty(P_NumCores, &ncores) > 0)
    {
        for (unsigned i = 0; i < ncores; i++)
        {
            if ((c = m->getCore(i)))
            {
                c->removeStepCallback(Core::REMOVE_ALL);
                c->removeBreakpoint(Core::REMOVE_ALL);
            }
            else fprintf(stderr,"ModelMgr: Bogus core %u\n",
                         i);
        }
    }
    else if ((c = m->getCore(0)))
    {
        // in case model reports wrong property.
        printf("ModelMgr: Model reported no cores but has"
               " at least one?");
        c->removeStepCallback(Core::REMOVE_ALL);
        c->removeBreakpoint(Core::REMOVE_ALL);
    }
    else fprintf(stderr, "ModelMgr: No cores\n");

    m->reset(Reset_por);
}


Model *ModelMgr::load(const char *dllname, const char *device)
{
    if (m_handle && m_pModel) 
    {
        if (m_deferUnload)
        {
            // Bug 12425: If both DLL name and device name is the same as
            // the last time, skip the unload-load cycle and just give
            // the model a power-on reset instead. This to retain NVM data
            // stored in the model.
            char lastdevice[100];
            if (device 
                && m_pModel->getStringProperty(P_DeviceName, sizeof(lastdevice),
                                              lastdevice))
            {
                if (stricmp(m_dllname, dllname) == 0
                    && stricmp(lastdevice, device) == 0)
                {
                    if (m_pModel)
                    {
                        cleanup(m_pModel);
                        return m_pModel;
                    }
                }
            }
        }
    }
    // If we get here, really unload previous model if any (Bug 12425/12835)
    _unload();
    init();
    strncpy(m_dllname, dllname, sizeof(m_dllname));
    if (device) strncpy(m_device, device, sizeof(m_device));

    if ((m_handle = LoadLibrary(dllname)) == NULL)
    {
        err(MODEL_LOAD_ERROR, errorText()); 
        return NULL;
    }
    if ((m_fCtor = (ModelConstructor*)GetProcAddress(m_handle, "model_ctor")) == NULL)
    {
        err(MODEL_MALFORMED, "Missing constructor symbol: model_ctor");
        FreeLibrary(m_handle);
        m_handle = NULL;
        return NULL;
    }    
    if ((m_fDtor = (ModelDestructor*)GetProcAddress(m_handle, "model_dtor")) == NULL)
    {
        
        err(MODEL_MALFORMED, "Missing destructor symbol: model_dtor");
        FreeLibrary(m_handle);
        m_handle = NULL;
        m_fCtor = NULL;
        return NULL;
    }
    if ((m_fVer = (ModelApiVersion*)GetProcAddress(m_handle, "model_api_ver")) == NULL)
    {
        err(MODEL_MALFORMED, "Missing symbol: model_api_ver");
        // keep the model handle until we have verified version, below.
      
    }
    SetDllName *fSetDll;
    if ((fSetDll = (SetDllName*)GetProcAddress(m_handle, "model_set_dllname")))
        fSetDll(m_dllname);     // AVRSIM-370

    m_pModel = m_fCtor(device, &m_error);
    //DBG(printf("Model %s loaded\n", m_pModel->getDeviceName()));

    if (!m_pModel)
    {
        // Error details except for dllname expected to be set by model
        m_error.dllname = m_dllname;
        // Parts of the loading failed, just unload everything.
        _unload();
        return NULL;
    }

    // if no device specified model selects default, get it from model.
    if (!m_device || *m_device == '\0')   
        m_pModel->getStringProperty(P_DeviceName, sizeof(m_device), m_device);

    // Check API version.
    unsigned ver;
    if ((ver = getModelApiVersion()) != MODEL_API_VERSION)
    {
        // @@FIXME: get rid of m_errmsg?
        snprintf(m_errmsg, sizeof(m_errmsg), "Incompatible model API version: "
                 "expected 0x%06x, got 0x%06x\n",
                 MODEL_API_VERSION, ver);
        err(MODEL_VERSION_ERROR, m_errmsg);
        return NULL;
    }

    // Missing pin model is not fatal.
    if ((m_fPin = (PinModel_f*)GetProcAddress(m_handle, "pinmodel")))
        m_pPinModel = m_fPin(m_pModel);
    else
        DBG(printf("Model %s does not have a pin model\n",  dllname));
    return m_pModel;
}


bool ModelMgr::unload()
{
    if (m_handle)
    {
        if (m_pModel)
            cleanup(m_pModel);                         // Bug 12835
        if (m_deferUnload)
            return true;             // Bug 12425: Defer unload until next load
        else
            return _unload();
    }
    return false;
}


bool ModelMgr::_unload()
{
    if (m_handle)
    {
        //DBG(printf("Unloading model %s\n", m_pModel->getDeviceName()));
        if (m_fDtor && m_pModel) m_fDtor(m_pModel);
        if (FreeLibrary(m_handle))
        {
            m_handle = NULL;
            return true;
        }
        else
        {
            err(MODEL_UNLOAD_ERROR, errorText()); 
            return false;
        }
    }
    return false;
}


char *ModelMgr::errorText()
{
#ifdef __unix__
    return dlerror();
#else
    // All this bureaucracy just to accomplish the same as dlerror()... :(

    FormatMessage(
        FORMAT_MESSAGE_FROM_SYSTEM,
        NULL,
        GetLastError(),
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        (LPTSTR) &m_errmsg,
        sizeof(m_errmsg), NULL );
    return m_errmsg;

#endif
}


// This can be used to extract other symbols from the dll, intended
// for test programs only, experimental. Beware of C++ mangling.
// Use at own risk! Seems to work with tt_cli in modeltester :)
void* ModelMgr::getsym(const char *sym)
{
    return m_handle? (void *)GetProcAddress(m_handle, sym) : NULL;
}

// Private method.

void ModelMgr::err(int errcode, const char *errstring)
{
    m_error.error_code = errcode;
    m_error.errstring = const_cast<char*>(errstring);
    m_error.device = m_device;
    m_error.dllname = m_dllname;
    m_error.product = m_error.version = m_error.licpath = m_error.licmgr
        = (char*)"";
}

#ifdef _MSC_VER
// warning C4996
#pragma warning (pop)
#endif
