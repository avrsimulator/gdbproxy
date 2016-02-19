/**
 * \file
 *
 * \brief PIN interface for simulator models
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

// This interface is only supported on some models. In order to find
// out if a given model supports this interface, one must use
// ModelMgr::pinmodel() to obtain a pointer to a PinModel object, if
// that returns NULL the model does not (yet) support a pin model.
// (@TODO: when the Model interface is updated, a method to obtain pin
// model will be added there as well)
//
// The recommended method of using the pin model is to obtain Pin objects
// for all pins of interest, and do all pin access through Pin objects.
// 
// The Pin object is defined in pin.h
//
// Versioning of this interface will follow Model interface? [TBD]

#ifndef PINMODEL_H_
#define PINMODEL_H_


class Model;
class Pin;
class PinModel;

// Pin trace callback function type
typedef void PinTraceCallback(Pin*, void*);


class PinModel
{
public:
    /**
     *  I/O pins are identifed by primary names like "PA0" etc.
     *  Other dedicated pins which may be read/written via this
     *  interface are VCC, dedicated RESET, etc.
     *
     *  Get Pin object pointer for named pin.
     *  Returns NULL ptr if named pin does not exist.
     *  Users not relating to pin objects can use this as a boolean
     *  function to determine if a named pin exists.
     */
    virtual Pin  *getPin(const char *name) = 0;


    /**
     *  get number of pins supported by model.
     */
    virtual unsigned getNumPins() = 0;

    /**
     *  get array of all pins supported by model (NULL-terminated pointer array)
     *  Returns NULL if no pins.
     */
    virtual Pin** getPins() = 0;

    /** 
     * @TODO: get map of all pins supported by model, indexed by pin numbers
     * (if required)
     */

    /**
     * convenience methods for users not wanting to relate to pin objects
     *
     * Read pin voltage.
     * returns: NaN if named pin does not exist. Last written voltage if
     * pin is input.
     */
    virtual double readPin(const char *name) = 0;
    /**
     * write pin voltage.
     * returns false if pin does not exist or is output.
     */
    virtual bool   writePin(const char *name, double voltage) = 0;

    /**
     * Determine if pin is currently output.
     */ 
    virtual bool   pinIsOutput(const char *name) = 0;

    /**
     * Convenience method: Get corresponding simulator Model object
     */
    virtual Model *getModel() = 0;

    /**
     * Set up pin-level trace callback.
     */
    virtual PinTraceCallback* setTraceCallback(PinTraceCallback *callback,
                                               void *userdata = 0) = 0;

    /**
     * Pin-level trace setup methods.
     */
    virtual bool tracePin(const char *name) = 0;
    virtual bool untracePin(const char *name) = 0;
    virtual void untraceAllPins() = 0;
};


#endif // PINMODEL_H_
