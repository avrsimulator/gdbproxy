/**
 * \file
 *
 * \brief Class representing a single pin
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


#include <string>

#ifndef PIN_H_
#define PIN_H_

class PinModel;


class Pin
{
public:
    enum PinType {GPIO, RESET, VCC, AVCC};
    enum PinMode {Digital, Analog};

    virtual ~Pin() {};

    /**
     * read pin voltage
     * For input pins the last voltage written to pin will be read back.
     */
    virtual double     read() = 0;
    /**
     * write pin voltage
     * Returns false if pin is configured as output.
     */
    virtual bool       write(double voltage) = 0;

    /**
     * get pin static information
     */
    virtual const char *getName() = 0;
    virtual unsigned    getNumber() = 0;     // returns 0 if pin number not mappped
    virtual PinType    getType() = 0; 
    virtual bool       isActiveLow() = 0;   // for dedicated pins like RESET.

    virtual unsigned   getIoAddress() = 0;
    virtual unsigned   getIoMask() = 0;

    /**
     * get pin dynamic information
     */
    virtual bool       isOutput() = 0;
    virtual PinMode    getMode() = 0;

    /**
     * get model this pin is associated with.
     */
    virtual PinModel  *getPinModel() = 0;

};

#endif //PIN_H_
