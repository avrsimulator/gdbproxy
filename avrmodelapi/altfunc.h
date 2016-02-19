/**
 * \file
 *
 * \brief Alternate pin function base class
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
 * There will be one AltFunc object associated with every pin having
 * an alternative function.
 */

#ifndef ALTFUNC_H_
#define ALTFUNC_H_

#include <string>
#include <pin.h>

class AltFunc
{
 public:
    // Get name of currently active alternative function (eg. "ADCA0").
    virtual const std::string &getName() = 0;
    
    // Perform alternative read and write function. Returns false if
    // none active.
    virtual bool read(double *value) = 0;
    virtual bool write(double value) = 0;

    // Check for alterantive mode of pin.
    virtual bool mode(bool *isOutput, Pin::PinMode *mode) = 0;
};


#endif  // ALTFUNC_H_