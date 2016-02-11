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
 
   
   Target initializtion module.
  
   Exported Data:
     None
  
   Imported Data:
     Target descriptors     
  
   Static Data:
     None
  
   Global Functions:  
     rp_init  - intialize list of targets
  
   Static Functions:  
     rp_init_check_target - check the target implements all methods      
   
   $Id: init.c,v 1.2 2010/02/10 11:41:51 vapier Exp $ */

/* If we have a target there is no point of excluding it from
   the image. Static initialization will, therefore, work just fine. */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <stdint.h>
#include <stdio.h>
#include <assert.h>

#include "gdbproxy.h"

/* List of targets */
extern rp_target remote_target;
extern rp_target extended_remote_target;

#if defined(HAVE_TARGET_SKELETON_C)
extern rp_target skeleton_target;
#endif

#if defined(HAVE_TARGET_MSP430_C)
extern rp_target msp430_target;
#endif

/* TODO: Add more targets here */

/* Function to check the target module implements all methods */
static int rp_init_check_target(rp_target *t);

/* Build target list and return pointer to its head. */
rp_target *rp_init(void)
{
    rp_target *t_list;
    rp_target *t;
    int ret;

    /* Initialize list */
    t_list = NULL;

    /* Add target to the list */
    remote_target.next          = t_list;
    t_list                      = &remote_target;

    extended_remote_target.next = t_list;
    t_list                      = &extended_remote_target;

#if defined(HAVE_TARGET_SKELETON_C)
    skeleton_target.next	= t_list;
    t_list			= &skeleton_target;
#endif
    
#if defined(HAVE_TARGET_MSP430_C)
    msp430_target.next          = t_list;
    t_list                      = &msp430_target;
#endif

    /* TODO: Add more targets here */

    /* Check list */
    for (t = t_list;  t;  t = t->next)
    {
        if (!(ret = rp_init_check_target(t)))
        {
            if (t->name)
            {
                fprintf(stderr,
		        "initial check failed for target %s\n",
			t->name);
            }
            else
            {
                fprintf(stderr,
		        "initial check failed for target 0x%08x\n",
                        (unsigned int) t);
            }
            assert(0);
            return NULL;
        }
    }
    return t_list;
}

/* Check the target module has all the required methods */
/* This is a fairly basic check of the integrity of the target module */
static int rp_init_check_target(rp_target *t)
{
    if (t->name == NULL)
        return  FALSE;

    if (t->desc == NULL)
        return  FALSE;

    if (t->help == NULL)
        return  FALSE;

    if (t->open == NULL)
        return  FALSE;

    if (t->close == NULL)
        return  FALSE;

    if (t->connect == NULL)
        return  FALSE;

    if (t->disconnect == NULL)
        return  FALSE;

    if (t->kill == NULL)
        return  FALSE;

    if (t->restart == NULL)
        return  FALSE;

    if (t->stop  == NULL)
        return  FALSE;

    if (t->set_gen_thread == NULL)
        return  FALSE;

    if (t->set_ctrl_thread == NULL)
        return  FALSE;

    if (t->is_thread_alive == NULL)
        return  FALSE;

    if (t->read_registers == NULL)
        return  FALSE;

    if (t->read_single_register == NULL)
	return  FALSE;

    if (t->write_registers == NULL)
        return  FALSE;

    if (t->write_single_register == NULL)
        return  FALSE;

    if (t->read_mem == NULL)
        return  FALSE;

    if (t->write_mem == NULL)
        return  FALSE;

    if (t->resume_from_addr == NULL)
        return  FALSE;

    if (t->resume_from_current == NULL)
        return  FALSE;

    if (t->go_waiting == NULL)
        return  FALSE;

    if (t->wait_partial == NULL)
        return  FALSE;

    if (t->wait == NULL)
        return  FALSE;

    if (t->process_query == NULL)
        return  FALSE;

    if (t->list_query == NULL)
        return  FALSE;

    if (t->current_thread_query == NULL)
        return  FALSE;

    if (t->offsets_query == NULL)
        return  FALSE;

    if (t->crc_query == NULL)
        return  FALSE;

    if (t->raw_query == NULL)
        return  FALSE;

    if (t->add_break == NULL)
        return  FALSE;

    if (t->remove_break == NULL)
        return  FALSE;

    return  TRUE;
}
