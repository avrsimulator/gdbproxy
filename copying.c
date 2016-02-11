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
 
   This module prints license and warranty information.
   
   Exported Data:
     None
  
   Imported Data:
     None     
  
   Global Functions:
     rpShowCopying     - print license information
     rpShowWarranty    - print warranty information
  
   Static Data:
     None
  
   Static Functions:
     None
   
   $Id: copying.c,v 1.1.1.1 2005/09/04 08:57:16 coppice Exp $ */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <stdint.h>
#include <stdio.h>

#include "gdbproxy.h"

/* Print copying part of license */
void rp_show_copying(void)
{
    printf("Original rproxy: Copyright (C) 1999-2001 Quality Quorum, Inc.\n");
    printf("GDBproxy adaptation: Copyright (C) 2002 Chris Liechti and Steve Underwood.\n");
    printf("\n");
    printf("Redistribution and use in source and binary forms, with or without\n");
    printf("modification, are permitted provided that the following conditions are met:\n");
    printf("\n");
    printf("  1. Redistributions of source code must retain the above copyright notice,\n");
    printf("     this list of conditions and the following disclaimer.\n");
    printf("  2. Redistributions in binary form must reproduce the above copyright\n");
    printf("     notice, this list of conditions and the following disclaimer in the\n");
    printf("     documentation and/or other materials provided with the distribution.\n");
    printf("  3. The name of the author may not be used to endorse or promote products\n");
    printf("     derived from this software without specific prior written permission.\n");
    printf("\n");
    printf("\n");
}

/* Print NO WARRANTY part of license */
void rp_show_warranty(void)
{
    printf("THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED\n");
    printf("WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF\n");
    printf("MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO\n");
    printf("EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,\n");
    printf("SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,\n");
    printf("PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;\n");
    printf("OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,\n");
    printf("WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR\n");
    printf("OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF\n");
    printf("ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.\n");
    printf("\n");
    printf("\n");
}

