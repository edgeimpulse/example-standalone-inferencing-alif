/* Copyright (c) 2021 ALIF SEMICONDUCTOR

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ALIF SEMICONDUCTOR nor the names of its contributors
     may be used to endorse or promote products derived from this software
     without specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/

/**************************************************************************//**
 * @file     display.h
 * @author   Girish BN
 * @email    girish.bn@alifsemi.com
 * @version  V1.0.0
 * @date     30-Sep-2021
 * @brief    display driver header.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#ifndef __DISPLAY_CFG_H__
#define __DISPLAY_CFG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define E50RA_MW550_N
//#define E43RB_FW405_C
//#define E43GB_MW405_C

#define RGB_BYTES 		3U
#define ARGB_BYTES 		4U
#define PIXEL_BYTES		ARGB_BYTES

// Display dimensions
#define DIMAGE_X		480U
#if defined(E50RA_MW550_N)
#define DIMAGE_Y		854U
#else
#define DIMAGE_Y		800U
#endif
#define DISPLAY_BUFFER_SIZE  (DIMAGE_X*DIMAGE_Y*PIXEL_BYTES)

void hw_disp_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __DISPLAY_CFG_H__ */
