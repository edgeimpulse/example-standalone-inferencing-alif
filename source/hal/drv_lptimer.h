/* Copyright (c) 2022 ALIF SEMICONDUCTOR

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

#ifndef INC_DRV_LPTIMER_H_
#define INC_DRV_LPTIMER_H_

#include <stdint.h>

typedef void (*ARM_LPTIMER_SignalEvent_t) ();

#define LPTIMER_TICK_RATE									(32768)
#define LPTIMER_MAX_CHANNEL_NUMBER                          (8)

#define LPTIMER_CONTROL_REG_TIMER_ENABLE_BIT                0x00000001
#define LPTIMER_CONTROL_REG_TIMER_MODE_BIT                  0x00000002
#define LPTIMER_CONTROL_REG_TIMER_INTERRUPT_MASK_BIT        0x00000004

typedef struct {
    volatile uint32_t load_count;                           /**< Channel load count register >*/
    volatile uint32_t current_value;                        /**< Channel current running count register >*/
    volatile uint32_t control_reg;                          /**< Channel operation control register >*/
    volatile uint32_t eoi;                                  /**< Channel end of interrupt register>*/
    volatile uint32_t int_status;                           /**< Channel Interrupt status register>*/
} CHANNEL_CONTROL_REG;

typedef struct {
    CHANNEL_CONTROL_REG ch_cntrl_reg[LPTIMER_MAX_CHANNEL_NUMBER];   /**< 8 Channels register instance>*/
    volatile uint32_t int_status;                                   /**< Interrupt status register >*/
    volatile uint32_t eoi;                                          /**< Interrupt end of interrupt register >*/
    volatile uint32_t raw_int_status;                               /**< raw Interrupt status register >*/
    volatile uint32_t comp_ver;                                     /**< Timer component version info >*/
    volatile uint32_t load_count2[LPTIMER_MAX_CHANNEL_NUMBER];      /**< 8 channel instance of load count 2 register >*/
} LPTIMER_reg_info;

void     LPTIMER_ll_Set_Count_Value(uint8_t channel, uint32_t count);
uint32_t LPTIMER_ll_Get_Count_Value(uint8_t channel);
void     LPTIMER_ll_Initialize     (uint8_t channel);
void     LPTIMER_ll_Start          (uint8_t channel);
void     LPTIMER_ll_Wait           (uint8_t channel);
void     LPTIMER_ll_Stop           (uint8_t channel);

#endif /* INC_DRV_LPTIMER_H_ */
