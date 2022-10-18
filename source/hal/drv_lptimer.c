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

#include "RTE_Components.h"
#include CMSIS_device_header
#include "drv_lptimer.h"
#include "hal/global_map.h"

void LPTIMER_ll_Initialize (uint8_t channel) {

    LPTIMER_reg_info *reg_ptr = (LPTIMER_reg_info*) LPTIMER_BASE;

    reg_ptr->ch_cntrl_reg[channel].control_reg &= ~LPTIMER_CONTROL_REG_TIMER_ENABLE_BIT;

	reg_ptr->ch_cntrl_reg[channel].control_reg |=  LPTIMER_CONTROL_REG_TIMER_MODE_BIT;

    reg_ptr->ch_cntrl_reg[channel].control_reg &= ~LPTIMER_CONTROL_REG_TIMER_INTERRUPT_MASK_BIT;

    *((uint32_t *)(VBAT_REGS_BASE + 0x28)) |= (0x3UL << (channel * 2));
}

void LPTIMER_ll_Set_Count_Value (uint8_t channel, uint32_t count) {

    LPTIMER_reg_info *reg_ptr = (LPTIMER_reg_info*) LPTIMER_BASE;

	reg_ptr->ch_cntrl_reg[channel].load_count = count;
}

uint32_t LPTIMER_ll_Get_Count_Value (uint8_t channel) {

    LPTIMER_reg_info *reg_ptr = (LPTIMER_reg_info*) LPTIMER_BASE;

    return reg_ptr->ch_cntrl_reg[channel].current_value;
}

void LPTIMER_ll_Start (uint8_t channel) {

    LPTIMER_reg_info *reg_ptr = (LPTIMER_reg_info*) LPTIMER_BASE;

    reg_ptr->ch_cntrl_reg[channel].control_reg |= LPTIMER_CONTROL_REG_TIMER_ENABLE_BIT;
}

void LPTIMER_ll_Stop (uint8_t channel) {

    LPTIMER_reg_info *reg_ptr = (LPTIMER_reg_info*) LPTIMER_BASE;

    reg_ptr->ch_cntrl_reg[channel].control_reg &= ~LPTIMER_CONTROL_REG_TIMER_ENABLE_BIT;
}

void LPTIMER_ll_Wait (uint8_t channel) {

    LPTIMER_reg_info *reg_ptr = (LPTIMER_reg_info*) LPTIMER_BASE;

    while (1) {
    	if (reg_ptr->ch_cntrl_reg[channel].int_status)
    		break;
    }

    (void) reg_ptr->ch_cntrl_reg[channel].eoi;
}
