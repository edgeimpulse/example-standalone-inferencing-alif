/* Copyright (c) 2020 - 2021 ALIF SEMICONDUCTOR

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
 * @file     GPIO_ll_drv.h
 * @author   Girish BN
 * @email    girish.bn@alifsemi.com
 * @version  V1.0.0
 * @date     21-Aug-2020
 * @brief    CMSIS-Driver for GPIO.
 * @bug      None.
 * @Note     None
 ******************************************************************************/
#ifndef __GPIO_LL_DRV_H__
#define __GPIO_LL_DRV_H__

#include "RTE_Device.h"
#include "Driver_GPIO.h"
#include "RTE_Components.h"
#include CMSIS_device_header


#define GPIO_PORT_MAX_PIN_NUMBER                (32)

#define ARM_GPIO_BIT_IRQ_POLARITY_Pos           0       ///< bits - 0
#define ARM_GPIO_BIT_IRQ_POLARITY_Msk           (1 << ARM_GPIO_BIT_IRQ_POLARITY_Pos)
#define ARM_GPIO_BIT_IRQ_POLARITY(x)            (((x)& ARM_GPIO_BIT_IRQ_POLARITY_Msk) >> ARM_GPIO_BIT_IRQ_POLARITY_Pos)

#define ARM_GPIO_BIT_IRQ_BOTH_EDGE_Pos          1       ///< bits - 1
#define ARM_GPIO_BIT_IRQ_BOTH_EDGE_Msk          (1 << ARM_GPIO_BIT_IRQ_BOTH_EDGE_Pos)
#define ARM_GPIO_BIT_IRQ_BOTH_EDGE(x)           (((x)& ARM_GPIO_BIT_IRQ_BOTH_EDGE_Msk) >> ARM_GPIO_BIT_IRQ_BOTH_EDGE_Pos)

#define ARM_GPIO_BIT_IRQ_SENSITIVE_Pos          2       ///< bits - 2
#define ARM_GPIO_BIT_IRQ_SENSITIVE_Msk          (1 << ARM_GPIO_BIT_IRQ_SENSITIVE_Pos)
#define ARM_GPIO_BIT_IRQ_SENSITIVE(x)           (((x)& ARM_GPIO_BIT_IRQ_SENSITIVE_Msk) >> ARM_GPIO_BIT_IRQ_SENSITIVE_Pos)

typedef struct _GPIO_RegInfo{
    __IOM uint32_t  gpio_swport_dr;             /**< Port A data register>*/
    __IOM uint32_t  gpio_swport_ddr;            /**< Port A Data Direction Register>*/
    __IM  uint32_t  Reserved0[10];              /**< Reserved register...>*/
    __IOM uint32_t  gpio_inten;                 /**< Interrupt enable register Note: This register is available only if Port A is configured to generate...>*/
    __IOM uint32_t  gpio_intmask;               /**< Interrupt mask register Note: This register is available only if Port A is configured to generate...>*/
    __IOM uint32_t  gpio_inttype_level;         /**< Interrupt level Note: This register is available only if Port A is configured to generate interrupts...>*/
    __IOM uint32_t  gpio_int_polarity;          /**< Interrupt polarity Note: This register is available only if Port A is configured to generate interrupts...>*/
    __IM  uint32_t  gpio_intstatus;             /**< Interrupt status Note: This register is available only if Port A is configured to generate interrupts...>*/
    __IM  uint32_t  gpio_raw_intstatus;         /**< Raw interrupt status Note: This register is available only if Port A is configured to generate...>*/
    __IOM uint32_t  gpio_debounce;              /**< Denounce enable Note: This register is available only if Port A is configured to generate interrupts...>*/
    __IOM uint32_t  gpio_porta_eoi;             /**< Port clear interrupt register Note: This register is available only if Port A is configured to...>*/
    __IM  uint32_t  gpio_ext_port;              /**< External port register>*/
    __IM  uint32_t  Reserved1[3];               /**< Reserved register...>*/
    __IOM uint32_t  gpio_ls_sync;               /**< Synchronization level>*/
    __IM  uint32_t  gpio_id_code;               /**< GPIO ID code>*/
    __IOM uint32_t  gpio_int_bothedge;          /**< Interrupt Both Edge type Note: This register is available only if PORT A is configured to generate...>*/
    __IM  uint32_t  gpio_ver_id_code;           /**< GPIO Component Version>*/
    __IM  uint32_t  gpio_config_reg2;           /**< GPIO Configuration Register 2 This register is a read-only register that is present when the configuration...>*/
    __IM  uint32_t  gpio_config_reg1;           /**< GPIO Configuration Register 1 This register is present when the configuration parameter GPIO_ADD_ENCODED_PARAMS...>*/
    __IM  uint32_t  Reserved2[994];             /**< Reserved register...>*/
} GPIO_RegInfo;

typedef struct {
    uint32_t *reg_base;                                               /**< GPIO PORT Base Address>*/
    IRQn_Type IRQ_base_num;                                            /**< GPIO PORT IRQ base Num>*/
    uint32_t IRQ_priority[GPIO_PORT_MAX_PIN_NUMBER];                 /**< GPIO PIN IRQ priority >*/
    ARM_GPIO_SignalEvent_t cb_event[GPIO_PORT_MAX_PIN_NUMBER];        /**< GPIO Call back function >*/
} GPIO_resources_t;

int32_t GPIO_ll_PowerControl        (GPIO_resources_t *GPIO, uint8_t pin_no, ARM_POWER_STATE state);
int32_t GPIO_ll_SetDirection        (GPIO_resources_t *GPIO, uint8_t pin_no, uint32_t dir);
int32_t GPIO_ll_GetDirection        (GPIO_resources_t *GPIO, uint8_t pin_no, uint32_t *dir);
int32_t GPIO_ll_SetValue            (GPIO_resources_t *GPIO, uint8_t pin_no, uint32_t value);
int32_t GPIO_ll_GetValue            (GPIO_resources_t *GPIO, uint8_t pin_no, uint32_t *value);
int32_t GPIO_ll_Debounce_Config     (GPIO_resources_t *GPIO, uint8_t pin_no, bool option);
int32_t GPIO_ll_Enable_Interrupt    (GPIO_resources_t *GPIO, uint8_t pin_no, uint32_t arg);
int32_t GPIO_ll_Disable_Interrupt   (GPIO_resources_t *GPIO, uint8_t pin_no);
int32_t GPIO_ll_Read_Config_Value   (GPIO_resources_t *GPIO, bool option, uint32_t *value);
int32_t GPIO_ll_Uninitialize        (GPIO_resources_t *GPIO, uint8_t pin_no);
void    GPIO_ll_IRQ_Handler         (GPIO_resources_t *GPIO, uint8_t pin_no);

#endif /* __GPIO_LL_DRV_H__ */
