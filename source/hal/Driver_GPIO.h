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
 * @file     Driver_GPIO.h
 * @author   Girish BN
 * @email    girish.bn@alifsemi.com
 * @version  V1.0.0
 * @date     21-Aug-2020
 * @brief    CMSIS-Driver for GPIO.
 * @bug      None.
 * @Note     None
 ******************************************************************************/
#ifndef __DRIVER_GPIO_H__
#define __DRIVER_GPIO_H__

#include "Driver_Common.h"

#define _ARM_Driver_GPIO_(n)      Driver_GPIO##n
#define  ARM_Driver_GPIO_(n) _ARM_Driver_GPIO_(n)

#define GPIO_PIN_SOFTWARE_MODE              (1)
#define GPIO_PIN_HARDWARE_MODE              (2)

/****** GPIO Control code : ARM_GPIO_ENABLE_INTERRUPT arg definition *****/
#define ARM_GPIO_IRQ_POLARITY_LOW           0x00000000
#define ARM_GPIO_IRQ_POLARITY_HIGH          0x00000001

#define ARM_GPIO_IRQ_EDGE_SENSITIVE_SINGLE  0x00000000
#define ARM_GPIO_IRQ_EDGE_SENSITIVE_BOTH    0x00000002

#define ARM_GPIO_IRQ_SENSITIVE_LEVEL        0x00000000
#define ARM_GPIO_IRQ_SENSITIVE_EDGE         0x00000004

/****** GPIO Interrupt events *****/
#define ARM_GPIO_IRQ_EVENT_EXTERNAL         (1)

/**< Initialization GPIO call back function declaration >*/
typedef void (*ARM_GPIO_SignalEvent_t) (uint32_t event);

/**< GPIO Control codes >*/
typedef enum _GPIO_OPERATION {
    ARM_GPIO_CONFIG_DEBOUNCE,               /**<GPIO DEBOUNCE configuration operation>*/
    ARM_GPIO_ENABLE_INTERRUPT,              /**<GPIO ENABLE interrupt configuration>*/
    ARM_GPIO_DISABLE_INTERRUPT,             /**<GPIO DISABLE interrupt configuration>*/
    ARM_GPIO_GET_CONFIG_VALUE1,             /**<GPIO GET Config reg-1 value operation>*/
    ARM_GPIO_GET_CONFIG_VALUE2              /**<GPIO GET Config reg-2 value operation>*/
} GPIO_OPERATION;

typedef enum _GPIO_PIN_DIRECTION {
    GPIO_PIN_DIRECTION_INPUT,               /**<GPIO PIN direction to input>*/
    GPIO_PIN_DIRECTION_OUTPUT,              /**<GPIO PIN direction to output>*/
} GPIO_PIN_DIRECTION;

typedef enum _GPIO_PIN_OUTPUT_STATE {
    GPIO_PIN_OUTPUT_STATE_LOW,              /**<GPIO PIN state to LOW>*/
    GPIO_PIN_OUTPUT_STATE_HIGH,             /**<GPIO PIN state to HIGH>*/
    GPIO_PIN_OUTPUT_STATE_TOGGLE,           /**<GPIO PIN state Toggle>*/
} GPIO_PIN_OUTPUT_STATE;

typedef enum _GPIO_PIN_STATE {
    GPIO_PIN_STATE_LOW,                     /**<GPIO PIN state to LOW>*/
    GPIO_PIN_STATE_HIGH,                    /**<GPIO PIN state to HIGH>*/
} GPIO_PIN_STATE;

/**
 \fn            int32_t ARM_GPIO_Initialize (uint8_t pin_no, ARM_GPIO_SignalEvent_t cb_event);
 \brief         Initialize GPIO interface and register signal (callback) functions.
 \param[in]     pin_no: GPIO Pin number
 \param[in]     cb_event: Pointer to \ref ARM_GPIO_SignalEvent_t callback function
 \param[out]    int32_t: execution_status

 \fn            int32_t ARM_GPIO_PowerControl (uint8_t pin_no, ARM_POWER_STATE state);
 \brief         Control GPIO interface power.
 \param[in]     pin_no: GPIO Pin number
 \param[in]     state: Power state
                - ARM_POWER_OFF : power off: no operation possible
                - ARM_POWER_LOW : low power mode: retain state, detect and signal wake-up events
                - ARM_POWER_FULL: power on: full operation at maximum performance
 \param[out]    int32_t: execution_status

 \fn            int32_t ARM_GPIO_SetDirection (uint32_t pin_no, uint32_t dir);
 \brief         Function Configure the GPIO to Input or Output operation .
 \param[in]     pin_no: GPIO Pin number.
 \param[in]     dir: GPIO direction.
 \param[out]    int32_t: execution status.

 \fn            int32_t ARM_GPIO_GetDirection (uint32_t pin_no, uint32_t *dir);
 \brief         Function to get the GPIO direction .
 \param[in]     pin_no: GPIO Pin number.
 \param[in]     *dir: pointer to get the status of GPIO direction.
 \param[out]    int32_t: execution status.

 \fn            int32_t ARM_GPIO_SetValue (uint32_t pin_no, uint32_t value);
 \brief         Function Configure the GPIO pin Output state.
 \param[in]     pin_no: GPIO Pin number.
 \param[in]     value:  Set the output pin status.
 \param[out]    int32_t: execution status.

 \fn            int32_t ARM_GPIO_GetValue (uint32_t pin_no, uint32_t *value);
 \brief         Function read the input GPIO status .
 \param[in]     pin_no: GPIO Pin number.
 \param[in]     *value: pointer to get the input pin status.
 \param[out]    int32_t: execution status.

 \fn            int32_t ARM_GPIO_Control (uint8_t pin_no, uint32_t control_code, uint32_t *arg);
 \brief         Control GPIO interface.
 \param[in]     pin_no: GPIO Pin number
 \param[in]     control_code: control operation
                - ARM_GPIO_CONFIG_DEBOUNCE: Perform the GPIO De-bounce configuration.
                - ARM_GPIO_ENABLE_INTERRUPT: GPIO ENABLE interrupt configuration.
                - ARM_GPIO_DISABLE_INTERRUPT: GPIO DISABLE interrupt configuration.
                - ARM_GPIO_GET_CONFIG_VALUE1: get Config reg-1 value.
                - ARM_GPIO_GET_CONFIG_VALUE2: get Config reg-2 value.
 \param[in]     *arg: pointer to operation parameters
 \param[out]    int32_t : execution_status

 \fn            int32_t ARM_GPIO_Uninitialize (uint32_t pin_no);
 \brief         UnInitialize GPIO interface and register signal (callback) functions.
 \param[in]     pin_no: GPIO Pin number.
 \param[out]    int32_t: execution status.
*/

/**
 * \brief   Access structure of Gpio module.
*/
typedef struct _ARM_DRIVER_GPIO {
    int32_t (*Initialize)   (uint8_t pin_no, ARM_GPIO_SignalEvent_t cb_event);              /**< Pointer to \ref ARM_GPIO_Initialize    : Initialize GPIO interface >*/
    int32_t (*PowerControl) (uint8_t pin_no, ARM_POWER_STATE state);                        /**< Pointer to \ref ARM_GPIO_PowerControl  : Control GPIO interface power. >*/
    int32_t (*SetDirection) (uint8_t pin_no, uint32_t dir);                                 /**< Pointer to \ref ARM_GPIO_SetDirection  : Set GPIO direction. >*/
    int32_t (*GetDirection) (uint8_t pin_no, uint32_t *dir);                                /**< Pointer to \ref ARM_GPIO_GetDirection  : Get GPIO direction. >*/
    int32_t (*SetValue)     (uint8_t pin_no, uint32_t value);                               /**< Pointer to \ref ARM_GPIO_SetValue      : Set GPIO Output pin status>*/
    int32_t (*GetValue)     (uint8_t pin_no, uint32_t *value);                              /**< Pointer to \ref ARM_GPIO_GetValue      : Get GPIO input pin status>*/
    int32_t (*Control)      (uint8_t pin_no, GPIO_OPERATION control_code, uint32_t *arg);   /**< Pointer to \ref ARM_GPIO_Control       : Control GPIO interface.>*/
    int32_t (*Uninitialize) (uint8_t pin_no);                                               /**< Pointer to \ref ARM_GPIO_Uninitialize  : Un-initialize the GPIO Pin configuration >*/
} ARM_DRIVER_GPIO;

#endif /* __DRIVER_GPIO_H__ */
