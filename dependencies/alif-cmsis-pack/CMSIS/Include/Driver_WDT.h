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
 * @file     Driver_WDT.h
 * @author   Tanay Rami
 * @email    tanay@alifsemi.com
 * @version  V1.0.0
 * @date     27-April-2021
 * @brief    Watchdog Timer(WDT) driver definitions.
 ******************************************************************************/

#ifndef DRIVER_WDT_H_
#define DRIVER_WDT_H_

#ifdef  __cplusplus
extern "C"
{
#endif

/* Includes --------------------------------------------------------------------------- */
#include "Driver_Common.h"

#define ARM_WDT_API_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,0)  /* API version */

/*----- Watchdog Control Codes -----*/
#define ARM_WATCHDOG_LOCK                      (0x01UL)    ///< watchdog lock;    arg = dummy
#define ARM_WATCHDOG_UNLOCK                    (0x02UL)    ///< watchdog unlock;  arg = dummy


// Function documentation
/**
  \fn          ARM_DRIVER_VERSION GetVersion (void)
  \brief       Get Watchdog driver version.
  \return      \ref ARM_DRIVER_VERSION

  \fn          ARM_WDT_CAPABILITIES GetCapabilities (void)
  \brief       Get Watchdog driver capabilities
  \return      \ref WDT_CAPABILITIES

  \fn          int32_t Initialize (uint32_t timeout)
  \brief       Initialize Watchdog Interface.
  \param[in]   timeout  : Timeout in msec, must be > 0.
  \return      \ref execution_status

  \fn          int32_t Uninitialize (void)
  \brief       De-initialize Watchdog Interface.
  \return      \ref execution_status

  \fn          int32_t PowerControl (ARM_POWER_STATE state)
  \brief       Control Watchdog Interface Power.
  \param[in]   state : Power state
  \return      \ref execution_status

  \fn          int32_t Start (void)
  \brief       Start Watchdog Interface.
  \return      \ref execution_status

  \fn          int32_t Feed (void)
  \brief       Feed Watchdog Interface.
  \return      \ref execution_status

  \fn          int32_t Stop (void)
  \brief       Stop Watchdog Interface.
  \return      \ref execution_status

  \fn          int32_t Control (uint32_t control, uint32_t arg)
  \brief       Control Watchdog Interface.
  \param[in]   control :  Operation
  \param[in]   arg     :  Argument of operation (optional)
  \return      common \ref execution_status

  \fn          int32_t GetRemainingTime (uint32_t *val)
  \brief       Get watchdog remaining time before reset.
  \param[out]  val     : Pointer to the address where watchdog remaining time before reset
                         needs to be copied to.
  \return      \ref execution_status
*/

/**
\brief Watchdog Device Driver Capabilities.
*/
typedef struct _ARM_WDT_CAPABILITIES {
  uint32_t lock               : 1;      ///< supports Watchdog Lock-Unlock
  uint32_t reserved           : 31;     ///< Reserved (must be zero)
} ARM_WDT_CAPABILITIES;

/**
\brief Access structure of the Watchdog Driver.
*/
typedef struct _ARM_DRIVER_WDT {
  ARM_DRIVER_VERSION     (*GetVersion)        (void);                            ///< Pointer to \ref WDT_GetVersion : Get driver version.
  ARM_WDT_CAPABILITIES   (*GetCapabilities)   (void);                            ///< Pointer to \ref WDT_GetCapabilities : Get driver capabilities.
  int32_t                (*Initialize)        (uint32_t timeout);                ///< Pointer to \ref WDT_Initialize : Initialize Watchdog Interface.
  int32_t                (*Uninitialize)      (void);                            ///< Pointer to \ref WDT_Uninitialize : De-initialize Watchdog Interface.
  int32_t                (*PowerControl)      (ARM_POWER_STATE state);           ///< Pointer to \ref WDT_PowerControl : Control Watchdog Interface Power.
  int32_t                (*Start)             (void);                            ///< Pointer to \ref WDT_Start : Start Watchdog Interface.
  int32_t                (*Feed)              (void);                            ///< Pointer to \ref WDT_Feed  : Feed  Watchdog Interface.
  int32_t                (*Stop)              (void);                            ///< Pointer to \ref WDT_Stop  : Stop  Watchdog Interface.
  int32_t                (*Control)           (uint32_t control, uint32_t arg);  ///< Pointer to \ref WDT_Control : Control Watchdog Interface.
  int32_t                (*GetRemainingTime)  (uint32_t *val);                   ///< Pointer to \ref WDT_GetRemainingTime : Get Watchdog remaining time before reset.
} const ARM_DRIVER_WDT;

#ifdef  __cplusplus
}
#endif

#endif /* DRIVER_WDT_H_ */

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
