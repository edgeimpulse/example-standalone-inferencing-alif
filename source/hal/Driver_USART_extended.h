/* Copyright (c) 2019 - 2020 ALIF SEMICONDUCTOR

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
 * @file     Driver_USART_extended.h
 * @author   Tanay Rami
 * @email    tanay@alifsemi.com
 * @version  V1.0.0
 * @date     20-May-2020
 * @brief    UART extended Driver definitions for "USART Control Codes"
 * 			 (as per ARM CMSIS-Driver specification developed by ALIF SEMICONDUCTOR).
 * 			 Control parameters can be use with following API:
 * 			 "ARM_USART_Control(uint32_t control, uint32_t arg)"
 * 			 Reference: "Driver_USART.h"
 * 			 Control parameters are added for both UART RS232 and RS485 modes.
 * @bug      None.
 * @Note	 None
 ******************************************************************************/

#ifndef DRIVER_USART_EXTENDED_H_
#define DRIVER_USART_EXTENDED_H_

#ifdef  __cplusplus
extern "C"
{
#endif

#include "Driver_USART.h"

/* Macros ----------------------------------------------------------------------------- */

/*----- Extended USART Control Codes: Miscellaneous Controls  -----*/
#define ARM_USART_SET_RX_TRIGGER_LVL        			(0x1BUL << ARM_USART_CONTROL_Pos)   /* Set RX trigger value; arg = value */
#define ARM_USART_GET_RX_TRIGGER_LVL        			(0x1CUL << ARM_USART_CONTROL_Pos)   /* Get RX trigger value; arg = value */

#define ARM_USART_GET_TX_FIFO_AVAIL_CNT    				(0x1DUL << ARM_USART_CONTROL_Pos)   /* Get TX fifo available count , arg = dummy */
#define ARM_USART_GET_RX_FIFO_AVAIL_CNT     			(0x1EUL << ARM_USART_CONTROL_Pos)   /* Get RX fifo available count , arg = dummy */

#define ARM_USART_SET_INT_PRIORITY    					(0x1FUL << ARM_USART_CONTROL_Pos)   /* Set interrupt priority , arg = interrupt priority */
#define ARM_USART_GET_INT_PRIORITY     					(0x20UL << ARM_USART_CONTROL_Pos)   /* Get interrupt priority , arg = dummy */

 /* RS485 control codes */
#define ARM_USART_CONTROL_RS485             			(0x21UL << ARM_USART_CONTROL_Pos)   /* Set Control RS485; arg: 0=disable, 1=enable */
#define ARM_USART_GET_RS485_STATE             			(0x22UL << ARM_USART_CONTROL_Pos)   /* Get RS485 status; arg: dummy, return 0=disabled, 1=enabled */

#define ARM_USART_SET_RS485_TRANSFER_MODE				(0x23UL << ARM_USART_CONTROL_Pos)   /* Set RS485 transfer modes \ref ARM_USART_RS485_TRANSFER_MODE */
#define ARM_USART_GET_RS485_TRANSFER_MODE				(0x24UL << ARM_USART_CONTROL_Pos)   /* Get RS485 transfer modes \ref ARM_USART_RS485_TRANSFER_MODE */

#define ARM_USART_SET_RS485_DE_ASSERTION_TIME			(0x25UL << ARM_USART_CONTROL_Pos)   /* Set RS485 Driver Enable DE Assertion Time 8-bit     */
#define ARM_USART_GET_RS485_DE_ASSERTION_TIME			(0x26UL << ARM_USART_CONTROL_Pos)   /* Get RS485 Driver Enable DE Assertion Time 8-bit     */

#define ARM_USART_SET_RS485_DE_DEASSERTION_TIME			(0x27UL << ARM_USART_CONTROL_Pos)   /* Set RS485 Driver Enable DE De-Assertion Time 8-bit  */
#define ARM_USART_GET_RS485_DE_DEASSERTION_TIME			(0x28UL << ARM_USART_CONTROL_Pos)   /* Get RS485 Driver Enable DE De-Assertion Time 8-bit  */

#define ARM_USART_SET_RS485_DE_TO_RE_TURN_AROUND_TIME	(0x29UL << ARM_USART_CONTROL_Pos)   /* Set RS485 Driver Enable DE to Receiver Enable RE turn around time 16-bit  */
#define ARM_USART_GET_RS485_DE_TO_RE_TURN_AROUND_TIME	(0x2AUL << ARM_USART_CONTROL_Pos)   /* Get RS485 Driver Enable DE to Receiver Enable RE turn around time 16-bit  */

#define ARM_USART_SET_RS485_RE_TO_DE_TURN_AROUND_TIME	(0x2BUL << ARM_USART_CONTROL_Pos)   /* Set RS485 Receiver Enable RE to Driver Enable DE turn around time 16-bit  */
#define ARM_USART_GET_RS485_RE_TO_DE_TURN_AROUND_TIME	(0x2CUL << ARM_USART_CONTROL_Pos)   /* Get RS485 Receiver Enable RE to Driver Enable DE turn around time 16-bit  */

#define ARM_USART_SET_RS485_DE_EN						(0x2DUL << ARM_USART_CONTROL_Pos)   /* Set RS485 Driver Enable signal DE, 0=disable,  1=enable					 */
#define ARM_USART_GET_RS485_DE_EN						(0x2EUL << ARM_USART_CONTROL_Pos)   /* Get RS485 Driver Enable signal DE, 0=disabled, 1=enabled, arg = dummy 	 */

#define ARM_USART_SET_RS485_RE_EN						(0x2FUL << ARM_USART_CONTROL_Pos)   /* Set RS485 Receiver Enable signal RE, 0=disable,  1=enable 				 */
#define ARM_USART_GET_RS485_RE_EN						(0x30UL << ARM_USART_CONTROL_Pos)   /* Get RS485 Receiver Enable signal RE, 0=disabled, 1=enabled, arg = dummy   */

/* End of RS485 control codes */

#define ARM_USART_SET_TX_TRIGGER_LVL        			(0x31UL << ARM_USART_CONTROL_Pos)   /* Set TX trigger value; arg = value */
#define ARM_USART_GET_TX_TRIGGER_LVL        			(0x32UL << ARM_USART_CONTROL_Pos)   /* Get TX trigger value; arg = value */


#ifdef  __cplusplus
}
#endif

#endif /* DRIVER_USART_EXTENDED_H_ */

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
