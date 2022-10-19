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
#ifndef RTE_Compiler_IO_STDOUT_User
#define RTE_Compiler_IO_STDOUT_User
#endif

#include "hal/uart_stdout.h"
#ifdef RTE_Compiler_IO_STDOUT_User
#include "hal/Driver_PINMUX_AND_PINPAD.h"
#include "hal/Driver_USART.h"

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

// <h>STDOUT USART Interface

//   <o>Connect to hardware via Driver_USART# <0-255>
//   <i>Select driver control block for USART interface
#define USART_DRV_NUM           2

//   <o>Enable Port A or Port B of the USART interface
#define USART_PORT_A            1
#define USART_PORT_B            0

//   <o>Baudrate
#define USART_BAUDRATE          115200

// </h>

#define _USART_Driver_(n)  Driver_USART##n
#define  USART_Driver_(n) _USART_Driver_(n)

extern ARM_DRIVER_USART  USART_Driver_(USART_DRV_NUM);
#define ptrUSART       (&USART_Driver_(USART_DRV_NUM))

/**
  Initialize pinmux

  \return          0 on success, or -1 on error.
*/
static int usart_pinmux_init(void)
{
	int32_t ret;
	uint32_t port_config = PAD_FUNCTION_READ_ENABLE				|
						   PAD_FUNCTION_DRIVER_DISABLE_STATE_WITH_PULL_UP;

	/* NOTE: VDD_IO_FLEX can be designed for 1.8V or 3.3V IO level
	 * and pins P3_[23:20] and P4_[3:0] will operate at that voltage */
#if USART_DRV_NUM == 0
#if USART_PORT_A == 1
	/* PINMUX UART0_A */
	/* Configure GPIO Pin : P3_20 as UART0_RX_A */
	ret = PINMUX_Config (PORT_NUMBER_3, PIN_NUMBER_20, PINMUX_ALTERNATE_FUNCTION_1);
	if(ret != ARM_DRIVER_OK) {
		return ret;
	}
	ret = PINPAD_Config(PORT_NUMBER_3, PIN_NUMBER_20, port_config);
	if(ret != ARM_DRIVER_OK) {
		return ret;
	}

	/* Configure GPIO Pin : P3_21 as UART0_TX_A */
	ret = PINMUX_Config (PORT_NUMBER_3, PIN_NUMBER_21, PINMUX_ALTERNATE_FUNCTION_1);
	if(ret != ARM_DRIVER_OK) {
		return ret;
	}
	ret = PINPAD_Config(PORT_NUMBER_3, PIN_NUMBER_21, port_config);
	if(ret != ARM_DRIVER_OK) {
		return ret;
	}
#elif USART_PORT_B == 1
#endif
#elif USART_DRV_NUM == 1
#if USART_PORT_A == 1
	/* PINMUX UART1_A */
	/* Configure GPIO Pin : P1_4 as UART1_RX_A */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_4, PINMUX_ALTERNATE_FUNCTION_1);
	if(ret != ARM_DRIVER_OK) {
		return ret;
	}
	ret = PINPAD_Config(PORT_NUMBER_1, PIN_NUMBER_4, port_config);
	if(ret != ARM_DRIVER_OK) {
		return ret;
	}

	/* Configure GPIO Pin : P1_5 as UART1_TX_A */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_5, PINMUX_ALTERNATE_FUNCTION_1);
	if(ret != ARM_DRIVER_OK) {
		return ret;
	}
	ret = PINPAD_Config(PORT_NUMBER_1, PIN_NUMBER_5, port_config);
	if(ret != ARM_DRIVER_OK) {
		return ret;
	}
#elif USART_PORT_B == 1
#endif
#elif USART_DRV_NUM == 2
#if USART_PORT_A == 1
	/* PINMUX UART2_A */
	/* Configure GPIO Pin : P1_10 as UART2_RX_A */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_10, PINMUX_ALTERNATE_FUNCTION_1);
	if(ret != ARM_DRIVER_OK) {
		return ret;
	}
	ret = PINPAD_Config(PORT_NUMBER_1, PIN_NUMBER_10, port_config);
	if(ret != ARM_DRIVER_OK) {
		return ret;
	}

	/* Configure GPIO Pin : P1_11 as UART2_TX_A */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_11, PINMUX_ALTERNATE_FUNCTION_1);
	if(ret != ARM_DRIVER_OK) {
		return ret;
	}
	ret = PINPAD_Config(PORT_NUMBER_1, PIN_NUMBER_11, port_config);
	if(ret != ARM_DRIVER_OK) {
		return ret;
	}
#elif USART_PORT_B == 1
	/* PINMUX UART2_B */
	/* Configure GPIO Pin : P3_16 as UART2_RX_B */
	ret = PINMUX_Config (PORT_NUMBER_3, PIN_NUMBER_16, PINMUX_ALTERNATE_FUNCTION_2);
	if(ret != ARM_DRIVER_OK) {
		return ret;
	}
	ret = PINPAD_Config(PORT_NUMBER_3, PIN_NUMBER_16, port_config);
	if(ret != ARM_DRIVER_OK) {
		return ret;
	}

	/* Configure GPIO Pin : P3_17 as UART2_TX_B */
	ret = PINMUX_Config (PORT_NUMBER_3, PIN_NUMBER_17, PINMUX_ALTERNATE_FUNCTION_2);
	if(ret != ARM_DRIVER_OK) {
		return ret;
	}
	ret = PINPAD_Config(PORT_NUMBER_3, PIN_NUMBER_17, port_config);
	if(ret != ARM_DRIVER_OK) {
		return ret;
	}
#endif
#elif USART_DRV_NUM == 4
#if USART_PORT_A == 1
	/* PINMUX UART4_A */
	/* Configure GPIO Pin : P1_2 as UART4_RX_A */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_2, PINMUX_ALTERNATE_FUNCTION_1);
	if(ret != ARM_DRIVER_OK) {
		return ret;
	}
	ret = PINPAD_Config(PORT_NUMBER_1, PIN_NUMBER_2, port_config);
	if(ret != ARM_DRIVER_OK) {
		return ret;
	}

	/* Configure GPIO Pin : P1_3 as UART4_TX_A */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_3, PINMUX_ALTERNATE_FUNCTION_1);
	if(ret != ARM_DRIVER_OK) {
		return ret;
	}
	ret = PINPAD_Config(PORT_NUMBER_1, PIN_NUMBER_3, port_config);
	if(ret != ARM_DRIVER_OK) {
		return ret;
	}
#elif USART_PORT_B == 1
	/* PINMUX UART4_B */
	/* Configure GPIO Pin : P3_1 as UART4_RX_B */
	ret = PINMUX_Config (PORT_NUMBER_3, PIN_NUMBER_1, PINMUX_ALTERNATE_FUNCTION_1);
	if(ret != ARM_DRIVER_OK) {
		return ret;
	}
	ret = PINPAD_Config(PORT_NUMBER_3, PIN_NUMBER_1, port_config);
	if(ret != ARM_DRIVER_OK) {
		return ret;
	}

	/* Configure GPIO Pin : P3_2 as UART4_TX_B */
	ret = PINMUX_Config (PORT_NUMBER_3, PIN_NUMBER_2, PINMUX_ALTERNATE_FUNCTION_1);
	if(ret != ARM_DRIVER_OK) {
		return ret;
	}
	ret = PINPAD_Config(PORT_NUMBER_3, PIN_NUMBER_2, port_config);
	if(ret != ARM_DRIVER_OK) {
		return ret;
	}
#endif
#endif

	return ret;
}

static void usart_out_callback(uint32_t event)
{
	if (event & ARM_USART_EVENT_SEND_COMPLETE)
	{
		/* Send Success */
		static uint32_t send_counter = 0;
		send_counter++;
	}

	if (event & ARM_USART_EVENT_RECEIVE_COMPLETE)
	{
		/* Receive Success */
		static uint32_t receive_counter = 0;
		receive_counter++;
	}

	if (event & ARM_USART_EVENT_RX_TIMEOUT)
	{
		/* Receive Success with rx timeout */
		static uint32_t timeout_counter = 0;
		timeout_counter++;
	}
}

/**
  Initialize stdout

  \return          0 on success, or -1 on error.
*/
int stdout_init (void) {
  int32_t status;

  status = usart_pinmux_init();
  if (status != ARM_DRIVER_OK) return (status);

  status = ptrUSART->Initialize(usart_out_callback);
  if (status != ARM_DRIVER_OK) return (status);

  status = ptrUSART->PowerControl(ARM_POWER_FULL);
  if (status != ARM_DRIVER_OK) return (status);

  status = ptrUSART->Control(ARM_USART_MODE_ASYNCHRONOUS |
                             ARM_USART_DATA_BITS_8       |
                             ARM_USART_PARITY_NONE       |
                             ARM_USART_STOP_BITS_1       |
                             ARM_USART_FLOW_CONTROL_NONE,
                             USART_BAUDRATE);
  if (status != ARM_DRIVER_OK) return (status);

  status = ptrUSART->Control(ARM_USART_CONTROL_TX, 1);
  if (status != ARM_DRIVER_OK) return (status);

  return (status);
}

/**
  Put a character to the stdout

  \param[in]   ch  Character to output
  \return          The character written, or -1 on write error.
*/
int stdout_putchar (int ch) {
  uint8_t buf[1];
  buf[0] = ch;

  while (ptrUSART->Send(buf, 1) == ARM_DRIVER_ERROR_BUSY);
  while (ptrUSART->GetTxCount() != 1);

  if (buf[0] == '\n') {
	  buf[0] = '\r';
	  while (ptrUSART->Send(buf, 1) == ARM_DRIVER_ERROR_BUSY);
	  while (ptrUSART->GetTxCount() != 1);
  }

  return (ch);
}
#endif /* RTE_Compiler_IO_STDOUT_User */
