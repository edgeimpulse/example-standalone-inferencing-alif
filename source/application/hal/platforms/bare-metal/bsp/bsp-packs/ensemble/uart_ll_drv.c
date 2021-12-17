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
 * @file     uart_ll_drv.c
 * @author   Tanay Rami
 * @email    tanay@alifsemi.com
 * @version  V1.0.0
 * @date     20-May-2020
 * @brief    CMSIS-Driver for UART.
 *           This file contains lower level (HAL) Apis for UART RS232 and RS485 Driver.
 * @bug      None.
 * @Note	 None.
 ******************************************************************************/

/* Includes --------------------------------------------------------------------------- */
/* System Includes */
#include <stddef.h>
#include <stdint.h>
#include "stdio.h"

/* Project Includes */
#include "uart_ll_drv.h"

/* Macros ----------------------------------------------------------------------------- */

/* convert baudrate to divisor */
#define UART_CONVERT_BAUD_TO_DIVISOR(peripheral_freq, baud)		((peripheral_freq) / ((baud)*16))

/* enable transmit/receive interrupt */
#define UART_ENABLE_TRANSMITTER_INT					(1U)	/* enable transmitter interrupt  */
#define UART_ENABLE_RECEIVER_INT					(2U)	/* enable receiver interrupt 	 */

/* disable transmit/receive interrupt */
#define UART_DISABLE_TRANSMITTER_INT				(3U)	/* disable transmitter interrupt */
#define UART_DISABLE_RECEIVER_INT					(4U)	/* disable receiver interrupt 	 */

/* Functions --------------------------------------------------------------------------- */

/**
 * @fn		int32_t uart_tx_ready (uart_reg_set_t *uart_reg_ptr)
 * @brief	check whether uart is ready to send; 1 ready, 0 not ready
 * @note	none
 * @param	uart_reg_ptr: Pointer to uart register set structure
 * @retval 	1 ready to send, 0 not ready to send
 */
static __inline int32_t uart_tx_ready (uart_reg_set_t *uart_reg_ptr)
{
	/* FIFO_STAT, FIFO MODE and ADDITIONAL_FEATURES are always enabled in ip.
	 * (FIFO_STAT == 1) && (FIFO_MODE != 0) && (ADDITIONAL_FEATURES == 1)
	 */

	/* read TFNF transmitt_fifo_not_full bit from usr uart status register */
	return ((uart_reg_ptr->usr & UART_USR_TRANSMIT_FIFO_NOT_FULL) ? 1 : 0);
}

/**
 * @fn		int32_t uart_rx_ready (uart_reg_set_t *uart_reg_ptr)
 * @brief	check whether uart is ready to receive; 1 ready, 0 not ready
 * @note	none
 * @param	uart_reg_ptr: Pointer to uart register set structure
 * @retval 	1 ready to receive, 0 not ready to receive
 */
static __inline int32_t uart_rx_ready (uart_reg_set_t *uart_reg_ptr)
{
	/* FIFO_STAT, FIFO MODE and ADDITIONAL_FEATURES are always enabled in ip.
	 * (FIFO_STAT == 1) && (FIFO_MODE != 0) && (ADDITIONAL_FEATURES == 1)
	 */

	/* read RFNE receive_fifo_not_empty bit from usr uart status register */
	return ((uart_reg_ptr->usr & UART_USR_RECEIVE_FIFO_NOT_EMPTY) ? 1 : 0);
}

/**
 * @fn 		void uart_send_a_char_to_thr (uart_reg_set_t *uart_reg_ptr,
									      char			  chr)
 * @brief	write a char to uart transmit holding register
 * @note	none
 * @param	uart_reg_ptr : Pointer to uart register set structure
 * @param	chr			 : char to send to thr register
 * @retval 	none
 */
static __inline void uart_send_a_char_to_thr (uart_reg_set_t *uart_reg_ptr,
											  char			  chr)
{
	/* write a char to thr transmit holding register */
	uart_reg_ptr->rbr_thr_dll = chr;
}

/**
 * @fn		int32_t uart_receive_a_char_from_rbr (uart_reg_set_t *uart_reg_ptr)
 * @brief	read data from uart receive buffer register
 * @note	none
 * @param	uart_reg_ptr: Pointer to uart register set structure
 * @retval 	received data
 */
static __inline int32_t uart_receive_a_char_from_rbr (uart_reg_set_t *uart_reg_ptr)
{
	/* read a char from receive buffer register */
	return (int32_t)uart_reg_ptr->rbr_thr_dll;
}

/**
 * @fn		void uart_send_a_char_blocking (uart_reg_set_t *uart_reg_ptr,
											char 			chr)
 * @brief	send one char to transmit holding register(thr) using blocking method
 * @note	this function is only used in polling method
 * @param	uart_reg_ptr	: Pointer to uart register set structure
 * @param	chr				: character which needs to send to thr
 * @retval 	none
 */
static __inline void uart_send_a_char_blocking (uart_reg_set_t *uart_reg_ptr,
												char 			chr)
{
	/* wait until uart is to ready to send */
	while (!uart_tx_ready(uart_reg_ptr)); /* blocked */

	/* send char to thr */
	uart_send_a_char_to_thr(uart_reg_ptr, chr);
}

/**
 * @fn		int32_t uart_receive_a_char_blocking (uart_reg_set_t *uart_reg_ptr)
 * @brief	receive one char from receive buffer register(rbr) using blocking method
 * @note	this function is only used in polling method
 * @param	uart_reg_ptr	: Pointer to uart register set structure
 * @retval 	received char from rbr
 */
static __inline int32_t uart_receive_a_char_blocking (uart_reg_set_t *uart_reg_ptr)
{
	/* wait until uart is ready to receive */
	while (!uart_rx_ready(uart_reg_ptr)); /* blocked */
	/* receive data from rbr */
	return uart_receive_a_char_from_rbr(uart_reg_ptr);
}

/**
 * @fn		int32_t uart_send_max_data_polling (uart_resources_t *uart,
												const void 		 *data,
												uint32_t 		  len)
 * @brief	send maximum length data to transmit holding register thr using polling method
 * @note	this function uses polling / blocking method
 * @param	uart	: Pointer to uart resources structure
 * @param	data	: Pointer to input data which needs to send
 * @param	len		: maximum length of the data
 * @retval 	count of send bytes
 */
int32_t uart_send_max_data_polling (uart_resources_t *uart,
									const void 		 *data,
									uint32_t 		  len)
{
	int32_t send_cnt = 0;
	const char *p_charbuf = (const char *)data;
	uart_reg_set_t *uart_reg_ptr = (uart_reg_set_t *)uart->reg_base;

	while (send_cnt < len)
	{
		/* send one char to thr */
		uart_send_a_char_blocking(uart_reg_ptr, p_charbuf[send_cnt++]); /* blocked */
	}

	return send_cnt;
}

/**
 * @fn		int32_t uart_receive_max_data_polling (uart_resources_t *uart,
									   	   	   	   void 			*data,
									   	   	   	   uint32_t 		 len)
 * @brief	receive maximum length data from receive buffer register rbr using polling method
 * @note	this function uses polling / blocking method
 * @param	uart	: Pointer to uart resources structure
 * @param	data	: Pointer to output data which needs to receive
 * @param	len		: maximum length of the data
 * @retval 	count of received bytes
 */
int32_t uart_receive_max_data_polling (uart_resources_t *uart,
									   void 			*data,
									   uint32_t 		 len)
{
	int32_t rcv_cnt = 0;
	char *p_charbuf = (char *)data;
	uart_reg_set_t *uart_reg_ptr = (uart_reg_set_t *)uart->reg_base;

	while (rcv_cnt < len)
	{
		/* receive one char from rbr */
		p_charbuf[rcv_cnt++] = uart_receive_a_char_blocking(uart_reg_ptr); /* blocked */
	}

	return rcv_cnt;
}

/**
 * @fn		void uart_software_reset (uart_reg_set_t *uart_reg_ptr)
 * @brief	uart software reset
 * @note	use shadow register srr software reset register if shadow mode is enable
 * 			else use fcr FIFO control register.
 * 			SHADOW mode is always enable in ip.
 * @param	uart_reg_ptr: Pointer to uart register set structure
 * @retval 	none
 */
static __inline void uart_software_reset (uart_reg_set_t *uart_reg_ptr)
{
	/* UART_CPR_SHADOW_MODE is always enable in ip. */

	/* use shadow register srr software reset register */
	uart_reg_ptr->srr = UART_SRR_UART_RESET|UART_SRR_RCVR_FIFO_RESET|UART_SRR_TRANSMIT_FIFO_RESET;

    /* wait until software reset completed */
    while(uart_reg_ptr->usr & UART_USR_BUSY);
}

/**
 * @fn		void uart_set_baudrate (uart_resources_t *uart,
							   	    uint32_t 		  arg)
 * @brief	set uart baudrate
 * @note	added support for fraction in dlf divisor latch fraction register
 * @param	uart	: Pointer to uart resources structure
 * @param	arg		: baudrate
 * @retval 	none
 */
static void uart_set_baudrate (uart_resources_t *uart,
							   uint32_t 		 arg)
{
	uint32_t baud 			= 0;
	int32_t baud_divisor 	= 0;
	int32_t fraction 		= 0;
	float decimal 			= 0.0;
	int i					= 0;
	uint32_t freq			= 0;

	uart_info_t 	*uart_info_ptr 	= uart->info;
	uart_reg_set_t 	*uart_reg_ptr 	= (uart_reg_set_t *)uart->reg_base;

	/* take frequency as a peripheral clock */
	freq = uart->clk;
	baud = arg;

	if(baud)
	{
		decimal = UART_CONVERT_BAUD_TO_DIVISOR((float)freq, (float)baud);
		baud_divisor = (int32_t)UART_CONVERT_BAUD_TO_DIVISOR(freq, baud);
		fraction = (int32_t)((decimal - baud_divisor)*(1 << UART_DLF_SIZE));
	}

	/* update baudrate to uart resource structure */
	uart_info_ptr->baudrate = baud;

	/* enable DLAB divisor latch access bit in lcr line control register  */
	uart_reg_ptr->lcr |= UART_LCR_DLAB;

	/* setting uart baudrate registers	 */
	uart_reg_ptr->rbr_thr_dll = baud_divisor & 0xff;	/* DLL divisor latch low register */
	uart_reg_ptr->ier_dlh = (baud_divisor>>8) & 0xff;	/* DLH divisor latch high register */
	uart_reg_ptr->dlf = fraction;   					/* DLF divisor latch fraction register */

	/* disable DLAB */
	uart_reg_ptr->lcr &= ~(UART_LCR_DLAB);

	/* hardware requires this delay before operating on new baud */
	for(i = 0; i < (32 * baud_divisor * 1); i++);
}

/**
 * @fn		int32_t uart_set_asynchronous_mode (uart_resources_t *uart,
												uint32_t          control,
												uint32_t          arg)
 * @brief	set uart asynchronous parameters
 * 			baudrate, data bits, parity, stop bits, flow control
 * @note	none
 * @param	uart							: Pointer to uart resources structure
 * @param	control							: Control Operation
 * @param	arg								: Argument of operation (optional)
 * @retval	ARM_USART_ERROR_DATA_BITS		: error in data length
 * @retval	ARM_USART_ERROR_PARITY			: error in parity
 * @retval	ARM_USART_ERROR_STOP_BITS		: error in stop bits
 * @retval	ARM_USART_ERROR_FLOW_CONTROL	: error in flow control
 * @retval	ARM_DRIVER_OK					: success
 */
int32_t uart_set_asynchronous_mode (uart_resources_t *uart,
									uint32_t          control,
									uint32_t          arg)
{
	int lcr 				= 0;
	int mcr 				= 0;
	uint32_t baud 			= 0;
	int32_t baud_divisor 	= 0;

	uart_reg_set_t 	*uart_reg_ptr 	= (uart_reg_set_t *)uart->reg_base;

	/* set the uart baudrate */
	uart_set_baudrate(uart, arg);

    /* UART Data bits */
    switch (control & ARM_USART_DATA_BITS_Msk)
    {
		/* Data bit is not configurable */
		/* set DLS data_length_select bit in lcr line control register */
		case ARM_USART_DATA_BITS_5: lcr |= (UART_LCR_DATA_LENGTH_5); break;
		case ARM_USART_DATA_BITS_6: lcr |= (UART_LCR_DATA_LENGTH_6); break;
		case ARM_USART_DATA_BITS_7: lcr |= (UART_LCR_DATA_LENGTH_7); break;
		case ARM_USART_DATA_BITS_8: lcr |= (UART_LCR_DATA_LENGTH_8); break;
		default: return ARM_USART_ERROR_DATA_BITS;
    }

    /* UART Parity */
    switch (control & ARM_USART_PARITY_Msk)
    {
		/* set PEN parity enable, EPS even parity select bit in lcr line control register */
		case ARM_USART_PARITY_NONE: lcr |= (UART_LCR_PARITY_NONE); break;
		case ARM_USART_PARITY_EVEN: lcr |= (UART_LCR_PARITY_EVEN); break;
		case ARM_USART_PARITY_ODD:  lcr |= (UART_LCR_PARITY_ODD);  break;
		default: return ARM_USART_ERROR_PARITY;
    }

    /* UART Stop bits */
    switch (control & ARM_USART_STOP_BITS_Msk)
    {
		/* set STOP number_of_stop_bits in lcr line control register */
		case ARM_USART_STOP_BITS_1: lcr |= (UART_LCR_STOP_1BIT); break;
		case ARM_USART_STOP_BITS_2: lcr |= (UART_LCR_STOP_2BIT); break;
		default: return ARM_USART_ERROR_STOP_BITS;
    }

	/* clear data,parity,stop bits */
	uart_reg_ptr->lcr &= (~UART_LCR_DATA_PARITY_STOP_MASK);

	/* set data,parity,stop bits */
	uart_reg_ptr->lcr |= lcr;

	/* uart flow control */
    switch (control & ARM_USART_FLOW_CONTROL_Msk)
    {
		/* set flow control bit in mcr modem control register */
		case ARM_USART_FLOW_CONTROL_NONE:
			uart_reg_ptr->mcr &= ~(UART_MCR_AFCE|UART_MCR_RTS);
			break;
		case ARM_USART_FLOW_CONTROL_RTS:
			uart_reg_ptr->mcr |= (UART_MCR_AFCE|UART_MCR_RTS);
			break;
		case ARM_USART_FLOW_CONTROL_CTS:
			uart_reg_ptr->mcr |= (UART_MCR_AFCE);
			break;
		case ARM_USART_FLOW_CONTROL_RTS_CTS:
			uart_reg_ptr->mcr |= (UART_MCR_AFCE|UART_MCR_RTS);
			break;
		default: return ARM_USART_ERROR_FLOW_CONTROL;
    }

    return ARM_DRIVER_OK;
}

/**
 * @fn		int32_t uart_set_rx_trigger (uart_resources_t  *uart,
							 	 	 	 uart_rx_trigger_t  rx_trigger)
 * @brief	set uart receiver trigger level
 * 			This is used to select the trigger level in the receiver FIFO
 * 			at which the Received Data Available Interrupt will be generated.
 * @note	/ref uart_rx_trigger_t enum
 * @param	uart		: Pointer to uart resources structure
 * @param	rx_trigger	: enum uart_rx_trigger_t
 * @retval	SUCCESS		:  0
 * @retval	FAILURE		: -1
 */
int32_t uart_set_rx_trigger (uart_resources_t  *uart,
							 uart_rx_trigger_t  rx_trigger)
{
	uart_reg_set_t *uart_reg_ptr = (uart_reg_set_t *)uart->reg_base;

	if ((rx_trigger < UART_RX_ONE_CHAR_IN_FIFO) || (rx_trigger > UART_RX_FIFO_TWO_LESS_FULL )) return -1;

	/* UART_CPR_SHADOW_MODE is always enable in ip. */

	/* update srt shadow receiver trigger register */
	uart_reg_ptr->srt = rx_trigger;
	return 0;
}

/**
 * @fn		uart_rx_trigger_t uart_get_rx_trigger (uart_resources_t *uart)
 * @brief	get uart receiver trigger level
 * @note	/ref uart_rx_trigger_t and uart_set_rx_trigger
 * @param	uart: Pointer to uart resources structure
 * @retval 	enum uart_rx_trigger_t
 */
uart_rx_trigger_t uart_get_rx_trigger (uart_resources_t *uart)
{
	uart_reg_set_t *uart_reg_ptr = (uart_reg_set_t *)uart->reg_base;

	/* UART_CPR_SHADOW_MODE is always enable in ip. */

	/* read srt shadow receiver trigger register */
	return uart_reg_ptr->srt;
}

/**
 * @fn		int32_t uart_set_tx_trigger (uart_resources_t  *uart,
							 	 	 	 uart_tx_trigger_t  tx_trigger)
 * @brief	set uart transmitter trigger level
 * 			This is used to select the empty threshold level in the transmitter FIFO
 * 			at which the THRE Interrupts will be generated.
 * @note	/ref uart_tx_trigger_t enum
 * @param	uart		: Pointer to uart resources structure
 * @param	tx_trigger	: enum uart_tx_trigger_t
 * @retval	SUCCESS		:  0
 * @retval	FAILURE		: -1
 */
int32_t uart_set_tx_trigger (uart_resources_t  *uart,
							 uart_tx_trigger_t  tx_trigger)
{
	uart_reg_set_t *uart_reg_ptr = (uart_reg_set_t *)uart->reg_base;

	if ((tx_trigger < UART_TX_FIFO_EMPTY) || (tx_trigger > UART_TX_FIFO_HALF_FULL )) return -1;

	/* UART_CPR_SHADOW_MODE is always enable in ip. */

	/* update stet shadow TX empty trigger register */
	uart_reg_ptr->stet = tx_trigger;
	return 0;
}

/**
 * @fn		uart_tx_trigger_t uart_get_tx_trigger (uart_resources_t *uart)
 * @brief	get uart transmitter trigger level
 * @note	/ref uart_tx_trigger_t and uart_set_tx_trigger
 * @param	uart: Pointer to uart resources structure
 * @retval 	enum uart_tx_trigger_t
 */
uart_tx_trigger_t uart_get_tx_trigger (uart_resources_t *uart)
{
	uart_reg_set_t *uart_reg_ptr = (uart_reg_set_t *)uart->reg_base;

	/* UART_CPR_SHADOW_MODE is always enable in ip. */

	/* read stet shadow TX empty trigger register */
	return uart_reg_ptr->stet;
}

/**
 * @fn		int32_t uart_get_tx_fifo_available_count (uart_resources_t *uart)
 * @brief	get available transmit fifo count
 * @note	useful in polling and interrupt callback
 * @param	uart: Pointer to uart resources structure
 * @retval 	available transmit fifo count
 */
int32_t uart_get_tx_fifo_available_count (uart_resources_t *uart)
{
	uart_reg_set_t 	*uart_reg_ptr 	= (uart_reg_set_t *)uart->reg_base;

	/* FIFO_STAT, FIFO MODE and ADDITIONAL_FEATURES are always enabled in ip.
	 * (FIFO_STAT == 1) && (FIFO_MODE != 0) && (ADDITIONAL_FEATURES == 1)
	 */

	/* read tfl transmit FIFO level register,
	 * TX_fifo available count = fifo depth - data entries in transmit fifo
	 */
	return (uart->info->tx_fifo_length - uart_reg_ptr->tfl);
}

/**
 * @fn		int32_t uart_get_rx_fifo_available_count (uart_resources_t *uart)
 * @brief	get available receive fifo count
 * @note	useful in polling and interrupt callback
 * @param	uart: Pointer to uart resources structure
 * @retval 	available receive fifo count
 */
int32_t uart_get_rx_fifo_available_count (uart_resources_t *uart)
{
	uart_reg_set_t	*uart_reg_ptr  = (uart_reg_set_t *)uart->reg_base;

	/* FIFO_STAT, FIFO MODE and ADDITIONAL_FEATURES are always enabled in ip.
	 * (FIFO_STAT == 1) && (FIFO_MODE != 0) && (ADDITIONAL_FEATURES == 1)
	 */

	/* read rfl receive FIFO level register */
	return (uart_reg_ptr->rfl);
}

/**
 * @fn		void uart_set_break_control (uart_resources_t *uart)
 * @brief 	set uart break control
 * @note	none
 * @param	uart: Pointer to uart resources structure
 * @retval 	none
 */
void uart_set_break_control (uart_resources_t *uart)
{
	uart_reg_set_t *uart_reg_ptr = (uart_reg_set_t *)uart->reg_base;
	/* set break_control bit in lcr line control register. */
	uart_reg_ptr->lcr |= UART_LCR_BREAK;
}

/**
 * @fn		void uart_clear_break_control (uart_resources_t *uart)
 * @brief 	clear uart break control
 * @note	none
 * @param	uart: Pointer to uart resources structure
 * @retval 	none
 */
void uart_clear_break_control (uart_resources_t *uart)
{
	uart_reg_set_t *uart_reg_ptr = (uart_reg_set_t *)uart->reg_base;
	/* clear break_control bit in lcr line control register. */
	uart_reg_ptr->lcr &= ~UART_LCR_BREAK;
}

/**
 * @fn		static void uart_enable_irq (uart_resources_t *uart,
							 	 	 	 uint32_t 		   arg)
 * @brief	enable uart transmit and receive interrupt
 * @note	only one combined interrupt is available for TX/RX
 * 			so if any one is enable then no need to enable interrupt again.
 * @param	uart	: Pointer to uart resources structure
 * @param	arg		: UART_ENABLE_TRANSMITTER_INT or UART_ENABLE_RECEIVER_INT
 * @retval 	none
 */
static void uart_enable_irq (uart_resources_t *uart,
							 uint32_t 		   arg)
{
	uart_info_t 	*uart_info_ptr = uart->info;
	uart_reg_set_t 	*uart_reg_ptr  = (uart_reg_set_t *)uart->reg_base;

	/* pass only if parameter is TX/RX enable interrupt */
	if( (arg != UART_ENABLE_TRANSMITTER_INT) && (arg != UART_ENABLE_RECEIVER_INT) ) return;

	/* enable transmitter interrupt */
	if(arg == UART_ENABLE_TRANSMITTER_INT)
	{
		/* enable transmit_holding_register_empty bit in ier interrupt enable register */
		uart_reg_ptr->ier_dlh |= UART_IER_ENABLE_TRANSMIT_HOLD_REG_EMPTY;

		/* set TX interrupt enable flag */
		uart_info_ptr->int_status |= UART_FLAG_TX_INT_ENABLE;
	}

	/* enable receiver interrupt */
	if(arg == UART_ENABLE_RECEIVER_INT)
	{
		/* enable receive_data_available_interrupt bit in ier interrupt enable register */
		uart_reg_ptr->ier_dlh |= UART_IER_ENABLE_RECEIVED_DATA_AVAILABLE;

		/* set RX interrupt enable flag */
		uart_info_ptr->int_status |= UART_FLAG_RX_INT_ENABLE;
	}

	/* only one combined interrupt is available for TX/RX
	 * so if any one is enable then no need to enable interrupt again.
	 */

	/* if TX and RX both are disable then only enable interrupt. */
	if ((uart_info_ptr->int_status & UART_FLAG_TX_OR_RX_INT_ENABLE) == 0) /* TX and RX both are disable. */
	{
		/* anyone should be enable either TX or RX */
		if ( (uart_info_ptr->int_status & UART_FLAG_TX_INT_ENABLE) ||
			 (uart_info_ptr->int_status & UART_FLAG_RX_INT_ENABLE) )
		{
			if (uart->irq_num != UART_INVALID_INT_NUM)
			{
				if (uart->irq_priority != UART_INVALID_PRIORITY)
				{
					NVIC_SetPriority(uart->irq_num, uart->irq_priority);
					if (NVIC_GetPriority(uart->irq_num) != uart->irq_priority)
					{
						return; /* error */
					}
				}
				/* enable the NVIC interrupt. */
				NVIC_EnableIRQ(uart->irq_num);
			}
			/* set the global flag as TX or RX interrupt enable. */
			uart_info_ptr->int_status |= UART_FLAG_TX_OR_RX_INT_ENABLE;
		}
	}
	/* else interrupt is already enabled. */
}

/**
 * @fn		static void uart_disable_irq (uart_resources_t *uart,
							  	  	  	  uint32_t 			arg)
 * @brief	disable uart transmit and receive interrupt
 * @note	only one combined interrupt is available for TX/RX
 * 			so if both TX and RX are disable then only disable interrupt.
 * @param	uart	: Pointer to uart resources structure
 * @param	arg		: UART_DISABLE_TRANSMITTER_INT or UART_DISABLE_RECEIVER_INT
 * @retval 	none
 */
static void uart_disable_irq (uart_resources_t *uart,
							  uint32_t 			arg)
{
	uart_info_t 	*uart_info_ptr = uart->info;
	uart_reg_set_t 	*uart_reg_ptr  = (uart_reg_set_t *)uart->reg_base;

	/* pass only if parameter is TX/RX diable interrupt */
	if( (arg != UART_DISABLE_TRANSMITTER_INT) && (arg != UART_DISABLE_RECEIVER_INT) ) return;

	/* disable transmit interrupt */
	if(arg == UART_DISABLE_TRANSMITTER_INT)
	{
		/* disable transmit_holding_register_empty bit in ier interrupt enable register */
		uart_reg_ptr->ier_dlh &= ~UART_IER_ENABLE_TRANSMIT_HOLD_REG_EMPTY;

		/* reset TX interrupt enable flag */
		uart_info_ptr->int_status &= ~UART_FLAG_TX_INT_ENABLE;
	}

	/* disable receiver interrupt */
	if(arg == UART_DISABLE_RECEIVER_INT)
	{
		/* disable receive_data_available_interrupt bit in ier interrupt enable register */
		uart_reg_ptr->ier_dlh &= ~UART_IER_ENABLE_RECEIVED_DATA_AVAILABLE;

		/* reset RX interrupt enable flag */
		uart_info_ptr->int_status &= ~UART_FLAG_RX_INT_ENABLE;
	}

	/* only one combined interrupt is available for TX/RX
	 * so if both TX and RX are disable then only disable interrupt.
	 */

	/* disable if anyone TX or RX interrupt is already enabled. */
	if (uart_info_ptr->int_status & UART_FLAG_TX_OR_RX_INT_ENABLE)
	{
		/* both TX and RX flag should be disable then only disable interrupt. */
		if ( ((uart_info_ptr->int_status & UART_FLAG_TX_INT_ENABLE) == 0) &&
			 ((uart_info_ptr->int_status & UART_FLAG_RX_INT_ENABLE) == 0) )
		{
			if (uart->irq_num != UART_INVALID_INT_NUM)
			{
				/* disable the NVIC interrupt. */
				NVIC_DisableIRQ(uart->irq_num);
			}
			/* set the global flag as TX or RX interrupt is enable. */
			uart_info_ptr->int_status &= ~UART_FLAG_TX_OR_RX_INT_ENABLE;
		}
	}
}

/**
 * @fn		int32_t uart_enable_send_interrupt (uart_resources_t *uart,
								   	   	   	    const void 		 *data,
								   	   	   	   	uint32_t 		  len)
 * @brief 	setup for send interrupt
 * 			this function will fill the uart transfer structure as per user input details
 * 			and enable the transmitter interrupt
 * @note	none
 * @param	uart	: Pointer to uart resources structure
 * @param	data	: Pointer to input data
 * @param	len		: total length of the send data
 * @retval  SUCCESS	: ARM_DRIVER_OK
 * @retval  FAILURE	: ARM_DRIVER_ERROR_BUSY.
 */
int32_t uart_enable_send_interrupt (uart_resources_t *uart,
								   const void 		*data,
								   uint32_t 		 len)
{
	const char *p_charbuf = (const char *)data;

	uart_info_t *uart_info_ptr  = uart->info;

	/* check previous send is completed or not? */
	if (uart_info_ptr->transfer.send_busy != 0U)
	{
		/* return busy as previous send is not yet completed */
	    return ARM_DRIVER_ERROR_BUSY;
	}

	/* Set send busy flag to active */
	uart_info_ptr->transfer.send_busy = 1U;

	/* fill the uart transfer structure as per user input */
	uart_info_ptr->transfer.tx_buf = (uint8_t *)data;
	uart_info_ptr->transfer.tx_total_num = len;
	uart_info_ptr->transfer.tx_curr_cnt = 0U;

	/* enable the transmitter interrupt.*/
	uart_enable_irq(uart, UART_ENABLE_TRANSMITTER_INT);
	return ARM_DRIVER_OK;
}

/**
 * @fn		int32_t uart_enable_receive_interrupt (uart_resources_t *uart,
									  	  	  	   const void 	   	*data,
									  	  	  	   uint32_t 		 len)
 * @brief 	setup for receive interrupt
 * 			this function will fill the uart receiver structure as per user input details
 * 			and enable the receiver interrupt
 * @note	none
 * @param	uart	: Pointer to uart resources structure
 * @param	data	: Pointer to output data
 * @param	len		: total length of the receive data
 * @retval  SUCCESS	: ARM_DRIVER_OK
 * @retval  FAILURE	: ARM_DRIVER_ERROR_BUSY.
 */
int32_t uart_enable_receive_interrupt (uart_resources_t *uart,
									  const void 	   *data,
									  uint32_t 			len)
{
	uart_info_t *uart_info_ptr = uart->info;

	/* check previous receive is completed or not? */
	if (uart->info->rx_status.rx_busy == 1U)
	{
		/* return busy as previous receive is not yet completed */
		return ARM_DRIVER_ERROR_BUSY;
	}

	/* set rx busy flag to active */
	uart_info_ptr->rx_status.rx_busy = 1U;

	/* clear rx status */
	uart_info_ptr->rx_status.rx_break          = 0U;
	uart_info_ptr->rx_status.rx_framing_error  = 0U;
	uart_info_ptr->rx_status.rx_overflow       = 0U;
	uart_info_ptr->rx_status.rx_parity_error   = 0U;

	/* fill the uart transfer structure as per user input */
	uart_info_ptr->transfer.rx_buf = (uint8_t *)data;
	uart_info_ptr->transfer.rx_total_num = len;
	uart_info_ptr->transfer.rx_curr_cnt = 0U;

	/* enable the receiver interrupt. */
	uart_enable_irq(uart, UART_ENABLE_RECEIVER_INT);
	return ARM_DRIVER_OK;
}

/**
 * @fn		void uart_reset_txfifo (uart_reg_set_t *uart_reg_ptr)
 * @brief	reset transmit fifo
 * @note	none
 * @param	uart_reg_ptr	: Pointer to uart register set structure
 * @retval 	none
 */
static __inline void uart_reset_txfifo (uart_reg_set_t *uart_reg_ptr)
{
	/* UART_CPR_SHADOW_MODE is always enable in ip. */

	/* set XMIT_FIFO_Reset bit in shadow register srr software reset register */
	uart_reg_ptr->srr  = UART_SRR_TRANSMIT_FIFO_RESET;

	/* wait until software reset completed */
    while(uart_reg_ptr->usr & UART_USR_BUSY);
}

/**
 * @fn		void uart_reset_rxfifo (uart_reg_set_t *uart_reg_ptr)
 * @brief	reset receiver fifo
 * @note	none
 * @param	uart_reg_ptr	: Pointer to uart register set structure
 * @retval 	none
 */
static __inline void uart_reset_rxfifo (uart_reg_set_t *uart_reg_ptr)
{
	/* UART_CPR_SHADOW_MODE is always enable in ip. */

	/* set RCVR_FIFO_Reset bit in shadow register srr software reset register */
	uart_reg_ptr->srr  = UART_SRR_RCVR_FIFO_RESET;

	/* wait until software reset completed */
    while(uart_reg_ptr->usr & UART_USR_BUSY);
}

/**
 * @fn		void uart_flush_rxfifo (uart_resources_t *uart)
 * @brief	flush receiver fifo
 * @note	/ref uart_abort_rx
 * @param	uart: Pointer to uart resources structure
 * @retval 	none
 */
void uart_flush_rxfifo (uart_resources_t *uart)
{
	uart_info_t 	*uart_info_ptr = uart->info;
	uart_reg_set_t 	*uart_reg_ptr  = (uart_reg_set_t *)uart->reg_base;

	/* read all the bytes which are available in RX fifo till given rx total length */
	while ( uart_rx_ready(uart_reg_ptr) && (uart_info_ptr->transfer.rx_curr_cnt != uart_info_ptr->transfer.rx_total_num) )
	{
		/* read a char from rbr receive buffer register. */
		uart_info_ptr->transfer.rx_buf[uart_info_ptr->transfer.rx_curr_cnt] = uart_receive_a_char_from_rbr(uart_reg_ptr);
		uart_info_ptr->transfer.rx_curr_cnt++;
	}
}

/**
 * @fn		void uart_abort_tx (uart_resources_t *uart)
 * @brief	abort transmitter
 * @note	none
 * @param	uart: Pointer to uart resources structure
 * @retval 	none
 */
void uart_abort_tx (uart_resources_t *uart)
{
	uart_info_t 	*uart_info_ptr = uart->info;
	uart_reg_set_t 	*uart_reg_ptr  = (uart_reg_set_t *)uart->reg_base;

	/* if tx interrupt flag is enable then only disable transmit interrupt */
	if (uart_info_ptr->int_status & UART_FLAG_TX_INT_ENABLE)
	{
		/* disable the transmit interrupt.*/
		uart_disable_irq(uart, UART_DISABLE_TRANSMITTER_INT);
	}

	/* reset TX fifo */
	uart_reset_txfifo(uart_reg_ptr);

	/* clear Send active flag */
	uart_info_ptr->transfer.send_busy = 0U;

	/* Reset the tx_buffer */
	uart_info_ptr->transfer.tx_total_num = 0U;
}

/**
 * @fn		void uart_abort_rx (uart_resources_t *uart)
 * @brief	abort receiver
 * @note	none
 * @param	uart: Pointer to uart resources structure
 * @retval 	none
 */
void uart_abort_rx (uart_resources_t *uart)
{
	uart_info_t 	*uart_info_ptr = uart->info;
	uart_reg_set_t	*uart_reg_ptr  = (uart_reg_set_t *)uart->reg_base;

	/* if rx interrupt flag is enable then only disable receiver interrupt */
	if (uart_info_ptr->int_status & UART_FLAG_RX_INT_ENABLE)
	{
		/* disable the receiver interrupt. */
		uart_disable_irq(uart, UART_DISABLE_RECEIVER_INT);
	}

	/* flush-out rx fifo */
	uart_flush_rxfifo(uart);

    /* Reset rx fifo */
	uart_reset_rxfifo(uart_reg_ptr);

	/* clear Receive busy flag */
	uart_info_ptr->rx_status.rx_busy 		= 0U;
	uart_info_ptr->transfer.rx_total_num 	= 0U;
}

/**
 * @fn		void uart_disable_interrupt (uart_resources_t *uart)
 * @brief 	disable transmit and receive interrupt
 * @note	none
 * @param	uart: Pointer to uart resources structure
 * @retval 	none
 */
static void uart_disable_interrupt (uart_resources_t *uart)
{
	/* disable uart transmit interrupt  */
	uart_disable_irq(uart, UART_DISABLE_TRANSMITTER_INT);

	/* disable uart receive interrupt  */
	uart_disable_irq(uart, UART_DISABLE_RECEIVER_INT);

	/* disable uart interrupt */
	if (uart->irq_num != UART_INVALID_INT_NUM)
	{
		/* disable NVIC interrupt. */
		NVIC_DisableIRQ(uart->irq_num);
	}

	/* reset the global flag for interrupt. */
	uart->info->int_status &= ~(UART_FLAG_TX_OR_RX_INT_ENABLE|UART_FLAG_TX_INT_ENABLE|UART_FLAG_RX_INT_ENABLE);
}

/**
 * @fn		void uart_irq_handler (uart_resources_t *uart)
 * @brief	uart interrupt handler
 * @note	only one combined interrupt for
 * 				-TX / RX
 * 				-RX_Character_Timeout
 * 				-Modem status
 * 				-Receiver Line status
 *			in RX_Timeout case not clearing rx busy flag, it is up to user to decide whether
 *			to wait for remaining bytes or call the abort rx. /ref abort_rx
 * @param	uart: Pointer to uart resources structure
 * @retval 	none
 */
void uart_irq_handler (uart_resources_t *uart)
{
	uint32_t uart_int_status = 0U; 		/* uart interrupt status */
	volatile uint32_t temp	 = 0U; 		/* read error status to clear interrupt */
	uint32_t event 		   	 = 0U; 		/* callback event */
	uint32_t tx_fifo_available_cnt 	 = 0U; 		/* TX fifo Available count. */
	uint32_t rx_fifo_available_cnt 	 = 0U; 		/* RX fifo Available count. */
	uint32_t i = 0U;


	uart_info_t 	*uart_info_ptr = uart->info;
	uart_reg_set_t 	*uart_reg_ptr  = (uart_reg_set_t *)uart->reg_base;

	/* get uart interrupt status from iir interrupt identity register */
	uart_int_status = (uart_reg_ptr->iir_fcr) & UART_IIR_INTERRUPT_ID_MASK;

	switch (uart_int_status)
	{
	    case UART_IIR_MODEM_STATUS: /* modem status */

			temp = (volatile uint32_t)(uart_reg_ptr->msr);
			/* yet not implemented. */
			break;

		case UART_IIR_RECEIVER_LINE_STATUS: /* receiver line status */

			/* yet not implemented. */
			break;

		case UART_IIR_TRANSMIT_HOLDING_REG_EMPTY: /* transmit holding register empty */

			do
			{
				/* Query how many characters are available in TX fifo. */
				tx_fifo_available_cnt = uart_get_tx_fifo_available_count (uart);

				/* Write maximum number of characters to the TX fifo as per available space. */
				for(i=0; i<tx_fifo_available_cnt; i++)
				{
					if(uart_info_ptr->transfer.tx_curr_cnt >= uart_info_ptr->transfer.tx_total_num)
					{
						/* Come out as it transmitted all the user data. */
						break;
					}

					/* send character to thr register. */
					uart_send_a_char_to_thr(uart_reg_ptr, uart_info_ptr->transfer.tx_buf[uart_info_ptr->transfer.tx_curr_cnt]);
					uart_info_ptr->transfer.tx_curr_cnt++; /* increment the tx current count */
				}

				/* write again to tx fifo if it is not full and user data is remaining to send. */
			} while( uart_tx_ready(uart_reg_ptr) && (uart_info_ptr->transfer.tx_curr_cnt < uart_info_ptr->transfer.tx_total_num) );

			/* check whether it transmitted all the bytes? */
			if (uart_info_ptr->transfer.tx_curr_cnt >= uart_info_ptr->transfer.tx_total_num)
			{
				/* yes then disable the transmitter interrupt */
				uart_disable_irq(uart, UART_DISABLE_TRANSMITTER_INT);

				/* clear tx busy flag */
				uart_info_ptr->transfer.send_busy = 0U;

				/* mark event as send Complete */
				event |= ARM_USART_EVENT_SEND_COMPLETE;
			}
			break;

		case UART_IIR_CHARACTER_TIMEOUT:		/* character timeout */
		case UART_IIR_RECEIVED_DATA_AVAILABLE:	/* received data available. */

			do
			{
				/* Query how many characters are available in RX fifo. */
				rx_fifo_available_cnt = uart_get_rx_fifo_available_count (uart);

				/* Read maximum number of characters available from the RX fifo or till rx total number. */
				for(i=0; i<rx_fifo_available_cnt; i++)
				{
					if (uart_info_ptr->transfer.rx_curr_cnt >= uart_info_ptr->transfer.rx_total_num)
					{
						/* Come out as it received all the user data. */
						break;
					}

					/* read character from rbr receive buffer register. */
					uart_info_ptr->transfer.rx_buf[uart_info_ptr->transfer.rx_curr_cnt] = uart_receive_a_char_from_rbr(uart_reg_ptr);
					uart_info_ptr->transfer.rx_curr_cnt++;
				}

				/* read again from rx fifo if it is not empty and data is remaining to read. */
			} while( uart_rx_ready(uart_reg_ptr) && (uart_info_ptr->transfer.rx_curr_cnt < uart_info_ptr->transfer.rx_total_num) );


			/* check whether it received all the bytes? */
			if (uart_info_ptr->transfer.rx_curr_cnt >= uart_info_ptr->transfer.rx_total_num)
			{
				/* yes than disable the receiver interrupt */
				uart_disable_irq(uart, UART_DISABLE_RECEIVER_INT);

				/* clear rx busy flag */
				uart_info_ptr->rx_status.rx_busy = 0U;

				/* mark event as receive complete */
				event |= ARM_USART_EVENT_RECEIVE_COMPLETE;
			}
			else /* fifo is empty. */ /* Added on 9-Oct-2020 */
			{
				/* FIXME After debugging we found that without else case, code is getting hang here as
				 * 1.) fifo is empty here, (as we are reading fifo in while loop),
				 *     and if we have not read total number of RX bytes.
				 * 2.) RX_CHAR_TIMEOUT interrupt will not come as fifo is empty. (As per Synopsys Datasheet page no: 51)
				 * 3.) Added this to fix timeout issue (work-around)
				 * 4.) scenario: expecting 10 bytes but receiving only 1 byte? needs one more upper layer timeout.
				 * 5.) or Mark as separate event for "FIFO_EMPTY", Don't include in RX_Timeout.
				 */


				/* in RX_Timeout case not clearing rx busy flag
				 * it is up to user to decide whether
				 * to wait for remaining bytes or call the abort rx.
				*/
				/* mark event as RX Timeout */
				event |= ARM_USART_EVENT_RX_TIMEOUT;
			}

			/* got character Timeout? mark event as a RX Timeout. */
			if (uart_int_status == UART_IIR_CHARACTER_TIMEOUT)
			{
				/* in RX_Timeout case not clearing rx busy flag
				 * it is up to user to decide whether
				 * to wait for remaining bytes or call the abort rx.
				 */
				/* mark event as RX Timeout */
				event |= ARM_USART_EVENT_RX_TIMEOUT;
			}

			break;

		default:
			/* read the usr uart status register */
			temp = (volatile uint32_t)(uart_reg_ptr->usr);
			break;
	}

	/* call the user callback if any event occurs */
	if ((event != 0U) && (uart_info_ptr->cb_event != NULL) )
	{
		/* call the user callback */
		uart_info_ptr->cb_event(event);
	}

	return;
}

/**
 * @fn		int32_t uart_Configure_ClockSource (bool enable, uart_resources_t *uart)
 * @brief	uart clock configuration: control UART Interface Power.
 * @note	none
 * @param	enable: Clock state
 * @param	uart: 	Pointer to uart resources structure
 * @retval 	\ref execution_status
 */
static int32_t uart_Configure_ClockSource (bool enable, uart_resources_t *uart)
{
	uint32_t clock_select = 0;

	if (enable)
	{
		if (uart->cfg->clk_source == UART_CLK_SOURCE_0)
			clock_select = 38400000UL; //38.4 Mhz
		else if (uart->cfg->clk_source == UART_CLK_SOURCE_1)
			clock_select = 100000000UL; //100 Mhz
		else if (uart->cfg->clk_source == UART_CLK_SOURCE_2)
			clock_select = 20000000UL; //20 Mhz //fpga
		else
			return ARM_DRIVER_ERROR_PARAMETER;

		/* update the clock frequency. */
		uart->clk = (uint32_t)clock_select;
	}

	return ARM_DRIVER_OK;
}

/**
 * @fn		int32_t uart_configure_control_reg (bool enable, uart_resources_t *uart)
 * @brief	uart EXPMST0 control register configuration
 * 			This function will configure
 * 			 - Enable bits for selected UART Clock Source
 * 			 - Enable selected UART Module
 * @note	none
 * @param	none
 * @retval 	none
 */
static int32_t uart_configure_control_reg (bool enable, uart_resources_t *uart)
{

#define BYPASS_CLK 		 1
#define UART_CTRL_OFFSET (0x08) /* UART ctrl offset */

#if BYPASS_CLK //bypass clock gating
	__IOM uint32_t *clkreg_expmst0 = (uint32_t *) CFG_APB_EXPMST0;
	*clkreg_expmst0 = (1 << 0) | (1 << 4);
#endif

	/* uart EXPMST0 config settings */
	__IOM uint32_t *reg_uart_ctrl = (uint32_t *) (CFG_APB_EXPMST0 + UART_CTRL_OFFSET);
	uint8_t uart_instance_num = uart->instance_num;

	if(enable)
	{
		/* UART clock select. bits 8-15. (one bit for each instance.)
		 * 	    0: 38.4 MHz
		 * 	    1: APB bus clock, 100 MHz
		 */
		if (uart->cfg->clk_source == UART_CLK_SOURCE_0) //38.4 Mhz
		{
			*reg_uart_ctrl &= ( ~ ( (1 << uart_instance_num) << 8 ) );
		}
		else if (uart->cfg->clk_source == UART_CLK_SOURCE_1) //100 Mhz
		{
			*reg_uart_ctrl |= ( (1 << uart_instance_num) << 8 );
		}
		else
		{
			return ARM_DRIVER_ERROR_PARAMETER;
		}

		/* Enable selected UART module (instance). bits 0-7. (one bit for each instance.) */
		*reg_uart_ctrl |= (1 << uart_instance_num);
	}
	else /* Disable */
	{
		/* Disable selected UART module (instance). bits 0-7. (one bit for each instance.) */
		*reg_uart_ctrl &= ( ~ (1 << uart_instance_num) );
	}

	return ARM_DRIVER_OK;
}

/**
 * @fn		void uart_initialize (uart_resources_t *uart)
 * @brief	initialize uart
 * @note	this function will
 * 				-set tx/rx fifo length,
 * 				-reset uart,
 * 				-disable the interrupt,
 * 				-intialize all the variable to 0
 * @param	uart: Pointer to uart resources structure
 * @retval 	none
 */
void uart_initialize (uart_resources_t *uart)
{
	uart_info_t 	*uart_info_ptr = uart->info;
	uart_reg_set_t 	*uart_reg_ptr  = (uart_reg_set_t *)uart->reg_base;

	/* UART_CPR_FIFO_STAT and FIFO_MODE are always enable in IP and UART FIFO Depth is 32. */
	uart_info_ptr->tx_fifo_length = UART_FIFO_DEPTH;
	uart_info_ptr->rx_fifo_length = UART_FIFO_DEPTH;

	/* uart EXPMST0 configuration, Enable clock source and selected UART Module. */
	uart_configure_control_reg(true, uart);

	/* Configure the UART Peripheral Clock */
	uart_Configure_ClockSource (true, uart);

	/* reset the uart. */
	uart_software_reset(uart_reg_ptr);

	/* enable uart fifo fcr FIFO control register */
	uart_reg_ptr->iir_fcr = UART_FCR_FIFO_ENABLE;

	/* disable all uart interrupt ier interrupt enable register */
	uart_reg_ptr->ier_dlh = 0x0;

	/* disable TX/RX interrupt. */
	uart_disable_interrupt(uart);

	/* initialize the tx_buffer */
	uart_info_ptr->transfer.tx_buf 				= NULL;
	uart_info_ptr->transfer.tx_total_num 		= 0U;
	uart_info_ptr->transfer.tx_curr_cnt 		= 0U;

	/* clear Send active flag */
	uart_info_ptr->transfer.send_busy	 		= 0U;

	/* initialize the rx_buffer */
	uart_info_ptr->transfer.rx_buf 				= NULL;
	uart_info_ptr->transfer.rx_total_num 		= 0U;
	uart_info_ptr->transfer.rx_curr_cnt 		= 0U;

	/* clear Receive active flag */
	uart_info_ptr->rx_status.rx_busy 			= 0U;

    /* Clear RX status */
	uart_info_ptr->rx_status.rx_break          	= 0U;
	uart_info_ptr->rx_status.rx_framing_error	= 0U;
	uart_info_ptr->rx_status.rx_overflow      	= 0U;
	uart_info_ptr->rx_status.rx_parity_error    = 0U;
}

/**
 * @fn		void uart_uninitialize (uart_resources_t *uart)
 * @brief	uninitialize uart
 * @note	this function will
 * 				-disable the interrupt,
 * 				-abort TX/RX,
 * 				-reset tx/rx fifo,
 * 				-set baudrate to 0
 * 				-initialize all the variable to 0
 * @note	needs to initialize first if wants to use it again.
 * @param	uart: Pointer to uart resources structure
 * @retval 	none
 */
void uart_uninitialize (uart_resources_t *uart)
{
	uart_info_t *uart_info_ptr = uart->info;

	/* disable transmit and Receive interrupt. */
	uart_disable_interrupt(uart);

	/* abort tx and rx */
	uart_abort_tx(uart);
	uart_abort_rx(uart);

	/* set baud to 0 */
	uart_set_baudrate(uart, 0);

	/* initialize all variables to 0 */

	/* initialize the tx_buffer */
	uart_info_ptr->transfer.tx_buf 				= NULL;
	uart_info_ptr->transfer.tx_total_num 		= 0U;
	uart_info_ptr->transfer.tx_curr_cnt 		= 0U;

	/* clear Send active flag */
	uart_info_ptr->transfer.send_busy 			= 0U;

	/* initialize the rx_buffer */
	uart_info_ptr->transfer.rx_buf	 			= NULL;
	uart_info_ptr->transfer.rx_total_num	 	= 0U;
	uart_info_ptr->transfer.rx_curr_cnt 		= 0U;

	/* clear Receive active flag */
	uart_info_ptr->rx_status.rx_busy 			= 0U;

    /* Clear RX status */
	uart_info_ptr->rx_status.rx_break           = 0U;
	uart_info_ptr->rx_status.rx_framing_error   = 0U;
	uart_info_ptr->rx_status.rx_overflow        = 0U;
	uart_info_ptr->rx_status.rx_parity_error    = 0U;

	/* uart EXPMST0 configuration, Disable the selected UART Module. */
	uart_configure_control_reg(false, uart);
}


/* =============================================== RS485 Functions ====================================================== */
#if RS485_SUPPORT	/* RS485 Support Enable? */

/**
 * @fn		int32_t uart_set_rs485_transfer_mode (uart_resources_t *uart, uart_rs485_transfer_mode_t mode)
 * @brief	set RS485 transfer modes \ref uart_rs485_transfer_mode_t
 * @note	none
 * @param	uart: Pointer to uart resources structure
 * @param	mode: Available RS485 transfer modes
 * 				   - UART_RS485_FULL_DULPLX_MODE
 * 				   - UART_RS485_SW_CONTROL_HALF_DULPLX_MODE
 * 				   - UART_RS485_HW_CONTROL_HALF_DULPLX_MODE
 * @retval	SUCCESS		:  0
 * @retval	FAILURE		: -1
 */
int32_t uart_set_rs485_transfer_mode (uart_resources_t *uart, uart_rs485_transfer_mode_t mode)
{
	uart_reg_set_t 	*uart_reg_ptr  = (uart_reg_set_t *)uart->reg_base;

	if ((mode < UART_RS485_FULL_DULPLX_MODE) || (mode > UART_RS485_HW_CONTROL_HALF_DULPLX_MODE )) return -1;

	/* clear Transfer modes bits[4:3] */
	uart_reg_ptr->tcr &= (~UART_TCR_XFER_MODE_MASK);

	if(mode == UART_RS485_FULL_DULPLX_MODE) 			uart_reg_ptr->tcr |= UART_TCR_XFER_MODE_FULL_DUPLEX;
	if(mode == UART_RS485_SW_CONTROL_HALF_DULPLX_MODE) uart_reg_ptr->tcr |= UART_TCR_XFER_MODE_SW_CONTROL_HALF_DUPLEX;
	if(mode == UART_RS485_HW_CONTROL_HALF_DULPLX_MODE) uart_reg_ptr->tcr |= UART_TCR_XFER_MODE_HW_CONTROL_HALF_DUPLEX;

	return 0;
}

/**
 * @fn		uart_rs485_transfer_mode_t uart_get_rs485_transfer_mode (uart_resources_t *uart)
 * @brief   get selected RS485 transfer mode \ref uart_rs485_transfer_mode_t
 * @note	none
 * @param	uart: Pointer to uart resources structure
 * @retval 	selected RS485 transfer mode
 */
uart_rs485_transfer_mode_t uart_get_rs485_transfer_mode (uart_resources_t *uart)
{
	uart_reg_set_t 	*uart_reg_ptr  = (uart_reg_set_t *)uart->reg_base;

	return ((uart_reg_ptr->tcr & UART_TCR_XFER_MODE_MASK) >> 3);
}

/**
 * @fn		int32_t uart_set_rs485_de_assertion_time (uart_resources_t *uart, uint32_t assertion_time)
 * @brief	set RS485 DE Assertion time for
 * 			DET driver output enable timing register
 * @note	DE Assertion time: 8 bit only, DET register bits (7:0)
 * @param	uart			: Pointer to uart resources structure
 * @param   assertion_time	: 8-bit DE Assertion time
 * @retval	SUCCESS			:  0
 * @retval	FAILURE			: -1
 */
int32_t uart_set_rs485_de_assertion_time (uart_resources_t *uart, uint32_t assertion_time)
{
	uart_reg_set_t 	*uart_reg_ptr  = (uart_reg_set_t *)uart->reg_base;

	/* DE Assertion time: 8 bit only. */
	if(assertion_time > 0xFF) return -1; /* error maximum 8-bit only. */

	/* clear DE Assertion time: bits (7:0). */
	uart_reg_ptr->det &= (~UART_DET_TIME_MASK);

	/* DE Assertion time: bits (7:0). */
	uart_reg_ptr->det |= (assertion_time & UART_DET_TIME_MASK);

	return 0;
}

/**
 * @fn		int32_t uart_get_rs485_de_assertion_time (uart_resources_t *uart)
 * @brief	get RS485 DE Assertion time for
 * 			DET driver output enable timing register
 * @note	DE Assertion time: 8 bit only, DET register bits (7:0)
 * @param	uart			: Pointer to uart resources structure
 * @retval 	8-bit DET DE Assertion time
 */
int32_t uart_get_rs485_de_assertion_time (uart_resources_t *uart)
{
	uart_reg_set_t 	*uart_reg_ptr  = (uart_reg_set_t *)uart->reg_base;

	/* DE Assertion time: bits (7:0). */
	return (uart_reg_ptr->det & UART_DET_TIME_MASK);
}

/**
 * @fn		int32_t uart_set_rs485_de_deassertion_time (uart_resources_t *uart, uint32_t deassertion_time)
 * @brief	set RS485 DE De-Assertion time for
 * 			DET driver output enable timing register
 * @note	DE Assertion time: 8 bit only, DET register bits (23:16)
 * @param	uart				: Pointer to uart resources structure
 * @param   deassertion_time	: 8-bit DE De-Assertion time
 * @retval	SUCCESS				:  0
 * @retval	FAILURE				: -1
 */
int32_t uart_set_rs485_de_deassertion_time (uart_resources_t *uart, uint32_t deassertion_time)
{
	uart_reg_set_t 	*uart_reg_ptr  = (uart_reg_set_t *)uart->reg_base;

	/* DE De-Assertion time: 8 bit only. */
	if(deassertion_time > 0xFF) return -1; /* error maximum 8-bit only. */

	/* clear DE De-Assertion time: bits (23:16). */
	uart_reg_ptr->det &= ( ~ (UART_DET_TIME_MASK << UART_DET_DE_DEASSERTION_TIME_BIT_SHIFT) );

	/* DE De-Assertion time: bits (23:16). */
	uart_reg_ptr->det |= ( (deassertion_time & UART_DET_TIME_MASK) << UART_DET_DE_DEASSERTION_TIME_BIT_SHIFT );

	return 0;
}

/**
 * @fn		int32_t uart_get_rs485_de_deassertion_time (uart_resources_t *uart)
 * @brief	get RS485 DE De-Assertion time for
 * 			DET driver output enable timing register
 * @note	DE De-Assertion time: 8 bit , DET register bits (23:16)
 * @param	uart			: Pointer to uart resources structure
 * @retval 	8-bit DET DE De-Assertion time
 */
int32_t uart_get_rs485_de_deassertion_time (uart_resources_t *uart)
{
	uart_reg_set_t 	*uart_reg_ptr  = (uart_reg_set_t *)uart->reg_base;

	/* DE Assertion time: bits (23:16). */
	return ((uart_reg_ptr->det >> UART_DET_DE_DEASSERTION_TIME_BIT_SHIFT) & UART_DET_TIME_MASK);
}

/**
 * @fn		int32_t uart_set_rs485_de_to_re_turn_around_time (uart_resources_t *uart, uint32_t de_to_re_time)
 * @brief	set RS485 Driver Enable DE to Receive Enable RE Turn Around time for
 * 			TAT turn-around timing register
 * @note	TAT DE to RE Turn Around time: 16 bit , TAT register bits (15:0)
 * @param	uart				: Pointer to uart resources structure
 * @param   de_to_re_time		: 16-bit DE to RE time
 * @retval	SUCCESS				:  0
 * @retval	FAILURE				: -1
 */
int32_t uart_set_rs485_de_to_re_turn_around_time (uart_resources_t *uart, uint32_t de_to_re_time)
{
	uart_reg_set_t 	*uart_reg_ptr  = (uart_reg_set_t *)uart->reg_base;

	/* Driver Enable DE to Receive Enable RE Turn Around time: 16 bit . */
	if(de_to_re_time > 0xFFFF) return -1; /* error maximum 16-bit . */

	/* Clear TAT DE to RE Turn Around time bits (15:0). */
	uart_reg_ptr->tat &= (~UART_TAT_TIME_MASK);

	/* TAT DE to RE Turn Around time bits (15:0). */
	uart_reg_ptr->tat |= (de_to_re_time & UART_TAT_TIME_MASK);

	return 0;
}

/**
 * @fn 		int32_t uart_get_rs485_de_to_re_turn_around_time (uart_resources_t *uart)
 * @brief	get RS485 Driver Enable DE to Receive Enable RE Turn Around time for
 * 			TAT turn-around timing register
 * @note	TAT DE to RE Turn Around time: 16 bit , TAT register bits (15:0)
 * @param	uart			: Pointer to uart resources structure
 * @retval 	16-bit TAT register DE to RE time
 */
int32_t uart_get_rs485_de_to_re_turn_around_time (uart_resources_t *uart)
{
	uart_reg_set_t 	*uart_reg_ptr  = (uart_reg_set_t *)uart->reg_base;

	/* Driver Enable DE to Receive Enable RE Turn Around time: 16 bit  bits (15:0). */
	return (uart_reg_ptr->tat & UART_TAT_TIME_MASK);
}

/**
 * @fn 		int32_t uart_set_rs485_re_to_de_turn_around_time (uart_resources_t *uart, uint32_t re_to_de_time)
 * @brief	set RS485 Receive Enable RE to Driver Enable DE Turn Around time for
 * 			TAT turn-around timing register
 * @note	TAT RE to DE Turn Around time: 16 bit , TAT register bits (31:16)
 * @param	uart				: Pointer to uart resources structure
 * @param   re_to_de_time		: 16-bit RE to DE time
 * @retval	SUCCESS				:  0
 * @retval	FAILURE				: -1
 */
int32_t uart_set_rs485_re_to_de_turn_around_time (uart_resources_t *uart, uint32_t re_to_de_time)
{
	uart_reg_set_t 	*uart_reg_ptr  = (uart_reg_set_t *)uart->reg_base;

	/* Receive Enable RE to Driver Enable DE Turn Around time: 16 bit . */
	if(re_to_de_time > 0xFFFF) return -1; /* error maximum 16-bit . */

	/* Clear TAT RE to DE Turn Around time bits (31:16). */
	uart_reg_ptr->tat &= ( ~ (UART_TAT_TIME_MASK << UART_TAT_RE_TO_DE_TIME_BIT_SHIFT) );

	/* TAT RE to DE Turn Around time bits (31:16). */
	uart_reg_ptr->tat |= ( (re_to_de_time & UART_TAT_TIME_MASK) << UART_TAT_RE_TO_DE_TIME_BIT_SHIFT );

	return 0;
}

/**
 * @fn		int32_t uart_get_rs485_re_to_de_turn_around_time (uart_resources_t *uart)
 * @brief	get RS485 Receive Enable RE to Driver Enable DE Turn Around time for
 * 			TAT turn-around timing register
 * @note	TAT RE to DE Turn Around time: 16 bit , TAT register bits (31:16)
 * @param	uart			: Pointer to uart resources structure
 * @retval 	16-bit TAT register RE to DE time
 */
int32_t uart_get_rs485_re_to_de_turn_around_time (uart_resources_t *uart)
{
	uart_reg_set_t 	*uart_reg_ptr  = (uart_reg_set_t *)uart->reg_base;

	/* Receive Enable RE to Driver Enable DE Turn Around time: 16 bit  bits (31:16). */
	return (uart_reg_ptr->tat >> UART_TAT_RE_TO_DE_TIME_BIT_SHIFT);
}

/**
 * @fn		int32_t uart_set_rs485_de_en (uart_resources_t *uart, uint32_t arg)
 * @brief	set RS485 DE Driver Enable Signal State
 * @note	none
 * @param	uart		: Pointer to uart resources structure
 * @param	arg 		: 0=disable, 1=enable
 * @retval	SUCCESS		: 0
 * @retval	FAILURE		:-1
 */
int32_t uart_set_rs485_de_en (uart_resources_t *uart, uint32_t arg)
{
	uart_reg_set_t 	*uart_reg_ptr  = (uart_reg_set_t *)uart->reg_base;
	uint32_t mode;

	if( (arg != UART_DE_EN_DISABLE) && (arg != UART_DE_EN_ENABLE) ) return -1;

	if(arg == UART_DE_EN_DISABLE)
	{
		/* Disable the DE Driver Enable signal. */
		uart_reg_ptr->de_en = UART_DE_EN_DISABLE;
	}

	if(arg == UART_DE_EN_ENABLE)
	{
		/* Special check only for Software control Half-Duplex Mode as DE and RE are mutually exclusive here. */

		/* Check rs485 transfer mode. */
		mode = uart_get_rs485_transfer_mode(uart);

		if(mode == UART_RS485_SW_CONTROL_HALF_DULPLX_MODE)
		{
			/* In Software control Half-Duplex Mode DE and RE are mutually exclusive.
			 * so anyone either DE or RE can be enable at a time.
			 */

			/* in S/W Half-Duplex Mode first disable RE signal before enabling DE signal. */
			/* disable RE. */
			uart_reg_ptr->re_en = UART_RE_EN_DISABLE;
		}

		/* enable DE Driver Enable signal. */
		uart_reg_ptr->de_en = UART_DE_EN_ENABLE;
	}

	return 0;
}

/**
 * @fn		int32_t uart_get_rs485_de_en (uart_resources_t *uart)
 * @brief	get RS485 DE Drive Enable Signal State
 * @note	none
 * @param	uart	: Pointer to uart resources structure
 * @retval 	Disabled: 0
 * @retval 	Enabled : 1
 */
int32_t uart_get_rs485_de_en (uart_resources_t *uart)
{
	uart_reg_set_t 	*uart_reg_ptr  = (uart_reg_set_t *)uart->reg_base;

	/* get DE Driver Enable Signal State. */
	return (uart_reg_ptr->de_en);
}

/**
 * @fn 		int32_t uart_set_rs485_re_en (uart_resources_t *uart, uint32_t arg)
 * @brief	set RS485 RE Receiver Enable Signal State
 * @note	none
 * @param	uart		: Pointer to uart resources structure
 * @param	arg 		: 0=disable, 1=enable
 * @retval	SUCCESS		: 0
 * @retval	FAILURE		: -1
 */
int32_t uart_set_rs485_re_en (uart_resources_t *uart, uint32_t arg)
{
	uart_reg_set_t 	*uart_reg_ptr  = (uart_reg_set_t *)uart->reg_base;
	uint32_t mode;

	if( (arg != UART_RE_EN_DISABLE) && (arg != UART_RE_EN_ENABLE) ) return -1;

	if(arg == UART_RE_EN_DISABLE)
	{
		/* Disable the RE Receiver Enable signal. */
		uart_reg_ptr->re_en = UART_RE_EN_DISABLE;
	}

	if(arg == UART_RE_EN_ENABLE)
	{
		/* Special check only for Software control Half-Duplex Mode as DE and RE are mutually exclusive here. */

		/* Check rs485 transfer mode. */
		mode = uart_get_rs485_transfer_mode(uart);

		if(mode == UART_RS485_SW_CONTROL_HALF_DULPLX_MODE)
		{
			/* In Software control Half-Duplex Mode DE and RE are mutually exclusive.
			 * so anyone either DE or RE can be enable at a time.
			 */

			/* in S/W Half-Duplex Mode first disable DE signal before enabling RE signal. */
			/* disable DE. */
			uart_reg_ptr->de_en = UART_DE_EN_DISABLE;
		}

		/* enable RE Receiver Enable signal. */
		uart_reg_ptr->re_en = UART_RE_EN_ENABLE;
	}

	return 0;
}

/**
 * @fn		int32_t uart_get_rs485_re_en (uart_resources_t *uart)
 * @brief	get RS485 RE Receiver Enable Signal State
 * @note	none
 * @param	uart	: Pointer to uart resources structure
 * @retval 	Disabled: 0
 * @retval 	Enabled : 1
 */
int32_t uart_get_rs485_re_en (uart_resources_t *uart)
{
	uart_reg_set_t 	*uart_reg_ptr  = (uart_reg_set_t *)uart->reg_base;

	/* get RE Receiver Enable Signal State. */
	return (uart_reg_ptr->re_en);
}

/**
 * @fn		int32_t uart_rs485_configuration (uart_resources_t *uart)
 * @brief 	uart RS485 configuration from RTE_device.h
 * @note  	none
 * @param 	uart: Pointer to uart resources structure
 * @retval	SUCCESS		: 0
 * @retval	FAILURE		:-1
 */
static int32_t uart_rs485_configuration (uart_resources_t *uart)
{
	int ret = ARM_DRIVER_OK;

	/* uart RS485 configuration from RTE_device.h */

	/* uart set RS485 transfer mode */
	ret = uart_set_rs485_transfer_mode(uart, uart->rs485_cfg->rs485_transfer_mode);

	/* uart set RS485 Driver Enable DE Assertion Time (8-bit) */
	ret = uart_set_rs485_de_assertion_time(uart, uart->rs485_cfg->rs485_de_assertion_time_8bit);

	/* uart set RS485 Driver Enable DE De-Assertion Time (8-bit) */
	ret = uart_set_rs485_de_deassertion_time(uart, uart->rs485_cfg->rs485_de_deassertion_time_8bit);

	/* uart set RS485 Turn Around Time TAT for Driver Enable DE to Receive Enable RE */
	ret = uart_set_rs485_de_to_re_turn_around_time(uart, uart->rs485_cfg->rs485_de_to_re_turn_around_time_16bit);

	/* uart set RS485 Turn Around Time TAT for Receive Enable RE to Driver Enable DE */
	ret = uart_set_rs485_re_to_de_turn_around_time(uart, uart->rs485_cfg->rs485_re_to_de_turn_around_time_16bit);

	return ret;
}

/**
 * @fn		int32_t uart_enable_rs485 (uart_resources_t *uart)
 * @brief	enable uart RS485 mode with default settings
 * @note	default settings:
 * 				- TCR register transfer mode: Hardware Control Half-Duplex mode,
 * 				- DET register De Assertion/De-Assertion time: 0,
 * 				- TAT register DE to RE / RE to DE time: 0
 * @param	uart: Pointer to uart resources structure
 * @retval 	none
 */
int32_t uart_enable_rs485 (uart_resources_t *uart)
{
	int ret = ARM_DRIVER_OK;
	uart_reg_set_t 	*uart_reg_ptr  = (uart_reg_set_t *)uart->reg_base;

	/* reset TCR transceiver control register. */
	uart_reg_ptr->tcr = 0;

	/* enable RS485 mode in TCR transceiver control register. */
	uart_reg_ptr->tcr |= UART_TCR_RS485_ENABLE;

	/* de polarity: active high, re polarity: active low/high. */
	uart_reg_ptr->tcr |= UART_TCR_DE_POL_ACTIVE_HIGH;
	uart_reg_ptr->tcr |= UART_TCR_RE_POL_ACTIVE_HIGH;

	ret = uart_rs485_configuration (uart);
	return ret;
}

/**
 * @fn		void uart_disable_rs485 (uart_resources_t *uart)
 * @brief	disable uart RS485 mode
 * @note	none
 * @param	uart: Pointer to uart resources structure
 * @retval 	none
 */
void uart_disable_rs485 (uart_resources_t *uart)
{
	uart_reg_set_t 	*uart_reg_ptr  = (uart_reg_set_t *)uart->reg_base;

	/* disable RS485 mode in TCR transceiver control register. */
	uart_reg_ptr->tcr &= ~UART_TCR_RS485_ENABLE;
}

#endif /* RS485 functions. */


/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
