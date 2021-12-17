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
 * @file     Driver_USART.c
 * @author   Tanay Rami
 * @email    tanay@alifsemi.com
 * @version  V1.0.0
 * @date     20-May-2020
 * @brief    ARM CMSIS-Driver for UART RS232 and RS485.
 * @bug      None.
 * @Note	 None.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "uart_ll_drv.h"

#define ARM_USART_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)  /* driver version */

/* UART Driver Instances 0-7 */
#define UART0_INSTANCE_NUM		0      /* UART0 instance */
#define UART1_INSTANCE_NUM		1	   /* UART1 instance */
#define UART2_INSTANCE_NUM		2      /* UART2 instance */
#define UART3_INSTANCE_NUM		3      /* UART3 instance */
#define UART4_INSTANCE_NUM		4      /* UART4 instance */
#define UART5_INSTANCE_NUM		5      /* UART5 instance */
#define UART6_INSTANCE_NUM		6      /* UART6 instance */
#define UART7_INSTANCE_NUM		7      /* UART7 instance */

/* UART Interrupt Number */
#define	UART0_IRQ_NUM			126  	/* UART0 Interrupt number */
#define UART1_IRQ_NUM  			127  	/* UART1 Interrupt number */
#define UART2_IRQ_NUM			128  	/* UART2 Interrupt number */
#define UART3_IRQ_NUM			129  	/* UART3 Interrupt number */
#define UART4_IRQ_NUM			130  	/* UART4 Interrupt number */
#define UART5_IRQ_NUM			131  	/* UART5 Interrupt number */
#define UART6_IRQ_NUM			132  	/* UART6 Interrupt number */
#define UART7_IRQ_NUM			133  	/* UART7 Interrupt number */

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
    ARM_USART_API_VERSION,
    ARM_USART_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_USART_CAPABILITIES DriverCapabilities = {
    1, /* supports UART (Asynchronous) mode */
    0, /* supports Synchronous Master mode */
    0, /* supports Synchronous Slave mode */
    0, /* supports UART Single-wire mode */
    0, /* supports UART IrDA mode */
    0, /* supports UART Smart Card mode */
    0, /* Smart Card Clock generator available */
    1, /* RTS Flow Control available */
    1, /* CTS Flow Control available */
    1, /* Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE */
    1, /* Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT */
    0, /* RTS Line: 0=not available, 1=available */
    0, /* CTS Line: 0=not available, 1=available */
    0, /* DTR Line: 0=not available, 1=available */
    0, /* DSR Line: 0=not available, 1=available */
    0, /* DCD Line: 0=not available, 1=available */
    0, /* RI Line: 0=not available, 1=available */
    0, /* Signal CTS change event: \ref ARM_USART_EVENT_CTS */
    0, /* Signal DSR change event: \ref ARM_USART_EVENT_DSR */
    0, /* Signal DCD change event: \ref ARM_USART_EVENT_DCD */
    0, /* Signal RI change event: \ref ARM_USART_EVENT_RI */
    0  /* Reserved (must be zero) */
};

//
//   Functions
//
/**
 * @fn		ARM_DRIVER_VERSION ARM_USART_GetVersion(void)
 * @brief	get uart version
 * @note	none
 * @param	none
 * @retval 	driver version
 */
static ARM_DRIVER_VERSION ARM_USART_GetVersion(void)
{
  return DriverVersion;
}

/**
 * @fn		ARM_USART_CAPABILITIES ARM_USART_GetCapabilities(void)
 * @brief 	get uart capabilites
 * @note	none
 * @param	none
 * @retval 	driver capabilites
 */
static ARM_USART_CAPABILITIES ARM_USART_GetCapabilities(void)
{
  return DriverCapabilities;
}

/**
 * @fn		int32_t ARM_USART_PowerControl (ARM_POWER_STATE   state,
                                   	   	    uart_resources_t *uart)
 * @brief 	CMSIS-Driver uart power control
 * @note	none
 * @param	state	: Power state
 * @param	uart	: Pointer to uart resources structure
 * @retval	ARM_DRIVER_ERROR_UNSUPPORTED
 */
static int32_t ARM_USART_PowerControl (ARM_POWER_STATE   state,
                                   	   uart_resources_t *uart)
{
    switch (state)
    {
    case ARM_POWER_OFF:

		/* Disable uart IRQ */
		NVIC_DisableIRQ (uart->irq_num);

		/* Clear Any Pending IRQ*/
		NVIC_ClearPendingIRQ (uart->irq_num);

	    /* Reset the power status of uart. */
	    uart->info->flags &= ~UART_FLAG_POWERED;
        break;

    case ARM_POWER_FULL:

    	if ( (uart->info->flags & UART_FLAG_INITIALIZED) == 0U)
    	{
    	    /* error: Driver is not initialized /ref ARM_USART_Initialize */
    	    return ARM_DRIVER_ERROR;
    	}

    	if ( (uart->info->flags & UART_FLAG_POWERED) )
    	{
    	    return ARM_DRIVER_OK;
    	}

		/* Enable uart IRQ*/
		NVIC_ClearPendingIRQ (uart->irq_num);
		NVIC_SetPriority(uart->irq_num, uart->irq_priority);
		NVIC_EnableIRQ (uart->irq_num);

		/* Set the power flag enabled */
		uart->info->flags |= UART_FLAG_POWERED;
        break;

	case ARM_POWER_LOW:
	default:
		return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

/**
 * @fn		int32_t ARM_USART_Initialize (ARM_USART_SignalEvent_t   cb_event,
									 	  uart_resources_t         *uart)
 * @brief	CMSIS-Driver uart initialize
 * @note	if cb_event is NULL it will use polling method
 * 			else it will use interrupt method for data send and receive.
 * @param	cb_event	: Pointer to USART Event \ref ARM_USART_SignalEvent
 * @param	uart		: Pointer to uart resources structure
 * @retval	ARM_DRIVER_OK
 */
static int32_t ARM_USART_Initialize (ARM_USART_SignalEvent_t   cb_event,
									 uart_resources_t         *uart)
{
	if (uart->info->flags & UART_FLAG_INITIALIZED)
	{
	    /* Driver is already initialized */
	    return ARM_DRIVER_OK;
	}

	/* if callback event is NULL it will use polling method
	 * else it will use interrupt method for data send and receive.
	 */

	/* set the user callback event. */
    uart->info->cb_event = cb_event;

    /* calling uart initialize lower level api */
    uart_initialize(uart);

    /* set the flag as initialized. */
    uart->info->flags = UART_FLAG_INITIALIZED;
    return ARM_DRIVER_OK;
}

/**
 * @fn		int32_t ARM_USART_Uninitialize (uart_resources_t *uart)
 * @brief	CMSIS-Driver uart uninitialize
 * @note	none
 * @param	uart		: Pointer to uart resources structure
 * @retval	ARM_DRIVER_OK
 */
static int32_t ARM_USART_Uninitialize (uart_resources_t *uart)
{
	if ( (uart->info->flags & UART_FLAG_INITIALIZED) == 0)
	{
	    /* Driver is not initialized */
	    return ARM_DRIVER_OK;
	}

	/* Disable the power. */
	ARM_USART_PowerControl (ARM_POWER_OFF, uart);

	/* calling uart uninitialize lower level api */
    uart_uninitialize(uart);

	/* Reset UART flags. */
	uart->info->flags = 0U;
	return ARM_DRIVER_OK;
}

/**
 * @fn		int32_t ARM_USART_Send (const void       *data,
                               	    uint32_t          num,
                               	    uart_resources_t *uart)
 * @brief	CMSIS-Driver uart send
 * 			Start sending data to UART transmitter.
 * @note	tx flag UART_FLAG_TX_ENABLED should be enabled first /ref ARM_USART_CONTROL_TX
 * @param	data	: Pointer to buffer with data to send to USART transmitter
 * @param	num		: Number of data items to send
 * @param	uart	: Pointer to uart resources structure
 * @retval	ARM_DRIVER_ERROR_PARAMETER	: error in parameter
 * @retval	ARM_DRIVER_ERROR			: error in driver
 * @retval	ARM_DRIVER_OK				: success in interrupt case
 * @retval  ARM_DRIVER_ERROR_BUSY		: driver busy in interrupt case
 * @retval	send count					: in polling case only
 */
static int32_t ARM_USART_Send (const void       *data,
                               uint32_t          num,
                               uart_resources_t *uart)
{
  int ret = ARM_DRIVER_OK;

  if ((data == NULL) || (num == 0U)) {
	/* Invalid parameters */
	return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((uart->info->flags & UART_FLAG_TX_ENABLED) == 0U) {
	/* error: UART is not configured (mode not selected)
	 * tx flag UART_FLAG_TX_ENABLED should be enabled first /ref ARM_USART_CONTROL_TX
	 */
	return ARM_DRIVER_ERROR;
  }

  /* if callback assigned use interrupt method else use polling method */
  if( uart->info->cb_event != NULL )
  {
	  /* callback is assigned, use interrupt method */

	  /* fill the user input details for uart send transfer structure
	   * and enable the send interrupt.
	   */
	  ret = uart_enable_send_interrupt(uart, data, num);  /* non-blocked */
  }
  else
  {
	  /* callback is not assigned, use polling method */

	  /* send data to uart using polling/blocking method. */
	  ret = uart_send_max_data_polling (uart, data, num); /* blocked */
  }

  return ret;
}

/**
 * @fn		int32_t ARM_USART_Receive (void             *data,
								  	   uint32_t          num,
                                  	   uart_resources_t *uart)
 * @brief	CMSIS-Driver uart receive
 * 			Start receiving data from USART receiver.
 * @note	none
 * @param	data						: Pointer to buffer for data to receive from UART receiver
 * @param	num							: Number of data items to receive
 * @param	uart						: Pointer to uart resources structure
 * @retval	ARM_DRIVER_ERROR_PARAMETER	: error in parameter
 * @retval	ARM_DRIVER_ERROR			: error in driver
 * @retval	ARM_DRIVER_OK				: success in interrupt case
 * @retval  ARM_DRIVER_ERROR_BUSY		: driver busy in interrupt case
 * @retval	received count				: in polling case only
 */
static int32_t ARM_USART_Receive (void             *data,
								  uint32_t          num,
                                  uart_resources_t *uart)
{
	  int ret = ARM_DRIVER_OK;

	  if ((data == NULL) || (num == 0U)) {
		/* Invalid parameters */
		return ARM_DRIVER_ERROR_PARAMETER;
	  }

	  if ((uart->info->flags & UART_FLAG_RX_ENABLED) == 0U) {
		/* error: UART is not configured (mode not selected) */
		return ARM_DRIVER_ERROR;
	  }

	  /* if callback assigned use interrupt else use polling method */
	  if( uart->info->cb_event != NULL )
	  {
		  /* callback is assigned, use interrupt method */
		  ret = uart_enable_receive_interrupt(uart, data, num);  /* non-blocked */
	  }
	  else
	  {
		  /* callback is not assigned, use polling method */
		  ret = uart_receive_max_data_polling (uart, data, num); /* blocked */
	  }
	  return ret;
}

/**
 * @fn		int32_t ARM_USART_Transfer (const void       *data_out,
                                   	    void             *data_in,
                                   	    uint32_t          num,
                                   	    uart_resources_t *uart)
 * @brief	CMSIS-Driver uart transfer
 * 			Start sending/receiving data to/from UART transmitter/receiver.
 * @note	use in synchronous mode, currently our driver is not supporting it.
 * @param	data_out	: Pointer to buffer with data to send to USART transmitter
 * @param	data_in		: Pointer to buffer for data to receive from USART receiver
 * @param	num			: Number of data items to transfer
 * @param	uart		: Pointer to uart resources structure
 * @retval	ARM_DRIVER_ERROR_UNSUPPORTED
 */
static int32_t ARM_USART_Transfer (const void       *data_out,
                                   void             *data_in,
                                   uint32_t          num,
                                   uart_resources_t *uart)
{
	/* Use with Synchronous mode only */
	/* Not supported as our driver is only Asynchronous. */
	return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
 * @fn		uint32_t ARM_USART_GetTxCount (uart_resources_t *uart)
 * @brief	CMSIS-Driver uart get transmitted data count
 * @note	only in interrupt mode
 * @param	uart	: Pointer to uart resources structure
 * @retval	transmitted data count
 */
static uint32_t ARM_USART_GetTxCount (uart_resources_t *uart)
{
	uint32_t cnt;
	cnt = uart->info->transfer.tx_curr_cnt;
	return cnt;
}

/**
 * @fn		uint32_t ARM_USART_GetRxCount (uart_resources_t *uart)
 * @brief	CMSIS-Driver uart get received data count
 * @note	only in interrupt mode
 * @param	uart	: Pointer to uart resources structure
 * @retval	received data count
 */
static uint32_t ARM_USART_GetRxCount (uart_resources_t *uart)
{
	uint32_t cnt;
	cnt = uart->info->transfer.rx_curr_cnt;
	return cnt;
}

/**
 * @fn		int32_t ARM_USART_Control (uint32_t          control,
                              	  	   uint32_t          arg,
								  	   uart_resources_t *uart)
 * @brief	CMSIS-Driver uart control
 * 			Control USART Interface.
 * @note	none
 * @param	control		: Operation
 * @param	arg			: Argument of operation (optional)
 * @param	uart		: Pointer to uart resources structure
 * @retval	common \ref execution_status and driver specific \ref usart_execution_status
 */
static int32_t ARM_USART_Control (uint32_t          control,
                              	  uint32_t          arg,
								  uart_resources_t *uart)
{
	int ret = ARM_DRIVER_OK;

    switch (control & ARM_USART_CONTROL_Msk)
    {
        case ARM_USART_MODE_ASYNCHRONOUS:
        	/* uart asynchronous mode */
        	if ( (uart->info->flags & UART_FLAG_INITIALIZED) == 0U)
        	{
        	    /* error: Driver is not initialized /ref ARM_USART_Initialize */
        	    return ARM_DRIVER_ERROR;
        	}

        	/* set uart asynchronous mode parameters as per arg
        	 * set baudrate, data length, parity, stop bits,
        	 * and flow control.
        	 */
        	uart_set_asynchronous_mode(uart, control, arg);
            break;

        case ARM_USART_CONTROL_TX:
        	/* uart enable/disable transmitter */
        	if ( (uart->info->flags & UART_FLAG_INITIALIZED) == 0U)
        	{
        	    /* error: Driver is not initialized /ref ARM_USART_Initialize */
        	    return ARM_DRIVER_ERROR;
        	}

			if (arg)
			{
				/* uart set transmitter trigger level as per RTE configuration */
				uart_set_tx_trigger(uart, uart->cfg->tx_fifo_trg_lvl);

				/* setting TX flag to enabled. */
				 uart->info->flags |= UART_FLAG_TX_ENABLED;
			}
			else
			{
				/* clear TX flag to enabled. */
				uart->info->flags &= ~UART_FLAG_TX_ENABLED;
			}
            break;

        case ARM_USART_CONTROL_RX:
        	/* uart enable/disable receiver */
        	if ( (uart->info->flags & UART_FLAG_INITIALIZED) == 0U)
        	{
        	    /* error: Driver is not initialized /ref ARM_USART_Initialize */
        	    return ARM_DRIVER_ERROR;
        	}

			if (arg)
			{
				/* uart set receiver trigger level as per RTE configuration */
				uart_set_rx_trigger(uart, uart->cfg->rx_fifo_trg_lvl);

				/* setting RX flag to enabled. */
				uart->info->flags |= UART_FLAG_RX_ENABLED;
			}
			else
			{
				/* clear RX flag to enabled. */
				uart->info->flags &= ~UART_FLAG_RX_ENABLED;
			}
            break;

        case ARM_USART_ABORT_SEND:
        	/* uart abort transmitter */
			if ((uart->info->flags & UART_FLAG_TX_ENABLED) == 0U)
			{
				/* error: UART transmitter is not enabled
				 * ref ARM_USART_CONTROL_TX
				 */
				return ARM_DRIVER_ERROR;
			}

			uart_abort_tx(uart);
        	break;

        case ARM_USART_ABORT_RECEIVE:
        	/* uart abort receiver */
        	if ((uart->info->flags & UART_FLAG_RX_ENABLED) == 0U)
        	{
				/* error: UART receiver is not enabled
				 * ref ARM_USART_CONTROL_RX
				 */
        		return ARM_DRIVER_ERROR;
        	}

        	uart_abort_rx(uart);
        	break;

        case ARM_USART_SET_RX_TRIGGER_LVL:
        	/* uart set receiver trigger level */
        	if ( (uart->info->flags & UART_FLAG_INITIALIZED) == 0U)
        	{
        	    /* error: Driver is not initialized /ref ARM_USART_Initialize */
        	    return ARM_DRIVER_ERROR;
        	}

       	    uart_set_rx_trigger(uart, arg);
            break;

        case ARM_USART_GET_RX_TRIGGER_LVL:
        	/* uart get receiver trigger level */
        	if ( (uart->info->flags & UART_FLAG_INITIALIZED) == 0U)
        	{
        	    /* error: Driver is not initialized /ref ARM_USART_Initialize */
        	    return ARM_DRIVER_ERROR;
        	}

        	ret = uart_get_rx_trigger(uart);
            break;

        case ARM_USART_SET_TX_TRIGGER_LVL:
        	/* uart set transmitter trigger level */
        	if ( (uart->info->flags & UART_FLAG_INITIALIZED) == 0U)
        	{
        	    /* error: Driver is not initialized /ref ARM_USART_Initialize */
        	    return ARM_DRIVER_ERROR;
        	}

       	    uart_set_tx_trigger(uart, arg);
            break;

        case ARM_USART_GET_TX_TRIGGER_LVL:
        	/* uart get transmitter trigger level */
        	if ( (uart->info->flags & UART_FLAG_INITIALIZED) == 0U)
        	{
        	    /* error: Driver is not initialized /ref ARM_USART_Initialize */
        	    return ARM_DRIVER_ERROR;
        	}

        	ret = uart_get_tx_trigger(uart);
            break;

        case ARM_USART_GET_TX_FIFO_AVAIL_CNT:
        	/* uart get transmit fifo available count */
			if ((uart->info->flags & UART_FLAG_TX_ENABLED) == 0U)
			{
				/* error: UART transmitter is not enabled
				 * ref ARM_USART_CONTROL_TX
				 */
				return ARM_DRIVER_ERROR;
			}

        	ret = uart_get_tx_fifo_available_count(uart);
       	    break;

        case ARM_USART_GET_RX_FIFO_AVAIL_CNT:
        	/* uart get receiver fifo available count */
        	if ((uart->info->flags & UART_FLAG_RX_ENABLED) == 0U)
        	{
				/* error: UART receiver is not enabled
				 * ref ARM_USART_CONTROL_RX
				 */
        		return ARM_DRIVER_ERROR;
        	}

        	ret = uart_get_rx_fifo_available_count(uart);
       	    break;

        case ARM_USART_SET_INT_PRIORITY:
        	/* uart Set interrupt priority , arg = interrupt priority */
        	if ( (uart->info->flags & UART_FLAG_INITIALIZED) == 0U)
        	{
        	    /* error: Driver is not initialized /ref ARM_USART_Initialize */
        	    return ARM_DRIVER_ERROR;
        	}

        	uart->irq_priority = arg;
            break;

        case ARM_USART_GET_INT_PRIORITY:
        	/* uart Get interrupt priority , arg = dummy */
        	if ( (uart->info->flags & UART_FLAG_INITIALIZED) == 0U)
        	{
        	    /* error: Driver is not initialized /ref ARM_USART_Initialize */
        	    return ARM_DRIVER_ERROR;
        	}

        	ret = uart->irq_priority;
            break;

        case ARM_USART_CONTROL_BREAK:
        	/* set/clear break */
        	if ( (uart->info->flags & UART_FLAG_INITIALIZED) == 0U)
        	{
        	    /* error: Driver is not initialized /ref ARM_USART_Initialize */
        	    return ARM_DRIVER_ERROR;
        	}

        	if (arg)
        	{
        		uart_set_break_control(uart);
        	}
        	else
        	{
        		uart_clear_break_control(uart);
        	}
            break;

#if RS485_SUPPORT    /* RS485 control codes. */

            /* enable/disable RS485 support. */
        case ARM_USART_CONTROL_RS485:
        	/* uart enable RS485 support , arg = enable/disable */
        	if ( (uart->info->flags & UART_FLAG_INITIALIZED) == 0U)
        	{
        	    /* error: Driver is not initialized /ref ARM_USART_Initialize */
        	    return ARM_DRIVER_ERROR;
        	}

        	if (arg)
        	{
        		ret = uart_enable_rs485(uart);

				/* setting RS485 flag to enabled. */
				uart->info->flags |= UART_FLAG_RS485_ENABLE;
        	}
        	else
        	{
        		uart_disable_rs485(uart);

				/* clear RS485 enable flag. */
				uart->info->flags &= ~UART_FLAG_RS485_ENABLE;
        	}
            break;

            /* get RS485 state: enable/disable */
        case ARM_USART_GET_RS485_STATE:
        	/* uart get RS485 state: return 0: disable, 1: enable */
        	if ( (uart->info->flags & UART_FLAG_RS485_ENABLE) == 0U)
        	{
        	    /* error: RS485 is not enabled /ref ARM_USART_CONTROL_RS485 */
        		ret = 0;
        	}
        	else
        	{
        		ret = 1; /* RS485 is enabled. */
        	}
            break;

            /* set RS485 transfer modes */
        case ARM_USART_SET_RS485_TRANSFER_MODE:
        	/* uart set RS485 transfer mode */
        	if ( (uart->info->flags & UART_FLAG_RS485_ENABLE) == 0U)
        	{
        	    /* error: RS485 is not enabled /ref ARM_USART_CONTROL_RS485 */
        	    return ARM_DRIVER_ERROR;
        	}

        	ret = uart_set_rs485_transfer_mode(uart, arg);
            break;

            /* get RS485 transfer modes */
        case ARM_USART_GET_RS485_TRANSFER_MODE:
        	/* uart set RS485 transfer mode */
        	if ( (uart->info->flags & UART_FLAG_RS485_ENABLE) == 0U)
        	{
        	    /* error: RS485 is not enabled /ref ARM_USART_CONTROL_RS485 */
        	    return ARM_DRIVER_ERROR;
        	}

       	    ret = uart_get_rs485_transfer_mode(uart);
            break;

            /* set RS485 DE Assertion time */
        case ARM_USART_SET_RS485_DE_ASSERTION_TIME:
        	/* uart set RS485 Driver Enable DE Assertion Time (8-bit only) */
        	if ( (uart->info->flags & UART_FLAG_RS485_ENABLE) == 0U)
        	{
        	    /* error: RS485 is not enabled /ref ARM_USART_CONTROL_RS485 */
        	    return ARM_DRIVER_ERROR;
        	}

        	ret = uart_set_rs485_de_assertion_time(uart, arg);
            break;

            /* get RS485 DE Assertion time */
        case ARM_USART_GET_RS485_DE_ASSERTION_TIME:
        	/* uart get RS485 Driver Enable DE Assertion Time (8-bit) */
        	if ( (uart->info->flags & UART_FLAG_RS485_ENABLE) == 0U)
        	{
        	    /* error: RS485 is not enabled /ref ARM_USART_CONTROL_RS485 */
        	    return ARM_DRIVER_ERROR;
        	}

       	    ret = uart_get_rs485_de_assertion_time(uart);
            break;

            /* set RS485 DE De-Assertion time */
        case ARM_USART_SET_RS485_DE_DEASSERTION_TIME:
        	/* uart set RS485 Driver Enable DE De-Assertion Time (8-bit) */
        	if ( (uart->info->flags & UART_FLAG_RS485_ENABLE) == 0U)
        	{
        	    /* error: RS485 is not enabled /ref ARM_USART_CONTROL_RS485 */
        	    return ARM_DRIVER_ERROR;
        	}

        	ret = uart_set_rs485_de_deassertion_time(uart, arg);
            break;

            /* get RS485 DE De-Assertion time */
        case ARM_USART_GET_RS485_DE_DEASSERTION_TIME:
        	/* uart get RS485 Driver Enable DE De-Assertion Time (8-bit) */
        	if ( (uart->info->flags & UART_FLAG_RS485_ENABLE) == 0U)
        	{
        	    /* error: RS485 is not enabled /ref ARM_USART_CONTROL_RS485 */
        	    return ARM_DRIVER_ERROR;
        	}

        	ret = uart_get_rs485_de_deassertion_time(uart);
            break;

            /* set RS485 DE to RE turn around time (16-bit) */
        case ARM_USART_SET_RS485_DE_TO_RE_TURN_AROUND_TIME:
        	/* uart set RS485 Turn Around Time TAT for
        	 * Driver Enable DE to Receive Enable RE */
        	if ( (uart->info->flags & UART_FLAG_RS485_ENABLE) == 0U)
        	{
        	    /* error: RS485 is not enabled /ref ARM_USART_CONTROL_RS485 */
        	    return ARM_DRIVER_ERROR;
        	}

        	ret = uart_set_rs485_de_to_re_turn_around_time(uart, arg);
            break;

            /* get RS485 DE to RE turn around time (16-bit) */
        case ARM_USART_GET_RS485_DE_TO_RE_TURN_AROUND_TIME:
        	/* uart get RS485 Turn Around Time TAT for
        	 * Driver Enable DE to Receive Enable RE */
        	if ( (uart->info->flags & UART_FLAG_RS485_ENABLE) == 0U)
        	{
        	    /* error: RS485 is not enabled /ref ARM_USART_CONTROL_RS485 */
        	    return ARM_DRIVER_ERROR;
        	}

        	ret = uart_get_rs485_de_to_re_turn_around_time(uart);
            break;

            /* set RS485 RE to DE turn around time (16-bit) */
        case ARM_USART_SET_RS485_RE_TO_DE_TURN_AROUND_TIME:
        	/* uart set RS485 Turn Around Time TAT for
        	 * Receive Enable RE to Driver Enable DE */
        	if ( (uart->info->flags & UART_FLAG_RS485_ENABLE) == 0U)
        	{
        	    /* error: RS485 is not enabled /ref ARM_USART_CONTROL_RS485 */
        	    return ARM_DRIVER_ERROR;
        	}

        	ret = uart_set_rs485_re_to_de_turn_around_time(uart, arg);
            break;

            /* get RS485 RE to DE turn around time (16-bit) */
        case ARM_USART_GET_RS485_RE_TO_DE_TURN_AROUND_TIME:
        	/* uart get RS485 Turn Around Time TAT for
        	 * Receive Enable RE to Driver Enable DE */
        	if ( (uart->info->flags & UART_FLAG_RS485_ENABLE) == 0U)
        	{
        	    /* error: RS485 is not enabled /ref ARM_USART_CONTROL_RS485 */
        	    return ARM_DRIVER_ERROR;
        	}

        	ret = uart_get_rs485_re_to_de_turn_around_time(uart);
            break;

            /* Set RS485 Driver Enable DE_EN signal, arg: 0=disable, 1=enable */
        case ARM_USART_SET_RS485_DE_EN:

        	if ( (uart->info->flags & UART_FLAG_RS485_ENABLE) == 0U)
        	{
        	    /* error: RS485 is not enabled /ref ARM_USART_CONTROL_RS485 */
        	    return ARM_DRIVER_ERROR;
        	}

        	/* uart set Driver Enable DE_EN signal. */
        	ret = uart_set_rs485_de_en(uart, arg);
            break;

            /* Get RS485 Driver Enable DE_EN signal state, 0=disable, 1=enable */
        case ARM_USART_GET_RS485_DE_EN:

        	if ( (uart->info->flags & UART_FLAG_RS485_ENABLE) == 0U)
        	{
        	    /* error: RS485 is not enabled /ref ARM_USART_CONTROL_RS485 */
        	    return ARM_DRIVER_ERROR;
        	}

        	/* uart Get Driver Enable DE_EN signal state. 0=disable, 1=enable */
        	ret = uart_get_rs485_de_en(uart);
            break;

            /* Set RS485 Receiver Enable RE_EN signal, arg: 0=disable, 1=enable */
        case ARM_USART_SET_RS485_RE_EN:

        	if ( (uart->info->flags & UART_FLAG_RS485_ENABLE) == 0U)
        	{
        	    /* error: RS485 is not enabled /ref ARM_USART_CONTROL_RS485 */
        	    return ARM_DRIVER_ERROR;
        	}

        	/* uart set Receiver Enable RE_EN signal. */
        	ret = uart_set_rs485_re_en(uart, arg);
            break;

            /* Get RS485 Receiver Enable RE_EN signal state, 0=disable, 1=enable */
        case ARM_USART_GET_RS485_RE_EN:

        	if ( (uart->info->flags & UART_FLAG_RS485_ENABLE) == 0U)
        	{
        	    /* error: RS485 is not enabled /ref ARM_USART_CONTROL_RS485 */
        	    return ARM_DRIVER_ERROR;
        	}

        	/* uart Get Receiver Enable DE_EN signal state. */
        	ret = uart_get_rs485_re_en(uart);
            break;

#endif  /* RS485 control codes. */

            /* Unsupported command */
        default:
        	ret =  ARM_DRIVER_ERROR_UNSUPPORTED;
            break;
    }
    return ret;
}

/**
 * @fn		ARM_USART_STATUS ARM_USART_GetStatus (uart_resources_t *uart)
 * @brief	CMSIS-Driver uart get status
 * @note	not implemented yet.
 * @param	uart	: Pointer to uart resources structure
 * @retval	ARM_USART_STATUS
 */
static ARM_USART_STATUS ARM_USART_GetStatus (uart_resources_t *uart)
{
	/* not implemented yet. */
	ARM_USART_STATUS status = {0, 0, 0, 0, 0, 0, 0, 0};
	return status;
}

/**
 * @fn		int32_t ARM_USART_SetModemControl (ARM_USART_MODEM_CONTROL   control,
										  	   uart_resources_t         *usart)
 * @brief	CMSIS-Driver Set UART Modem Control line state.
 * @note	not implemented yet.
 * @param	control : \ref ARM_USART_MODEM_CONTROL
 * @param	uart	: Pointer to uart resources structure
 * @retval	\ref execution_status
 */
static int32_t ARM_USART_SetModemControl (ARM_USART_MODEM_CONTROL   control,
										  uart_resources_t         *usart)
{
	/* not implemented yet. */
	return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
 * @fn		ARM_USART_MODEM_STATUS ARM_USART_GetModemStatus (uart_resources_t *uart)
 * @brief	CMSIS-Driver uart Get UART Modem Status lines state.
 * @note	not implemented yet.
 * @param	uart	: Pointer to uart resources structure
 * @retval	modem status \ref ARM_USART_MODEM_STATUS
 */
static ARM_USART_MODEM_STATUS ARM_USART_GetModemStatus (uart_resources_t *uart)
{
	/* not implemented yet. */
	ARM_USART_MODEM_STATUS status = {0, 0, 0, 0, 0};
	return status;
}

/**
 * @fn		void ARM_USART_IRQHandler (uart_resources_t *uart)
 * @brief	CMSIS-Driver uart interrupt handler
 * @note	none
 * @param	uart	: Pointer to uart resources structure
 * @retval 	none
 */
static void ARM_USART_IRQHandler (uart_resources_t *uart)
{
	uart_irq_handler (uart);
}

// End UART Interface


/* UART0 Driver Instance */
#if (RTE_UART0)

static uart_config_info_t uart0_config =
{
	.clk_source      = RTE_UART0_CLK_SOURCE,
	.rx_fifo_trg_lvl = RTE_UART0_RX_TRIG_LVL,
	.tx_fifo_trg_lvl = RTE_UART0_TX_TRIG_LVL,
};
static uart_info_t uart0_info = {0};

#if RS485_SUPPORT
static uart_config_rs485_info_t uart0_rs485_config = {0};
#endif

/* UART0 Driver Resources */
static uart_resources_t UART0_Resources =
{
	.reg_base 		= (uint32_t)RTE_UART0_PHY_ADDR,
	.clk 			= (uint32_t)0,
	.cfg			= &uart0_config,
	.info 			= &uart0_info,
	.irq_num		= UART0_IRQ_NUM,
	.irq_priority	= (uint32_t)RTE_UART0_IRQ_PRI,
	.instance_num	= UART0_INSTANCE_NUM,

#if RS485_SUPPORT
	.rs485_cfg		= &uart0_rs485_config,
#endif
};


/* Function Name: UART0_Initialize */
static int32_t UART0_Initialize(ARM_USART_SignalEvent_t cb_event) 
{
    return (ARM_USART_Initialize(cb_event,  &UART0_Resources));
}/* End of function UART0_Initialize() */


/* Function Name: UART0_Uninitialize */
static int32_t UART0_Uninitialize(void) 
{
    return (ARM_USART_Uninitialize(&UART0_Resources));
}/* End of function UART0_Uninitialize() */


/* Function Name: UART0_PowerControl */
static int32_t UART0_PowerControl(ARM_POWER_STATE state) 
{
    return (ARM_USART_PowerControl(state, &UART0_Resources));
}/* End of function UART0_PowerControl() */


/* Function Name: UART0_Send */
static int32_t UART0_Send(void const * const p_data, uint32_t num) 
{
    return (ARM_USART_Send(p_data, num , &UART0_Resources));
}/* End of function UART0_Send() */


/* Function Name: UART0_Receive */
static int32_t UART0_Receive(void * const p_data, uint32_t num) 
{
    return (ARM_USART_Receive(p_data, num, &UART0_Resources));
}/* End of function UART0_Receive() */


/* Function Name: UART0_Transfer */
static int32_t UART0_Transfer(void const * const p_data_out, void * const p_data_in, uint32_t num)
{
    return (ARM_USART_Transfer(p_data_out, p_data_in, num, &UART0_Resources));
}/* End of function UART0_Transfer() */


/* Function Name: UART0_GetTxCount */
static uint32_t UART0_GetTxCount(void) 
{
    return (ARM_USART_GetTxCount(&UART0_Resources));
}/* End of function UART0_GetTxCount() */


/* Function Name: UART0_GetRxCount */
static uint32_t UART0_GetRxCount(void) 
{
    return (ARM_USART_GetRxCount(&UART0_Resources));
}/* End of function UART0_GetRxCount() */


/* Function Name: UART0_Control */
static int32_t UART0_Control(uint32_t control, uint32_t arg) 
{
    return (ARM_USART_Control(control, arg, &UART0_Resources));
}/* End of function UART0_Control() */


/* Function Name: UART0_GetStatus */
static ARM_USART_STATUS UART0_GetStatus(void) 
{
    return (ARM_USART_GetStatus(&UART0_Resources));
}/* End of function UART0_GetStatus() */


/* Function Name: UART0_GetModemStatus */
static ARM_USART_MODEM_STATUS UART0_GetModemStatus(void) 
{
    return (ARM_USART_GetModemStatus(&UART0_Resources));
}/* End of function UART0_GetModemStatus() */


/* Function Name: UART0_SetModemControl */
static int32_t UART0_SetModemControl(ARM_USART_MODEM_CONTROL control)
{
    return (ARM_USART_SetModemControl(control, &UART0_Resources));
}/* End of function UART0_SetModemControl() */

void UART0_IRQHandler (void) 
{
  ARM_USART_IRQHandler (&UART0_Resources);
}

extern ARM_DRIVER_USART Driver_UART0;
ARM_DRIVER_USART Driver_UART0 = 
{
    ARM_USART_GetVersion,  
    ARM_USART_GetCapabilities,
    UART0_Initialize,
    UART0_Uninitialize,
    UART0_PowerControl,
    UART0_Send,
    UART0_Receive,
    UART0_Transfer,
    UART0_GetTxCount,
    UART0_GetRxCount,
    UART0_Control,
    UART0_GetStatus,
    UART0_SetModemControl,
    UART0_GetModemStatus
};
#endif /* RTE_UART0 */


/* UART1 Driver Instance */
#if (RTE_UART1)

static uart_config_info_t uart1_config =
{
	.clk_source      = RTE_UART1_CLK_SOURCE,
	.rx_fifo_trg_lvl = RTE_UART1_RX_TRIG_LVL,
	.tx_fifo_trg_lvl = RTE_UART1_TX_TRIG_LVL,
};
static uart_info_t uart1_info = {0};

#if RS485_SUPPORT
static uart_config_rs485_info_t uart1_rs485_config = {0};
#endif

/* UART1 Driver Resources */
static uart_resources_t UART1_Resources =
{
	.reg_base 		= (uint32_t)RTE_UART1_PHY_ADDR,
	.clk 			= (uint32_t)0,
	.cfg			= &uart1_config,
	.info 			= &uart1_info,
	.irq_num		= UART1_IRQ_NUM,
	.irq_priority	= (uint32_t)RTE_UART1_IRQ_PRI,
	.instance_num	= UART1_INSTANCE_NUM,

#if RS485_SUPPORT
	.rs485_cfg		= &uart1_rs485_config,
#endif
};


/* Function Name: UART1_Initialize */
static int32_t UART1_Initialize(ARM_USART_SignalEvent_t cb_event)
{
    return (ARM_USART_Initialize(cb_event,  &UART1_Resources));
}/* End of function UART1_Initialize() */


/* Function Name: UART1_Uninitialize */
static int32_t UART1_Uninitialize(void)
{
    return (ARM_USART_Uninitialize(&UART1_Resources));
}/* End of function UART1_Uninitialize() */


/* Function Name: UART1_PowerControl */
static int32_t UART1_PowerControl(ARM_POWER_STATE state)
{
    return (ARM_USART_PowerControl(state, &UART1_Resources));
}/* End of function UART1_PowerControl() */


/* Function Name: UART1_Send */
static int32_t UART1_Send(void const * const p_data, uint32_t num)
{
    return (ARM_USART_Send(p_data, num , &UART1_Resources));
}/* End of function UART1_Send() */


/* Function Name: UART1_Receive */
static int32_t UART1_Receive(void * const p_data, uint32_t num)
{
    return (ARM_USART_Receive(p_data, num, &UART1_Resources));
}/* End of function UART1_Receive() */


/* Function Name: UART1_Transfer */
static int32_t UART1_Transfer(void const * const p_data_out, void * const p_data_in, uint32_t num)
{
    return (ARM_USART_Transfer(p_data_out, p_data_in, num, &UART1_Resources));
}/* End of function UART1_Transfer() */


/* Function Name: UART1_GetTxCount */
static uint32_t UART1_GetTxCount(void)
{
    return (ARM_USART_GetTxCount(&UART1_Resources));
}/* End of function UART1_GetTxCount() */


/* Function Name: UART1_GetRxCount */
static uint32_t UART1_GetRxCount(void)
{
    return (ARM_USART_GetRxCount(&UART1_Resources));
}/* End of function UART1_GetRxCount() */


/* Function Name: UART1_Control */
static int32_t UART1_Control(uint32_t control, uint32_t arg)
{
    return (ARM_USART_Control(control, arg, &UART1_Resources));
}/* End of function UART1_Control() */


/* Function Name: UART1_GetStatus */
static ARM_USART_STATUS UART1_GetStatus(void)
{
    return (ARM_USART_GetStatus(&UART1_Resources));
}/* End of function UART1_GetStatus() */


/* Function Name: UART1_GetModemStatus */
static ARM_USART_MODEM_STATUS UART1_GetModemStatus(void)
{
    return (ARM_USART_GetModemStatus(&UART1_Resources));
}/* End of function UART1_GetModemStatus() */


/* Function Name: UART1_SetModemControl */
static int32_t UART1_SetModemControl(ARM_USART_MODEM_CONTROL control)
{
    return (ARM_USART_SetModemControl(control, &UART1_Resources));
}/* End of function UART1_SetModemControl() */

void UART1_IRQHandler (void)
{
  ARM_USART_IRQHandler (&UART1_Resources);
}

extern ARM_DRIVER_USART Driver_UART1;
ARM_DRIVER_USART Driver_UART1 =
{
    ARM_USART_GetVersion,
    ARM_USART_GetCapabilities,
    UART1_Initialize,
    UART1_Uninitialize,
    UART1_PowerControl,
    UART1_Send,
    UART1_Receive,
    UART1_Transfer,
    UART1_GetTxCount,
    UART1_GetRxCount,
    UART1_Control,
    UART1_GetStatus,
    UART1_SetModemControl,
    UART1_GetModemStatus
};
#endif /* RTE_UART1 */


/* UART2 Driver Instance */
#if (RTE_UART2)

static uart_config_info_t uart2_config =
{
	.clk_source      = RTE_UART2_CLK_SOURCE,
	.rx_fifo_trg_lvl = RTE_UART2_RX_TRIG_LVL,
	.tx_fifo_trg_lvl = RTE_UART2_TX_TRIG_LVL,
};
static uart_info_t uart2_info = {0};

#if RS485_SUPPORT
static uart_config_rs485_info_t uart2_rs485_config = {0};
#endif

/* UART2 Driver Resources */
static uart_resources_t UART2_Resources =
{
	.reg_base 		= (uint32_t)RTE_UART2_PHY_ADDR,
	.clk 			= (uint32_t)0,
	.cfg			= &uart2_config,
	.info 			= &uart2_info,
	.irq_num		= UART2_IRQ_NUM,
	.irq_priority	= (uint32_t)RTE_UART2_IRQ_PRI,
	.instance_num	= UART2_INSTANCE_NUM,

#if RS485_SUPPORT
	.rs485_cfg		= &uart2_rs485_config,
#endif
};


/* Function Name: UART2_Initialize */
static int32_t UART2_Initialize(ARM_USART_SignalEvent_t cb_event)
{
    return (ARM_USART_Initialize(cb_event,  &UART2_Resources));
}/* End of function UART2_Initialize() */


/* Function Name: UART2_Uninitialize */
static int32_t UART2_Uninitialize(void)
{
    return (ARM_USART_Uninitialize(&UART2_Resources));
}/* End of function UART2_Uninitialize() */


/* Function Name: UART2_PowerControl */
static int32_t UART2_PowerControl(ARM_POWER_STATE state)
{
    return (ARM_USART_PowerControl(state, &UART2_Resources));
}/* End of function UART2_PowerControl() */


/* Function Name: UART2_Send */
static int32_t UART2_Send(void const * const p_data, uint32_t num)
{
    return (ARM_USART_Send(p_data, num , &UART2_Resources));
}/* End of function UART2_Send() */


/* Function Name: UART2_Receive */
static int32_t UART2_Receive(void * const p_data, uint32_t num)
{
    return (ARM_USART_Receive(p_data, num, &UART2_Resources));
}/* End of function UART2_Receive() */


/* Function Name: UART2_Transfer */
static int32_t UART2_Transfer(void const * const p_data_out, void * const p_data_in, uint32_t num)
{
    return (ARM_USART_Transfer(p_data_out, p_data_in, num, &UART2_Resources));
}/* End of function UART2_Transfer() */


/* Function Name: UART2_GetTxCount */
static uint32_t UART2_GetTxCount(void)
{
    return (ARM_USART_GetTxCount(&UART2_Resources));
}/* End of function UART2_GetTxCount() */


/* Function Name: UART2_GetRxCount */
static uint32_t UART2_GetRxCount(void)
{
    return (ARM_USART_GetRxCount(&UART2_Resources));
}/* End of function UART2_GetRxCount() */


/* Function Name: UART2_Control */
static int32_t UART2_Control(uint32_t control, uint32_t arg)
{
    return (ARM_USART_Control(control, arg, &UART2_Resources));
}/* End of function UART2_Control() */


/* Function Name: UART2_GetStatus */
static ARM_USART_STATUS UART2_GetStatus(void)
{
    return (ARM_USART_GetStatus(&UART2_Resources));
}/* End of function UART2_GetStatus() */


/* Function Name: UART2_GetModemStatus */
static ARM_USART_MODEM_STATUS UART2_GetModemStatus(void)
{
    return (ARM_USART_GetModemStatus(&UART2_Resources));
}/* End of function UART2_GetModemStatus() */


/* Function Name: UART2_SetModemControl */
static int32_t UART2_SetModemControl(ARM_USART_MODEM_CONTROL control)
{
    return (ARM_USART_SetModemControl(control, &UART2_Resources));
}/* End of function UART2_SetModemControl() */

void UART2_IRQHandler (void)
{
  ARM_USART_IRQHandler (&UART2_Resources);
}

extern ARM_DRIVER_USART Driver_UART2;
ARM_DRIVER_USART Driver_UART2 =
{
    ARM_USART_GetVersion,
    ARM_USART_GetCapabilities,
    UART2_Initialize,
    UART2_Uninitialize,
    UART2_PowerControl,
    UART2_Send,
    UART2_Receive,
    UART2_Transfer,
    UART2_GetTxCount,
    UART2_GetRxCount,
    UART2_Control,
    UART2_GetStatus,
    UART2_SetModemControl,
    UART2_GetModemStatus
};
#endif /* RTE_UART2 */


/* UART3 Driver Instance */
#if (RTE_UART3)

static uart_config_info_t uart3_config =
{
	.clk_source      = RTE_UART3_CLK_SOURCE,
	.rx_fifo_trg_lvl = RTE_UART3_RX_TRIG_LVL,
	.tx_fifo_trg_lvl = RTE_UART3_TX_TRIG_LVL,
};
static uart_info_t uart3_info = {0};

#if RS485_SUPPORT
static uart_config_rs485_info_t uart3_rs485_config = {0};
#endif

/* UART3 Driver Resources */
static uart_resources_t UART3_Resources =
{
	.reg_base 		= (uint32_t)RTE_UART3_PHY_ADDR,
	.clk 			= (uint32_t)0,
	.cfg			= &uart3_config,
	.info 			= &uart3_info,
	.irq_num		= UART3_IRQ_NUM,
	.irq_priority	= (uint32_t)RTE_UART3_IRQ_PRI,
	.instance_num	= UART3_INSTANCE_NUM,

#if RS485_SUPPORT
	.rs485_cfg		= &uart3_rs485_config,
#endif
};


/* Function Name: UART3_Initialize */
static int32_t UART3_Initialize(ARM_USART_SignalEvent_t cb_event)
{
    return (ARM_USART_Initialize(cb_event,  &UART3_Resources));
}/* End of function UART3_Initialize() */


/* Function Name: UART3_Uninitialize */
static int32_t UART3_Uninitialize(void)
{
    return (ARM_USART_Uninitialize(&UART3_Resources));
}/* End of function UART3_Uninitialize() */


/* Function Name: UART3_PowerControl */
static int32_t UART3_PowerControl(ARM_POWER_STATE state)
{
    return (ARM_USART_PowerControl(state, &UART3_Resources));
}/* End of function UART3_PowerControl() */


/* Function Name: UART3_Send */
static int32_t UART3_Send(void const * const p_data, uint32_t num)
{
    return (ARM_USART_Send(p_data, num , &UART3_Resources));
}/* End of function UART3_Send() */


/* Function Name: UART3_Receive */
static int32_t UART3_Receive(void * const p_data, uint32_t num)
{
    return (ARM_USART_Receive(p_data, num, &UART3_Resources));
}/* End of function UART3_Receive() */


/* Function Name: UART3_Transfer */
static int32_t UART3_Transfer(void const * const p_data_out, void * const p_data_in, uint32_t num)
{
    return (ARM_USART_Transfer(p_data_out, p_data_in, num, &UART3_Resources));
}/* End of function UART3_Transfer() */


/* Function Name: UART3_GetTxCount */
static uint32_t UART3_GetTxCount(void)
{
    return (ARM_USART_GetTxCount(&UART3_Resources));
}/* End of function UART3_GetTxCount() */


/* Function Name: UART3_GetRxCount */
static uint32_t UART3_GetRxCount(void)
{
    return (ARM_USART_GetRxCount(&UART3_Resources));
}/* End of function UART3_GetRxCount() */


/* Function Name: UART3_Control */
static int32_t UART3_Control(uint32_t control, uint32_t arg)
{
    return (ARM_USART_Control(control, arg, &UART3_Resources));
}/* End of function UART3_Control() */


/* Function Name: UART3_GetStatus */
static ARM_USART_STATUS UART3_GetStatus(void)
{
    return (ARM_USART_GetStatus(&UART3_Resources));
}/* End of function UART3_GetStatus() */


/* Function Name: UART3_GetModemStatus */
static ARM_USART_MODEM_STATUS UART3_GetModemStatus(void)
{
    return (ARM_USART_GetModemStatus(&UART3_Resources));
}/* End of function UART3_GetModemStatus() */


/* Function Name: UART3_SetModemControl */
static int32_t UART3_SetModemControl(ARM_USART_MODEM_CONTROL control)
{
    return (ARM_USART_SetModemControl(control, &UART3_Resources));
}/* End of function UART3_SetModemControl() */

void UART3_IRQHandler (void)
{
  ARM_USART_IRQHandler (&UART3_Resources);
}

extern ARM_DRIVER_USART Driver_UART3;
ARM_DRIVER_USART Driver_UART3 =
{
    ARM_USART_GetVersion,
    ARM_USART_GetCapabilities,
    UART3_Initialize,
    UART3_Uninitialize,
    UART3_PowerControl,
    UART3_Send,
    UART3_Receive,
    UART3_Transfer,
    UART3_GetTxCount,
    UART3_GetRxCount,
    UART3_Control,
    UART3_GetStatus,
    UART3_SetModemControl,
    UART3_GetModemStatus
};
#endif /* RTE_UART3 */


/* UART4 Driver Instance */
#if (RTE_UART4)

static uart_config_info_t uart4_config =
{
	.clk_source      = RTE_UART4_CLK_SOURCE,
	.rx_fifo_trg_lvl = RTE_UART4_RX_TRIG_LVL,
	.tx_fifo_trg_lvl = RTE_UART4_TX_TRIG_LVL,
};

#if RS485_SUPPORT
static uart_config_rs485_info_t uart4_rs485_config =
{
	.rs485_transfer_mode 					= RTE_UART4_RS485_TRANSFER_MODE,
	.rs485_de_assertion_time_8bit 			= RTE_UART4_RS485_DE_ASSERTION_TIME_8BIT,
	.rs485_de_deassertion_time_8bit 		= RTE_UART4_RS485_DE_DEASSERTION_TIME_8BIT,
	.rs485_de_to_re_turn_around_time_16bit  = RTE_UART4_RS485_DE_TO_RE_TURN_AROUND_TIME_16BIT,
	.rs485_re_to_de_turn_around_time_16bit  = RTE_UART4_RS485_RE_TO_DE_TURN_AROUND_TIME_16BIT,
};
#endif //END of RS485_SUPPORT

static uart_info_t uart4_info = {0};

/* UART4 Driver Resources */
static uart_resources_t UART4_Resources =
{
	.reg_base 		= (uint32_t)RTE_UART4_PHY_ADDR,
	.clk 			= (uint32_t)0,
	.cfg			= &uart4_config,
	.info 			= &uart4_info,
	.irq_num		= UART4_IRQ_NUM,
	.irq_priority	= (uint32_t)RTE_UART4_IRQ_PRI,
	.instance_num	= UART4_INSTANCE_NUM,

#if RS485_SUPPORT
	.rs485_cfg		= &uart4_rs485_config,
#endif

};

/* Function Name: UART4_Initialize */
static int32_t UART4_Initialize(ARM_USART_SignalEvent_t cb_event)
{
    return (ARM_USART_Initialize(cb_event,  &UART4_Resources));
}/* End of function UART4_Initialize() */


/* Function Name: UART4_Uninitialize */
static int32_t UART4_Uninitialize(void)
{
    return (ARM_USART_Uninitialize(&UART4_Resources));
}/* End of function UART4_Uninitialize() */


/* Function Name: UART4_PowerControl */
static int32_t UART4_PowerControl(ARM_POWER_STATE state)
{
    return (ARM_USART_PowerControl(state, &UART4_Resources));
}/* End of function UART4_PowerControl() */


/* Function Name: UART4_Send */
static int32_t UART4_Send(void const * const p_data, uint32_t num)
{
    return (ARM_USART_Send(p_data, num , &UART4_Resources));
}/* End of function UART4_Send() */


/* Function Name: UART4_Receive */
static int32_t UART4_Receive(void * const p_data, uint32_t num)
{
    return (ARM_USART_Receive(p_data, num, &UART4_Resources));
}/* End of function UART4_Receive() */


/* Function Name: UART4_Transfer */
static int32_t UART4_Transfer(void const * const p_data_out, void * const p_data_in, uint32_t num)
{
    return (ARM_USART_Transfer(p_data_out, p_data_in, num, &UART4_Resources));
}/* End of function UART4_Transfer() */


/* Function Name: UART4_GetTxCount */
static uint32_t UART4_GetTxCount(void)
{
    return (ARM_USART_GetTxCount(&UART4_Resources));
}/* End of function UART4_GetTxCount() */


/* Function Name: UART4_GetRxCount */
static uint32_t UART4_GetRxCount(void)
{
    return (ARM_USART_GetRxCount(&UART4_Resources));
}/* End of function UART4_GetRxCount() */


/* Function Name: UART4_Control */
static int32_t UART4_Control(uint32_t control, uint32_t arg)
{
    return (ARM_USART_Control(control, arg, &UART4_Resources));
}/* End of function UART4_Control() */


/* Function Name: UART4_GetStatus */
static ARM_USART_STATUS UART4_GetStatus(void)
{
    return (ARM_USART_GetStatus(&UART4_Resources));
}/* End of function UART4_GetStatus() */


/* Function Name: UART4_GetModemStatus */
static ARM_USART_MODEM_STATUS UART4_GetModemStatus(void)
{
    return (ARM_USART_GetModemStatus(&UART4_Resources));
}/* End of function UART4_GetModemStatus() */


/* Function Name: UART4_SetModemControl */
static int32_t UART4_SetModemControl(ARM_USART_MODEM_CONTROL control)
{
    return (ARM_USART_SetModemControl(control, &UART4_Resources));
}/* End of function UART4_SetModemControl() */

void UART4_IRQHandler (void)
{
  ARM_USART_IRQHandler (&UART4_Resources);
}

extern ARM_DRIVER_USART Driver_UART4;
ARM_DRIVER_USART Driver_UART4 =
{
    ARM_USART_GetVersion,
    ARM_USART_GetCapabilities,
    UART4_Initialize,
    UART4_Uninitialize,
    UART4_PowerControl,
    UART4_Send,
    UART4_Receive,
    UART4_Transfer,
    UART4_GetTxCount,
    UART4_GetRxCount,
    UART4_Control,
    UART4_GetStatus,
    UART4_SetModemControl,
    UART4_GetModemStatus
};
#endif /* RTE_UART4 */


/* UART5 Driver Instance */
#if (RTE_UART5)

static uart_config_info_t uart5_config =
{
	.clk_source      = RTE_UART5_CLK_SOURCE,
	.rx_fifo_trg_lvl = RTE_UART5_RX_TRIG_LVL,
	.tx_fifo_trg_lvl = RTE_UART5_TX_TRIG_LVL,
};

#if RS485_SUPPORT
static uart_config_rs485_info_t uart5_rs485_config =
{
	.rs485_transfer_mode 					= RTE_UART5_RS485_TRANSFER_MODE,
	.rs485_de_assertion_time_8bit 			= RTE_UART5_RS485_DE_ASSERTION_TIME_8BIT,
	.rs485_de_deassertion_time_8bit 		= RTE_UART5_RS485_DE_DEASSERTION_TIME_8BIT,
	.rs485_de_to_re_turn_around_time_16bit  = RTE_UART5_RS485_DE_TO_RE_TURN_AROUND_TIME_16BIT,
	.rs485_re_to_de_turn_around_time_16bit  = RTE_UART5_RS485_RE_TO_DE_TURN_AROUND_TIME_16BIT,
};
#endif //END of RS485_SUPPORT

static uart_info_t uart5_info = {0};

/* UART5 Driver Resources */
static uart_resources_t UART5_Resources =
{
	.reg_base 		= (uint32_t)RTE_UART5_PHY_ADDR,
	.clk 			= (uint32_t)0,
	.cfg			= &uart5_config,
	.info 			= &uart5_info,
	.irq_num		= UART5_IRQ_NUM,
	.irq_priority	= (uint32_t)RTE_UART5_IRQ_PRI,
	.instance_num	= UART5_INSTANCE_NUM,

#if RS485_SUPPORT
	.rs485_cfg		= &uart5_rs485_config,
#endif
};

/* Function Name: UART5_Initialize */
static int32_t UART5_Initialize(ARM_USART_SignalEvent_t cb_event)
{
    return (ARM_USART_Initialize(cb_event,  &UART5_Resources));
}/* End of function UART5_Initialize() */


/* Function Name: UART5_Uninitialize */
static int32_t UART5_Uninitialize(void)
{
    return (ARM_USART_Uninitialize(&UART5_Resources));
}/* End of function UART5_Uninitialize() */


/* Function Name: UART5_PowerControl */
static int32_t UART5_PowerControl(ARM_POWER_STATE state)
{
    return (ARM_USART_PowerControl(state, &UART5_Resources));
}/* End of function UART5_PowerControl() */


/* Function Name: UART5_Send */
static int32_t UART5_Send(void const * const p_data, uint32_t num)
{
    return (ARM_USART_Send(p_data, num , &UART5_Resources));
}/* End of function UART5_Send() */


/* Function Name: UART5_Receive */
static int32_t UART5_Receive(void * const p_data, uint32_t num)
{
    return (ARM_USART_Receive(p_data, num, &UART5_Resources));
}/* End of function UART5_Receive() */


/* Function Name: UART5_Transfer */
static int32_t UART5_Transfer(void const * const p_data_out, void * const p_data_in, uint32_t num)
{
    return (ARM_USART_Transfer(p_data_out, p_data_in, num, &UART5_Resources));
}/* End of function UART5_Transfer() */


/* Function Name: UART5_GetTxCount */
static uint32_t UART5_GetTxCount(void)
{
    return (ARM_USART_GetTxCount(&UART5_Resources));
}/* End of function UART5_GetTxCount() */


/* Function Name: UART5_GetRxCount */
static uint32_t UART5_GetRxCount(void)
{
    return (ARM_USART_GetRxCount(&UART5_Resources));
}/* End of function UART5_GetRxCount() */


/* Function Name: UART5_Control */
static int32_t UART5_Control(uint32_t control, uint32_t arg)
{
    return (ARM_USART_Control(control, arg, &UART5_Resources));
}/* End of function UART5_Control() */


/* Function Name: UART5_GetStatus */
static ARM_USART_STATUS UART5_GetStatus(void)
{
    return (ARM_USART_GetStatus(&UART5_Resources));
}/* End of function UART5_GetStatus() */


/* Function Name: UART5_GetModemStatus */
static ARM_USART_MODEM_STATUS UART5_GetModemStatus(void)
{
    return (ARM_USART_GetModemStatus(&UART5_Resources));
}/* End of function UART5_GetModemStatus() */


/* Function Name: UART5_SetModemControl */
static int32_t UART5_SetModemControl(ARM_USART_MODEM_CONTROL control)
{
    return (ARM_USART_SetModemControl(control, &UART5_Resources));
}/* End of function UART5_SetModemControl() */

void UART5_IRQHandler (void)
{
  ARM_USART_IRQHandler (&UART5_Resources);
}

extern ARM_DRIVER_USART Driver_UART5;
ARM_DRIVER_USART Driver_UART5 =
{
    ARM_USART_GetVersion,
    ARM_USART_GetCapabilities,
    UART5_Initialize,
    UART5_Uninitialize,
    UART5_PowerControl,
    UART5_Send,
    UART5_Receive,
    UART5_Transfer,
    UART5_GetTxCount,
    UART5_GetRxCount,
    UART5_Control,
    UART5_GetStatus,
    UART5_SetModemControl,
    UART5_GetModemStatus
};
#endif /* RTE_UART5 */


/* UART6 Driver Instance */
#if (RTE_UART6)

static uart_config_info_t uart6_config =
{
	.clk_source      = RTE_UART6_CLK_SOURCE,
	.rx_fifo_trg_lvl = RTE_UART6_RX_TRIG_LVL,
	.tx_fifo_trg_lvl = RTE_UART6_TX_TRIG_LVL,
};
static uart_info_t uart6_info = {0};

#if RS485_SUPPORT
static uart_config_rs485_info_t uart6_rs485_config =
{
	.rs485_transfer_mode 					= RTE_UART6_RS485_TRANSFER_MODE,
	.rs485_de_assertion_time_8bit 			= RTE_UART6_RS485_DE_ASSERTION_TIME_8BIT,
	.rs485_de_deassertion_time_8bit 		= RTE_UART6_RS485_DE_DEASSERTION_TIME_8BIT,
	.rs485_de_to_re_turn_around_time_16bit  = RTE_UART6_RS485_DE_TO_RE_TURN_AROUND_TIME_16BIT,
	.rs485_re_to_de_turn_around_time_16bit  = RTE_UART6_RS485_RE_TO_DE_TURN_AROUND_TIME_16BIT,
};
#endif //END of RS485_SUPPORT

/* UART6 Driver Resources */
static uart_resources_t UART6_Resources =
{
	.reg_base 		= (uint32_t)RTE_UART6_PHY_ADDR,
	.clk 			= (uint32_t)0,
	.cfg			= &uart6_config,
	.info 			= &uart6_info,
	.irq_num		= UART6_IRQ_NUM,
	.irq_priority	= (uint32_t)RTE_UART6_IRQ_PRI,
	.instance_num	= UART6_INSTANCE_NUM,

#if RS485_SUPPORT
	.rs485_cfg		= &uart6_rs485_config,
#endif

};

/* Function Name: UART6_Initialize */
static int32_t UART6_Initialize(ARM_USART_SignalEvent_t cb_event)
{
    return (ARM_USART_Initialize(cb_event,  &UART6_Resources));
}/* End of function UART6_Initialize() */


/* Function Name: UART6_Uninitialize */
static int32_t UART6_Uninitialize(void)
{
    return (ARM_USART_Uninitialize(&UART6_Resources));
}/* End of function UART6_Uninitialize() */


/* Function Name: UART6_PowerControl */
static int32_t UART6_PowerControl(ARM_POWER_STATE state)
{
    return (ARM_USART_PowerControl(state, &UART6_Resources));
}/* End of function UART6_PowerControl() */


/* Function Name: UART6_Send */
static int32_t UART6_Send(void const * const p_data, uint32_t num)
{
    return (ARM_USART_Send(p_data, num , &UART6_Resources));
}/* End of function UART6_Send() */


/* Function Name: UART6_Receive */
static int32_t UART6_Receive(void * const p_data, uint32_t num)
{
    return (ARM_USART_Receive(p_data, num, &UART6_Resources));
}/* End of function UART6_Receive() */


/* Function Name: UART6_Transfer */
static int32_t UART6_Transfer(void const * const p_data_out, void * const p_data_in, uint32_t num)
{
    return (ARM_USART_Transfer(p_data_out, p_data_in, num, &UART6_Resources));
}/* End of function UART6_Transfer() */


/* Function Name: UART6_GetTxCount */
static uint32_t UART6_GetTxCount(void)
{
    return (ARM_USART_GetTxCount(&UART6_Resources));
}/* End of function UART6_GetTxCount() */


/* Function Name: UART6_GetRxCount */
static uint32_t UART6_GetRxCount(void)
{
    return (ARM_USART_GetRxCount(&UART6_Resources));
}/* End of function UART6_GetRxCount() */


/* Function Name: UART6_Control */
static int32_t UART6_Control(uint32_t control, uint32_t arg)
{
    return (ARM_USART_Control(control, arg, &UART6_Resources));
}/* End of function UART6_Control() */


/* Function Name: UART6_GetStatus */
static ARM_USART_STATUS UART6_GetStatus(void)
{
    return (ARM_USART_GetStatus(&UART6_Resources));
}/* End of function UART6_GetStatus() */


/* Function Name: UART6_GetModemStatus */
static ARM_USART_MODEM_STATUS UART6_GetModemStatus(void)
{
    return (ARM_USART_GetModemStatus(&UART6_Resources));
}/* End of function UART6_GetModemStatus() */


/* Function Name: UART6_SetModemControl */
static int32_t UART6_SetModemControl(ARM_USART_MODEM_CONTROL control)
{
    return (ARM_USART_SetModemControl(control, &UART6_Resources));
}/* End of function UART6_SetModemControl() */

void UART6_IRQHandler (void)
{
  ARM_USART_IRQHandler (&UART6_Resources);
}

extern ARM_DRIVER_USART Driver_UART6;
ARM_DRIVER_USART Driver_UART6 =
{
    ARM_USART_GetVersion,
    ARM_USART_GetCapabilities,
    UART6_Initialize,
    UART6_Uninitialize,
    UART6_PowerControl,
    UART6_Send,
    UART6_Receive,
    UART6_Transfer,
    UART6_GetTxCount,
    UART6_GetRxCount,
    UART6_Control,
    UART6_GetStatus,
    UART6_SetModemControl,
    UART6_GetModemStatus
};
#endif /* RTE_UART6 */


/* UART7 Driver Instance */
#if (RTE_UART7)

static uart_config_info_t uart7_config =
{
	.clk_source      = RTE_UART7_CLK_SOURCE,
	.rx_fifo_trg_lvl = RTE_UART7_RX_TRIG_LVL,
	.tx_fifo_trg_lvl = RTE_UART7_TX_TRIG_LVL,
};
static uart_info_t uart7_info = {0};

#if RS485_SUPPORT
static uart_config_rs485_info_t uart7_rs485_config =
{
	.rs485_transfer_mode 					= RTE_UART7_RS485_TRANSFER_MODE,
	.rs485_de_assertion_time_8bit 			= RTE_UART7_RS485_DE_ASSERTION_TIME_8BIT,
	.rs485_de_deassertion_time_8bit 		= RTE_UART7_RS485_DE_DEASSERTION_TIME_8BIT,
	.rs485_de_to_re_turn_around_time_16bit  = RTE_UART7_RS485_DE_TO_RE_TURN_AROUND_TIME_16BIT,
	.rs485_re_to_de_turn_around_time_16bit  = RTE_UART7_RS485_RE_TO_DE_TURN_AROUND_TIME_16BIT,
};
#endif //END of RS485_SUPPORT

/* UART7 Driver Resources */
static uart_resources_t UART7_Resources =
{
	.reg_base 		= (uint32_t)RTE_UART7_PHY_ADDR,
	.clk 			= (uint32_t)0,
	.cfg			= &uart7_config,
	.info 			= &uart7_info,
	.irq_num		= UART7_IRQ_NUM,
	.irq_priority	= (uint32_t)RTE_UART7_IRQ_PRI,
	.instance_num	= UART7_INSTANCE_NUM,

#if RS485_SUPPORT
	.rs485_cfg		= &uart7_rs485_config,
#endif

};

/* Function Name: UART7_Initialize */
static int32_t UART7_Initialize(ARM_USART_SignalEvent_t cb_event)
{
    return (ARM_USART_Initialize(cb_event,  &UART7_Resources));
}/* End of function UART7_Initialize() */


/* Function Name: UART7_Uninitialize */
static int32_t UART7_Uninitialize(void)
{
    return (ARM_USART_Uninitialize(&UART7_Resources));
}/* End of function UART7_Uninitialize() */


/* Function Name: UART7_PowerControl */
static int32_t UART7_PowerControl(ARM_POWER_STATE state)
{
    return (ARM_USART_PowerControl(state, &UART7_Resources));
}/* End of function UART7_PowerControl() */


/* Function Name: UART7_Send */
static int32_t UART7_Send(void const * const p_data, uint32_t num)
{
    return (ARM_USART_Send(p_data, num , &UART7_Resources));
}/* End of function UART7_Send() */


/* Function Name: UART7_Receive */
static int32_t UART7_Receive(void * const p_data, uint32_t num)
{
    return (ARM_USART_Receive(p_data, num, &UART7_Resources));
}/* End of function UART7_Receive() */


/* Function Name: UART7_Transfer */
static int32_t UART7_Transfer(void const * const p_data_out, void * const p_data_in, uint32_t num)
{
    return (ARM_USART_Transfer(p_data_out, p_data_in, num, &UART7_Resources));
}/* End of function UART7_Transfer() */


/* Function Name: UART7_GetTxCount */
static uint32_t UART7_GetTxCount(void)
{
    return (ARM_USART_GetTxCount(&UART7_Resources));
}/* End of function UART7_GetTxCount() */


/* Function Name: UART7_GetRxCount */
static uint32_t UART7_GetRxCount(void)
{
    return (ARM_USART_GetRxCount(&UART7_Resources));
}/* End of function UART7_GetRxCount() */


/* Function Name: UART7_Control */
static int32_t UART7_Control(uint32_t control, uint32_t arg)
{
    return (ARM_USART_Control(control, arg, &UART7_Resources));
}/* End of function UART7_Control() */


/* Function Name: UART7_GetStatus */
static ARM_USART_STATUS UART7_GetStatus(void)
{
    return (ARM_USART_GetStatus(&UART7_Resources));
}/* End of function UART7_GetStatus() */


/* Function Name: UART7_GetModemStatus */
static ARM_USART_MODEM_STATUS UART7_GetModemStatus(void)
{
    return (ARM_USART_GetModemStatus(&UART7_Resources));
}/* End of function UART7_GetModemStatus() */


/* Function Name: UART7_SetModemControl */
static int32_t UART7_SetModemControl(ARM_USART_MODEM_CONTROL control)
{
    return (ARM_USART_SetModemControl(control, &UART7_Resources));
}/* End of function UART7_SetModemControl() */

void UART7_IRQHandler (void)
{
  ARM_USART_IRQHandler (&UART7_Resources);
}

extern ARM_DRIVER_USART Driver_UART7;
ARM_DRIVER_USART Driver_UART7 =
{
    ARM_USART_GetVersion,
    ARM_USART_GetCapabilities,
    UART7_Initialize,
    UART7_Uninitialize,
    UART7_PowerControl,
    UART7_Send,
    UART7_Receive,
    UART7_Transfer,
    UART7_GetTxCount,
    UART7_GetRxCount,
    UART7_Control,
    UART7_GetStatus,
    UART7_SetModemControl,
    UART7_GetModemStatus
};
#endif /* RTE_UART7 */


/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
