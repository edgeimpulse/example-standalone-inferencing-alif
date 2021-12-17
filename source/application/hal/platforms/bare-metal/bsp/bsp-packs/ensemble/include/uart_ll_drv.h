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
 * @file     uart_ll_drv.h
 * @author   Tanay Rami
 * @email    tanay@alifsemi.com
 * @version  V1.0.0
 * @date     20-May-2020
 * @brief    Low-level Driver Header file for both UART RS232 and RS485 modes.
 *
 *           This file is the unique include file that the application
 *           programmer is using in the C source code, usually in main.c.
 *           This file contains:
 *            - Global data types.
 *            - typedefs & enums.
 *            - define & macro.
 *            - Structures and Unions
 *            - Function prototypes.
 ******************************************************************************/


#ifndef UART_LL_DRV_H__ /* include guard */
#define UART_LL_DRV_H__

//#if defined (ARMCM55)
#include "ARMCM55.h"
//#else
//  #error device not specified!
//#endif

/* Includes --------------------------------------------------------------------------- */
/* System Includes */
#include <stdint.h>
#include "RTE_Device.h"
//#include "RTE_Components.h"
//#include CMSIS_device_header


#define CMSIS_device_header "ARMCM55.h"

/* Project Includes */
#include "Driver_USART_extended.h"

#ifdef __cplusplus
extern "C" {
#endif


#if ( (RTE_UART4) || (RTE_UART5) || (RTE_UART6) || (RTE_UART7) )
	#ifndef RS485_SUPPORT
	#define RS485_SUPPORT  (1) /* Enable(1): RS485 Support */
	#endif
#else
	#ifndef RS485_SUPPORT
	#define RS485_SUPPORT  (0) /* Disable(0): RS485 Support */
	#endif
#endif


/* UART register bit definitions ------------------------------------------------------- */

/* IER: interrupt enable register */
#define UART_IER_ENABLE_RECEIVED_DATA_AVAILABLE			(0x01)
#define UART_IER_ENABLE_TRANSMIT_HOLD_REG_EMPTY			(0x02)
#define UART_IER_ENABLE_RECEIVER_LINE_STATUS			(0x04)
#define UART_IER_ENABLE_MODEM_STATUS					(0x08)
#define UART_IER_PTIME			    					(0x80)

/* IIR: interrupt identity register */
#define UART_IIR_INTERRUPT_PENDING						(0x01)
#define UART_IIR_MASK									(0x0E)
#define UART_IIR_FIFO_ENABLE_STATUS						(0xC0)

/* interrupt IIR_MASK values */
#define UART_IIR_MODEM_STATUS							(0x00)
#define UART_IIR_TRANSMIT_HOLDING_REG_EMPTY				(0x02)
#define UART_IIR_RECEIVED_DATA_AVAILABLE				(0x04)
#define UART_IIR_RECEIVER_LINE_STATUS					(0x06)
#define UART_IIR_CHARACTER_TIMEOUT						(0x0C)
#define UART_IIR_INTERRUPT_ID_MASK						(0x0f)

/* FCR: FIFO control register */
#define UART_FCR_FIFO_ENABLE							(0x01)
#define UART_FCR_RCVR_FIFO_RESET						(0x02)
#define UART_FCR_TRANSMIT_FIFO_RESET					(0x04)
#define UART_FCR_DMA_SIGNAL_MODE						(0x08)
#define UART_FCR_RCVR_TRIGGER							(0xC0)

/* LCR: line control register */
#define UART_LCR_DATA_LENGTH_MASK				        (0x03)
#define UART_LCR_STOP_BIT_MASK				            (0x04)
#define UART_LCR_PARITY_MASK					        (0x38)
#define UART_LCR_DATA_PARITY_STOP_MASK					(0x3F)
#define UART_LCR_STICK_PARITY		   				    (0x20)
#define UART_LCR_BREAK			               			(0x40)
#define UART_LCR_DLAB									(0x80)

/* data length values */
#define UART_LCR_DATA_LENGTH_5							(0x00)
#define UART_LCR_DATA_LENGTH_6							(0x01)
#define UART_LCR_DATA_LENGTH_7							(0x02)
#define UART_LCR_DATA_LENGTH_8							(0x03)

/* stop bit values */
#define UART_LCR_STOP_1BIT								(0x00)
#define UART_LCR_STOP_1_5BIT							(0x04)
#define UART_LCR_STOP_2BIT								(0x04)

/* Parity bit values */
#define UART_LCR_PARITY_NONE							(0x00)
#define UART_LCR_PARITY_ODD								(0x08)
#define UART_LCR_PARITY_EVEN							(0x18)
#define UART_LCR_PARITY_STICK_LOGIC1					(0x28)
#define UART_LCR_PARITY_STICK_LOGIC0					(0x38)

/* MCR: modem control register */
#define UART_MCR_DTR									(0x01)
#define UART_MCR_RTS									(0x02)
#define UART_MCR_LOOPBACK								(0x10)
#define UART_MCR_AFCE									(0x20)
#define UART_MCR_SIRE									(0x40)

/* LSR: line status register */
#define UART_LSR_RCVR_DATA_READY						(0x01)
#define UART_LSR_OVERRUN_ERR							(0x02)
#define UART_LSR_PARITY_ERR								(0x04)
#define UART_LSR_FRAME_ERR								(0x08)
#define UART_LSR_BREAK_INTERRUPT						(0x10)
#define UART_LSR_TRANSMIT_HOLDING_REG_EMPTY				(0x20)
#define UART_LSR_TRANSMITTER_EMPTY						(0x40)
#define UART_LSR_RECEIVER_FIFO_ERR						(0x80)

/* MSR: modem status register */
#define UART_MSR_DCTS									(0x01)
#define UART_MSR_DDSR									(0x02)
#define UART_MSR_TERI									(0x04)
#define UART_MSR_DDCD									(0x08)
#define UART_MSR_CTS									(0x10)
#define UART_MSR_DSR									(0x20)
#define UART_MSR_RI										(0x40)
#define UART_MSR_DCD									(0x80)

/* USR: uart status register */
#define UART_USR_BUSY									(0x01)
#define UART_USR_TRANSMIT_FIFO_NOT_FULL					(0x02)
#define UART_USR_TRANSMIT_FIFO_EMPTY					(0x04)
#define UART_USR_RECEIVE_FIFO_NOT_EMPTY					(0x08)
#define UART_USR_RECEIVE_FIFO_FULL						(0x10)

/* SFE: shadow FIFO enable register */
#define UART_SFE_SHADOW_FIFO_ENABLE						(0x01)

/* SRR: software reset register */
#define UART_SRR_UART_RESET								(0x01)
#define UART_SRR_RCVR_FIFO_RESET						(0x02)
#define UART_SRR_TRANSMIT_FIFO_RESET					(0x04)

/* SRT: shadow receiver trigger register */
#define UART_SRT_TRIGGER_1_CHAR_IN_FIFO					(0x00)
#define UART_SRT_TRIGGER_FIFO_1_BY_4_FULL				(0x01)
#define UART_SRT_TRIGGER_FIFO_1_BY_2_FULL				(0x02)
#define UART_SRT_TRIGGER_FIFO_2_LESS_THAN_FULL			(0x03)

/* STET: Shadow TX empty register */
#define UART_STET_FIFO_EMPTY							(0x00)
#define UART_STET_2_CHARS_IN_FIFO						(0x01)
#define UART_STET_1_BY_4_FULL							(0x02)
#define UART_STET_1_BY_2_FULL							(0x03)

/* CPR: component parameter register */
#define UART_CPR_FIFO_STAT								(1<<10)
#define UART_CPR_FIFO_MODE_OFFSET						(16)
#define UART_CPR_FIFO_MODE_MASK							(0xFF)
#define UART_CPR_FIFO_MODE								(0xFF0000)
#define UART_CPR_SHADOW_MODE 					        (1 << 11)

/* DLF: divisor latch fraction register */
#define UART_DLF_SIZE									(0x04)

/* UART FIFO depth. */
#define UART_FIFO_DEPTH 								(32)        /* FIFO Depth for Tx & Rx  */

/* defines for uart baudrates */
#define UART_BAUDRATE_9600								(9600)		/* uart baudrate 9600bps 	*/
#define UART_BAUDRATE_115200							(115200)	/* uart baudrate 115200bps 	*/
#define UART_BAUDRATE_230400							(230400)	/* uart baudrate 230400bps 	*/
#define UART_BAUDRATE_460800							(460800)	/* uart baudrate 460800bps 	*/
#define UART_BAUDRATE_921600							(921600)	/* uart baudrate 921600bps 	*/

/**
 * Invalid Interrupt Vector Number When interrupt number is set to this value,
 * all interrupt related function in the device driver source code shouldn't be called.
 */
#define UART_INVALID_INT_NUM							(0xFFFFFFFF) /* invalid interrupt number 	*/
#define	UART_INVALID_PRIORITY							(0xFFFFFFFF) /* invalid interrupt priority  */

/* defines for uart flag */
#define UART_FLAG_INITIALIZED       					(1U << 0) 	/* uart initialized	   */
#define UART_FLAG_POWERED           					(1U << 1)	/* uart powered 	   */
#define UART_FLAG_CONFIGURED        					(1U << 2)	/* uart configured     */
#define UART_FLAG_TX_ENABLED       						(1U << 3)	/* uart tx enabled     */
#define UART_FLAG_RX_ENABLED        					(1U << 4)	/* uart rx enabled     */
#define UART_FLAG_SEND_ACTIVE     						(1U << 5)	/* uart send active    */
#define UART_FLAG_RS485_ENABLE							(1U << 6)	/* uart RS485 enabled  */

/* interrupt related flags */
#define UART_FLAG_TX_OR_RX_INT_ENABLE					(1U << 0)	/* to check tx or rx anyone interrupt is enable */
#define UART_FLAG_TX_INT_ENABLE							(1U << 1)	/* tx interrupt enable							*/
#define UART_FLAG_RX_INT_ENABLE							(1U << 2)	/* rx interrupt enable 							*/


/* ============================= RS485 control registers. ============================= */
#if RS485_SUPPORT

/* TCR: transceiver control register */
#define UART_TCR_RS485_DISABLE							(0x00)
#define UART_TCR_RS485_ENABLE							(0x01)

/* RE_POL */
#define UART_TCR_RE_POL_ACTIVE_LOW						(0x00)
#define UART_TCR_RE_POL_ACTIVE_HIGH						(0x02)

/* DE_POL */
#define UART_TCR_DE_POL_ACTIVE_LOW						(0x00)
#define UART_TCR_DE_POL_ACTIVE_HIGH						(0x04)

/* Transfer Modes */
#define UART_TCR_XFER_MODE_MASK							(0x18)
#define UART_TCR_XFER_MODE_FULL_DUPLEX					(0x00)
#define UART_TCR_XFER_MODE_SW_CONTROL_HALF_DUPLEX		(0x08)
#define UART_TCR_XFER_MODE_HW_CONTROL_HALF_DUPLEX		(0x10)

/* DE_EN: driver output enable register */
#define UART_DE_EN_DISABLE								(0x00) 	/* de-assert de signal */
#define UART_DE_EN_ENABLE								(0x01) 	/* assert    de signal */

/* RE_EN: receiver output enable register */
#define UART_RE_EN_DISABLE								(0x00) 	/* de-assert re signal */
#define UART_RE_EN_ENABLE								(0x01) 	/* assert    re signal */

/* DET: driver output enable timing register */
#define UART_DET_TIME_MASK 								(0xFF) 	/* 8 bits allocated for assertion and de-assertion time. */
#define UART_DET_DE_DEASSERTION_TIME_BIT_SHIFT			(16)  	/* bit-shift for DE de-assertion time.			         */

/* TAT: turn-around timing register */
#define UART_TAT_TIME_MASK 								(0xFFFF) /* 16 bits allocated for RE to DE and DE to RE time.    */
#define UART_TAT_RE_TO_DE_TIME_BIT_SHIFT				(16)   	 /* bit-shift for RE to DE time.		   			 	 */

#endif /* RS485 end  */


/**
 * UART Device TX Empty Trigger (TET) Types Enum
 * This is used to select the empty threshold level
 * in the transmitter FIFO at which the THRE
 * Interrupts will be generated.
 */
typedef enum
{
	UART_TX_FIFO_EMPTY   	  = 0,	    /* FIFO Empty   		 */
	UART_TX_FIFO_CHAR_2  	  = 1,	    /* 2 characters in FIFO	 */
	UART_TX_FIFO_QUARTER_FULL = 2,	    /* FIFO 1/4 full		 */
	UART_TX_FIFO_HALF_FULL 	  = 3	    /* FIFO 1/2 full 		 */
} uart_tx_trigger_t;

/**
 * UART Device RX Trigger (RT) Types Enum
 * This is used to select the trigger level
 * in the receiver FIFO at which the Received Data Available
 * Interrupt will be generated.
 */
typedef enum
{
	UART_RX_ONE_CHAR_IN_FIFO   = 0,	    /* 1 character in FIFO   */
	UART_RX_FIFO_QUARTER_FULL  = 1,	    /* FIFO 1/4 Full		 */
	UART_RX_FIFO_HALF_FULL     = 2,	    /* FIFO 1/2 full		 */
	UART_RX_FIFO_TWO_LESS_FULL = 3	    /* FIFO 2 less than full */
} uart_rx_trigger_t;

typedef enum
{
	UART_CLK_SOURCE_0 = 0,                  /* UART 38.4Mhz Clock Source */
	UART_CLK_SOURCE_1 = 1,                  /* UART 100Mhz  Clock Source */
	UART_CLK_SOURCE_2 = 2                  /* UART 20Mhz   Clock Source (fpga) */
} uart_clk_source_t;

/* UART register set */
typedef struct uart_reg_set
{
	volatile uint32_t rbr_thr_dll;      /* receive buffer register         	     (0x00)         */
                                        /* transmit holding register      	     (0x00)         */
                                        /* divisor latch low                     (0x00)         */

    volatile uint32_t ier_dlh;          /* interrupt enable register             (0x04)         */
                                        /* divisor latch high                    (0x04)         */

    volatile uint32_t iir_fcr;          /* interrupt identity register           (0x08)         */
                                        /* FIFO control register                 (0x08)         */

    /* control and status register */
    volatile uint32_t lcr;              /* line control register                 (0x0c)         */
    volatile uint32_t mcr;              /* modem control register                (0x10)         */
    volatile uint32_t lsr;              /* line status register                  (0x14)         */
    volatile uint32_t msr;              /* modem status register                 (0x18)         */

    volatile uint32_t scr;              /* scratchpad register                   (0x1c)         */
    volatile uint32_t lpdll;            /* low power divisor latch (low)         (0x20)         */
    volatile uint32_t lpdlh;            /* low power divisor latch (high)        (0x24)         */
    volatile uint32_t reserved1[2];     /* reserved                              (0x28-0x2c)    */

    volatile uint32_t srbr_sthr[16];    /* shadow receive buffer register     	 (0x30-         */
                                        /* shadow transmit holding register       0x6c)         */
    /* FIFO registers */
    volatile uint32_t far;              /* FIFO access register                  (0x70)         */
    volatile uint32_t tfr;              /* transmit FIFO read register           (0x74)         */
    volatile uint32_t rfw;              /* receiver FIFO write register          (0x78)         */
    volatile uint32_t usr;              /* uart status register                  (0x7c)         */
    volatile uint32_t tfl;              /* transmit FIFO level register          (0x80)         */
    volatile uint32_t rfl;              /* receive FIFO level register           (0x84)         */

    /* shadow registers */
    volatile uint32_t srr;              /* software reset register               (0x88)         */
	volatile uint32_t srts;             /* shadow request to send                (0x8c)         */
	volatile uint32_t sbcr;             /* shadow break control                  (0x90)         */
	volatile uint32_t sdmam;            /* shadow dma mode register              (0x94)         */
	volatile uint32_t sfe;              /* shadow FIFO enable register           (0x98)         */
	volatile uint32_t srt;              /* shadow receiver trigger register      (0x9c)         */
	volatile uint32_t stet;             /* Shadow TX empty trigger register      (0xa0)         */

	volatile uint32_t htx;              /* halt TX                               (0xa4)         */
	volatile uint32_t dmasa;            /* dma software acknowledge register     (0xa8)         */

    //volatile uint32_t reserved2[18];  /* reserved                          	 (0xac-0xf0) 	*/
    //volatile uint32_t reserved2[5];	/* reserved                        		 (0xac-0xbc) 	*/

	/* RS485 control registers */
	volatile uint32_t tcr;				/* transceiver control register 		 (0xac)		    */
	volatile uint32_t de_en;			/* driver   output enable register 		 (0xb0)		    */
	volatile uint32_t re_en;			/* receiver output enable register 		 (0xb4)		    */
	volatile uint32_t det;				/* driver output enable timing register  (0xb8)		    */
	volatile uint32_t tat;				/* turn-around timing register 			 (0xbc)		    */

    volatile uint32_t dlf;				/* divisor latch fraction register 		 (0xc0)		    */
    volatile uint32_t reserved3[12];	/* reserved                              (0xc4-0xf0) 	*/

    /* component registers */
    volatile uint32_t cpr;              /* component parameter register          (0xf4)         */
    volatile uint32_t ucv;              /* UART component version                (0xf8)         */
    volatile uint32_t ctr;              /* component type register               (0xfc)         */
} uart_reg_set_t;

/* UART Transfer Information (Run-Time) */
typedef struct uart_transfer_info
{
  uint8_t  *tx_buf;        				/* Pointer to out data buffer 					*/
  uint32_t  tx_total_num;  				/* Total number of data to be send 				*/
  uint32_t  tx_curr_cnt;   				/* Current number of data sent from total num  	*/
  uint8_t  *rx_buf;        				/* Pointer to in data buffer 					*/
  uint32_t  rx_total_num;  				/* Total number of data to be received 			*/
  uint32_t  rx_curr_cnt;   				/* Number of data received 						*/
  uint8_t   send_busy;     				/* Send active flag 							*/
} uart_transfer_info_t;

/* UART Receive Status */
typedef struct uart_rx_status
{
  uint8_t rx_busy;                    	/* Receiver busy flag 																*/
  uint8_t rx_overflow;                	/* Receive data overflow detected (cleared on start of next receive operation) 	    */
  uint8_t rx_break;                   	/* Break detected on receive (cleared on start of next receive operation) 			*/
  uint8_t rx_framing_error;           	/* Framing error detected on receive (cleared on start of next receive operation) 	*/
  uint8_t rx_parity_error;            	/* Parity error detected on receive (cleared on start of next receive operation) 	*/
} uart_rx_status_t;

/* UART Information (Run-Time) */
/* Structure to save UART settings and statuses */
typedef struct uart_info
{
  ARM_USART_SignalEvent_t cb_event;      /* Event callback 				 */
  uart_rx_status_t        rx_status;     /* Receive status flags 		 */
  uart_transfer_info_t    transfer;      /* Transfer information 		 */
  uint8_t                 flags;         /* UART driver flags 			 */
  uint32_t                baudrate;      /* Baudrate 					 */
  uint32_t 				  int_status;	 /*	interrupt status for uart 	 */
  uint32_t 				  tx_fifo_length;/* transmit fifo length		 */
  uint32_t 				  rx_fifo_length;/* receive  fifo length		 */
} uart_info_t;

/* UART Configuration definitions */
/* @brief Structure to save contexts for a UART configurations */
typedef struct uart_config_info
{
	uart_clk_source_t clk_source;           /* UART clock source 		  */
	uart_rx_trigger_t rx_fifo_trg_lvl;      /* UART Rx FIFO Trigger Level */
	uart_tx_trigger_t tx_fifo_trg_lvl;      /* UART Tx FIFO Trigger Level */
} uart_config_info_t;


#if RS485_SUPPORT
/**
	UART RS485 transfer modes
*/
typedef enum
{
  UART_RS485_FULL_DULPLX_MODE = 0,	     			/* RS485 full duplex mode 					*/
  UART_RS485_SW_CONTROL_HALF_DULPLX_MODE = 1,     	/* RS485 software control half duplex mode 	*/
  UART_RS485_HW_CONTROL_HALF_DULPLX_MODE = 2,    	/* RS485 hardware control half duplex mode 	*/
} uart_rs485_transfer_mode_t;

/* UART Configuration definitions for RS485. */
/* @brief Structure to save contexts for a UART configurations */
typedef struct uart_config_rs485_info
{
	uart_rs485_transfer_mode_t	rs485_transfer_mode; /* uart rs485 transfer mode. 												   			*/
	uint8_t	 rs485_de_assertion_time_8bit;           /* uart RS485 Driver Enable DE Assertion Time (8-bit).						   			*/
	uint8_t	 rs485_de_deassertion_time_8bit;         /* uart RS485 Driver Enable DE De-Assertion Time (8-bit). 					   			*/
	uint16_t rs485_de_to_re_turn_around_time_16bit;  /* uart RS485 Turn Around Time TAT for Driver Enable DE to Receive Enable RE (16-bit).	*/
	uint16_t rs485_re_to_de_turn_around_time_16bit;  /* uart RS485 Turn Around Time TAT for Receive Enable RE to Driver Enable DE (16-bit).	*/
} uart_config_rs485_info_t;

#endif //END RS485_SUPORT

/* UART Resources definitions */
/* @brief Structure to save contexts for a UART channel */
typedef struct uart_resources
{
	uint32_t 	 reg_base;					/* uart register base address 			 */
 	uint32_t 	 clk;  	   					/* system clock 			 			 */
 	uart_config_info_t *cfg;				/* uart configuration information.		 */
 	uart_info_t *info;	   					/* uart device information 				 */
 	uint32_t	 irq_num;  					/* uart interrupt vector number	 		 */
	uint32_t	 irq_priority; 				/* uart interrupt priority 				 */
	uint8_t      instance_num;          	/* uart instance number					 */

#if RS485_SUPPORT
	uart_config_rs485_info_t *rs485_cfg; 	/* uart rs485 configuration information. */
#endif

} uart_resources_t;


/* function declarations */
extern void 		     uart_initialize (uart_resources_t	*uart);
extern void 		     uart_uninitialize (uart_resources_t	*uart);
extern int32_t 		     uart_set_asynchronous_mode ( uart_resources_t *uart, uint32_t control, uint32_t arg);

extern int32_t 		     uart_send_max_data_polling (uart_resources_t *uart, const void *data, uint32_t len);
extern int32_t 		     uart_receive_max_data_polling (uart_resources_t *uart, void *data, uint32_t len);

extern int32_t 		  	 uart_set_rx_trigger (uart_resources_t *uart, uart_rx_trigger_t rx_trigger);
extern uart_rx_trigger_t uart_get_rx_trigger (uart_resources_t *uart);

extern int32_t 		  	 uart_set_tx_trigger (uart_resources_t *uart, uart_tx_trigger_t tx_trigger);
extern uart_tx_trigger_t uart_get_tx_trigger (uart_resources_t *uart);

extern int32_t 		     uart_enable_send_interrupt (uart_resources_t *uart, const void *data, uint32_t len);
extern int32_t 		   	 uart_enable_receive_interrupt (uart_resources_t *uart, const void *data, uint32_t len);
extern void 		     uart_irq_handler (uart_resources_t *uart);

extern int32_t 		  	 uart_get_tx_fifo_available_count (uart_resources_t *uart);
extern int32_t 		  	 uart_get_rx_fifo_available_count(uart_resources_t *uart);

extern void 		  	 uart_abort_tx (uart_resources_t *uart);
extern void 		 	 uart_abort_rx (uart_resources_t *uart);

extern void 		   	 uart_set_break_control (uart_resources_t *uart);
extern void			   	 uart_clear_break_control (uart_resources_t *uart);


/* ======================================= RS485 functions. ================================================= */

#if RS485_SUPPORT
extern int32_t 			 uart_enable_rs485  (uart_resources_t *uart);
extern void 			 uart_disable_rs485 (uart_resources_t *uart);

extern int32_t 			 			 uart_set_rs485_transfer_mode (uart_resources_t *uart, uart_rs485_transfer_mode_t mode);
extern uart_rs485_transfer_mode_t 	 uart_get_rs485_transfer_mode (uart_resources_t *uart);

extern int32_t 			 uart_set_rs485_de_assertion_time	(uart_resources_t *uart, uint32_t assertion_time);
extern int32_t 			 uart_get_rs485_de_assertion_time	(uart_resources_t *uart);

extern int32_t 			 uart_set_rs485_de_deassertion_time	(uart_resources_t *uart, uint32_t deassertion_time);
extern int32_t 			 uart_get_rs485_de_deassertion_time	(uart_resources_t *uart);

extern int32_t 			 uart_set_rs485_de_to_re_turn_around_time (uart_resources_t *uart, uint32_t de_to_re_time);
extern int32_t 			 uart_get_rs485_de_to_re_turn_around_time (uart_resources_t *uart);

extern int32_t 			 uart_set_rs485_re_to_de_turn_around_time (uart_resources_t *uart, uint32_t re_to_de_time);
extern int32_t 			 uart_get_rs485_re_to_de_turn_around_time (uart_resources_t *uart);

extern int32_t 			 uart_set_rs485_de_en(uart_resources_t *uart, uint32_t arg);
extern int32_t 			 uart_get_rs485_de_en(uart_resources_t *uart);

extern int32_t 			 uart_set_rs485_re_en(uart_resources_t *uart, uint32_t arg);
extern int32_t 			 uart_get_rs485_re_en(uart_resources_t *uart);

#endif /* End of RS485 functions */


#ifdef __cplusplus
}
#endif

#endif /* UART_LL_DRV_H__ */

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
