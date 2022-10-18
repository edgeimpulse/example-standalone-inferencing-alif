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

#include "hal/Driver_PINMUX_AND_PINPAD.h"
#include "hal/drv_i2c_bitbang.h"

/* Half of I2C Cycle Delay */
#define i2c_delay() PMU_delay_loop_us(2)
#define i2c_half_delay() PMU_delay_loop_us(1)
#define I2C_TIMEOUT_MS	10

volatile int started = 0;
extern volatile uint32_t ms_ticks;

void i2c_init () {

	/* Required for I2C Bit-Bang to work */
	HW_REG32(CFGMST0_BASE,0x0) |= (1 << 0) | (1 << 4);

	i2c_SDA_High(); /* Setting the IO level before PINMUX prevents glitches */
	i2c_SCL_High();
	i2c_SDA_Input();	/* PAD in open-drain mode uses GPIO input */
	i2c_SCL_Input();

	PINMUX_Config(PINMUX_SDA_PORT_NUMBER, PINMUX_SDA_PIN_NUMBER, 0);
	PINPAD_Config(PINMUX_SDA_PORT_NUMBER, PINMUX_SDA_PIN_NUMBER, \
						PAD_FUNCTION_READ_ENABLE |
						PAD_FUNCTION_DRIVER_OPEN_DRAIN);

	PINMUX_Config(PINMUX_SCL_PORT_NUMBER, PINMUX_SCL_PIN_NUMBER, 0);
	PINPAD_Config(PINMUX_SCL_PORT_NUMBER, PINMUX_SCL_PIN_NUMBER, \
						PAD_FUNCTION_READ_ENABLE |
						PAD_FUNCTION_DRIVER_OPEN_DRAIN);

	i2c_delay();
	i2c_delay();
	i2c_delay();
	i2c_delay();
}

int i2c_start () {
	uint32_t ms_wait;

	if (started)  {
		/* pre-conditions */
		i2c_SDA_High();
		i2c_delay();
		i2c_SCL_High();

		/* timeout if pre-conditions are not met */
		ms_wait = ms_ticks + I2C_TIMEOUT_MS;
		while (1) {
			/* loop as long as SCL is Low */
			if (i2c_SCL_Status() != 0) break;
			if (ms_wait == ms_ticks) {
				return -1;
			}
		}

		i2c_delay();
		i2c_delay();
	}

	/* start condition */
	i2c_SDA_Low();
	i2c_delay();
	i2c_SCL_Low();
	i2c_delay();

	started = 1;
	return 0;
}

int i2c_stop () {
	uint32_t ms_wait;

	i2c_SDA_Low();
	i2c_half_delay();
	i2c_SCL_High();

	/* timeout if pre-conditions are not met */
	ms_wait = ms_ticks + I2C_TIMEOUT_MS;
	while (1) {
		/* loop as long as SCL is Low */
		if (i2c_SCL_Status() != 0) break;
		if (ms_wait == ms_ticks) {
			return -1;
		}
	}

	/* stop condition */
	i2c_half_delay();
	i2c_SDA_High();
	i2c_half_delay();

	/* extra delay */
	i2c_delay();
	i2c_delay();

	started = 0;
	return 0;
}

int i2c_write_byte (uint8_t data_byte) {
	for(int i = 8; i; i--) {
		if ((data_byte & 0x80) != 0)
			i2c_SDA_High();
		else
			i2c_SDA_Low();

		data_byte <<= 1;
		i2c_half_delay();
		i2c_SCL_High();
		i2c_delay();
		i2c_SCL_Low();
		i2c_half_delay();
	}
	i2c_half_delay();
	i2c_SDA_High();
	i2c_SCL_High();
	i2c_half_delay();

	int nack = i2c_SDA_Status() == 0 ? 0 : 1;    // return 1 for NACK

	i2c_half_delay();
	i2c_SCL_Low();
	i2c_SDA_Low();
	i2c_delay();

	return nack;
}

uint8_t i2c_read_byte (int ack) {
	uint8_t rcv_data = 0;

	i2c_SDA_High();
	for (int i = 0; i < 8; i++) {
		rcv_data <<= 1;
		do {
			i2c_SCL_High();
		} while(i2c_SCL_Status() == 0);
		i2c_half_delay();

		if (i2c_SDA_Status() != 0)
			rcv_data |= 1;

		i2c_half_delay();
		i2c_SCL_Low();
		i2c_delay();
	}

	if (ack) {
		i2c_SDA_Low();
	}
	else {
		i2c_SDA_High();
	}
	i2c_SCL_High();
	i2c_delay();

	i2c_SCL_Low();
	i2c_SDA_High();
	i2c_delay();

	return rcv_data;
}

int i2c_master_WriteData (uint8_t addr, const uint8_t *data, uint16_t len) {

	if (i2c_start()) return -1;

	if (i2c_write_byte((addr << 1))) return -1;

	for (int i = 0; i < len; i++) {
		if (i2c_write_byte(data[i])) return -1;
	}

	i2c_stop();

	return 0;
}

int i2c_master_ReadData (uint8_t addr, uint8_t *data, uint16_t len) {

	if (i2c_start()) return -1;

	if (i2c_write_byte((addr << 1) | 1)) return -1;

	len--;
	for (int i = 0; i < len; i++) {
		data[i] = i2c_read_byte(1);
	}
	data[len] = i2c_read_byte(0);

	i2c_stop();

	return 0;
}
