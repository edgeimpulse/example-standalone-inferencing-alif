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

#include <string.h>
#include <stdbool.h>

#include "hal/Driver_GPIO.h"
#include "hal/Driver_PINMUX_AND_PINPAD.h"

#include "BMI323_imu_driver.h"
#include "system_utils.h"

#include <stdio.h>
#define DEBUG_PRINTF(...) (0)
//#define DEBUG_PRINTF printf

#define USE_I2C_GPIO_BITBANG

volatile uint8_t touch_event_gpio;

uint8_t i2c_addr = i2c_addr_68;

#ifdef USE_I2C_GPIO_BITBANG
#include "drv_i2c_bitbang.h"
#else
#include "Driver_I3C.h"
extern ARM_DRIVER_I3C Driver_I3C0;
static ARM_DRIVER_I3C *I3Cdrv = &Driver_I3C0;
volatile uint8_t touch_event_i2c;

#define IMU_I2C_TIMEOUT  10
extern volatile uint32_t ms_ticks;
#endif

extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(2);
static ARM_DRIVER_GPIO *gpio2 = &ARM_Driver_GPIO_(2);

static int32_t IMU_Write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t num_bytes)
{
	uint8_t  tx_data[24] = {0};
	uint8_t  i;
	int32_t  ret;

	tx_data[0] = reg_addr;

	if (num_bytes) {
		for(i = 0; i < num_bytes; i++) {
			tx_data[i + BMI323_I2C_WRITE_ADDR_BYTES] = reg_data[i];
		}
	}

	num_bytes += BMI323_I2C_WRITE_ADDR_BYTES;

#ifdef USE_I2C_GPIO_BITBANG
	ret = i2c_master_WriteData(dev_addr, tx_data, num_bytes);
	if (ret == 0) {
		/* TX Success: Got ACK from slave */
		DEBUG_PRINTF(">> I2C Master Transmit Success: Got ACK from slave addr: 0x%X\n", dev_addr);
		DEBUG_PRINTF("\twrote %d bytes to reg addr: 0x%X\n", num_bytes, reg_addr);
		DEBUG_PRINTF("\tTransmited Data to slave: 0x");
		for (i = 0; i < num_bytes; i++) {
			DEBUG_PRINTF("%02X", tx_data[i]);
		}
		DEBUG_PRINTF("\n");
	} else {
		/* TX Error: Got NACK from slave */
		DEBUG_PRINTF(">> I2C Master Transmit Error: Got NACK from slave addr: 0x%X\n", dev_addr);
	}
#else
	touch_event_i2c = 0;
	do {
		ret = I3Cdrv->MasterTransmit(dev_addr, tx_data, num_bytes + BMI323_I2C_WRITE_ADDR_BYTES);
	} while(ret == ARM_DRIVER_ERROR_BUSY);

	if(ret != ARM_DRIVER_OK) {
		DEBUG_PRINTF("MasterTransmit: %d\r\n", ret);
		return ret;
	}

	uint32_t ms_wait = ms_ticks + IMU_I2C_TIMEOUT;
	while (1) {
		if (touch_event_i2c) break;
		if (ms_wait == ms_ticks) {
			DEBUG_PRINTF("Error: I2C Master Transmit timeout after %dms\n", IMU_I2C_TIMEOUT);
			return -1;
		}
	}

	if (touch_event_i2c & ARM_I3C_EVENT_TRANSFER_DONE) {
		/* TX Success: Got ACK from slave */
		DEBUG_PRINTF(">> I2C Master Transmit Success: Got ACK from slave addr: 0x%X\n", dev_addr);
		DEBUG_PRINTF("\twrote %d bytes to reg addr: 0x%X\n", num_bytes, reg_addr);
		DEBUG_PRINTF("\tTransmited Data to slave: 0x");
		for (i = 0; i < (num_bytes + BMI323_I2C_WRITE_ADDR_BYTES); i++) {
			DEBUG_PRINTF("%02X", tx_data[i]);
		}
		DEBUG_PRINTF("\n");
	}

	if (touch_event_i2c & ARM_I3C_EVENT_TRANSFER_ERROR) {
		/* TX Error: Got NACK from slave */
		DEBUG_PRINTF(">> I2C Master Transmit Error: Got NACK from slave addr: 0x%X\n", dev_addr);
		return -1;
	}
#endif

	return 0;
}

static int32_t IMU_Read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t num_bytes)
{
	uint8_t  rx_data[24] = {0};
	uint8_t  i;
	int32_t  ret;

	if (IMU_Write(dev_addr, reg_addr, 0, 0)) return -1;

	num_bytes += BMI323_I2C_READ_EXTRA_BYTES;

#ifdef USE_I2C_GPIO_BITBANG
	ret = i2c_master_ReadData(dev_addr, rx_data, num_bytes);
	if (ret == 0) {
		/* RX Success: Got ACK from slave */
		DEBUG_PRINTF(">> I2C Master Receive Success: Got ACK from slave addr: 0x%X\n", dev_addr);
		DEBUG_PRINTF("\tread %d bytes from reg addr: 0x%X\n", num_bytes, reg_addr);
		DEBUG_PRINTF("\tReceived Data from slave: 0x");
		for (i = 0; i < num_bytes; i++) {
			DEBUG_PRINTF("%02X", rx_data[i]);
		}
		DEBUG_PRINTF("\n");
	} else {
		/* RX Error: Got NACK from slave */
		DEBUG_PRINTF(">> I2C Master Receive Error: Got NACK from slave addr: 0x%X\n", dev_addr);
	}
#else
	touch_event_i2c = 0;
	ret = I3Cdrv->MasterReceive(dev_addr, rx_data, num_bytes);

	if(ret != ARM_DRIVER_OK) {
		DEBUG_PRINTF("MasterReceive: %d\r\n", ret);
		return -12;
	}

	uint32_t ms_wait = ms_ticks + IMU_I2C_TIMEOUT;
	while (1) {
		if (touch_event_i2c) break;
		if (ms_wait == ms_ticks) {
			DEBUG_PRINTF("Error: I2C Master Receive timeout after %dms\n", IMU_I2C_TIMEOUT);
			return -1;
		}
	}

	if (touch_event_i2c & ARM_I3C_EVENT_TRANSFER_DONE) {
		/* RX Success: Got ACK from slave */
		DEBUG_PRINTF(">> I2C Master Receive Success: Got ACK from slave addr: 0x%X\n", dev_addr);
		DEBUG_PRINTF("\tread %d bytes from reg addr: 0x%X\n", num_bytes, reg_addr);
		DEBUG_PRINTF("\tReceived Data from slave: 0x");
		for (i = 0; i < num_bytes; i++) {
			DEBUG_PRINTF("%02X", rx_data[i]);
		}
		DEBUG_PRINTF("\n");
	}

	if (touch_event_i2c & ARM_I3C_EVENT_TRANSFER_ERROR) {
		DEBUG_PRINTF(">> I2C Master Receive Error: Got NACK from slave addr: 0x%X\n", dev_addr);
		return -1;
	}
#endif

	num_bytes -= BMI323_I2C_READ_EXTRA_BYTES;
	memcpy(reg_data, &rx_data[BMI323_I2C_READ_EXTRA_BYTES], num_bytes);

	return 0;
}

static void IMU_IntEnable0(bool enable)
{
	uint32_t arg = ARM_GPIO_IRQ_POLARITY_LOW | ARM_GPIO_IRQ_EDGE_SENSITIVE_SINGLE | ARM_GPIO_IRQ_SENSITIVE_EDGE;

	enable ?
			gpio2->Control(PIN_NUMBER_24, ARM_GPIO_ENABLE_INTERRUPT, &arg):
			gpio2->Control(PIN_NUMBER_24, ARM_GPIO_DISABLE_INTERRUPT, NULL);
}

static void IMU_IntEnable1(bool enable)
{
	uint32_t arg = ARM_GPIO_IRQ_POLARITY_LOW | ARM_GPIO_IRQ_EDGE_SENSITIVE_SINGLE | ARM_GPIO_IRQ_SENSITIVE_EDGE;

	enable ?
			gpio2->Control(PIN_NUMBER_25, ARM_GPIO_ENABLE_INTERRUPT, &arg):
			gpio2->Control(PIN_NUMBER_25, ARM_GPIO_DISABLE_INTERRUPT, NULL);
}

#ifndef USE_I2C_GPIO_BITBANG
static void IMU_I2C_CB(uint32_t event)
{
	touch_event_i2c |= event;
}
#endif

static void IMU_GPIO_CB0(uint32_t event)
{
	IMU_IntEnable0(false);
	touch_event_gpio = 1;
}

static void IMU_GPIO_CB1(uint32_t event)
{
	IMU_IntEnable1(false);
	touch_event_gpio = 1;
}

int32_t IMU_Init(uint8_t dev_addr)
{
	uint32_t config_gpio =
			PAD_FUNCTION_READ_ENABLE |
			PAD_FUNCTION_SCHMITT_TRIGGER_ENABLE |
			PAD_FUNCTION_DRIVER_DISABLE_STATE_WITH_HIGH_Z;

	/* gpio2 config */
	PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_24, PINMUX_ALTERNATE_FUNCTION_0);
	PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_25, PINMUX_ALTERNATE_FUNCTION_0);
	PINPAD_Config(PORT_NUMBER_2, PIN_NUMBER_24, config_gpio);
	PINPAD_Config(PORT_NUMBER_2, PIN_NUMBER_25, config_gpio);

#ifdef USE_I2C_GPIO_BITBANG
	i2c_init();
#else
	/* i3c */
	uint32_t config_i3c =
			PAD_FUNCTION_READ_ENABLE |
			PAD_FUNCTION_DRIVER_DISABLE_STATE_WITH_PULL_UP |
			PAD_FUNCTION_DRIVER_OPEN_DRAIN;

	PINMUX_Config(PORT_NUMBER_3, PIN_NUMBER_8, PINMUX_ALTERNATE_FUNCTION_3);	// P3_8: SDA (mux mode 3)
	PINPAD_Config(PORT_NUMBER_3, PIN_NUMBER_8, config_i3c);
	PINMUX_Config(PORT_NUMBER_3, PIN_NUMBER_9, PINMUX_ALTERNATE_FUNCTION_4);	// P3_9: SCL (mux mode 4)
	PINPAD_Config(PORT_NUMBER_3, PIN_NUMBER_9, config_i3c);

	I3Cdrv->Initialize(TOUCH_I2C_CB);
	I3Cdrv->PowerControl(ARM_POWER_FULL);
	I3Cdrv->Control(
			I3C_MASTER_SET_BUS_MODE,
			I3C_BUS_MODE_MIXED_FAST_I2C_FM_SPEED_400_KBPS);

	I3Cdrv->AttachI2Cdev(dev_addr);
#endif

	uint8_t transfer_buffer[24] = {0};

	/* Validate that we are talking to the BMI323 */
	IMU_Read(dev_addr, BMI323_CHIP_ID_REG, transfer_buffer, 1);
	if (transfer_buffer[0] != 0x43) {
		DEBUG_PRINTF("[ERROR] Value of BMI323_CHIP_ID_REG did not match.\n");
		return -1;
	}

	/* Initialize the ACC */
	/* Low-power mode */
	/*
	transfer_buffer[0] = 0x27;
	transfer_buffer[1] = 0x31;
	*/
	/* Normal mode */
	transfer_buffer[0] = 0x27;
	transfer_buffer[1] = 0x43;
	/* High-performance mode */
	/*
	transfer_buffer[0] = 0xA9;
	transfer_buffer[1] = 0x71;
	*/
	IMU_Write(dev_addr, BMI323_ACC_CONF, transfer_buffer, 2);

	/* Initialize the GYR */
	/* Normal mode */
	/*
	transfer_buffer[0] = 0x4B;
	transfer_buffer[1] = 0x40;
	*/
	/* High-performance mode */
	/*
	transfer_buffer[0] = 0xC9;
	transfer_buffer[1] = 0x70;
	*/
	/*
	IMU_Write(dev_addr, BMI323_GYR_CONF, transfer_buffer, 2);
	*/

	/* Check for ACC and GYR configuration error */
	IMU_Read(dev_addr, BMI323_ERROR_REG, transfer_buffer, 1);
	if (transfer_buffer[0] != 0) {
		DEBUG_PRINTF("[ERROR] Value of BMI323_ERROR_REG indicates an error.\n");
		DEBUG_PRINTF("\t\tvalue: %02X\n", transfer_buffer[0]);
		return -1;
	}

	/* Initialization is done */

	return 0;

	/* Enable IMU interrupts */

	/* BMI323_IO_INT_CTRL */
	/* INT1 pin is enabled, active-low, open-drain
	 * INT2 pin is enabled, active-low, open-drain */
	transfer_buffer[0] = 0x06;
	transfer_buffer[1] = 0x06;
	IMU_Write(dev_addr, BMI323_IO_INT_CTRL, transfer_buffer, 2);

	/* BMI323_INT_MAP1 */
	transfer_buffer[0] = 0x00;
	transfer_buffer[1] = 0x00;
	IMU_Write(dev_addr, BMI323_INT_MAP1, transfer_buffer, 2);

	/* BMI323_INT_MAP2 */
	/* ACC data ready on P2_24 / IMU_GPIO_CB0
	 * GYR data ready on P2_25 / IMU_GPIO_CB1 */
	transfer_buffer[0] = 0x00;
	transfer_buffer[1] = 0x09;
	IMU_Write(dev_addr, BMI323_INT_MAP2, transfer_buffer, 2);

	gpio2->Initialize(PIN_NUMBER_24, IMU_GPIO_CB0);
	gpio2->Initialize(PIN_NUMBER_25, IMU_GPIO_CB1);
	IMU_IntEnable0(true);
	IMU_IntEnable1(true);

	return 0;
}

void IMU_ACC_Get(xyz_accel_s *get)
{
	uint8_t data[6];
	xyz_internal_s motion;

	IMU_Read(i2c_addr, BMI323_ACC_X_REG, data, 6);

	motion.x = data[0] | ((uint16_t)data[1] << 8);
	motion.y = data[2] | ((uint16_t)data[3] << 8);
	motion.z = data[4] | ((uint16_t)data[5] << 8);

	memcpy(get, &motion, sizeof(xyz_internal_s));

	return;
}

void IMU_GYR_Get(xyz_accel_s *get)
{
	uint8_t data[6];
	xyz_internal_s motion;

	IMU_Read(i2c_addr, BMI323_GYR_X_REG, data, 6);

	motion.x = data[0] | ((uint16_t)data[1] << 8);
	motion.y = data[2] | ((uint16_t)data[3] << 8);
	motion.z = data[4] | ((uint16_t)data[5] << 8);

	memcpy(get, &motion, sizeof(xyz_internal_s));

	return;
}

int32_t IMU_Deinit(uint8_t dev_addr)
{
#ifndef USE_I2C_GPIO_BITBANG
	I3Cdrv->Detachdev(dev_addr);

	I3Cdrv->PowerControl(ARM_POWER_OFF);
	I3Cdrv->Uninitialize();
#endif
	return 0;
}

