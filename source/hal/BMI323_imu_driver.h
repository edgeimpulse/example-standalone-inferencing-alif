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

#include <stdint.h>

#define i2c_addr_68                 0b1101000
#define i2c_addr_69                 0b1101001

#define BMI323_I2C_WRITE_ADDR_BYTES 1
#define BMI323_I2C_READ_EXTRA_BYTES 2
#define BMI323_I2C_DELAY_US         2

#define BMI323_STATUS_ACCRDY        0x80
#define BMI323_STATUS_GYRRDY        0x40
#define BMI323_STATUS_TMPRDY        0x20
#define BMI323_CMD_RESET            0xDEAF

typedef enum {
BMI323_CHIP_ID_REG,
BMI323_ERROR_REG,
BMI323_STATUS_REG,

BMI323_ACC_X_REG,
BMI323_ACC_Y_REG,
BMI323_ACC_Z_REG,

BMI323_GYR_X_REG,
BMI323_GYR_Y_REG,
BMI323_GYR_Z_REG,

BMI323_TEMP_DATA_REG,
BMI323_TIME_0_REG,
BMI323_TIME_1_REG,
BMI323_SAT_FLAGS,
BMI323_INT_STATUS_INT1,
BMI323_INT_STATUS_INT2,
BMI323_INT_STATUS_IBI,
BMI323_FEATURE_IO0,
BMI323_FEATURE_IO1,
BMI323_FEATURE_IO2,
BMI323_FEATURE_IO3,
BMI323_FEATURE_IO_STATUS,
BMI323_FIFO_FILL_LEVEL,
BMI323_FIFO_DATA,

BMI323_ACC_CONF = 0x20,
BMI323_GYR_CONF,

BMI323_ACC_CONF_ALT = 0x28,
BMI323_GYR_CONF_ALT,
BMI323_ALT_CONF,
BMI323_ALT_STATUS,
BMI323_FIFO_WATERMARK = 0x35,
BMI323_FIFO_CONF,
BMI323_FIFO_CTRL,
BMI323_IO_INT_CTRL,

BMI323_INT_CONF,
BMI323_INT_MAP1,
BMI323_INT_MAP2,

BMI323_FEATURE_CTRL = 0x40,
BMI323_FEATURE_DATA_ADDR,
BMI323_FEATURE_DATA_TX,

BMI323_CMD_REG = 0x7E,
} BMI323_REG_MAP_t;

typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t z;
} xyz_internal_s;

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} xyz_accel_s;


int32_t IMU_Init(uint8_t i2c_addr);
void    IMU_ACC_Get(xyz_accel_s *);
void    IMU_GYR_Get(xyz_accel_s *);
int32_t IMU_Deinit(uint8_t i2c_addr);
