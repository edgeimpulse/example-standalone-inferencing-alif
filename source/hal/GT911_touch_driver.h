#include <stdint.h>

//#define ALLOW_184B_I2C

#define i2c_addr_5D 					0b1011101

#define GT911_COMMAND_REG        		(uint16_t)0x8040
#define GT911_PRODUCT_ID       	        (uint16_t)0x8140
#define GT911_TOUCH_STATUS       		(uint16_t)0x814E

#define GT911_CONFIG_START             	(uint16_t)0x8047
#define GT911_CONFIG_CHECKSUM       	(uint16_t)0x80FF

#define GT911_FIRST_TOUCHPOINT_DATA     (uint16_t)0x8150
#define GT911_FIFTH_TOUCHPOINT_DATA     (uint16_t)0x8170

#define X_RESOLUTION					(uint16_t)480
#define Y_RESOLUTION					(uint16_t)800

typedef struct {
	int16_t x;
	int16_t y;
} xypair_t;

int32_t TOUCH_Init(uint8_t i2c_addr);
void TOUCH_Loop(uint8_t i2c_addr);
int32_t TOUCH_GetPointer(uint8_t i2c_addr, xypair_t * xypair);
int32_t TOUCH_Deinit(uint8_t i2c_addr);
