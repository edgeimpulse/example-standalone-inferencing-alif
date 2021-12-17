#ifndef _MT9M114_RAW_H_
#define _MT9M114_RAW_H_

#define VGA  (1)     //648x400
#define QCIF (0)     //536x440


/* Sysctl registers */
#define MT9M114_CHIP_ID								 0x0000
#define MT9M114_COMMAND_REGISTER			         0x0080
#define MT9M114_COMMAND_REGISTER_APPLY_PATCH		(1 << 0)
#define MT9M114_COMMAND_REGISTER_SET_STATE		    (1 << 1)
#define MT9M114_COMMAND_REGISTER_REFRESH		    (1 << 2)
#define MT9M114_COMMAND_REGISTER_WAIT_FOR_EVENT		(1 << 3)
#define MT9M114_COMMAND_REGISTER_OK			        (1 << 15)


/* Camera Control registers */
#define MT9M114_CAM_OUTPUT_FORMAT			0xc86c
#define MT9M114_CAM_OUTPUT_FORMAT_FORMAT_BAYER            (2 << 8)
#define MT9M114_CAM_OUTPUT_FORMAT_BAYER_FORMAT_RAWR10        (0 << 10)
#define MT9M114_CAM_OUTPUT_FORMAT_BAYER_FORMAT_PROCESSED8    (3 << 10)

/* System Manager registers */
#define MT9M114_SYSMGR_NEXT_STATE			0xdc00
#define MT9M114_SYSMGR_CURRENT_STATE		0xdc01
#define MT9M114_SYSMGR_CMD_STATUS			0xdc02


/* System States */
#define MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE		0x28

#define MT9M114_SYS_STATE_STREAMING			        0x31
#define MT9M114_SYS_STATE_START_STREAMING		    0x34

#define MT9M114_SYS_STATE_ENTER_SUSPEND			    0x40
#define MT9M114_SYS_STATE_SUSPENDED			        0x41

#define MT9M114_SYS_STATE_ENTER_STANDBY			    0x50
#define MT9M114_SYS_STATE_STANDBY			        0x52

#define MT9M114_SYS_STATE_LEAVE_STANDBY			    0x54

typedef struct
{
	uint16_t                ui16Reg;
	uint16_t                value_size;
	uint32_t                ui32Val;
} mt9m114_reg;

void onsemi_mt9m114_image_capture(void);

//as per wizard tool VGA Binning 640P 5 fps 648x400

#endif // _MT9M114_RAW_H_
