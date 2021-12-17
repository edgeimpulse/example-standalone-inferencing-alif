/* Copyright (c) 2021 ALIF SEMICONDUCTOR

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
 * @file     onsem_mt9m114_bare.c
 * @author   Tanay Rami, Pankaj Pandey
 * @email    tanay@alifsemi.com, ppandey@alifsemi.com
 * @version  V1.0.0
 * @date     12-July-2021
 * @brief
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

/* Includes ----------------------------------------------------------------- */
/* System Includes */

#if 0

#include "stdio.h"
#include "string.h"
#include <stdint.h>
#include <stdlib.h>
#include "bayer.h"
/* Project Includes */

#include "ARMCM55.h"

//todo added
//#include "RTE_Components.h"
//#include CMSIS_device_header

#define CMSIS_device_header "ARMCM55.h"


#include "onsemi_mt9m114_bare.h"

#if 1
const mt9m114_reg mt9m114_qvga_24mhz_pll[] = {

{ 0xC97E, 2, 0x01 }, /* cam_sysctl_pll_enable = 1 */  ////////


	{ 0xC980, 2, 0x0120 }, /* cam_sysctl_pll_divider_m_n = 288 */  /////////////

	{ 0xC982, 2, 0x0700 }, /* cam_sysctl_pll_divider_p = 1792 */   /////////

	{ 0xC984, 2, 0x8000 }, /* cam_port_output_control = 32776 */


	{ 0xC800, 2, 0x0108 }, /* cam_sensor_cfg_y_addr_start = 0 */
	{ 0xC802, 2, 0x01A8 }, /* cam_sensor_cfg_x_addr_start = 0 */
	{ 0xC804, 2, 0x02C5 }, /* cam_sensor_cfg_y_addr_end = 973 */
	{ 0xC806, 2, 0x0365 }, /* cam_sensor_cfg_x_addr_end = 1293 */

	{ 0xC808, 4, 0x2DC6C00 }, /* cam_sensor_cfg_pixclk = 48000000 */ //original //todo removed for internal osc

	{ 0xC80C, 2, 0x0001 }, /* cam_sensor_cfg_row_speed = 1 */


	{ 0xC80E, 2, 0x01C3 }, /* cam_sensor_cfg_fine_integ_min = 219 */


	{ 0xC810, 2, 0x0213 }, /* cam_sensor_cfg_fine_integ_max = 1986 */


	{ 0xC812, 2, 0x0845 }, /* cam_sensor_cfg_frame_length_lines = 766 */


	{ 0xC814, 2, 0x02FE }, /* cam_sensor_cfg_line_length_pck = 2117 */


	{ 0xC816, 2, 0x00E0 }, /* cam_sensor_cfg_fine_correction = 96 */

	{ 0xC818, 2, 0x00DB }, /* cam_sensor_cfg_cpipe_last_row = 483 */

	{ 0xC826, 2, 0x0020 }, /* cam_sensor_cfg_reg_0_data = 32 */
	{ 0xC834, 2, 0x0333 }, /* cam_sensor_control_read_mode = 272 */ //todo H and V flip
	{ 0xC854, 2, 0x0 }, /* cam_crop_window_xoffset = 0 */ //70 //150 //140 //120 //81 //162
	{ 0xC856, 2, 0x0 }, /* cam_crop_window_yoffset = 0 *///50//110 //100 //80 //61 //122
	{ 0xC858, 2, 1280 }, /* cam_crop_window_width = 640 */ //todo original
	{ 0xC85A, 2, 960 }, /* cam_crop_window_height = 480 */
	{ 0xC85C, 1, 0x03 }, /* cam_crop_cropmode = 3 */ //todo tanay

	{ 0xC868, 2, 224 }, /* cam_output_width = 640 */
	{ 0xC86A, 2, 224 }, /* cam_output_height = 480 */
	{ 0xC878, 1, 0x00 }, /* cam_aet_aemode = 0 */

	{ 0xC88C, 2, 0x1D9A }, /* cam_aet_max_frame_rate = 7578 */
	{ 0xC88E, 2, 0x1D9A }, /* cam_aet_min_frame_rate = 7578 */
	{ 0xC914, 2, 0x0000 }, /* cam_stat_awb_clip_window_xstart = 0 */
	{ 0xC916, 2, 0x0000 }, /* cam_stat_awb_clip_window_ystart = 0 */
	{ 0xC918, 2, 0x00D7 }, /* cam_stat_awb_clip_window_xend = 323 */
	{ 0xC91A, 2, 0x00D7 }, /* cam_stat_awb_clip_window_yend = 243 */
	{ 0xC91C, 2, 0x0000 }, /* cam_stat_ae_initial_window_xstart = 0 */
	{ 0xC91E, 2, 0x0000 }, /* cam_stat_ae_initial_window_ystart = 0 */
	{ 0xC920, 2, 0x002A }, /* cam_stat_ae_initial_window_xend = 127 */
	{ 0xC922, 2, 0x002A }, /* cam_stat_ae_initial_window_yend = 95 */
	{ 0xA404, 2, 0x0003 }, /* Adaptive Settings */

};
#endif

//as per wizard tool QCIF bining 5FPS use min frequency 536x440
#if 0
const mt9m114_reg mt9m114_qvga_24mhz_pll[] = {

		{ 0xC97E, 2, 0x01	}, /* cam_sysctl_pll_enable = 1 */  ////////

		{ 0xC980, 2, 0x0110	}, /* cam_sysctl_pll_divider_m_n = 272 */  /////////////
		{ 0xC982, 2, 0x0700	}, /* cam_sysctl_pll_divider_p = 1792 */   /////////

		{ 0xC984, 2, 0x8000	}, /* cam_port_output_control = 32768 */

		{ 0xC800, 2, 0x0030	}, /* cam_sensor_cfg_y_addr_start = 48 */
		{ 0xC802, 2, 0x0070	}, /* cam_sensor_cfg_x_addr_start = 112 */
		{ 0xC804, 2, 0x039D	}, /* cam_sensor_cfg_y_addr_end = 925 */
		{ 0xC806, 2, 0x049D	}, /* cam_sensor_cfg_x_addr_end = 1181 */

		{ 0xC808, 4, 0x16E3600	}, /* cam_sensor_cfg_pixclk = 24000000 */ //original //todo removed for internal osc

		{ 0xC80C, 2, 0x0001	}, /* cam_sensor_cfg_row_speed = 1 */

		{ 0xC80E, 2, 0x01C3	}, /* cam_sensor_cfg_fine_integ_min = 451 */

		{ 0xC810, 2, 0x0409	}, /* cam_sensor_cfg_fine_integ_max = 1033 */

		{ 0xC812, 2, 0x0E78	}, /* cam_sensor_cfg_frame_length_lines = 3704 */

		{ 0xC814, 2, 0x04F4}, /* cam_sensor_cfg_line_length_pck = 1268 */

		{ 0xC816, 2, 0x00E0	}, /* cam_sensor_cfg_fine_correction = 224 */

		{ 0xC818, 2, 0x01B3	}, /* cam_sensor_cfg_cpipe_last_row = 435 */

		{ 0xC826, 2, 0x0020	}, /* cam_sensor_cfg_reg_0_data = 32 */

		{ 0xC834, 2, 0x0333	}, /* cam_sensor_control_read_mode = 819 */ //todo H and V flip

		{ 0xC854, 2, 80	}, //100 /* cam_crop_window_xoffset = 0 */
		{ 0xC856, 2, 60	}, /*//70 cam_crop_window_yoffset = 0 */

		{ 0xC858, 2, 0x0210	}, /* cam_crop_window_width = 528 */ //todo original
		{ 0xC85A, 2, 0x01B0	}, /* cam_crop_window_height = 432 */

		{ 0xC85C, 1, 0x03	}, /* cam_crop_cropmode = 3 */ //todo tanay

		{ 0xC868, 2, 0x00B0	}, /* cam_output_width = 176 */
		{ 0xC86A, 2, 0x0078	}, /* cam_output_height = 120 */

		{ 0xC878, 1, 0x00	}, /* cam_aet_aemode = 0 */

		// 5 fps
		{ 0xC88C, 2, 0x051C	}, /* cam_aet_max_frame_rate = 1308 */
		{ 0xC88E, 2, 0x051C	}, /* cam_aet_min_frame_rate = 1308 */

		{ 0xC914, 2, 0x0000	}, /* cam_stat_awb_clip_window_xstart = 0 */
		{ 0xC916, 2, 0x0000	}, /* cam_stat_awb_clip_window_ystart = 0 */


		{ 0xC918, 2, 0x00AF	}, /* cam_stat_awb_clip_window_xend = 175 */
		{ 0xC91A, 2, 0x0077	}, /* cam_stat_awb_clip_window_yend = 119 */

		{ 0xC91C, 2, 0x0000	}, /* cam_stat_ae_initial_window_xstart = 0 */
		{ 0xC91E, 2, 0x0000	}, /* cam_stat_ae_initial_window_ystart = 0 */

		{ 0xC920, 2, 0x0022	}, /* cam_stat_ae_initial_window_xend = 34 */
		{ 0xC922, 2, 0x0017	}, /* cam_stat_ae_initial_window_yend = 23 */

		//todo tanay
		{ 0xA404, 2, 0x0003	}, /* Adaptive Settings */

};
#endif


/* Test Pattern */
const mt9m114_reg sMT9M114TestModeScript_Walking1s[] =
{
		{0xc84c, 1, 0x02},
		{0xc84d, 1, 0x0A}, //walking 1s 10-bit
};




#define CAMERA_BASE_ADDR		0x49030000
#define I3C0_BASE_ADDR			0x49018000

#define HW_REG_WORD(base,offset) *((volatile unsigned int *) (base + offset))
#define HW_REG_BYTE(base,offset) *((volatile unsigned char *) (base + offset))

#define ENABLE_TEST_PATTERN_WALKING_1s (0)
#define REAL_IMAGE (1)

#define one_fifty_six_KB        0x27200
#define SRC_MEM_ADDRESS 0x62000000   //Cortex55-M TCM 2MB
#define DEST_MEM_ADDRESS 0x63100000  //DSP TCM 2MB


#define PAD_FUNCTION_OUTPUT_DRIVE_STRENGTH_04_MILI_AMPS     (0x20)
#define TIFF_HDR_NUM_ENTRY 8
#define TIFF_HDR_SIZE 10+TIFF_HDR_NUM_ENTRY*12
#define RGB_BYTES 3
#define WIDTH 224
#define HEIGHT 224


#if VGA
#define WIDTH 648
#define HEIGHT 400
#endif

#if QCIF
#define WIDTH 536
#define HEIGHT 440
#endif

//#define IMAGE_DATA_SIZE  ((WIDTH*HEIGHT*RGB_BYTES)+ TIFF_HDR_SIZE)

extern const uint8_t im0[150528];

//#define ENABLE_PRINTF
#ifdef ENABLE_PRINTF
#define    DEBUG_PRINTF(f,...)    printf(f,##__VA_ARGS__)
#else
#define    DEBUG_PRINTF(f,...)
#endif


void i2c_write(uint16_t addr, uint32_t value, uint32_t data_len);
int i2c_read(uint16_t addr, uint32_t *rcv_data, uint32_t data_len);
void camera_controller_stop();

//camera irq
int CAMERA_IRQ = 336;
uint32_t rcv_data = 0;
uint32_t read_int = 0;
static int count=1;

/* wait function for adding sleep or wait */
void sleep_wait(int wait)
{
	uint32_t k;

	while(wait--)
	{
		for(k=0;k<950000;k++); //100ms/2/2 sec sleep
	}
}

/* Reset Camera Video memories addresses. */
void reset_camera_video_addr()
{
	uint32_t i = 0;
	int write;
	int flip=1;


	for(i=DEST_MEM_ADDRESS; i<(DEST_MEM_ADDRESS+0x200000); i+=4)  //dest  //sram0 Resetting 800KB
		{
			HW_REG_WORD(i,0x00) = 0x00;
		}


	for(i=SRC_MEM_ADDRESS; i<(SRC_MEM_ADDRESS+0x200000); i+=4) //src //TCM --> Resetting 512Kb
		{
			HW_REG_WORD(i,0x00) = 0x00;
		}

}

/* Copying Image data from SRC address to Destination address by skipping last 8 bytes. */
#if ENABLE_TEST_PATTERN_WALKING_1s
void copying_image_data(void)
{
	uint32_t i = 0, readl, total_dump;
	int write;
	int read = 0;
	int offset = 0;
	int j = 0;
	int k;
	uint32_t bytes=4, frame_cnt;
	uint8_t *src = (uint8_t *) SRC_MEM_ADDRESS; //TCM

	uint8_t *dest = (uint8_t *) 0x02000000;  //sram0
	uint8_t *image_start_addr= dest;
	uint8_t buff[16]={0x50, 0x35, 0x0A, 0x36, 0x34, 0x30, 0x20, 0x32, 0x34, 0x34, 0x0A, 0x32, 0x35, 0x35, 0x0A, 0x00}; //pgm headers //640x244
	memcpy(dest, buff, 16);
	dest=dest+16;


	frame_cnt=1; //one still image

	while(frame_cnt--) //total number of frames dumping to destination and skipping last 8 bytes
	{
		for(k=0; k<HEIGHT; k++) //for 80032KB bytes(648x244)
		{
			for(j=0; j<160; j++) //skip last 8 bytes (640/4)
			{
				memcpy(dest, src, bytes);
				dest=dest+4;
				src=src+4;
			}

			src = src+4; //for skipping 4 bytes
			src = src+4; //for skipping 8 bytes
		}
	}

	printf("Captured BMP Image start addr:0x%x, and end addr: 0x%x\n", (uint32_t) image_start_addr, (uint32_t) (dest -1));

}
#endif


// tiff types: short = 3, int = 4
// Tags: ( 2-byte tag ) ( 2-byte type ) ( 4-byte count ) ( 4-byte data )
//    0100 0003 0000 0001 0064 0000
//       |        |    |         |
// tag --+        |    |         |
// short int -----+    |         |
// one value ----------+         |
// value of 100 -----------------+
//

uint8_t tiff_header[TIFF_HDR_SIZE] = {
		// I     I     42
		0x49, 0x49, 0x2a, 0x00,
		// ( offset to tags, 0 )
		0x08, 0x00, 0x00, 0x00,
		// ( num tags )
		0x08, 0x00,
		// ( newsubfiletype, 0 full-image )
		0xfe, 0x00, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		// ( image width )
		0x00, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		// ( image height )
		0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		// ( bits per sample )
		0x02, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		// ( Photometric Interpretation, 2 = RGB )
		0x06, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
		// ( Strip offsets, 8 )
		0x11, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
		// ( samples per pixel, 3 - RGB)
		0x15, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
		// ( Strip byte count )
		0x17, 0x01, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};


/* this is the method used inside AVT cameras. See AVT docs. */
dc1394error_t
dc1394_bayer_Simple(const uint8_t *bayer, uint8_t *rgb, int sx, int sy, int tile)
{
	DEBUG_PRINTF("\r\n\r\n >>> 1 bayer:0x%X rgb:0x%X sx:0x%X sy:0x%X tile:0x%X <<< \r\n",(uint32_t)bayer,(uint32_t)rgb,sx,sy,tile);

	const int bayerStep = sx;
	const int rgbStep = 3 * sx;
	int width = sx;
	int height = sy;
	int blue = tile == DC1394_COLOR_FILTER_BGGR
			|| tile == DC1394_COLOR_FILTER_GBRG ? -1 : 1;
	int start_with_green = tile == DC1394_COLOR_FILTER_GBRG
			|| tile == DC1394_COLOR_FILTER_GRBG;
	int i, imax, iinc;

	if ((tile>DC1394_COLOR_FILTER_MAX)||(tile<DC1394_COLOR_FILTER_MIN))
		return DC1394_INVALID_COLOR_FILTER;

	DEBUG_PRINTF("\r\n\r\n >>> dc1394_bayer_Simple 1 <<< \r\n");

	DEBUG_PRINTF("\r\n\r\n >>> 2 bayer:0x%X rgb:0x%X sx:0x%X sy:0x%X tile:0x%X imax:0x%X iinc:0x%X i:0x%X<<< \r\n",(uint32_t)bayer,(uint32_t)rgb,sx,sy,tile,imax,iinc,i);

	/* add black border */
	imax = sx * sy * 3;
	for (i = sx * (sy - 1) * 3; i < imax; i++)
	{
		rgb[i] = 0;
	}

	DEBUG_PRINTF("\r\n\r\n >>> dc1394_bayer_Simple 11 <<< \r\n");

	iinc = (sx - 1) * 3;

	DEBUG_PRINTF("\r\n\r\n >>> bayer:0x%X rgb:0x%X sx:0x%X sy:0x%X tile:0x%X imax:0x%X iinc:0x%X i:0x%X<<< \r\n",(uint32_t)bayer,(uint32_t)rgb,sx,sy,tile,imax,iinc,i);

	for (i = (sx - 1) * 3; i < imax; i += iinc)
	{
		rgb[i++] = 0;
		rgb[i++] = 0;
		rgb[i++] = 0;
	}

	DEBUG_PRINTF("\r\n\r\n >>> dc1394_bayer_Simple 111 <<< \r\n");

	rgb += 1;
	width -= 1;
	height -= 1;

	DEBUG_PRINTF("\r\n\r\n >>> dc1394_bayer_Simple 2 <<< \r\n");

	for (; height--; bayer += bayerStep, rgb += rgbStep) {
		const uint8_t *bayerEnd = bayer + width;

		if (start_with_green) {
			rgb[-blue] = bayer[1];
			rgb[0] = (bayer[0] + bayer[bayerStep + 1] + 1) >> 1;
			rgb[blue] = bayer[bayerStep];
			bayer++;
			rgb += 3;
		}

		if (blue > 0) {
			for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
				rgb[-1] = bayer[0];
				rgb[0] = (bayer[1] + bayer[bayerStep] + 1) >> 1;
				rgb[1] = bayer[bayerStep + 1];

				rgb[2] = bayer[2];
				rgb[3] = (bayer[1] + bayer[bayerStep + 2] + 1) >> 1;
				rgb[4] = bayer[bayerStep + 1];
			}
		} else {
			for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
				rgb[1] = bayer[0];
				rgb[0] = (bayer[1] + bayer[bayerStep] + 1) >> 1;
				rgb[-1] = bayer[bayerStep + 1];

				rgb[4] = bayer[2];
				rgb[3] = (bayer[1] + bayer[bayerStep + 2] + 1) >> 1;
				rgb[2] = bayer[bayerStep + 1];
			}
		}

		if (bayer < bayerEnd) {
			rgb[-blue] = bayer[0];
			rgb[0] = (bayer[1] + bayer[bayerStep] + 1) >> 1;
			rgb[blue] = bayer[bayerStep + 1];
			bayer++;
			rgb += 3;
		}

		bayer -= width;
		rgb -= width * 3;

		blue = -blue;
		start_with_green = !start_with_green;
	}

	DEBUG_PRINTF("\r\n\r\n >>> dc1394_bayer_Simple END <<< \r\n");

	return DC1394_SUCCESS;

}

/*Bayer 8-bit decoding for RGB */
dc1394error_t
dc1394_bayer_decoding_8bit(const uint8_t *restrict bayer, uint8_t *restrict rgb, uint32_t sx, uint32_t sy, dc1394color_filter_t tile, dc1394bayer_method_t method)
{
	switch (method) {

	case DC1394_BAYER_METHOD_SIMPLE:
		return dc1394_bayer_Simple(bayer, rgb, sx, sy, tile);

	default:
		return DC1394_INVALID_BAYER_METHOD;
	}

}

/*Add tiff format headers */
uint8_t *
put_tiff(uint8_t * rgb, uint32_t width, uint32_t height, uint16_t bpp)
{
	uint32_t ulTemp=0;
	uint16_t sTemp=0;
	memcpy(rgb, tiff_header, TIFF_HDR_SIZE);

	sTemp = TIFF_HDR_NUM_ENTRY;
	memcpy(rgb + 8, &sTemp, 2);

	memcpy(rgb + 10 + 1*12 + 8, &width, 4);
	memcpy(rgb + 10 + 2*12 + 8, &height, 4);
	memcpy(rgb + 10 + 3*12 + 8, &bpp, 2);

	// strip byte count
	ulTemp = width * height * (bpp / 8) * 3;
	memcpy(rgb + 10 + 7*12 + 8, &ulTemp, 4);

	//strip offset
	sTemp = TIFF_HDR_SIZE;
	memcpy(rgb + 10 + 5*12 + 8, &sTemp, 2);

	return rgb + TIFF_HDR_SIZE;
};

/*Bayer to RGB conversion */
int bayer_to_RGB(void)
{
	uint32_t in_size=0, out_size=0;
	uint32_t width=0, height=0, bpp=0;
	int result;

	int first_color = DC1394_COLOR_FILTER_GBRG;
	int tiff = 0;
	int method = DC1394_BAYER_METHOD_SIMPLE;

	char *infile=NULL, *outfile=NULL;
	int input_fd = 0;
	int output_fd = 0;

	uint8_t * bayer = NULL;
	uint8_t * rgb = NULL;
	uint8_t *rgb_start = NULL;

	int c;
	int optidx = 0;
	int swap = 0;

	uint8_t *src = (uint8_t *)  (SRC_MEM_ADDRESS);
	uint8_t *dest = (uint8_t *) (DEST_MEM_ADDRESS);

	width  = WIDTH; //640 col
	height = HEIGHT; //244 row
	bpp = 8;
	tiff = TIFF_HDR_SIZE;

	out_size = width * height * (bpp / 8) * 3 + tiff;

	bayer = (uint8_t *) src;     //src addr
	rgb   = (uint8_t *) dest;    //des addr

	DEBUG_PRINTF("\r\n\r\n >>> bayer_to_RGB put_tiff <<< \r\n");

	rgb_start = put_tiff(rgb, width, height, bpp);

	DEBUG_PRINTF("\r\n\r\n >>> bayer_to_RGB dc1394_bayer_decoding_8bit <<< \r\n");

	dc1394_bayer_decoding_8bit((const uint8_t*)bayer, (uint8_t*)rgb_start, width, height, first_color, method);

	memcpy(im0, rgb, sizeof(im0));
	result = memcmp(im0, rgb, sizeof(im0));
	if(result != 0)
	{
		printf("\r\n XXX Memcopy failed, error status %d \r\n", result);
		return result;
	}

	printf("\n Captured Tiff Color Image start:0x%x and  end addr: 0x%x\n", (uint32_t) im0, (uint32_t) (im0+sizeof(im0)-1));

	return result;
}

/* I2C write for 1, 2 and 4 bytes. */
void i2c_write(uint16_t addr, uint32_t value, uint32_t data_len)
{
	int write = 0;
	int read = 0;
	int i = 0;
	int offset = 0;
	int rd_data ;
	int rsp_q_data ;
	int ret = 0;
	uint32_t tx_fifo = 0;
	uint32_t tx_fifo2 = 0;

	uint8_t addr_high = (addr >> 8) & 0xFF;
	uint8_t addr_low = addr & 0xFF;


	if( data_len == 1) //data val 1 byte: 8-bit
	{
		// 2 byte, 16 bit slave addr + 1 byte 8-bit data
		// tx len = 2 + 1 = 3
		write = 0x00030001; //  data length 3 //camera location 2 byte

		//tx fifo : high byte slave + low byte slave + data
		//tx_fifo = (addr_low << 8) | addr_high | ( (value & 0xff) << 16 );
		tx_fifo = (addr_high) | (addr_low << 8) | ( (value & 0xff) << 16 );
	}

	if( data_len == 2) //data val 2 byte: 16-bit
	{
		// 2 byte, 16 bit slave addr + 2 byte 16-bit data
		// tx len = 2 + 2 = 4
		write = 0x00040001; //  data length 4 //camera location 2 byte

		//tx fifo : high byte slave + low byte slave + data
		//tx_fifo = (addr_low << 8) | addr_high | ( (value & 0xff) << 16 );
		tx_fifo = (addr_high) | (addr_low << 8) | ( ((value >> 8) & 0xff) << 16 ) | ( ((value) & 0xff) << 24 );
	}

	if( data_len == 4) //data val 4 byte: 32-bit
	{
		// 2 byte, 16 bit slave addr + 4 byte 32-bit data
		// tx len = 2 + 4 = 6
		write = 0x00060001; //  data length 6 //camera location 2 byte

		//tx fifo : high byte slave + low byte slave + data
		//tx_fifo = (addr_low << 8) | addr_high | ( (value & 0xff) << 16 );
		tx_fifo = (addr_high) | (addr_low << 8) | ( ((value >> 24) & 0xff) << 16 ) | ( ((value >> 16) & 0xff) << 24 );

		tx_fifo2 = ( ((value >> 8) & 0xff) ) | ( ((value) & 0xff) << 8 );

	}

	offset = 0xc;
	HW_REG_WORD(I3C0_BASE_ADDR,offset) = write;
	rsp_q_data =  HW_REG_WORD((I3C0_BASE_ADDR),0x10);


	HW_REG_WORD((I3C0_BASE_ADDR),0x14) = tx_fifo;

	if( data_len == 4) //data val 4 byte: 32-bit
	{
		HW_REG_WORD((I3C0_BASE_ADDR),0x14) = tx_fifo2;
	}

	write = 0x44000018; // write + stop
	offset = 0xc;
	HW_REG_WORD(I3C0_BASE_ADDR,offset) = write;

	rsp_q_data =  HW_REG_WORD((I3C0_BASE_ADDR),0x10);
}


/* I2C read for 1, 2 and 4 bytes. */
int i2c_read(uint16_t addr, uint32_t *rcv_data, uint32_t data_len)
{
	int write = 0;
	int read = 0;
	int i = 0;
	int offset = 0;
	uint32_t rd_data ;
	int rsp_q_data ;
	int ret = 0;

	uint8_t addr_high = (addr >> 8) & 0xFF;
	uint8_t addr_low = addr & 0xFF;


	write = 0x00020001; //  data length 2 //camera location 2 byte
	offset = 0xc;
	HW_REG_WORD(I3C0_BASE_ADDR,offset) = write;

	rsp_q_data =  HW_REG_WORD((I3C0_BASE_ADDR),0x10);

	HW_REG_WORD((I3C0_BASE_ADDR),0x14) = (addr_low << 8) | addr_high ;

	write = 0x44000018; // write + stop
	offset = 0xc;
	HW_REG_WORD(I3C0_BASE_ADDR,offset) = write;
	read = HW_REG_WORD(I3C0_BASE_ADDR,offset);

	rsp_q_data =  HW_REG_WORD((I3C0_BASE_ADDR),0x10);

	///// read that location //////////////////

	if( data_len == 1) //data val 1 byte: 8-bit
	{
		write = 0x00010001; //  data length 1
	}

	if( data_len == 2) //data val 2 byte: 16-bit
	{
		write = 0x00020001; //  data length 1
	}

	if( data_len == 4) //data val 4 byte: 32-bit
	{
		write = 0x00040001; //  data length 1
	}

	offset = 0xc;
	HW_REG_WORD(I3C0_BASE_ADDR,offset) = write;

	rsp_q_data =  HW_REG_WORD((I3C0_BASE_ADDR),0x10);

	write = 0x54000018; // read + stop
	offset = 0xc;
	HW_REG_WORD(I3C0_BASE_ADDR,offset) = write;

	rsp_q_data =  HW_REG_WORD((I3C0_BASE_ADDR),0x10);

	sleep_wait(10); //todo added
	//READ RX_DATA_PORT register to read data received

	rd_data = 0;
	for (int i=0 ;i<1; i++)
	{
		rd_data =  HW_REG_WORD((I3C0_BASE_ADDR),0x14);
	}

	ret = rd_data;

	//higher byte will come first in fifo
	if( data_len == 1) //data val 1 byte: 8-bit
	{
		rd_data = rd_data & 0xff; //  data length 1
	}

	if( data_len == 2) //data val 2 byte: 16-bit
	{
		rd_data = rd_data & 0xffff; //  data length 2

		rd_data =  ( (rd_data & 0xff) << 8) | ( (rd_data & 0xff00) >> 8);
	}

	if( data_len == 4) //data val 4 byte: 32-bit
	{
		rd_data = ( (rd_data & 0xff) << 24) | ((rd_data & 0xff00) << 8) | ((rd_data & 0xff0000) >> 8) | ((rd_data & 0xff000000) >> 24); //  data length 4
	}

	*rcv_data = rd_data;

	return 0;
}


/* Wait for a command to complete. */
static int mt9m114_poll_command(uint32_t command)
{
	unsigned int i;
	uint32_t j;
	uint32_t value;
	int ret;

	for (i = 0; i < 100; ++i) {
		ret = i2c_read(MT9M114_COMMAND_REGISTER, &value,2);
		if (ret < 0)
			return ret;

		if (!(value & command))
			break;

		sleep_wait(10);
	}

	if (value & command) {
		printf("Command %u completion timeout\n", command);
		return -1;
	}

	if (!(value & MT9M114_COMMAND_REGISTER_OK)) {
		printf("Command %u failed\n", command);
		return -1;
	}

	printf("Poll command 0x%x and value 0x%x \n", command, value);

	return 0;
}


/* The Change-Config state. */
static int mt9m114_set_state(uint8_t next_state)
{
	int ret = 0;

	/* Set the next desired state and start the state transition. */
	i2c_write(MT9M114_SYSMGR_NEXT_STATE, next_state, 1);
	if (ret < 0)
		return ret;

	i2c_write(MT9M114_COMMAND_REGISTER,
			MT9M114_COMMAND_REGISTER_OK |
			MT9M114_COMMAND_REGISTER_SET_STATE, 2);
	if (ret < 0)
		return ret;

	/* Wait for the state transition to complete. */
	ret = mt9m114_poll_command(MT9M114_COMMAND_REGISTER_SET_STATE);
	if (ret < 0)
		return ret;

	printf("Ret state 0x%x \n", ret);

	return 0;
}

/* camera register state change. */
static int mt9m114_reg_init()
{
	return mt9m114_set_state(MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);
}

/* camera sensor start. */
static int mt9m114_stream_start()
{
	return mt9m114_set_state(MT9M114_SYS_STATE_START_STREAMING);
}


/* camera sensor stop. */
static int mt9m114_stream_stop()
{
	return mt9m114_set_state(MT9M114_SYS_STATE_ENTER_SUSPEND);
}

/* camera sensor Initialization. */
int mt9m114_camera_init(void)
{
	int i = 0, ret;
	int total = 0;
	uint32_t j;
	uint16_t output_format;
	int delay = 0;
	uint32_t value;

	total = (sizeof(mt9m114_qvga_24mhz_pll) / sizeof(mt9m114_reg));

	for(i = 0; i < total; i++)
	{
		DEBUG_PRINTF("\r\n i: %d reg:0x%X val: 0x%X size: 0x%x", i, mt9m114_qvga_24mhz_pll[i].ui16Reg, mt9m114_qvga_24mhz_pll[i].ui32Val, mt9m114_qvga_24mhz_pll[i].value_size);
		sleep_wait(1);
		i2c_write(mt9m114_qvga_24mhz_pll[i].ui16Reg, mt9m114_qvga_24mhz_pll[i].ui32Val, mt9m114_qvga_24mhz_pll[i].value_size);

	}


	/* Raw bayer 10bit format */
	i2c_write(0x001e, 0x0, 2); //slew rate
	output_format =  MT9M114_CAM_OUTPUT_FORMAT_FORMAT_BAYER; //todo working with OSC
	i2c_write(MT9M114_CAM_OUTPUT_FORMAT, output_format, sizeof(output_format));

	//flip capture image H and V
	//value = (0x0110 | 0x01 | 0x02) ; //H is bit 0, V is bit 1
	//i2c_write(0xC834, value, 2);

	sleep_wait(1);

	return 0;
}

/* Walking 1s test function. */
void mt9m114_camera_testPattern_walking_1s(void)
{
	int i = 0;
	uint32_t j;
	int total = 0;
	int ret = 0;

	total = (sizeof(sMT9M114TestModeScript_Walking1s) / sizeof(mt9m114_reg));

	printf("\r\n MT9M114_camera_testPattern_walking_1s total: %d \r\n", total);


	for(i = 0; i < total; i++)
	{
		printf("\r\n i: %d reg:0x%X val: 0x%X size: 0x%x", i, sMT9M114TestModeScript_Walking1s[i].ui16Reg, sMT9M114TestModeScript_Walking1s[i].ui32Val, sMT9M114TestModeScript_Walking1s[i].value_size);
		i2c_write(sMT9M114TestModeScript_Walking1s[i].ui16Reg, sMT9M114TestModeScript_Walking1s[i].ui32Val, sMT9M114TestModeScript_Walking1s[i].value_size);
		sleep_wait(1);
	}
}

/* I3c master configuration . */
void i3c_cnfg_mst0()
{

#define BYPASS_CLK 1
#define CFG_APB_EXPMST0 0x4902F000

#if BYPASS_CLK //bypass clock
	__IOM uint32_t *clkreg_expmst0 = (uint32_t *) CFG_APB_EXPMST0;

	*clkreg_expmst0 = 0xffffffff;

	DEBUG_PRINTF("\r\n \t\t >>> bypass clock 0x4902F000 :%0X %0X ...\r\n", (unsigned int)clkreg_expmst0, *clkreg_expmst0);
#endif

	/* i3c master0 config settings */
#define OFFSET_I3C_CTRL      0x24 /* i3c ctrl offset */

	__IOM uint32_t *reg_uart_ctrl = (uint32_t *) (CFG_APB_EXPMST0 + OFFSET_I3C_CTRL);

	DEBUG_PRINTF("\r\n \t\t >>> i3c before setting 0x4902F024 :%0X %0X ...\r\n", (unsigned int)reg_uart_ctrl, *reg_uart_ctrl);

	*reg_uart_ctrl = 0x01000001; //original //todo
	DEBUG_PRINTF("\r\n \t\t \t\t  >>> i3c after setting 0x4902F024 :%0X %0X ...\r\n", (unsigned int)reg_uart_ctrl, *reg_uart_ctrl);
}


/* Camera controller Interrupt handler. */
void IntHandler_camera (void)
{
	unsigned int i,data;
	int cnt;
	uint32_t read_int2;
	read_int = HW_REG_WORD(CAMERA_BASE_ADDR,0x0004);

	HW_REG_WORD(CAMERA_BASE_ADDR,0x0004) = 0xffffffff;//clear all Irqs
}




/* Camera controller registers configuration. */
void camera_controller_conf()
{
	int write = 0;
	int read = 0;
	int i = 0;
	int offset = 0;


	write = 0xffffffff;//enable irq
	offset = 0x0008;
	HW_REG_WORD(CAMERA_BASE_ADDR,offset) = write;
	read = HW_REG_WORD(CAMERA_BASE_ADDR,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (CAMERA_BASE_ADDR+offset) );



	write = 0x00001100; //camera config: 8-bit, pclk invert, no_hsync=1, pclk from sensor
	offset = 0x0010;
	HW_REG_WORD(CAMERA_BASE_ADDR,offset) = write;
	read = HW_REG_WORD(CAMERA_BASE_ADDR,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (CAMERA_BASE_ADDR+offset) );


	write = 0x0012000f;//camera fifo water mark
	offset = 0x0014;
	HW_REG_WORD(CAMERA_BASE_ADDR,offset) = write;
	read = HW_REG_WORD(CAMERA_BASE_ADDR,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (CAMERA_BASE_ADDR+offset) );


	write = 0x00000000;//camera hbp,hfp
	offset = 0x0020;
	HW_REG_WORD(CAMERA_BASE_ADDR,offset) = write;
	read = HW_REG_WORD(CAMERA_BASE_ADDR,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (CAMERA_BASE_ADDR+offset) );


	write = 0x00000000;//camera vbp,vfp
	offset = 0x0024;
	HW_REG_WORD(CAMERA_BASE_ADDR,offset) = write;
	read = HW_REG_WORD(CAMERA_BASE_ADDR,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (CAMERA_BASE_ADDR+offset) );



	//write = 0x01908287; // 400x648
	write = (HEIGHT << 16) | (1 << 15) | (WIDTH-1); //400x648
	//write = 0x00F48287; //244x328 //// //working for Onsmei bayer + 10 bit //todo original
	//write = 0x00F48288; //244x328 //// //working for Onsmei bayer + 10 bit //todo original
	//write = 0x00028287; //244x328 //// //working for Onsmei bayer + 10 bit
	offset = 0x0028;
	HW_REG_WORD(CAMERA_BASE_ADDR,offset) = write;
	read = HW_REG_WORD(CAMERA_BASE_ADDR,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (CAMERA_BASE_ADDR+offset) );


	write = 0x00; //0x00000000;//camera color mode
	offset = 0x002c;
	HW_REG_WORD(CAMERA_BASE_ADDR,offset) = write;
	read = HW_REG_WORD(CAMERA_BASE_ADDR,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (CAMERA_BASE_ADDR+offset) );


	write = SRC_MEM_ADDRESS; //SRAM1 addr
	offset = 0x0030;
	HW_REG_WORD(CAMERA_BASE_ADDR,offset) = write;
	read = HW_REG_WORD(CAMERA_BASE_ADDR,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (CAMERA_BASE_ADDR+offset) );


	write = SRC_MEM_ADDRESS + one_fifty_six_KB;
	offset = 0x0034;
	HW_REG_WORD(CAMERA_BASE_ADDR,offset) = write;
	read = HW_REG_WORD(CAMERA_BASE_ADDR,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (CAMERA_BASE_ADDR+offset) );


	write = SRC_MEM_ADDRESS + (2 * one_fifty_six_KB) ;
	offset = 0x0038;
	HW_REG_WORD(CAMERA_BASE_ADDR,offset) = write;
	read = HW_REG_WORD(CAMERA_BASE_ADDR,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (CAMERA_BASE_ADDR+offset) );


	//soft-reset
	write = 0x00000100; //camera soft reset
	offset = 0x0000;
	HW_REG_WORD(CAMERA_BASE_ADDR,offset) = write;
	read = HW_REG_WORD(CAMERA_BASE_ADDR,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (CAMERA_BASE_ADDR+offset) );

	write = 0x00000000; //camera soft reset
	offset = 0x0000;
	HW_REG_WORD(CAMERA_BASE_ADDR,offset) = write;
	read = HW_REG_WORD(CAMERA_BASE_ADDR,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (CAMERA_BASE_ADDR+offset) );


	write = 0x00000011; //camera start + snapshot
	offset = 0x0000;
	HW_REG_WORD(CAMERA_BASE_ADDR,offset) = write;
	read = HW_REG_WORD(CAMERA_BASE_ADDR,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (CAMERA_BASE_ADDR+offset) );
}

/* Camera controller stop. */
void camera_controller_stop()
{
	int write = 0;
	int read = 0;
	int i = 0;
	int offset = 0;

	write = 0x00000000; //camera stop
	offset = 0x0000;
	HW_REG_WORD(CAMERA_BASE_ADDR,offset) = write;

}

/* Camera controller Pinmax. */
void camera_pinmux()
{
	int ret;
	int write = 0;
	int read = 0;
	int i = 0;
	int offset = 0;

	DEBUG_PRINTF("\r\n\r\n >>> Camera Pin-Mux <<< \r\n");

	write = 0x06670000; ////cam hsyncB,camvsyncB,campclkB,camXVCLK disable
	offset = 0x0;
	HW_REG_WORD(0X71006020,offset) = write;
	read = HW_REG_WORD(0X71006020,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (0X71006020+offset) );


	write = 0x65666600; ////data lines from D2 to D7 on B
	offset = 0x0;
	HW_REG_WORD(0x71006024,offset) = write;
	read = HW_REG_WORD(0x71006024,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (0x71006024+offset) );

	write = 0x00660000; ////data lines D0 and D1 lines on A
	offset = 0x0;
	HW_REG_WORD(0x7100602c,offset) = write;
	read = HW_REG_WORD(0x7100602c,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (0x71006024+offset) );
}



/* I3c and camera controller pinmux and pinpad settings. */
int hardware_init(void)
{
	int ret;
	int write = 0;
	int read = 0;
	int i = 0;
	int offset = 0;

	//as per thunder datasheet
	// I3C_SDA_B  -> pin P3_8
	// I3C_SCL_B  -> pin P3_9

	DEBUG_PRINTF("\r\n\r\n >>> i3c Pin-Mux <<< \r\n");

	write = 0x43; //// P3_8 as I3C_SDA_B, P3_9 as I3C_SCL_B
	offset = 0x0;
	HW_REG_WORD(0X71006034,offset) = write;
	read = HW_REG_WORD(0X71006034,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (0X71006034+offset) );

	/* Pin-Pad */
	DEBUG_PRINTF("\r\n\r\n >>> i3c Pin-Pad <<< \r\n");


	//bare-metal pinpad P3_8 as I3C_SDA_B
	write = (0x09 | PAD_FUNCTION_OUTPUT_DRIVE_STRENGTH_04_MILI_AMPS); ////
	offset = 0x0;
	HW_REG_WORD(0X71007120,offset) = write;
	read = HW_REG_WORD(0X71007120,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (0X71007120+offset) );

	//bare-metal pinpad P3_9 as I3C_SCL_B
	write = (0x09 | PAD_FUNCTION_OUTPUT_DRIVE_STRENGTH_04_MILI_AMPS); ////
	offset = 0x0;
	HW_REG_WORD(0X71007124,offset) = write;
	read = HW_REG_WORD(0X71007124,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (0X71007124+offset) );


	reset_camera_video_addr();

	camera_pinmux(); //Pinmux B

	return 0;
}

/* MT9M114 hard reset. */
void mt9m114_hard_reset()
{

	int write = 0;
	int read = 0;
	int i = 0;
	int offset = 0;
	int rd_data ;
	int rsp_q_data ;
	int ret = 0;

	//todo //reset
	write = 0x20; //
	offset = 0x0;
	HW_REG_WORD(0x49002000,offset) = write;
	read = HW_REG_WORD(0x49002000,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X \r\n",i, write, read, offset);

	write = 0x20; //
	offset = 0x4;
	HW_REG_WORD(0x49002000,offset) = write;
	read = HW_REG_WORD(0x49002000,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X \r\n",i, write, read, offset);

	write = 0x20; //
	offset = 0x8;
	HW_REG_WORD(0x49002000,offset) = write;
	read = HW_REG_WORD(0x49002000,offset);
	DEBUG_PRINTF("\r\n %d  write:%X read:%X offset:%X \r\n",i, write, read, offset);

	sleep_wait(10);

	write = 0x0; //
	offset = 0x0;
	HW_REG_WORD(0x49002000,offset) = write;
	read = HW_REG_WORD(0x49002000,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X \r\n",i, write, read, offset);


	write = 0x0; //
	offset = 0x4;
	HW_REG_WORD(0x49002000,offset) = write;
	read = HW_REG_WORD(0x49002000,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X \r\n",i, write, read, offset);
}

/* I2C over I3C configuration.  */
void i3c_controller_conf(void)
{
	int write = 0;
	int read = 0;
	int i = 0;
	int offset = 0;
	int rd_data ;
	int rsp_q_data ;
	int ret = 0;


	write = 0x0;
	offset = 0x0;
	HW_REG_WORD(I3C0_BASE_ADDR,offset) = write;
	read = HW_REG_WORD(I3C0_BASE_ADDR,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (I3C0_BASE_ADDR+offset) );



	write = 0x80550000;
	offset = 0x04;
	HW_REG_WORD(I3C0_BASE_ADDR,offset) = write;
	read = HW_REG_WORD(I3C0_BASE_ADDR,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (I3C0_BASE_ADDR+offset) );


	write = 0x00000001;
	offset = 0x1c;
	HW_REG_WORD(I3C0_BASE_ADDR,offset) = write;
	read = HW_REG_WORD(I3C0_BASE_ADDR,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (I3C0_BASE_ADDR+offset) );


	write = 0x0000300;
	offset = 0x20;
	HW_REG_WORD(I3C0_BASE_ADDR,offset) = write;
	read = HW_REG_WORD(I3C0_BASE_ADDR,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (I3C0_BASE_ADDR+offset) );


	write = 0x0000002;
	offset = 0x44;
	HW_REG_WORD(I3C0_BASE_ADDR,offset) = write;
	read = HW_REG_WORD(I3C0_BASE_ADDR,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (I3C0_BASE_ADDR+offset) );


	write = 0x00000012;
	offset = 0x40;
	HW_REG_WORD(I3C0_BASE_ADDR,offset) = write;
	read = HW_REG_WORD(I3C0_BASE_ADDR,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (I3C0_BASE_ADDR+offset) );


	write = 0x000a0010;
	offset = 0xb4;
	HW_REG_WORD(I3C0_BASE_ADDR,offset) = write;
	read = HW_REG_WORD(I3C0_BASE_ADDR,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (I3C0_BASE_ADDR+offset) );


	write = 0x000a000a;
	offset = 0xb8;
	HW_REG_WORD(I3C0_BASE_ADDR,offset) = write;
	read = HW_REG_WORD(I3C0_BASE_ADDR,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (I3C0_BASE_ADDR+offset) );


	write = 0x01F401F4; //100Khz
	offset = 0xbc;
	HW_REG_WORD(I3C0_BASE_ADDR,offset) = write;
	read = HW_REG_WORD(I3C0_BASE_ADDR,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (I3C0_BASE_ADDR+offset) );


	write = 0x80000048; //onsemi address
	offset = 0x280;
	HW_REG_WORD(I3C0_BASE_ADDR,offset) = write;
	read = HW_REG_WORD(I3C0_BASE_ADDR,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (I3C0_BASE_ADDR+offset) );


	write = 0x88000080;
	offset = 0x00;
	HW_REG_WORD(I3C0_BASE_ADDR,offset) = write;
	read = HW_REG_WORD(I3C0_BASE_ADDR,offset);
	DEBUG_PRINTF("\r\n %d write:%X read:%X offset:%X read:%X \r\n",i, write, read, offset, (I3C0_BASE_ADDR+offset) );

}

/* MT9M114 soft reset. */
void mt9m114_soft_reset()
{

	i2c_write(0x001A, 0x0001, 2);
	sleep_wait(1); //100msec
	i2c_write(0x001A, 0x0000, 2);
}


/* MT9M114 image capturing. */
void onsemi_mt9m114_image_capture(void)
{
	int write = 0;
	int read = 0;
	uint32_t i = 0, result;
	int offset = 0;
	uint32_t rd_data ;
	int rsp_q_data ;
	int ret = 0;
	int temp = 0;
	int j=0;
	int value = 0;
	uint16_t output_format = 0;

	/* Initialize i3c hardware pins using PinMux Driver. */
	ret = hardware_init();  //i3c and camera pinmux and pinpad
	if(ret != 0)
	{
		printf("\r\n Error in i3c hardware_init.\r\n");
		return;
	}

	i3c_cnfg_mst0();
	i3c_controller_conf();
	mt9m114_hard_reset();
	sleep_wait(1); //25msec delay

	/* Soft Reset the camera */
	mt9m114_soft_reset();
	sleep_wait(1); //25msec delay

	ret = i2c_read(0x0000, &rcv_data, 2);

	if(rcv_data !=  0x2481)
	{

	printf(">>>> Image capture failed :0x%X <<<< \n", rcv_data);
	return;
	}
	// chip_id should be 0x2481
	printf("\r\n >>> onsemi chip-id :0x%X!!! <<< \r\n", rcv_data);

	printf("\r\n XXX mt9m114_camera_init \r\n");
	/* Initialize the camera */
	mt9m114_camera_init();
	sleep_wait(20); //500msec delay

#if ENABLE_TEST_PATTERN_WALKING_1s  //walking ones
	mt9m114_camera_testPattern_walking_1s();
#endif

	printf("\r\n XXX mt9m114 register state change \r\n");
	ret=mt9m114_reg_init();
	sleep_wait(20); //500msec delay
	if (ret < 0)
	{
		printf("\r\n XXX mt9m114 register state change failed \r\n");
		return;
	}


	//streaming
	printf("\r\n XXX mt9m114_stream_start \r\n");
	ret=mt9m114_stream_start();
	sleep_wait(40); //1-sec delay
	if (ret < 0)
	{
		printf("\r\n XXX mt9m114_stream_start failed \r\n");
		return;
	}

	//camera controller start
	printf("\r\n camera controller start \r\n");
	camera_controller_conf();
	sleep_wait(50); //1.1-sec delay

	/* Suspend any stream */
	printf("\r\n XXX Final mt9m114_stream_suspend \r\n");
	ret=mt9m114_stream_stop();
	if (ret < 0)
	{
		printf("\r\n XXX Final mt9m114_stream_suspend failed\r\n");
		return;
	}

	/* Stop the camera controller */
	printf("\r\n camera controller stop \r\n");
	camera_controller_stop();
	sleep_wait(1); //25millesec delay

	result = HW_REG_WORD((SRC_MEM_ADDRESS + (WIDTH*HEIGHT) - 4),0x0);
	if(result == 0x0)
	{
		printf("\r\n XXX Image captured  failed \r\n");
		return;
	}

#if ENABLE_TEST_PATTERN_WALKING_1s
	printf("\r\n XXX Adding bmp headers for walking 1s\r\n");
	copying_image_data();
#endif

#if REAL_IMAGE
	printf("\r\n XXX mt9m114 bayer_to_RGB coversion\r\n");
	result = bayer_to_RGB();
	if(result != 0)
	{
		return;
	}

#endif


	NVIC_DisableIRQ (CAMERA_IRQ);
	NVIC_ClearPendingIRQ(CAMERA_IRQ);
	printf("\r\n Image captured \r\n");
        printf("\r\n Image Address: %x \r\n",im0);
	//while(1); //-------> END xxxxx
}

/* Define main entry point.  */
//int main()
//{

	//printf("\r\n >>> Onsemi MT9M114 image capturing started !!! <<< \r\n");
	//onsemi_mt9m114_image_capture();

//}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
#endif