/* Copyright (c) 2022 ALIF SEMICONDUCTOR
#include <display_cfg.h>

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
 * @file     lv_port_disp.c
 * @author   Ahmad Rashed
 * @email    ahmad.rashed@alifsemi.com
 * @version  V1.0.0
 * @date     23-Feb-2022
 * @brief    for lvgl library display init
 * @bug      None.
 * @Note     None
 ******************************************************************************/
#include <RTE_Components.h>
#include CMSIS_device_header

#include "lvgl.h"
#include "display_cfg.h"

#define DISABLE_I2C_TOUCH

uint8_t lcd_image[DIMAGE_Y][DIMAGE_X][PIXEL_BYTES] __attribute__((section("lcd_image_buf")));
static int lv_port_initialized = 0;

static void lv_disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
	int32_t y, x = area->x1;
	int32_t x_len = 1 + area->x2 - area->x1;
	for(y = area->y1; y <= area->y2; y++) {
		memcpy(&lcd_image[y][x][0], color_p, PIXEL_BYTES * x_len);
		color_p += x_len;
	}
	lv_disp_flush_ready(disp_drv);
}

#ifndef DISABLE_I2C_TOUCH
#include "GT911_touch_driver.h"

void hw_touch_init(void)
{
	TOUCH_Init(i2c_addr_5D);
}

static void lv_touch_get(lv_indev_drv_t * drv, lv_indev_data_t * data)
{
	static xypair_t xypair = {0, 0};
	if (TOUCH_GetPointer(i2c_addr_5D, &xypair) == 0) {
	    data->state = LV_INDEV_STATE_PRESSED;
	} else {
		data->state = LV_INDEV_STATE_RELEASED;
	}
	data->point.x = xypair.x;
	data->point.y = xypair.y;
}
#endif

void lv_port_disp_init(void)
{
	/* display structures */
	static lv_disp_drv_t disp_drv;
	static lv_disp_draw_buf_t disp_buf;
	static lv_color_t color_buf[DIMAGE_Y * 10];

	hw_disp_init();

	lv_init();
	lv_disp_drv_init(&disp_drv);
	lv_disp_draw_buf_init(&disp_buf, color_buf, NULL, DIMAGE_Y * 10);
	disp_drv.draw_buf = &disp_buf;
	disp_drv.hor_res = DIMAGE_X;
	disp_drv.ver_res = DIMAGE_Y;
//	disp_drv.rotated = LV_DISP_ROT_90;
//	disp_drv.sw_rotate = 1;
	disp_drv.flush_cb = lv_disp_flush;
	lv_disp_drv_register(&disp_drv);

#ifndef DISABLE_I2C_TOUCH
	hw_touch_init();

	/* input device structure */
	static lv_indev_drv_t touch_drv;

	lv_indev_drv_init(&touch_drv);
	touch_drv.type = LV_INDEV_TYPE_POINTER;
	touch_drv.read_cb = lv_touch_get;
	lv_indev_drv_register(&touch_drv);
#endif

    lv_port_initialized = 1;
}

void lv_port_tick_inc()
{
    if (lv_port_initialized == 1)
        lv_tick_inc(1);
}

