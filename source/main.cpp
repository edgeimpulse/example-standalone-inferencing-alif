/* Edge Impulse inferencing library
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdio.h>
#include "hal.h"
#include "hal_image.h"
#include "log_macros.h"
#include "ScreenLayout.hpp"

#include "lvgl.h"
#include "lv_port.h"
#include "lv_paint_utils.h"


#define LIMAGE_X        192
#define LIMAGE_Y        192
#define LV_ZOOM         (2 * 256)

namespace {
lv_style_t boxStyle;
lv_color_t  lvgl_image[LIMAGE_Y][LIMAGE_X] __attribute__((section(".bss.lcd_image_buf")));                      // 448x448x4 = 802,856
};

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/ei_utils.h"

#include <cstdio>

#if !NDEBUG
extern "C" void __stack_chk_fail(void) { 
    ei_printf("Stack overflow caught\n");
    while (1) {} 
} // trap stack overflow
void* __stack_chk_guard = (void*)0xaeaeaeae;
#endif

/* Project Includes */
int main()
{
    hal_platform_init();
    info("Initializing Camera...\r\n");

    hal_image_init();
    info("Initializing Camera OK\n");

    // alif::app::ScreenLayoutInit(lvgl_image, sizeof lvgl_image, LIMAGE_X, LIMAGE_Y, LV_ZOOM);

    printf_err("test\n");
    while(1) {
        //const uint8_t* currImage = hal_get_image_data(192, 192);
        //if (!currImage) {
        //    printf_err("hal_get_image_data failed");
        //    return false;
        //}

        {
            //ScopedLVGLLock lv_lock;

            /* Display this image on the LCD. */
            // write_to_lvgl_buf(192, 192,
            //                 currImage, &lvgl_image[0][0]);
            // lv_obj_invalidate(alif::app::ScreenLayoutImageObject());

            // if (!run_requested()) {
            //    lv_led_off(alif::app::ScreenLayoutLEDObject());
            //    return false;
            // }

            //lv_led_on(alif::app::ScreenLayoutLEDObject());

        }
    }

    hal_platform_release();
}
