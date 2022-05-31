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

#include "hal/hal.h"
#include "hal/uart_stdout.h"
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
#include "Driver_PINMUX_AND_PINPAD.h"
#include "Driver_GPIO.h" 

#include "RTE_Device.h"
#include "RTE_Components.h"

#include "hal/bsp_core_log.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "image_processing.h"

uint8_t raw_image[CIMAGE_X*CIMAGE_Y*RGB_BYTES + 0x460];
uint8_t rgb_image[CIMAGE_X*CIMAGE_Y*RGB_BYTES];
extern ARM_DRIVER_GPIO Driver_GPIO1;

void SetupLEDs()
{
	// Green LED
	Driver_GPIO1.Initialize(PIN_NUMBER_15,NULL);
	Driver_GPIO1.PowerControl(PIN_NUMBER_15,  ARM_POWER_FULL);
	Driver_GPIO1.SetDirection(PIN_NUMBER_15, GPIO_PIN_DIRECTION_OUTPUT);
	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_15, PINMUX_ALTERNATE_FUNCTION_0);
	PINPAD_Config (PORT_NUMBER_1, PIN_NUMBER_15, (0x09 | PAD_FUNCTION_OUTPUT_DRIVE_STRENGTH_04_MILI_AMPS));
	Driver_GPIO1.SetValue(PIN_NUMBER_15, GPIO_PIN_OUTPUT_STATE_LOW);

	// Red LED
	Driver_GPIO1.Initialize(PIN_NUMBER_14,NULL);
	Driver_GPIO1.PowerControl(PIN_NUMBER_14,  ARM_POWER_FULL);
	Driver_GPIO1.SetDirection(PIN_NUMBER_14, GPIO_PIN_DIRECTION_OUTPUT);
	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_14, PINMUX_ALTERNATE_FUNCTION_0);
	PINPAD_Config (PORT_NUMBER_1, PIN_NUMBER_14, (0x09 | PAD_FUNCTION_OUTPUT_DRIVE_STRENGTH_04_MILI_AMPS));
	Driver_GPIO1.SetValue(PIN_NUMBER_14, GPIO_PIN_OUTPUT_STATE_LOW);
}

extern "C" void ei_sleep_c(int32_t time_ms) { ei_sleep( time_ms ); }

int main()
{
    // System init takes place in Reset function, see startup_M55_HP.c
#if defined(ARM_NPU)
    /* If Arm Ethos-U NPU is to be used, we initialize it here */
    if (0 != arm_npu_init()) {
        ei_printf("Failed to initialize NPU");
    }
#endif /* ARM_NPU */


    UartStdOutInit();

    // Setup Pin-Mux and Pad Control registers
    SetupLEDs();
    while(camera_init() !=0) {};
	ei_printf("Camera initialized... \n");

    while (1)
    {
        while (camera_start(CAMERA_MODE_SNAPSHOT, raw_image) != ARM_DRIVER_OK);
        camera_vsync(100);
        // RGB conversion and frame resize
        bayer_to_RGB(raw_image+0x460, rgb_image);

        for( int i=0; i<999; i++ ) {
            ei_printf("%hi,",rgb_image[i]);
        }
        ei_printf("\n\nEnd of stream\n");
    }
}

// remove unneeded bloat

namespace __gnu_cxx
{
    void __verbose_terminate_handler()
    {
        for (;;)
            ;
    }
}
