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

	int cinit = camera_init(raw_image);
	if (cinit != 0) {
		while(1) {
			Driver_GPIO1.SetValue(PIN_NUMBER_14, GPIO_PIN_OUTPUT_STATE_LOW);
			ei_sleep(300);
			Driver_GPIO1.SetValue(PIN_NUMBER_14, GPIO_PIN_OUTPUT_STATE_HIGH);
			ei_sleep(300);
		}
	}
	ei_printf("Camera initialized... \n");

    camera_start(CAMERA_MODE_SNAPSHOT);
    camera_vsync(100);
    // RGB conversion and frame resize
    bayer_to_RGB(raw_image+0x460, rgb_image);

    for( int i=0; i<999; i++ ) {
        ei_printf("%hi,",rgb_image[i]);
    }

    ei_printf("\n\nEnd of stream\n");

    while(1);

//     ei_impulse_result_t result;

//     signal_t signal;
//     numpy::signal_from_buffer(&raw_features[0], ARRAY_LENGTH(raw_features), &signal);

//     EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
//     ei_printf("run_classifier returned: %d (DSP %lld us., Classification %lld us., Anomaly %d ms.)\n", res,
//         result.timing.dsp_us, result.timing.classification_us, result.timing.anomaly);

//     ei_printf("Begin output\n");

// #if EI_CLASSIFIER_OBJECT_DETECTION == 1
//     for (size_t ix = 0; ix < EI_CLASSIFIER_OBJECT_DETECTION_COUNT; ix++) {
//         auto bb = result.bounding_boxes[ix]; 
//         if (bb.value == 0) {
//             continue;
//         }

//         ei_printf("%s (%f) [ x: %u, y: %u, width: %u, height: %u ]\n", bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
//     }
// #else
//     // print the predictions
//     ei_printf("[");
//     for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
//         ei_printf("%.5f", result.classification[ix].value);
// #if EI_CLASSIFIER_HAS_ANOMALY == 1
//         ei_printf(", ");
// #else
//         if (ix != EI_CLASSIFIER_LABEL_COUNT - 1) {
//             ei_printf(", ");
//         }
// #endif
//     }
// #if EI_CLASSIFIER_HAS_ANOMALY == 1
//     ei_printf("%.3f", result.anomaly);
// #endif
//     ei_printf("]\n");
// #endif

//     ei_printf("End output\n");
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
