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
#include "hal/RTE_Components.h"
#include CMSIS_device_header
#include "hal/hal.h"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/ei_utils.h"

#define G_TO_MS 9.80665

extern "C" {
    #include "hal/uart_stdout.h"
    #include "hal/drv_lptimer.h"
    #include "hal/BMI323_imu_driver.h"
}

#include <cstdio>

volatile uint32_t ms_ticks = 0;
void delay(uint32_t nticks) { nticks += ms_ticks; while(ms_ticks < nticks); }
#define TICKS_PER_SECOND    100

#define IMU_SAMPLE_RATE		(62.5)
#define LPTIMER_INSTANCE	2

static float raw_features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

#if !NDEBUG
extern "C" void __stack_chk_fail(void) { 
    ei_printf("Stack overflow caught\n");
    while (1) {} 
} // trap stack overflow
void* __stack_chk_guard = (void*)0xaeaeaeae;
#endif

void calibtrate_IMU(xyz_accel_s *calibration)
{
	xyz_accel_s motion, temp;
	motion.x = 0;
	motion.y = 0;
	motion.z = 0;

	/* Use 100 or less, otherwise int16_t will overflow */
	for (int i = 0; i < 100; i++) {
		IMU_ACC_Get(&temp);
		motion.x += temp.x;
		motion.y += temp.y;
		motion.z += temp.z;
	}
	motion.x = motion.x / 100;
	motion.y = motion.y / 100;
	motion.z = motion.z / 100;

	*calibration = motion;
}

int main()
{
    // System init takes place in Reset function, see irqs.c

    #if defined(ARM_NPU)

    /* If Arm Ethos-U NPU is to be used, we initialise it here */
    if (0 != arm_npu_init()) {
        ei_printf("Failed to initialize NPU");
    }

    #endif /* ARM_NPU */

    stdout_init();

    xyz_accel_s motion, calibration;
	  IMU_Init(i2c_addr_68);

    /* Optional 'calibration' step */
    // calibtrate_IMU(&calibration);

	  /* Initialize timer to poll IMU at specified rate */
    LPTIMER_ll_Initialize(LPTIMER_INSTANCE);
    LPTIMER_ll_Set_Count_Value(LPTIMER_INSTANCE, LPTIMER_TICK_RATE / IMU_SAMPLE_RATE);

    /* Start timer and poll interrupt register */
    LPTIMER_ll_Start(LPTIMER_INSTANCE);

    while (1) {
        for (int i = 0; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 2; i += 3) {
            LPTIMER_ll_Wait(LPTIMER_INSTANCE);
            IMU_ACC_Get(&motion);

            raw_features[i + 0] = (-1.0 * (float) (motion.x - calibration.x)) / 4096.0 * G_TO_MS;
            raw_features[i + 1] = (-1.0 * (float) (motion.y - calibration.y)) / 4096.0 * G_TO_MS;
            raw_features[i + 2] = ((float) (motion.z - calibration.z)) / 4096.0 * G_TO_MS;

            	/* Uncomment this line, and comment out the lines 120 to 134, to collect data
            	 * via the edge-impulse-data-forwarder cli:
            	 * 	https://docs.edgeimpulse.com/docs/edge-impulse-cli/cli-data-forwarder
               */
		        // ei_printf("%.9g, %.9g,%.9g\n", raw_features[i + 0], raw_features[i + 1], raw_features[i + 2]);
        }

         ei_impulse_result_t result;
 
         signal_t signal;
         numpy::signal_from_buffer(&raw_features[0], ARRAY_LENGTH(raw_features), &signal);
 
         EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

         ei_printf("\nrun_classifier returned: %d (DSP %lld us., Classification %lld us., Anomaly %d ms.)\n", res, result.timing.dsp_us, result.timing.classification_us, result.timing.anomaly);
         ei_printf("\nInference Results:\n");
         for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
             ei_printf("%s: %.9g\n", result.classification[ix].label, result.classification[ix].value);
         }
 #if EI_CLASSIFIER_HAS_ANOMALY == 1
         ei_printf("anomaly score: %.9g\n", result.anomaly);
 #endif
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
