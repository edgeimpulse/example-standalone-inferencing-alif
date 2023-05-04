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

#include "hal.h"
#include "delay.h"
#include "model-parameters/model_metadata.h"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "hal.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"

#include <cstdio>
#include "log_macros.h"

extern "C" void ei_sleep_c(int32_t time_ms) { ei_sleep( time_ms ); }
extern "C" int Init_SysTick(void);

extern "C" unsigned char UartGetc(void);
extern "C" int arm_ethosu_npu_init(void);

static const float raw_features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {
    // copy raw features here (for example from the 'Live classification' page)
};

#if !NDEBUG
extern "C" void __stack_chk_fail(void) { 
    ei_printf("Stack overflow caught\n");
    while (1) {} 
} // trap stack overflow
void* __stack_chk_guard = (void*)0xaeaeaeae;
#endif

int main()
{
    // System init takes place in Reset function, see irqs.c
    init_trigger_rx();
    hal_platform_init();
    info("ei init begins\r\n");
    printf("printf test.\r\n");
    ei_printf("ei_printf test.\r\n");

    #if ARM_NPU
    arm_ethosu_npu_init();
    ei_printf("ARM ethos init\r\n");
    #endif

    sleep_or_wait_msec(10);

    while (1) {
	    signal_t signal;
            ei_impulse_result_t result = { 0 };
	    numpy::signal_from_buffer(&raw_features[0], ARRAY_LENGTH(raw_features), &signal);

	    EI_IMPULSE_ERROR res = run_classifier(&signal, &result, true);
	    ei_printf("run_classifier returned: %d (DSP %lld us., Classification %lld us., Anomaly %d ms.)\n", res,
		result.timing.dsp_us, result.timing.classification_us, result.timing.anomaly);

	    ei_printf("Begin output\n");

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
	    for (size_t ix = 0; ix < EI_CLASSIFIER_OBJECT_DETECTION_COUNT; ix++) {
		auto bb = result.bounding_boxes[ix]; 
		if (bb.value == 0) {
		    continue;
		}

		ei_printf("%s (%f) [ x: %u, y: %u, width: %u, height: %u ]\n", bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
	    }
#else
	    // print the predictions
	    ei_printf("[");
	    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
		ei_printf("%.5f", result.classification[ix].value);
#if EI_CLASSIFIER_HAS_ANOMALY == 1
		ei_printf(", ");
#else
		if (ix != EI_CLASSIFIER_LABEL_COUNT - 1) {
		    ei_printf(", ");
		}
#endif
	    }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
	    ei_printf("%.3f", result.anomaly);
#endif
	    ei_printf("]\n");
#endif

	    ei_printf("End output\n");
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
