/* Edge Impulse ingestion SDK
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

/* Include ----------------------------------------------------------------- */
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"
#include "ei_microphone.h"
#include "ei_device_interface.h"

#if defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA
#include "firmware-sdk-alif/ei_image_nn.h"
#include <memory>

void run_nn(bool debug) {

    // size enough for packed RGB888
    const int IMAGE_SIZE = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 3;

    uint8* image_p;
    EI_ALLOCATE_AUTO_POINTER(image_p, IMAGE_SIZE);

    if( !image_p ) {
        ei_printf("run_nn out of memory\n");
        return;
    }

    EiImageNN imageNN(
        image_p,
        IMAGE_SIZE,
        EI_CLASSIFIER_INPUT_WIDTH,
        EI_CLASSIFIER_INPUT_HEIGHT,
        EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE,
        EI_CLASSIFIER_LABEL_COUNT);

    imageNN.run_nn(debug, debug? 0:1000, false);
}


#else

void run_nn(bool debug) {}
#error "EI_CLASSIFIER_SENSOR not configured, cannot configure `run_nn`"

#endif // EI_CLASSIFIER_SENSOR

bool run_nn_normal(void) {
    run_nn(false);
    return true;
}

bool run_nn_debug(const char**, int) {
    run_nn(true);
    return true;
}

bool run_nn_continuous_normal(void) {
#if defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_MICROPHONE
    run_nn_continuous(false);
#else
    ei_printf("Error no continuous classification available for current model\r\n");
#endif
    return true;
}
