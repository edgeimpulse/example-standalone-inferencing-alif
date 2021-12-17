#include "hal.h"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/ei_utils.h"
#include "features.h"

#include <cstdio>

int main()
{
    // System init takes place in Reset function, see irqs.c

    #if defined(ARM_NPU)

    /* If Arm Ethos-U NPU is to be used, we initialise it here */
    if (0 != arm_npu_init()) {
        ei_printf("Failed to initialize NPU");
    }

    #endif /* ARM_NPU */

    ei_impulse_result_t result;

    signal_t signal;
    numpy::signal_from_buffer(&raw_features[0], ARRAY_LENGTH(raw_features), &signal);

    EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
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
