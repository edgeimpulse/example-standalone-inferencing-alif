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
#include "RTE_Components.h"
#include CMSIS_device_header
#include "hal.h"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/classifier/ei_classifier_smooth.h"
#include "edge-impulse-sdk/dsp/ei_utils.h"
#include "lvgl.h"

#define G_TO_MS 9.80665

extern "C" void lv_port_disp_init();
extern "C" {
    #include "uart_stdout.h"
    #include "drv_lptimer.h"
    #include "BMI323_imu_driver.h"
}

#include <cstdio>

volatile uint32_t ms_ticks = 0;
void delay(uint32_t nticks) { nticks += ms_ticks; while(ms_ticks < nticks); }
#define TICKS_PER_SECOND    1000

#define IMU_SAMPLE_RATE		(25)
#define LPTIMER_INSTANCE	2

static float raw_features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};
static size_t raw_features_index = 0;
static ei_classifier_smooth smooth_data;
static const char * processed_label;

enum DemoState {
    START,
    FIRST_COOLDOWN,
    FIRST_LETTER,
    SECOND_COOLDOWN,
    SECOND_LETTER,
    MAYDAY,
    MAYDAY_COOLDOWN,
    ERROR,
};

#define STATE_TIMEOUT 100
#define STATE_COOLDOWN 30
static DemoState state = START;
static size_t state_counter;

#if !NDEBUG
extern "C" void __stack_chk_fail(void) {
    ei_printf("Stack overflow caught\n");
    while (1) {}
} // trap stack overflow
void* __stack_chk_guard = (void*)0xaeaeaeae;
#endif

#define LABEL_WIDTH  320
#define LABEL_HEIGHT 80
static lv_obj_t *sos_banner;
static lv_obj_t *classification_label;
static lv_obj_t *classification_state;
static lv_obj_t *classification_time_dsp, *classification_time_nn;
static lv_style_t style_label_48pt, style_label_24pt;
static lv_style_t style_sos_banner;

static void lv_create_labels()
{
    lv_style_init(&style_label_48pt);
    lv_style_init(&style_label_24pt);
    lv_style_set_text_font(&style_label_48pt, &lv_font_montserrat_48);
    lv_style_set_text_font(&style_label_24pt, &lv_font_montserrat_24);

    classification_label    = lv_label_create(lv_scr_act());
    classification_state    = lv_label_create(lv_scr_act());
    classification_time_dsp = lv_label_create(lv_scr_act());
    classification_time_nn  = lv_label_create(lv_scr_act());

    lv_obj_set_size(classification_label, LABEL_WIDTH, LABEL_HEIGHT);
    lv_obj_align(classification_label, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_size(classification_state, LABEL_WIDTH, LABEL_HEIGHT);
    lv_obj_align(classification_state, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_size(classification_time_dsp, LABEL_WIDTH, LABEL_HEIGHT);
    lv_obj_set_size(classification_time_nn,  LABEL_WIDTH, LABEL_HEIGHT);
    lv_obj_align(classification_time_dsp, LV_ALIGN_BOTTOM_MID, 0, -LABEL_HEIGHT);
    lv_obj_align(classification_time_nn,  LV_ALIGN_BOTTOM_MID, 0, 0);

    lv_obj_add_style(classification_label, &style_label_48pt, 0);
    lv_obj_add_style(classification_state, &style_label_48pt, 0);
    lv_obj_add_style(classification_time_dsp, &style_label_24pt, 0);
    lv_obj_add_style(classification_time_nn,  &style_label_24pt, 0);

    lv_obj_set_style_text_align(classification_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_align(classification_state, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_align(classification_time_dsp, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_align(classification_time_nn,  LV_TEXT_ALIGN_CENTER, 0);

    lv_style_init(&style_sos_banner);
    lv_style_set_bg_opa(&style_sos_banner, LV_OPA_0);

    sos_banner = lv_btn_create(lv_scr_act());
    lv_obj_set_size(sos_banner, 480, 854);
    lv_obj_center(sos_banner);
    lv_obj_add_style(sos_banner, &style_sos_banner, 0);
}

static void lv_flash_screen()
{
    int count = 15;

    /* Initialize timer to poll IMU at specified rate */
    LPTIMER_ll_Initialize(0);
    LPTIMER_ll_Set_Count_Value(0, LPTIMER_TICK_RATE / 3);
    LPTIMER_ll_Start(0);

    lv_style_set_bg_opa(&style_sos_banner, LV_OPA_100);

    while (count-- != 0)
    {
        lv_style_set_bg_color(&style_sos_banner, lv_palette_main(LV_PALETTE_RED));
        lv_obj_remove_style(sos_banner, &style_sos_banner, 0);
        lv_obj_add_style(sos_banner, &style_sos_banner, 0);
        lv_task_handler();
        LPTIMER_ll_Wait(0);

        lv_style_set_bg_color(&style_sos_banner, lv_palette_main(LV_PALETTE_ORANGE));
        lv_obj_remove_style(sos_banner, &style_sos_banner, 0);
        lv_obj_add_style(sos_banner, &style_sos_banner, 0);
        lv_task_handler();
        LPTIMER_ll_Wait(0);
    }

    lv_style_set_bg_opa(&style_sos_banner, LV_OPA_0);
    lv_obj_remove_style(sos_banner, &style_sos_banner, 0);
    lv_obj_add_style(sos_banner, &style_sos_banner, 0);
    lv_task_handler();

    LPTIMER_ll_Stop(0);
}

static char motion_label[4][8] = {"Idle\0", "Snake\0", "Updown\0", "Wave\0"};
static char time_str[24];

static void lv_update_label(ei_impulse_result_t const * result)
{
    uint32_t index, count = 0;
    float value;

    index = 0;
    count = 0;
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        if (result->classification[ix].value > 0.85) {
						value = result->classification[ix].value;
            index = ix;
            count++;
        }
    }

    switch (state) {
        case START:
            lv_label_set_text_fmt(classification_state, "%s", "_. _. _.\0");
            break;
        case FIRST_COOLDOWN:
            lv_label_set_text_fmt(classification_state, "%s", "S. _. _.\0");
            break;
        case FIRST_LETTER:
            lv_label_set_text_fmt(classification_state, "%s", "S. _. _.\0");
            break;
        case SECOND_COOLDOWN:
            lv_label_set_text_fmt(classification_state, "%s", "S. O. _.\0");
            break;
        case SECOND_LETTER:
            lv_label_set_text_fmt(classification_state, "%s", "S. O. _.\0");
            break;
        case MAYDAY:
            lv_label_set_text_fmt(classification_state, "%s", "S. O. S.\0");
            break;
        case ERROR:
            lv_label_set_text_fmt(classification_state, "%s", "State Error!\0");
            break;
    }

    lv_label_set_text_fmt(classification_label, "%s", processed_label);

    sprintf(time_str, "DSP: %d us.", result->timing.dsp_us);
    lv_label_set_text_fmt(classification_time_dsp, "%s", time_str);

    sprintf(time_str, "NN: %d us.", result->timing.classification_us);
    lv_label_set_text_fmt(classification_time_nn, "%s", time_str);
}

static int get_signal_data(size_t offset, size_t length, float *out_ptr) {
    for (size_t i = 0; i < length; i++) {
        size_t input_index = (i + offset + raw_features_index) % length;
        out_ptr[i] = raw_features[input_index];
    }
    return 0;
}

static void calibtrate_IMU(xyz_accel_s *calibration)
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

    lv_port_disp_init();
    lv_create_labels();

    xyz_accel_s motion, calibration;
    IMU_Init(i2c_addr_68);

    /* Optional 'calibration' step */
    calibtrate_IMU(&calibration);

    /* Initialize timer to poll IMU at specified rate */
    LPTIMER_ll_Initialize(LPTIMER_INSTANCE);
    LPTIMER_ll_Set_Count_Value(LPTIMER_INSTANCE, LPTIMER_TICK_RATE / IMU_SAMPLE_RATE);

    /* Start timer and poll interrupt register */
    LPTIMER_ll_Start(LPTIMER_INSTANCE);

    for (int i = 0; i < (EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3); i += 3) {
        LPTIMER_ll_Wait(LPTIMER_INSTANCE);
        IMU_ACC_Get(&motion);

        raw_features[i + 0] = (-1.0 * (float) (motion.x - calibration.x)) / 4096.0 * G_TO_MS;
        raw_features[i + 1] = (-1.0 * (float) (motion.y - calibration.y)) / 4096.0 * G_TO_MS;
        raw_features[i + 2] = ((float) (motion.z - calibration.z)) / 4096.0 * G_TO_MS;
    }

    ei_classifier_smooth_init(&smooth_data, 32, 10, 0.85);

    while (1) {

        ei_impulse_result_t result;

        signal_t signal;
        signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
        signal.get_data = &get_signal_data;

        EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
        processed_label = ei_classifier_smooth_update(&smooth_data, &result);

        // Update State
        char match_char = processed_label[0]; // match only on first char of label, assumes labels are:
                                         // uncertain, unknown, S, O
        DemoState next_state;
        if (state_counter > STATE_TIMEOUT) {
            next_state = START;
            state_counter = 0;
        } else {
            switch (state) {

                case START:
                    if (match_char == 'u' || match_char == 'O') {
                        next_state = state;
                    } else if (match_char == 'S') {
                        next_state = FIRST_COOLDOWN;
                        state_counter = 0;
                    } else {
                        next_state = ERROR;
                    }
                    break;

                case FIRST_COOLDOWN:
                    if (state_counter > STATE_COOLDOWN) {
                        next_state = FIRST_LETTER;
                        state_counter = 0;
                    } else {
                        next_state = state;
                    }
                    break;

                case FIRST_LETTER: // letter S
                    if (match_char == 'u' || match_char == 'S') {
                        next_state = state;
                    } else if (match_char == 'O') {
                        next_state = SECOND_COOLDOWN;
                        state_counter = 0;
                    } else {
                        next_state = ERROR;
                    }
                    break;

                case SECOND_COOLDOWN:
                    if (state_counter > STATE_COOLDOWN) {
                        next_state = SECOND_LETTER;
                        state_counter = 0;
                    } else {
                        next_state = state;
                    }
                    break;

                case SECOND_LETTER: // letter O
                    if (match_char == 'u' || match_char == 'O') {
                        next_state = state;
                    } else if (match_char == 'S') {
                        next_state = MAYDAY;
                        state_counter = 0;
                    } else {
                        next_state = ERROR;
                    }
                    break;
                case MAYDAY: // letter S
                        next_state = MAYDAY_COOLDOWN;
                        state_counter = 0;
                        lv_flash_screen();
                    break;
                case MAYDAY_COOLDOWN: // long cooldown, waits for whole counter to reset
                    if (state_counter > STATE_COOLDOWN) {
                        next_state = START;
                        state_counter = 0;
                    } else {
                        next_state = state;
                    }
                    break;
                case ERROR:
                    next_state = ERROR; // show error until counter resets
            }
        }

        state = next_state;
        state_counter++;

        lv_update_label(&result);
        lv_task_handler();

//         if ((i % 62) == 0) {
//             ei_printf("run_classifier returned: %d (DSP %lld us., Classification %lld us., Anomaly %d ms.)\n", res,
//                     result.timing.dsp_us, result.timing.classification_us, result.timing.anomaly);
//
//             ei_printf("Begin output:\n");
//
//             // print the predictions
//             ei_printf("[");
//             for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
//                 ei_printf("%.5f", result.classification[ix].value);
// #if EI_CLASSIFIER_HAS_ANOMALY == 1
//                 ei_printf(", ");
// #else
//                 if (ix != EI_CLASSIFIER_LABEL_COUNT - 1) {
//                     ei_printf(", ");
//                 }
// #endif
//             }
// #if EI_CLASSIFIER_HAS_ANOMALY == 1
//             ei_printf("%.3f", result.anomaly);
// #endif
//             ei_printf("]\n");
//             ei_printf("End output\n");
//         }

        LPTIMER_ll_Wait(LPTIMER_INSTANCE);
        IMU_ACC_Get(&motion);

        raw_features[raw_features_index + 0] = (-1.0 * (float) (motion.x - calibration.x)) / 4096.0 * G_TO_MS;
        raw_features[raw_features_index + 1] = (-1.0 * (float) (motion.y - calibration.y)) / 4096.0 * G_TO_MS;
        raw_features[raw_features_index + 2] = ((float) (motion.z - calibration.z)) / 4096.0 * G_TO_MS;

        // print compatible with our data forwarder for simple sampling
        ei_printf("%.9g, %.9g,%.9g\n",
                raw_features[raw_features_index + 0],
                raw_features[raw_features_index + 1],
                raw_features[raw_features_index + 2]);

        if (raw_features_index < (EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3))
            raw_features_index += 3;
        else
            raw_features_index = 0;
    }

    while (1) {
        lv_task_handler();
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
