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

static const float raw_features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {
    // copy raw features here (for example from the 'Live classification' page)
    // see TBD
};

#if !NDEBUG
extern "C" void __stack_chk_fail(void) { 
    ei_printf("Stack overflow caught\n");
    while (1) {} 
} // trap stack overflow
void* __stack_chk_guard = (void*)0xaeaeaeae;
#endif


/* Project Includes */
#include "hal/Driver_SAI.h"
#include "hal/Driver_PINMUX_AND_PINPAD.h"

#include "hal/RTE_Device.h"
#include "hal/RTE_Components.h"

#include "hal/bsp_core_log.h"

#define I2S_DAC 0               /* DAC I2S Controller 0 */
#define I2S_ADC 2               /* ADC I2S Controller 2 */

extern ARM_DRIVER_SAI ARM_Driver_SAI_(I2S_ADC);

#define AUDIO_SAMPLE_NUM        (5)
#define AUDIO_SAMPLE_SIZE       (1 << 12)	// 4096
#define AUDIO_BUFFER_SIZE       (AUDIO_SAMPLE_NUM*AUDIO_SAMPLE_SIZE)

ARM_DRIVER_SAI      *i2s_drv;
ARM_DRIVER_VERSION   version;
ARM_SAI_CAPABILITIES cap;

uint32_t wlen = 16;
uint32_t sampling_rate = 16000;

//int16_t audio_samples[AUDIO_SAMPLE_NUM][AUDIO_SAMPLE_SIZE] __attribute__((section(".ARM.__at_0x02000000")));

int16_t audio0[64000];

uint32_t volatile i2s_callback_flag;
/**
\fn          void i2s_callback(uint32_t event)
\brief       Callback routine from the i2s driver
\param[in]   event Event for which the callback has been called
*/
void i2s_callback(uint32_t event)
{
    if (event & ARM_SAI_EVENT_RECEIVE_COMPLETE)
    {
        /* Receive Success */
        i2s_callback_flag = 1;
    }
    if (event & ARM_SAI_EVENT_RX_OVERFLOW)
    {

    }
}

int init_data_acq_kws(int idx)
{
    int32_t status = 0;

      /* Configure the adc pins */
   /* Configure P2_1.I2S2_SDI_A */
    status |= PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_1, PINMUX_ALTERNATE_FUNCTION_3);
    status |= PINPAD_Config(PORT_NUMBER_2, PIN_NUMBER_1, PAD_FUNCTION_DRIVER_DISABLE_STATE_WITH_PULL_DOWN | PAD_FUNCTION_READ_ENABLE);

    /* Configure P2_3.I2S2_SCLK_A */
    status |= PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_3, PINMUX_ALTERNATE_FUNCTION_3);
    status |= PINPAD_Config(PORT_NUMBER_2, PIN_NUMBER_3, PAD_FUNCTION_READ_ENABLE);

    /* Configure P2_3.I2S2_WS_A */
    status |= PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_4, PINMUX_ALTERNATE_FUNCTION_2);
    status |= PINPAD_Config(PORT_NUMBER_2, PIN_NUMBER_4, PAD_FUNCTION_READ_ENABLE);
    if (status)
    {
        printf_err("I2S pinmux configuration failed\n");
        return -1;
    }
    
   /* Use the I2S as Receiver */
    i2s_drv = &ARM_Driver_SAI_(I2S_ADC);

    /* Verify the I2S API version for compatibility*/
    version = i2s_drv->GetVersion();
    info("I2S API version = %d\n", version.api);

    /* Verify if I2S protocol is supported */
    ARM_SAI_CAPABILITIES cap = i2s_drv->GetCapabilities();
    if (!cap.protocol_i2s)
    {
        printf_err("I2S is not supported\n");
        return -1;
    }

    /* Initializes I2S interface */
    status = i2s_drv->Initialize(i2s_callback);
    if (status)
    {
        printf_err("I2S Init failed status = %d\n", status);
        goto error_i2s_initialize;
    }

    /* Enable the power for I2S */
    status = i2s_drv->PowerControl(ARM_POWER_FULL);
    if (status)
    {
        printf_err("I2S Power failed status = %d\n", status);
        goto error_i2s_power;
    }

    /* configure I2S Receiver to Asynchronous Master */
    status = i2s_drv->Control(ARM_SAI_CONFIGURE_RX |
                              ARM_SAI_MODE_MASTER  |
                              ARM_SAI_MONO_MODE    |
                              ARM_SAI_ASYNCHRONOUS |
                              ARM_SAI_PROTOCOL_I2S |
                              ARM_SAI_DATA_SIZE(wlen), wlen*2, sampling_rate);
    if (status)
    {
        printf_err("I2S Control status = %d\n", status);
        goto error_i2s_control;
    }

    return 0;

error_i2s_control:
    i2s_drv->PowerControl(ARM_POWER_OFF);
error_i2s_power:
    i2s_drv->Uninitialize();
error_i2s_initialize:
    return -1;
}

int get_data_kws(int idx)
{
    int32_t status = 0;
    i2s_callback_flag = 0;

    /* Enable Receiver */
    status = i2s_drv->Control(ARM_SAI_CONTROL_RX, 1, 0);
    if (status)
    {
        printf_err("I2S Control RX start status = %d\n", status);
        return -1;
    }

    /* Receive data */
    status = i2s_drv->Receive(&audio0[0], 64000);
    //status = i2s_drv->Receive(&audio0[sample_index*4096], AUDIO_SAMPLE_SIZE);
    //status = i2s_drv->Receive(&audio_samples[sample_index][0], AUDIO_SAMPLE_SIZE);
    if (status)
    {
        printf_err("I2S Receive status = %d\n", status);
        return -1;
    }

    /* Wait for the completion event */
    while(1) {
        /*TODO: Add timeout */
        if (i2s_callback_flag) {
            break;
        }
    }

    /* Stop the RX */
    status = i2s_drv->Control(ARM_SAI_CONTROL_RX, 0, 0);
    if (status)
    {
        printf_err("I2S Control RX stop status = %d\n", status);
        return -1;
    }

    return 0;
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

    UartStdOutInit();

    init_data_acq_kws(0);

    get_data_kws(0);

    for( auto val : audio0 ) {
        ei_printf("%hi,",val);
    }

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
