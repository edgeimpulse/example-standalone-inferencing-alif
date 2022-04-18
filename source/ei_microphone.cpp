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

#include "ei_microphone.h"

#include "edge-impulse-sdk/CMSIS/DSP/Include/dsp/support_functions.h"
#include "edge-impulse-sdk/dsp/memory.hpp"
#include "firmware-sdk/ei_microphone_lib.h"
#include "edge-impulse-sdk/dsp/ei_utils.h"

extern "C"{
#include "hal/Driver_SAI.h"
#include "hal/Driver_PINMUX_AND_PINPAD.h"
}

//TODO: use multiply of memory block size
#define MIC_SAMPLE_SIZE 2048

/** Status and control struct for inferencing struct */
typedef struct {
    int16_t *buffers[2];
    uint8_t buf_select;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

static inference_t inference;

static EiMicrophoneDummy mic;
static microphone_sample_t* sample_buffer = nullptr;


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

static ei_device_sensor_t sensor_list[] = {
    { 
        .name = "Microphone",
        .frequencies = { 16000.0 },
        .max_sample_length_s = 2,
        .start_sampling_cb = ei_microphone_sample_record
    }
};


class EiDeviceAlif : public EiDeviceInfo
{
public:
    bool get_sensor_list(const ei_device_sensor_t **p_sensor_list, size_t *sensor_list_size) override
    {
        *p_sensor_list = sensor_list;
        *sensor_list_size = ARRAY_LENGTH(sensor_list);
        return true;
    }

    bool read_encode_send_sample_buffer(size_t address, size_t length)
    {
        if(sample_buffer) {
            base64_encode((char *)sample_buffer+address, length, ei_putchar);
            return true;
        } else {
            return false;
        }
    }
};

EiDeviceInfo *EiDeviceInfo::get_device()
{
    static EiDeviceAlif dev;
    return &dev;
}

int ei_microphone_init(int idx)
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
        ei_printf("I2S pinmux configuration failed\n");
        return -1;
    }
    
   /* Use the I2S as Receiver */
    i2s_drv = &ARM_Driver_SAI_(I2S_ADC);

    /* Verify the I2S API version for compatibility*/
    version = i2s_drv->GetVersion();
    // info("I2S API version = %d\n", version.api);

    /* Verify if I2S protocol is supported */
    ARM_SAI_CAPABILITIES cap = i2s_drv->GetCapabilities();
    if (!cap.protocol_i2s)
    {
        ei_printf("I2S is not supported\n");
        return -1;
    }

    /* Initializes I2S interface */
    status = i2s_drv->Initialize(i2s_callback);
    if (status)
    {
        ei_printf("I2S Init failed status = %d\n", status);
        goto error_i2s_initialize;
    }

    /* Enable the power for I2S */
    status = i2s_drv->PowerControl(ARM_POWER_FULL);
    if (status)
    {
        ei_printf("I2S Power failed status = %d\n", status);
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
        ei_printf("I2S Control status = %d\n", status);
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

bool ei_microphone_sample_record(void)
{
    auto size = ei_microphone_get_buffer_size();
    if(sample_buffer) {
        ei_free(sample_buffer);
    }
    sample_buffer = (microphone_sample_t*)ei_malloc(size);
    if (!sample_buffer) {
        ei_printf("Failed to allocate memory for audio sampling");
        return false;
    }
    EiDeviceRAM ram;
    ram.assign_memory((uint32_t*)sample_buffer, size);
    return ei_microphone_sample_record_lib(&mic, &ram);
}

static void inference_samples_callback(const int16_t *buffer, uint32_t sample_count)
{
    for (uint32_t i = 0; i < sample_count; i++) {
        if (inference.buf_count >= inference.n_samples) {
            inference.buf_select ^= 1;
            inference.buf_count = 0;
            inference.buf_ready = 1;
        }
    }
}

bool ei_microphone_inference_start(uint32_t n_samples, float interval_ms)
{
    // free up the sampling buffer
    if(sample_buffer) {
        ei_free(sample_buffer);
        sample_buffer = nullptr;
    }

    inference.buffers[0] = (int16_t *)ei_malloc(n_samples * sizeof(microphone_sample_t));
    if (inference.buffers[0] == NULL) {
        return false;
    }

    inference.buffers[1] = (int16_t *)ei_malloc(n_samples * sizeof(microphone_sample_t));
    if (inference.buffers[1] == NULL) {
        delete inference.buffers[0];
        return false;
    }

    inference.buf_select = 0;
    inference.buf_count = 0;
    inference.n_samples = n_samples;
    inference.buf_ready = 0;

    //TODO
    //sl_mic_init((uint32_t)(1000.0f / dev->get_sample_interval_ms()), 1);
    //sl_mic_start_streaming(sample_buffer, MIC_SAMPLE_SIZE, (sl_mic_buffer_ready_callback_t) inference_samples_callback);

    return true;
}

bool ei_microphone_inference_is_recording(void)
{
    return inference.buf_ready == 0;
}

void ei_microphone_inference_reset_buffers(void)
{
    inference.buf_ready = 0;
    inference.buf_count = 0;
}

bool ei_microphone_inference_end(void)
{
    //TODO
    // sl_mic_stop();

    ei_free(inference.buffers[0]);
    ei_free(inference.buffers[1]);

    return true;
}
