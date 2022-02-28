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

#include "firmware-sdk/ei_microphone_lib.h"
#include "edge-impulse-sdk/CMSIS/DSP/Include/dsp/support_functions.h"

//TODO: use multiply of memory block size
#define MIC_SAMPLE_SIZE 2048

/** Status and control struct for inferencing struct */
typedef struct
{
    int16_t *buffers[2];
    uint8_t buf_select;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

static inference_t inference;
//TODO malloc
microphone_sample_t sample_buffer[MIC_SAMPLE_SIZE * 2];
microphone_sample_t sample_buffer_processed[MIC_SAMPLE_SIZE];

extern bool ei_microphone_sample_start(void)
{
    return ei_microphone_sample_start_lib([] { return 0; },
                                   [] { return dummy_mic_start(sample_buffer, MIC_SAMPLE_SIZE); },
                                   [] { return 0; });
}

static void inference_samples_callback(const int16_t *buffer, uint32_t sample_count)
{
    for (uint32_t i = 0; i < sample_count; i++)
    {
        if (inference.buf_count >= inference.n_samples)
        {
            inference.buf_select ^= 1;
            inference.buf_count = 0;
            inference.buf_ready = 1;
        }
    }
}

bool ei_microphone_inference_start(uint32_t n_samples, float interval_ms)
{
    inference.buffers[0] = (int16_t *)ei_malloc(n_samples * sizeof(microphone_sample_t));
    if (inference.buffers[0] == NULL)
    {
        return false;
    }

    inference.buffers[1] = (int16_t *)ei_malloc(n_samples * sizeof(microphone_sample_t));
    if (inference.buffers[1] == NULL)
    {
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
    dummy_mic_start(sample_buffer, MIC_SAMPLE_SIZE);

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
