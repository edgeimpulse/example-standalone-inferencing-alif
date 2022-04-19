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
#include "firmware-sdk/at_base64_lib.h"

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


static ei_device_sensor_t sensor_list[] = {
    { 
        .name = "Microphone",
        .frequencies = { 16000.0 },
        .max_sample_length_s = 2,
        .start_sampling_cb = ei_microphone_sample_record
    }
};

class EiCameraAlif : public EiCamera
{
    virtual bool ei_camera_capture_rgb888_packed_big_endian(
        uint8_t *image,
        uint32_t image_size) override
    {
        camera_start(CAMERA_MODE_SNAPSHOT);
        camera_wait(100);
        // RGB conversion and frame resize
        bayer_to_RGB(raw_image + 0x460, image);
    }

    /**
     * @brief Get the list of supported resolutions, ie. not requiring
     * any software processing like crop or resize
     * 
     * @param res pointer to store the list of resolutions
     * @param res_num pointer to a variable that will contain size of the res list
     */
    virtual void get_resolutions(const ei_device_snapshot_resolutions_t **res, uint8_t *res_num) override
    {

        static ei_device_snapshot_resolutions_t snapshot_resolutions[] =
            { { 320,240 },
              { 32,32 } };

        *res = snapshot_resolutions;
        *res_num = ARRAY_LENGTH(snapshot_resolutions);
    }

    virtual ei_device_snapshot_resolutions_t get_min_resolution(void)
    {
        return { 32,32 };
    }

    virtual bool set_resolution(const ei_device_snapshot_resolutions_t res) override
    {
        return true;
    }
};

EiCamera* EiCamera::get_camera()
{
    static EiCameraAlif cam;
    return &cam;
}

class EiDeviceAlif : public EiDeviceInfo
{
public:
    bool get_sensor_list(const ei_device_sensor_t **p_sensor_list, size_t *sensor_list_size) override
    {
        *p_sensor_list = sensor_list;
        *sensor_list_size = ARRAY_LENGTH(sensor_list);
        return true;
    }

    bool read_encode_send_sample_buffer(size_t address, size_t length) override
    {
        if(sample_buffer) {
            base64_encode((char *)sample_buffer+address, length, ei_putchar);
            return true;
        } else {
            return false;
        }
    }

    bool get_snapshot_list(
        const ei_device_snapshot_resolutions_t **snapshot_list,
        size_t *snapshot_list_size,
        const char **color_depth) override
    {
        auto cam = EiCamera::get_camera();
        uint8_t size;
        cam->get_resolutions(snapshot_list, &size);
        *snapshot_list_size = size;
        static const char color[] = "RGB";
        static const char* p_color = color;
        *color_depth = p_color;
        return false;
    }
};

EiDeviceInfo *EiDeviceInfo::get_device()
{
    static EiDeviceAlif dev;
    return &dev;
}

extern bool ei_microphone_sample_record(void)
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
