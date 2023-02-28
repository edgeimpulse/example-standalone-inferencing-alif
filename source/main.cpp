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

#include <stdio.h>
#include "hal.h"
#include "hal_image.h"
#include "log_macros.h"

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/ei_utils.h"

#include <cstdio>

#define AUDIO_SAMPLES 16000 // 16k samples/sec, 1sec sample
#define AUDIO_STRIDE 8000 // 0.5 seconds
#define RESULTS_MEMORY 8

static int16_t audio_inf[AUDIO_SAMPLES + AUDIO_STRIDE];

#if !NDEBUG
extern "C" void __stack_chk_fail(void) { 
    ei_printf("Stack overflow caught\n");
    while (1) {} 
} // trap stack overflow
void* __stack_chk_guard = (void*)0xaeaeaeae;
#endif

/* Print application information. */
static void print_application_intro()
{
    info("copyright 2021-2022 arm limited and/or its affiliates <open-source-office@arm.com>\n\n");
}



/* Project Includes */
int main()
{
    info("Initializing Platform...\r\n");
    if (hal_platform_init()) {
        /* Application information, UART should have been initialised. */
        print_application_intro();
        info("Initializing Platform OK\r\n");

        info("Initializing Audio...\r\n");
        int err = hal_audio_init(16000, 32);
        if (err) {
            printf_err("hal_audio_init failed with error: %d\n", err);
        }
        info("Initializing Audio OK\r\n");

        info("Initializing Camera...\r\n");
        // hal_image_init();
        info("Initializing Camera OK\r\n");

        info("Starting audio sampling...\r\n");
        // Start first fill of final stride section of buffer
        hal_get_audio_data(audio_inf + AUDIO_SAMPLES, AUDIO_STRIDE);

        do {
            // Wait until stride buffer is full - initiated above or by previous interation of loop
            int err = hal_wait_for_audio();
            if (err) {
                printf_err("hal_get_audio_data failed with error: %d\n", err);
            }
            for (int16_t sample : audio_inf) {
                printf("%d\n", sample);
            }
        } while(1);
    }

    printf_err("Initializing Platform failed\r\n");
    hal_platform_release();
}
