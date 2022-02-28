#ifndef EI_MICROPHONE_H
#define EI_MICROPHONE_H

#include <cstdint>
#include <cstdlib>

typedef int16_t microphone_sample_t;

extern bool ei_microphone_sample_start(void);
int ei_microphone_inference_get_data(size_t offset, size_t length, float *out_ptr);
bool ei_microphone_inference_start(uint32_t n_samples, float interval_ms);
bool ei_microphone_inference_is_recording(void);
void ei_microphone_inference_reset_buffers(void);
bool ei_microphone_inference_end(void);

#endif /* EI_MICROPHONE_H */