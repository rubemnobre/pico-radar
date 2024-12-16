#ifndef FFT_H
#define FFT_H

#include <stdint.h>

int fix_fftr(int16_t f[], int m, int inverse);

void adc12_to_int16(uint16_t inbuf[], int16_t outbuf[], int length);

#endif