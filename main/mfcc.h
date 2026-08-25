#ifndef MFCC_H
#define MFCC_H

#include <stdint.h>

#define MFCC_SAMPLE_RATE 16000
#define MFCC_FFT_SIZE 4096
#define MFCC_NUM_MEL 20
#define MFCC_NUM_COEFF 13

void mfcc_init(void);
void compute_mfcc(int16_t *pcm, float *mfcc_out);

#endif