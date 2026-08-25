#include "mfcc.h"

#include "math.h"
#include "string.h"
#include "esp_dsp.h"

// ===== Internal buffers =====
static float fft_buffer[MFCC_FFT_SIZE * 2] __attribute__((aligned(16)));
static float power_spectrum[MFCC_FFT_SIZE / 2];
static float mel_energies[MFCC_NUM_MEL];

// ===== Utility =====
static float hz_to_mel(float hz)
{
    return 2595.0f * log10f(1.0f + hz / 700.0f);
}

static float mel_to_hz(float mel)
{
    return 700.0f * (powf(10.0f, mel / 2595.0f) - 1.0f);
}

// ===== Window =====
static void apply_hamming(float *x)
{
    for (int i = 0; i < MFCC_FFT_SIZE; i++)
    {
        float w = 0.54f - 0.46f * cosf(2 * M_PI * i / (MFCC_FFT_SIZE - 1));
        x[2 * i] *= w;   // real part only
    }
}

// ===== PCM → float =====
static void pcm_to_float(int16_t *in, float *out)
{
    for (int i = 0; i < MFCC_FFT_SIZE; i++)
    {
        out[2 * i] = (float)in[i] / 32768.0f;
        out[2 * i + 1] = 0.0f;
    }
}

// ===== Power spectrum =====
static void compute_power(float *fft, float *power)
{
    for (int i = 0; i < MFCC_FFT_SIZE / 2; i++)
    {
        float real = fft[2 * i];
        float imag = fft[2 * i + 1];
        power[i] = real * real + imag * imag;
    }
}

// ===== Mel filterbank (FIXED VERSION) =====
static void compute_mel(float *power, float *mel_out)
{
    memset(mel_out, 0, sizeof(float) * MFCC_NUM_MEL);

    float mel_min = hz_to_mel(0);
    float mel_max = hz_to_mel(MFCC_SAMPLE_RATE / 2);

    float mel_points[MFCC_NUM_MEL + 2];
    float hz_points[MFCC_NUM_MEL + 2];
    int bin[MFCC_NUM_MEL + 2];

    // Create mel points and bins
    for (int i = 0; i < MFCC_NUM_MEL + 2; i++)
    {
        mel_points[i] = mel_min +
                        (mel_max - mel_min) * i / (MFCC_NUM_MEL + 1);

        hz_points[i] = mel_to_hz(mel_points[i]);

        int b = (int)(hz_points[i] * MFCC_FFT_SIZE / MFCC_SAMPLE_RATE);

        // CLAMP (critical fix)
        if (b < 0)
            b = 0;
        if (b >= MFCC_FFT_SIZE / 2)
            b = MFCC_FFT_SIZE / 2 - 1;

        bin[i] = b;
    }

    // Apply triangular filters
    for (int m = 0; m < MFCC_NUM_MEL; m++)
    {

        float sum = 0.0f;

        int b0 = bin[m];
        int b1 = bin[m + 1];
        int b2 = bin[m + 2];

        // Avoid degenerate filters
        if (b1 <= b0)
            b1 = b0 + 1;
        if (b2 <= b1)
            b2 = b1 + 1;

        // First slope
        int denom1 = b1 - b0;
        if (denom1 == 0)
            denom1 = 1;

        for (int k = b0; k < b1 && k < MFCC_FFT_SIZE / 2; k++)
        {
            float w = (float)(k - b0) / denom1;
            sum += power[k] * w;
        }

        // Second slope
        int denom2 = b2 - b1;
        if (denom2 == 0)
            denom2 = 1;

        for (int k = b1; k < b2 && k < MFCC_FFT_SIZE / 2; k++)
        {
            float w = (float)(b2 - k) / denom2;
            sum += power[k] * w;
        }

        // Prevent log(0)
        if (sum < 1e-10f)
            sum = 1e-10f;

        mel_out[m] = logf(sum);
    }
}

// ===== DCT =====
static void compute_dct(float *mel, float *mfcc)
{
    for (int n = 0; n < MFCC_NUM_COEFF; n++)
    {

        float sum = 0.0f;

        for (int m = 0; m < MFCC_NUM_MEL; m++)
        {
            sum += mel[m] *
                   cosf(M_PI * n * (m + 0.5f) / MFCC_NUM_MEL);
        }
        
        float scale = (n == 0) ? sqrtf(1.0f / MFCC_NUM_MEL)
                                : sqrtf(2.0f / MFCC_NUM_MEL);

        mfcc[n] = sum * scale;
    }
}

// ===== Public API =====
void mfcc_init(void)
{
    dsps_fft2r_init_fc32(NULL, MFCC_FFT_SIZE);
}

void compute_mfcc(int16_t *pcm, float *mfcc_out)
{
    // 1. PCM → float
    pcm_to_float(pcm, fft_buffer);

    // 2. Window
    apply_hamming(fft_buffer);

    // 3. FFT
    dsps_fft2r_fc32(fft_buffer, MFCC_FFT_SIZE);
    dsps_bit_rev_fc32(fft_buffer, MFCC_FFT_SIZE);

    // 4. Power spectrum
    compute_power(fft_buffer, power_spectrum);

    // 5. Mel filterbank
    compute_mel(power_spectrum, mel_energies);

    // 6. DCT → MFCC
    compute_dct(mel_energies, mfcc_out);
}