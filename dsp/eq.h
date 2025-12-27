#ifndef PICO_AMP_EQ_H
#define PICO_AMP_EQ_H

#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"

#include "dsp.h"
#include "vol.h"
#include "i2s.h"

// Runtime tuning knob: bass EQ gain (true EQ adjustment), in 0.1 dB steps.
// Example: +60 => +6.0 dB.
extern int16_t g_bass_eq_gain_db_x10;

// Coefficients for a user-controlled low-shelf filter, stored in Q3.28 and used
// as (a0,a1,a2,b1,b2) in process_biquad().
static int32_t g_user_bass_a0 = 0;
static int32_t g_user_bass_a1 = 0;
static int32_t g_user_bass_a2 = 0;
static int32_t g_user_bass_b1 = 0;
static int32_t g_user_bass_b2 = 0;

static inline float clamp_f(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// RBJ Audio EQ Cookbook low-shelf. We compute coefficients in float and convert
// to Q3.28 once per parameter update (NOT per sample).
static inline void user_bass_eq_set_gain_db_x10(int16_t gain_db_x10) {
  // Clamp to a sane range. Users can still clip if they boost too much; the UI
  // should encourage adding headroom via volume offset.
  if (gain_db_x10 < -120) gain_db_x10 = -120;
  if (gain_db_x10 >  120) gain_db_x10 =  120;
  g_bass_eq_gain_db_x10 = gain_db_x10;

  // If gain is ~0, make it identity
  if (gain_db_x10 == 0) {
    g_user_bass_a0 = (int32_t)floatfx3(1.0f);
    g_user_bass_a1 = 0;
    g_user_bass_a2 = 0;
    g_user_bass_b1 = 0;
    g_user_bass_b2 = 0;
    return;
  }

  const float fs = 48000.0f;       // only supported rate in this firmware
  const float f0 = 95.0f;          // low-shelf corner frequency (Hz)
  const float S  = 1.0f;           // shelf slope
  const float gain_db = (float)gain_db_x10 / 10.0f;

  const float A = powf(10.0f, gain_db / 40.0f);
  const float PI = 3.14159265358979323846f;
  const float w0 = 2.0f * PI * (f0 / fs);
  const float cw0 = cosf(w0);
  const float sw0 = sinf(w0);
  const float alpha = (sw0 / 2.0f) * sqrtf((A + 1.0f / A) * (1.0f / S - 1.0f) + 2.0f);
  const float two_sqrtA_alpha = 2.0f * sqrtf(A) * alpha;

  float b0 =    A * ((A + 1.0f) - (A - 1.0f) * cw0 + two_sqrtA_alpha);
  float b1 =  2*A * ((A - 1.0f) - (A + 1.0f) * cw0);
  float b2 =    A * ((A + 1.0f) - (A - 1.0f) * cw0 - two_sqrtA_alpha);
  float a0 =        (A + 1.0f) + (A - 1.0f) * cw0 + two_sqrtA_alpha;
  float a1 =   -2.0f * ((A - 1.0f) + (A + 1.0f) * cw0);
  float a2 =        (A + 1.0f) + (A - 1.0f) * cw0 - two_sqrtA_alpha;

  // Normalize
  b0 /= a0; b1 /= a0; b2 /= a0;
  a1 /= a0; a2 /= a0;

  // Basic safety clamp before quantization (these ranges are conservative)
  b0 = clamp_f(b0, -4.0f, 4.0f);
  b1 = clamp_f(b1, -4.0f, 4.0f);
  b2 = clamp_f(b2, -4.0f, 4.0f);
  a1 = clamp_f(a1, -4.0f, 4.0f);
  a2 = clamp_f(a2, -4.0f, 4.0f);

  // Convert to Q3.28 (stored in int32)
  g_user_bass_a0 = (int32_t)floatfx3(b0);
  g_user_bass_a1 = (int32_t)floatfx3(b1);
  g_user_bass_a2 = (int32_t)floatfx3(b2);
  g_user_bass_b1 = (int32_t)floatfx3(a1);
  g_user_bass_b2 = (int32_t)floatfx3(a2);
}

// dsp audio buffers
dspfx buf0[192];
dspfx buf1[192];
dspfx buf2[192];
dspfx out_buf[192];

// equalizer filters
biquad(eq_bq_0)
biquad(eq_bq_00) // TODO:
biquad(eq_bq_user_bass)
biquad(eq_bq_1)
biquad(eq_bq_2)
biquad(eq_bq_3)
biquad(eq_bq_4)
biquad(eq_bq_5)
biquad(eq_bq_6)
biquad(eq_bq_7)
biquad(eq_bq_8)
biquad(eq_bq_9)
biquad(eq_bq_10)
biquad(eq_bq_11)
biquad(eq_bq_12)
biquad(eq_bq_13)
biquad(eq_bq_14)
biquad(eq_bq_15)
biquad(eq_bq_16)
biquad(eq_bq_17)
biquad(eq_bq_18)

//#define PASSTHRU_ENABLE

#define EQ_ENABLE
#define BASS_ENABLE

/*
 * -2: index source (v3.2)
 * -1: index source (v3.1), -1dB/oct
 *  0: index v5.31 psy
 *  1: index v1.3
 *  2: s05 v4
 *  3: ksc75 v1
 *  4: s05 v5
 */
#define USE_EQ -2

#include "eq_configs.h"
#include "eq_default_config.h"

#ifdef PASSTHRU_ENABLE
#ifdef EQ_ENABLE
#undef EQ_ENABLE
#endif
#ifdef BASS_ENABLE
#undef BASS_ENABLE
#endif
#endif

int32_t current_vol_l = 0;
int32_t current_vol_r = 0;
#define VOL_STEP 300000

dspfx limit[192]; // sample store
int limit_index = 0;
int32_t limit_vol = 0;
//#define LIMIT_MUL ((dspfx)(0.04*(double)(1<<30))) // 0.04x > -28dB limit relative to input (-28+18 > Stops limiting at -10dB)
//#define BASS_MUL floatfx(1./8.) // Bass peak filter is 18dB > 8x
int64_t targ = 0;
#define BASS_STEP 10000
#define BASS_STEP_DOWN 600000
#define BASS_STEP_THRESHOLD ((dspfx)(0.05*(double)(1<<30))) // 1% (i dunno)
#define BASS_STEP_DELAY_US 500*1000

uint cur_alt = 1;

uint64_t bass_step_time = 0;

/**
 * @brief Stores uint8_t data sent from USB in the i2s buffer.
 *
 * @param in Data to store
 * @param sample Number of bytes to store
 * @param resolution Sample bit depth (16, 24, 32)
 * @return true Success
 * @return false Failure (buffer full)
 */
static void __not_in_flash_func(eq_process)(uint8_t* buffer, int sample, uint8_t resolution) {
    uint64_t now_time = time_us_64();

    // NOTE: Internal DSP buffers are sized for max 96 stereo frames (192 samples).
    // Some hosts/stack paths can occasionally deliver larger OUT packets.
    // Process in chunks to avoid overruns (which can manifest as periodic pops).
    const int16_t MAX_FRAMES = 96;
    int bytes_per_frame = 0;
    switch (resolution) {
    case 32: bytes_per_frame = 8; break; // 2ch * 4 bytes
    case 24: bytes_per_frame = 6; break; // 2ch * 3 bytes
    case 16: bytes_per_frame = 4; break; // 2ch * 2 bytes
    default: bytes_per_frame = 0; break;
    }
    if (bytes_per_frame == 0 || sample <= 0) return;

#ifdef PASSTHRU_ENABLE
#define HEADROOM 0
#else
#define HEADROOM 1
#endif
    uint8_t *in_ptr = buffer;
    int bytes_left = sample;
    while (bytes_left >= bytes_per_frame) {
        int chunk_bytes = bytes_left;
        int max_chunk_bytes = MAX_FRAMES * bytes_per_frame;
        if (chunk_bytes > max_chunk_bytes) chunk_bytes = max_chunk_bytes;
        // Ensure we only process whole frames
        chunk_bytes -= (chunk_bytes % bytes_per_frame);
        int16_t count = (int16_t)(chunk_bytes / bytes_per_frame);
        if (count <= 0) break;

        // Convert input chunk to internal fixed-point stereo sample buffer buf0[]
        switch (resolution)
        {
        case 32: // 32bit
            {
                int32_t *in = (int32_t *) in_ptr;
                for (int i = 0; i < count * 2; i++)
                    buf0[i] = in[i] >> HEADROOM; // headroom for filters
            }
            break;
        case 24: // 24bit packed
            {
                uint8_t *in = (uint8_t *) in_ptr;
                for (int i = 0; i < count * 2; i += 2) {
                    int j = i * 3;
                    buf0[i]   = (in[j]   << 8 | in[j+1] << 16 | (int8_t)in[j+2] << 24) >> HEADROOM;
                    buf0[i+1] = (in[j+3] << 8 | in[j+4] << 16 | (int8_t)in[j+5] << 24) >> HEADROOM;
                }
            }
            break;
        case 16: // 16bit
            {
                int16_t *in = (int16_t *) in_ptr;
                for (int i = 0; i < count * 2; i++)
                    buf0[i] = (in[i] << 16) >> HEADROOM;
            }
            break;
        default:
            return;
        }

    // main filters
#ifdef EQ_ENABLE
    process_biquad(&eq_bq_1, biquadconstsfx(EQ_I_0), count, buf0, buf1);
    process_biquad(&eq_bq_2, biquadconstsfx(EQ_I_1), count, buf1, buf0);
#ifdef EQ_I_2
    process_biquad(&eq_bq_3, biquadconstsfx(EQ_I_2), count, buf0, buf1);
    process_biquad(&eq_bq_4, biquadconstsfx(EQ_I_3), count, buf1, buf0);
#endif
#ifdef EQ_I_4
    process_biquad(&eq_bq_5, biquadconstsfx(EQ_I_4), count, buf0, buf1);
#endif
#ifdef EQ_I_5
    process_biquad(&eq_bq_6, biquadconstsfx(EQ_I_5), count, buf1, buf0);
#endif
#ifdef EQ_I_6
    process_biquad(&eq_bq_7, biquadconstsfx(EQ_I_6), count, buf0, buf1);
    process_biquad(&eq_bq_8, biquadconstsfx(EQ_I_7), count, buf1, buf0);
#endif
#ifdef EQ_I_8
    process_biquad(&eq_bq_9, biquadconstsfx(EQ_I_8), count, buf0, buf1);
    process_biquad(&eq_bq_10, biquadconstsfx(EQ_I_9), count, buf1, buf0);
#endif
#ifdef EQ_I_10
    process_biquad(&eq_bq_11, biquadconstsfx(EQ_I_10), count, buf0, buf1);
    process_biquad(&eq_bq_12, biquadconstsfx(EQ_I_11), count, buf1, buf0);
#endif
#ifdef EQ_I_12
    process_biquad(&eq_bq_13, biquadconstsfx(EQ_I_12), count, buf0, buf1);
    process_biquad(&eq_bq_14, biquadconstsfx(EQ_I_13), count, buf1, buf0);
#endif
#ifdef EQ_I_14
    process_biquad(&eq_bq_15, biquadconstsfx(EQ_I_14), count, buf0, buf1);
    process_biquad(&eq_bq_16, biquadconstsfx(EQ_I_15), count, buf1, buf0);
#endif
#ifdef EQ_I_16
    process_biquad(&eq_bq_17, biquadconstsfx(EQ_I_16), count, buf0, buf1);
    process_biquad(&eq_bq_18, biquadconstsfx(EQ_I_17), count, buf1, buf0);
#endif
#endif

    // Select the EQ output buffer (compile-time macro decides which buffer holds the last stage)
    dspfx* eq_src = (dspfx*)LAST_EQ_BUF;

    // User bass EQ (true low-shelf) on the final EQ output.
    // This is a real EQ adjustment (not the bass enhancer/mixer below).
    if (g_bass_eq_gain_db_x10 != 0) {
        process_biquad(&eq_bq_user_bass, g_user_bass_a0, g_user_bass_a1, g_user_bass_a2, g_user_bass_b1, g_user_bass_b2, count, eq_src, buf2);
        eq_src = buf2;
    }

    // volume
    dspfx volume_mul[96] = {0};
    for (int i = 0; i < count; i++) {
        if (current_vol_l - VOL_STEP > (mute_l ? 0 : vol_mul_l))
            current_vol_l -= VOL_STEP;
        else if (current_vol_l < (mute_l ? 0 : vol_mul_l) - VOL_STEP)
            current_vol_l += VOL_STEP;
        else
            current_vol_l = (mute_l ? 0 : vol_mul_l);
        if (current_vol_r - VOL_STEP > (mute_r ? 0 : vol_mul_r))
            current_vol_r -= VOL_STEP;
        else if (current_vol_r < (mute_r ? 0 : vol_mul_r) - VOL_STEP)
            current_vol_r += VOL_STEP;
        else
            current_vol_r = (mute_r ? 0 : vol_mul_r);
        volume_mul[i] = current_vol_l > current_vol_r ? current_vol_l : current_vol_r;
        buf0[i*2] = mulfx2(eq_src[i*2], current_vol_l);
        buf0[i*2+1] = mulfx2(eq_src[i*2+1], current_vol_r);
    }

    // amp
#ifdef POWER_LIMIT
    for (int i = 0; i < count * 2; i++)
        buf0[i] = buf0[i] >> POWER_LIMIT;
#endif

    // bass filters
#ifdef BASS_ENABLE
    for (int i = 0; i < count * 2; i++)
        buf0[i] = buf0[i] >> 3; // divide by 8, headroom for bass eq

    process_biquad(&eq_bq_0, biquadconstsfx(EQ_BASS), count, buf0, buf1);
#ifdef EQ_BASS_2
    process_biquad(&eq_bq_00, biquadconstsfx(EQ_BASS_2), count, buf1, buf2);
    #define BASS_BUF buf2
#else
    #define BASS_BUF buf1
#endif

    limit[limit_index] = fxabs(BASS_BUF[0]);
    if (limit[limit_index]>0) 
        limit_index++;
    limit_index %= 192;
    limit[limit_index] = fxabs(BASS_BUF[1]);
    if (limit[limit_index]>0)
        limit_index++;
    limit_index %= 192;

    dspfx max = 0;
    for (int i = 0; i < 192; i++)
        if (limit[i] > max) max = limit[i];

    if (max < LIMIT_MUL)
        max = LIMIT_MUL;
    int64_t actualtarg = (int64_t)(LIMIT_MUL - mulfx(max, BASS_MUL)) * (int64_t)(1<<30) / (int64_t)(max - mulfx(max, BASS_MUL)); // target mix to limit bass
    if (actualtarg < 0) // if the mix goes negative, ignore it
        actualtarg = 0;

    for (int i = 0; i < count * 2; i += 2) {
        if (targ > actualtarg - BASS_STEP_THRESHOLD)
            bass_step_time = now_time + BASS_STEP_DELAY_US;
        if (targ > actualtarg || now_time > bass_step_time) {
            if (targ - BASS_STEP_DOWN > actualtarg)
                targ -= BASS_STEP_DOWN;
            else if (targ < actualtarg - BASS_STEP)
                targ += BASS_STEP;
            else
                targ = actualtarg;
        }
        buf0[i] = (mulfx2(buf0[i], (1<<30) - targ) + mulfx2(BASS_BUF[i], targ)) << 3;
        buf0[i+1] = (mulfx2(buf0[i+1], (1<<30) - targ) + mulfx2(BASS_BUF[i+1], targ)) << 3;
    }
#endif

    // limiter
#ifndef PASSTHRU_ENABLE
#ifdef POWER_LIMIT
// it seems like the bass filters are putting the signal level above slightly, compensate by raising limiter
#define LIMIT_MAX ((LIMIT_MUL << 3) + ((dspfx)(0.05*(double)(1<<30))))
#else
#define LIMIT_MAX ((1 << 30) - 1)
#endif
    dspfx limit_max = ((1 << 30) - 1);
    dspfx limit_min;
    for (int i = 0; i < count; i++) {
        if (LIMIT_MAX > 0) {
            limit_max = volume_mul[i] > LIMIT_MAX ? volume_mul[i] : LIMIT_MAX;
        }
        limit_min = -LIMIT_MAX - 1;
        if (buf0[i*2] > limit_max)
            buf0[i*2] = limit_max;
        if (buf0[i*2+1] < limit_min)
            buf0[i*2+1] = limit_min;
    }
#endif

    for (int i = 0; i < count * 2; i++)
        out_buf[i] = buf0[i] << HEADROOM;

    // output to i2s
    if (current_vol_l != 0 || current_vol_r != 0)
        i2s_enqueue((uint8_t *)out_buf, count * 8, 32);

        // Advance to next chunk
        in_ptr += chunk_bytes;
        bytes_left -= chunk_bytes;
    }
}

#endif