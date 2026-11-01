#ifndef FORGE_STABILITY_HEURISTICS_HPP
#define FORGE_STABILITY_HEURISTICS_HPP

#include <stdint.h>
#include <math.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FORGE_TOTAL_BANDS         80
#define FORGE_CHATTER_START_BAND  10
#define FORGE_CHATTER_END_BAND    79

typedef enum {
    TOOL_STATE_HEALTHY     = 0,
    TOOL_STATE_BREAK_IN    = 1,
    TOOL_STATE_FLANK_WORN  = 2,  // VB > 0.2mm: Schedule tool change at pass completion
    TOOL_STATE_CHIPPING    = 3   // Micro-cracking: Immediate feed override / retract
} tool_condition_t;

/**
 * 1. Tooth-Harmonic Synchronicity Ratio (THSR - Γ)
 * Differentiates stable high-load cuts (dense wood knots) from regenerative chatter.
 *
 * @param bands_db Array of 80 dB acoustic power bands
 * @param rpm Current spindle speed (e.g. 18000 - 24000)
 * @param flutes Number of cutter teeth (Z = 2, 3, or 4)
 * @param sample_rate Sampling rate (48000 Hz)
 * @param fft_size FFT length (2048)
 * @return Gamma ratio: > 0.80 = Stable forced cut | < 0.45 = Regenerative chatter
 */
static inline float calculate_thsr(const float* bands_db, float rpm, int flutes, float sample_rate, int fft_size) {
    float f_tooth = (rpm * (float)flutes) / 60.0f;
    float hz_per_bin = sample_rate / (float)fft_size;
    float e_sync = 0.0f;
    float e_async = 0.0f;

    for (int b = 0; b < FORGE_TOTAL_BANDS; b++) {
        float f_center = (b < 10) ? (1.0f + b * 49.9f + 24.95f) : (500.0f + (b - 10) * 278.57f + 139.28f);
        float pwr_linear = powf(10.0f, bands_db[b] / 10.0f);

        // Check if band aligns with any tooth harmonic k * f_tooth
        bool is_sync = false;
        for (int k = 1; k <= 8; k++) {
            float harmonic = (float)k * f_tooth;
            float tolerance = (b < 10) ? 25.0f : 140.0f;
            if (fabsf(f_center - harmonic) <= tolerance) {
                is_sync = true;
                break;
            }
        }

        if (is_sync) {
            e_sync += pwr_linear;
        } else {
            e_async += pwr_linear;
        }
    }

    float total = e_sync + e_async;
    if (total <= 1e-9f) return 1.0f;
    return e_sync / total;
}

/**
 * 2. Spectral Entropy (H_spec)
 * Quantifies process damping and frictional rubbing from tool flank wear (VB).
 */
static inline float calculate_spectral_entropy(const float* bands_db) {
    float pwr[70];
    float sum_pwr = 0.0f;

    for (int i = 0; i < 70; i++) {
        pwr[i] = powf(10.0f, bands_db[i + FORGE_CHATTER_START_BAND] / 10.0f);
        sum_pwr += pwr[i];
    }

    if (sum_pwr <= 1e-9f) return 0.0f;

    float entropy = 0.0f;
    for (int i = 0; i < 70; i++) {
        float q = pwr[i] / sum_pwr;
        if (q > 1e-12f) {
            entropy -= q * logf(q);
        }
    }

    // Normalize against ln(70) ~ 4.2485
    return entropy / 4.248495f;
}

/**
 * 3. Sub-Window Transient Burst Kurtosis (K_burst)
 * Detects impending carbide micro-chipping transients across 16 temporal sub-windows.
 */
static inline float calculate_burst_kurtosis(const float* sub_rms, int count) {
    if (count <= 1) return 3.0f;

    float mean = 0.0f;
    for (int i = 0; i < count; i++) {
        mean += sub_rms[i];
    }
    mean /= (float)count;

    float var = 0.0f;
    float m4 = 0.0f;
    for (int i = 0; i < count; i++) {
        float diff = sub_rms[i] - mean;
        float diff2 = diff * diff;
        var += diff2;
        m4 += diff2 * diff2;
    }

    var /= (float)count;
    m4 /= (float)count;

    if (var <= 1e-9f) return 3.0f;
    return m4 / (var * var);
}

/**
 * 4. Autonomous Adaptive Feed-Rate Override (FRO) Transfer Function
 * Closed-loop proportional-integral damping to pull the cutter out of chatter.
 */
typedef struct {
    float kp;
    float ki;
    float integral;
    float gamma_ref;   // e.g. 0.70f
    float fro_current; // 35.0% to 100.0%
} forge_fro_controller_t;

static inline void forge_fro_init(forge_fro_controller_t* c) {
    c->kp = 45.0f;
    c->ki = 15.0f;
    c->integral = 0.0f;
    c->gamma_ref = 0.70f;
    c->fro_current = 100.0f;
}

static inline float forge_fro_step(forge_fro_controller_t* c, float gamma, float dt) {
    float err = 0.0f;
    if (gamma < c->gamma_ref) {
        err = c->gamma_ref - gamma; // Chatter deficit
    }

    c->integral += err * dt;
    // Anti-windup clamping
    if (c->integral > 3.0f) c->integral = 3.0f;

    float delta = (c->kp * err) + (c->ki * c->integral);
    float target_fro = 100.0f - delta;

    // Asymmetric Slew-Rate Limiting:
    // Drop rapid (-150%/s), recover slow (+15%/s)
    if (target_fro < c->fro_current) {
        float max_drop = 150.0f * dt;
        c->fro_current = fmaxf(c->fro_current - max_drop, target_fro);
    } else {
        float max_rise = 15.0f * dt;
        c->fro_current = fminf(c->fro_current + max_rise, target_fro);
    }

    // Hard Anti-Rubbing Clamp (35% floor)
    if (c->fro_current < 35.0f) c->fro_current = 35.0f;
    if (c->fro_current > 100.0f) c->fro_current = 100.0f;

    return c->fro_current;
}

#ifdef __cplusplus
}
#endif

#endif // FORGE_STABILITY_HEURISTICS_HPP
