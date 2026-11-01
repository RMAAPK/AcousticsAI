#ifndef FORGE_MODAL_CALIBRATION_HPP
#define FORGE_MODAL_CALIBRATION_HPP

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FORGE_TAP_BLANKING_SAMPLES    24     // Discard first 0.5ms @ 48kHz (contact shock)
#define FORGE_TAP_DECAY_SAMPLES       1024   // 21.33 ms analysis window
#define FORGE_MODBUS_REG_BASE         0x0040

typedef enum {
    MODAL_STATE_IDLE       = 0,
    MODAL_STATE_ARMED      = 1,
    MODAL_STATE_SUCCESS    = 2,
    MODAL_STATE_POOR_FIT   = 3
} modal_cal_state_t;

typedef struct {
    uint16_t rpm_sweet_spot;          // Recommended stable RPM
    uint8_t  lobe_index;              // Integer lobe k
    float    predicted_gain_factor;   // a_lim / a_asymptote scaling
} forge_lobe_rec_t;

typedef struct {
    float             f_n;                  // Extracted dominant natural frequency (Hz)
    float             zeta;                 // Estimated modal damping ratio
    modal_cal_state_t state;                // Calibration machine state
    uint8_t           valid_lobe_count;     // Number of valid operational lobes
    forge_lobe_rec_t  candidate_lobes[4];   // Top sweet-spot recommendations
    float             noise_sigma;          // Ambient noise standard deviation
} forge_modal_context_t;

/**
 * @brief Initialize modal calibration state machine.
 */
static inline void forge_modal_init(forge_modal_context_t* ctx, float ambient_sigma) {
    if (!ctx) return;
    memset(ctx, 0, sizeof(forge_modal_context_t));
    ctx->noise_sigma = (ambient_sigma > 1.0f) ? ambient_sigma : 10.0f;
    ctx->state = MODAL_STATE_IDLE;
}

/**
 * @brief Continuous parabolic interpolation to find continuous peak frequency.
 * Refines 46.875 Hz discrete FFT bin down to +/- 1.5 - 3.0 Hz.
 */
static inline float forge_parabolic_interpolate_peak(const float* mag_db,
                                                     int k_peak,
                                                     float hz_per_bin) {
    if (k_peak <= 0 || k_peak >= (FORGE_TAP_DECAY_SAMPLES / 2) - 1) {
        return (float)k_peak * hz_per_bin;
    }

    float alpha = mag_db[k_peak - 1];
    float beta  = mag_db[k_peak];
    float gamma = mag_db[k_peak + 1];

    float denom = (alpha - 2.0f * beta + gamma);
    if (fabsf(denom) < 1e-6f) {
        return (float)k_peak * hz_per_bin;
    }

    float delta = 0.5f * (alpha - gamma) / denom;
    if (delta > 1.0f) delta = 1.0f;
    if (delta < -1.0f) delta = -1.0f;

    return ((float)k_peak + delta) * hz_per_bin;
}

/**
 * @brief Extract damping ratio zeta via log-envelope linear regression.
 * Fits: ln(A[n]) = beta_0 - lambda * n
 * where lambda = 2 * pi * f_n * zeta * T_s
 */
static inline float forge_extract_damping_ratio(const float* envelope,
                                                int count,
                                                float f_n,
                                                float sample_rate) {
    if (count <= 10 || f_n <= 10.0f) return 0.025f; // Fallback default carbide damping

    // Perform linear regression over ln(envelope)
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_xy = 0.0f;
    float sum_x2 = 0.0f;
    int valid_points = 0;

    for (int n = 0; n < count; n++) {
        if (envelope[n] > 1e-6f) {
            float y = logf(envelope[n]);
            float x = (float)n;
            sum_x += x;
            sum_y += y;
            sum_xy += x * y;
            sum_x2 += x * x;
            valid_points++;
        }
    }

    if (valid_points < 10) return 0.025f;

    float denom = ((float)valid_points * sum_x2 - sum_x * sum_x);
    if (fabsf(denom) < 1e-6f) return 0.025f;

    // Slope of log decay: lambda
    float slope = ((float)valid_points * sum_xy - sum_x * sum_y) / denom;
    float lambda = -slope; // Decay rate per sample

    if (lambda <= 0.0f) return 0.010f; // Minimal damping lower bound

    float zeta = (lambda * sample_rate) / (2.0f * 3.141592653589793f * f_n);

    // Physical bounds check for solid carbide / collet assembly: 0.005 to 0.100
    if (zeta < 0.005f) zeta = 0.005f;
    if (zeta > 0.100f) zeta = 0.100f;

    return zeta;
}

/**
 * @brief Pre-compute optimal chatter-free spindle sweet-spot speeds from Altintas-Budak lobe theory.
 * Identifies maximum depth of cut sweet spots within spindle hardware limits.
 */
static inline void forge_compute_spindle_lobes(forge_modal_context_t* ctx,
                                               uint8_t flutes,
                                               uint16_t rpm_min,
                                               uint16_t rpm_max) {
    if (!ctx || ctx->f_n <= 0.0f || flutes == 0) return;
    ctx->valid_lobe_count = 0;

    // Evaluate integer lobes k = 1 to 10
    for (int k = 1; k <= 10; k++) {
        float optimal_rpm = (60.0f * ctx->f_n) / (float)(k * flutes);

        if (optimal_rpm >= (float)rpm_min && optimal_rpm <= (float)rpm_max) {
            uint8_t idx = ctx->valid_lobe_count;
            ctx->candidate_lobes[idx].rpm_sweet_spot = (uint16_t)roundf(optimal_rpm);
            ctx->candidate_lobes[idx].lobe_index = (uint8_t)k;

            // Stability gain factor relative to baseline asymptote:
            // S_k ~ 1 + (1 / (4 * zeta * k))
            float z = (ctx->zeta > 0.005f) ? ctx->zeta : 0.025f;
            ctx->candidate_lobes[idx].predicted_gain_factor = 1.0f + (1.0f / (4.0f * z * (float)k));

            ctx->valid_lobe_count++;
            if (ctx->valid_lobe_count >= 4) break;
        }
    }
}

/**
 * @brief Execute complete tap-test identification pipeline on a recorded impact buffer.
 * @param ctx Calibration context
 * @param tap_pcm Raw 16-bit PCM buffer of length (24 + 1024 = 1048 samples)
 * @param flutes Number of cutter teeth (e.g. 2, 3, or 4)
 * @param rpm_min Minimum machine spindle speed (e.g. 18000)
 * @param rpm_max Maximum machine spindle speed (e.g. 24000)
 * @return True if natural frequency and sweet spots successfully resolved
 */
static inline bool forge_process_tap_impact(forge_modal_context_t* ctx,
                                            const int16_t* tap_pcm,
                                            uint8_t flutes,
                                            uint16_t rpm_min,
                                            uint16_t rpm_max) {
    if (!ctx || !tap_pcm) return false;

    // 1. Skip initial contact deformation (24 samples = 0.5 ms)
    const int16_t* decay_pcm = tap_pcm + FORGE_TAP_BLANKING_SAMPLES;

    // 2. Compute peak search in frequency domain (1024-pt magnitude array simulation)
    // Find dominant spectral peak within [1000 Hz, 8000 Hz]
    float hz_per_bin = 48000.0f / 1024.0f; // 46.875 Hz/bin
    int min_bin = (int)(1000.0f / hz_per_bin);
    int max_bin = (int)(8000.0f / hz_per_bin);

    // Compute simple power spectrum from decay buffer
    float mag_db[512];
    float max_mag = -100.0f;
    int k_peak = min_bin;

    // Synthetic/Direct DFT bin calculation over structural range
    for (int k = min_bin; k <= max_bin; k++) {
        float re = 0.0f;
        float im = 0.0f;
        float omega = 2.0f * 3.141592653589793f * (float)k / 1024.0f;
        for (int n = 0; n < FORGE_TAP_DECAY_SAMPLES; n++) {
            float s = (float)decay_pcm[n];
            re += s * cosf(omega * (float)n);
            im -= s * sinf(omega * (float)n);
        }
        float pwr = re * re + im * im;
        mag_db[k] = 10.0f * log10f(pwr + 1e-9f);
        if (mag_db[k] > max_mag) {
            max_mag = mag_db[k];
            k_peak = k;
        }
    }

    // 3. Continuous Parabolic Peak Interpolation
    ctx->f_n = forge_parabolic_interpolate_peak(mag_db, k_peak, hz_per_bin);

    // 4. Construct envelope & estimate damping ratio zeta
    float envelope[128];
    int env_count = 128;
    for (int i = 0; i < env_count; i++) {
        float max_val = 0.0f;
        for (int j = 0; j < 8; j++) {
            float v = fabsf((float)decay_pcm[i * 8 + j]);
            if (v > max_val) max_val = v;
        }
        envelope[i] = max_val;
    }
    ctx->zeta = forge_extract_damping_ratio(envelope, env_count, ctx->f_n, 48000.0f / 8.0f);

    // 5. Compute Optimal Stability Sweet-Spot RPMs
    forge_compute_spindle_lobes(ctx, flutes, rpm_min, rpm_max);

    if (ctx->valid_lobe_count > 0 && ctx->f_n >= 1000.0f && ctx->f_n <= 8000.0f) {
        ctx->state = MODAL_STATE_SUCCESS;
        return true;
    } else {
        ctx->state = MODAL_STATE_POOR_FIT;
        return false;
    }
}

// ----------------------------------------------------------------------------
// Modbus-RTU Export Struct for Machine Operator Screen
// ----------------------------------------------------------------------------
typedef struct {
    uint16_t reg_fn_hz;              // Register 0x0041: Natural Frequency (Hz)
    uint16_t reg_zeta_scaled;        // Register 0x0042: Damping Ratio (zeta * 10,000)
    uint16_t reg_primary_rpm;        // Register 0x0043: Top Recommended Sweet-Spot RPM
    uint16_t reg_secondary_rpm;      // Register 0x0044: Secondary Sweet-Spot RPM
    uint16_t reg_gain_factor_scaled; // Register 0x0045: Depth Multiplier (factor * 100)
    uint16_t reg_state;              // Register 0x0046: 0=IDLE, 1=ARMED, 2=SUCCESS, 3=POOR_FIT
} forge_modal_modbus_export_t;

static inline forge_modal_modbus_export_t forge_export_modal_modbus(const forge_modal_context_t* ctx) {
    forge_modal_modbus_export_t out;
    memset(&out, 0, sizeof(out));
    if (!ctx) return out;

    out.reg_fn_hz = (uint16_t)roundf(ctx->f_n);
    out.reg_zeta_scaled = (uint16_t)roundf(ctx->zeta * 10000.0f);
    out.reg_state = (uint16_t)ctx->state;

    if (ctx->valid_lobe_count > 0) {
        out.reg_primary_rpm = ctx->candidate_lobes[0].rpm_sweet_spot;
        out.reg_gain_factor_scaled = (uint16_t)roundf(ctx->candidate_lobes[0].predicted_gain_factor * 100.0f);
    }
    if (ctx->valid_lobe_count > 1) {
        out.reg_secondary_rpm = ctx->candidate_lobes[1].rpm_sweet_spot;
    }
    return out;
}

#ifdef __cplusplus
}
#endif

#endif // FORGE_MODAL_CALIBRATION_HPP
