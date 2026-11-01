#ifndef FORGE_STABILITY_HEURISTICS_HPP
#define FORGE_STABILITY_HEURISTICS_HPP

#include <stdint.h>
#include <stdbool.h>
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
    TOOL_STATE_CHIPPING    = 3,  // Micro-cracking: Immediate feed override / retract
    TOOL_STATE_CLEAVAGE    = 4   // Catastrophic tip fracture: Immediate emergency E-stop
} tool_condition_t;

typedef enum {
    TRANSIENT_STATE_STEADY    = 0,
    TRANSIENT_STATE_ARMED     = 1   // Temporal blanking window (<15ms)
} entry_transient_state_t;

// ----------------------------------------------------------------------------
// 1. Entry Shock & Plunge Transient Filter
// Separates self-limiting workpiece entry shock from brittle carbide tip cleavage.
// ----------------------------------------------------------------------------
typedef struct {
    entry_transient_state_t state;
    float pre_contact_rms;       // Baseline RMS prior to impact
    float elapsed_ms;            // Timer in armed transient state
    float peak_entry_rms;        // Peak registered during shock
    float dE_dt_threshold;       // Derivative threshold to detect plunge (e.g. 5000.0f)
    float last_rms;
} forge_entry_transient_filter_t;

static inline void forge_entry_filter_init(forge_entry_transient_filter_t* f) {
    f->state = TRANSIENT_STATE_STEADY;
    f->pre_contact_rms = 100.0f;
    f->elapsed_ms = 0.0f;
    f->peak_entry_rms = 0.0f;
    f->dE_dt_threshold = 5000.0f;
    f->last_rms = 100.0f;
}

/**
 * @brief Evaluates whether a high-energy RMS spike is a normal plunge or tip fracture.
 * @param f Filter state
 * @param current_rms Sub-window RMS energy
 * @param dt_ms Time step in milliseconds (e.g. 1.33 ms for 64 samples at 48kHz)
 * @param out_suppress_trip Pointer returning true if E-stop should be temporarily blanked
 * @return True if catastrophic carbide tip cleavage is verified; False if normal plunge/steady
 */
static inline bool forge_entry_filter_step(forge_entry_transient_filter_t* f,
                                           float current_rms,
                                           float dt_ms,
                                           bool* out_suppress_trip) {
    if (out_suppress_trip) *out_suppress_trip = false;

    float dE_dt = (current_rms - f->last_rms) / (dt_ms > 0.0f ? dt_ms : 1.0f);
    f->last_rms = current_rms;

    if (f->state == TRANSIENT_STATE_STEADY) {
        // Track pre-contact baseline during steady cutting
        f->pre_contact_rms = 0.95f * f->pre_contact_rms + 0.05f * current_rms;

        // Detect sharp impact shock
        if (dE_dt > f->dE_dt_threshold) {
            f->state = TRANSIENT_STATE_ARMED;
            f->elapsed_ms = 0.0f;
            f->peak_entry_rms = current_rms;
            if (out_suppress_trip) *out_suppress_trip = true; // Blank initial 15ms
        }
        return false;
    } else { // TRANSIENT_STATE_ARMED
        f->elapsed_ms += dt_ms;

        if (current_rms > f->peak_entry_rms) {
            f->peak_entry_rms = current_rms;
        }

        // Within 15 ms blanking window: suppress nuisance tripping
        if (f->elapsed_ms < 15.0f) {
            if (out_suppress_trip) *out_suppress_trip = true;
            return false;
        }

        // Secondary verification window (15 ms to 30 ms): check ring-down decay
        float residual_ratio = current_rms / (f->pre_contact_rms > 1e-3f ? f->pre_contact_rms : 1.0f);

        if (f->elapsed_ms >= 30.0f) {
            // Evaluated over full 30ms: if energy decayed back to near baseline, reset
            if (residual_ratio < 2.5f) {
                f->state = TRANSIENT_STATE_STEADY;
                return false; // Normal workpiece entry confirmed (damped ring-down)
            } else {
                // Permanent step shift + rubbing: confirmed tip cleavage!
                f->state = TRANSIENT_STATE_STEADY;
                return true; // Trip emergency E-stop immediately!
            }
        } else {
            // Still in 15-30ms verification window:
            // If energy already dropped back toward baseline, can unblank safely
            if (residual_ratio < 2.0f) {
                if (out_suppress_trip) *out_suppress_trip = false;
            } else {
                if (out_suppress_trip) *out_suppress_trip = false; // Allow trip if extreme
            }
        }
    }
    return false;
}

// ----------------------------------------------------------------------------
// 2. Runout-Compensated Generalized Comb Filter (THSR-Comp)
// Compensates for tool collet Total Indicator Reading (TIR 5-18 um) and flute load asymmetry.
// ----------------------------------------------------------------------------
typedef struct {
    float thsr_comp;             // Runout-compensated synchronicity ratio (0.0 - 1.0)
    float rho_runout;            // Runout asymmetry ratio: E(f_rot) / E(f_tooth)
    float e_coherent;            // Tooth + Runout synchronous energy
    float e_async;               // Incommensurate regenerative chatter energy
    bool  is_flute_fractured;    // Asymmetry index spike (>300% jump) detection
} forge_runout_metrics_t;

/**
 * @brief Generalized runout-aware synchronicity calculation.
 * Redefines comb filter over complete integer grid of spindle speed (m * f_rot),
 * incorporating intermediate harmonics into coherent cutting energy to prevent
 * false-positive chatter trips from collet runout.
 */
static inline forge_runout_metrics_t calculate_thsr_runout_comp(const float* bands_db,
                                                                float rpm,
                                                                int flutes,
                                                                float sample_rate,
                                                                int fft_size,
                                                                float baseline_rho) {
    forge_runout_metrics_t metrics;
    memset(&metrics, 0, sizeof(metrics));

    float f_rot = rpm / 60.0f;
    float f_tooth = f_rot * (float)flutes;

    float e_tooth = 0.0f;
    float e_runout = 0.0f;
    float e_rot_1x = 0.0f;
    float e_async = 0.0f;

    for (int b = 0; b < FORGE_TOTAL_BANDS; b++) {
        float f_center = (b < 10) ? (1.0f + b * 49.9f + 24.95f) : (500.0f + (b - 10) * 278.57f + 139.28f);
        float pwr_linear = powf(10.0f, bands_db[b] / 10.0f);
        float tolerance = (b < 10) ? 25.0f : 140.0f;

        // Check alignment with 1X spindle fundamental
        if (fabsf(f_center - f_rot) <= tolerance) {
            e_rot_1x += pwr_linear;
        }

        bool is_tooth_harmonic = false;
        bool is_rot_harmonic = false;

        // Check two-tier synchronous grid: m * f_rot
        int max_m = flutes * 8;
        for (int m = 1; m <= max_m; m++) {
            float harmonic = (float)m * f_rot;
            if (fabsf(f_center - harmonic) <= tolerance) {
                if (m % flutes == 0) {
                    is_tooth_harmonic = true;
                } else {
                    is_rot_harmonic = true;
                }
                break;
            }
        }

        if (is_tooth_harmonic) {
            e_tooth += pwr_linear;
        } else if (is_rot_harmonic) {
            e_runout += pwr_linear;
        } else {
            e_async += pwr_linear;
        }
    }

    // Measure tool asymmetry index rho_runout = E(f_rot) / E(f_tooth)
    metrics.rho_runout = (e_tooth > 1e-6f) ? (e_rot_1x / e_tooth) : 0.0f;

    // Detect broken single flute (abrupt >300% jump in asymmetry index from baseline)
    if (baseline_rho > 0.05f && metrics.rho_runout > (baseline_rho * 3.0f)) {
        metrics.is_flute_fractured = true;
    }

    // Coherent energy incorporates both tooth passage and runout harmonics
    metrics.e_coherent = e_tooth + 1.0f * e_runout;
    metrics.e_async = e_async;

    float total = metrics.e_coherent + metrics.e_async;
    metrics.thsr_comp = (total > 1e-9f) ? (metrics.e_coherent / total) : 1.0f;

    return metrics;
}

/**
 * 3. Legacy Wrapper for backward compatibility
 */
static inline float calculate_thsr(const float* bands_db, float rpm, int flutes, float sample_rate, int fft_size) {
    forge_runout_metrics_t m = calculate_thsr_runout_comp(bands_db, rpm, flutes, sample_rate, fft_size, 0.20f);
    return m.thsr_comp;
}

/**
 * 4. Spectral Entropy (H_spec)
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
 * 5. Sub-Window Transient Burst Kurtosis (K_burst)
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
 * 6. Autonomous Adaptive Feed-Rate Override (FRO) Transfer Function
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
