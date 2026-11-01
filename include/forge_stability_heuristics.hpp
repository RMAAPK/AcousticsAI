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

// Machine Kinematic & Loading State
typedef enum {
    MACHINE_STATE_AIR_CUTTING  = 0, // Zero load (<30W, tooth harmonic below floor): MUTE / STANDBY
    MACHINE_STATE_G0_TRANSIT   = 1, // Rapid positioning with high df/dt (>1kHz/frame): MUTE FRO
    MACHINE_STATE_IN_CUT       = 2, // Active solid mechanical cutting (delta_P > 30W): ARMED
    MACHINE_STATE_CROSS_TALK   = 3  // Multi-spindle interference detected: NOTCH FILTERED
} machine_kinematic_state_t;

// ----------------------------------------------------------------------------
// 1. Workpiece Material Classification & Adaptive Profiles
// Autonomous inference within first 100ms of steady engagement
// ----------------------------------------------------------------------------
typedef enum {
    MAT_UNKNOWN   = 0,
    MAT_ALUMINUM  = 1,
    MAT_WOOD_MDF  = 2,
    MAT_BRASS     = 3
} workpiece_mat_t;

typedef struct {
    float gamma_ref;         // Stability metric baseline (0.55 - 0.78)
    float crest_factor_trip; // Crest factor alarm ratio (3.5x - 6.0x)
    float slew_down;         // Rapid de-escalation rate (-60%/s to -200%/s)
    float slew_up;           // Slow recovery rate (+8%/s to +25%/s)
    float fro_min_floor;     // Hard anti-rubbing / anti-BUE clamp (25% - 40%)
    float kp;                // PI proportional gain
    float ki;                // PI integral gain
} forge_material_profile_t;

static inline forge_material_profile_t forge_get_material_profile(workpiece_mat_t mat) {
    forge_material_profile_t p;
    switch (mat) {
        case MAT_ALUMINUM:
            p.gamma_ref         = 0.78f;
            p.crest_factor_trip = 3.5f;
            p.slew_down         = 200.0f; // -200%/s
            p.slew_up           = 8.0f;   // +8%/s
            p.fro_min_floor     = 40.0f;  // Prevent strain-hardening & BUE
            p.kp                = 65.0f;
            p.ki                = 22.0f;
            break;
        case MAT_WOOD_MDF:
            p.gamma_ref         = 0.55f;
            p.crest_factor_trip = 6.0f;
            p.slew_down         = 60.0f;  // -60%/s
            p.slew_up           = 25.0f;  // +25%/s
            p.fro_min_floor     = 25.0f;  // Prevent friction burning/charring
            p.kp                = 30.0f;
            p.ki                = 8.0f;
            break;
        case MAT_BRASS:
            p.gamma_ref         = 0.65f;
            p.crest_factor_trip = 4.2f;
            p.slew_down         = 120.0f; // -120%/s
            p.slew_up           = 15.0f;  // +15%/s
            p.fro_min_floor     = 30.0f;
            p.kp                = 45.0f;
            p.ki                = 15.0f;
            break;
        case MAT_UNKNOWN:
        default:
            p.gamma_ref         = 0.70f;
            p.crest_factor_trip = 4.5f;
            p.slew_down         = 150.0f;
            p.slew_up           = 15.0f;
            p.fro_min_floor     = 35.0f;
            p.kp                = 45.0f;
            p.ki                = 15.0f;
            break;
    }
    return p;
}

static inline workpiece_mat_t forge_classify_workpiece(float h_spec,
                                                       float c_spec,
                                                       float psi_hl,
                                                       float cf_comb) {
    if (h_spec < 0.48f && cf_comb > 5.5f && psi_hl < 1.2f) {
        return MAT_ALUMINUM;
    }
    if (h_spec >= 0.70f && c_spec > 6500.0f && psi_hl >= 2.0f) {
        return MAT_BRASS;
    }
    if (h_spec >= 0.48f && h_spec < 0.72f && c_spec <= 6500.0f) {
        return MAT_WOOD_MDF;
    }
    return MAT_UNKNOWN;
}

// ----------------------------------------------------------------------------
// 2. Kinematic G0 Transit & Air-Cutting State Detector
// ----------------------------------------------------------------------------
typedef struct {
    float last_peak_freq;
    float tare_power_watts;     // Baseline spindle idle power P_tare(RPM)
    machine_kinematic_state_t state;
} forge_kinematic_gate_t;

static inline void forge_kinematic_gate_init(forge_kinematic_gate_t* g, float tare_power_watts) {
    g->last_peak_freq = 0.0f;
    g->tare_power_watts = tare_power_watts;
    g->state = MACHINE_STATE_AIR_CUTTING;
}

/**
 * @brief Evaluates whether machine is cutting stock, air cutting, or performing rapid G0 transit.
 * @param g Gate tracker state
 * @param vfd_active_power Total power read from VFD Modbus register (Watts)
 * @param tooth_energy Acoustic energy at fundamental tooth-pass frequency
 * @param ambient_energy Background acoustic energy baseline
 * @param dominant_peak_freq Peak acoustic frequency in current 42.67ms frame (Hz)
 * @param dt_frame_sec Elapsed frame time (0.04267s)
 * @return True if system is in solid mechanical cut (Armed); False if Muted/Standby
 */
static inline bool forge_kinematic_gate_step(forge_kinematic_gate_t* g,
                                             float vfd_active_power,
                                             float tooth_energy,
                                             float ambient_energy,
                                             float dominant_peak_freq,
                                             float dt_frame_sec) {
    float delta_p = vfd_active_power - g->tare_power_watts;
    float tooth_snr_db = 10.0f * log10f((tooth_energy + 1e-9f) / (ambient_energy + 1e-9f));

    // Calculate frequency chirp rate df/dt (only after first baseline frame)
    float df_dt = 0.0f;
    if (g->last_peak_freq > 0.0f) {
        df_dt = fabsf(dominant_peak_freq - g->last_peak_freq) / (dt_frame_sec > 0.0f ? dt_frame_sec : 0.04267f);
    }
    g->last_peak_freq = dominant_peak_freq;

    // Check 1: Ball-Screw Rapid Transit Chirp (G0 move: df/dt > 1000 Hz/frame ~ 23.4 kHz/s)
    if (df_dt > 20000.0f && delta_p < 40.0f) {
        g->state = MACHINE_STATE_G0_TRANSIT;
        return false; // MUTE
    }

    // Check 2: Air Cutting Gate (No mechanical resistance: delta_p < 30W or low tooth impact)
    if (delta_p < 30.0f || tooth_snr_db < 6.0f) {
        g->state = MACHINE_STATE_AIR_CUTTING;
        return false; // STANDBY
    }

    // Solid Mechanical In-Cut State
    g->state = MACHINE_STATE_IN_CUT;
    return true; // ARMED
}

// ----------------------------------------------------------------------------
// 3. Entry Shock & Plunge Transient Filter
// ----------------------------------------------------------------------------
typedef struct {
    entry_transient_state_t state;
    float pre_contact_rms;
    float elapsed_ms;
    float peak_entry_rms;
    float dE_dt_threshold;
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

static inline bool forge_entry_filter_step(forge_entry_transient_filter_t* f,
                                           float current_rms,
                                           float dt_ms,
                                           bool* out_suppress_trip) {
    if (out_suppress_trip) *out_suppress_trip = false;

    float dE_dt = (current_rms - f->last_rms) / (dt_ms > 0.0f ? dt_ms : 1.0f);
    f->last_rms = current_rms;

    if (f->state == TRANSIENT_STATE_STEADY) {
        f->pre_contact_rms = 0.95f * f->pre_contact_rms + 0.05f * current_rms;

        if (dE_dt > f->dE_dt_threshold) {
            f->state = TRANSIENT_STATE_ARMED;
            f->elapsed_ms = 0.0f;
            f->peak_entry_rms = current_rms;
            if (out_suppress_trip) *out_suppress_trip = true;
        }
        return false;
    } else {
        f->elapsed_ms += dt_ms;

        if (current_rms > f->peak_entry_rms) {
            f->peak_entry_rms = current_rms;
        }

        if (f->elapsed_ms < 15.0f) {
            if (out_suppress_trip) *out_suppress_trip = true;
            return false;
        }

        float residual_ratio = current_rms / (f->pre_contact_rms > 1e-3f ? f->pre_contact_rms : 1.0f);

        if (f->elapsed_ms >= 30.0f) {
            if (residual_ratio < 2.5f) {
                f->state = TRANSIENT_STATE_STEADY;
                return false;
            } else {
                f->state = TRANSIENT_STATE_STEADY;
                return true;
            }
        } else {
            if (residual_ratio < 2.0f) {
                if (out_suppress_trip) *out_suppress_trip = false;
            } else {
                if (out_suppress_trip) *out_suppress_trip = false;
            }
        }
    }
    return false;
}

// ----------------------------------------------------------------------------
// 4. Runout & Multi-Spindle Cross-Talk Compensated Comb Filter
// ----------------------------------------------------------------------------
typedef struct {
    float thsr_comp;             // Runout-compensated synchronicity ratio (0.0 - 1.0)
    float rho_runout;            // Runout asymmetry ratio: E(f_rot) / E(f_tooth)
    float e_coherent;            // Tooth + Runout synchronous energy
    float e_async;               // Incommensurate regenerative chatter energy
    float spectral_centroid_hz;  // Spectral Centroid (Hz)
    float high_low_ratio;        // Psi_HL (Bands 28-79 vs 0-27)
    float comb_crest_factor;     // Peak to mean synchronous power
    float dominant_peak_freq;    // Frequency of highest peak (for Doppler df/dt tracking)
    float tooth_energy;          // Energy strictly at tooth pass frequency
    bool  is_flute_fractured;    // Asymmetry index spike (>300% jump) detection
} forge_runout_metrics_t;

/**
 * @brief Generalized runout and multi-spindle cross-talk aware comb filter.
 * Notches out adjacent spindle harmonics (rpm_spindle2) from asynchronous chatter budget.
 */
static inline forge_runout_metrics_t calculate_thsr_runout_comp_ex(const float* bands_db,
                                                                   float rpm,
                                                                   int flutes,
                                                                   float rpm_spindle2,
                                                                   float sample_rate,
                                                                   int fft_size,
                                                                   float baseline_rho) {
    forge_runout_metrics_t metrics;
    memset(&metrics, 0, sizeof(metrics));

    float f_rot = rpm / 60.0f;
    float f_tooth = f_rot * (float)flutes;
    float f_rot2 = rpm_spindle2 > 0.0f ? (rpm_spindle2 / 60.0f) : 0.0f;

    float e_tooth = 0.0f;
    float e_runout = 0.0f;
    float e_rot_1x = 0.0f;
    float e_async = 0.0f;

    float sum_freq_pwr = 0.0f;
    float sum_total_pwr = 0.0f;
    float e_low = 0.0f;
    float e_high = 0.0f;
    float max_sync_pwr = 0.0f;
    float sum_sync_pwr = 0.0f;
    int   sync_band_count = 0;

    float max_any_pwr = 0.0f;
    float max_any_freq = 0.0f;

    for (int b = 0; b < FORGE_TOTAL_BANDS; b++) {
        float f_center = (b < 10) ? (1.0f + b * 49.9f + 24.95f) : (500.0f + (b - 10) * 278.57f + 139.28f);
        float pwr_linear = powf(10.0f, bands_db[b] / 10.0f);
        float tolerance = (b < 10) ? 25.0f : 140.0f;

        if (pwr_linear > max_any_pwr) {
            max_any_pwr = pwr_linear;
            max_any_freq = f_center;
        }

        sum_freq_pwr += f_center * pwr_linear;
        sum_total_pwr += pwr_linear;

        if (b < 28) {
            e_low += pwr_linear;
        } else {
            e_high += pwr_linear;
        }

        if (fabsf(f_center - f_rot) <= tolerance) {
            e_rot_1x += pwr_linear;
        }
        if (fabsf(f_center - f_tooth) <= tolerance) {
            metrics.tooth_energy += pwr_linear;
        }

        bool is_tooth_harmonic = false;
        bool is_rot_harmonic = false;
        bool is_adjacent_spindle = false;

        // Check local spindle grid
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

        // Check adjacent spindle cross-talk grid: n * f_rot2
        if (!is_tooth_harmonic && !is_rot_harmonic && f_rot2 > 10.0f) {
            for (int n = 1; n <= 16; n++) {
                float harmonic2 = (float)n * f_rot2;
                if (fabsf(f_center - harmonic2) <= tolerance) {
                    is_adjacent_spindle = true;
                    break;
                }
            }
        }

        if (is_tooth_harmonic) {
            e_tooth += pwr_linear;
            if (pwr_linear > max_sync_pwr) max_sync_pwr = pwr_linear;
            sum_sync_pwr += pwr_linear;
            sync_band_count++;
        } else if (is_rot_harmonic) {
            e_runout += pwr_linear;
            if (pwr_linear > max_sync_pwr) max_sync_pwr = pwr_linear;
            sum_sync_pwr += pwr_linear;
            sync_band_count++;
        } else if (is_adjacent_spindle) {
            // NOTCH OUT adjacent spindle cross-talk from chatter residue!
            // Structural vibration from second head rejected.
        } else {
            e_async += pwr_linear;
        }
    }

    metrics.dominant_peak_freq = max_any_freq;
    metrics.rho_runout = (e_tooth > 1e-6f) ? (e_rot_1x / e_tooth) : 0.0f;

    if (baseline_rho > 0.05f && metrics.rho_runout > (baseline_rho * 3.0f)) {
        metrics.is_flute_fractured = true;
    }

    metrics.e_coherent = e_tooth + 1.0f * e_runout;
    metrics.e_async = e_async;

    float total = metrics.e_coherent + metrics.e_async;
    metrics.thsr_comp = (total > 1e-9f) ? (metrics.e_coherent / total) : 1.0f;

    metrics.spectral_centroid_hz = (sum_total_pwr > 1e-9f) ? (sum_freq_pwr / sum_total_pwr) : 1000.0f;
    metrics.high_low_ratio = e_high / (e_low + 1e-9f);

    if (sync_band_count > 0 && sum_sync_pwr > 1e-9f) {
        float mean_sync = sum_sync_pwr / (float)sync_band_count;
        metrics.comb_crest_factor = max_sync_pwr / mean_sync;
    } else {
        metrics.comb_crest_factor = 1.0f;
    }

    return metrics;
}

static inline forge_runout_metrics_t calculate_thsr_runout_comp(const float* bands_db,
                                                                float rpm,
                                                                int flutes,
                                                                float sample_rate,
                                                                int fft_size,
                                                                float baseline_rho) {
    return calculate_thsr_runout_comp_ex(bands_db, rpm, flutes, 0.0f, sample_rate, fft_size, baseline_rho);
}

static inline float calculate_thsr(const float* bands_db, float rpm, int flutes, float sample_rate, int fft_size) {
    forge_runout_metrics_t m = calculate_thsr_runout_comp(bands_db, rpm, flutes, sample_rate, fft_size, 0.20f);
    return m.thsr_comp;
}

// ----------------------------------------------------------------------------
// 5. Spectral Entropy (H_spec)
// ----------------------------------------------------------------------------
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

    return entropy / 4.248495f;
}

// ----------------------------------------------------------------------------
// 6. Sub-Window Transient Burst Kurtosis (K_burst)
// ----------------------------------------------------------------------------
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

// ----------------------------------------------------------------------------
// 7. Material-Adaptive Closed-Loop Feed-Rate Override (FRO) Controller
// ----------------------------------------------------------------------------
typedef struct {
    workpiece_mat_t          material;
    forge_material_profile_t profile;
    float                    integral;
    float                    fro_current; // 25.0% to 100.0%
} forge_fro_controller_t;

static inline void forge_fro_init(forge_fro_controller_t* c) {
    c->material = MAT_UNKNOWN;
    c->profile = forge_get_material_profile(MAT_UNKNOWN);
    c->integral = 0.0f;
    c->fro_current = 100.0f;
}

static inline void forge_fro_set_material(forge_fro_controller_t* c, workpiece_mat_t mat) {
    c->material = mat;
    c->profile = forge_get_material_profile(mat);
}

static inline float forge_fro_step(forge_fro_controller_t* c, float gamma, float dt) {
    float err = 0.0f;
    if (gamma < c->profile.gamma_ref) {
        err = c->profile.gamma_ref - gamma; // Chatter deficit scaled to material
    }

    c->integral += err * dt;
    if (c->integral > 3.0f) c->integral = 3.0f;

    float delta = (c->profile.kp * err) + (c->profile.ki * c->integral);
    float target_fro = 100.0f - delta;

    if (target_fro < c->fro_current) {
        float max_drop = c->profile.slew_down * dt;
        c->fro_current = fmaxf(c->fro_current - max_drop, target_fro);
    } else {
        float max_rise = c->profile.slew_up * dt;
        c->fro_current = fminf(c->fro_current + max_rise, target_fro);
    }

    if (c->fro_current < c->profile.fro_min_floor) {
        c->fro_current = c->profile.fro_min_floor;
    }
    if (c->fro_current > 100.0f) {
        c->fro_current = 100.0f;
    }

    return c->fro_current;
}

#ifdef __cplusplus
}
#endif

#endif // FORGE_STABILITY_HEURISTICS_HPP
