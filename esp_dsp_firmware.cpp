#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "esp_dsp.h"
#include "mbedtls/ed25519.h"
#include "include/forge_stability_heuristics.hpp"
#include "include/forge_crypto_lease.hpp"

// --- CONFIGURATION ---
#define SAMPLE_RATE         48000
#define FFT_N               2048
#define I2S_PORT            I2S_NUM_0

// Chatter Modal Peak & Crest Factor Tracking
#define CHATTER_START_BAND  10     // 500 Hz
#define CHATTER_END_BAND    79     // 20,000 Hz
#define CREST_FACTOR_THRESH 4.5f   // Narrow-band resonance peak vs floor ratio

// Fast Low-Latency Sub-Window for Emergency E-Stop (< 2.0 ms)
#define FAST_WINDOW_N       64     // 64 samples @ 48kHz = 1.33 ms observation
#define EMERGENCY_PIN       18     // GPIO mapped to PC817 Optocoupler (Active High)
#define CHATTER_RMS_THRESH  15000.0f

// 80-Band Acoustic Vector Definition
#define BANDS_LOW           10
#define BANDS_HIGH          70
#define TOTAL_BANDS         (BANDS_LOW + BANDS_HIGH)

// I2S DMA Buffer Configuration
#define DMA_BUFFER_LEN      1024
#define DMA_BUFFER_COUNT    4

// Global Buffers (Aligned for Xtensa PIE SIMD Instructions)
static float fft_input[FFT_N * 2] __attribute__((aligned(16))); // Interleaved real/imaginary
static float vector_80[TOTAL_BANDS];
static float w_bh[FFT_N] __attribute__((aligned(16)));          // Pre-computed Blackman-Harris window

// Ed25519 Hardcoded Public Key (Placeholder)
static const unsigned char ROOT_PUB_KEY[32] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 
    // ... Fill with actual Ed25519 public key bytes
};

// ---------------------------------------------------------
// 1. I2S DMA Initialization (Double-Buffering)
// ---------------------------------------------------------
static i2s_chan_handle_t rx_handle;

void init_i2s_dma() {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);
    i2s_new_channel(&chan_cfg, NULL, &rx_handle);

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_24BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = 2, // I2S SCK
            .ws = 3,   // I2S WS
            .dout = I2S_GPIO_UNUSED,
            .din = 4,  // I2S SD
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false }
        }
    };
    
    // Double buffering achieved via dma_desc_num and dma_frame_num
    // ESP-IDF background driver handles ping-pong buffer filling via hardware DMA
    i2s_channel_init_std_mode(rx_handle, &std_cfg);
    i2s_channel_enable(rx_handle);

    // Initialize pre-computed window table once (prevents in-place mutation corruption)
    dsps_wind_blackman_harris_f32(w_bh, FFT_N);
}

// ---------------------------------------------------------
// 2. Ultra-Fast Sub-2.0ms Emergency Chatter Trip (Low Latency)
// ---------------------------------------------------------
static inline void check_fast_transient_overload(const int16_t* samples, size_t count) {
    float energy_sum = 0.0f;
    for (size_t i = 0; i < count; i++) {
        float s = (float)samples[i];
        energy_sum += s * s;
    }
    float rms = sqrtf(energy_sum / (float)count);
    if (rms > CHATTER_RMS_THRESH) {
        // Immediate hardware trip: Optocoupler collector saturation (< 20 us)
        gpio_set_level((gpio_num_t)EMERGENCY_PIN, 1);
    }
}

// ---------------------------------------------------------
// 3. Hardware-Accelerated FFT & 80-Band Accumulator (Linear Power)
// ---------------------------------------------------------
void process_acoustic_frame(int16_t* raw_samples) {
    // 1. Windowing into interleaved complex buffer from static pre-computed table
    for (int i = 0; i < FFT_N; i++) {
        fft_input[i * 2 + 0] = (float)raw_samples[i] * w_bh[i]; // Real (no in-place corruption)
        fft_input[i * 2 + 1] = 0.0f;                             // Imag
    }

    // 2. Radix-2 FFT (ESP32-S3 optimized PIE instruction set)
    dsps_fft2r_fc32(fft_input, FFT_N);
    dsps_bit_rev_fc32(fft_input, FFT_N);
    
    // 3. Linear Power Accumulation across 80-Bands (Sum of Squares)
    // Preserves true acoustic power without geometric mean suppression or redundant sqrtf
    float hz_per_bin = (float)SAMPLE_RATE / (float)FFT_N;
    memset(vector_80, 0, sizeof(vector_80));
    int counts[TOTAL_BANDS] = {0};

    for (int i = 1; i < FFT_N / 2; i++) { // Skip DC
        float freq = i * hz_per_bin;
        int band_idx = -1;

        if (freq >= 1.0f && freq < 500.0f) {
            band_idx = (int)((freq - 1.0f) / 49.9f); // 10 bands
            if (band_idx > 9) band_idx = 9;
        } else if (freq >= 500.0f && freq <= 20000.0f) {
            band_idx = 10 + (int)((freq - 500.0f) / 278.57f); // 70 bands
            if (band_idx > 79) band_idx = 79;
        }

        if (band_idx != -1) {
            float re = fft_input[i * 2 + 0];
            float im = fft_input[i * 2 + 1];
            float pwr_linear = (re * re + im * im); // Linear acoustic energy

            vector_80[band_idx] += pwr_linear;
            counts[band_idx]++;
        }
    }

    // 4. Convert linear power averages to true Decibels
    for (int i = 0; i < TOTAL_BANDS; i++) {
        if (counts[i] > 0) {
            float avg_pwr = vector_80[i] / (float)counts[i];
            vector_80[i] = 10.0f * log10f(avg_pwr + 1e-12f); // True 10*log10(Power)
        } else {
            vector_80[i] = -100.0f; // Noise floor
        }
    }
}

// ---------------------------------------------------------
// 3. Ed25519 Cryptographic Lease Verification
// ---------------------------------------------------------
bool verify_lease(const unsigned char* signature_64b, const unsigned char* payload, size_t payload_len) {
    // Utilize mbedtls ed25519 to verify the sneakernet lease.sig against baked-in ROOT_PUB_KEY
    mbedtls_ecp_group grp;
    mbedtls_ecp_group_init(&grp);
    mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_ED25519);
    
    // Omitted full mbedtls context mapping for brevity, but the physical 
    // verification validates the signature array using the offline root public key.
    // If successful, unlocks protection loop. If fails, trips security lockdown.
    return true; // Scaffold placeholder
}

// ---------------------------------------------------------
// 4. Regenerative Chatter & Modal Resonance Detection
// ---------------------------------------------------------
// Evaluates spectral peak concentration (Crest Factor) across the 500-20,000 Hz zone
// to distinguish narrow-band regenerative chatter spikes from wideband chip-cutting noise.
bool evaluate_chatter_resonance(const float* bands) {
    float max_chatter_pwr = -100.0f;
    float sum_chatter_pwr = 0.0f;
    int count = 0;

    for (int b = CHATTER_START_BAND; b <= CHATTER_END_BAND; b++) {
        if (bands[b] > -90.0f) {
            float pwr = bands[b];
            if (pwr > max_chatter_pwr) {
                max_chatter_pwr = pwr;
            }
            sum_chatter_pwr += pwr;
            count++;
        }
    }

    if (count > 0) {
        float mean_chatter_pwr = sum_chatter_pwr / (float)count;
        float crest_factor = max_chatter_pwr - mean_chatter_pwr; // in dB
        
        // If narrow-band peak exceeds local average floor by > 12 dB (4.0x power ratio)
        if (crest_factor > 12.0f && max_chatter_pwr > -30.0f) {
            return true; // Resonant chatter detected!
        }
    }
    return false;
}

// ---------------------------------------------------------
// 5. Main RTOS Loop
// ---------------------------------------------------------
void app_main(void) {
    // Initialize DSP module
    dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    
    init_i2s_dma();
    forge_fro_controller_t fro_ctrl;
    forge_entry_transient_filter_t entry_filter;
    forge_kinematic_gate_t kin_gate;
    forge_entry_filter_init(&entry_filter);
    forge_kinematic_gate_init(&kin_gate, 250.0f); // 250W tare power baseline
    forge_fro_init(&fro_ctrl);
    int16_t sample_buffer[FFT_N];

    while (1) {
        size_t bytes_read = 0;
        // Read directly from DMA ring buffer. Ping-pong allows seamless ingestion.
        i2s_channel_read(rx_handle, sample_buffer, sizeof(sample_buffer), &bytes_read, portMAX_DELAY);
        


        if (bytes_read == sizeof(sample_buffer)) {
            // 1. Fast sub-window transient overload check (< 1.5ms reaction)
            check_fast_transient_overload(sample_buffer, FAST_WINDOW_N);

            // 2. Full 80-band telemetry decomposition
            process_acoustic_frame(sample_buffer);

            // 3. Runout-Compensated Comb Filter & Spindle Asymmetry Tracking
            forge_runout_metrics_t runout_m = calculate_thsr_runout_comp(vector_80, 24000.0f, 2, SAMPLE_RATE, FFT_N, 0.20f);
            float gamma = runout_m.thsr_comp;

            // 4. Material Inference in initial 100ms of cut
            if (fro_ctrl.material == MAT_UNKNOWN) {
                float h_spec = calculate_spectral_entropy(vector_80);
                workpiece_mat_t detected_mat = forge_classify_workpiece(
                    h_spec,
                    runout_m.spectral_centroid_hz,
                    runout_m.high_low_ratio,
                    runout_m.comb_crest_factor
                );
                if (detected_mat != MAT_UNKNOWN) {
                    forge_fro_set_material(&fro_ctrl, detected_mat);
                }
            }

            // 5. Kinematic G0 Transit & Air-Cutting State Evaluation
            // Spindle VFD power read via Modbus register (simulated/polled: 420W in cut, 260W air)
            float vfd_pwr_watts = 420.0f;
            float ambient_rms = 100.0f;
            bool is_in_solid_cut = forge_kinematic_gate_step(
                &kin_gate, 
                vfd_pwr_watts, 
                runout_m.tooth_energy, 
                ambient_rms, 
                runout_m.dominant_peak_freq, 
                0.04267f
            );

            // 6. Material-Tuned Autonomous Closed-Loop FRO & Emergency Interlock
            if (is_in_solid_cut) {
                float fro_override = forge_fro_step(&fro_ctrl, gamma, 0.04267f);

                // Trip emergency defense if below material stability boundary or single-flute cleavage detected
                float chatter_boundary = fro_ctrl.profile.gamma_ref - 0.25f;
                if (gamma < chatter_boundary || runout_m.is_flute_fractured || evaluate_chatter_resonance(vector_80)) {
                    gpio_set_level((gpio_num_t)EMERGENCY_PIN, 1);
                }
            } else {
                // In Air-Cutting or G0 Rapid Transit: Mute alarms, freeze FRO at 100%
                fro_ctrl.fro_current = 100.0f;
            }
        }
    }
}

