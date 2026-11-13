[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.23858855.svg)](https://doi.org/10.5281/zenodo.23858855)

> [!IMPORTANT]
> **Proprietary Notice & Institutional Attribution**
> **RMAA Acoustics AI** is an intellectual property and flagship industrial execution product of **RMAA HOLDINGS**. All rights reserved.

> [!CAUTION]
> **Advisory & Experimental Disclaimer for Evaluators and Investors**
> This repository is maintained as an active research and development workspace. Modules within this codebase undergo rapid iteration and autonomous synthesis. Portions of the codebase are experimental and subject to ongoing validation and physical bench testing. Evaluators, partners, and investors should review system claims against the hardware validation logs in `/benchmarks` and official entity certifications in `/credentials`.

---

# RMAA Acoustics AI — CNC Spindle Defense System
### Pure Bare-Metal DSP Silicon • 80-Band Frequency Partitioning • Microsecond Optical E-Stop
Maintained by **RMAA HOLDINGS** • Licensed under **BSL 1.1**

[![License: BSL 1.1](https://img.shields.io/badge/License-BSL%201.1-blue.svg)](LICENSE.md)
[![Platform: ESP32-S3](https://img.shields.io/badge/Platform-ESP32--S3-red.svg)](esp_dsp_firmware.cpp)
[![Architecture: Bare--Metal DSP](https://img.shields.io/badge/Architecture-Bare--Metal%20DSP-emerald.svg)](esp_dsp_firmware.cpp)

RMAA Acoustics AI is an industrial-grade bare-metal edge hardware defense system designed for high-speed CNC spindle protection, acoustic load classification, deterministic tool breakage defense, and autonomous closed-loop feed-rate override (FRO).

Operating directly on edge silicon (ESP32-S3 @ 240 MHz with Xtensa LX7 SIMD vector extensions), it continuously samples spindle micro-acoustics at 48 kHz / 24-bit, executes Radix-2 Real FFTs, partitions the spectrum into 80 physical frequency bands, and trips an optical E-stop circuit in **under 1.5 milliseconds** upon detecting severe chatter resonance, catastrophic tip fracture, or tool deflection.

---

## 🏛️ End-to-End Hardware Interception & Control Architecture

```
 ┌────────────────────────────────────────────────────────────────────────┐
 │                    PHYSICAL HARDWARE & SENSOR LAYER                    │
 │                                                                        │
 │  [Spindle Micro-Acoustics & Vibration]     [Tool / Spindle Gantry]    │
 │       │ (Bottom-Port INMP441 + Shore 40A)         │                    │
 │       │ (Shielded Twisted Pair + Choke)           │                    │
 │       ▼                                           ▼                    │
 │   [ESP32-S3 Dual-Core DSP Engine]       [Hardware Interception]        │
 │      ├── Core 0: Zero-Copy Ping-Pong DMA   ├── 6N137 High-Speed Opto   │
 │      │   └── Sub-Window RMS (< 1.5ms)      │   + 2N7002 Active 24V     │
 │      ├── Core 1: Radix-2 FFT (80 Bins)     └── Instantaneous E-Stop    │
 │      └── RS-485 Modbus RTU (FRO Control) ──► Modbus Feed-Rate Override │
 └────────────────────────────────────────────────────────────────────────┘
```

---

## 🔬 Core Physics & Algorithmic Modules

### 1. Dual-Rate Acoustic Engine & Sub-1.5ms Transient Protection
- **Fast Transient Sub-Window (1.33 ms):** Processes 64-sample RMS sub-windows directly from DMA memory to trigger optical E-stop protection in $< 1.5\text{ ms}$, bypassing FFT latency for sudden catastrophic overload.
- **80-Band Power Decomposition (42.67 ms):** Accumulates true linear acoustic energy $(re^2 + im^2)$ followed by single-conversion $10 \log_{10}(P)$ decibel scaling across 10 kinematic bins (1–500 Hz) and 70 high-frequency chatter bins (500–20,000 Hz).
- **Pre-Computed Windowing:** Aligns 4-term Blackman-Harris window coefficients to 16-byte boundaries, preventing runtime table mutation and phase corruption.

### 2. Altintas-Budak Synchronicity & Runout-Aware Comb Filter
- **Tooth-Harmonic Synchronicity Ratio (THSR — $\Gamma$):** Comb filters spectral energy to distinguish stable cutting through dense knots ($\Gamma > 0.80$) from self-excited regenerative chatter ($\Gamma < 0.45$).
- **5–18 μm Collet Runout Compensation:** Expands synchronous comb masks across the entire spindle rotational harmonic grid ($m \cdot f_{\text{rot}}$), isolating tool runout asymmetry without triggering false chatter alarms.
- **Tool Asymmetry Index ($\rho_{\text{runout}}$):** Continuously tracks $\frac{E(f_{\text{rot}})}{E(f_{\text{tooth}})}$ to identify sudden single-flute chipping ($>300\%$ baseline jump).

### 3. Entry Shock Blanking & Workpiece Material Intelligence
- **15 ms Plunge Blanking Filter:** Differentiates self-limiting workpiece entry shock ($dE/dt > \kappa$) from brittle tip fracture by verifying structural ring-down decay ($\tau_{\text{decay}} \approx 4\text{–}8\text{ ms}$) vs. permanent step-level shift over a 30 ms observation window.
- **Autonomous 100 ms Material Inference:** Evaluates a 4-parameter acoustic decision space (Spectral Entropy $H_{\text{spec}}$, Spectral Centroid $C_{\text{spec}}$, High/Low Ratio $\Psi_{\text{HL}}$, and Comb Crest Factor $CF_{\text{comb}}$) to classify stock materials:
  - **Aluminum 6061-T6:** Strict stability boundary ($\Gamma_{\text{ref}} = 0.78$), aggressive de-escalation ($-200\%/\text{s}$), $40\%$ floor to prevent built-up edge (BUE).
  - **Hardwood / Dense MDF:** Forgiving boundary ($\Gamma_{\text{ref}} = 0.55$), gradual de-escalation ($-60\%/\text{s}$), fast recovery ($+25\%/\text{s}$), $25\%$ floor to prevent charring.
  - **360 Brass:** Balanced boundary ($\Gamma_{\text{ref}} = 0.65$), $-120\%/\text{s}$ drop, $30\%$ floor.

### 4. Kinematic G0 Transit & Multi-Spindle Cross-Talk Rejection
- **Ball-Screw Doppler Chirp Tracker:** Flags linear frequency ramps ($\vert{}df/dt\vert{} > 20{,}000\text{ Hz/s}$) during rapid positioning ($G0, > 1.5\text{g}$) to isolate ball-pass whine from stationary structural chatter modes.
- **Air-Cutting Power Gating:** Monitors VFD mechanical power draw ($\Delta P > 30\text{ W}$) and tooth-pass SNR ($> 6\text{ dB}$) to automatically mute chatter detection during non-cutting rapid positioning.
- **Multi-Spindle Notch Filter:** Filters secondary spindle harmonic lines ($n \cdot f_{\text{rot2}}$) vibrating through common gantry bridges on multi-head CNC machines.

### 5. In-Situ Acoustic Tap-Testing & Modal Sweet-Spot Generator
- **Cantilever Ring-Down Calibration:** Blanks initial contact impact (0.5 ms) and isolates free cantilever decay.
- **Parabolic Sub-Bin Peak Interpolation:** Evaluates discrete FFT bins to resolve tool assembly natural frequency ($f_n$) down to $\pm 1.5\text{–}3.0\text{ Hz}$.
- **Hilbert Analytic Log-Decay Regression:** Extracts structural damping ratio ($\zeta$).
- **Analytical Stability Lobes:** Solves optimal chatter-free spindle speeds:
  $$\text{RPM}_{\text{optimal}}(k) = \frac{60 \cdot f_n}{k \cdot Z}$$
  and outputs recommendations to Modbus holding registers `0x0041`–`0x0046` for machine HMI display.

### 6. Air-Gapped Cryptographic Lease & Anti-Tamper Ratchet
- **Work-Denominated Lease Ratchet:** Gates license consumption strictly on physical cutting hours ($I_{\text{spindle}} > I_{\text{idle}} \land \text{RPM} > 3000$), completely decoupled from insecure wall clocks.
- **Three-Tier Anti-Rollback Progression:** SRAM accumulator $\to$ Wear-leveled append-only NOR flash ring $\to$ Monotonic polysilicon eFuse burns (`EFUSE_BLK3`) every 100 operating hours.
- **Authenticated Telemetry Chaining:** Chunks telemetry into 4 KB blocks protected by hardware AES-256-GCM AEAD encryption and a rolling SHA-256 hash ratchet ($H_k = \text{SHA-256}(B_k \parallel H_{k-1})$), finalized with Ed25519 device signatures.

---

## 📁 Repository Manifest

- **[`esp_dsp_firmware.cpp`](esp_dsp_firmware.cpp):** Bare-metal C++ firmware for the ESP32-S3 running FreeRTOS with hardware-accelerated ESP-DSP routines (I2S DMA, Blackman-Harris windowing, Radix-2 FFT, 80-band accumulator, and safety interlocks).
- **[`include/acoustics_ai_stability_heuristics.hpp`](include/acoustics_ai_stability_heuristics.hpp):** Real-time cutting mechanics algorithms: Altintas-Budak THSR comb filter, runout compensation, 15ms entry plunge filter, material classifier, kinematic $G0$ transit gate, and closed-loop FRO controller.
- **[`include/acoustics_ai_crypto_lease.hpp`](include/acoustics_ai_crypto_lease.hpp):** Air-gapped cryptographic lease validation, spindle work-sensing ticks, flash ring commit, and authenticated telemetry chaining.
- **[`include/acoustics_ai_modal_calibration.hpp`](include/acoustics_ai_modal_calibration.hpp):** In-situ acoustic tap-testing, parabolic sub-bin frequency interpolation, Hilbert decay damping extraction, and Modbus lobe recommendations.
- **[`hardware/BOM_REV1.0.md`](hardware/BOM_REV1.0.md):** Official hardware BOM: 6N137 high-speed optoisolator, 2N7002 MOSFET 24V pull-down, Shore 40A acoustic compression gasket, and shielded twisted-pair cabling.
- **[`hardware/ACOUSTICS_AI_HARDWARE_PROTOTYPE_REV1.0.pdf`](hardware/ACOUSTICS_AI_HARDWARE_PROTOTYPE_REV1.0.pdf):** Official Engineering Release Rev 1.0 PDF document authored by Ali CNC Engineering.
- **[`benchmarks/`](benchmarks/):** Reproducible C++17 desktop simulation harness ([`bench_dsp.cpp`](benchmarks/bench_dsp.cpp)), automated test runner ([`run_benchmark.sh`](benchmarks/run_benchmark.sh)), and full empirical execution logs ([`desktop_dsp_benchmark_linux.txt`](benchmarks/desktop_dsp_benchmark_linux.txt)).
- **[`credentials/`](credentials/):** Physical CNC machining certifications (TITANS of CNC multi-axis CAD/CAM).
- **[`LICENSE.md`](LICENSE.md):** Business Source License 1.1 terms.

---

## 🛠️ Hardware Specification (Rev 1.1)

- **Compute Core:** ESP32-S3 DevKit (WROOM-1 / N16R8), Dual-core Xtensa LX7 @ 240 MHz (16MB Flash, 8MB PSRAM).
- **Acoustic Transducer:** INMP441 Digital MEMS Microphone (Bottom-ported, 24-bit I2S PCM, 61 dBA SNR) mounted in direct contact with spindle casting via a **Shore 40A viscoelastic silicone/polyurethane compression gasket** (>30 dB ambient shop isolation).
- **Electrical Interception:** High-speed **6N137 / FOD260L optocoupler** driven by active **2N7002 N-MOSFET** buffer (< 20 μs propagation delay, guaranteed saturation against 24V industrial CNC controller inputs).
- **Signal Cabling:** Shielded Twisted Pair (STP) with 33Ω series source termination resistors and split-core NiZn ferrite chokes to reject high $dv/dt$ (5–15 kV/μs) VFD spindle switching noise.
- **Industrial Interface:** Galvanically isolated RS-485 transceiver (MAX13487) for Modbus-RTU dynamic feed-rate override (FRO) and modal sweet-spot parameter export.

---

## ⚡ Verification & Automated Test Harness

The mathematical and signal processing pipelines are verified on desktop Linux and embedded test harnesses:

```bash
# Run the automated DSP benchmark suite
./benchmarks/run_benchmark.sh
```

### Verified Performance Benchmarks
- **Acoustic Sampling:** 48,000 Hz / 24-bit PCM in 1024-sample blocks (21.33 ms frame resolution).
- **Windowing:** 4-term Blackman-Harris ($>92\text{ dB}$ side-lobe rejection).
- **DSP Frame Latency:** **~31.35 μs per frame** (**0.031 ms**), leaving $>99\%$ CPU headroom for background tasks.
- **Emergency Hardware Trip:** $< 1.5\text{ ms}$ dual-rate transient trip.
- **Modal Calibration Accuracy:** Extracted natural frequency $f_n$ within $\pm 2.0\text{ Hz}$ of ground truth ($2842.3\text{ Hz}$ observed on $2842.0\text{ Hz}$ benchmark).
- **Runout Resilience:** $12\,\mu\text{m}$ TIR collet runout maintains stable cut synchronicity ($\Gamma_{\text{comp}} = 0.999985$), eliminating nuisance trips.

---

## 📜 Licensing
This project is licensed under the **Business Source License 1.1 (BSL 1.1)**. See [LICENSE.md](LICENSE.md) for full terms and conditions.
