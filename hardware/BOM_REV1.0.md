# ENGINEERING RELEASE • REV 1.0 — RMAA ACOUSTICS AI — HARDWARE PROTOTYPE
### Complete Bill of Materials (BOM) & Bench Assembly Technical Specifications
**Author:** Ali CNC Engineering • **Project:** RMAA Acoustics AI Spindle Defense System for CNCs
**Target Silicon:** ESP32-S3 (Dual-Core Xtensa LX7 @ 240 MHz) • **Response Latency:** < 2.0 ms (Edge E-Stop)  
**Estimated Local Market Total:** Rs. 3,000 – 3,400 PKR (~$10–12 USD) • **Licensed under:** BSL 1.1 • Rawalpindi, Pakistan  
**Engineering hardware prototype PDF:** [Acoustics AI hardware prototype (Rev 1.0)](ACOUSTICS_AI_HARDWARE_PROTOTYPE_REV1.0.pdf)

---

## 1. Executive Summary & System Overview

The **RMAA Acoustics AI Hardware Prototype** is an industrial edge acoustic monitoring instrument designed to detect high-frequency micro-chatter, spindle bearing wear, and cutting tool fracture on CNC milling spindles before catastrophic breakage occurs.

Audio is ingested at 48 kHz (24-bit Mono) via an omnidirectional digital MEMS microphone over an I2S DMA bus, transformed into an 80-band acoustic vector using hardware-accelerated Radix-2 Real FFT, and evaluated against safety heuristics. A tripped anomaly triggers an optically isolated solid-state E-Stop circuit within **< 2.0 milliseconds** (total system interception guaranteed **< 10 ms**).

---

## 2. Official Bill of Materials (BOM) — Procurement Checklist

Itemized checklist for component distributors (College Road / Electronic Market):

| # | Component Name | Technical Specification | Qty | Function / Role |
|---|---|---|---|---|
| **1** | **ESP32-S3 DevKit Board** | ESP32-S3-WROOM-1 / N16R8 (16MB Flash, 8MB Octal PSRAM, Dual Type-C) | 1 pc | Main Edge Processor • Runs 48 kHz I2S DMA + 2048-pt hardware FFT |
| **2** | **INMP441 MEMS Microphone** | Omnidirectional Digital I2S Module (High SNR 61 dBA, Bottom-ported, 24-bit) | 1 pc | Acoustic ingestion with acoustic compression coupling |
| **2b**| **Acoustic Gasket Ring** | Shore 40A Viscoelastic Silicone / Polyurethane O-ring (OD 8mm, ID 4mm) | 1 pc | Sealed compression chamber (>30 dB ambient shop noise isolation) |
| **3** | **High-Speed Optoisolator** | 6N137 / FOD260L High-Speed (10 MBd) with internal Schmitt Trigger & 2N7002 MOSFET buffer | 1 pc | Active 24V industrial pull-down (< 75 ns delay, immune to CTR aging) |
| **4** | **Prototyping Breadboard** | MB-102 Full Size (830 Tie-Points) *(Do not purchase mini 400-point board)* | 1 pc | Provides sufficient clearance on both sides of wide ESP32-S3 |
| **5** | **Shielded Twisted Pair (STP)** | 4-Pair Shielded Cable with Drain Wire (replaces raw unshielded ribbon) | 1 m | Eliminates VFD PWM capacitive crosstalk on SCK/WS/SD lines |
| **5b**| **NiZn Ferrite Clamp Choke** | Split-Core Ferrite Clamp (>150Ω @ 100MHz, e.g. TDK / Würth) | 1 pc | Common-mode VFD transient suppression on sensor harness |
| **5c**| **Series Damping Resistors** | 33Ω Metal Film 1/4W Resistors | 3 pcs | Terminated on SCK, WS, and SD to suppress transmission line ringing |
| **6** | **Dupont Jumper Wires (M-M)** | Male-to-Male Ribbon Cable (20cm) | 1 pack (40 pcs) | Breadboard rail jumpers and circuit interconnects |
| **7** | **Visual Trip Indicator LEDs** | 5mm High-Brightness Diffused Red LEDs | 5 pcs | Visual E-Stop trip confirmation on test bench |
| **8** | **Current-Limiting Resistors** | 220Ω or 330Ω Metal Film (1/4 Watt, 5%) | 5 pcs | Limits LED & optocoupler diode forward current |
| **9** | **Isolated Test Power Supply** | 9V Heavy Duty Battery + 9V Snap Connector Clip | 1 set | Powers the fully isolated secondary side of the E-Stop loop |
| **10** | **USB Data & Flashing Cable** | USB Type-A to Type-C (or Type-C to C, Must support Data/Sync) | 1 pc | Firmware flashing, serial console telemetry, and 5V power |

**Estimated Local Market Total:** **Rs. 3,000 – 3,400 PKR**

---

## 3. Electrical Pinout & Wiring Schematic

### A. INMP441 Microphone → ESP32-S3 Wiring
> [!CAUTION]
> **CRITICAL VOLTAGE NOTICE:** Connect INMP441 VDD strictly to the **3.3V** pin of the ESP32-S3. Connecting VDD to 5V will permanently destroy the MEMS sensor.

| INMP441 Pin | ESP32-S3 Pin | Signal Type | Firmware Role |
|---|---|---|---|
| **VDD** | `3.3V` | Power Supply | 3.3V Regulated Rail |
| **GND** | `GND` | Ground Reference | Common System Ground |
| **SD (Serial Data)** | `GPIO 4` | Digital Input | I2S Direct Memory Access (DMA) Ingestion |
| **SCK (Bit Clock)** | `GPIO 2` | Digital Clock | I2S Serial Clock (BCLK) |
| **WS (Word Select)** | `GPIO 3` | Digital Sync | I2S Frame / Left-Right Clock (LRCK) |
| **L/R** | `GND` | Channel Select | Tied to GND for Left Channel (Mono Mode) |

---

### B. Active High-Speed Optoisolator (6N137 / FOD260L) + 2N7002 Driver Circuit

Upgraded from standard PC817 to eliminate CTR saturation starvation on 24V industrial CNC controller inputs (e.g., Mach3, NcStudio, RichAuto DSP).

```
   ESP32-S3 (Primary Side)                      CNC Controller (Secondary Side - 24V Loop)
  ┌─────────────────────────┐                  ┌──────────────────────────────────────────┐
  │                         │                  │                                          │
  │  GPIO 18 ──[ 330Ω ]─────┼─►[Pin 2: Anode ] │  +24V CNC Rail ──[ 2.2kΩ Pull-up ]──┐    │
  │                         │  [      6N137  ] │                                     │    │
  │  GND ───────────────────┼─►[Pin 3: Cath ]  │  Gate (2N7002) ◄───[Pin 6: VO (Open)]│   │
  │                         │                  │  Drain (2N7002) ────────────────────┴─► E-Stop Input (Tripped)
  │                         │                  │  Source (2N7002) ────────────────────► CNC Isolated GND
  └─────────────────────────┘                  └──────────────────────────────────────────┘
```

| Component Pin | Connection Point | Technical Purpose |
|---|---|---|
| **6N137 Pin 2 (Anode)** | `GPIO 18` via 330Ω | Clean ~6.2 mA forward drive current |
| **6N137 Pin 3 (Cathode)** | `ESP32 GND` | System digital ground return |
| **6N137 Pin 6 (V_OUT)** | Gate of `2N7002` N-MOSFET | High-speed logic edge (< 75 ns transition) |
| **2N7002 Drain** | CNC 24V E-Stop Terminal | Direct pull-down (< 0.1V saturation, zero CTR degradation) |
| **2N7002 Source** | CNC Isolated Ground | Isolated machine control loop reference |

> **VFD NOISE & ACOUSTIC IMPEDANCE DEFENSE:**  
> 1. **Acoustic Waveguide:** The INMP441 bottom port is mechanically compression-sealed to the spindle casing using a Shore 40A silicone O-ring, creating an enclosed resonant cavity isolating airborne factory noise (> 30 dB attenuation).  
> 2. **I2S Shielded Line:** I2S lines (SCK, WS, SD) run over Shielded Twisted Pair (STP) with 33Ω series damping resistors at both ends and a NiZn ferrite choke to reject high dv/dt VFD motor switching transients.

---

## 4. Step-by-Step Bench Assembly Instructions

1. **Mount ESP32-S3**: Insert the ESP32-S3 dev board across the center divider of the MB-102 breadboard so row pins on both sides remain accessible.
2. **Wire INMP441**: Connect `GND` and `3.3V` first. Connect `SCK` → `GPIO 2`, `WS` → `GPIO 3`, and `SD` → `GPIO 4`. Ensure `L/R` pin is firmly strapped to `GND`.
3. **Wire Optocoupler Primary**: Connect ESP32 `GPIO 5` in series with a 330Ω resistor to PC817 Pin 1 (Anode). Connect Pin 2 (Cathode) to ESP32 `GND`.
4. **Wire Optocoupler Secondary**: Connect the 9V battery red lead (+) to PC817 Pin 4 (Collector). Connect PC817 Pin 3 (Emitter) to the Red LED long leg (Anode). Connect the short leg (Cathode) through a 330Ω resistor back to the black lead (-) of the 9V battery.
5. **Firmware Flash**: Connect USB-C data cable from PC to the ESP32-S3 USB / COM port. Compile and flash `esp_dsp_firmware.cpp` using ESP-IDF v5.x.
