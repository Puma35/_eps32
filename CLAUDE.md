# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Embedded firmware for the **Seeed Studio XIAO ESP32S3 Sense** that performs Voice Activity Detection (VAD) and saves audio (WAV) + photo (JPEG) sessions to an SD card. Written in C++ using the Arduino framework, built with PlatformIO.

## Build & Flash Commands

```bash
# Build and upload to device
pio run --target upload --environment xiaos3sense

# Serial monitor (115200 baud, USB CDC)
pio device monitor --environment xiaos3sense

# Build only (no upload)
pio run --environment xiaos3sense
```

There are no tests, linters, or CI pipelines.

## Setup Prerequisite

Before building, copy `src/credentials.h.example` to `src/credentials.h` and fill in `WIFI_SSID` and `WIFI_PASSWORD`. This file is gitignored and required for compilation.

## Architecture

All firmware lives in a single file: **`src/main.cpp`** (~679 lines). The flow on boot is:

1. **WiFi + NTP sync** → sets RTC for session timestamps (timezone: France/CET). Falls back to epoch if WiFi is unavailable within 8 seconds.
2. **Noise floor calibration** → 1500 ms of silence to seed the adaptive EMA (`noise_ema`).
3. **Main loop** → continuously reads I2S PDM audio in 30 ms chunks through the DSP pipeline, running VAD logic on each chunk.

### DSP / VAD Pipeline

Each 30 ms audio chunk goes through:

1. **Biquad Butterworth bandpass filter** (300–3400 Hz, Direct Form II):
   - 2nd-order high-pass at 300 Hz
   - 2nd-order low-pass at 3400 Hz
2. **RMS AC** — DC-offset-removed energy measurement
3. **ZCR (Zero Crossing Rate)** — discriminates voice from impulse noise

VAD uses a **3-criterion, 3-vote system**:
- RMS > `vad_trigger` (= `noise_ema × 6.0`)
- ZCR in voice range (3–180 crossings per 30 ms chunk)
- 3 consecutive valid chunks required to start a session

Thresholds adapt continuously: `vad_trigger = noise_ema × 6.0`, `vad_silence = noise_ema × 3.0`.

### Session Recording

When VAD triggers, the firmware:
1. Flushes a **300 ms circular pre-roll buffer** (10 × 30 ms chunks) into the WAV file
2. Continues recording until 5 s of silence or 120 s maximum
3. **After** audio ends, captures a single JPEG photo (to avoid I2S/camera bus conflicts)
4. Writes everything to SD card under `/session_YYYYMMDD_HHMMSS/audio.wav` and `/session_YYYYMMDD_HHMMSS/photo.jpg`

Audio output: 16 kHz, 16-bit signed PCM, mono, ×8 gain applied, DC-removed, proper WAV header.

### Hardware Peripherals

| Peripheral | Interface | Key GPIOs |
|---|---|---|
| PDM Microphone | I2S (PDM mode) | CLK=GPIO42, DATA=GPIO41 |
| OV2640 Camera | Parallel (8-bit) | Multiple data/ctrl pins |
| SD Card | SPI | CS=GPIO21, SCK=GPIO7, MISO=GPIO8, MOSI=GPIO9 |

Camera is configured for UXGA (1600×1200), JPEG quality 10, 180° rotation (vflip + hmirror), manual exposure (150/1200) and manual gain (×20).

### Key Global State

- `noise_ema` (float) — exponential moving average of background noise floor
- `vad_trigger` / `vad_silence` (uint16_t) — adaptive thresholds derived from `noise_ema`
- `recording` (bool) — true while a session is active
- Pre-roll circular buffer: 10 slots × 30 ms of PCM samples

## Key Build Flags (`platformio.ini`)

- `-DBOARD_HAS_PSRAM` — enables the 8 MB OPI PSRAM required for camera framebuffers
- `-DARDUINO_USB_CDC_ON_BOOT=1` — routes `Serial` over USB CDC (no UART adapter needed)
- `board_build.flash_size = 8MB` / `board_build.partitions = default_8MB.csv`
