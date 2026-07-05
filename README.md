# DuoDX

**RSP IQ Recorder for Windows** — a native GUI application that captures I/Q
samples from SDRplay RSP receivers and writes them to disk in Linrad raw,
WavViewDX-raw, SDRuno WAV, or SDR Connect WAV format.

Optimised for medium wave (MW) DXing with the RSPduo and RSPdx, including
native RSPduo dual-tuner recording of two frequencies to a single interleaved
file.

---

### ⚠️ A Quick Word of Warning: This is an AI-Assisted Project ⚠️

**This project is a personal project for my dx hobby.**

A large language model (AI) helped write a significant portion of this code, *if not most.* I guided it, reviewed its output the best I could, and tested the result, but this project didn't evolve through the typical trial-and-error of a human-only endeavor. Even this README you're reading was drafted by the AI based on the source code, then edited and refined by me.

Second, it's worth knowing that this was a learning project for myself. The focus was always on getting a practical, working result, which means some of the solutions are probably not what you'd find in a textbook. 

*What does this mean?*

*   **This is experimental.** While it works, it hasn't been battle-tested across all possible options and hardware.
*   **Design choices not stable.** You may see features etc. suddenly appear and disappear. You may also see large commits of lots of changes. 
*   **Bugs are expected.** The logic very likely has quirks that haven't been discovered yet. 

---


> Version 2.1.2 fixes an RSPduo LNA validation bug, adds device-specific parameter validation, and adds a COHERENT indicator for dual-channel phase-coherent recording. Version 2.1.0 added graduated meter styles, an optional live clock, and numerous UI improvements. Version 2.0.0 was the initial GUI release replacing the original console tool. All
> settings are now read from `duodx.ini`; there are no command-line options.

## Features

- Single and dual-channel recording (Linrad raw, WavViewDX-raw, SDRuno WAV,
  SDR Connect WAV).
- RSPduo dual-tuner mode for simultaneous reception on two frequencies.
- Low-IF mode tuned for MW, plus Zero-IF for wider/shortwave captures.
- Scheduled, hourly and repeating unattended recording.
- Lock-free ring buffer with zero-fill gap compensation to preserve timing.
- Live GUI: signal meters, elapsed/file/overflow/dropped/ring-buffer readouts,
  colour-coded log, recording LED, and an optional clock.
- Built-in HTTP dashboard for monitoring from a phone or browser on the LAN.
- Post-recording file verification.

## Requirements

- Windows 10 (1511+) or Windows 11.
- [SDRplay API 3.x](https://www.sdrplay.com/) installed and the API service
  running.
- A supported SDRplay device (RSP1/1A/1B, RSP2, RSPduo, RSPdx/RSPdxR2).

If `sdrplay_api.dll` is not found at startup, copy it from
`C:\Program Files\SDRplay\API\x64\` into the same folder as `duodx.exe`.

## Build

Built with MinGW-w64 (MSYS2). From the project folder:

```
make
```

This produces a windowed `duodx.exe` with the application icon embedded.
The Makefile uses `windres` to compile `duodx.rc` (which references `duodx.ico`)
into the executable. All three source files — `duodx.c`, `duodx.rc`, and
`duodx.ico` — must be in the same folder.

Override the SDRplay API location if it is installed elsewhere:

```
make SDR_API="D:/SDRplay/API"
```

`make debug` builds an unstripped binary with a console attached for
diagnostics. See the `Makefile` for details.

## Usage

1. Place `duodx.ini` in the same folder as `duodx.exe` (copy and edit
   `duodx.example.ini` to start).
2. Run `duodx.exe`.
3. Set your frequency, format, gain and schedule in `duodx.ini`, then press
   **Start**.

All configuration is documented in the user guide. Times follow the `use_utc`
setting (UTC by default) consistently across the scheduler, log and filenames.

## Documentation

Full documentation — every INI key, IF/sample-rate combinations, output file
formats, dual-tuner and phasing setup, scheduling, HDR mode, and the HTTP
dashboard — is in the **DuoDX User Guide** (`DuoDX_User_Guide_2_1_2.docx`) in
this repository.

## Acknowledgements

Linrad is © Leif Åsbrink SM5BSZ. WavViewDX is © Reinhard Weiß. DuoDX was
inspired by [rsp-recorder](https://github.com/fventuri/rsp-recorder) by Franco
Venturi. Developed with the assistance of Claude (Anthropic). SDRplay, RSPduo,
RSPdx and related names are trademarks of SDRplay Ltd. DuoDX is an independent
application and is not affiliated with or endorsed by SDRplay Ltd, Leif
Åsbrink, or Reinhard Weiß.

## Author

Dave Headland — <https://github.com/45south/DuoDX-recorder>
