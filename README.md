# DuoDX

A Windows graphical IQ recorder for SDRplay RSP receivers, built primarily for medium wave (MW) DX work, with support for adjacent bands and portions of shortwave.

DuoDX's standout feature is native support for the RSPduo's two tuners recording **simultaneously**: either phase-coherent on the same frequency (for antenna phasing / diversity work in your own post-processing software), or on two genuinely independent frequencies — even different bands entirely — each to its own file.

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

## Features

- Native Win32 GUI — live signal meters, status readouts, colour-coded log, no installer or runtime dependencies beyond the SDRplay API itself.
- Recording in **Linrad raw**, **WavViewDX-raw**, **SDRuno WAV**, **Winrad WAV**, **SDR Connect WAV**, **Perseus WAV**, and **Jaguar WAV** formats.
- **RSPduo dual-tuner recording**:
  - Same-frequency phase-coherent interleaved capture, for antenna phasing/diversity analysis (Linrad raw and WavViewDX-raw).
  - Master/Slave mode — genuinely independent Tuner 1 / Tuner 2 frequencies, each to its own file, in any output format.
- **Low-IF mode** optimised for MW DX (1620/2048 kHz IF, eliminates the DC spike at centre frequency), alongside Zero-IF for wider and shortwave captures.
- Scheduling: single deferred start, hourly recording windows, and up to 32-entry multi-recording overnight schedules that repeat automatically each night.
- Live audio monitor, entirely separate from the recording path — AM/LSB/USB/CW/FM demodulation, adaptive AGC, scrollable per-digit tuning, IF notch, and a narrowband S-meter — for listening to and adjusting a session without generating throwaway test files.
- Lock-free ring buffer with automatic zero-fill gap compensation, so a disk overflow never desyncs the recording's timeline.
- Built-in HTTP status dashboard for monitoring from a phone or browser on the local network.
- In-app Settings dialog covering nearly all of `duodx.ini`, editing the file in place and applying changes immediately — no restart required for most settings.

See the [User Guide](DuoDX_User_Guide_3_1_0.docx) for full documentation.

<img width="1282" height="938" alt="image" src="https://github.com/user-attachments/assets/de74e89a-30e2-42d2-9b44-40ca7e83263a" />

## Supported Devices

| Device | Antenna Inputs | Notes |
|---|---|---|
| RSP1 | 1 × SMA | Basic single-channel device |
| RSP1A / RSP1B | 1 × SMA | RF notch, DAB notch, Bias-T |
| RSP2 | A + B (SMA) | RF notch, antenna switching, Bias-T on Port B |
| RSPduo | T1: Hi-Z + 50 Ω SMA; T2: 50 Ω SMA | Dual-tuner mode, Hi-Z AM notch, Bias-T on Tuner 2 only |
| RSPdx / RSPdxR2 | A, B (SMA), C (BNC) | HDR mode, RF notch, DAB notch, Bias-T on Port B only |

## Requirements

- Windows 10 (version 1511+) or Windows 11.
- [SDRplay API](https://www.sdrplay.com/api/) 3.0 or later, installed and running as a service.
- SDRplay device drivers (installed as part of the SDRplay software package).

## Getting Started

1. Download `duodx.exe` and `duodx.ini` and place them together in any folder.
2. Edit `duodx.ini` (or use the in-app Settings dialog once it's running) to set your frequency, output format, gain, and antenna.
3. Launch `duodx.exe`. Press **Record** to start, or **Monitor** to listen live without recording.

If DuoDX fails to start with a missing-DLL error, copy `sdrplay_api.dll` from `C:\Program Files\SDRplay\API\x64\` into the same folder as `duodx.exe`.

## Building from Source

DuoDX is a single C source file (`duodx.c`), cross-compiled from Linux with MinGW-w64, or natively with MSYS2/MinGW on Windows.

```bash
x86_64-w64-mingw32-gcc -Wall -O2 duodx.c -o duodx.exe \
    -I/path/to/sdrplay_api/headers \
    -lgdi32 -lcomctl32 -lwinmm -lws2_32
```

You'll need the SDRplay API headers (`sdrplay_api.h` and friends) available on your include path — these ship with the SDRplay API installer, or can be sourced from SDRplay's own developer documentation. The exact link flags above are indicative rather than a verified build command — adjust to match whichever libraries your toolchain actually needs; every compile check during this project's development was compile-only (`-c`), not a full link against the real Windows import libraries.

## Configuration Reference

Nearly every setting is editable from the in-app Settings dialog. A handful of advanced or list-shaped settings (device selection by serial, log file path, AGC feedback-loop tuning, and the multi-entry schedule itself) remain INI-only — see the User Guide's Appendix and INI Key Quick Reference for the complete list.

## Acknowledgements

- Linrad is copyright © Leif Åsbrink, SM5BSZ. The Linrad raw file format is documented at [sm5bsz.com](https://www.sm5bsz.com/linuxdsp/linrad.htm).
- WavViewDX is copyright © Reinhard Weiß; its file naming convention is used by the `wavviewdx` output format.
- Perseus is copyright © Microtelecom S.r.l.; the Perseus and Jaguar output formats match the file layout their respective software produces.
- DuoDX was inspired by [rsp-recorder](https://github.com/fventuri/rsp-recorder) by Franco Venturi.
- SDRplay, RSPduo, RSPdx, and related names are trademarks of SDRplay Ltd. DuoDX uses the SDRplay API, copyright SDRplay Ltd.

DuoDX is an independent, unofficial project and is not affiliated with or endorsed by SDRplay Ltd, Leif Åsbrink, Reinhard Weiß, or Microtelecom S.r.l.

## License

*(Not yet specified — add your chosen license here, e.g. MIT, GPL-3.0.)*
