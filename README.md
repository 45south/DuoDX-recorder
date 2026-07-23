# DuoDX

**RSP Dual Channel IQ Recorder** — a Windows native GUI application for recording raw I/Q data from SDRplay RSP receivers, with a live audio monitor, genuine dual-tuner support on RSPduo, and RSPduo Master/Slave mode for two entirely independent frequencies at once.

Version 3.0.0

## Features

- Records raw I/Q to Linrad-compatible format, plus SDRuno and SDR Connect WAV output
- Full single-tuner support for all RSP models (RSP1, RSP1A, RSP1B, RSPdx, RSPdx-R2)
- Genuine dual-channel recording on RSPduo — both tuners to one interleaved file
- RSPduo Master/Slave mode — two independently tuned frequencies, each to its own file, including entirely different bands (e.g. medium wave on one tuner, shortwave on the other)
- Live audio monitor, independent of the recording path: AM (6/4/2.4 kHz), FM-N, FM-W, LSB, USB, and CW demodulation, with its own AGC, volume, low-cut filter, and scrollable-digit frequency tuning
- Carrier frequency offset display (AM mode) — measures how far a station's actual carrier sits from the dial, with adaptive integration time for weak/DX-level signals
- Narrowband S-meter for the live monitor, showing the tuned station's own strength
- In-app Settings dialog covering nearly every commonly-changed option, patching `duodx.ini` in place without disturbing anything not shown in the dialog
- Multi-entry and hourly scheduling, with a live HTTP status dashboard and an optional named-pipe interface
- HDR mode, Bias-T, antenna selection, IF notch, DC/IQ correction — all per-tuner where the hardware supports it

See `DuoDX_User_Guide_3_0_0.docx` for full usage documentation, and `DuoDX_Version_History.docx` for the complete changelog back to the original 1.x console version.

## Requirements

- Windows 10 or 11 (x64)
- [SDRplay API](https://www.sdrplay.com/api/) (v3.x) installed, with the appropriate device drivers
- An SDRplay RSP-series receiver (RSP1, RSP1A, RSP1B, RSPdx, RSPdx-R2, or RSPduo)

## Building

Requires MinGW-w64 (via [MSYS2](https://www.msys2.org/) is the easiest route) and the SDRplay API's headers and import library.

```sh
gcc -O2 -o duodx.exe duodx.c -I"C:\Program Files\SDRplay\API\inc" \
    -L"C:\Program Files\SDRplay\API\x64" -lsdrplay_api -lwinmm -mthreads
```

Or with MSVC:

```bat
cl /O2 duodx.c /I"C:\Program Files\SDRplay\API\inc" ^
   /link "C:\Program Files\SDRplay\API\x64\sdrplay_api.lib" winmm.lib
```

Adjust the include/library paths if the SDRplay API is installed somewhere other than the default location. No other third-party dependencies are required — everything else is drawn from the standard Windows API.

## Running

Place `duodx.exe` and `sdrplay_api.dll` (or ensure the SDRplay API is installed so the DLL is on the system path) in the same folder, and run it. A `duodx.ini` will be created alongside the executable on first run with sensible defaults; see the User Guide for the full list of settings, both those exposed in the in-app Settings dialog and the smaller number that are INI-only.

## Output formats

DuoDX writes raw, uncompressed 16-bit interleaved I/Q samples in a Linrad-compatible format by default, and can optionally write directly in SDRuno or SDR Connect WAV format instead. See the User Guide's Recording section for the exact byte layout of each.

## License

Not yet specified — see the repository owner before reusing this code.

## Acknowledgements

Built against the [SDRplay API](https://www.sdrplay.com/api/). SDRplay and RSP are trademarks of SDRplay Limited; this project is not affiliated with or endorsed by SDRplay.
