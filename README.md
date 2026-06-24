# DuoDX

A high-reliability SDR recording application for Windows, optimised for medium wave DXing with SDRplay hardware (RSPdx, RSPdx R2, RSPduo, RSP1A, RSP2).

Records directly to Linrad raw, WavViewDX, SDRuno WAV, or SDR Connect WAV format. Supports single-tuner and RSPduo dual-tuner coherent recording, multi-session overnight scheduling with automatic nightly repeat, and browser-based remote monitoring over a local network.

---

### ⚠️ A Quick Word of Warning: This is an AI-Assisted Project ⚠️

**This project is a personal project for my dx hobby.**

Let's be upfront: a large language model (AI) helped write a significant portion of this code, *if not most.* I guided it, reviewed its output the best I could, and tested the result, but this project didn't evolve through the typical trial-and-error of a human-only endeavor. Even this README you're reading was drafted by the AI based on the source code, then edited and refined by me.

Second, it's worth knowing that this was a learning project for myself. The focus was always on getting a practical, working result, which means some of the solutions are probably not what you'd find in a textbook. 

*What does this mean?*

*   **It's Experimental.** While it works, it hasn't been battle-tested across a wide variety of SDR formats.
*   **Design choices not stable.** You may see features etc. suddenly appear and disappear. You may also see large commits of lots of changes. **The mainline codebase may also be broken at times due to fast moving code and changes. Releases may be more stable.** 
*   **Bugs are expected.** The logic very likely has quirks that haven't been discovered yet. Other issues causing crashes likely exist too. 
*   **Use with caution!** Check edited and converted files before deleting your original recordings, if they are important to you.

---

## Features

- Single and dual-channel (RSPduo) recording
- Output formats: Linrad raw, WavViewDX, SDRuno WAV, SDR Connect WAV
- Multi-recording overnight scheduler with `schedule_only` and `schedule_repeat` modes
- HTTP status dashboard — monitor live recordings from any phone or browser on your local network
- Ring buffer with overflow detection and zero-fill logging
- Drive spin-up pre-write to prevent gaps on idle hard disks
- HDR mode support (RSPdx / RSPdx R2)
- Named pipe for real-time IQ monitoring by a compatible client
- Signal level metering (peak dBFS) for both tuners
- INI file configuration with CLI overrides

---

## Requirements

### Hardware
- SDRplay RSPdx, RSPdx R2, RSPduo, RSP1A, or RSP2

### Software
- Windows 10 or 11 (64-bit)
- SDRplay API 3.x — installed automatically with SDRuno or SDR Connect
  - Download standalone: https://www.sdrplay.com/api/
- MSYS2 / MinGW-w64 (for building from source)
  - Download: https://www.msys2.org/

---

## Building from Source

### 1. Install MSYS2
Download and install MSYS2 from https://www.msys2.org/, then open the **MSYS2 MinGW64** shell and install the toolchain:

```bash
pacman -S mingw-w64-x86_64-gcc mingw-w64-x86_64-make
```

### 2. Clone the repository

```bash
git clone https://github.com/45south/DuoDX-recorder.git
cd DuoDX-recorder
```

### 3. Verify the SDRplay API path

The Makefile assumes the SDRplay API is installed at:

```
C:/Program Files/SDRplay/API/inc/    (headers)
C:/Program Files/SDRplay/API/x64/   (libraries)
```

If your installation is elsewhere, edit the `SDRPLAY_INC` and `SDRPLAY_LIB` variables at the top of the Makefile.

### 4. Build

```bash
make
```

This produces `duodx.exe` in the current directory. For a debug build:

```bash
make debug
```

---

## Configuration

Copy `duodx.ini` to the same directory as `duodx.exe` and edit it to suit your setup. All settings are documented inline in the INI file. Key settings:

| Key | Default | Description |
|---|---|---|
| `frequency_mhz` | 0.875 | Centre frequency in MHz |
| `output_format` | linrad | `linrad` \| `wavviewdx` \| `sdruno` \| `sdrconnect` |
| `duration_sec` | 0 | Recording duration. 0 = unlimited |
| `antenna` | A | Antenna input: A \| B \| C |
| `hdr_enable` | 0 | HDR mode (RSPdx only) |
| `http_port` | 0 | HTTP monitor port. 0 = disabled |
| `schedule_only` | 0 | 1 = use schedule entries only, no instant recording |
| `schedule_repeat` | 0 | 1 = repeat schedule nightly (requires `schedule_only = 1`) |

See the full User Guide for complete documentation.

---

## Remote Monitoring

Set `http_port = 8080` in `duodx.ini` to enable the browser-based status dashboard. Open `http://<PC-IP>:8080/` from any browser on your local network, including a phone.

To allow access from other devices, add a Windows Firewall rule once (run as Administrator):

```
netsh advfirewall firewall add rule name="DuoDX HTTP Status" dir=in action=allow protocol=TCP localport=8080
```

The dashboard shows elapsed time, file size, disk free, signal levels (dBFS), overflow count, AGC/HDR state, and alerts. It updates every 2 seconds by default (configurable via `http_interval_ms`).

Before the first recording starts and between schedule entries, the elapsed card shows **NEXT AT HH:MMZ** in orange. After all recordings finish it shows **FINISHED** in gold.

---

## Overnight Scheduling

Use `schedule_only = 1` with numbered `schedule_N_*` entries for unattended overnight recordings. Add `schedule_repeat = 1` to repeat automatically every night:

```ini
schedule_only   = 1
schedule_repeat = 1

schedule_1_start_time  = 09:00:00
schedule_1_duration    = 14400
schedule_1_frequency   = 0.875
schedule_1_antenna     = A
schedule_1_output_file =

schedule_2_start_time  = 13:00:00
schedule_2_duration    = 14400
schedule_2_frequency   = 1.125
schedule_2_antenna     = A
schedule_2_output_file =
```

DuoDX always waits for a scheduled time — it will never start recording immediately when `schedule_only = 1`, regardless of what time you launch it.

---

## Windows Defender False Positive

Windows Defender may flag `duodx.exe` as `Program:Win32/Wacapew.C!ml`. This is a false positive — only 1 of 70+ antivirus engines detects it, and it is caused by the built-in HTTP monitoring server being flagged by Microsoft's machine learning model.

To resolve:
- Allow the file manually in Defender (**Virus & threat protection → Protection history → Allow**)
- Or submit a false positive report to Microsoft: https://www.microsoft.com/en-us/wdsi/filesubmission

The full source code is available in this repository for review.

---

## Documentation

Full documentation is available in `DuoDX_User_Guide.docx`, included in each release.

---

## Acknowledgements

DuoDX was inspired by [rsp-recorder](https://github.com/fventuri/rsp-recorder) by Franco Venturi. The author wishes to thank Franco for making rsp-recorder publicly available; working with it was the original motivation for developing DuoDX.

DuoDX was developed with the assistance of [Claude](https://www.anthropic.com) (Anthropic), which contributed substantially to the C programming, debugging, and documentation throughout the project.

WavViewDX is copyright © Reinhard Weiß. SDRplay, RSPduo, RSPdx, and related product names are trademarks of SDRplay Ltd.

---

## License

See [LICENSE](LICENSE) for details.
