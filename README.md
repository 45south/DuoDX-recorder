# DuoDX

A high-reliability SDR recording application for Windows, optimised for medium wave DXing with SDRplay hardware (RSPdx, RSPdx R2, RSPduo, RSP1A, RSP2).

Records directly to Linrad raw, WavViewDX, SDRuno WAV, or SDR Connect WAV format. Supports single-tuner and RSPduo dual-tuner coherent recording, multi-session overnight scheduling, and browser-based remote monitoring over a local network.

---

## Features

- Single and dual-channel (RSPduo) recording
- Output formats: Linrad raw, WavViewDX, SDRuno WAV, SDR Connect WAV
- Multi-recording overnight scheduler with `schedule_only` mode
- HTTP status dashboard — monitor live recordings from a phone or browser on your local network
- Ring buffer with overflow detection and zero-fill logging
- Drive spin-up pre-write to prevent gaps on idle hard disks
- HDR mode support (RSPdx / RSPdx R2)
- Named pipe for real-time IQ monitoring by a compatible client
- Scheduled start time (UTC) with 12-hour window logic
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
git clone https://github.com/yourusername/duodx.git
cd duodx
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

To clean build output:

```bash
make clean
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
| `schedule_only` | 0 | 1 = skip to schedule_1, ignore top-level recording |

See the full User Guide for complete documentation.

---

## Remote Monitoring

Set `http_port = 8080` in `duodx.ini` to enable the browser-based status dashboard. Open `http://<PC-IP>:8080/` from any browser on your local network, including a phone.

To allow access from other devices, add a Windows Firewall rule once (run as Administrator):

```
netsh advfirewall firewall add rule name="DuoDX HTTP Status" dir=in action=allow protocol=TCP localport=8080
```

Find your PC's IP address with `ipconfig` and look for the IPv4 Address under your active network adapter.

---

## Overnight Scheduling

Use `schedule_only = 1` with numbered `schedule_N_*` entries for unattended overnight recordings:

```ini
schedule_only = 1

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

**Note:** Start DuoDX before the first `schedule_1_start_time`. If launched after that time has already passed today, recording will start immediately.

---

## Documentation

Full documentation is available in `DuoDX_User_Guide.pdf`, included in each release.

---

## Acknowledgements

DuoDX was inspired by [rsp-recorder](https://github.com/fventuri/rsp-recorder) by Franco Venturi. The author wishes to thank Franco for making rsp-recorder publicly available; working with it was the original motivation for developing DuoDX.

DuoDX was developed with the assistance of [Claude](https://www.anthropic.com) (Anthropic), which contributed substantially to the C programming, debugging, and documentation throughout the project.

WavViewDX is copyright © Reinhard Weiß. Linrad is copyright © Leif Asbrink SM5BSZ

SDRplay, RSPduo, RSPdx, and related product names are trademarks of SDRplay Ltd.

---

## License

See [LICENSE](LICENSE) for details.
