# Changelog

This file summarizes the 3.0.0 release line. For the complete, detailed changelog back to the original 1.x console version — including every fix, INI key, and section reference into the User Guide — see `DuoDX_Version_History.docx`.

## 3.0.0

Major feature release: live signal monitor, RSPduo Master/Slave mode, in-app Settings dialog, and genuine dual-tuner Listening. (2.2.0 was developed but never released; its changes are folded into 3.0.0 below.)

### Added
- Live audio signal monitor, independent of the recording path: AM (6/4/2.4 kHz), FM-N, FM-W, LSB, USB, and CW demodulation, with a feedback-loop AGC, scrollable-digit tuning, an independent IF notch, and a Tuner A/B selector
- Carrier frequency offset display (AM mode) — measures the difference between the dial and a station's actual carrier, with adaptive integration time (1–12s) so strong signals lock in seconds while weak/DX-level signals integrate longer for a reliable reading; shows a "(lock)" indicator once the reading has genuinely settled
- RSPduo Master/Slave mode — independently tuned Tuner A/B frequencies, each recorded to its own file, including entirely different bands
- Dual-channel and Master/Slave Listening — Monitor now opens both tuners at once for these session types
- Narrowband S-meter for the live monitor, with a gain-compensated, user-calibratable dBm reading
- In-app Settings dialog (Receiver, Recording, Monitor, Network, Schedule, Miscellaneous tabs), patching `duodx.ini` in place
- Genuine independent Tuner 2 AGC/DC/IQ/notch settings
- A=B frequency lock for quickly comparing tuners/antennas at the same frequency
- Crash reports written to disk (diagnostic `.txt` and, where possible, a `.dmp` minidump)
- Window position/size and Volume level remembered across restarts
- Auto-saved per-session log file (independent of the INI-only `log_file` setting)
- Schedule Enable/Disable toggle, always visible
- Dark Grey color scheme option (now default for new installs)

### Changed
- The device-info status line (RSPdx / Ant / GR / LNA / SR) has been removed from the main window
- Monitor AGC now uses a proper envelope follower for its level detection instead of reacting to raw instantaneous samples, reducing audible pumping/breathing on program audio
- `ppm` frequency correction now accepts ±1000 instead of ±100 — SDRplay's API documents no hard limit for this value
- Antenna, Bias-T, and Hi-Z notch now apply live while Listening, not just at the next Record/restart

### Fixed
- A crash that could occur when opening Settings immediately after stopping a session, before the Tuner button visually went inactive (a race between Settings' own device probe and the still-finishing device teardown)
- Numerous Timer/Schedule/Hourly interaction bugs (see the full history document for the complete list)

## 2.1.x (2.1.0–2.1.3)

Significant GUI improvements: graduated/greyscale meter styles, live clock, coloured AGC/HDR indicators, scheduling status text, and a range of stability fixes around scheduled and hourly recording.

## 2.0.0

First graphical version of DuoDX, moving on from the original command-line recorder.

## 1.x

Original command-line console recorder for SDRplay devices.
