# Changelog

All notable changes to DuoDX are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [2.1.2] - 2026-07-04

### Added
- Record Now button: appears while waiting for a scheduled start; runs an immediate ad-hoc recording using current INI settings, then resumes waiting for the original scheduled time without disturbing the schedule.
- Duration shown on the disk-info line (e.g. DUR: 9h10m).
- Startup warning when schedule entries are not in chronological order.

### Fixed
- Pressing Stop during a scheduled wait no longer closes the application.
- Scheduling status text now clears when recording starts and is not overwritten during active recording.
- Log messages referencing Ctrl+C updated to reference the Stop button.

## [2.1.1] - 2026-06-29

### Added
- Green COHERENT indicator on the disk-info line, shown only while recording on an RSPduo in dual-channel mode with both tuners set to the same frequency (the phase-coherent diversity condition).
- Device-specific validation for `lna_state` (and `lna_state_b`) against the correct maximum for the connected device.
- Validation for `duration_sec`, `ring_buffer_sec`, `ppm`, and `hdr_bw_khz` with explanatory error messages.
- Startup log warning when `agc_enable=1`, explaining that `gain_reduction` will have little visible effect while AGC is on.

### Fixed
- `lna_state_b=-1` (the "inherit from Tuner A" default) was incorrectly rejected by the device-specific LNA validation added in 2.1.0's later builds, breaking RSPduo recordings that left Tuner B settings at their default. The inherited value is now resolved before validating.
- A spurious "RSPduo AM port select failed: sdrplay_api_NotInitialised" warning when a multi-entry schedule applied an antenna setting between recordings after the device had already been uninitialised. The antenna setting is still applied correctly at the next recording's startup.

### Documentation
- Clarified that AGC overrides `gain_reduction` in real time, and that gain changes have little effect while AGC is enabled.
- Clarified that the signal meters give a general indication of RF level for monitoring, not a precise calibrated measurement.
- Documented the new COHERENT indicator.

## [2.1.0] - 2026-06-28

### Added
- Graduated and greyscale meter styles via `meter_style` INI key (0=zone, 1=graduated, 2=greyscale).
- Optional live clock top-right, via `show_clock` INI key. Labelled UTC or local to match time mode.
- Dedicated overload indicator segment at the far right of each meter bar.
- AGC button turns green when AGC is enabled; shows N/A in amber and is disabled in HDR mode.
- Scheduling status text along the bottom of the window.
- Coloured AGC: ON/OFF and HDR: ON/OFF indicators on the disk line.
- Application icon (duodx.ico) embedded via Windows resource file (duodx.rc), visible on the taskbar, Alt+Tab switcher, and title bar.
- Version information block in the executable (visible in Properties → Details).

### Changed
- Recording LED fixed at a constant position — no longer shifts when state word changes length.
- LED colours updated to traffic-light red (recording) and bright green (finished).
- Start/Stop and AGC buttons moved to a bottom bar under the log window.
- Log window scrolls to bottom on each new line.
- Clock aligned to the left edge of the state text.
- Window default height increased to 660 pixels.
- Ring buffer fill shown with one decimal place.
- Dark scrollbar theme applied to the log window.

### Fixed
- App no longer closes when a recording finishes (window stays open showing FINISHED).
- Elapsed time retained after scheduled or hourly recording stops.
- Hourly recording no longer reuses the previous session's filename (error 32).
- Stop during inter-recording wait in hourly mode no longer crashes the application.
- Clock no longer briefly appears at startup when show_clock=0.
- Overload indicator segment now lights correctly on hardware ADC overload.
- Recording duration rounded to nearest second (40s no longer displays as 39s).
- start_time_utc accepted as alias for start_time (no longer logs Unknown config key).
- Completed recording files are now closed immediately after each recording, so they are accessible to other applications (WavViewDX, HxD etc.) without closing DuoDX.
- Stop button no longer flickers during inter-recording wait.
- Scheduling status text now visible during inter-recording wait (hourly next time, scheduled start time).

## [2.0.0] - 2026-06-27

Initial Win32 GUI release replacing the console application.

### Added
- Native Win32 graphical interface with a dark theme.
- Start/Stop toggle button and AGC button.
- Recording status LED, channel A/B signal meters, live counter tiles.
- Colour-coded log window.
- Centre-frequency readout with coverage span.
- Disk free space read and displayed at startup.

### Changed
- All settings now read from duodx.ini only — no command-line options.
- use_utc governs the scheduler, log timestamps, filenames and clock.
- Window header and title bar read "RSP IQ Recorder".
- File-size derived from live written-sample count.

### Removed
- All command-line options.
- Keyboard runtime controls (G key); AGC is now the on-screen button.
- Console status line and ANSI colour handling.

### Fixed
- App stays open after recording finishes.
- AGC debounced to prevent NotInitialised errors on rapid toggling.
- start_time_utc alias recognised.

## [1.2.8] - 2026

Final console release. Linrad, WavViewDX, SDRuno and SDR Connect output formats; RSPduo
dual-tuner recording; scheduled, hourly and repeating recording; lock-free ring buffer with
zero-fill compensation; HTTP remote monitoring dashboard; named-pipe streaming;
post-recording verification.

[2.1.2]: https://github.com/45south/DuoDX-recorder/releases/tag/v2.1.2
[2.1.1]: https://github.com/45south/DuoDX-recorder/releases/tag/v2.1.1
[2.1.0]: https://github.com/45south/DuoDX-recorder/releases/tag/v2.1.0
[2.0.0]: https://github.com/45south/DuoDX-recorder/releases/tag/v2.0.0
[1.2.8]: https://github.com/45south/DuoDX-recorder/releases/tag/v1.2.8
