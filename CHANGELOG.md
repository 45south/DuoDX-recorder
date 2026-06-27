# Changelog

All notable changes to DuoDX are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [2.0.0] - 2026-06-27

Version 2.0.0 is a major release that replaces the command-line console
application with a native Win32 graphical interface. The recording engine
(ring buffer, writer thread, device handling, file formats, scheduling and
HTTP monitoring) is carried over unchanged; the console front end has been
replaced by a windowed UI.

### Added
- Native Win32 GUI front end with a dark theme.
- Start/Stop toggle button and an AGC button.
- Recording status LED (red while recording, green when finished).
- Channel A and B signal-strength bar meters driven by peak dBFS.
- Live counter tiles: elapsed time, file size, overflows, dropped (zero-fill)
  frames, and ring-buffer fill percentage.
- Colour-coded log window (green for success/verification, orange for
  warnings, red for errors, white otherwise).
- Centre-frequency readout with a coverage span (e.g. `125 - 2125 kHz`).
- AGC and HDR state indicators with coloured ON/OFF text.
- Optional live clock in the GUI, controlled by the new `show_clock` INI key.
- Disk free space is read and displayed at startup, before the first recording.
- `show_clock` INI key (default on) to show or hide the live clock.

### Changed
- The application is now a windowed app (`-mwindows`); there is no console
  window and no "Press Enter to close" prompt.
- `use_utc` now governs the GUI clock and the log timestamps as well as the
  scheduler and output filenames, so the whole application uses one clock.
  Log lines carry a `Z` suffix in UTC mode.
- Window header and title bar now read "RSP IQ Recorder".
- The file-size readout is derived from the live written-sample count instead
  of the on-disk size, which the OS write cache made lag during recording.
- The reported total recording duration is rounded to the nearest second so a
  requested 40-second capture no longer displays as 39 seconds.
- `start_time_utc` is accepted as an alias for `start_time`.

### Removed
- All command-line options. Configuration is read from `duodx.ini` only.
- Keyboard runtime controls (the `G` key); AGC is now the on-screen AGC button.
- The console status line and ANSI colour handling.

### Fixed
- Elapsed time now retains the last completed recording's length after a
  scheduled recording stops, instead of resetting to 00:00.
- The application no longer closes automatically when a recording finishes; the
  final FINISHED state and verification output remain on screen.
- Repeated rapid AGC toggling no longer triggers `NotInitialised` errors
  (the toggle is debounced and retried once on a transient failure).
- The `Unknown config key: 'start_time_utc'` warning at startup.

## [1.2.8] - 2026

Final console release. See the user guide for the full feature history of the
1.x command-line versions, including Linrad/WavViewDX/SDRuno/SDR Connect output
formats, RSPduo dual-tuner recording, scheduled and hourly recording, the HTTP
remote monitoring dashboard, and the lock-free ring buffer.

[2.0.0]: https://github.com/45south/DuoDX-recorder/releases/tag/v2.0.0
[1.2.8]: https://github.com/45south/DuoDX-recorder/releases/tag/v1.2.8
