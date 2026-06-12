# Changelog

All notable changes to DuoDX are documented here.

---

## [1.2.7] - 2026

### Added
- `schedule_repeat = 1` INI key — when enabled with `schedule_only = 1`, DuoDX automatically reloads the schedule and waits for the next day's first scheduled time after all entries complete. Runs indefinitely until Ctrl+C.
- Remote monitoring: elapsed card now shows **NEXT AT HH:MMZ** in orange between schedule entries and before the first recording starts.
- Remote monitoring: elapsed card shows **FINISHED** in gold when all recordings complete (with `schedule_repeat = 0`).
- Remote monitoring: HTTP server now starts before the scheduled wait, so the dashboard is accessible immediately when DuoDX launches.

### Fixed
- Scheduling: instant recording when `schedule_only = 0` with schedule entries present — entries are now completely ignored when `schedule_only = 0`.
- Scheduling: between-recording waits incorrectly rolling over to next day when a recording overran slightly. Entries within 5 minutes of their scheduled time now start immediately.
- Remote monitoring: HTTP listening message no longer interrupts the scheduled countdown line in the console.
- Duration report: fixed incorrectly large duration values in recording complete statistics.

---

## [1.2.6]

### Fixed
- Remote monitoring: post-recording dashboard now correctly shows frozen final elapsed time, file size, and disk free after recording ends.
- Remote monitoring: dashboard update loop no longer breaks after recording completes.
- Scheduling: `schedule_only` initialisation order bug fixed.

### Added
- Remote monitoring: elapsed card shows **FINISHED** in gold after all recordings complete.
- Remote monitoring: AGC card relabels to **HDR** and shows "HDR enabled" when `hdr_enable = 1` (RSPdx only).
- Remote monitoring: `http_interval_ms` INI key for configurable dashboard refresh interval (500–30000 ms, default 2000 ms).

---

## [1.2.5]

### Added
- HTTP remote monitoring server (`http_port` INI key). Live dashboard accessible from any browser on the local network including phones and tablets.
- `schedule_only` INI key — skips top-level recording and starts directly from `schedule_1`.
- Post-recording hold — HTTP server stays alive after recordings finish until Ctrl+C.

### Fixed
- Multi-recording schedule: duplicate writer thread creation between entries.
- Multi-recording schedule: frequency update now only calls live API when stream is running.
- Multi-recording schedule: elapsed time and file size reset correctly between entries.
- Dangling `ch_a_params` pointer after `sdrplay_api_ReleaseDevice` causing crash when HTTP dashboard polled after recording ended.

---

## [1.2.4]

### Added
- SDR Connect WAV output format (`output_format = sdrconnect`).
- SDRuno WAV output format (`output_format = sdruno`).
- Drive spin-up pre-write (`spinup_enable`, `spinup_bytes`).
- HDR mode support for RSPdx and RSPdx R2 (`hdr_enable`, `hdr_bw_khz`).
- Named pipe real-time IQ output (`pipe_enable`, `pipe_name`).
- Verbose mode (`verbose = 1`).

---

## [1.2.3]

### Added
- Multi-recording schedule (`schedule_N_*` INI keys). Up to 32 entries.
- Dual-channel RSPduo support (`dual_channel = 1`).
- Scheduled UTC start time (`start_time_utc`).

---

## [1.2.2]

### Added
- WavViewDX raw output format (`output_format = wavviewdx`).
- IQ imbalance correction (`iq_correct`).
- DC offset correction (`dc_correct`).
- RF and DAB notch filters (`notch_rf`, `notch_dab`).

---

## [1.2.1]

### Added
- Initial public release.
- Linrad raw format recording.
- Single-tuner support for RSPdx, RSPduo, RSP1A, RSP2.
- INI file configuration.
- Real-time status bar with dBFS metering, overflow count, ring buffer utilisation, disk free.
- Software decimation (`decimation`).
- Recording verification on completion.
