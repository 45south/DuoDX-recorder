# Changelog

All notable changes to DuoDX are documented here.

---

## [1.2.6] - 2026

### Fixed
- Remote monitoring: post-recording dashboard now correctly shows frozen final elapsed time, file size, and disk free after recording ends, rather than reverting to zeros.
- Remote monitoring: dashboard update loop no longer breaks after recording completes.
- Scheduling: `schedule_only` initialisation order bug fixed — settings now correctly applied after config is fully loaded.

### Added
- Remote monitoring: elapsed card shows **FINISHED** in gold after all recordings complete.
- Remote monitoring: AGC card relabels to **HDR** and shows "HDR enabled" when `hdr_enable = 1` (RSPdx only).
- Remote monitoring: `http_interval_ms` INI key for configurable dashboard refresh interval (500–30000 ms, default 2000 ms).

---

## [1.2.5]

### Added
- HTTP remote monitoring server (`http_port` INI key). Serves a live dashboard at `http://<PC-IP>:<port>/` showing elapsed time, file size, disk free, signal levels (dBFS) for both tuners, overflow count, AGC/HDR state, and active alerts. Auto-refreshes at a configurable interval. Accessible from any browser on the local network including phones and tablets.
- `schedule_only` INI key — when set to 1, the top-level recording is skipped and DuoDX starts directly from `schedule_1`. Recommended for unattended overnight multi-recording sessions.
- Post-recording hold — when `http_port` is set, DuoDX keeps the HTTP server alive after all recordings finish and waits for Ctrl+C, allowing the phone dashboard to be checked after an overnight session.

### Fixed
- Multi-recording schedule: corrected duplicate writer thread creation between schedule entries.
- Multi-recording schedule: `sdrplay_api_Update` was being called before stream re-initialisation between recordings. Frequency update now only calls the live API when the stream is running.
- Multi-recording schedule: elapsed time and file size now reset correctly between entries on the remote dashboard.
- Multi-recording schedule: dangling `ch_a_params` pointer after `sdrplay_api_ReleaseDevice` could cause access violation when HTTP worker thread polled after recording ended.

---

## [1.2.4]

### Added
- SDR Connect WAV output format (`output_format = sdrconnect`).
- SDRuno WAV output format (`output_format = sdruno`).
- Drive spin-up pre-write (`spinup_enable`, `spinup_bytes`) to wake idle spinning hard disks before recording begins.
- HDR mode support for RSPdx and RSPdx R2 (`hdr_enable`, `hdr_bw_khz`).
- Named pipe real-time IQ output (`pipe_enable`, `pipe_name`) for monitoring by a compatible IQ client while recording.
- Verbose mode (`verbose = 1`) for gain change event logging.

### Changed
- AGC redesign for improved stability during DX sessions.

---

## [1.2.3]

### Added
- Multi-recording schedule (`schedule_N_*` INI keys) for unattended overnight recordings. Up to 32 entries. Hardware stays initialised between entries.
- Dual-channel RSPduo support (`dual_channel = 1`) with independent Tuner A and B frequency, gain, AGC, and correction settings.
- Scheduled UTC start time (`start_time_utc`) with 12-hour window logic for overnight use.

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
- Linrad raw format recording (`output_format = linrad`).
- Single-tuner support for RSPdx, RSPduo, RSP1A, RSP2.
- INI file configuration.
- Real-time status bar with dBFS signal metering, overflow count, ring buffer utilisation, and disk free.
- Ring buffer with configurable size (`ring_buffer_sec`).
- Software decimation (`decimation`).
- Antenna selection (`antenna`), Bias-T (`bias_t`), Hi-Z notch (`hiz_notch`).
- Recording verification on completion.
