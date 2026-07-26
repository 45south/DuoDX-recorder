/*
 * duodx.c
 *
 * SDRplay to Linrad raw format recorder for Windows 11
 * Supports single-tuner RSP devices and dual-channel RSPduo
 *
 * Build (MinGW-w64):
 *   gcc -O2 -o duodx.exe duodx.c -I"C:\Program Files\SDRplay\API\inc" \
 *       -L"C:\Program Files\SDRplay\API\x64" -lsdrplay_api -lwinmm -mthreads
 *
 * Build (MSVC):
 *   cl /O2 duodx.c /I"C:\Program Files\SDRplay\API\inc" \
 *      /link "C:\Program Files\SDRplay\API\x64\sdrplay_api.lib" winmm.lib
 *
 * Linrad raw format (16-bit):
 *   - Fixed 1024-byte header block (text fields, zero-padded)
 *   - Followed by interleaved signed 16-bit I/Q samples
 *   - Single channel:  I0 Q0 I1 Q1 ...
 *   - Dual channel:    IA0 QA0 IB0 QB0 IA1 QA1 IB1 QB1 ...
 */

/* WIN32_LEAN_AND_MEAN and __STDC_FORMAT_MACROS are defined by sdrplay_api.h */
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdint.h>
#include <stddef.h>
#include <inttypes.h>
#include <time.h>
#include <math.h>
#include <signal.h>
#include <commctrl.h>
#include <richedit.h>
#include <mmsystem.h>   /* waveOut* - live monitor audio output (WIN32_LEAN_AND_MEAN
                         * excludes this from windows.h, so pull it in explicitly).
                         * Requires -lwinmm at link time (already in the build line). */
#include <dbghelp.h>    /* MINIDUMP_* types for the crash handler below - loaded
                         * dynamically at crash time, so no -ldbghelp needed. */
#include <malloc.h>     /* _resetstkoflw() - reclaims stack space on a stack
                         * overflow so the crash handler itself has room to run. */

#include "sdrplay_api.h"
/* RSP1B was added in API 3.14; older headers may not define this ID. */
#ifndef SDRPLAY_RSP1B_ID
#  define SDRPLAY_RSP1B_ID  6
#endif

/* =========================================================================
 * Constants and configuration defaults
 * ========================================================================= */

#define VERSION                 "3.0.1"

#define CONFIG_FILE             "duodx.ini"
#define DEFAULT_OUTPUT_FILE     "recording.raw"
#define DEFAULT_FREQUENCY_HZ    1000000.0       /* 1 MHz */
#define DEFAULT_SAMPLE_RATE_HZ  2000000.0       /* 2 Msps */
#define DEFAULT_GAIN_REDUCTION  40              /* dB */
#define DEFAULT_LNA_STATE       0
#define DEFAULT_IF_KHZ          0               /* Zero-IF */
#define DEFAULT_BW_KHZ          600             /* 600 kHz IF bandwidth */
#define DEFAULT_DURATION_SEC    0               /* 0 = unlimited */
#define DEFAULT_AGC_ENABLE      0
#define DEFAULT_DC_CORRECT      1
#define DEFAULT_IQ_CORRECT      1
#define DEFAULT_NOTCH_RF        0
#define DEFAULT_NOTCH_DAB       0

/*
 * Ring buffer:  2 seconds at 2 Msps dual-channel 16-bit = 32 MB
 * We allocate for worst case: 10 Msps dual-channel = 160 MB
 * Default: 4 seconds worth at configured sample rate, minimum 64 MB
 */
#define RING_BUFFER_SECONDS     4
#define RING_BUFFER_MIN_BYTES   (64 * 1024 * 1024)
#define RING_BUFFER_MAX_BYTES   (256 * 1024 * 1024)

/* Monitor update interval in milliseconds */
#define DEFAULT_MONITOR_INTERVAL_MS  500

/* Maximum path length */
#define MAX_PATH_LEN            512
#define MAX_SCHEDULE_ENTRIES    32   /* max entries in [schedule] section */

/* One entry in a multi-recording schedule */
typedef struct {
    char   start_time[16];     /* HH:MM:SS UTC, or "" = immediate after previous */
    int    duration_sec;       /* 0 = unlimited (run until next entry start) */
    double frequency_hz;       /* Centre frequency, 0.0 = keep current */
    double freq_b_hz;          /* Tuner B frequency, 0.0 = keep current */
    char   output_file[MAX_PATH_LEN]; /* "" = auto-generate */
    char   antenna[8];         /* "" = keep current, else "A"/"B"/"C"/"Hi-Z" */
} ScheduleEntry;

/* =========================================================================
 * Ring buffer - lock-free single-producer / single-consumer
 *
 * The SDRplay callback is the producer; the writer thread is the consumer.
 * We use a power-of-2 size and atomic read/write indices.
 *
 * No mutexes, no malloc, no blocking in the callback path.
 * ========================================================================= */
typedef struct {
    uint8_t  *buf;              /* Allocated buffer (power of 2 size) */
    SIZE_T    size;             /* Size in bytes (power of 2) */
    SIZE_T    mask;             /* size - 1, for fast modulo */
    volatile LONG64 write_idx;  /* Written by callback thread */
    volatile LONG64 read_idx;   /* Written by writer thread */
    volatile LONG   overflow;   /* Incremented on overflow (callback) */
    volatile LONG64 bytes_dropped; /* Exact byte count lost to overflow;
                                    * writer thread reads this to insert
                                    * compensating zero-fill frames. */
} RingBuffer;

/* =========================================================================
 * Output format selector
 * ========================================================================= */
typedef enum {
    FORMAT_LINRAD    = 0,  /* 41-byte binary header + raw 16-bit I/Q        */
    FORMAT_WAVVIEWDX = 1,  /* No header - pure 16-bit I/Q, WavViewDX naming */
    FORMAT_SDRUNO    = 2,  /* RIFF/WAV with fmt+auxi chunks, 216-byte header.
                               SDRuno itself doesn't support RF64, so this
                               format always splits at ~4 GiB - no choice
                               offered in Settings.                        */
    FORMAT_SDRCONNECT= 3,  /* RIFF/WAV with JUNK padding, 80-byte header    */
    FORMAT_WINRAD    = 4   /* Same 216-byte fmt+auxi header/data layout as
                               FORMAT_SDRUNO (reuses its writer functions),
                               just a different filename prefix - for
                               software that does support RF64 for files
                               over 4 GiB, unlike SDRuno itself.            */
} OutputFormat;

/* Large-file handling for WAV-based output formats where it's actually
 * offered as a choice (SDR Connect and Winrad) - not meaningful for
 * Linrad or WavViewDX, which have no 4 GiB header limit to work around,
 * and not offered for SDRuno, which always splits (see FORMAT_SDRUNO
 * above - SDRuno itself doesn't support RF64 playback). A plain WAV
 * data_size field is 32-bit and cannot describe more than ~4 GiB of audio
 * data; these two modes are the alternatives:                            */
#define LARGE_FILE_SPLIT 0   /* auto-split into numbered parts at ~4 GiB  */
#define LARGE_FILE_RF64  1   /* single file, RF64 extended header        */

/* =========================================================================
 * Configuration - populated from INI file then overridden by CLI args
 * ========================================================================= */
typedef struct {
    char     recording_path[MAX_PATH_LEN]; /* directory for recordings */
    char     output_file[MAX_PATH_LEN];
    double   frequency_hz;
    double   sample_rate_hz;
    int      gain_reduction;
    int      lna_state;
    int      if_khz;
    int      bw_khz;
    int      duration_sec;
    int      agc_enable;
    int      agc_setpoint_dbfs;  /* SDRplay API default: -60. Where the AGC
                                     targets the signal level.            */
    int      agc_attack_ms;      /* SDRplay API default: 0 (no smoothing -
                                     reacts instantly to every fluctuation,
                                     which reads as continuous hunting on
                                     a signal sitting near the AGC's
                                     decision point). DuoDX default: 50.   */
    int      agc_decay_ms;       /* Same reasoning, decay side. DuoDX
                                     default: 100 (slower than attack, so
                                     a momentary dip doesn't immediately
                                     pull gain back up and overshoot).     */
    int      dc_correct;
    int      iq_correct;
    int      notch_rf;
    int      notch_dab;
    int      dual_channel;      /* 1 = force RSPduo dual mode */
    /* Which physical RSPduo tuner a single-tuner (non-dual) session uses -
     * "A" (Tuner 1, default) or "B" (Tuner 2). Irrelevant for dual_channel
     * sessions (both are active there) and for every other device, which
     * only has one tuner in the first place. Tuner 2 has no Hi-Z port and
     * is the only one with Bias-T - see apply_antenna_and_biast().       */
    char     rspduo_single_tuner[4];
    int      monitor_bar_visible; /* 1 (default) = show the live monitor bar
                                    * along the bottom of the window; 0 =
                                    * hide it entirely and let the Record/
                                    * AGC/Schedule/Settings buttons sit flush
                                    * with the bottom edge instead. Purely a
                                    * layout preference - doesn't disable
                                    * the underlying feature, just its
                                    * visibility. Only takes effect while
                                    * idle - see g_monitor_bar_visible_eff. */
    int      monitor_hpf_enable; /* 1 = apply an adjustable low-cut filter
                                   * to the live monitor's audio, on top of
                                   * whatever each mode already does (AM's
                                   * DC blocker stays separate and always
                                   * on - this is an additional, optional
                                   * stage, not a replacement for it).
                                   * Listening only - never affects the
                                   * recorded file. Default off.         */
    double   monitor_hpf_hz;     /* Low-cut cutoff frequency in Hz when
                                   * monitor_hpf_enable=1. Default 100.0. */
    int      monitor_volume_percent; /* Last-used Vol slider position,
                                   * 0-100, remembered across restarts.
                                   * Default 15.                          */
    int      monitor_smeter_mode; /* Narrowband S-meter: 0=peak (default,
                                    * more responsive), 1=averaged (steadier,
                                    * more representative of typical strength
                                    * on a fluctuating signal).             */
    double   monitor_smeter_cal_offset; /* dB added to the gain-compensated
                                    * dBFS reading to approximate dBm.
                                    * Defaults to 0 - deliberately not a
                                    * guessed constant, since the exact ADC
                                    * full-scale-to-dBm reference isn't
                                    * published and needs calibrating
                                    * against a known signal source (or
                                    * another calibrated tool, e.g. SDRuno's
                                    * own dBm readout) to be meaningful.     */
    double   freq_b_hz;         /* RSPduo tuner B frequency (dual mode) */
    int      ring_buffer_sec;   /* Override ring buffer duration */
    int      monitor_interval_ms; /* Status bar update interval (ms) */
    char     log_file[MAX_PATH_LEN];
    int      log_auto_save;   /* 1 = write a timestamped session log file
                                  automatically each session, no manual
                                  path needed - independent of log_file
                                  above, which is a fixed, always-append
                                  path for anyone who wants that instead. */
    int      verbose;
    OutputFormat output_format;  /* FORMAT_LINRAD or FORMAT_WAVVIEWDX */
    int      large_file_mode;  /* LARGE_FILE_SPLIT or LARGE_FILE_RF64 -
                                     consulted when output_format is
                                     FORMAT_WINRAD or FORMAT_SDRCONNECT.
                                     Ignored for FORMAT_SDRUNO, which always
                                     splits regardless of this value (it
                                     can't play back RF64), and irrelevant
                                     for Linrad/WavViewDX.                 */

    /* ── Observer location, for the sunrise/sunset tile ─────────────────── */
    double   latitude;          /* Decimal degrees, +north / -south, 0.0 default */
    double   longitude;         /* Decimal degrees, +east  / -west,  0.0 default */
    int      show_sun_times;    /* Show the SUN tile on the main window if set   */


    /* ── Antenna selection ──────────────────────────────────────────────── */
    /* RSPdx/RSPdxR2 : "A", "B", or "C"                                     */
    /* RSP2          : "A" or "B"                                            */
    /* RSPduo Tuner1 : "Hi-Z" or "50ohm"  (Tuner 2 is always 50 ohm)        */
    char     antenna[8];

    /* ── Bias-T ─────────────────────────────────────────────────────────── */
    /* Supported: RSP1A, RSPduo (Tuner 2 only), RSPdx, RSPdxR2              */
    int      bias_t;

    /* ── RSPduo Hi-Z AM notch ────────────────────────────────────────────  */
    /* Only relevant when RSPduo Tuner 1 is using the Hi-Z (AMPORT_1) port  */
    int      hiz_notch;

    /* ── HDR mode (RSPdx / RSPdx R2 only) ───────────────────────────────  */
    int      hdr_enable;        /* 1=enable High Dynamic Range mode         */
    int      hdr_bw_khz;        /* HDR bandwidth: 200, 500, 1200, or 1700   */

    /* ── PPM frequency correction ────────────────────────────────────────  */
    /* Software offset applied to tuning - use when no external reference.   */
    /* Set to 0.0 when a GPSDO is connected (hardware handles it).           */
    double   ppm;

    /* ── Device selection ───────────────────────────────────────────────── */
    /* Leave empty to use first device found. Set to serial number string    */
    /* (as shown at startup) to select a specific device when multiple are   */
    /* connected, e.g. device_serial = 1234567890                            */
    char     device_serial[64];

    /* ── Software decimation ─────────────────────────────────────────────  */
    /* Additional decimation factor on top of hardware IF decimation.        */
    /* Valid values: 1 (none), 2, 4, 8, 16, 32. Output rate = hw_rate / n.  */
    int      decimation;
    double   expected_output_rate_hz; /* computed by validate_config */
    double   expected_usable_bw_hz;   /* computed by validate_config - the
                                        * selected IF filter bandwidth
                                        * (bw_khz), narrower than the full
                                        * recorded span for every combo in
                                        * VALID_COMBOS */


    /* ── Per-tuner B overrides (RSPduo dual mode only) ─────────────────
     * Each defaults to the corresponding Tuner A value if not set.
     * tuner_b_settings_set tracks whether any _b key was explicitly     
     * provided, so we can log clearly which values are in effect.       */
    int      gain_reduction_b;
    int      lna_state_b;
    int      agc_enable_b;
    int      dc_correct_b;
    int      iq_correct_b;
    int      notch_rf_b;
    int      notch_dab_b;
    int      tuner_b_settings_set;

    /* ── Scheduled recording ─────────────────────────────────────────── */
    /* UTC time string HH:MM:SS. Empty string = start immediately.        */
    char     start_time[16];

    /* ── Drive spin-up ───────────────────────────────────────────────── */
    /* Write a small dummy block to wake a spinning disk before recording.
     * Skipped automatically when the output path is on the C: drive.    */
    int      spinup_enable;        /* 1=on (default), 0=off              */
    int      spinup_bytes;         /* bytes to write for spin-up (default 1 MB) */
    int      pipe_enable;          /* 1=create named pipe for real-time monitoring   */
    char     pipe_name[128];       /* pipe name, default \\.\pipe\duodx              */
    int      http_port;            /* 0=disabled, else TCP port for status server    */
    int      http_interval_ms;     /* dashboard auto-refresh interval in ms          */

    /* ── Multi-recording schedule ────────────────────────────────────── */
    ScheduleEntry schedule[MAX_SCHEDULE_ENTRIES];
    int           schedule_count;  /* number of entries parsed */
    int           schedule_only;   /* 1 = skip top-level recording, start from schedule_1 */

    int           use_utc;         /* 1=UTC timestamps (default), 0=local time */
    int           show_clock;      /* 1=show live clock in GUI (default), 0=hide */
    int           meter_style;     /* 0=zone colours (default), 1=graduated blend */
    int           hourly_enable;   /* 1 = enable hourly recording mode */
    int           hourly_window_min; /* recording window centred on the hour (minutes) */
    char          hourly_start[8]; /* window open time HH:MM */
    char          hourly_stop[8];  /* window close time HH:MM */

    /* ── Main window geometry ────────────────────────────────────────── */
    /* Saved on close, restored on next launch. x/y default to
     * CW_USEDEFAULT (let Windows place it) rather than a fixed screen
     * position, since a saved position could easily fall outside a
     * monitor that's since been reconfigured or disconnected.          */
    int           window_x, window_y, window_w, window_h;
    int           window_maximized; /* 1 = start maximized */
    char          color_scheme[8];  /* "navy" (default) or "grey" - see
                                      * apply_color_scheme(). Applied once
                                      * at startup, not switchable live.   */

    /* "schedule" or "hourly" - which of the two mutually-exclusive timer
     * modes the main window's Timer button re-enables when pressed from
     * OFF. Updated whenever either mode is turned on, from any of the
     * three places that can do that (this button, the Settings dialog's
     * Schedule tab, or the mutual-exclusivity enforcement in either of
     * those) - see gui_set_timer_last_mode().                            */
    char          timer_last_mode[12];
} Config;

/* =========================================================================
 * Global state - kept minimal, shared only through well-defined paths
 * ========================================================================= */
typedef struct {
    /* SDRplay handles */
    sdrplay_api_DeviceT          device;
    sdrplay_api_DeviceParamsT   *dev_params;
    sdrplay_api_RxChannelParamsT *ch_a_params;
    sdrplay_api_RxChannelParamsT *ch_b_params;

    /* Ring buffer */
    RingBuffer  ring;

    /* Writer thread */
    HANDLE      writer_thread;
    HANDLE      writer_ready_event;
    volatile int writer_running;
    volatile int writer_error;       /* Set by writer thread on disk error */

    /* File output */
    HANDLE      out_file;
    HANDLE      pipe_handle;       /* named pipe for real-time monitoring (INVALID_HANDLE_VALUE if unused) */

    /* Dual-tuner, different CFs: written as two separate single-channel
     * files instead of one interleaved dual file (a Linrad-format dual
     * file only has one centre-frequency field, so it cannot correctly
     * represent two tuners on two different frequencies). */
    int         dual_separate_files;
    HANDLE      out_file_b;
    char        output_file_b[MAX_PATH_LEN];

    /* RSPduo Master/Slave mode: Tuner A runs as Master in this process,
     * Tuner B runs as Slave in a second, hidden child process (this is
     * the only combination that reliably applies an independent RF
     * frequency to Tuner B on this hardware/driver - see notes at the
     * launch site). */
    int         master_slave_active;
    HANDLE      slave_process;
    HANDLE      slave_job;
    HANDLE      slave_stop_event;
    DWORD       slave_pid;
    HANDLE      slave_monitor_thread;
    volatile int slave_monitor_running;

    /* Config */
    Config      cfg;

    /* Statistics (updated atomically by callback, read by monitor) */
    volatile LONG64 samples_received;    /* Total sample frames received */
    volatile LONG64 samples_written;     /* Total sample frames written */
    volatile LONG64 segment_samples_written; /* Frames written to the CURRENT
                                     physical file - equals samples_written
                                     unless split mode has rolled over to a
                                     new part file, in which case it resets
                                     to 0 at each rollover.                */
    int         output_part_number;     /* 1 = first file, 2+ = how many
                                            times split mode has rolled
                                            over so far this recording -
                                            purely informational (each
                                            rolled-over file gets its own
                                            fresh timestamped name, see
                                            check_split_rollover()), used
                                            only for the log messages.     */
    volatile LONG64 callback_count;      /* Number of callback invocations */
    volatile LONG   overflows;           /* Ring buffer overflow count */
    volatile LONG   teardown_forced;     /* 1 = sdrplay_api_Uninit gave up while
                                          * still StopPending; every streaming
                                          * callback checks this first and
                                          * returns immediately without
                                          * touching the ring, since the
                                          * ring/device may be freed shortly
                                          * after this is set - see the
                                          * "continuing anyway" StopPending
                                          * timeout handling.                */
    volatile LONG64 zero_frames_written;  /* Frames written as zero-fill gap compensation */
    volatile float  peak_dbfs;            /* Peak signal level dBFS, updated each callback */
    volatile float  peak_dbfs_b;          /* Peak dBFS for Tuner B (dual mode only)        */
    volatile int    overload_tuner_a;     /* 1=Tuner A overload active   */
    volatile int    overload_tuner_b;     /* 1=Tuner B overload active   */
    volatile int    stream_running;
    volatile int    listening;   /* 1 = device is streaming (Monitor-only
                                   * start) but no file is open yet. Kept
                                   * separate from stream_running, which
                                   * still means "actively writing a file" -
                                   * see gui_start_listening() and the
                                   * listening branch in recording_worker. */
    volatile int    sample_rate_measured;  /* 1 once actual rate is known */
    double          actual_sample_rate;    /* measured from callbacks */
    LONG64          measure_start_samples; /* snapshot when measurement begins */
    LARGE_INTEGER   measure_start_time;    /* wall clock at measurement start */

    /* Dual-channel sync */
    int16_t    *dual_merge_buf;          /* Temporary merge buffer */
    SIZE_T      dual_merge_buf_size;
    CRITICAL_SECTION dual_lock;          /* Protects dual merge buffer */

    /* RSPduo channel A pending data */
    int16_t    *pending_a_i;
    int16_t    *pending_a_q;
    int         pending_a_count;
    int         pending_a_valid;

    /* Timing */
    LARGE_INTEGER perf_freq;
    LARGE_INTEGER start_time;

    /* Log file */
    FILE       *log_fp;

    /* Control */
    volatile int stop_requested;
    volatile int adhoc_recording;  /* 1 = Record Now ad-hoc, resume wait after */
    double   adhoc_frequency_hz;   /* top-level freq saved before schedule promotion */
    double   adhoc_freq_b_hz;
    int      adhoc_duration_sec;
    double   sched1_frequency_hz;  /* schedule_1 freq (after promotion) */
    double   sched1_freq_b_hz;
    int      sched1_duration_sec;

    /* Disk space monitoring (writer thread) */
    volatile int disk_warn_issued;   /* 1 once the low-space warning has fired */
    volatile int disk_stop;           /* 1 when stopped cleanly due to low disk space */
    volatile LONG64 disk_free_mb;     /* last computed free space, shared with HTTP thread */
    volatile LONG64 frozen_file_mb;   /* file size frozen at end of recording, -1 = not set */
    volatile double frozen_elapsed_sec; /* elapsed time frozen at end of all recordings */
    volatile double last_display_elapsed; /* last completed recording length, for GUI - never reset between entries */
    volatile LONG64 last_display_file_mb; /* last completed file size, for GUI - never reset between entries */
    volatile int    session_complete;   /* 1 after all recordings finish */
    char            frozen_json[4096];  /* pre-built JSON served after session ends */
    char            next_start[16];     /* next schedule start time HH:MM:SS or empty */

    /* Antenna actually in effect for the current session, snapshotted at
     * session start (see the infobar pre-population in recording_worker).
     * Deliberately separate from cfg.antenna: Settings can be saved while
     * listening and cfg.antenna updates immediately, but antenna itself
     * is not one of the values pushed live (see gui_apply_live_gain and
     * the comment beside its call in settings_save) - only gain/LNA are.
     * The status line must keep showing this frozen value, not cfg.antenna,
     * until the session actually restarts with the new antenna; otherwise
     * it claims a change took effect on the running receiver when it
     * hasn't. */
    char            live_antenna[8];

    /* Same idea as live_antenna, for the same reason: sample rate / IF /
     * bandwidth is never pushed to an already-running device (it needs a
     * full re-Init), but Settings Save calls validate_config() on every
     * save regardless of session state, which recomputes and overwrites
     * cfg.expected_output_rate_hz immediately. Anything that reads that
     * field to describe or process the CURRENT stream - the live audio
     * monitor's resampling math, its tuning-coverage clamp, and the
     * SR/Coverage status text - must use this frozen value instead while
     * a session is active, or a sample-rate change saved mid-session
     * makes the monitor think samples are arriving faster or slower than
     * they really are. That mismatch is audible as chuffing, the same
     * failure mode the decimation/expected_output_rate_hz fix addressed
     * earlier - this is the same bug, reachable via a different field. */
    double          live_expected_output_rate_hz;
    double          live_expected_usable_bw_hz;    /* frozen alongside
                                                      * live_expected_output_rate_hz,
                                                      * same reasoning */
} AppState;

static AppState g_state;
static volatile int g_running = 1;
static volatile int g_recording = 0; /* 1 during recording: suppress console LOG */
static volatile int g_http_running = 1; /* controls HTTP accept loop lifetime */
static volatile int g_http_ready   = 0; /* set by HTTP thread when listening  */

/* =========================================================================
 * Logging
 * ========================================================================= */
/* =========================================================================
 * GUI front-end globals and helpers
 *
 * This block replaces the console front-end (the original log_write,
 * keyboard thread, console monitor status line, and main()).
 *
 * The recording engine below is unchanged. All log output is routed to a
 * read-only multiline edit control; live statistics are routed to a set of
 * static labels updated by a GUI monitor thread.
 * ========================================================================= */

/* Control IDs */
#define IDC_LOG          1001
#define IDC_BTN_TOGGLE   1002
#define IDC_BTN_AGC      1004
#define IDC_BTN_SCHED_TOGGLE 1006

/* Live monitor bar control IDs */
#define IDC_BTN_MONITOR      1020
#define IDC_BTN_FREQ_LOCK    1177
#define IDC_EDIT_MON_FREQ    1021
#define IDC_COMBO_MON_MODE   1022
#define IDC_BW_DIGITS        1023
#define IDC_NOTCH_DIGITS     1024
#define IDC_BTN_NOTCH_ENABLE 1025
#define IDC_FREQ_DIGITS      1026
#define IDC_SLIDER_MON_VOL   1032
#define IDC_BTN_HPF_ENABLE   1033
#define IDC_SLIDER_HPF_HZ    1034
#define IDC_SMETER           1035
#define FREQ_DIGITS_COUNT    9   /* up to 999,999,999 Hz - covers MW/HF/VHF */
#define BW_DIGITS_COUNT      4   /* DDD.D kHz - covers 000.1 to 500.0        */
#define NOTCH_DIGITS_COUNT   4   /* DD.DD kHz magnitude, sign shown separately */

#define IDC_BTN_SETTINGS      1050

/* Settings dialog controls */
#define IDC_SET_FREQ_A        1100
#define IDC_SET_RATECOMBO     1101
#define IDC_SET_RANGE_START   1200
#define IDC_SET_RANGE_END     1201
#define IDC_BTN_RANGE_CALC    1202
#define IDC_SET_GR_A          1102
#define IDC_SET_LNA_A         1103
#define IDC_SET_AGC           1104
#define IDC_SET_DUAL          1105
#define IDC_SET_FREQ_B        1106
#define IDC_SET_GR_B          1107
#define IDC_SET_GR_B_SAME     1108
#define IDC_SET_LNA_B         1109
#define IDC_SET_LNA_B_SAME    1110
#define IDC_SET_DURATION      1111
#define IDC_SET_ANTENNA       1112
#define IDC_SET_FORMAT        1113
#define IDC_SET_PATH          1114
#define IDC_SET_BROWSE        1115
#define IDC_SET_SAVE          1116
#define IDC_SET_CANCEL        1117
#define IDC_SET_HDR           1119
#define IDC_SET_MON_VISIBLE   1123
#define IDC_SET_DECIM         1124
#define IDC_SET_TAB0          1149
#define IDC_SET_TAB1          1150
#define IDC_SET_TAB2          1151
#define IDC_SET_TAB3          1152
#define IDC_SET_TAB4          1153
#define IDC_SET_TAB5          1178
#define IDC_SET_TAB6          1203
#define IDC_SET_LARGEMODE 1204
#define IDC_SET_LATITUDE  1205
#define IDC_SET_LONGITUDE 1206
#define IDC_SET_SHOW_SUN  1207
#define IDC_SET_COLOR_SCHEME  1179
#define IDC_SET_VERBOSE       1180
#define IDC_SET_LOG_AUTOSAVE  1181
#define IDC_SET_SCHED_ONLY    1154
#define IDC_SET_SCHED_PREV    1156
#define IDC_SET_SCHED_NEXT    1157
#define IDC_SET_SCHED_ADD     1158
#define IDC_SET_SCHED_DEL     1159
#define IDC_SET_SCHED_START   1160
#define IDC_SET_SCHED_DURATION 1161
#define IDC_SET_SCHED_FREQ    1162
#define IDC_SET_SCHED_FREQ_B  1163
#define IDC_SET_SCHED_ANTENNA 1164
#define IDC_SET_SCHED_OUTFILE 1165
#define IDC_SET_HOURLY_EN     1166
#define IDC_SET_HOURLY_WIN    1167
#define IDC_SET_HOURLY_START  1168
#define IDC_SET_HOURLY_STOP   1169
#define IDC_SET_SINGLE_TUNER  1170
#define IDC_SET_DUALT1_FREQ   1171
#define IDC_SET_DUALT1_GR     1172
#define IDC_SET_DUALT1_LNA    1173
#define IDC_SET_DUALT1_ANTENNA 1174
#define IDC_SET_TUNER1_EN     1175
#define IDC_SET_TUNER2_EN     1176
#define IDC_SET_PPM           1126
#define IDC_SET_DC            1127
#define IDC_SET_IQ            1128
#define IDC_SET_NOTCH_RF      1129
#define IDC_SET_NOTCH_DAB     1130
#define IDC_SET_BIAST         1131
#define IDC_SET_HIZ           1132
#define IDC_SET_HDR_BW        1133
#define IDC_SET_B_SAME_CORR   1134
#define IDC_SET_AGC_B         1141
#define IDC_SET_DC_B          1142
#define IDC_SET_IQ_B          1143
#define IDC_SET_NOTCH_RF_B    1144
#define IDC_SET_NOTCH_DAB_B   1145
#define IDC_SET_RING_SEC      1135
#define IDC_SET_SPINUP_EN     1136
#define IDC_SET_SPINUP_BYTES  1137
#define IDC_SET_MON_INTERVAL  1138
#define IDC_SET_SMETER_MODE   1139
#define IDC_SET_SMETER_CAL    1140
#define IDC_SET_USE_UTC       1139
#define IDC_SET_SHOW_CLOCK    1140
#define IDC_SET_METER_STYLE   1141
#define IDC_SET_HTTP_PORT     1142
#define IDC_SET_HTTP_INTERVAL 1143
#define IDC_SET_PIPE_EN       1144
#define IDC_SET_PIPE_NAME     1145

/* Private window messages */
#define WM_APP_LOG       (WM_APP + 1)   /* lParam = char* (heap), append to log  */
#define WM_APP_DONE      (WM_APP + 3)   /* worker thread finished                */
#define WM_APP_RECORDING_STARTED (WM_APP + 4)  /* listening -> recording, seamless
                                                 * transition inside recording_worker
                                                 * - the GUI thread owns the Record/
                                                 * Stop button label and must be the
                                                 * one to update it, not the worker. */
#define WM_APP_DOWNGRADED_TO_LISTENING (WM_APP + 5)  /* the reverse transition -
                                                 * a scheduled/hourly wait was
                                                 * cancelled (Timer turned off)
                                                 * but the device stayed open for
                                                 * Monitor; same cross-thread
                                                 * button-ownership reason as
                                                 * WM_APP_RECORDING_STARTED.       */
#define WM_APP_TUNER_SWITCH_RESTART (WM_APP + 6)  /* right-click tuner switch
                                                 * in plain single-tuner RSPduo
                                                 * mode - the old session has
                                                 * now fully stopped (device
                                                 * released) on a helper
                                                 * thread; restart on the new
                                                 * tuner here, on the GUI
                                                 * thread, since starting a
                                                 * session must happen there. */
#define WM_APP_RESTART_LISTENING (WM_APP + 7)  /* Timer turned back on while a
                                                 * session had already downgraded
                                                 * to the generic no-schedule
                                                 * listening loop (see
                                                 * IDC_BTN_SCHED_TOGGLE) - the old
                                                 * worker's WM_APP_DONE is already
                                                 * queued ahead of this one, so by
                                                 * the time this is processed the
                                                 * old session's cleanup (handle,
                                                 * UI state) has already run and
                                                 * it's safe to start fresh here. */

/* 1-second timer drives the live clock display. */
#define ID_TIMER_CLOCK   1

/* =========================================================================
 * Live IQ monitor - listen to a demodulated signal while recording.
 *
 * Design: the SDRplay callback (stream_callback_single / dual_a / dual_b)
 * pushes the selected tuner's raw I/Q into a small, separate, lossy ring
 * buffer (monitor_feed()) - a cheap memcpy, no locks, same "no malloc/no
 * blocking" contract as the main ring. A dedicated monitor thread drains
 * that ring, does NCO mix -> 2-stage decimate -> IF notch -> selective
 * filter -> demod -> resample -> waveOut, entirely independent of the
 * writer thread that streams the IQ file to disk. If the monitor thread
 * ever falls behind, its ring just drops samples (an audio glitch) - it
 * can NEVER stall or slow the recording path.
 * ========================================================================= */
typedef enum {
    MON_MODE_AM6 = 0,
    MON_MODE_AM4,
    MON_MODE_AM24,
    MON_MODE_FMN,
    MON_MODE_FMW,
    MON_MODE_LSB,
    MON_MODE_USB,
    MON_MODE_CW,
    MON_MODE_COUNT
} MonMode;

typedef struct { const char *name; double bw_khz_default; } MonModeInfo;
static const MonModeInfo MON_MODE_INFO[MON_MODE_COUNT] = {
    { "AM 6kHz",   6.0 },
    { "AM 4kHz",   4.0 },
    { "AM 2.4kHz", 2.4 },
    { "FM-N",     12.0 },
    { "FM-W",    180.0 },
    { "LSB",       2.4 },
    { "USB",       2.4 },
    { "CW",        0.5 },
};

/* Order the mode dropdown is displayed in - AM variants, then SSB/CW,
 * then FM-N/FM-W last, rather than the MonMode enum's declaration order
 * (which groups FM right after the AM entries). Every other switch,
 * array index, and range check on MonMode elsewhere in the code is
 * untouched by this - only the two places that talk to the combo box
 * (population, and reading back the selection) go through this table. */
static const MonMode MON_MODE_DISPLAY_ORDER[MON_MODE_COUNT] = {
    MON_MODE_AM6, MON_MODE_AM4, MON_MODE_AM24,
    MON_MODE_LSB, MON_MODE_USB, MON_MODE_CW,
    MON_MODE_FMN, MON_MODE_FMW
};

#define MON_PI                 3.14159265358979323846
#define MON_RING_BYTES         (2 * 1024 * 1024)   /* ~0.5s @ 2Msps, lossy */
#define MON_WORK_RATE_NARROW   32000.0             /* complex rate: AM/SSB/CW/FM-N */
#define MON_WORK_RATE_WIDE     250000.0            /* complex rate: FM-W           */
#define CARRIER_WINDOW_SAMPLES_MIN   32000  /* 1s - strong signals (>= S9),
                                             * tested down to ~0.3 Hz spread
                                             * between windows on a strong
                                             * local station                */
#define CARRIER_WINDOW_SAMPLES_MAX  384000  /* 12s - weak DX-level signals.
                                             * A DX recorder's actual use
                                             * case is weak, distant
                                             * catches, not just strong
                                             * local stations - a fixed 1s
                                             * window that's fine for the
                                             * latter leaves far too little
                                             * signal energy to average
                                             * down the noise on the former,
                                             * observed as visibly wider
                                             * window-to-window scatter
                                             * (~0.1-1.5 Hz on an S9-ish
                                             * signal, vs ~0.3 Hz on a
                                             * strong local one) even while
                                             * still technically "locked".
                                             * carrier_window_samples_for_
                                             * signal() below scales
                                             * between these two based on
                                             * how far below S9 the current
                                             * reading is, using the same
                                             * band-aware S9 reference the
                                             * S-meter already established -
                                             * strong signals stay fast,
                                             * weak ones trade speed for
                                             * the averaging they actually
                                             * need for a reliable reading. */
#define CARRIER_LPF_CUTOFF_HZ  100.0  /* one-pole narrowband filter applied
                                       * to sel specifically for carrier
                                       * measurement, before the phase-
                                       * difference calculation - see the
                                       * field comment on carrier_lpf_state.
                                       * Was 8 Hz - too tight, and this
                                       * turned out to be a real bug, not
                                       * just a tradeoff: it's a lowpass
                                       * fixed at 0 Hz (the dial's own
                                       * position), so a genuine dial-
                                       * tuning error of ~100 Hz (not an
                                       * edge case - confirmed as a real
                                       * scenario) got attenuated by
                                       * roughly 22 dB by the filter
                                       * itself, either extinguishing the
                                       * carrier's own contribution to the
                                       * measurement or biasing it near
                                       * the filter's rolloff - explaining
                                       * reports of the readout either not
                                       * appearing at all, or appearing
                                       * tens of Hz wrong, specifically
                                       * when the dial wasn't precisely on
                                       * frequency. 100 Hz still delivers
                                       * a large (~18 dB / 60x bandwidth)
                                       * SNR improvement over the ~6 kHz
                                       * channel bandwidth's worth of
                                       * program audio and in-band noise
                                       * the raw (unfiltered) approach was
                                       * exposed to - none of that energy
                                       * has anything to do with the
                                       * carrier's own frequency.          */
#define CARRIER_LOCK_TOLERANCE_HZ  5.0     /* max disagreement between two
                                            * consecutive raw windows to
                                            * still count as "the same
                                            * carrier" - a live station's
                                            * own window-to-window spread
                                            * was observed at ~0.3 Hz, so
                                            * this has a lot of headroom
                                            * over normal measurement noise
                                            * while still rejecting the much
                                            * larger, essentially random
                                            * disagreement pure noise gives */
#define CARRIER_LOCK_MIN_AGREE     2       /* consecutive agreeing gaps
                                            * required (3 total windows)
                                            * before showing the readout    */
#define CARRIER_SETTLE_TOLERANCE_HZ 0.05   /* max change in the PUBLISHED
                                            * (smoothed) value between
                                            * windows to still count as
                                            * "settled" - half the display's
                                            * own resolution (4 decimal
                                            * places in kHz = 0.1 Hz), so a
                                            * change too small to even show
                                            * up on screen doesn't reset
                                            * the settle count             */
#define CARRIER_SETTLE_MIN_COUNT   5       /* consecutive stable windows
                                            * required before showing
                                            * "(lock)" - separate from,
                                            * and additional to,
                                            * CARRIER_LOCK_MIN_AGREE: that
                                            * governs whether the raw
                                            * measurement is trustworthy
                                            * enough to show at all, this
                                            * governs whether the smoothed
                                            * value has actually stopped
                                            * moving yet.                  */
#define CARRIER_AGC_GUARD_SAMPLES  96000   /* 3s at MON_WORK_RATE_NARROW.
                                            * agc_gain (see its own field
                                            * comment) starts at a fixed
                                            * 3.0 regardless of actual
                                            * signal strength, with a slow-
                                            * release time constant of
                                            * roughly 0.8s - if the initial
                                            * guess is badly wrong for a
                                            * given station, the signal can
                                            * ride into the soft-knee
                                            * limiter while gain is still
                                            * hunting for the right point,
                                            * and a limiter is a nonlinear
                                            * operation that can distort
                                            * phase, not just amplitude.
                                            * Observed as raw carrier
                                            * measurements decaying smoothly
                                            * from very wrong toward correct
                                            * over several seconds after
                                            * retuning - not noise, which
                                            * would scatter randomly around
                                            * the truth from the start.
                                            * Simplest fix without touching
                                            * AGC's own tuning (which was
                                            * set for the primary listening
                                            * experience, not this) is to
                                            * just not trust phase data
                                            * until it's had time to settle. */
#define CARRIER_AGC_STABILITY_FRAC  1.0    /* if agc_gain moves by more than
                                            * this fraction of its value
                                            * during a single window, that
                                            * window's phase data gets
                                            * discarded. Was 0.15, loosened
                                            * substantially: with the
                                            * narrowband tracking filter and
                                            * AGC envelope follower both now
                                            * in place, real data showed raw
                                            * measurements staying identical
                                            * to within 0.002 Hz regardless
                                            * of whether this check passed
                                            * or failed - it had become a
                                            * major source of slow lock
                                            * (discarding good data, not bad)
                                            * rather than genuine protection.
                                            * Left in place as a loose
                                            * backstop against truly extreme
                                            * excursions rather than removed
                                            * outright.                     */
#define CARRIER_FADE_DROP_DB       8.0     /* if a window's own dbm is this
                                            * far below the tracked
                                            * baseline, it's treated as a
                                            * fade in progress and
                                            * discarded outright, same as
                                            * an AGC-unstable window -
                                            * confirmed with real data: a
                                            * genuine ~20 dB fade lasting
                                            * over a minute produced
                                            * consecutive windows that
                                            * agreed closely with EACH
                                            * OTHER while both were wrong
                                            * relative to the true value,
                                            * which the agreement check
                                            * alone can't catch (it can
                                            * only tell "windows differ",
                                            * not "windows agree but are
                                            * both wrong"). 8 dB is well
                                            * above ordinary window-to-
                                            * window dbm jitter on a
                                            * stable signal (~2-5 dB
                                            * observed) but well below a
                                            * genuine fade (~20 dB
                                            * observed), so it shouldn't
                                            * false-trigger on normal
                                            * variation.                   */
#define MON_AUDIO_RATE_HZ      48000
#define MON_AUDIO_BUF_SAMPLES  1024
#define MON_AUDIO_NUM_BUFS     6
#define MON_CW_PITCH_HZ        700.0
#define MON_DECIM_TAPS          63    /* stage1/stage2 anti-alias decimator taps */
#define MON_SEL_TAPS           127    /* selective (bandpass/lowpass) filter taps */
#define MON_AGC_TARGET       12000.0f /* feedback-loop AGC target level. Used to be
                                        * scaled by a user-adjustable monitor_gain
                                        * (removed - see monitor_apply_volume_from_
                                        * slider): the AGC's own adaptive response
                                        * meant changing this target rarely produced
                                        * an audible difference, since the loop just
                                        * converges toward roughly the same output
                                        * level regardless. The waveOutSetVolume
                                        * slider (a direct, always-effective, linear
                                        * control) replaced it entirely. This value
                                        * matches the old default (monitor_gain=1.0),
                                        * so baseline AGC behaviour is unchanged.   */
#define MON_AGC_ENV_ATTACK      0.05f  /* envelope follower attack - fast
                                        * enough to still catch a genuinely
                                        * loud passage within a couple of
                                        * milliseconds (protecting against
                                        * clipping, the original fast-attack
                                        * design's actual intent), but no
                                        * longer instantaneous, so a single
                                        * sample's peak doesn't directly
                                        * yank the gain calculation around. */
#define MON_AGC_ENV_DECAY       0.005f /* envelope follower decay - slower
                                        * than attack (standard peak-
                                        * detector shape), fast enough to
                                        * track genuine level changes over
                                        * tens of milliseconds, slow enough
                                        * that individual quiet moments
                                        * between syllables don't register
                                        * as a real drop in level.          */
#define MON_FM_N_MAXDEV_HZ    5000.0
#define MON_FM_W_MAXDEV_HZ   75000.0

/* Shared bottom-of-window geometry - used by BOTH paint_window() (for the
 * info strip / scheduling text) and layout_children() (for the actual
 * child controls), so the custom-painted text always lines up with the
 * button row instead of ending up hidden behind the monitor bar. */
#define BOTTOM_MON_BAR_H     28
#define BOTTOM_MON_BAR2_H    28
#define BOTTOM_MON_ROW_GAP    6
#define BOTTOM_MON_GAP       8
#define BOTTOM_BTN_ROW_H     26
#define BOTTOM_BTN_GAP       26

/* ---- Theme colours (deep navy / cyan / white) -------------------------- */
/* Runtime variables (not #define) so the Settings dialog's Miscellaneous
 * tab can pick between two known palettes at startup - see
 * apply_color_scheme() below. Values here are "navy", the original.       */
static COLORREF COL_BG        = RGB(13, 27, 53);     /* window background  - deep navy    */
static COLORREF COL_PANEL     = RGB(20, 40, 74);     /* meter/counter panel - lighter navy*/
static COLORREF COL_PANEL_EDGE = RGB(45, 80, 130);   /* panel border                      */
#define COL_TEXT      RGB(232, 240, 255)  /* primary text - near white         */
#define COL_TEXT_DIM  RGB(150, 175, 210)  /* labels - muted blue-grey          */
#define COL_ACCENT    RGB(80, 200, 255)   /* cyan accent / values              */
#define COL_LED_OFF   RGB(60, 40, 40)     /* recording LED when idle           */
#define COL_LED_ON    RGB(255, 0, 0)      /* recording LED when active         */
#define COL_BAR_BG    RGB(8, 16, 32)      /* meter track background            */
#define COL_SEG_GREEN RGB(40, 220, 90)
#define COL_SEG_AMBER RGB(255, 190, 40)
#define COL_SEG_RED   RGB(255, 60, 50)
/* Sunrise/sunset tile - "navy blue" is brightened well past a literal
 * navy (which would be near-invisible on this dark a panel background)
 * so it actually reads as blue rather than just vanishing.               */
#define COL_SUNRISE   RGB(212, 175, 55)   /* metallic gold */
#define COL_SUNSET    RGB(70, 110, 210)   /* navy blue, brightened for legibility */
#define COL_BTN_FACE  RGB(30, 58, 100)
#define COL_BTN_HOT   RGB(45, 85, 140)
#define COL_BTN_DIS   RGB(22, 38, 62)
#define COL_BTN_START RGB(28, 96, 62)    /* green-ish: other "primary action" buttons, e.g. Settings Save */
#define COL_BTN_STOP  RGB(140, 44, 44)   /* red-ish:   recording toggle="Stop"*/
#define COL_BTN_RECORD RGB(90, 30, 40)   /* muted maroon: recording toggle="Record" (idle/listening) -
                                           * a nod to the usual red record convention, kept dark and
                                           * blue-shifted enough to stay clearly distinct from Stop's
                                           * brighter, warmer red rather than reading as "same button". */

/* Selects between the two background/panel palettes - "navy" (the
 * original, and the default) or "grey" (tuned together: a dark neutral
 * background with panels one subtle step lighter, rather than the more
 * pronounced contrast tried and discarded along the way). Called once at
 * startup (WinMain), before any window or brush is created - everything
 * else (text, accent, meter segments, buttons) is identical between the
 * two, so this is the only thing that needs resolving early.              */
static void apply_color_scheme(const char *scheme)
{
    if (!scheme || strcmp(scheme, "grey") != 0)
        return;   /* anything else (including "navy" or unrecognised) keeps the navy defaults */

    COL_BG         = RGB(32, 33, 36);
    COL_PANEL      = RGB(42, 43, 47);
    COL_PANEL_EDGE = RGB(80, 83, 90);
}

/* GUI globals */
static HWND   g_hwnd        = NULL;
static HWND   g_hLog        = NULL;
static HWND   g_hBtnToggle  = NULL;
static HWND   g_hBtnAgc     = NULL;
static HWND   g_hBtnSchedToggle = NULL;   /* toggles schedule_only live */
/* Periodic button-sync "did this change" tracking for gui_monitor_thread_func
 * and monitor_sync_button_label(). These used to be function-static locals,
 * but static storage persists across separate session threads (each new
 * session launches gui_monitor_thread_func fresh, but the statics keep
 * their value from whatever the last session left them at) - the exact bug
 * that made unconditional InvalidateRect calls seem necessary in the first
 * place. Explicitly reset to -1 (a sentinel matching no real 0/1 state) in
 * gui_start_session()/gui_start_listening(), so the first tick of every
 * new session always resyncs correctly, and every tick after that can
 * safely skip repainting when nothing has actually changed.               */
static int g_last_agc_on        = -1;
static int g_last_agc_enabled   = -1;
static int g_last_sched_on      = -1;
static int g_last_sched_enabled = -1;
static int g_last_has_tuner_b   = -1;
static int g_last_tuner_sel     = -1;
static int g_last_lock_coherent = -1;
/* Separate from g_state.cfg.monitor_bar_visible: that field updates the
 * instant Settings is saved, same as every other setting, but applying
 * a bar hide/show mid-session could strand an active listening session
 * with no way to reach the Monitor button to cancel it. This mirrors
 * cfg.monitor_bar_visible but is only refreshed at idle - see
 * gui_refresh_monitor_bar_visibility().                                */
static int g_monitor_bar_visible_eff = 1;
static char g_last_sched_text[96]   = {0};
static char g_last_infobar_text[128] = {0};
static char g_last_infostrip_text[64] = {0};   /* tracks whichever of
                                                * carrier/infobar/sched last
                                                * actually got drawn in that
                                                * shared slot, so any of the
                                                * three changing triggers a
                                                * repaint - see the paint
                                                * function's identical
                                                * priority order            */
static HWND   g_hBtnSettings = NULL;
static HFONT  g_hFontLog    = NULL;
static HFONT  g_hFontUI     = NULL;   /* labels                              */
static HFONT  g_hFontVal    = NULL;   /* bold values / counters              */
static HFONT  g_hFontBig    = NULL;   /* big 7-seg-ish counters              */
static HFONT  g_hFontCarrier = NULL;  /* carrier readout digits - sized to
                                       * approximate the frequency dial
                                       * (18pt), marginally smaller (16pt),
                                       * same Consolas family for a visual
                                       * match                              */
static HBRUSH g_hbrBg       = NULL;
static HBRUSH g_hbrPanel    = NULL;

static HANDLE g_worker_thread  = NULL;   /* recording_worker thread             */
static HANDLE g_gui_mon_thread = NULL;   /* GUI status monitor thread           */
static volatile int g_worker_active = 0; /* 1 while a recording session runs    */
/* Separate from g_worker_active on purpose. g_worker_active is cleared at
 * several points inside recording_worker() well before the function
 * actually returns - as soon as the GUI monitor thread is signalled to
 * stop, so it can exit promptly before ring/critical-section resources it
 * reads are freed. That is correct for its GUI-facing purpose (button
 * state, painting), but gui_start_session()/gui_start_listening() also
 * used g_worker_active as their "is it safe to start a new session" gate
 * - which meant a new session could call sdrplay_api_Open() while the
 * previous worker thread was still mid-cleanup, still holding the device
 * and the API open, about to call sdrplay_api_ReleaseDevice()/Close()
 * itself. Re-entering the SDRplay API from a second thread while it's
 * being torn down on the first crashed the process (ACCESS_VIOLATION
 * inside sdrplay_api.dll, well outside DuoDX's own code - see the crash
 * report this was diagnosed from). g_device_busy is set at the same time
 * as g_worker_active but is only ever cleared once, at the true end of
 * recording_worker() after the API has actually been released - so it's
 * safe to gate new-session starts on, even though g_worker_active alone
 * is not.                                                                */
static volatile int g_device_busy   = 0;
static volatile unsigned char g_last_known_hwVer = 0;
/* Latest currGain reported per tuner, from the sdrplay_api_GainChange
 * event callback - used to compensate the S-meter's dBFS reading for
 * whatever gain is currently applied, since the same RF signal reads
 * completely differently at different GR/LNA settings. */
static volatile double g_curr_gain_a = 0.0, g_curr_gain_b = 0.0;
/* Record/Stop button colour, set synchronously by gui_set_recording_ui()
 * and gui_set_listening_ui() at the exact moment each triggers the
 * button's repaint - not derived at paint time from g_worker_active/
 * g_state.listening, which are worker-thread-owned and can still be
 * mid-transition (e.g. g_worker_active already 1 but g_state.listening
 * not yet set) at the instant gui_set_listening_ui() runs on the GUI
 * thread the moment Monitor is pressed. Deriving colour from those live
 * flags at paint time made the button flash stop-red for one frame
 * before correcting itself to green; this flag can't be stale, because
 * it's set by the same call that requests the repaint.                 */
static volatile int g_toggle_btn_recording = 0;
static volatile int g_agc_toggle_req = 0;/* set by AGC button, serviced in engine*/
/* (LED is static while recording) */

static char   g_config_file[MAX_PATH_LEN] = CONFIG_FILE;

/* Clock display: read from the INI at startup and on each Start, so the GUI
 * can show a live clock even while idle (before the worker loads config). */
static volatile int g_clock_show   = 1;   /* 1 = show live clock              */
static volatile int g_clock_utc    = 1;   /* 1 = UTC, 0 = local               */
static volatile int g_meter_style  = 0;   /* 0 = zone, 1 = graduated          */
static volatile int g_record_now   = 0;
static volatile int g_enter_listening_req = 0;  /* Monitor pressed while
                                                  * fully idle: worker should
                                                  * start the device but wait
                                                  * before opening a file.  */
static volatile int g_cancel_listening    = 0;  /* Monitor pressed again
                                                  * while listening: abort
                                                  * and tear down without
                                                  * ever opening a file.    */
static volatile int g_downgrade_to_listening = 0; /* Timer turned off while
                                                  * waiting for a scheduled/
                                                  * hourly window: cancel
                                                  * just the wait, keep the
                                                  * device open and Monitor
                                                  * running - the user only
                                                  * wanted to stop the auto-
                                                  * record plan, not stop
                                                  * listening entirely.     */
static volatile int g_log_freeze   = 0;  /* suppress auto-scroll after error */
static volatile int g_in_generic_listen_wait = 0; /* 1 while recording_worker
                                                  * is inside the generic
                                                  * "Listening - waiting for
                                                  * Record (no schedule set)"
                                                  * loop specifically - that
                                                  * loop only responds to
                                                  * g_record_now, unlike a
                                                  * genuine hourly/schedule
                                                  * wait, which also notices
                                                  * g_toggle_btn_recording on
                                                  * its own. The Record/Start
                                                  * click handler needs to
                                                  * know which situation it's
                                                  * in - see IDC_BTN_TOGGLE.  */

/* ---- Live UI snapshot ---------------------------------------------------
 * The GUI monitor thread fills this; WM_PAINT reads it. Plain scalars,
 * updated/read atomically enough for display purposes.                     */
typedef struct {
    int    recording;     /* 1 = stream live                                 */
    int    listening;     /* 1 = device streaming, no file open (Monitor-    *
                            * only start) - meters/monitor active, status    *
                            * shown as Idle rather than Recording/Waiting    */
    int    finished;      /* 1 = session complete (verification in log)      */
    int    dual;          /* 1 = dual channel                                */
    double elapsed_sec;
    double file_mb;
    double disk_free_mb;
    float  peak_a;        /* dBFS, -90 = silence                             */
    float  peak_b;
    int    overload_a;
    int    overload_b;
    int    agc_on;        /* 1 = AGC currently enabled on tuner A            */
    int    hdr_on;        /* 1 = HDR mode enabled                            */
    int    coherent;      /* 1 = RSPduo dual-channel, both tuners same freq  */
    int    master_slave;  /* 1 = RSPduo Master/Slave mode (Tuner B via slave process) */
    long   overflows;
    long long dropped;    /* zero-fill frames                                */
    float  ring_pct;      /* ring buffer fill 0..100 (percent, one decimal)  */
    char   state[48];
    char   next[32];
    char   freq[48];
    char   span[96];      /* coverage range, e.g. "150 - 1750 kHz (~150 kHz usable)" */
    char   sched[96];     /* scheduling status line for the bottom bar       */
    char   infobar[128];  /* recording info strip (device, antenna, gain...) */
} UiSnapshot;

static UiSnapshot g_ui;   /* zero-initialised                                */

/* =========================================================================
 * Live monitor - DSP helper types
 * ========================================================================= */
typedef struct { float re, im; } MCplx;

/* Real-coefficient decimating FIR stage (anti-alias low-pass + decimate).
 * Used twice in cascade to bring the native IQ rate down to a working
 * audio-DSP rate cheaply (two modest FIR stages instead of one huge one). */
typedef struct {
    float  coeffs[MON_DECIM_TAPS];
    MCplx  hist[MON_DECIM_TAPS];
    int    head;
    int    decim;
    int    counter;
} MonDecimStage;

/* Selective filter: a real low-pass prototype, optionally frequency-shifted
 * (complex-modulated) to give a genuinely one-sided passband for SSB/CW -
 * this is what lets the notch/selectivity reject a real interferer on the
 * unwanted side of the tuned frequency, not just a phase-cancellation trick. */
typedef struct {
    float  coeffs_re[MON_SEL_TAPS];
    float  coeffs_im[MON_SEL_TAPS];
    MCplx  hist[MON_SEL_TAPS];
    int    head;
} MonSelFilter;

/* Single complex pole/zero IF notch - tunable anywhere within +/-10 kHz of
 * the monitor frequency, independent of which side of the passband it's on. */
typedef struct {
    float  z_re, z_im;   /* zero location on the unit circle                */
    float  r;            /* pole radius - controls notch width              */
    MCplx  x1, y1;
    int    active;
} MonNotch;

typedef struct {
    /* --- settings, written by the GUI thread, read by the monitor thread --- */
    volatile LONG   enabled;      /* 1 = monitor on                          */
    volatile LONG   tuner_sel;    /* 0 = Tuner A, 1 = Tuner B                */
    volatile LONG   freq_locked;  /* 1 = keep Tuner A/B monitor frequencies
                                    * in sync - tuning either one moves both,
                                    * for quick side-by-side antenna/tuner
                                    * comparisons at the same frequency.      */
    volatile LONG   mode;         /* MonMode                                 */
    double          freq_hz;      /* Tuner A dial / suppressed-carrier freq  */
    double          freq_hz_b;    /* Tuner B's own remembered frequency -
                                    * kept separate so switching Monitor A/B
                                    * doesn't lose your place on either one,
                                    * particularly important when the two
                                    * tuners are on completely different
                                    * bands (Master/Slave, e.g. VHF + UHF).
                                    * 0.0 = never set - auto-centres to that
                                    * tuner's configured frequency instead
                                    * of using an invalid remembered value. */
    double          bw_khz;       /* user selectivity bandwidth (kHz)        */
    double          notch_khz;    /* IF notch offset from freq_hz, -10..+10  */
    volatile LONG   notch_enabled; /* explicit ON/OFF, independent of slider */
    CRITICAL_SECTION settings_lock; /* guards freq_hz, freq_hz_b, bw_khz,
                                     * notch_khz                             */

    /* --- IQ tap ring buffer, fed from the SDRplay callback thread --------- */
    RingBuffer ring;
    int        ring_ready;

    /* --- monitor/demod/audio thread --------------------------------------- */
    HANDLE     thread;
    volatile int thread_stop_req;

    /* --- audio output (opened/closed as the monitor is enabled/disabled) -- */
    HWAVEOUT   hwo;
    int        audio_open;
    HANDLE     audio_done_event;
    WAVEHDR    hdr[MON_AUDIO_NUM_BUFS];
    int16_t    audio_buf[MON_AUDIO_NUM_BUFS][MON_AUDIO_BUF_SAMPLES];
    int        cur_buf;
    int        cur_buf_fill;

    /* --- DSP working state, touched only by the monitor thread ------------ */
    double        native_rate_hz;
    double        work_rate_hz;
    MCplx         nco_rot, nco_step;
    MCplx         cw_nco_rot, cw_nco_step;  /* shifts CW's filtered baseband
                                              * (carrier at 0Hz) up to an
                                              * audible pitch after filtering -
                                              * NOT the same as shifting the
                                              * filter itself, which would
                                              * reject an on-frequency carrier. */
    MonDecimStage st1, st2;
    MonNotch      notch;
    MonSelFilter  sel;
    MCplx         fm_prev;       /* previous baseband sample, FM discriminator */
    float         dc_prev_in, dc_prev_out; /* AM DC-blocker state              */
    float         hpf_prev_in, hpf_prev_out; /* general low-cut filter state, all modes */
    int           hpf_primed;
    double        hpf_last_hz;   /* last cutoff the coefficient was computed for,
                                   * so it's only recomputed when it actually
                                   * changes rather than every single sample   */
    float         hpf_coeff;
    float         deemph_state; /* FM-W de-emphasis low-pass state            */
    /* Narrowband S-meter - measured on `sel`, the already NCO-shifted,
     * decimated, notched, and selectivity-filtered complex signal (see
     * the main per-sample loop) - i.e. specifically the tuned station's
     * own signal within its selected bandwidth, not the wideband RF
     * level the main A/B meters show. Peak and averaged accumulators are
     * both always kept up to date regardless of which mode is currently
     * selected, so switching the setting doesn't need to reset/re-prime
     * anything - it just starts reading from the other one immediately. */
    float         smeter_peak_accum;   /* max |sel| since the last publish */
    float         smeter_avg_pow;      /* exponential moving average of |sel|^2 */
    int            smeter_avg_primed;
    volatile float smeter_dbm_pub;        /* published approximate dBm value
                                            * the GUI thread reads for display */
    DWORD          smeter_publish_tick; /* GetTickCount() at last publish -
                                          * throttles to a consistent,
                                          * human-readable real-time rate
                                          * regardless of which bandwidth
                                          * mode (and therefore working
                                          * sample rate) is selected       */
    /* Carrier frequency offset - AM only. `sel` is already NCO-shifted so
     * the dial frequency sits at 0 Hz; a real AM carrier is otherwise
     * phase-continuous (only its amplitude carries the audio), so any
     * residual rotation of `sel` from one sample to the next is the
     * carrier's true frequency minus the dial setting, not programme
     * content - amplitude modulation alone doesn't perturb the carrier's
     * own phase. Tracked as a magnitude-weighted average of the per-
     * sample instantaneous frequency (the weighting is what keeps this
     * stable through envelope nulls near 100% negative modulation,
     * where phase is momentarily undefined but the product magnitude
     * that weights it is naturally near zero too, self-selecting for
     * the reliable segments) over an adaptive-length window (see
     * carrier_window_samples_for_signal()),
     * then smoothed window-to-window with a slow EMA for display
     * stability - see the main per-sample loop for where this runs.    */
    MCplx          carrier_prev_sample;
    int            carrier_prev_valid;
    MCplx          carrier_lpf_state;   /* one-pole narrowband tracking
                                        * filter, applied to sel before
                                        * phase-difference measurement -
                                        * separate from the actual AM
                                        * demod path (audio still uses the
                                        * full 6/4/2.4kHz-filtered sel
                                        * unchanged). The carrier is a
                                        * narrow, concentrated spectral
                                        * line; the rest of the channel
                                        * bandwidth (program audio
                                        * sidebands, in-band noise) is
                                        * pure dead weight for measuring
                                        * ITS frequency specifically, and
                                        * was contaminating every phase-
                                        * difference sample up to now.
                                        * See CARRIER_LPF_CUTOFF_HZ.       */
    double         carrier_phase_accum;
    double         carrier_weight_accum;
    int            carrier_accum_samples;
    int            carrier_window_target;   /* this window's target length -
                                             * see carrier_window_samples_
                                             * for_signal(); set once when
                                             * the window starts, not
                                             * recomputed mid-window        */
    float          carrier_agc_gain_at_start;  /* AGC gain snapshotted when
                                                * this window began - if
                                                * agc_gain has moved a lot
                                                * by the time the window
                                                * completes, that window's
                                                * phase data likely rode
                                                * through active AGC
                                                * hunting (a fading/
                                                * fluctuating signal keeps
                                                * this happening the whole
                                                * session, not just at
                                                * startup) and gets
                                                * discarded rather than
                                                * treated as a trustworthy
                                                * measurement.               */
    float          carrier_dbm_baseline;    /* slow EMA of dbm, updated only
                                             * from windows that already
                                             * passed every other quality
                                             * check - a genuine deep fade
                                             * (confirmed with real data:
                                             * a ~20 dB drop lasting over a
                                             * minute) can still produce
                                             * consecutive windows that
                                             * agree with EACH OTHER while
                                             * both are wrong relative to
                                             * the true value, since the
                                             * agreement check alone can't
                                             * tell "consistently right"
                                             * from "consistently wrong in
                                             * the same way" - a direct
                                             * signal-strength check catches
                                             * what the agreement check
                                             * missed. int, not bool: 0
                                             * means "not established yet". */
    int            carrier_dbm_baseline_valid;
    volatile float carrier_offset_hz_pub;   /* dial-relative, +/- Hz       */
    volatile int   carrier_offset_valid_pub;
    int            carrier_window_count;    /* windows completed since the
                                              * last reset - drives the
                                              * fast-then-settling smoothing
                                              * below */
    int            carrier_agc_guard_remaining;  /* samples left to skip
                                                   * before trusting phase
                                                   * data - see
                                                   * CARRIER_AGC_GUARD_SAMPLES */
    /* Lock detection: a real carrier gives tightly-clustered raw
     * measurements window to window (observed: ~0.3 Hz spread on a live
     * station); noise on a dead channel has no coherent tone to track,
     * so consecutive windows disagree by a lot, essentially at random.
     * Requiring several consecutive windows to agree within a tight
     * tolerance is a self-calibrating "is this real" test, rather than
     * an absolute signal-strength threshold that would need its own
     * per-band tuning the same way the S-meter's S9 reference did.     */
    double         carrier_last_raw_hz;
    int            carrier_last_raw_valid;
    int            carrier_consec_agree;
    volatile int   carrier_locked_pub;
    /* Settle indicator ("(lock)" suffix) - tracks the PUBLISHED
     * (smoothed) value's own recent stability, separate from
     * carrier_locked_pub (which only means the raw measurement is
     * trustworthy enough to show at all). The smoothing keeps refining
     * for a while after that, so there's a real difference between
     * "showing a genuine reading" and "that reading has stopped
     * changing" - this tracks the latter.                               */
    float          carrier_last_published_hz;
    int            carrier_last_published_valid;
    int            carrier_settled_count;
    volatile int   carrier_settled_pub;
    double        resamp_acc;
    float         resamp_prev, resamp_cur;
    float         agc_gain;      /* smoothed, persistent gain - feedback loop,
                                   * not recomputed from scratch each sample */
    float         agc_envelope;  /* envelope-followed level feeding the gain
                                   * calculation below, instead of the raw
                                   * instantaneous sample - a proper detector
                                   * stage, same as any compressor/AGC design
                                   * normally has, separate from the gain
                                   * smoother itself. Without this, normal
                                   * program audio's peak-to-average swings
                                   * (a single loud syllable, a drum hit)
                                   * were yanking the gain calculation around
                                   * on every momentary peak - confirmed with
                                   * real data: a genuinely stable signal
                                   * (dbm varying ~4 dB) still showed
                                   * agc_gain swinging nearly 5x, since the
                                   * gain was chasing program dynamics, not
                                   * actual RF level.                        */
    int           dc_primed;   /* avoids a huge AM DC-blocker transient at start */
    int           mute_samples_left; /* forced silence right after any reset,
                                       * masks filter/AGC settling glitches   */

    /* cached copies used to detect when settings changed (thread-local) */
    int    last_mode, last_tuner;
    int    last_notch_enabled;
    double last_bw, last_notch, last_freq, last_center, last_native;
    int    last_mode_for_mute;   /* separate from last_mode - that field is
                                  * already updated by the decim-redesign
                                  * step earlier in the same call, so it
                                  * can't also be used to decide the mute
                                  * duration afterwards.                  */
} MonitorState;

static MonitorState g_monitor;
static int    g_ab_auto_pending = 1;    /* 1 = eligible to auto-enable A=B
                                         * the next time Monitor starts a
                                         * fresh session with matching Tuner
                                         * A/B frequencies. Starts eligible
                                         * (covers "since app start"), and
                                         * re-arms on every Settings Save -
                                         * consumed (set to 0) the moment it
                                         * actually fires, so it only ever
                                         * auto-applies once per app-start-
                                         * or-Save "epoch", never overriding
                                         * a deliberate manual toggle after. */

/* Monitor bar GUI controls */
static HWND g_hBtnMonitor    = NULL;
static HWND g_hBtnFreqLock   = NULL;
static HWND   g_hFreqDigits    = NULL;
static HFONT  g_hFontFreqDigits = NULL;
static int    g_freqDigitsHover = -1;   /* which digit the mouse is over, -1 = none */
static HWND g_hMonHzLbl      = NULL;
static HWND g_hMonModeLbl    = NULL;
static HWND g_hMonMode       = NULL;
static HWND g_hBwDigits      = NULL;
static int  g_bwDigitsHover  = -1;   /* which digit the mouse is over, -1 = none */
static HWND g_hMonKhzLbl     = NULL;
static HWND g_hBtnNotchEnable = NULL;
static HWND g_hNotchDigits   = NULL;
static int  g_notchDigitsHover = -1; /* -2 = sign cell, -1 = none, 0+ = digit  */
static int  g_notchNegative  = 0;    /* sign is tracked separately from the
                                      * magnitude digits - see monitor_layout
                                      * comment above notch_digits_wndproc */
static HWND g_hNotchKhzLbl   = NULL;
static HWND g_hMonVolLbl     = NULL;
static HWND g_hMonVol        = NULL;
static HWND g_hMonVolVal     = NULL;
static HWND g_hBtnHpfEnable  = NULL;
static HWND g_hHpfSlider     = NULL;
static HWND g_hHpfVal        = NULL;
static HWND g_hSMeter        = NULL;
static int  g_monitorVolPercent = 15;  /* Windows waveOut volume, 0-100,
                                          * kept here so re-opening the audio
                                          * device (monitor toggled off/on)
                                          * re-applies the level the user
                                          * last set rather than resetting. */

static void monitor_global_init(void);
static void monitor_shutdown(void);
static DWORD WINAPI monitor_thread_func(LPVOID param);
static const char *gui_record_btn_idle_label(void);
static void monitor_feed(const int16_t *xi, const int16_t *xq, unsigned int n);
typedef struct {
    const char *key;
    char        value[80];
    int         applied;
} IniPatchEntry;
static int ini_patch_values(const char *path, IniPatchEntry *entries, int n_entries);
static int ini_rewrite_schedule(const char *path, ScheduleEntry *entries, int count);

static HWND mk_button(HWND parent, int id, const char *text);
static void open_settings_dialog(HWND parent);
static void refresh_known_device_type(void);
static int  launch_slave_b_process(AppState *state, const char *outfile_b,
                                    int duration_sec, int listen_only);
static void stop_slave_b_process(AppState *state);
static void setup_slave_channel_b(AppState *state);
static void apply_slave_biast_b(AppState *state);
static DWORD WINAPI slave_b_monitor_reader_thread(LPVOID param);
static void layout_children(HWND hwnd);
static void gui_refresh_monitor_bar_visibility(void);
static void monitor_create_controls(HWND parent, HINSTANCE hInst);
static void monitor_layout(HWND hwnd, int right_edge, int bar_y, int bar_h);
static void monitor_apply_mode_from_combo(void);
static void monitor_apply_volume_from_slider(void);
static void monitor_apply_hpf_hz_from_slider(void);
static void monitor_toggle_tuner_sel(void);
static void monitor_switch_single_tuner_live(void);
static void monitor_sync_button_label(void);
static double monitor_center_for_tuner(int tuner_sel);
static double monitor_clamp_to_coverage(double freq, double center);
static double *monitor_active_freq_ptr(void);
static LRESULT CALLBACK freqdigits_wndproc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp);
static HWND freqdigits_create(HWND parent, HINSTANCE hInst);
static LRESULT CALLBACK bwdigits_wndproc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp);
static HWND bwdigits_create(HWND parent, HINSTANCE hInst);
static LRESULT CALLBACK notchdigits_wndproc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp);
static HWND notchdigits_create(HWND parent, HINSTANCE hInst);
static LRESULT CALLBACK smeter_wndproc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp);
static float smeter_s9_dbm_for_current_tuner(void);
static int   carrier_window_samples_for_signal(void);
static HWND smeter_create(HWND parent, HINSTANCE hInst);
static int  monitor_draw_button(LPDRAWITEMSTRUCT di);
static int  notch_draw_button(LPDRAWITEMSTRUCT di);
static int  hpf_draw_button(LPDRAWITEMSTRUCT di);

/* Forward declarations for the GUI front-end */
static DWORD WINAPI recording_worker(LPVOID param);
static DWORD WINAPI gui_monitor_thread_func(LPVOID param);
static void  gui_apply_agc_toggle(AppState *state);
static inline SIZE_T ring_available(const RingBuffer *rb);
static void  apply_reversed_wheel_subclass(HWND h);

/* -------------------------------------------------------------------------
 * log_write - GUI version. Formats the message and posts a heap-allocated
 * string to the main window, which appends it to the log edit control on
 * the UI thread. Also writes to the log file if one is open. Thread-safe.
 * ------------------------------------------------------------------------- */
/* Last N log lines, kept in a flat array purely so the crash handler
 * (crash_exception_filter, below) has something readable to dump without
 * touching g_hLog - a crash could happen on any thread, including one
 * that isn't the GUI thread, and a crash handler poking at HWNDs or
 * SendMessage is exactly the kind of thing that can turn one crash into
 * a hang or a second, unrecoverable one. Plain array, single atomic
 * increment per line, no lock: worst case under a genuine race is a
 * torn/stale line in the crash report, which is an acceptable trade for
 * never blocking or re-faulting inside the handler itself. */
#define CRASH_LOG_RING_LINES    40
#define CRASH_LOG_RING_LINE_LEN 200
static char         g_crash_log_ring[CRASH_LOG_RING_LINES][CRASH_LOG_RING_LINE_LEN];
static volatile LONG g_crash_log_ring_pos = 0;

static void log_write(const char *level, const char *fmt, ...)
{
    char buf[512];
    char line[640];
    va_list args;
    SYSTEMTIME st;

    /* Log clock follows the same UTC/local choice as the scheduler and
     * filenames (use_utc). With everything on one clock the timestamps in
     * the log line up with the scheduled times the user entered.          */
    if (g_state.cfg.use_utc)
        GetSystemTime(&st);
    else
        GetLocalTime(&st);
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    snprintf(line, sizeof(line), "[%02d:%02d:%02d%s] [%s] %s\r\n",
             st.wHour, st.wMinute, st.wSecond,
             g_state.cfg.use_utc ? "Z" : "", level, buf);

    if (g_hwnd) {
        /* Choose a colour code for the log window:
         *   0 = white (default)  1 = green (ok)  2 = orange (warn)  3 = red */
        int colour = 0;
        if      (strcmp(level, "ERROR") == 0) colour = 3;
        else if (strcmp(level, "WARN ") == 0) colour = 2;
        else if (strcmp(level, "OK   ") == 0) colour = 1;
        else {
            /* INFO lines that report a clean result also go green. */
            if (strstr(buf, "PASSED") || strstr(buf, "Verification") ||
                strstr(buf, "complete") || strstr(buf, "Session ended"))
                colour = 1;
        }

        size_t n = strlen(line) + 1;
        char *copy = (char *)malloc(n);
        if (copy) {
            memcpy(copy, line, n);
            /* Posted, not sent: never blocks the engine threads. */
            if (!PostMessageA(g_hwnd, WM_APP_LOG, (WPARAM)colour, (LPARAM)copy))
                free(copy);
        }
    }

    if (g_state.log_fp) {
        fprintf(g_state.log_fp, "[%04d-%02d-%02d %02d:%02d:%02d.%03d %s] [%s] %s\n",
                st.wYear, st.wMonth, st.wDay,
                st.wHour, st.wMinute, st.wSecond, st.wMilliseconds,
                g_state.cfg.use_utc ? "UTC" : "local",
                level, buf);
        fflush(g_state.log_fp);
    }

    {
        LONG slot = InterlockedIncrement(&g_crash_log_ring_pos) - 1;
        char *dst = g_crash_log_ring[slot % CRASH_LOG_RING_LINES];
        strncpy(dst, line, CRASH_LOG_RING_LINE_LEN - 1);
        dst[CRASH_LOG_RING_LINE_LEN - 1] = '\0';
    }
}

#define LOG_INFO(...)  log_write("INFO ", __VA_ARGS__)
#define LOG_OK(...)    log_write("OK   ", __VA_ARGS__)
#define LOG_WARN(...)  log_write("WARN ", __VA_ARGS__)
#define LOG_ERROR(...) log_write("ERROR", __VA_ARGS__)

/* =========================================================================
 * Crash handler
 *
 * Installed via SetUnhandledExceptionFilter() near the top of WinMain, this
 * catches any exception that would otherwise show the generic "DuoDX has
 * stopped working" Windows dialog and leave no trace of what happened.
 * Instead it writes, next to duodx.exe:
 *   duodx_crash_YYYYMMDDTHHMMSSZ.txt  - exception info, app state, the
 *                                       last CRASH_LOG_RING_LINES log
 *                                       lines, and a raw stack backtrace
 *   duodx_crash_YYYYMMDDTHHMMSSZ.dmp  - a minidump (best effort; if
 *                                       dbghelp.dll can't be loaded for
 *                                       any reason, the .txt above is
 *                                       still written on its own)
 *
 * The stack backtrace in the .txt is module-relative offsets rather than
 * resolved symbol names - MinGW's DWARF debug info isn't something
 * dbghelp's symbol engine reads, so resolving names here isn't reliable.
 * Offsets can be turned into file/line with, against the exact duodx.exe
 * that crashed (built with -g):
 *     addr2line -e duodx.exe -f -C <offset>
 *
 * Deliberately conservative throughout: this code runs in an already-
 * crashed process, so it avoids anything that could itself fault - no
 * dynamic allocation beyond fixed stack buffers, no touching HWNDs, no
 * locks, no calling back into log_write(). Best effort; if something here
 * fails, it fails silently and moves on rather than risking a second
 * crash while trying to report the first one.
 * ========================================================================= */
typedef BOOL (WINAPI *PFN_MiniDumpWriteDump)(
    HANDLE hProcess, DWORD ProcessId, HANDLE hFile, MINIDUMP_TYPE DumpType,
    PMINIDUMP_EXCEPTION_INFORMATION ExceptionParam,
    PMINIDUMP_USER_STREAM_INFORMATION UserStreamParam,
    PMINIDUMP_CALLBACK_INFORMATION CallbackParam);

static const char *crash_exception_name(DWORD code)
{
    switch (code) {
    case EXCEPTION_ACCESS_VIOLATION:         return "ACCESS_VIOLATION";
    case EXCEPTION_STACK_OVERFLOW:           return "STACK_OVERFLOW";
    case EXCEPTION_ILLEGAL_INSTRUCTION:      return "ILLEGAL_INSTRUCTION";
    case EXCEPTION_INT_DIVIDE_BY_ZERO:       return "INT_DIVIDE_BY_ZERO";
    case EXCEPTION_ARRAY_BOUNDS_EXCEEDED:    return "ARRAY_BOUNDS_EXCEEDED";
    case EXCEPTION_DATATYPE_MISALIGNMENT:    return "DATATYPE_MISALIGNMENT";
    case EXCEPTION_FLT_DIVIDE_BY_ZERO:       return "FLT_DIVIDE_BY_ZERO";
    case EXCEPTION_FLT_INVALID_OPERATION:    return "FLT_INVALID_OPERATION";
    case EXCEPTION_FLT_OVERFLOW:             return "FLT_OVERFLOW";
    case EXCEPTION_FLT_UNDERFLOW:            return "FLT_UNDERFLOW";
    case EXCEPTION_PRIV_INSTRUCTION:         return "PRIV_INSTRUCTION";
    case EXCEPTION_IN_PAGE_ERROR:            return "IN_PAGE_ERROR";
    case EXCEPTION_NONCONTINUABLE_EXCEPTION: return "NONCONTINUABLE_EXCEPTION";
    case EXCEPTION_BREAKPOINT:               return "BREAKPOINT";
    case EXCEPTION_SINGLE_STEP:              return "SINGLE_STEP";
    default:                                 return "UNKNOWN";
    }
}

static LONG WINAPI crash_exception_filter(EXCEPTION_POINTERS *ep)
{
    static volatile LONG already_handling = 0;
    char exe_path[MAX_PATH];
    char dir[MAX_PATH];
    char path_txt[MAX_PATH];
    char path_dmp[MAX_PATH];
    char *slash;
    SYSTEMTIME st;
    HANDLE hFile;
    DWORD written;
    char buf[2048];
    int len, i;
    void *stack_addrs[32];
    USHORT n_frames;
    HMODULE hSelf = GetModuleHandleA(NULL);
    DWORD code = ep->ExceptionRecord->ExceptionCode;
    void *addr = ep->ExceptionRecord->ExceptionAddress;

    /* A crash while already handling a crash - stop here rather than
     * risk looping or corrupting the first (more useful) report.        */
    if (InterlockedCompareExchange(&already_handling, 1, 0) != 0)
        return EXCEPTION_EXECUTE_HANDLER;

    /* Reclaim the guard-page stack space a stack overflow just used up,
     * or WriteFile/CaptureStackBackTrace below may themselves fault.    */
    if (code == EXCEPTION_STACK_OVERFLOW)
        _resetstkoflw();

    GetSystemTime(&st);
    GetModuleFileNameA(NULL, exe_path, MAX_PATH);
    strncpy(dir, exe_path, MAX_PATH - 1);
    dir[MAX_PATH - 1] = '\0';
    slash = strrchr(dir, '\\');
    if (slash) *slash = '\0'; else dir[0] = '\0';

    snprintf(path_txt, MAX_PATH, "%s%sduodx_crash_%04d%02d%02dT%02d%02d%02dZ.txt",
             dir, dir[0] ? "\\" : "",
             st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
    snprintf(path_dmp, MAX_PATH, "%s%sduodx_crash_%04d%02d%02dT%02d%02d%02dZ.dmp",
             dir, dir[0] ? "\\" : "",
             st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);

    /* ---- Text report ---- */
    hFile = CreateFileA(path_txt, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS,
                         FILE_ATTRIBUTE_NORMAL, NULL);
    if (hFile != INVALID_HANDLE_VALUE) {
        len = snprintf(buf, sizeof(buf),
            "DuoDX crash report\r\n"
            "Time (UTC)      : %04d-%02d-%02d %02d:%02d:%02d\r\n"
            "Exception       : %s (0x%08lX)\r\n"
            "Fault address   : %p  (module+0x%llX)\r\n"
            "Module base     : %p\r\n"
            "Recording       : %s\r\n"
            "Listening       : %s\r\n"
            "Device          : hwVer=%u  SerNo=%s\r\n"
            "Frequency (MHz) : %.6f\r\n"
            "Dual channel    : %d   HDR: %d   Decimation: %d\r\n"
            "\r\n"
            "-- Last log lines --\r\n",
            st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond,
            crash_exception_name(code), code,
            addr, (unsigned long long)((UINT_PTR)addr - (UINT_PTR)hSelf),
            (void *)hSelf,
            g_state.stream_running ? "yes" : "no",
            g_state.listening ? "yes" : "no",
            (unsigned)g_state.device.hwVer, g_state.device.SerNo,
            g_state.cfg.frequency_hz / 1e6,
            g_state.cfg.dual_channel, g_state.cfg.hdr_enable, g_state.cfg.decimation);
        WriteFile(hFile, buf, (DWORD)len, &written, NULL);

        {
            LONG pos = g_crash_log_ring_pos;
            int count = (pos >= CRASH_LOG_RING_LINES) ? CRASH_LOG_RING_LINES : (int)pos;
            int start = (pos >= CRASH_LOG_RING_LINES) ? (int)(pos % CRASH_LOG_RING_LINES) : 0;
            for (i = 0; i < count; i++) {
                int idx = (start + i) % CRASH_LOG_RING_LINES;
                if (g_crash_log_ring[idx][0])
                    WriteFile(hFile, g_crash_log_ring[idx],
                              (DWORD)strlen(g_crash_log_ring[idx]), &written, NULL);
            }
        }

        n_frames = CaptureStackBackTrace(0, 32, stack_addrs, NULL);
        len = snprintf(buf, sizeof(buf),
            "\r\n-- Stack (module-relative offsets; resolve with "
            "'addr2line -e duodx.exe -f -C <offset>' against this build) --\r\n");
        WriteFile(hFile, buf, (DWORD)len, &written, NULL);
        for (i = 0; i < n_frames; i++) {
            len = snprintf(buf, sizeof(buf), "  0x%llX\r\n",
                (unsigned long long)((UINT_PTR)stack_addrs[i] - (UINT_PTR)hSelf));
            WriteFile(hFile, buf, (DWORD)len, &written, NULL);
        }

        CloseHandle(hFile);
    }

    /* ---- Minidump (best effort) ---- */
    {
        HMODULE hDbg = LoadLibraryA("dbghelp.dll");
        if (hDbg) {
            PFN_MiniDumpWriteDump pMdwd =
                (PFN_MiniDumpWriteDump)GetProcAddress(hDbg, "MiniDumpWriteDump");
            if (pMdwd) {
                HANDLE hDump = CreateFileA(path_dmp, GENERIC_WRITE, 0, NULL,
                                           CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
                if (hDump != INVALID_HANDLE_VALUE) {
                    MINIDUMP_EXCEPTION_INFORMATION mei;
                    mei.ThreadId          = GetCurrentThreadId();
                    mei.ExceptionPointers = ep;
                    mei.ClientPointers    = FALSE;
                    pMdwd(GetCurrentProcess(), GetCurrentProcessId(), hDump,
                          MiniDumpNormal, &mei, NULL, NULL);
                    CloseHandle(hDump);
                }
            }
            FreeLibrary(hDbg);
        }
    }

    if (g_state.log_fp) fflush(g_state.log_fp);

    return EXCEPTION_EXECUTE_HANDLER;
}

/* -------------------------------------------------------------------------
 * gui_apply_agc_toggle - toggle AGC on the live stream. Extracted from the
 * original keyboard handler's 'G' key. Called from the engine's main wait
 * loop when the AGC button has been pressed (g_agc_toggle_req).
 * ------------------------------------------------------------------------- */
/* Pushes the current gain_reduction/lna_state to an already-running
 * device without restarting it - used when Settings is saved while
 * listening, so gain can actually be tuned against the live meters.
 * Single-tuner only, matching listening mode's current scope. Mirrors
 * the live-update pattern gui_apply_agc_toggle() already uses.          */
static void gui_apply_live_gain(AppState *state)
{
    sdrplay_api_ErrT err;
    if (!state->ch_a_params) return;

    state->ch_a_params->tunerParams.gain.gRdB     = state->cfg.gain_reduction;
    state->ch_a_params->tunerParams.gain.LNAstate = (unsigned char)state->cfg.lna_state;

    err = sdrplay_api_Update(state->device.dev, state->device.tuner,
                              sdrplay_api_Update_Tuner_Gr,
                              sdrplay_api_Update_Ext1_None);
    if (err != sdrplay_api_Success) {
        Sleep(120);
        err = sdrplay_api_Update(state->device.dev, state->device.tuner,
                                  sdrplay_api_Update_Tuner_Gr,
                                  sdrplay_api_Update_Ext1_None);
    }
    if (err == sdrplay_api_Success)
        LOG_INFO("Gain applied live: GR=%d dB  LNA=%d",
                 state->cfg.gain_reduction, state->cfg.lna_state);
    else
        LOG_WARN("Live gain update failed: %s", sdrplay_api_GetErrorString(err));
}

static void gui_apply_agc_toggle(AppState *state)
{
    int is_dual = state->cfg.dual_channel;

    if (state->cfg.hdr_enable) {
        LOG_INFO("AGC control disabled in HDR mode (fixed gain path).");
        return;
    }
    if (!state->ch_a_params) return;

    int cur_agc = state->ch_a_params->ctrlParams.agc.enable;
    int new_agc = (cur_agc == sdrplay_api_AGC_DISABLE)
                  ? sdrplay_api_AGC_CTRL_EN
                  : sdrplay_api_AGC_DISABLE;

    state->ch_a_params->ctrlParams.agc.enable = new_agc;
    if (is_dual && state->ch_b_params)
        state->ch_b_params->ctrlParams.agc.enable = new_agc;

    sdrplay_api_TunerSelectT t = is_dual
        ? sdrplay_api_Tuner_Both : state->device.tuner;

    if (new_agc == sdrplay_api_AGC_DISABLE) {
        int gr_a = state->cfg.gain_reduction;
        int gr_b = (state->cfg.gain_reduction_b >= 0)
                   ? state->cfg.gain_reduction_b
                   : state->cfg.gain_reduction;
        state->ch_a_params->tunerParams.gain.gRdB = gr_a;
        if (is_dual && state->ch_b_params)
            state->ch_b_params->tunerParams.gain.gRdB = gr_b;

        sdrplay_api_ErrT err = sdrplay_api_Update(
            state->device.dev, t,
            (sdrplay_api_ReasonForUpdateT)
                (sdrplay_api_Update_Ctrl_Agc |
                 sdrplay_api_Update_Tuner_Gr),
            sdrplay_api_Update_Ext1_None);
        if (err != sdrplay_api_Success) {
            /* Transient NotInitialised can occur if a previous update is
             * still settling; wait briefly and retry once.                 */
            Sleep(120);
            err = sdrplay_api_Update(
                state->device.dev, t,
                (sdrplay_api_ReasonForUpdateT)
                    (sdrplay_api_Update_Ctrl_Agc |
                     sdrplay_api_Update_Tuner_Gr),
                sdrplay_api_Update_Ext1_None);
        }
        if (err == sdrplay_api_Success) {
            if (is_dual)
                LOG_INFO("AGC off - T1=%d dB  T2=%d dB", gr_a, gr_b);
            else
                LOG_INFO("AGC off - GR=%d dB", gr_a);
        } else {
            state->ch_a_params->ctrlParams.agc.enable = cur_agc;
            if (is_dual && state->ch_b_params)
                state->ch_b_params->ctrlParams.agc.enable = cur_agc;
            LOG_WARN("AGC off failed: %s", sdrplay_api_GetErrorString(err));
        }
    } else {
        sdrplay_api_ErrT err = sdrplay_api_Update(
            state->device.dev, t,
            sdrplay_api_Update_Ctrl_Agc, sdrplay_api_Update_Ext1_None);
        if (err != sdrplay_api_Success) {
            Sleep(120);
            err = sdrplay_api_Update(
                state->device.dev, t,
                sdrplay_api_Update_Ctrl_Agc, sdrplay_api_Update_Ext1_None);
        }
        if (err == sdrplay_api_Success) {
            LOG_INFO(is_dual ? "AGC on (both tuners)" : "AGC on");
        } else {
            state->ch_a_params->ctrlParams.agc.enable = cur_agc;
            if (is_dual && state->ch_b_params)
                state->ch_b_params->ctrlParams.agc.enable = cur_agc;
            LOG_WARN("AGC on failed: %s", sdrplay_api_GetErrorString(err));
        }
    }
}

/* -------------------------------------------------------------------------
 * GUI monitor thread - mirrors the engine statistics into the status
 * labels. Runs for the lifetime of a worker session (g_worker_active).
 * Uses SetWindowTextA via the UI thread is not required for static labels;
 * SetWindowText is thread-safe enough for our update rate, but to be safe
 * we marshal through the message queue is unnecessary here - we update
 * labels directly at a modest interval.
 * ------------------------------------------------------------------------- */

/* CF display text, purely derived from config - no live session data
 * needed, same reasoning as gui_compute_coverage_span above.           */
static void gui_compute_cf_text(char *out, size_t out_size)
{
    if (g_state.cfg.dual_channel || g_state.master_slave_active)
        snprintf(out, out_size, "A %.3f / B %.3f MHz",
                 g_state.cfg.frequency_hz / 1e6,
                 g_state.cfg.freq_b_hz / 1e6);
    else
        snprintf(out, out_size, "%.3f MHz", g_state.cfg.frequency_hz / 1e6);
}

/* Coverage span = centre frequency +/- half the USABLE signal bandwidth -
 * the analog IF filter's actual width, not the full recorded IQ span (the
 * two differ for every combination in VALID_COMBOS; see
 * expected_usable_bw_hz's own comment for why). Previously this showed the
 * full recorded span instead, which for a narrow filter setting read as if
 * the whole span were solid signal - e.g. "9.000 - 11.000 MHz" for a 200
 * kHz filter, when only the ~150 kHz in the middle is actually usable.
 * Falls back to the full span if no usable-BW figure is available (an
 * unrecognised combo set directly via ini, outside VALID_COMBOS).
 *
 * While idle this reads the live config, so a Settings save previews the
 * upcoming session's coverage immediately (right after a save, before
 * Record/Monitor is even pressed). But once a session is actually
 * running, expected_output_rate_hz/expected_usable_bw_hz no longer
 * describe it reliably: sample rate/IF/BW is never pushed to a running
 * device, yet validate_config() still recomputes both on every Settings
 * save regardless of session state - so this must fall back to the
 * frozen live_ versions while listening or recording, or a mid-session
 * Settings save would show a coverage window that doesn't match what's
 * actually streaming (see the AppState field comments).                 */
static void gui_compute_coverage_span(char *out, size_t out_size)
{
    double rate = (g_state.listening || g_state.stream_running)
                      ? g_state.live_expected_output_rate_hz
                      : g_state.cfg.expected_output_rate_hz;
    if (rate <= 0.0) rate = g_state.cfg.sample_rate_hz;

    double usable = (g_state.listening || g_state.stream_running)
                        ? g_state.live_expected_usable_bw_hz
                        : g_state.cfg.expected_usable_bw_hz;
    if (usable <= 0.0) usable = rate;

    if (g_state.cfg.dual_channel || g_state.master_slave_active) {
        char span_a[40], span_b[40];
        double half = usable / 2.0;
        double lo_a = g_state.cfg.frequency_hz - half;
        double hi_a = g_state.cfg.frequency_hz + half;
        double lo_b = g_state.cfg.freq_b_hz - half;
        double hi_b = g_state.cfg.freq_b_hz + half;
        if (lo_a < 0.0) lo_a = 0.0;
        if (lo_b < 0.0) lo_b = 0.0;

        if (g_state.cfg.frequency_hz < 3.0e6)
            snprintf(span_a, sizeof(span_a), "A %.0f-%.0f kHz", lo_a / 1e3, hi_a / 1e3);
        else
            snprintf(span_a, sizeof(span_a), "A %.3f-%.3f MHz", lo_a / 1e6, hi_a / 1e6);

        if (g_state.cfg.freq_b_hz < 3.0e6)
            snprintf(span_b, sizeof(span_b), "B %.0f-%.0f kHz", lo_b / 1e3, hi_b / 1e3);
        else
            snprintf(span_b, sizeof(span_b), "B %.3f-%.3f MHz", lo_b / 1e6, hi_b / 1e6);

        snprintf(out, out_size, "%s  /  %s", span_a, span_b);
    } else {
        double half = usable / 2.0;
        double lo = g_state.cfg.frequency_hz - half;
        double hi = g_state.cfg.frequency_hz + half;
        if (lo < 0.0) lo = 0.0;
        /* Show in kHz for MW, MHz once we are above ~3 MHz centre. */
        if (g_state.cfg.frequency_hz < 3.0e6)
            snprintf(out, out_size, "%.0f - %.0f kHz", lo / 1e3, hi / 1e3);
        else
            snprintf(out, out_size, "%.3f - %.3f MHz", lo / 1e6, hi / 1e6);
    }
}

static DWORD WINAPI gui_monitor_thread_func(LPVOID param)
{
    AppState *state = (AppState *)param;

    while (g_worker_active) {
        UiSnapshot s;
        memset(&s, 0, sizeof(s));
        /* Preserve infobar from previous tick until device is known */
        strncpy(s.infobar, g_ui.infobar, sizeof(s.infobar) - 1);

        s.recording = state->stream_running ? 1 : 0;
        s.listening = state->listening ? 1 : 0;
        s.finished  = (!state->stream_running && state->session_complete) ? 1 : 0;
        s.dual      = state->cfg.dual_channel;
        /* Coherent dual-channel indicator: only meaningful on an RSPduo with
         * dual_channel enabled and both tuners on the same frequency (the
         * condition for phase-coherent diversity reception - see Section 7.3
         * of the user guide). Hidden for any other device or configuration. */
        s.coherent = ((s.recording || s.listening) && state->cfg.dual_channel &&
                      state->device.hwVer == SDRPLAY_RSPduo_ID &&
                      fabs(state->cfg.frequency_hz - state->cfg.freq_b_hz) < 1.0)
                     ? 1 : 0;
        s.master_slave = ((s.recording || s.listening) && state->master_slave_active) ? 1 : 0;

        /* Info strip — built whenever device is known, persists after recording */
        {
            const char *dn = "RSP";
            unsigned char hw = state->device.hwVer;
            if (hw) {  /* only once device has been enumerated */
                if      (hw == SDRPLAY_RSP1_ID)    dn = "RSP1";
                else if (hw == SDRPLAY_RSP1A_ID)   dn = "RSP1A";
                else if (hw == SDRPLAY_RSP1B_ID)   dn = "RSP1B";
                else if (hw == SDRPLAY_RSP2_ID)    dn = "RSP2";
                else if (hw == SDRPLAY_RSPduo_ID)  dn = "RSPduo";
                else if (hw == SDRPLAY_RSPdx_ID)   dn = "RSPdx";
                else if (hw == SDRPLAY_RSPdxR2_ID) dn = "RSPdxR2";
                double sr_display = state->live_expected_output_rate_hz > 0.0
                                        ? state->live_expected_output_rate_hz
                                        : state->cfg.expected_output_rate_hz;
                if (state->cfg.dual_channel)
                    snprintf(s.infobar, sizeof(s.infobar),
                             "%s  \xB7  Ant: %s  \xB7  GR: %d/%d dB  LNA: %d/%d  \xB7  SR: %.3g Msps",
                             dn,
                             state->live_antenna[0] ? state->live_antenna : "-",
                             state->cfg.gain_reduction,
                             state->cfg.gain_reduction_b >= 0 ? state->cfg.gain_reduction_b : state->cfg.gain_reduction,
                             state->cfg.lna_state,
                             state->cfg.lna_state_b >= 0 ? state->cfg.lna_state_b : state->cfg.lna_state,
                             sr_display / 1e6);
                else
                    snprintf(s.infobar, sizeof(s.infobar),
                             "%s  \xB7  Ant: %s  \xB7  GR: %d dB  LNA: %d  \xB7  SR: %.3g Msps",
                             dn,
                             state->live_antenna[0] ? state->live_antenna : "-",
                             state->cfg.gain_reduction,
                             state->cfg.lna_state,
                             sr_display / 1e6);
            }
        }

        /* Elapsed */
        if (state->stream_running && state->start_time.QuadPart > 0
                && state->perf_freq.QuadPart > 0) {
            LARGE_INTEGER now;
            QueryPerformanceCounter(&now);
            s.elapsed_sec = (double)(now.QuadPart - state->start_time.QuadPart)
                            / (double)state->perf_freq.QuadPart;
        } else if (state->last_display_elapsed > 0.0) {
            /* Not streaming: show the last completed recording's length.
             * Uses last_display_elapsed (not frozen_elapsed_sec) so it is
             * not wiped by the between-entry reset.                        */
            s.elapsed_sec = state->last_display_elapsed;
        }

        /* File size: compute from samples_written (a live counter the writer
         * increments), NOT GetFileSizeEx. The OS write cache makes the on-disk
         * size lag by many MB during recording, which previously made the
         * display appear stuck. frame = 4 bytes (single) or 8 (dual).        */
        if (state->stream_running) {
            int frame_bytes = state->cfg.dual_channel ? 8 : 4;
            double bytes = (double)state->samples_written * frame_bytes;
            s.file_mb = bytes / (1024.0 * 1024.0);
        } else if (state->last_display_file_mb >= 0) {
            s.file_mb = (double)state->last_display_file_mb;
        }

        s.disk_free_mb = (double)state->disk_free_mb;

        /* Peak dBFS - read the current peak, then reset to silence so the
         * next interval captures a fresh peak from the callbacks. The
         * callback only ever raises the stored value, so this read/reset is
         * safe for a level display.                                         */
        s.peak_a = state->peak_dbfs;
        s.peak_b = state->peak_dbfs_b;
        state->peak_dbfs   = -90.0f;
        state->peak_dbfs_b = -90.0f;

        s.overload_a = state->overload_tuner_a;
        s.overload_b = state->overload_tuner_b;
        /* Real AGC state read from the live channel-A control params. */
        s.agc_on = (state->ch_a_params &&
                    state->ch_a_params->ctrlParams.agc.enable
                        != sdrplay_api_AGC_DISABLE) ? 1 : 0;
        s.hdr_on = state->cfg.hdr_enable ? 1 : 0;
        s.overflows  = state->overflows;
        s.dropped    = state->zero_frames_written;

        /* Ring buffer fill % */
        if (state->ring.size > 0) {
            SIZE_T used = ring_available(&state->ring);
            s.ring_pct = (float)((100.0 * (double)used) / (double)state->ring.size);
            if (s.ring_pct > 100.0f) s.ring_pct = 100.0f;
        }

        /* State text. A pending next_start takes priority over plain
         * "listening" - the device can be open and streaming (so Monitor
         * works) while still fundamentally waiting for a specific time,
         * e.g. during an hourly or scheduled pre-window wait, and WAITING
         * is the more informative thing to show in that case.            */
        if (s.finished)
            strncpy(s.state, "FINISHED", sizeof(s.state) - 1);
        else if (state->next_start[0])
            strncpy(s.state, "WAITING", sizeof(s.state) - 1);
        else if (s.listening)
            strncpy(s.state, "IDLE", sizeof(s.state) - 1);
        else if (s.recording)
            strncpy(s.state, "RECORDING", sizeof(s.state) - 1);
        else
            strncpy(s.state, "IDLE", sizeof(s.state) - 1);

        strncpy(s.next, state->next_start, sizeof(s.next) - 1);

        /* Scheduling status for the bottom info line.
         * Only shown while waiting, not during active recording
         * (the recording LED makes it obvious what state we're in). */
        if (s.recording) {
            s.sched[0] = '\0';
        } else if (state->cfg.hourly_enable) {
            /* Both hourly and schedule (below) repeat unconditionally
             * once armed - see the loop-back logic in recording_worker -
             * so there's nothing conditional worth calling out in the
             * text here.                                                */
            snprintf(s.sched, sizeof(s.sched),
                     "Hourly %d min%s%s",
                     state->cfg.hourly_window_min,
                     state->next_start[0] ? "  starts " : "",
                     state->next_start[0] ? state->next_start : "");
        } else if (state->cfg.schedule_only && state->cfg.schedule_count > 0) {
            snprintf(s.sched, sizeof(s.sched),
                     "Schedule: %d entr%s%s%s",
                     state->cfg.schedule_count,
                     state->cfg.schedule_count == 1 ? "y" : "ies",
                     state->next_start[0] ? "  starts " : "",
                     state->next_start[0] ? state->next_start : "");
        } else if (state->next_start[0]) {
            snprintf(s.sched, sizeof(s.sched), "Start at %s",
                     state->next_start);
        } else {
            s.sched[0] = '\0';
        }

        gui_compute_cf_text(s.freq, sizeof(s.freq));

        /* Coverage span = centre frequency +/- half the recorded IQ bandwidth.
         * In dual or Master/Slave mode, show both tuners' windows separately -
         * they can be on completely unrelated bands (e.g. MW and shortwave),
         * so a single combined range wouldn't mean anything.              */
        gui_compute_coverage_span(s.span, sizeof(s.span));

        g_ui = s;   /* publish */

        /* Repaint only the dynamic area above the log control, not the whole
         * window. Painting behind the child log every tick caused a visible
         * flicker. Also refresh the bottom info line + buttons region.       */
        if (g_hwnd) {
            RECT cr, lr;
            GetClientRect(g_hwnd, &cr);
            /* top region: from top down to just above the log */
            if (g_hLog) {
                GetWindowRect(g_hLog, &lr);
                POINT tl = { lr.left, lr.top };
                ScreenToClient(g_hwnd, &tl);
                RECT top = { 0, 0, cr.right, tl.y };
                InvalidateRect(g_hwnd, &top, FALSE);
                /* bottom region: only the scheduling-text strip between the
                 * log and the button bar. Exclude the button row itself so
                 * the owner-drawn buttons don't flicker every monitor tick.
                 * The buttons are repainted only when their state changes.
                 * Now also gated on the text actually changing - this strip's
                 * bounds vertically span the whole monitor bar area too (its
                 * bottom reaches cr.bottom, not just above the button row),
                 * so invalidating it unconditionally every tick was repainting
                 * the entire left portion of the monitor bar controls too. */
                int log_bot = tl.y + (int)(lr.bottom - lr.top);
                int mon_h3 = g_monitor_bar_visible_eff
                           ? (BOTTOM_MON_BAR_H + BOTTOM_MON_ROW_GAP + BOTTOM_MON_BAR2_H)
                           : 0;
                int btn_top = g_monitor_bar_visible_eff
                    ? (cr.bottom - mon_h3 - BOTTOM_MON_GAP - BOTTOM_BTN_ROW_H - BOTTOM_BTN_GAP)
                    : (cr.bottom - BOTTOM_BTN_ROW_H - BOTTOM_MON_GAP);
                if (log_bot < btn_top &&
                        strncmp(g_ui.sched, g_last_sched_text,
                                sizeof(g_last_sched_text)) != 0) {
                    RECT sched_strip = { 0, log_bot, cr.right, btn_top };
                    InvalidateRect(g_hwnd, &sched_strip, FALSE);
                    strncpy(g_last_sched_text, g_ui.sched,
                            sizeof(g_last_sched_text) - 1);
                }
                /* Carrier readout - now drawn aligned above the frequency
                 * dial rather than sharing the info-strip's left-hand
                 * slot with infobar/sched, so it needs its own comparison
                 * and its own invalidate rect (full row width, since its
                 * X position tracks the dial and isn't fixed at the left
                 * margin the way infobar's rect above assumes).          */
                {
                    char cur[64];
                    cur[0] = '\0';
                    if (g_monitor.enabled && g_monitor.carrier_offset_valid_pub &&
                            g_monitor.carrier_locked_pub) {
                        double dial_hz = (g_monitor.tuner_sel == 1)
                                       ? g_monitor.freq_hz_b : g_monitor.freq_hz;
                        double carrier_hz = dial_hz + (double)g_monitor.carrier_offset_hz_pub;
                        snprintf(cur, sizeof(cur), "OFFSET %.4f %d", carrier_hz / 1000.0,
                                 g_monitor.carrier_settled_pub);
                    }
                    if (strncmp(cur, g_last_infostrip_text, sizeof(g_last_infostrip_text)) != 0) {
                        RECT ib_strip2 = { 0, btn_top, cr.right, cr.bottom };
                        InvalidateRect(g_hwnd, &ib_strip2, FALSE);
                        strncpy(g_last_infostrip_text, cur, sizeof(g_last_infostrip_text) - 1);
                        g_last_infostrip_text[sizeof(g_last_infostrip_text) - 1] = '\0';
                    }
                }
            } else {
                InvalidateRect(g_hwnd, NULL, FALSE);
            }
        }
        /* Update AGC button state only when it changes, to avoid flicker. */
        if (g_hBtnAgc) {
            int agc_enabled = (s.recording || s.listening) && !s.hdr_on;
            if (s.agc_on != g_last_agc_on || agc_enabled != g_last_agc_enabled) {
                EnableWindow(g_hBtnAgc, agc_enabled);
                InvalidateRect(g_hBtnAgc, NULL, FALSE);
                g_last_agc_on      = s.agc_on;
                g_last_agc_enabled = agc_enabled;
            }
        }
        /* Timer button: reflects either timer mode being active (schedule_only
         * OR hourly_enable - see the unified button's own click handler).
         * Always enabled now - turning it off is meaningful both while
         * listening (cancels an active wait) and while actually recording
         * (stops the hourly/schedule sequence repeating afterward, without
         * touching the file currently being written) - unlike the old
         * Schedule button this replaced, where toggling schedule_only
         * mid-session genuinely couldn't do anything. Still only repaints
         * when the label actually changes, using the session-reset
         * globals above, to avoid needless flicker on every tick.        */
        if (g_hBtnSchedToggle) {
            int sched_on = (g_state.cfg.schedule_only || g_state.cfg.hourly_enable) ? 1 : 0;
            if (sched_on != g_last_sched_on) {
                SetWindowTextA(g_hBtnSchedToggle,
                               sched_on ? "Timer: ON" : "Timer: OFF");
                EnableWindow(g_hBtnSchedToggle, TRUE);
                InvalidateRect(g_hBtnSchedToggle, NULL, FALSE);
                g_last_sched_on = sched_on;
            }
        }
        monitor_sync_button_label();

        if (g_hSMeter && g_monitor.enabled)
            InvalidateRect(g_hSMeter, NULL, FALSE);

        Sleep(state->cfg.monitor_interval_ms > 0
              ? state->cfg.monitor_interval_ms : 250);
    }
    return 0;
}

/* =========================================================================
 * Ring buffer implementation
 * ========================================================================= */
static int ring_init(RingBuffer *rb, SIZE_T min_size)
{
    SIZE_T size = 1;

    /* Round up to power of 2 */
    while (size < min_size)
        size <<= 1;

    /* Cap at maximum */
    if (size > RING_BUFFER_MAX_BYTES)
        size = RING_BUFFER_MAX_BYTES;

    rb->buf = (uint8_t *)VirtualAlloc(NULL, size,
                                       MEM_COMMIT | MEM_RESERVE,
                                       PAGE_READWRITE);
    if (!rb->buf)
        return -1;

    rb->size      = size;
    rb->mask      = size - 1;
    rb->write_idx    = 0;
    rb->read_idx     = 0;
    rb->overflow     = 0;
    rb->bytes_dropped = 0;
    return 0;
}

static void ring_free(RingBuffer *rb)
{
    if (rb->buf) {
        VirtualFree(rb->buf, 0, MEM_RELEASE);
        rb->buf = NULL;
    }
}

/* Reset read/write indices to reuse an already-allocated ring buffer */
static void ring_reset(RingBuffer *rb)
{
    rb->write_idx     = 0;
    rb->read_idx      = 0;
    rb->overflow      = 0;
    rb->bytes_dropped = 0;
}

/* Returns number of bytes free in the ring buffer */
static inline SIZE_T ring_free_space(const RingBuffer *rb)
{
    LONG64 w = rb->write_idx;
    LONG64 r = rb->read_idx;
    return (SIZE_T)(rb->size - (SIZE_T)(w - r));
}

/* Returns number of bytes available to read */
static inline SIZE_T ring_available(const RingBuffer *rb)
{
    LONG64 w = rb->write_idx;
    LONG64 r = rb->read_idx;
    return (SIZE_T)(w - r);
}

/*
 * Write bytes into the ring buffer from the callback.
 * NEVER blocks. Returns 0 on success, -1 on overflow.
 * CRITICAL: this must be fast - no system calls, no locks.
 */
static int ring_write(RingBuffer *rb, const void *data, SIZE_T len)
{
    SIZE_T free_space;
    SIZE_T write_pos;
    SIZE_T first_chunk;
    const uint8_t *src = (const uint8_t *)data;

    free_space = ring_free_space(rb);
    if (len > free_space) {
        /* Record exactly how many bytes are being dropped so the writer
         * thread can insert compensating zero frames to preserve timing. */
        InterlockedIncrement(&rb->overflow);
        InterlockedAdd64(&rb->bytes_dropped, (LONG64)len);
        return -1;
    }

    write_pos   = (SIZE_T)(rb->write_idx & rb->mask);
    first_chunk = rb->size - write_pos;

    if (len <= first_chunk) {
        memcpy(rb->buf + write_pos, src, len);
    } else {
        memcpy(rb->buf + write_pos, src, first_chunk);
        memcpy(rb->buf, src + first_chunk, len - first_chunk);
    }

    /* Memory barrier then advance write index */
    MemoryBarrier();
    InterlockedAdd64(&rb->write_idx, (LONG64)len);
    return 0;
}

/*
 * Read bytes from the ring buffer on the writer thread.
 * Returns number of bytes actually read.
 */
static SIZE_T ring_read(RingBuffer *rb, void *dst, SIZE_T len)
{
    SIZE_T avail     = ring_available(rb);
    SIZE_T to_read   = (len < avail) ? len : avail;
    SIZE_T read_pos  = (SIZE_T)(rb->read_idx & rb->mask);
    SIZE_T first_chunk;
    uint8_t *out = (uint8_t *)dst;

    if (to_read == 0)
        return 0;

    first_chunk = rb->size - read_pos;
    if (to_read <= first_chunk) {
        memcpy(out, rb->buf + read_pos, to_read);
    } else {
        memcpy(out, rb->buf + read_pos, first_chunk);
        memcpy(out + first_chunk, rb->buf, to_read - first_chunk);
    }

    MemoryBarrier();
    InterlockedAdd64(&rb->read_idx, (LONG64)to_read);
    return to_read;
}

/* =========================================================================
 * Auto-generate output filename - format depends on output_format setting.
 *
 * Linrad / rsp-recorder convention:
 *   YYYYMMDD_HHMMSSZ_<freq>kHz.raw
 *   e.g. 20260516_043013Z_900kHz.raw
 *
 * WavViewDX-raw convention (parsed by WavViewDX from filename):
 *   iq_pcm16_ch<n>_cf<hz>_sr<hz>_dt<YYYYMMDD>-<HHMMSS>z.raw
 *   e.g. iq_pcm16_ch1_cf900000_sr2000000_dt20260516-043013z.raw
 *
 * Both timestamps are UTC.
 * ========================================================================= */
/* Forward declaration - defined later after g_state is available */
static void get_timestamp(SYSTEMTIME *st);

static void generate_output_filename(Config *cfg, int num_channels)
{
    SYSTEMTIME st;
    get_timestamp(&st);

    if (cfg->output_format == FORMAT_WAVVIEWDX) {
        /* WavViewDX parses the filename - lowercase z in dt field is required
         * by the format specification regardless of use_utc setting.        */
        snprintf(cfg->output_file, MAX_PATH_LEN,
                 "iq_pcm16_ch%d_cf%lld_sr%lld_dt%04d%02d%02d-%02d%02d%02dz.raw",
                 num_channels,
                 (long long)cfg->frequency_hz,
                 (long long)cfg->expected_output_rate_hz,
                 st.wYear, st.wMonth, st.wDay,
                 st.wHour, st.wMinute, st.wSecond);
    } else if (cfg->output_format == FORMAT_SDRUNO || cfg->output_format == FORMAT_WINRAD) {
        /* SDRuno and Winrad share the same WAV layout and naming pattern -
         * Winrad just gets a different filename prefix so it's obvious at
         * a glance which files are RF64-capable and which are SDRuno's
         * plain-WAV split segments. Keep Z regardless of use_utc for
         * compatibility.                                                 */
        long long freq_khz = (long long)(cfg->frequency_hz / 1000.0 + 0.5);
        char freq_str[32];
        const char *prefix = (cfg->output_format == FORMAT_WINRAD) ? "Winrad" : "SDRuno";
        if (fabs(cfg->frequency_hz - (double)(freq_khz * 1000)) < 1.0)
            snprintf(freq_str, sizeof(freq_str), "%lldkHz", freq_khz);
        else
            snprintf(freq_str, sizeof(freq_str), "%.1fkHz", cfg->frequency_hz / 1000.0);
        snprintf(cfg->output_file, MAX_PATH_LEN,
                 "%s_%04d%02d%02d_%02d%02d%02dZ_%s.wav",
                 prefix, st.wYear, st.wMonth, st.wDay,
                 st.wHour, st.wMinute, st.wSecond, freq_str);
    } else if (cfg->output_format == FORMAT_SDRCONNECT) {
        /* SDR Connect WAV - no Z suffix in this format */
        long long freq_hz = (long long)(cfg->frequency_hz + 0.5);
        snprintf(cfg->output_file, MAX_PATH_LEN,
                 "SDRconnect_IQ_%04d%02d%02d_%02d%02d%02d_%lldHZ.wav",
                 st.wYear, st.wMonth, st.wDay,
                 st.wHour, st.wMinute, st.wSecond, freq_hz);
    } else {
        /* Linrad: Z suffix only when use_utc = 1 */
        long long freq_khz = (long long)(cfg->frequency_hz / 1000.0 + 0.5);
        char freq_str[32];
        if (fabs(cfg->frequency_hz - (double)(freq_khz * 1000)) < 1.0)
            snprintf(freq_str, sizeof(freq_str), "%lldkHz", freq_khz);
        else
            snprintf(freq_str, sizeof(freq_str), "%.1fkHz",
                     cfg->frequency_hz / 1000.0);
        snprintf(cfg->output_file, MAX_PATH_LEN,
                 "%04d%02d%02d_%02d%02d%02d%s_%s.raw",
                 st.wYear, st.wMonth, st.wDay,
                 st.wHour, st.wMinute, st.wSecond,
                 cfg->use_utc ? "Z" : "",
                 freq_str);
    }
}

/* Inserts a tag (e.g. "_TunerA") immediately before the file extension.
 * Used only for the dual-tuner, different-CF case where two separate
 * single-channel files are written instead of one interleaved dual file. */
static void insert_tuner_tag(char *path, size_t path_size, const char *tag)
{
    char *dot = strrchr(path, '.');
    char tmp[MAX_PATH_LEN];

    if (!dot) {
        snprintf(tmp, sizeof(tmp), "%s%s", path, tag);
    } else {
        size_t base_len = (size_t)(dot - path);
        if (base_len >= sizeof(tmp)) base_len = sizeof(tmp) - 1;
        memcpy(tmp, path, base_len);
        tmp[base_len] = '\0';
        snprintf(tmp + base_len, sizeof(tmp) - base_len, "%s%s", tag, dot);
    }
    strncpy(path, tmp, path_size - 1);
    path[path_size - 1] = '\0';
}

/* Builds cfg->output_file (Tuner A) and state->output_file_b (Tuner B) as
 * two separate, correctly-frequency-labelled single-channel filenames. */
static void generate_dual_separate_filenames(AppState *state)
{
    Config *cfg = &state->cfg;
    double freq_a_saved = cfg->frequency_hz;

    /* File B first: temporarily borrow the generator with Tuner B's own
     * frequency, then stash the result and restore Tuner A's frequency. */
    cfg->frequency_hz = cfg->freq_b_hz;
    generate_output_filename(cfg, 1);
    strncpy(state->output_file_b, cfg->output_file, MAX_PATH_LEN - 1);
    state->output_file_b[MAX_PATH_LEN - 1] = '\0';
    insert_tuner_tag(state->output_file_b, sizeof(state->output_file_b), "_TunerB");
    cfg->frequency_hz = freq_a_saved;

    /* File A: normal generation, using Tuner A's own (real) frequency. */
    generate_output_filename(cfg, 1);
    insert_tuner_tag(cfg->output_file, sizeof(cfg->output_file), "_TunerA");
}


/* =========================================================================
 * Linrad raw file header writer
 *
 * Format documented by Franco Venturi (rsp-recorder author), confirmed on
 * the Linrad mailing list Jan 2023:
 *
 *   int    remember_proprietary_chunk;  // always -1 (REMEMBER_UNKNOWN)
 *   double timestamp;                   // seconds since 1970-01-01 00:00:00 UTC
 *   double passband_center;             // centre frequency in MHz
 *   int    passband_direction;          // always 0
 *   int    rx_input_mode;               // always 0
 *   int    rx_rf_channels;              // 1=single, 2=dual
 *   int    rx_ad_channels;              // 2=single (I,Q), 4=dual (I1,Q1,I2,Q2)
 *   int    rx_ad_speed;                 // sample rate in samples/s
 *   uchar  save_init_flag;              // always 0
 *
 * Total: 4 + 8 + 8 + 4 + 4 + 4 + 4 + 4 + 1 = 41 bytes
 * ========================================================================= */
#pragma pack(push, 1)
typedef struct {
    int32_t  remember_proprietary_chunk; /* always -1 */
    double   timestamp;                  /* Unix UTC epoch (seconds) */
    double   passband_center;            /* MHz */
    int32_t  passband_direction;         /* 0 */
    int32_t  rx_input_mode;              /* 0 */
    int32_t  rx_rf_channels;            /* 1 or 2 */
    int32_t  rx_ad_channels;            /* 2 or 4 */
    int32_t  rx_ad_speed;               /* sample rate Hz */
    uint8_t  save_init_flag;            /* 0 */
} LinradRawHeader;                       /* 41 bytes */
#pragma pack(pop)

static int write_linrad_header(HANDLE fh, const Config *cfg, int num_channels,
                                double passband_override_hz)
{
    LinradRawHeader hdr;
    DWORD written;
    BOOL ok;
    FILETIME ft;
    ULARGE_INTEGER ull;
    double unix_ts;

    /* Zero all fields first - struct is on the stack and C does not
     * initialise local variables. Any field we forget to set explicitly
     * must be zero, not garbage. */
    memset(&hdr, 0, sizeof(hdr));

    /* Build the 41-byte header */
    hdr.remember_proprietary_chunk = -1;  /* REMEMBER_UNKNOWN */

    /* Windows FILETIME -> Unix timestamp:
     * FILETIME is 100-nanosecond intervals since 1601-01-01.
     * Unix epoch starts 1970-01-01, offset = 116444736000000000 intervals. */
    GetSystemTimeAsFileTime(&ft);
    ull.LowPart  = ft.dwLowDateTime;
    ull.HighPart = ft.dwHighDateTime;
    unix_ts = (double)((int64_t)ull.QuadPart - 116444736000000000LL) / 1.0e7;

    hdr.timestamp          = unix_ts;
    /* passband_center: RF centre frequency in MHz.
     * WavViewDX reads this as the display centre frequency.           */
    hdr.passband_center    = (passband_override_hz > 0.0)
                                  ? passband_override_hz / 1.0e6
                                  : cfg->frequency_hz / 1.0e6;
    hdr.passband_direction = 1;  /* 1=normal, -1=inverted. Must not be 0. */
    /* rx_input_mode = 0x26 is a constant used by rsp-recorder for all
     * SDRplay recordings regardless of device or mode.
     * rx_rf_channels and rx_ad_channels scale with actual channel count:
     *   Single channel: rx_rf_channels=1, rx_ad_channels=2
     *   Dual channel:   rx_rf_channels=2, rx_ad_channels=4
     * WavViewDX uses rx_ad_channels to calculate file duration:
     *   duration = file_size / (rx_ad_speed * rx_ad_channels * 2) */
    hdr.rx_input_mode      = 0x26;
    hdr.rx_rf_channels     = num_channels;
    hdr.rx_ad_channels     = num_channels * 2;
    /* Use the expected output rate (after hardware decimation), not the
     * ADC rate. For IF=1620/SR=6Msps this is 2000000, not 6000000.  */
    hdr.rx_ad_speed        = (int32_t)cfg->expected_output_rate_hz;
    hdr.save_init_flag     = 0;

    if (g_state.cfg.verbose)
        LOG_INFO("Writing Linrad 41-byte header: passband_center=%.6f MHz  "
             "SR=%d sps  CH=%d",
             hdr.passband_center, hdr.rx_ad_speed, hdr.rx_rf_channels);

    ok = WriteFile(fh, &hdr, (DWORD)sizeof(LinradRawHeader), &written, NULL);
    if (!ok || written != (DWORD)sizeof(LinradRawHeader)) {
        LOG_ERROR("Header write failed: wrote %lu of %zu bytes, error %lu",
                  written, sizeof(LinradRawHeader), GetLastError());
        return 0;
    }
    return 1;
}

/* =========================================================================
 * SDRuno WAV header (216 bytes):
 *   RIFF(8) + fmt (8+16) + auxi(8+164) + data(8)
 * auxi: StartTime(16) + StopTime(16) + CentreFreqHz(4) + zeros(128)
 *
 * SDR Connect WAV header (80 bytes):
 *   RIFF(8) + JUNK(8+28) + fmt (8+16) + data(8)
 *   Centre freq and timestamp in filename: SDRconnect_IQ_*_<hz>HZ.wav
 * ========================================================================= */
#pragma pack(push,1)
typedef struct {
    uint16_t wYear, wMonth, wDayOfWeek, wDay;
    uint16_t wHour, wMinute, wSecond, wMilliseconds;
} WavSystime;  /* 16 bytes */

typedef struct {
    char riff_id[4]; uint32_t riff_size; char wave_id[4];
    char fmt_id[4];  uint32_t fmt_size;
    uint16_t audio_format, num_channels;
    uint32_t sample_rate, byte_rate;
    uint16_t block_align, bits_per_sample;
    char auxi_id[4]; uint32_t auxi_size;
    WavSystime start_time, stop_time;
    uint32_t centre_freq_hz;
    uint8_t  auxi_pad[128];
    char data_id[4]; uint32_t data_size;
} SDRunoHeader;   /* 216 bytes */

typedef struct {
    char riff_id[4]; uint32_t riff_size; char wave_id[4];
    char junk_id[4]; uint32_t junk_size; uint8_t junk_pad[28];
    char fmt_id[4];  uint32_t fmt_size;
    uint16_t audio_format, num_channels;
    uint32_t sample_rate, byte_rate;
    uint16_t block_align, bits_per_sample;
    char data_id[4]; uint32_t data_size;
} SDRConnectHeader;  /* 80 bytes */

/* RF64 variant of the SDRuno header (252 bytes) - EBU Tech 3306 framing.
 * "RF64" replaces "RIFF" as the outer id, riff_size and data_size are
 * both permanently set to the 0xFFFFFFFF sentinel (per spec - it tells a
 * reader "see the ds64 chunk instead"), and a ds64 chunk carrying real
 * 64-bit sizes is inserted right after the WAVE id, before fmt. Layout
 * otherwise matches SDRunoHeader (same fmt/auxi content) so the audio
 * data itself is byte-identical to the plain-WAV version.                */
typedef struct {
    char riff_id[4]; uint32_t riff_size; char wave_id[4];   /* "RF64", 0xFFFFFFFF, "WAVE" */
    char ds64_id[4]; uint32_t ds64_size;                     /* "ds64", 28                 */
    uint64_t riff_size64, data_size64, sample_count64;
    uint32_t table_length;                                   /* 0 - no extra table entries */
    char fmt_id[4];  uint32_t fmt_size;
    uint16_t audio_format, num_channels;
    uint32_t sample_rate, byte_rate;
    uint16_t block_align, bits_per_sample;
    char auxi_id[4]; uint32_t auxi_size;
    WavSystime start_time, stop_time;
    uint32_t centre_freq_hz;
    uint8_t  auxi_pad[128];
    char data_id[4]; uint32_t data_size;                     /* 0xFFFFFFFF sentinel        */
} SDRunoRF64Header;  /* 252 bytes */

/* RF64 variant of the SDR Connect header (116 bytes). Same reasoning as
 * SDRunoRF64Header above - the ds64 chunk must come immediately after the
 * WAVE id per the RF64 spec, ahead of every other chunk including this
 * format's JUNK padding, so it's inserted there rather than at the end.  */
typedef struct {
    char riff_id[4]; uint32_t riff_size; char wave_id[4];   /* "RF64", 0xFFFFFFFF, "WAVE" */
    char ds64_id[4]; uint32_t ds64_size;                     /* "ds64", 28                 */
    uint64_t riff_size64, data_size64, sample_count64;
    uint32_t table_length;
    char junk_id[4]; uint32_t junk_size; uint8_t junk_pad[28];
    char fmt_id[4];  uint32_t fmt_size;
    uint16_t audio_format, num_channels;
    uint32_t sample_rate, byte_rate;
    uint16_t block_align, bits_per_sample;
    char data_id[4]; uint32_t data_size;                     /* 0xFFFFFFFF sentinel        */
} SDRConnectRF64Header;  /* 116 bytes */
#pragma pack(pop)

/* =========================================================================
 * SDRuno WAV header writer (216 bytes)
 * ========================================================================= */
static int write_sdruno_header(HANDLE fh, const Config *cfg)
{
    SDRunoHeader hdr; DWORD written; SYSTEMTIME st; get_timestamp(&st);
    memset(&hdr, 0, sizeof(hdr));
    memcpy(hdr.riff_id, "RIFF", 4);  hdr.riff_size = 0;
    memcpy(hdr.wave_id, "WAVE", 4);
    memcpy(hdr.fmt_id,  "fmt ", 4);  hdr.fmt_size  = 16;
    hdr.audio_format = 1;  hdr.num_channels = 2;
    hdr.sample_rate  = (uint32_t)cfg->expected_output_rate_hz;
    hdr.byte_rate    = hdr.sample_rate * 4;
    hdr.block_align  = 4;  hdr.bits_per_sample = 16;
    memcpy(hdr.auxi_id, "auxi", 4);  hdr.auxi_size = 164;
    hdr.start_time.wYear         = st.wYear;
    hdr.start_time.wMonth        = st.wMonth;
    hdr.start_time.wDayOfWeek    = st.wDayOfWeek;
    hdr.start_time.wDay          = st.wDay;
    hdr.start_time.wHour         = st.wHour;
    hdr.start_time.wMinute       = st.wMinute;
    hdr.start_time.wSecond       = st.wSecond;
    hdr.start_time.wMilliseconds = st.wMilliseconds;
    hdr.centre_freq_hz = (uint32_t)(cfg->frequency_hz + 0.5);
    memcpy(hdr.data_id, "data", 4);  hdr.data_size = 0;
    if (g_state.cfg.verbose)
        LOG_INFO("Writing SDRuno WAV header: SR=%u Hz  CF=%u Hz",
             hdr.sample_rate, hdr.centre_freq_hz);
    if (!WriteFile(fh, &hdr, (DWORD)sizeof(hdr), &written, NULL)
            || written != (DWORD)sizeof(hdr)) {
        LOG_ERROR("SDRuno header write failed: %lu bytes, error %lu",
                  written, GetLastError());
        return 0;
    }
    return 1;
}

/* =========================================================================
 * SDRuno RF64 header writer (252 bytes) - single-file alternative to the
 * split-at-4GB mode, for recordings expected to exceed the plain WAV
 * 4 GiB data limit. riff_size/data_size are written as 0xFFFFFFFF and the
 * ds64 chunk's real sizes are left at 0 here, to be patched in once the
 * actual sample count is known (see patch_wav_sizes_rf64()).             */
static int write_sdruno_header_rf64(HANDLE fh, const Config *cfg)
{
    SDRunoRF64Header hdr; DWORD written; SYSTEMTIME st; get_timestamp(&st);
    memset(&hdr, 0, sizeof(hdr));
    memcpy(hdr.riff_id, "RF64", 4);  hdr.riff_size = 0xFFFFFFFFu;
    memcpy(hdr.wave_id, "WAVE", 4);
    memcpy(hdr.ds64_id, "ds64", 4);  hdr.ds64_size = 28;
    hdr.riff_size64 = 0;  hdr.data_size64 = 0;  hdr.sample_count64 = 0;
    hdr.table_length = 0;
    memcpy(hdr.fmt_id,  "fmt ", 4);  hdr.fmt_size  = 16;
    hdr.audio_format = 1;  hdr.num_channels = 2;
    hdr.sample_rate  = (uint32_t)cfg->expected_output_rate_hz;
    hdr.byte_rate    = hdr.sample_rate * 4;
    hdr.block_align  = 4;  hdr.bits_per_sample = 16;
    memcpy(hdr.auxi_id, "auxi", 4);  hdr.auxi_size = 164;
    hdr.start_time.wYear         = st.wYear;
    hdr.start_time.wMonth        = st.wMonth;
    hdr.start_time.wDayOfWeek    = st.wDayOfWeek;
    hdr.start_time.wDay          = st.wDay;
    hdr.start_time.wHour         = st.wHour;
    hdr.start_time.wMinute       = st.wMinute;
    hdr.start_time.wSecond       = st.wSecond;
    hdr.start_time.wMilliseconds = st.wMilliseconds;
    hdr.centre_freq_hz = (uint32_t)(cfg->frequency_hz + 0.5);
    memcpy(hdr.data_id, "data", 4);  hdr.data_size = 0xFFFFFFFFu;
    if (g_state.cfg.verbose)
        LOG_INFO("Writing SDRuno RF64 header: SR=%u Hz  CF=%u Hz",
             hdr.sample_rate, hdr.centre_freq_hz);
    if (!WriteFile(fh, &hdr, (DWORD)sizeof(hdr), &written, NULL)
            || written != (DWORD)sizeof(hdr)) {
        LOG_ERROR("SDRuno RF64 header write failed: %lu bytes, error %lu",
                  written, GetLastError());
        return 0;
    }
    return 1;
}

/* =========================================================================
 * SDR Connect WAV header writer (80 bytes)
 * ========================================================================= */
static int write_sdrconnect_header(HANDLE fh, const Config *cfg)
{
    SDRConnectHeader hdr; DWORD written;
    memset(&hdr, 0, sizeof(hdr));
    memcpy(hdr.riff_id, "RIFF", 4);  hdr.riff_size = 0;
    memcpy(hdr.wave_id, "WAVE", 4);
    memcpy(hdr.junk_id, "JUNK", 4);  hdr.junk_size = 28;
    memcpy(hdr.fmt_id,  "fmt ", 4);  hdr.fmt_size  = 16;
    hdr.audio_format = 1;  hdr.num_channels = 2;
    hdr.sample_rate  = (uint32_t)cfg->expected_output_rate_hz;
    hdr.byte_rate    = hdr.sample_rate * 4;
    hdr.block_align  = 4;  hdr.bits_per_sample = 16;
    memcpy(hdr.data_id, "data", 4);  hdr.data_size = 0;
    if (g_state.cfg.verbose)
        LOG_INFO("Writing SDR Connect WAV header: SR=%u Hz  CF=%.0f Hz",
             hdr.sample_rate, cfg->frequency_hz);
    if (!WriteFile(fh, &hdr, (DWORD)sizeof(hdr), &written, NULL)
            || written != (DWORD)sizeof(hdr)) {
        LOG_ERROR("SDR Connect header write failed: %lu bytes, error %lu",
                  written, GetLastError());
        return 0;
    }
    return 1;
}

/* =========================================================================
 * SDR Connect RF64 header writer (116 bytes) - single-file alternative to
 * split-at-4GB mode for SDR Connect recordings expected to exceed the
 * plain WAV 4 GiB data limit. Same pattern as write_sdruno_header_rf64().
 * ========================================================================= */
static int write_sdrconnect_header_rf64(HANDLE fh, const Config *cfg)
{
    SDRConnectRF64Header hdr; DWORD written;
    memset(&hdr, 0, sizeof(hdr));
    memcpy(hdr.riff_id, "RF64", 4);  hdr.riff_size = 0xFFFFFFFFu;
    memcpy(hdr.wave_id, "WAVE", 4);
    memcpy(hdr.ds64_id, "ds64", 4);  hdr.ds64_size = 28;
    hdr.riff_size64 = 0;  hdr.data_size64 = 0;  hdr.sample_count64 = 0;
    hdr.table_length = 0;
    memcpy(hdr.junk_id, "JUNK", 4);  hdr.junk_size = 28;
    memcpy(hdr.fmt_id,  "fmt ", 4);  hdr.fmt_size  = 16;
    hdr.audio_format = 1;  hdr.num_channels = 2;
    hdr.sample_rate  = (uint32_t)cfg->expected_output_rate_hz;
    hdr.byte_rate    = hdr.sample_rate * 4;
    hdr.block_align  = 4;  hdr.bits_per_sample = 16;
    memcpy(hdr.data_id, "data", 4);  hdr.data_size = 0xFFFFFFFFu;
    if (g_state.cfg.verbose)
        LOG_INFO("Writing SDR Connect RF64 header: SR=%u Hz  CF=%.0f Hz",
             hdr.sample_rate, cfg->frequency_hz);
    if (!WriteFile(fh, &hdr, (DWORD)sizeof(hdr), &written, NULL)
            || written != (DWORD)sizeof(hdr)) {
        LOG_ERROR("SDR Connect RF64 header write failed: %lu bytes, error %lu",
                  written, GetLastError());
        return 0;
    }
    return 1;
}

/* =========================================================================
 * RIFF size patcher -- seeks back and updates riff_size and data_size.
 * Called before CloseHandle for SDRuno and SDR Connect formats.
 * Both fields were written as 0 in the initial header.
 * ========================================================================= */
static void patch_wav_sizes(HANDLE fh, const Config *cfg, LONG64 samples_written)
{
    int64_t  data_bytes  = samples_written * 4;
    int64_t  header_size = (cfg->output_format == FORMAT_SDRUNO ||
                             cfg->output_format == FORMAT_WINRAD) ? 216 : 80;
    int64_t  riff_total  = header_size - 8 + data_bytes;
    /* Clamp to 0xFFFFFFFF rather than wrapping mod 2^32 - this matches
     * genuine SDRuno's own behaviour for oversized files (confirmed via
     * SDR Trim's write_wav_header(), which was built against real SDRuno
     * output). In current DuoDX usage this path should never actually see
     * an oversized value - split mode rolls the file over well before 4
     * GiB - but the clamp is correct defensive behaviour regardless.     */
    uint32_t riff_size32 = (riff_total  > 0xFFFFFFFFLL) ? 0xFFFFFFFFu : (uint32_t)riff_total;
    uint32_t data_size32 = (data_bytes  > 0xFFFFFFFFLL) ? 0xFFFFFFFFu : (uint32_t)data_bytes;
    DWORD written; LARGE_INTEGER li;
    li.QuadPart = 4;
    if (SetFilePointerEx(fh, li, NULL, FILE_BEGIN))
        WriteFile(fh, &riff_size32, 4, &written, NULL);
    li.QuadPart = header_size - 4;
    if (SetFilePointerEx(fh, li, NULL, FILE_BEGIN))
        WriteFile(fh, &data_size32, 4, &written, NULL);
    LOG_INFO("WAV sizes patched: riff_size=%u  data_size=%u", riff_size32, data_size32);

    /* Patch the auxi chunk's StopTime field (SDRuno/Winrad only - SDR
     * Connect's header has no auxi chunk). This was never being written
     * at all before now: every DuoDX SDRuno/Winrad file left StopTime at
     * all-zero from the initial memset in write_sdruno_header(). Real
     * SDRuno files always carry a genuine StopTime here (confirmed from
     * two actual SDRuno recordings), and SDR Console's recordings browser
     * appears to compute displayed duration from StopTime-StartTime
     * rather than from data_size - with StopTime stuck at zero (an
     * "invalid"/epoch-like date well before StartTime), that subtraction
     * produces exactly the large negative garbage duration being seen.   */
    if (cfg->output_format == FORMAT_SDRUNO || cfg->output_format == FORMAT_WINRAD) {
        SYSTEMTIME st;
        WavSystime stop_time;
        get_timestamp(&st);
        stop_time.wYear         = st.wYear;
        stop_time.wMonth        = st.wMonth;
        stop_time.wDayOfWeek    = st.wDayOfWeek;
        stop_time.wDay          = st.wDay;
        stop_time.wHour         = st.wHour;
        stop_time.wMinute       = st.wMinute;
        stop_time.wSecond       = st.wSecond;
        stop_time.wMilliseconds = st.wMilliseconds;
        li.QuadPart = offsetof(SDRunoHeader, stop_time);
        if (SetFilePointerEx(fh, li, NULL, FILE_BEGIN))
            WriteFile(fh, &stop_time, sizeof(stop_time), &written, NULL);
        LOG_INFO("StopTime patched: %04d-%02d-%02d %02d:%02d:%02d",
                 stop_time.wYear, stop_time.wMonth, stop_time.wDay,
                 stop_time.wHour, stop_time.wMinute, stop_time.wSecond);
    }
}

/* =========================================================================
 * RF64 size patcher -- seeks back and updates the ds64 chunk's 64-bit
 * riff/data/sample-count fields. riff_size and data_size at the fixed
 * 32-bit offsets stay at the 0xFFFFFFFF sentinel written at open time -
 * that is what tells an RF64-aware reader to look at ds64 instead, so
 * unlike patch_wav_sizes() those two fields are never touched here.
 * ========================================================================= */
static void patch_wav_sizes_rf64(HANDLE fh, const Config *cfg, LONG64 samples_written)
{
    int64_t  data_bytes    = samples_written * 4;
    int64_t  header_size   = (cfg->output_format == FORMAT_SDRCONNECT)
                              ? (int64_t)sizeof(SDRConnectRF64Header)
                              : (int64_t)sizeof(SDRunoRF64Header); /* also covers Winrad */
    uint64_t riff_size64   = (uint64_t)(header_size - 8 + data_bytes);
    uint64_t data_size64   = (uint64_t)data_bytes;
    uint64_t sample_count64 = (uint64_t)samples_written;
    DWORD written; LARGE_INTEGER li;

    /* The ds64 chunk sits at identical byte offsets in both RF64 header
     * variants (same fields, same order, immediately after WAVE id in
     * both), guaranteed at compile time just below - so SDRunoRF64Header's
     * offsets are reused for SDR Connect too rather than duplicating this
     * function per format.                                               */
    li.QuadPart = offsetof(SDRunoRF64Header, riff_size64);
    if (SetFilePointerEx(fh, li, NULL, FILE_BEGIN))
        WriteFile(fh, &riff_size64, 8, &written, NULL);
    li.QuadPart = offsetof(SDRunoRF64Header, data_size64);
    if (SetFilePointerEx(fh, li, NULL, FILE_BEGIN))
        WriteFile(fh, &data_size64, 8, &written, NULL);
    li.QuadPart = offsetof(SDRunoRF64Header, sample_count64);
    if (SetFilePointerEx(fh, li, NULL, FILE_BEGIN))
        WriteFile(fh, &sample_count64, 8, &written, NULL);
    LOG_INFO("RF64 ds64 sizes patched: riff_size64=%llu  data_size64=%llu",
             (unsigned long long)riff_size64, (unsigned long long)data_size64);

    /* The classic 32-bit riff_size/data_size fields were written as the
     * RF64 spec's 0xFFFFFFFF sentinel at file-open time (real size wasn't
     * known yet). Now that it is, patch in the REAL value whenever it
     * actually fits in 32 bits - i.e. whenever this particular recording
     * never grew past ~4 GiB, which is the common case even in RF64 mode
     * (RF64 just means "won't be truncated IF it grows that large", not
     * that every recording does). Tools that don't implement true RF64
     * ds64 lookup and instead read these classic fields directly - which
     * is what appears to be causing corrupted/negative duration values in
     * SDR Console's recordings browser for these files - will then see an
     * entirely ordinary, correct WAV file. Only genuinely oversized files
     * keep the sentinel, since the real value can't fit in 32 bits there
     * regardless of what we do - a reader that can't handle RF64 simply
     * can't be told the true size of a file that large in any format.    */
    if (riff_size64 <= 0xFFFFFFFEULL && data_size64 <= 0xFFFFFFFEULL) {
        uint32_t riff_size32 = (uint32_t)riff_size64;
        uint32_t data_size32 = (uint32_t)data_size64;
        li.QuadPart = 4; /* riff_size32, right after the 4-byte "RF64" id */
        if (SetFilePointerEx(fh, li, NULL, FILE_BEGIN))
            WriteFile(fh, &riff_size32, 4, &written, NULL);
        li.QuadPart = header_size - 4; /* data_size32, right before audio data */
        if (SetFilePointerEx(fh, li, NULL, FILE_BEGIN))
            WriteFile(fh, &data_size32, 4, &written, NULL);
        LOG_INFO("RF64 classic 32-bit fields also patched with real values "
                 "(file stayed under 4GB): riff_size=%u  data_size=%u",
                 riff_size32, data_size32);
    }

    /* Same StopTime fix as patch_wav_sizes() - Winrad's auxi chunk had
     * never had StopTime written, only SDR Connect RF64 has no auxi chunk
     * to patch here (SDRuno itself never reaches this RF64 patcher at all
     * - see the OutputFormat enum comment).                              */
    if (cfg->output_format == FORMAT_WINRAD) {
        SYSTEMTIME st;
        WavSystime stop_time;
        get_timestamp(&st);
        stop_time.wYear         = st.wYear;
        stop_time.wMonth        = st.wMonth;
        stop_time.wDayOfWeek    = st.wDayOfWeek;
        stop_time.wDay          = st.wDay;
        stop_time.wHour         = st.wHour;
        stop_time.wMinute       = st.wMinute;
        stop_time.wSecond       = st.wSecond;
        stop_time.wMilliseconds = st.wMilliseconds;
        li.QuadPart = offsetof(SDRunoRF64Header, stop_time);
        if (SetFilePointerEx(fh, li, NULL, FILE_BEGIN))
            WriteFile(fh, &stop_time, sizeof(stop_time), &written, NULL);
        LOG_INFO("StopTime patched: %04d-%02d-%02d %02d:%02d:%02d",
                 stop_time.wYear, stop_time.wMonth, stop_time.wDay,
                 stop_time.wHour, stop_time.wMinute, stop_time.wSecond);
    }
}

/* Compile-time guard for the offset reuse above: if either RF64 header
 * struct's ds64 field layout ever changes so the two no longer match,
 * this fails the build instead of silently corrupting SDR Connect RF64
 * headers with SDRuno's offsets (or vice versa).                        */
typedef char ds64_offsets_must_match_riff
    [(offsetof(SDRunoRF64Header, riff_size64) == offsetof(SDRConnectRF64Header, riff_size64)) ? 1 : -1];
typedef char ds64_offsets_must_match_data
    [(offsetof(SDRunoRF64Header, data_size64) == offsetof(SDRConnectRF64Header, data_size64)) ? 1 : -1];
typedef char ds64_offsets_must_match_sampcount
    [(offsetof(SDRunoRF64Header, sample_count64) == offsetof(SDRConnectRF64Header, sample_count64)) ? 1 : -1];

/* Dispatch wrapper - picks the plain-WAV or RF64 size patcher depending on
 * output format/mode, so call sites don't need to know the difference.
 * SDRuno is deliberately excluded from the RF64 branch - it never sets
 * large_file_mode to LARGE_FILE_RF64 (Settings hides that choice for it),
 * but excluding it here too means a stale/hand-edited ini value can't
 * accidentally produce an RF64 file SDRuno itself can't play back.       */
static void finalize_output_header(HANDLE fh, const Config *cfg, LONG64 samples_written)
{
    if ((cfg->output_format == FORMAT_WINRAD || cfg->output_format == FORMAT_SDRCONNECT)
            && cfg->large_file_mode == LARGE_FILE_RF64)
        patch_wav_sizes_rf64(fh, cfg, samples_written);
    else
        patch_wav_sizes(fh, cfg, samples_written);
}

/* =========================================================================
 * Writer thread - drains the ring buffer to disk
 *
 * Runs at THREAD_PRIORITY_HIGHEST.
 * All disk I/O and error checking happens here, never in the callback.
 * ========================================================================= */
#define WRITE_CHUNK_SIZE (256 * 1024)   /* 256 KB per write call */

static uint8_t g_write_chunk[WRITE_CHUNK_SIZE];
static uint8_t g_zero_chunk[WRITE_CHUNK_SIZE]; /* Always zero - for gap fill */
/* De-interleave buffers for dual_separate_files mode: each dual 8-byte
 * frame (IA,QA,IB,QB) splits into two 4-byte single-channel frames. */
static uint8_t g_split_chunk_a[WRITE_CHUNK_SIZE / 2];
static uint8_t g_split_chunk_b[WRITE_CHUNK_SIZE / 2];

/* -------------------------------------------------------------------------
 * write_gap_fill
 *
 * Called by the writer thread when ring buffer overflows have occurred.
 * Writes 'bytes_to_fill' bytes of zeroes to the output file, aligned to
 * the sample frame size, to preserve correct time synchronisation.
 *
 * A zero I/Q sample (0, 0) represents silence in the recording - the
 * gap appears as a silent section in WavViewDX rather than a time shift.
 * -------------------------------------------------------------------------*/
static int write_gap_fill(AppState *state, LONG64 bytes_to_fill)
{
    int    frame_size = (state->cfg.dual_channel ? 8 : 4);
    LONG64 remaining;
    DWORD  written;
    BOOL   ok;
    LONG64 chunk;

    /* Align to frame boundary */
    bytes_to_fill = (bytes_to_fill / frame_size) * frame_size;
    if (bytes_to_fill <= 0)
        return 1;

    LOG_WARN("Writing %lld bytes of zero-fill to preserve time sync "
             "(%lld dropped sample frames)",
             bytes_to_fill, bytes_to_fill / frame_size);

    remaining = bytes_to_fill;
    while (remaining > 0) {
        chunk = remaining;
        if (chunk > (LONG64)WRITE_CHUNK_SIZE)
            chunk = (LONG64)WRITE_CHUNK_SIZE;

        if (state->dual_separate_files) {
            /* g_zero_chunk is all zero regardless of how it's sliced, so
             * chunk/2 bytes (frame_size 8 -> always even) go to each file. */
            DWORD half = (DWORD)(chunk / 2);
            ok = WriteFile(state->out_file, g_zero_chunk, half, &written, NULL);
            if (!ok || written != half) {
                LOG_ERROR("Gap fill WriteFile (A) failed: error %lu", GetLastError());
                return 0;
            }
            ok = WriteFile(state->out_file_b, g_zero_chunk, half, &written, NULL);
            if (!ok || written != half) {
                LOG_ERROR("Gap fill WriteFile (B) failed: error %lu", GetLastError());
                return 0;
            }
        } else {
            ok = WriteFile(state->out_file, g_zero_chunk, (DWORD)chunk,
                           &written, NULL);
            if (!ok || written != (DWORD)chunk) {
                LOG_ERROR("Gap fill WriteFile failed: error %lu", GetLastError());
                return 0;
            }
        }

        /* Count zero frames separately so we can report them distinctly.
         * They are also added to samples_written because they occupy
         * real time slots in the recording timeline. */
        InterlockedAdd64(&state->zero_frames_written,
                         (LONG64)(chunk / frame_size));
        InterlockedAdd64(&state->samples_written,
                         (LONG64)(chunk / frame_size));
        InterlockedAdd64(&state->segment_samples_written,
                         (LONG64)(chunk / frame_size));
        remaining -= chunk;
    }

    return 1;
}

/* Threshold comfortably under the 4 GiB (4,294,967,296 byte) WAV limit,
 * chosen as a round number divisible by both 4-byte (single-tuner) and
 * 8-byte (dual-tuner interleaved) sample frames.                        */
#define SPLIT_THRESHOLD_BYTES 4000000000ULL

/* =========================================================================
 * Split mode: once the currently-open file's data would cross the split
 * threshold, finalize it (patch its now-known size) and open a new file
 * to continue into, timestamped at the moment of rollover - this matches
 * the naming convention genuine SDRuno/SDR Connect recordings use when
 * they split a long capture: each segment gets its own current-time
 * filename (e.g. SDRuno_20260518_090743Z_1125kHz.wav next to
 * SDRuno_20260518_085324Z_1125kHz.wav), not a "_partN" suffix on a shared
 * name. Called for FORMAT_SDRUNO unconditionally (it always splits - see
 * the OutputFormat enum comment), and for FORMAT_WINRAD/FORMAT_SDRCONNECT
 * when large_file_mode==LARGE_FILE_SPLIT; RF64 mode (Winrad/SDR Connect
 * only) never rolls over, it just keeps growing one file.
 * ========================================================================= */
static void check_split_rollover(AppState *state)
{
    int64_t seg_bytes = state->segment_samples_written * 4;
    HANDLE new_file;
    int hdr_ok;

    if ((uint64_t)seg_bytes < SPLIT_THRESHOLD_BYTES)
        return;

    /* Finalize the segment that's about to close. */
    FlushFileBuffers(state->out_file);
    finalize_output_header(state->out_file, &state->cfg,
                            state->segment_samples_written);
    CloseHandle(state->out_file);
    LOG_OK("Part %d complete (%lld samples) - rolling over to a new file.",
           state->output_part_number, (long long)state->segment_samples_written);

    state->output_part_number++;
    /* num_channels is only consulted for FORMAT_WAVVIEWDX inside this
     * function - irrelevant here since split mode is SDRuno/SDR Connect
     * only, so the placeholder value has no effect on the name produced. */
    generate_output_filename(&state->cfg, 1);

    new_file = CreateFileA(state->cfg.output_file, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS,
                            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_SEQUENTIAL_SCAN, NULL);
    if (new_file == INVALID_HANDLE_VALUE) {
        LOG_ERROR("Split mode: cannot open '%s' (error %lu) - recording will stop.",
                  state->cfg.output_file, GetLastError());
        state->out_file = INVALID_HANDLE_VALUE;
        state->writer_error = 1;
        return;
    }

    hdr_ok = (state->cfg.output_format == FORMAT_SDRCONNECT)
             ? write_sdrconnect_header(new_file, &state->cfg)
             : write_sdruno_header(new_file, &state->cfg);
    if (!hdr_ok) {
        LOG_ERROR("Split mode: header write failed for '%s' - recording will stop.",
                  state->cfg.output_file);
        CloseHandle(new_file);
        state->out_file = INVALID_HANDLE_VALUE;
        state->writer_error = 1;
        return;
    }
    FlushFileBuffers(new_file);

    state->out_file = new_file;
    state->segment_samples_written = 0;
    LOG_OK("Part %d started: %s", state->output_part_number, state->cfg.output_file);
}

static DWORD WINAPI writer_thread_func(LPVOID param)
{
    AppState *state = (AppState *)param;
    RingBuffer *rb  = &state->ring;
    SIZE_T to_read;
    DWORD written;
    BOOL ok;
    LONG cur_overflow;
    LONG last_overflow = 0;
    LONG64 last_bytes_dropped = 0;

    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
    SetEvent(state->writer_ready_event);

    while (state->writer_running || ring_available(rb) > 0) {
        to_read = ring_available(rb);

        /* ---------------------------------------------------------------
         * Before writing real samples, check whether any bytes were
         * dropped since our last pass. If so, write zero-fill frames to
         * the output file to preserve time synchronisation.
         * --------------------------------------------------------------- */
        {
            LONG64 current_dropped = rb->bytes_dropped;
            if (current_dropped != last_bytes_dropped) {
                LONG64 new_dropped = current_dropped - last_bytes_dropped;
                last_bytes_dropped = current_dropped;

                /* Log overflow count for the monitor */
                cur_overflow = rb->overflow;
                if (cur_overflow != last_overflow) {
                    InterlockedAdd(&state->overflows,
                                   cur_overflow - last_overflow);
                    last_overflow = cur_overflow;

                    {
                        LARGE_INTEGER now_of;
                        QueryPerformanceCounter(&now_of);
                        double el_of = (double)(now_of.QuadPart
                                        - state->start_time.QuadPart)
                                       / (double)state->perf_freq.QuadPart;
                        int of_h  = (int)el_of / 3600;
                        int of_m  = ((int)el_of % 3600) / 60;
                        int of_s  = (int)el_of % 60;
                        LOG_WARN("Ring buffer overflow #%ld at [%02d:%02d:%02d] "
                                 "- %lld bytes dropped (%.1f ms lost)",
                                 (long)state->overflows,
                                 of_h, of_m, of_s,
                                 (long long)new_dropped,
                                 (double)new_dropped
                                 / (state->cfg.expected_output_rate_hz
                                    * (state->cfg.dual_channel ? 8.0 : 4.0))
                                 * 1000.0);
                    }
                }

                /* Write compensating zeros */
                if (!write_gap_fill(state, new_dropped)) {
                    state->writer_error = 1;
                    break;
                }
            }
        }

        if (to_read == 0) {
            /* Nothing to write - short sleep to avoid busy-waiting */
            Sleep(1);
            continue;
        }

        /* Cap at chunk size */
        if (to_read > WRITE_CHUNK_SIZE)
            to_read = WRITE_CHUNK_SIZE;

        /* Align to sample frame boundary:
         * Single channel: 4 bytes (I + Q, 2 bytes each)
         * Dual channel:   8 bytes (IA + QA + IB + QB)
         */
        {
            int frame_size = (state->cfg.dual_channel ? 8 : 4);
            to_read = (to_read / frame_size) * frame_size;
            if (to_read == 0) {
                Sleep(1);
                continue;
            }
        }

        to_read = ring_read(rb, g_write_chunk, to_read);
        if (to_read == 0)
            continue;

        /* ---------------------------------------------------------------
         * Disk space check.
         * GetDiskFreeSpaceExA requires a directory path, not a file path.
         * We extract the directory from output_file by copying up to and
         * including the last backslash. If no backslash, use "." (cwd).
         * Check frequency adapts: every 16 chunks when space is ample,
         * every 4 chunks once the 500 MB warning has fired.
         * --------------------------------------------------------------- */
        {
            static int disk_check_counter = 0;
            int check_interval = state->disk_warn_issued ? 4 : 16;
            if (++disk_check_counter >= check_interval) {
                ULARGE_INTEGER free_bytes;
                char dir_path[MAX_PATH] = ".";
                disk_check_counter = 0;

                /* Extract directory portion of the output path */
                {
                    const char *last_sep = NULL;
                    const char *p = state->cfg.output_file;
                    for (; *p; p++)
                        if (*p == '\\' || *p == '/') last_sep = p;
                    if (last_sep) {
                        size_t len = (size_t)(last_sep - state->cfg.output_file) + 1;
                        if (len < sizeof(dir_path)) {
                            memcpy(dir_path, state->cfg.output_file, len);
                            dir_path[len] = '\0';
                        }
                    }
                }

                if (GetDiskFreeSpaceExA(dir_path, &free_bytes, NULL, NULL)) {
                    LONGLONG free_mb = (LONGLONG)(free_bytes.QuadPart
                                                  / (1024ULL * 1024ULL));
                    InterlockedExchange64(&state->disk_free_mb, (LONG64)free_mb);
                    if (free_mb < 100) {
                        LOG_WARN("Disk space critically low (%lld MB free) - "
                                 "stopping recording to protect the file.",
                                 free_mb);
                        state->disk_stop = 1;
                        g_running = 0;
                        break;
                    } else if (free_mb < 500 && !state->disk_warn_issued) {
                        LOG_WARN("Disk space low: %lld MB remaining.", free_mb);
                        state->disk_warn_issued = 1;
                    }
                } else {
                    if (state->cfg.verbose)
                        LOG_INFO("GetDiskFreeSpaceExA failed for '%s' (error %lu)",
                                 dir_path, GetLastError());
                }
            }
        }

        if (state->dual_separate_files) {
            /* De-interleave IA,QA,IB,QB dual frames into two single-channel
             * streams and write each to its own file. */
            SIZE_T num_frames = to_read / 8;
            SIZE_T split_bytes = num_frames * 4;
            SIZE_T i;
            const uint8_t *src = g_write_chunk;

            for (i = 0; i < num_frames; i++) {
                memcpy(&g_split_chunk_a[i * 4], &src[i * 8], 4);
                memcpy(&g_split_chunk_b[i * 4], &src[i * 8 + 4], 4);
            }

            ok = WriteFile(state->out_file, g_split_chunk_a,
                           (DWORD)split_bytes, &written, NULL);
            if (!ok || written != (DWORD)split_bytes) {
                LOG_ERROR("Write failed (Tuner A file): Windows error %lu "
                          "(wrote %lu of %zu bytes)",
                          GetLastError(), written, split_bytes);
                state->writer_error = 1;
                break;
            }
            ok = WriteFile(state->out_file_b, g_split_chunk_b,
                           (DWORD)split_bytes, &written, NULL);
            if (!ok || written != (DWORD)split_bytes) {
                LOG_ERROR("Write failed (Tuner B file): Windows error %lu "
                          "(wrote %lu of %zu bytes)",
                          GetLastError(), written, split_bytes);
                state->writer_error = 1;
                break;
            }

            /* Pipe monitoring carries the combined stream, unchanged. */
            if (state->pipe_handle != INVALID_HANDLE_VALUE) {
                DWORD pipe_written;
                WriteFile(state->pipe_handle, g_write_chunk, (DWORD)to_read,
                          &pipe_written, NULL);
            }
        } else {
            /* out_file is INVALID_HANDLE_VALUE in the slave's listen-only
             * mode (no file is ever created there) - skip the write
             * rather than let it fail and tear down the writer thread,
             * which would also kill the pipe write below along with it.  */
            if (state->out_file != INVALID_HANDLE_VALUE) {
                ok = WriteFile(state->out_file, g_write_chunk, (DWORD)to_read,
                               &written, NULL);

                if (!ok || written != (DWORD)to_read) {
                    DWORD err = GetLastError();
                    LOG_ERROR("Write failed: Windows error %lu (wrote %lu of %zu bytes)",
                              err, written, to_read);
                    state->writer_error = 1;
                    break;
                }
            }

            /* Non-blocking pipe write - silently skip if no client connected
             * or client buffer full. Never blocks or errors the recording. */
            if (state->pipe_handle != INVALID_HANDLE_VALUE) {
                DWORD pipe_written;
                WriteFile(state->pipe_handle, g_write_chunk, (DWORD)to_read,
                          &pipe_written, NULL);
                /* Ignore return value intentionally - pipe is best-effort. */
            }
        }

        InterlockedAdd64(&state->samples_written,
                         (LONG64)(to_read / (state->cfg.dual_channel ? 8 : 4)));
        InterlockedAdd64(&state->segment_samples_written,
                         (LONG64)(to_read / (state->cfg.dual_channel ? 8 : 4)));

        {
            int split_active =
                (state->cfg.output_format == FORMAT_SDRUNO) ||
                ((state->cfg.output_format == FORMAT_WINRAD ||
                  state->cfg.output_format == FORMAT_SDRCONNECT) &&
                 state->cfg.large_file_mode == LARGE_FILE_SPLIT);
            if (split_active && state->out_file != INVALID_HANDLE_VALUE)
                check_split_rollover(state);
        }
    }

    if (g_state.cfg.verbose)
        LOG_INFO("Writer thread exiting");
    return 0;
}

/* =========================================================================
 * SDRplay streaming callback - single channel
 *
 * CONTRACT: No malloc, no free, no I/O, no locks, no system calls.
 * Only ring_write() and atomic increments.
 * ========================================================================= */
static void stream_callback_single(
    short *xi, short *xq,
    sdrplay_api_StreamCbParamsT *params,
    unsigned int num_samples,
    unsigned int reset,
    void *cbContext)
{
    AppState *state = (AppState *)cbContext;
    unsigned int i;

    (void)params;
    (void)reset;

    if (state->teardown_forced) return;   /* device is being torn down after
                                           * a StopPending timeout - ring may
                                           * be freed any moment, touch nothing */

    InterlockedIncrement64(&state->callback_count);
    InterlockedAdd64(&state->samples_received, (LONG64)num_samples);

    /* Compute peak absolute value for dBFS display.
     * 32767 = 0 dBFS (full scale for 16-bit signed).          */
    {
        unsigned int k;
        int peak = 0;
        for (k = 0; k < num_samples; k++) {
            int ai = xi[k] < 0 ? -xi[k] : xi[k];
            int aq = xq[k] < 0 ? -xq[k] : xq[k];
            if (ai > peak) peak = ai;
            if (aq > peak) peak = aq;
        }
        if (peak > 0) {
            float db = 20.0f * log10f((float)peak / 32767.0f);
            /* Only update if louder than current peak */
            if (db > state->peak_dbfs)
                state->peak_dbfs = db;
        }
    }

    /* Interleave I/Q into a small stack buffer and push to ring.
     * We process in chunks to avoid large stack allocations. */
    #define CHUNK 1024
    int16_t tmp[CHUNK * 2];
    unsigned int offset = 0;

    while (offset < num_samples) {
        unsigned int batch = num_samples - offset;
        if (batch > CHUNK) batch = CHUNK;

        if (!state->listening) {
            /* Nothing drains the ring while listening - no writer thread
             * exists until a file is actually opened at Record time - so
             * writing to it here would just fill it to 100% and start
             * dropping samples immediately, for no benefit: the monitor
             * and level meters below read straight from xi/xq, not from
             * the ring. Skip the interleave-and-write work entirely.     */
            for (i = 0; i < batch; i++) {
                tmp[i * 2]     = (int16_t)xi[offset + i];
                tmp[i * 2 + 1] = (int16_t)xq[offset + i];
            }
            ring_write(&state->ring, tmp, batch * 4);
        }
        /* This callback only ever carries Tuner A's real ADC data - for a
         * true single-tuner recording that's the only tuner anyway, so
         * feed unconditionally. But in Master/Slave mode, Tuner B's real
         * data lives entirely in the separate slave process and never
         * reaches this one - feeding it here when "Tuner B" is selected
         * would silently play Tuner A's signal mislabelled as B's, with
         * NCO math for the wrong centre frequency. Stay silent instead. */
        if (!state->master_slave_active || g_monitor.tuner_sel == 0)
            monitor_feed(&xi[offset], &xq[offset], batch);
        offset += batch;
    }
    #undef CHUNK
}

/* =========================================================================
 * SDRplay streaming callback - RSPduo Channel A (dual mode)
 *
 * In dual mode, we receive Channel A and Channel B in separate callbacks.
 * We must merge them: IA QA IB QB per frame.
 * Strategy: buffer Channel A, merge when Channel B arrives.
 * ========================================================================= */
static void stream_callback_dual_a(
    short *xi, short *xq,
    sdrplay_api_StreamCbParamsT *params,
    unsigned int num_samples,
    unsigned int reset,
    void *cbContext)
{
    AppState *state = (AppState *)cbContext;
    unsigned int i;

    (void)params;
    (void)reset;

    if (state->teardown_forced) return;

    InterlockedIncrement64(&state->callback_count);

    /* Store Channel A data - allocate/resize as needed */
    EnterCriticalSection(&state->dual_lock);

    if ((SIZE_T)(num_samples * sizeof(int16_t)) > state->dual_merge_buf_size / 4) {
        /* Resize merge buffer: 4 * num_samples * sizeof(int16_t) for IQIQ */
        if (state->dual_merge_buf)
            free(state->dual_merge_buf);
        state->dual_merge_buf_size = num_samples * 4 * sizeof(int16_t);
        state->dual_merge_buf = (int16_t *)malloc(state->dual_merge_buf_size);
    }

    if (state->pending_a_i == NULL ||
        (SIZE_T)(num_samples * sizeof(int16_t)) > state->dual_merge_buf_size / 4) {
        /* Allocate pending buffers */
        if (state->pending_a_i) free(state->pending_a_i);
        if (state->pending_a_q) free(state->pending_a_q);
        state->pending_a_i = (int16_t *)malloc(num_samples * sizeof(int16_t));
        state->pending_a_q = (int16_t *)malloc(num_samples * sizeof(int16_t));
    }

    for (i = 0; i < num_samples; i++) {
        state->pending_a_i[i] = (int16_t)xi[i];
        state->pending_a_q[i] = (int16_t)xq[i];
    }
    state->pending_a_count = (int)num_samples;
    state->pending_a_valid = 1;

    LeaveCriticalSection(&state->dual_lock);

    InterlockedAdd64(&state->samples_received, (LONG64)num_samples);
}

/* =========================================================================
 * SDRplay streaming callback - RSPduo Channel B (dual mode)
 * When Channel B arrives, merge with buffered Channel A and push to ring.
 * ========================================================================= */
static void stream_callback_dual_b(
    short *xi, short *xq,
    sdrplay_api_StreamCbParamsT *params,
    unsigned int num_samples,
    unsigned int reset,
    void *cbContext)
{
    AppState *state = (AppState *)cbContext;
    unsigned int i;
    int merge_count;

    (void)params;
    (void)reset;

    if (state->teardown_forced) return;

    EnterCriticalSection(&state->dual_lock);

    if (!state->pending_a_valid) {
        /* Channel A not yet ready - drop this Channel B frame */
        LeaveCriticalSection(&state->dual_lock);
        return;
    }

    /* Use the minimum of A and B sample counts for alignment */
    merge_count = state->pending_a_count;
    if ((int)num_samples < merge_count)
        merge_count = (int)num_samples;

    /* Build interleaved IA QA IB QB buffer and compute peak dBFS */
    if (state->dual_merge_buf) {
        int peak_a = 0, peak_b = 0;
        for (i = 0; i < (unsigned int)merge_count; i++) {
            int16_t ia = state->pending_a_i[i];
            int16_t qa = state->pending_a_q[i];
            int16_t ib = (int16_t)xi[i];
            int16_t qb = (int16_t)xq[i];
            state->dual_merge_buf[i * 4]     = ia;
            state->dual_merge_buf[i * 4 + 1] = qa;
            state->dual_merge_buf[i * 4 + 2] = ib;
            state->dual_merge_buf[i * 4 + 3] = qb;
            /* Track peaks independently per tuner */
            if (ia < 0) ia = -ia;
            if (qa < 0) qa = -qa;
            if (ib < 0) ib = -ib;
            if (qb < 0) qb = -qb;
            if (ia > peak_a) peak_a = ia;
            if (qa > peak_a) peak_a = qa;
            if (ib > peak_b) peak_b = ib;
            if (qb > peak_b) peak_b = qb;
        }

        if (peak_a > 0) {
            float db = 20.0f * log10f((float)peak_a / 32767.0f);
            if (db > state->peak_dbfs)
                state->peak_dbfs = db;
        }
        if (peak_b > 0) {
            float db = 20.0f * log10f((float)peak_b / 32767.0f);
            if (db > state->peak_dbfs_b)
                state->peak_dbfs_b = db;
        }

        if (!state->listening) {
            /* Same reasoning as stream_callback_single's own guard -
             * nothing drains the ring during listening, so writing here
             * unconditionally just fills it to 100% right away for no
             * benefit; the monitor and level meters read straight from
             * the merge buffer above, not from the ring.                 */
            ring_write(&state->ring, state->dual_merge_buf,
                       (SIZE_T)merge_count * 8);
        }

        /* Dual-channel: feed whichever tuner the monitor is listening to.
         * Both A and B are available right here at the merge point. */
        if (g_monitor.tuner_sel == 0)
            monitor_feed(state->pending_a_i, state->pending_a_q,
                         (unsigned int)merge_count);
        else
            monitor_feed(xi, xq, (unsigned int)merge_count);
    }

    state->pending_a_valid = 0;

    LeaveCriticalSection(&state->dual_lock);
}

/* =========================================================================
 * SDRplay event callback
 * ========================================================================= */
static void event_callback(
    sdrplay_api_EventT eventId,
    sdrplay_api_TunerSelectT tuner,
    sdrplay_api_EventParamsT *params,
    void *cbContext)
{
    AppState *state = (AppState *)cbContext;

    switch (eventId) {
    case sdrplay_api_GainChange:
        /* Log in verbose mode only. Do not write reported values back into
         * the channel parameter structs. The keyboard thread owns gain state
         * via private local variables; writing into the structs from this
         * callback thread races with those variables and causes erratic gain
         * behaviour regardless of any AGC guard condition.               */
        if (tuner == sdrplay_api_Tuner_B)
            g_curr_gain_b = params->gainParams.currGain;
        else
            g_curr_gain_a = params->gainParams.currGain;
        if (state->cfg.verbose) {
            static DWORD last_log_a = 0, last_log_b = 0;
            DWORD now = GetTickCount();
            DWORD *last = (tuner == sdrplay_api_Tuner_B) ? &last_log_b : &last_log_a;
            if (now - *last >= 3000) {
                *last = now;
                LOG_INFO("Gain change [%s]: lnaGRdB=%d, grDb=%d, currGain=%.1f",
                         tuner == sdrplay_api_Tuner_A ? "T1" :
                         tuner == sdrplay_api_Tuner_B ? "T2" : "Both",
                         params->gainParams.lnaGRdB,
                         params->gainParams.gRdB,
                         params->gainParams.currGain);
            }
        }
        break;

    case sdrplay_api_PowerOverloadChange:
        {
            int detected = (params->powerOverloadParams.powerOverloadChangeType
                            == sdrplay_api_Overload_Detected);

            switch (tuner) {
            case sdrplay_api_Tuner_A:
                state->overload_tuner_a = detected;
                break;
            case sdrplay_api_Tuner_B:
                state->overload_tuner_b = detected;
                break;
            case sdrplay_api_Tuner_Both:
                state->overload_tuner_a = detected;
                state->overload_tuner_b = detected;
                break;
            default:
                state->overload_tuner_a = detected;
                break;
            }

            /* Overload state is reflected by the indicator segment at the
             * right end of the level meter. No log message is written since
             * overload can occur frequently and would generate excessive log
             * entries during overnight recordings.                         */
        }
        sdrplay_api_Update(state->device.dev, state->device.tuner,
                           sdrplay_api_Update_Ctrl_OverloadMsgAck,
                           sdrplay_api_Update_Ext1_None);
        break;

    case sdrplay_api_DeviceRemoved:
        LOG_ERROR("Device removed unexpectedly!");
        g_running = 0;
        break;

    case sdrplay_api_RspDuoModeChange:
        LOG_INFO("RSPduo mode change event");
        break;

    default:
        break;
    }
}

/* =========================================================================
 * INI config file parser
 * Simple key=value parser, ignores lines starting with ; or #
 * ========================================================================= */
/* =========================================================================
 * Valid SDRplay API parameter combinations
 *
 * Zero-IF mode (if_khz = 0):
 *   Any ADC sample rate 2-10 Msps is valid.
 *   Output sample rate = ADC sample rate (no internal decimation).
 *   IF bandwidth must be <= sample rate.
 *   Valid bw_khz values: 200, 300, 600, 1536, 5000, 6000, 7000, 8000
 *
 * Low-IF mode (if_khz = 450, 1620, or 2048):
 *   Only specific (if_khz, bw_khz, adc_rate) combinations enable the API's
 *   internal down-conversion - confirmed against the official sdrplay_api
 *   v3.15 specification's documented conditions for sdrplay_api_Init():
 *
 *     (fsHz==8192000) && (bwType==BW_1_536) && (ifType==IF_2_048)
 *     (fsHz==8000000) && (bwType==BW_1_536) && (ifType==IF_2_048)
 *     (fsHz==8000000) && (bwType==BW_5_000) && (ifType==IF_2_048)
 *     (fsHz==2000000) && (bwType<=BW_0_300) && (ifType==IF_0_450)
 *     (fsHz==2000000) && (bwType==BW_0_600) && (ifType==IF_0_450)
 *     (fsHz==6000000) && (bwType<=BW_1_536) && (ifType==IF_1_620)
 *
 *   Two of these are NOT yet implemented below and are deliberately left
 *   out until confirmed against real hardware, since the spec doesn't
 *   state their resulting decimation/output rate the way it does for the
 *   others: 2048kHz IF at exactly 8.192 Msps (BW 1536), and 2048kHz IF at
 *   8 Msps with BW 5000.
 *
 *   IF=450 kHz (single-tuner only, not RSPduo dual-tuner compatible):
 *     ADC 2 Msps, BW=200  -> decimation /4 -> output 0.5 Msps
 *     ADC 2 Msps, BW=300  -> decimation /4 -> output 0.5 Msps
 *     ADC 2 Msps, BW=600  -> decimation /2 -> output 1 Msps
 *     (decimation/output confirmed against SDRplay's legacy mir_sdr API
 *     reference table, which documents this IF/decimation relationship
 *     explicitly; the v3 spec above confirms the same fsHz/bwType/ifType
 *     triples remain valid.)
 *
 *   IF=1620 kHz:
 *     ADC 6 Msps, BW=1536 -> decimation /3 -> output 2 Msps   **recommended for MW**
 *     ADC 6 Msps, BW=600  -> decimation /3 -> output 2 Msps   (assumed same decimation as BW=1536 at this fsHz/ifType - not yet confirmed against hardware)
 *     ADC 6 Msps, BW=300  -> decimation /3 -> output 2 Msps   (assumed, see above)
 *     ADC 6 Msps, BW=200  -> decimation /3 -> output 2 Msps   (assumed, see above)
 *     ADC 8 Msps, BW=1536 -> decimation /4 -> output 2 Msps
 *
 *   IF=2048 kHz, BW=1536 kHz:
 *     ADC 8 Msps  -> decimation /4 -> output 2 Msps
 *
 *   RSPduo dual-tuner mode additionally restricts to:
 *     ADC 6 Msps  (IF=1620, BW=1536) -> output 2 Msps
 *     ADC 8 Msps  (IF=2048, BW=1536) -> output 2 Msps
 *   (dual-tuner mode fixes the shared ADC rate at 6 or 8 Msps per the API
 *   spec's sdrplay_api_SwapRspDuoDualTunerModeSampleRate(), so the BW=200/
 *   300/600 options at 6 Msps and the IF=450 mode are single-tuner only.)
 *
 * All other combinations will either be rejected by the API or produce
 * unpredictable output sample rates.
 * ========================================================================= */
typedef struct {
    int    if_khz;          /* IF frequency in kHz */
    int    bw_khz;          /* IF bandwidth in kHz - also used directly as
                              * the usable signal bandwidth (see
                              * expected_usable_bw_hz's comment for why
                              * there's no separate, further-derated
                              * figure here). */
    double adc_rate_hz;     /* Required ADC sample rate */
    int    decimation;      /* Internal decimation factor */
    double output_rate_hz;  /* Resulting output sample rate */
    int    dual_ok;         /* Valid for RSPduo dual-tuner mode */
    const char *note;
} ValidCombo;

static const ValidCombo VALID_COMBOS[] = {
    /* Zero-IF - output rate = ADC rate, any BW <= SR */
    {    0,  200, 2000000.0, 1, 2000000.0, 0, "Zero-IF 2Msps/200kHz BW"},
    {    0,  300, 2000000.0, 1, 2000000.0, 0, "Zero-IF 2Msps/300kHz BW"},
    {    0,  600, 2000000.0, 1, 2000000.0, 0, "Zero-IF 2Msps/600kHz BW"},
    {    0, 1536, 2000000.0, 1, 2000000.0, 0, "Zero-IF 2Msps/1536kHz BW"},
    {    0,  300, 3000000.0, 1, 3000000.0, 0, "Zero-IF 3Msps/300kHz BW"},
    {    0,  600, 3000000.0, 1, 3000000.0, 0, "Zero-IF 3Msps/600kHz BW"},
    {    0, 1536, 3000000.0, 1, 3000000.0, 0, "Zero-IF 3Msps/1536kHz BW"},
    {    0,  600, 4000000.0, 1, 4000000.0, 0, "Zero-IF 4Msps/600kHz BW"},
    {    0, 1536, 4000000.0, 1, 4000000.0, 0, "Zero-IF 4Msps/1536kHz BW"},
    {    0, 5000, 5000000.0, 1, 5000000.0, 0, "Zero-IF 5Msps/5000kHz BW"},
    {    0, 1536, 5000000.0, 1, 5000000.0, 0, "Zero-IF 5Msps/1536kHz BW"},
    {    0, 5000, 6000000.0, 1, 6000000.0, 0, "Zero-IF 6Msps/5000kHz BW"},
    {    0, 6000, 6000000.0, 1, 6000000.0, 0, "Zero-IF 6Msps/6000kHz BW"},
    {    0, 1536, 6000000.0, 1, 6000000.0, 0, "Zero-IF 6Msps/1536kHz BW"},
    {    0, 6000, 7000000.0, 1, 7000000.0, 0, "Zero-IF 7Msps/6000kHz BW"},
    {    0, 7000, 7000000.0, 1, 7000000.0, 0, "Zero-IF 7Msps/7000kHz BW"},
    {    0, 6000, 8000000.0, 1, 8000000.0, 0, "Zero-IF 8Msps/6000kHz BW"},
    {    0, 7000, 8000000.0, 1, 8000000.0, 0, "Zero-IF 8Msps/7000kHz BW"},
    {    0, 8000, 8000000.0, 1, 8000000.0, 0, "Zero-IF 8Msps/8000kHz BW"},
    {    0, 8000,10000000.0, 1,10000000.0, 0, "Zero-IF 10Msps/8000kHz BW"},
    /* Low-IF 450 kHz - single-tuner only, not RSPduo dual-tuner compatible */
    {  450,  200, 2000000.0, 4,  500000.0, 0, "Low-IF 450/200kHz 2Msps->0.5Msps"},
    {  450,  300, 2000000.0, 4,  500000.0, 0, "Low-IF 450/300kHz 2Msps->0.5Msps"},
    {  450,  600, 2000000.0, 2, 1000000.0, 0, "Low-IF 450/600kHz 2Msps->1Msps"},
    /* Low-IF 1620 kHz - internal decimation /3 at 6Msps, /4 at 8Msps, output always 2 Msps */
    { 1620,  200, 6000000.0, 3, 2000000.0, 1, "Low-IF 1620/200kHz 6Msps->2Msps (dual)"},
    { 1620,  300, 6000000.0, 3, 2000000.0, 1, "Low-IF 1620/300kHz 6Msps->2Msps (dual)"},
    { 1620,  600, 6000000.0, 3, 2000000.0, 1, "Low-IF 1620/600kHz 6Msps->2Msps (dual)"},
    { 1620, 1536, 6000000.0, 3, 2000000.0, 1, "Low-IF 1620/1536kHz 6Msps->2Msps (dual, MW)"},
    { 1620, 1536, 8000000.0, 4, 2000000.0, 0, "Low-IF 1620/1536kHz 8Msps->2Msps"},
    /* Low-IF 2048 kHz - internal decimation /4, output always 2 Msps */
    { 2048, 1536, 8000000.0, 4, 2000000.0, 1, "Low-IF 2048/1536kHz 8Msps->2Msps (dual)"},
};
#define NUM_VALID_COMBOS (int)(sizeof(VALID_COMBOS)/sizeof(VALID_COMBOS[0]))

/* The only centre frequencies HDR mode actually supports on RSPdx/RSPdx R2.
 * Single source of truth - used both for the live Settings dialog hint and
 * for hard validation in validate_config(), so the two can never drift out
 * of sync with each other. */
static const double HDR_VALID_KHZ[] = {
    135, 175, 220, 250, 340, 475, 516, 875, 1125, 1900
};
#define NUM_HDR_VALID_KHZ (int)(sizeof(HDR_VALID_KHZ)/sizeof(HDR_VALID_KHZ[0]))

/*
 * Validate config against the table above.
 * Returns 1 if valid, 0 if invalid (prints error and suggested fix).
 * Also fills in cfg->actual_output_rate_hz with the expected output rate.
 */
static int validate_config(Config *cfg)
{
    int i;
    double sr = cfg->sample_rate_hz;
    int    bw = cfg->bw_khz;
    int    ifreq = cfg->if_khz;
    int    dual = cfg->dual_channel;

    /* Validate IF gain reduction range. The SDRplay API rejects values
     * below 20 dB because GR 0-19 always causes ADC clipping. Valid
     * range is 20-59 for all RSP devices.                              */
    if (cfg->gain_reduction < 20 || cfg->gain_reduction > 59) {
        LOG_ERROR("gain_reduction=%d is out of range. "
                  "Valid range for all RSP devices is 20-59 dB.",
                  cfg->gain_reduction);
        LOG_ERROR("Use 20 for maximum gain (strong signals / external preamp).");
        LOG_ERROR("Use 59 for minimum gain (very strong local signals).");
        LOG_ERROR("For MW DX, 30-45 is a typical starting range.");
        return 0;
    }

    /* Software decimation: validated up front (not just in the "combo not
     * found" fallback below) because it applies on top of ANY valid combo,
     * and expected_output_rate_hz below depends on it being sane. */
    {
        int d = cfg->decimation;
        if (d != 1 && d != 2 && d != 4 && d != 8 && d != 16 && d != 32) {
            LOG_ERROR("Invalid decimation=%d. Valid values: 1, 2, 4, 8, 16, 32", d);
            return 0;
        }
    }

    for (i = 0; i < NUM_VALID_COMBOS; i++) {
        const ValidCombo *c = &VALID_COMBOS[i];
        if (c->if_khz == ifreq &&
            c->bw_khz == bw &&
            fabs(c->adc_rate_hz - sr) < 1000.0) {

            if (dual && !c->dual_ok) {
                LOG_ERROR("Combination IF=%d kHz, BW=%d kHz, SR=%.0f sps "
                          "is NOT valid for RSPduo dual-tuner mode.",
                          ifreq, bw, sr);
                LOG_ERROR("For RSPduo dual mode use:");
                LOG_ERROR("  IF=1620 kHz, BW=1536 kHz, SR=6000000 sps  -> 2 Msps output");
                LOG_ERROR("  IF=2048 kHz, BW=1536 kHz, SR=8000000 sps  -> 2 Msps output");
                return 0;
            }

            /* Store the expected output rate so the header can be written
             * correctly upfront without any mid-recording seek/patch.
             * MUST include the software decimation factor: the API divides
             * the actual sample rate delivered to the stream callback by
             * cfg->decimation on top of the combo's own internal (Low-IF)
             * decimation, which c->output_rate_hz already accounts for.
             * Previously this was left out entirely, so anything reading
             * expected_output_rate_hz (monitor audio resampling, the demod
             * path, and the recorded file's own header) silently assumed
             * the un-decimated rate whenever decimation > 1 - the live
             * monitor starved for samples between arrivals (audible as
             * chuffing that got worse with a higher decimation factor),
             * and the file header ended up with the wrong sample rate. */
            cfg->expected_output_rate_hz = c->output_rate_hz / cfg->decimation;
            /* Usable BW is the filter bandwidth actually selected - the
             * one figure SDRplay's own API specifies, rather than a
             * further-derated estimate (an earlier version of this tried
             * to model analog roll-off with an invented percentage, but
             * real measurement showed that undershot reality noticeably -
             * see git history / changelog for the correction). Software
             * decimation can narrow the Nyquist window below the filter's
             * own width, so take whichever is smaller. */
            {
                double bw_hz = (double)c->bw_khz * 1000.0;
                cfg->expected_usable_bw_hz = (bw_hz < cfg->expected_output_rate_hz)
                                              ? bw_hz
                                              : cfg->expected_output_rate_hz;
            }

            LOG_INFO("Config validated: %s", c->note);
            if (c->decimation > 1)
                if (g_state.cfg.verbose)
                    LOG_INFO("  Internal decimation /%d: output will be %.0f sps",
                         c->decimation, c->output_rate_hz);
            if (cfg->decimation > 1)
                LOG_INFO("  Software decimation /%d: final output rate %.0f sps",
                         cfg->decimation, cfg->expected_output_rate_hz);
            return 1;
        }
    }

    /* Not found in table */
    LOG_ERROR("Invalid parameter combination:");
    LOG_ERROR("  sample_rate = %.0f sps (%.3f Msps)", sr, sr / 1e6);
    LOG_ERROR("  if_khz      = %d kHz", ifreq);
    LOG_ERROR("  bw_khz      = %d kHz", bw);
    LOG_ERROR("");
    if (ifreq == 0) {
        LOG_ERROR("For Zero-IF mode (if_khz=0), valid ADC sample rates are");
        LOG_ERROR("2, 3, 4, 5, 6, 7, 8, 10 Msps. IF bandwidth must be <= sample rate.");
        LOG_ERROR("Valid bw_khz values: 200, 300, 600, 1536, 5000, 6000, 7000, 8000");
    } else {
        LOG_ERROR("For Low-IF mode, valid combinations are:");
        LOG_ERROR("  if_khz=1620, bw_khz=1536, sample_rate_msps=6  -> 2 Msps output");
        LOG_ERROR("  if_khz=1620, bw_khz=1536, sample_rate_msps=8  -> 2 Msps output");
        LOG_ERROR("  if_khz=2048, bw_khz=1536, sample_rate_msps=8  -> 2 Msps output");
    }
    return 0;
}

static void config_set_defaults(Config *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    /* Leave output_file as DEFAULT_OUTPUT_FILE - main() will replace it
     * with a timestamped name (YYYYMMDD_HHMMSS_<freq>kHz.raw) unless
     * the user sets an explicit filename via -o or output_file= in INI. */
    cfg->recording_path[0] = '\0';
    strncpy(cfg->output_file, DEFAULT_OUTPUT_FILE, MAX_PATH_LEN - 1);
    cfg->frequency_hz   = DEFAULT_FREQUENCY_HZ;
    cfg->sample_rate_hz = DEFAULT_SAMPLE_RATE_HZ;
    cfg->gain_reduction = DEFAULT_GAIN_REDUCTION;
    cfg->lna_state      = DEFAULT_LNA_STATE;
    cfg->if_khz         = DEFAULT_IF_KHZ;
    cfg->bw_khz         = DEFAULT_BW_KHZ;
    cfg->duration_sec   = DEFAULT_DURATION_SEC;
    cfg->agc_enable     = DEFAULT_AGC_ENABLE;
    cfg->agc_setpoint_dbfs = -60;
    cfg->agc_attack_ms     = 50;
    cfg->agc_decay_ms      = 100;
    cfg->dc_correct     = DEFAULT_DC_CORRECT;
    cfg->iq_correct     = DEFAULT_IQ_CORRECT;
    cfg->notch_rf       = DEFAULT_NOTCH_RF;
    cfg->notch_dab      = DEFAULT_NOTCH_DAB;
    cfg->dual_channel   = 0;
    strncpy(cfg->rspduo_single_tuner, "A", sizeof(cfg->rspduo_single_tuner) - 1);
    cfg->monitor_bar_visible = 1;
    cfg->monitor_hpf_enable = 0;
    cfg->monitor_hpf_hz     = 100.0;
    cfg->monitor_volume_percent = 15;
    cfg->monitor_smeter_mode = 0;
    cfg->monitor_smeter_cal_offset = 0.0;
    cfg->freq_b_hz      = DEFAULT_FREQUENCY_HZ;
    cfg->ring_buffer_sec      = RING_BUFFER_SECONDS;
    cfg->monitor_interval_ms  = DEFAULT_MONITOR_INTERVAL_MS;
    cfg->verbose        = 0;
    cfg->output_format  = FORMAT_LINRAD;
    cfg->large_file_mode = LARGE_FILE_SPLIT;
    cfg->latitude        = 0.0;
    cfg->longitude       = 0.0;
    cfg->show_sun_times  = 0;

    strncpy(cfg->antenna, "A", 7);  /* default to Antenna A / 50 ohm */
    cfg->bias_t         = 0;
    cfg->hiz_notch      = 0;
    cfg->hdr_enable     = 0;
    cfg->hdr_bw_khz     = 1700;
    cfg->ppm            = 0.0;
    cfg->device_serial[0] = '\0'; /* empty = use first device found */
    cfg->decimation     = 1;      /* no additional decimation */

    cfg->expected_output_rate_hz = 0.0; /* set by validate_config */
    cfg->expected_usable_bw_hz = 0.0;   /* set by validate_config */
    /* Tuner B defaults - mirror Tuner A until explicitly overridden */
    cfg->gain_reduction_b = -1;  /* -1 = use gain_reduction value */
    cfg->lna_state_b      = -1;  /* -1 = use lna_state value */
    cfg->agc_enable_b     = -1;
    cfg->dc_correct_b     = -1;
    cfg->iq_correct_b     = -1;
    cfg->notch_rf_b       = -1;
    cfg->notch_dab_b      = -1;
    cfg->tuner_b_settings_set = 0;
    cfg->start_time[0]    = '\0';
    cfg->spinup_enable        = 1;
    cfg->spinup_bytes         = 1024 * 1024;
    cfg->pipe_enable          = 0;
    strncpy(cfg->pipe_name, "\\\\.\\pipe\\duodx", 127);
    cfg->http_port            = 0;
    cfg->http_interval_ms     = 2000;     /* 2 second default refresh */
    memset(cfg->schedule, 0, sizeof(cfg->schedule));
    cfg->schedule_only   = 0;
    cfg->use_utc         = 1;
    cfg->show_clock      = 1;
    cfg->meter_style     = 0;
    cfg->hourly_enable      = 0;
    cfg->hourly_window_min  = 10;
    strncpy(cfg->hourly_start, "18:00", 7);
    strncpy(cfg->hourly_stop,  "06:00", 7);
    cfg->window_x = cfg->window_y = (int)CW_USEDEFAULT;
    cfg->window_w = 930;
    cfg->window_h = 660;
    cfg->window_maximized = 0;
    strncpy(cfg->color_scheme, "navy", sizeof(cfg->color_scheme) - 1);
    strncpy(cfg->timer_last_mode, "schedule", sizeof(cfg->timer_last_mode) - 1);
}

static void trim(char *s)
{
    char *p = s + strlen(s) - 1;
    while (p >= s && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n'))
        *p-- = '\0';
    p = s;
    while (*p == ' ' || *p == '\t') p++;
    if (p != s) memmove(s, p, strlen(p) + 1);
}

static int parse_duration_hms(const char *str);
static void format_duration_hms(int total_sec, char *out, size_t out_size);

static void config_load_ini(Config *cfg, const char *path)
{
    FILE *fp = fopen(path, "r");
    char line[256], key[128], val[128];

    if (!fp) {
        LOG_INFO("No config file found at '%s', using defaults", path);
        return;
    }

    LOG_INFO("Loading config from '%s'", path);

    while (fgets(line, sizeof(line), fp)) {
        trim(line);
        if (line[0] == '\0' || line[0] == ';' || line[0] == '#' || line[0] == '[')
            continue;

        if (sscanf(line, "%127[^=]=%127[^\r\n]", key, val) != 2)
            continue;

        trim(key);
        trim(val);

        /* Strip inline comments (everything from first ' ;' or '\t;') */
        {
            char *sc = val;
            while (*sc) {
                if (*sc == ';' && (sc == val || *(sc-1) == ' ' || *(sc-1) == '\t')) {
                    *sc = '\0';
                    break;
                }
                sc++;
            }
            trim(val);  /* re-trim after comment removal */
        }

        if      (!strcmp(key, "recording_path")) strncpy(cfg->recording_path, val, MAX_PATH_LEN-1);
        else if (!strcmp(key, "output_file"))    strncpy(cfg->output_file, val, MAX_PATH_LEN-1);
        else if (!strcmp(key, "frequency_hz"))   cfg->frequency_hz   = atof(val);
        else if (!strcmp(key, "frequency_mhz"))  cfg->frequency_hz   = atof(val) * 1e6;
        else if (!strcmp(key, "sample_rate_hz")) cfg->sample_rate_hz = atof(val);
        else if (!strcmp(key, "sample_rate_msps"))cfg->sample_rate_hz= atof(val) * 1e6;
        else if (!strcmp(key, "gain_reduction")) cfg->gain_reduction = atoi(val);
        else if (!strcmp(key, "lna_state"))      cfg->lna_state      = atoi(val);
        else if (!strcmp(key, "if_khz"))         cfg->if_khz         = atoi(val);
        else if (!strcmp(key, "bw_khz"))         cfg->bw_khz         = atoi(val);
        else if (!strcmp(key, "duration"))       cfg->duration_sec   = parse_duration_hms(val);
        else if (!strcmp(key, "agc_enable"))     cfg->agc_enable     = atoi(val);
        else if (!strcmp(key, "agc_setpoint_dbfs")) cfg->agc_setpoint_dbfs = atoi(val);
        else if (!strcmp(key, "agc_attack_ms"))     cfg->agc_attack_ms     = atoi(val);
        else if (!strcmp(key, "agc_decay_ms"))      cfg->agc_decay_ms      = atoi(val);
        else if (!strcmp(key, "dc_correct"))     cfg->dc_correct     = atoi(val);
        else if (!strcmp(key, "iq_correct"))     cfg->iq_correct     = atoi(val);
        else if (!strcmp(key, "notch_rf"))       cfg->notch_rf       = atoi(val);
        else if (!strcmp(key, "notch_dab"))      cfg->notch_dab      = atoi(val);
        else if (!strcmp(key, "dual_channel"))   cfg->dual_channel   = atoi(val);
        else if (!strcmp(key, "rspduo_single_tuner"))
            strncpy(cfg->rspduo_single_tuner, val, sizeof(cfg->rspduo_single_tuner) - 1);
        else if (!strcmp(key, "monitor_bar_visible")) cfg->monitor_bar_visible = atoi(val);
        else if (!strcmp(key, "monitor_hpf_enable")) cfg->monitor_hpf_enable = atoi(val);
        else if (!strcmp(key, "monitor_hpf_hz"))     cfg->monitor_hpf_hz     = atof(val);
        else if (!strcmp(key, "monitor_volume_percent")) cfg->monitor_volume_percent = atoi(val);
        else if (!strcmp(key, "monitor_smeter_mode")) cfg->monitor_smeter_mode = atoi(val);
        else if (!strcmp(key, "monitor_smeter_cal_offset")) cfg->monitor_smeter_cal_offset = atof(val);
        else if (!strcmp(key, "freq_b_hz"))      cfg->freq_b_hz      = atof(val);
        else if (!strcmp(key, "freq_b_mhz"))     cfg->freq_b_hz      = atof(val) * 1e6;
        else if (!strcmp(key, "gain_reduction_b")) { cfg->gain_reduction_b = atoi(val); cfg->tuner_b_settings_set = 1; }
        else if (!strcmp(key, "lna_state_b"))    { cfg->lna_state_b   = atoi(val); cfg->tuner_b_settings_set = 1; }
        else if (!strcmp(key, "agc_enable_b"))   { cfg->agc_enable_b  = atoi(val); cfg->tuner_b_settings_set = 1; }
        else if (!strcmp(key, "dc_correct_b"))   { cfg->dc_correct_b  = atoi(val); cfg->tuner_b_settings_set = 1; }
        else if (!strcmp(key, "iq_correct_b"))   { cfg->iq_correct_b  = atoi(val); cfg->tuner_b_settings_set = 1; }
        else if (!strcmp(key, "notch_rf_b"))     { cfg->notch_rf_b    = atoi(val); cfg->tuner_b_settings_set = 1; }
        else if (!strcmp(key, "notch_dab_b"))    { cfg->notch_dab_b   = atoi(val); cfg->tuner_b_settings_set = 1; }
        else if (!strcmp(key, "start_time") ||
                 !strcmp(key, "start_time_utc")) strncpy(cfg->start_time, val, 15);
        else if (!strcmp(key, "spinup_enable"))        cfg->spinup_enable = atoi(val);
        else if (!strcmp(key, "spinup_bytes")) {
            /* Accept plain integers or values with KB/MB suffix,
             * e.g. "1 MB", "4MB", "512 KB", "1048576"              */
            char *end;
            long v = strtol(val, &end, 10);
            while (*end == ' ') end++;
            if (_strnicmp(end, "MB", 2) == 0)
                v *= 1024 * 1024;
            else if (_strnicmp(end, "KB", 2) == 0)
                v *= 1024;
            if (v > 0)
                cfg->spinup_bytes = (int)v;
        }
        else if (!strcmp(key, "pipe_enable"))        cfg->pipe_enable = atoi(val);
        else if (!strcmp(key, "pipe_name"))          strncpy(cfg->pipe_name, val, 127);
        else if (!strcmp(key, "http_port"))          cfg->http_port = atoi(val);
        else if (!strcmp(key, "http_interval_ms")) {
            int v = atoi(val);
            if (v >= 500 && v <= 30000)
                cfg->http_interval_ms = v;
            else
                LOG_WARN("http_interval_ms %d out of range (500-30000) - "
                         "using default 2000 ms.", v);
        }
        else if (!strcmp(key, "schedule_only"))      cfg->schedule_only = atoi(val);
        else if (!strcmp(key, "use_utc"))            cfg->use_utc = atoi(val);
        else if (!strcmp(key, "show_clock"))         cfg->show_clock = atoi(val);
        else if (!strcmp(key, "meter_style"))        cfg->meter_style = atoi(val);
        else if (!strcmp(key, "hourly_enable"))      cfg->hourly_enable = atoi(val);
        else if (!strcmp(key, "hourly_window_min"))  cfg->hourly_window_min = atoi(val);
        else if (!strcmp(key, "hourly_start")) strncpy(cfg->hourly_start, val, 7);
        else if (!strcmp(key, "hourly_stop"))  strncpy(cfg->hourly_stop,  val, 7);
        else if (!strcmp(key, "window_x"))          cfg->window_x = atoi(val);
        else if (!strcmp(key, "window_y"))          cfg->window_y = atoi(val);
        else if (!strcmp(key, "window_w"))          cfg->window_w = atoi(val);
        else if (!strcmp(key, "window_h"))          cfg->window_h = atoi(val);
        else if (!strcmp(key, "window_maximized"))  cfg->window_maximized = atoi(val);
        else if (!strcmp(key, "color_scheme"))
            strncpy(cfg->color_scheme, val, sizeof(cfg->color_scheme) - 1);
        else if (!strcmp(key, "timer_last_mode"))
            strncpy(cfg->timer_last_mode, val, sizeof(cfg->timer_last_mode) - 1);
        /* ── Schedule entries: schedule_N_key = value ──────────────────
         * e.g. schedule_1_start_time  = 08:00:00
         *      schedule_1_duration    = 3600
         *      schedule_1_frequency   = 1.125
         *      schedule_1_freq_b      = 0.900
         *      schedule_1_output_file = morning_session.raw          */
        else if (strncmp(key, "schedule_", 9) == 0) {
            const char *rest = key + 9;
            char *underscore = strchr(rest, '_');
            if (underscore) {
                int idx = atoi(rest) - 1;  /* 1-based in ini, 0-based here */
                const char *field = underscore + 1;
                if (idx >= 0 && idx < MAX_SCHEDULE_ENTRIES) {
                    if (idx >= cfg->schedule_count)
                        cfg->schedule_count = idx + 1;
                    ScheduleEntry *e = &cfg->schedule[idx];
                    if (!strcmp(field, "start_time"))
                        strncpy(e->start_time, val, 15);
                    else if (!strcmp(field, "duration"))
                        e->duration_sec = parse_duration_hms(val);
                    else if (!strcmp(field, "frequency"))
                        e->frequency_hz = atof(val) * 1e6;
                    else if (!strcmp(field, "freq_b"))
                        e->freq_b_hz = atof(val) * 1e6;
                    else if (!strcmp(field, "output_file"))
                        strncpy(e->output_file, val, MAX_PATH_LEN - 1);
                    else if (!strcmp(field, "antenna"))
                        strncpy(e->antenna, val, 7);
                }
            }
        }
        else if (!strcmp(key, "ring_buffer_sec"))cfg->ring_buffer_sec= atoi(val);
        else if (!strcmp(key, "monitor_interval_ms")) {
            int v = atoi(val);
            if (v >= 100 && v <= 5000)
                cfg->monitor_interval_ms = v;
            else
                LOG_WARN("monitor_interval_ms %d out of range (100-5000) - "
                         "using default %d ms", v, DEFAULT_MONITOR_INTERVAL_MS);
        }
        else if (!strcmp(key, "log_file"))       strncpy(cfg->log_file, val, MAX_PATH_LEN-1);
        else if (!strcmp(key, "log_auto_save"))  cfg->log_auto_save  = atoi(val);
        else if (!strcmp(key, "verbose"))        cfg->verbose        = atoi(val);
        else if (!strcmp(key, "antenna"))       strncpy(cfg->antenna, val, sizeof(cfg->antenna)-1);
        else if (!strcmp(key, "bias_t"))        cfg->bias_t         = atoi(val);
        else if (!strcmp(key, "hiz_notch"))     cfg->hiz_notch      = atoi(val);
        else if (!strcmp(key, "hdr_enable"))    cfg->hdr_enable     = atoi(val);
        else if (!strcmp(key, "hdr_bw_khz"))    cfg->hdr_bw_khz     = atoi(val);
        else if (!strcmp(key, "ppm"))           cfg->ppm            = atof(val);
        else if (!strcmp(key, "device_serial")) strncpy(cfg->device_serial, val, 63);
        else if (!strcmp(key, "decimation"))    cfg->decimation     = atoi(val);
        else if (!strcmp(key, "output_format")) {
            if (!strcmp(val, "wavviewdx") || !strcmp(val, "WavViewDX"))
                cfg->output_format = FORMAT_WAVVIEWDX;
            else if (!strcmp(val, "sdruno") || !strcmp(val, "SDRuno"))
                cfg->output_format = FORMAT_SDRUNO;
            else if (!strcmp(val, "sdrconnect") || !strcmp(val, "SDRConnect"))
                cfg->output_format = FORMAT_SDRCONNECT;
            else if (!strcmp(val, "winrad") || !strcmp(val, "Winrad"))
                cfg->output_format = FORMAT_WINRAD;
            else
                cfg->output_format = FORMAT_LINRAD;
        }
        else if (!strcmp(key, "large_file_mode")) {
            int m = atoi(val);
            cfg->large_file_mode = (m == LARGE_FILE_RF64)
                                      ? LARGE_FILE_RF64 : LARGE_FILE_SPLIT;
        }
        else if (!strcmp(key, "latitude"))
            cfg->latitude = atof(val);
        else if (!strcmp(key, "longitude"))
            cfg->longitude = atof(val);
        else if (!strcmp(key, "show_sun_times"))
            cfg->show_sun_times = atoi(val) ? 1 : 0;
        else
            LOG_WARN("Unknown config key: '%s'", key);
    }

    fclose(fp);
}


/* =========================================================================
 * SDRplay IF bandwidth selector
 * ========================================================================= */
static sdrplay_api_Bw_MHzT select_bandwidth(int bw_khz)
{
    if (bw_khz <= 200)  return sdrplay_api_BW_0_200;
    if (bw_khz <= 300)  return sdrplay_api_BW_0_300;
    if (bw_khz <= 600)  return sdrplay_api_BW_0_600;
    if (bw_khz <= 1536) return sdrplay_api_BW_1_536;
    if (bw_khz <= 5000) return sdrplay_api_BW_5_000;
    if (bw_khz <= 6000) return sdrplay_api_BW_6_000;
    if (bw_khz <= 7000) return sdrplay_api_BW_7_000;
    return sdrplay_api_BW_8_000;
}

/* =========================================================================
 * SDRplay IF frequency selector
 * ========================================================================= */
static sdrplay_api_If_kHzT select_if(int if_khz)
{
    switch (if_khz) {
    case 450:  return sdrplay_api_IF_0_450;
    case 1620: return sdrplay_api_IF_1_620;
    case 2048: return sdrplay_api_IF_2_048;
    default:   return sdrplay_api_IF_Zero;
    }
}

/* =========================================================================
 * Device setup - single channel
 * ========================================================================= */
static int setup_device_single(AppState *state)
{
    Config *cfg = &state->cfg;
    sdrplay_api_RxChannelParamsT *ch = state->ch_a_params;
    sdrplay_api_DevParamsT *dev = state->dev_params->devParams;

    /* Sample rate */
    dev->fsFreq.fsHz = cfg->sample_rate_hz;

    /* PPM frequency correction (0.0 = no correction; not needed with GPSDO) */
    dev->ppm = cfg->ppm;

    /* Tuner A frequency */
    ch->tunerParams.rfFreq.rfHz = cfg->frequency_hz;

    /* IF bandwidth */
    ch->tunerParams.bwType = select_bandwidth(cfg->bw_khz);

    /* IF frequency */
    ch->tunerParams.ifType = select_if(cfg->if_khz);

    /* Gain */
    ch->tunerParams.gain.gRdB   = cfg->gain_reduction;
    ch->tunerParams.gain.LNAstate = cfg->lna_state;

    /* AGC */
    ch->ctrlParams.agc.enable = cfg->agc_enable ?
        sdrplay_api_AGC_CTRL_EN : sdrplay_api_AGC_DISABLE;
    ch->ctrlParams.agc.setPoint_dBfs = cfg->agc_setpoint_dbfs;
    ch->ctrlParams.agc.attack_ms     = (unsigned short)cfg->agc_attack_ms;
    ch->ctrlParams.agc.decay_ms      = (unsigned short)cfg->agc_decay_ms;

    /* DC offset correction */
    ch->ctrlParams.dcOffset.DCenable = cfg->dc_correct ? 1 : 0;
    ch->ctrlParams.dcOffset.IQenable = cfg->iq_correct ? 1 : 0;

    /* Software decimation (additional factor on top of any hardware decimation) */
    if (cfg->decimation > 1) {
        ch->ctrlParams.decimation.enable         = 1;
        ch->ctrlParams.decimation.decimationFactor = (unsigned char)cfg->decimation;
        ch->ctrlParams.decimation.wideBandSignal  = 0;
    }

    /* Antenna selection and Bias-T for RSPdx/R2 must be set before Init */
    if (state->device.hwVer == SDRPLAY_RSPdx_ID ||
        state->device.hwVer == SDRPLAY_RSPdxR2_ID) {
        sdrplay_api_DevParamsT *dp = state->dev_params->devParams;
        if      (!strcmp(cfg->antenna, "B")) dp->rspDxParams.antennaSel = sdrplay_api_RspDx_ANTENNA_B;
        else if (!strcmp(cfg->antenna, "C")) dp->rspDxParams.antennaSel = sdrplay_api_RspDx_ANTENNA_C;
        else                                 dp->rspDxParams.antennaSel = sdrplay_api_RspDx_ANTENNA_A;
        if (cfg->bias_t)
            dp->rspDxParams.biasTEnable = 1;
    }

    /* NOTE: Notch filters are applied after sdrplay_api_Init() via
     * sdrplay_api_Update(). Antenna/BiasT for non-RSPdx devices are
     * also applied post-Init via apply_antenna_and_biast(). */

    return 0;
}

/* -------------------------------------------------------------------------
 * setup_slave_channel_b - RSPduo Slave session only.
 *
 * Deliberately does NOT touch state->dev_params->devParams (sample rate,
 * ppm, device-level antenna/HDR settings) - the SDRplay API documents that
 * device-level parameters in Master/Slave mode are the Master's
 * responsibility only. That pointer may not even be valid for a Slave
 * session; setup_device_single() dereferences it unconditionally, which is
 * exactly what was crashing the slave process silently (nothing logged
 * after channel selection - a NULL dereference, not a clean API error).
 * This sets only Tuner B's own channel-level fields.
 * ------------------------------------------------------------------------- */
static void setup_slave_channel_b(AppState *state)
{
    Config *cfg = &state->cfg;
    sdrplay_api_RxChannelParamsT *ch = state->ch_a_params;

    ch->tunerParams.rfFreq.rfHz    = cfg->frequency_hz;
    ch->tunerParams.bwType         = select_bandwidth(cfg->bw_khz);
    ch->tunerParams.ifType         = select_if(cfg->if_khz);
    ch->tunerParams.gain.gRdB      = cfg->gain_reduction;
    ch->tunerParams.gain.LNAstate  = cfg->lna_state;
    ch->ctrlParams.agc.enable      = cfg->agc_enable ?
        sdrplay_api_AGC_CTRL_EN : sdrplay_api_AGC_DISABLE;
    ch->ctrlParams.agc.setPoint_dBfs = cfg->agc_setpoint_dbfs;
    ch->ctrlParams.agc.attack_ms     = (unsigned short)cfg->agc_attack_ms;
    ch->ctrlParams.agc.decay_ms      = (unsigned short)cfg->agc_decay_ms;
    ch->ctrlParams.dcOffset.DCenable = cfg->dc_correct ? 1 : 0;
    ch->ctrlParams.dcOffset.IQenable = cfg->iq_correct ? 1 : 0;

    if (cfg->decimation > 1) {
        ch->ctrlParams.decimation.enable          = 1;
        ch->ctrlParams.decimation.decimationFactor = (unsigned char)cfg->decimation;
        ch->ctrlParams.decimation.wideBandSignal   = 0;
    }
}

/* =========================================================================
 * Device setup - RSPduo dual channel
 * ========================================================================= */
static int setup_device_rspduo_dual(AppState *state)
{
    Config *cfg = &state->cfg;
    sdrplay_api_DevParamsT *dev = state->dev_params->devParams;
    sdrplay_api_RxChannelParamsT *chA = state->ch_a_params;
    sdrplay_api_RxChannelParamsT *chB = state->ch_b_params;

    /* Sample rate (shared for both tuners in dual mode) */
    dev->fsFreq.fsHz = cfg->sample_rate_hz;

    /* PPM correction (applied device-wide) */
    dev->ppm = cfg->ppm;

    /* --- Tuner A --- */
    chA->tunerParams.rfFreq.rfHz  = cfg->frequency_hz;
    chA->tunerParams.bwType       = select_bandwidth(cfg->bw_khz);
    chA->tunerParams.ifType       = select_if(cfg->if_khz);
    chA->tunerParams.gain.gRdB    = cfg->gain_reduction;
    chA->tunerParams.gain.LNAstate = cfg->lna_state;
    chA->ctrlParams.agc.enable    = cfg->agc_enable ?
        sdrplay_api_AGC_CTRL_EN : sdrplay_api_AGC_DISABLE;
    chA->ctrlParams.agc.setPoint_dBfs = cfg->agc_setpoint_dbfs;
    chA->ctrlParams.agc.attack_ms     = (unsigned short)cfg->agc_attack_ms;
    chA->ctrlParams.agc.decay_ms      = (unsigned short)cfg->agc_decay_ms;
    chA->ctrlParams.dcOffset.DCenable = cfg->dc_correct ? 1 : 0;
    chA->ctrlParams.dcOffset.IQenable = cfg->iq_correct ? 1 : 0;

    /* --- Tuner B ---
     * Use per-tuner B overrides where set; fall back to Tuner A values.
     * A value of -1 means "not explicitly set - use Tuner A value".       */
    {
        int gr_b  = (cfg->gain_reduction_b >= 0) ? cfg->gain_reduction_b : cfg->gain_reduction;
        int lna_b = (cfg->lna_state_b      >= 0) ? cfg->lna_state_b      : cfg->lna_state;
        int agc_b = (cfg->agc_enable_b     >= 0) ? cfg->agc_enable_b     : cfg->agc_enable;
        int dc_b  = (cfg->dc_correct_b     >= 0) ? cfg->dc_correct_b     : cfg->dc_correct;
        int iq_b  = (cfg->iq_correct_b     >= 0) ? cfg->iq_correct_b     : cfg->iq_correct;

        chB->tunerParams.rfFreq.rfHz   = cfg->freq_b_hz;
        chB->tunerParams.bwType        = select_bandwidth(cfg->bw_khz);
        chB->tunerParams.ifType        = select_if(cfg->if_khz);
        chB->tunerParams.gain.gRdB     = gr_b;
        chB->tunerParams.gain.LNAstate = (unsigned char)lna_b;
        chB->ctrlParams.agc.enable     = agc_b ?
            sdrplay_api_AGC_CTRL_EN : sdrplay_api_AGC_DISABLE;
        chB->ctrlParams.agc.setPoint_dBfs = cfg->agc_setpoint_dbfs;
        chB->ctrlParams.agc.attack_ms     = (unsigned short)cfg->agc_attack_ms;
        chB->ctrlParams.agc.decay_ms      = (unsigned short)cfg->agc_decay_ms;
        chB->ctrlParams.dcOffset.DCenable = dc_b ? 1 : 0;
        chB->ctrlParams.dcOffset.IQenable = iq_b ? 1 : 0;
    }

    /* Software decimation applied to both channels */
    if (cfg->decimation > 1) {
        chA->ctrlParams.decimation.enable          = 1;
        chA->ctrlParams.decimation.decimationFactor = (unsigned char)cfg->decimation;
        chA->ctrlParams.decimation.wideBandSignal   = 0;
        chB->ctrlParams.decimation.enable          = 1;
        chB->ctrlParams.decimation.decimationFactor = (unsigned char)cfg->decimation;
        chB->ctrlParams.decimation.wideBandSignal   = 0;
    }

    /* RSPduo master/slave mode */
    state->dev_params->devParams->rspDuoParams.extRefOutputEn = 0;

    return 0;
}

/* =========================================================================
 * Apply HDR mode - RSPdx and RSPdx R2 only
 *
 * HDR (High Dynamic Range) mode improves intermodulation performance and
 * reduces spurious responses for frequencies below 2 MHz.
 * HDR bandwidth options:
 *   200 kHz  - LF/VLF, NDB bands
 *   500 kHz  - 630m, 160m amateur bands
 *  1200 kHz  - LW + lower MW
 *  1700 kHz  - Full MW band (recommended)
 *
 * Valid HDR centre frequencies (kHz): 135, 175, 220, 250, 340, 475, 516,
 * 875, 1125, 1900. Other frequencies cause the hardware to snap to the
 * nearest valid LO, producing a display frequency offset.
 * ========================================================================= */
static void apply_hdr_mode(AppState *state)
{
    sdrplay_api_RspDx_HdrModeBwT bw_enum;
    Config *cfg = &state->cfg;
    unsigned char hw = state->device.hwVer;

    if (!cfg->hdr_enable)
        return;

    if (hw != SDRPLAY_RSPdx_ID && hw != SDRPLAY_RSPdxR2_ID) {
        LOG_WARN("HDR mode requested but device is not an RSPdx or RSPdx R2 "
                 "(hwVer=%u) - HDR ignored", hw);
        cfg->hdr_enable = 0;
        return;
    }

    static const double hdr_valid_hz[] = {
        135e3, 175e3, 220e3, 250e3, 340e3,
        475e3, 516e3, 875e3, 1125e3, 1900e3
    };
    int n = (int)(sizeof(hdr_valid_hz) / sizeof(hdr_valid_hz[0]));
    int valid = 0;
    for (int i = 0; i < n; i++) {
        if (cfg->frequency_hz == hdr_valid_hz[i]) { valid = 1; break; }
    }
    if (!valid) {
        LOG_ERROR("HDR mode requires one of these specific frequencies:");
        LOG_ERROR("  135, 175, 220, 250, 340, 475, 516, 875, 1125, 1900 kHz");
        LOG_ERROR("Requested %.0f kHz is not valid for HDR - "
                  "use 1125 kHz for full MW band coverage.",
                  cfg->frequency_hz / 1e3);
        cfg->hdr_enable = 0;
        return;
    }

    if (cfg->sample_rate_hz != 6000000.0) {
        LOG_ERROR("HDR mode requires sample_rate_msps=6.0 - HDR disabled.");
        cfg->hdr_enable = 0;
        return;
    }
    if (cfg->if_khz != 1620) {
        LOG_ERROR("HDR mode requires if_khz=1620 - HDR disabled.");
        cfg->hdr_enable = 0;
        return;
    }

    if      (cfg->hdr_bw_khz <= 200)  bw_enum = sdrplay_api_RspDx_HDRMODE_BW_0_200;
    else if (cfg->hdr_bw_khz <= 500)  bw_enum = sdrplay_api_RspDx_HDRMODE_BW_0_500;
    else if (cfg->hdr_bw_khz <= 1200) bw_enum = sdrplay_api_RspDx_HDRMODE_BW_1_200;
    else                               bw_enum = sdrplay_api_RspDx_HDRMODE_BW_1_700;

    state->dev_params->devParams->rspDxParams.hdrEnable = 1;
    state->ch_a_params->rspDxTunerParams.hdrBw          = bw_enum;
}

/* =========================================================================
 * Apply antenna selection, Bias-T, and Hi-Z notch after streaming starts.
 *
 * Antenna inputs by device:
 *
 *   RSPdx / RSPdx R2:
 *     antenna = "A"  -> sdrplay_api_RspDx_ANTENNA_A  (SMA, wideband)
 *     antenna = "B"  -> sdrplay_api_RspDx_ANTENNA_B  (SMA, wideband)
 *     antenna = "C"  -> sdrplay_api_RspDx_ANTENNA_C  (SMA, < 200 MHz only)
 *     Bias-T available on all three ports.
 *     Update flag: sdrplay_api_Update_RspDx_AntennaControl (Ext1)
 *                  sdrplay_api_Update_RspDx_BiasTControl   (Ext1)
 *
 *   RSP2:
 *     antenna = "A"  -> sdrplay_api_Rsp2_ANTENNA_A  (SMA)
 *     antenna = "B"  -> sdrplay_api_Rsp2_ANTENNA_B  (SMA)
 *     (Hi-Z AM port on RSP2 is selected via mir_sdr_AmPortSelect - not in v3)
 *     Update flag: sdrplay_api_Update_Rsp2_AntennaControl
 *
 *   RSPduo Tuner 1:
 *     antenna = "Hi-Z"   -> sdrplay_api_RspDuo_AMPORT_1 (Hi-Z, MW optimised)
 *     antenna = "50ohm"  -> sdrplay_api_RspDuo_AMPORT_2 (50 ohm SMA, default)
 *     Bias-T: only on Tuner 2 (50 ohm port).
 *     Hi-Z AM notch (hiz_notch=1): suppresses MW broadcast when on Hi-Z.
 *     Update flag: sdrplay_api_Update_RspDuo_AmPortSelect
 *                  sdrplay_api_Update_RspDuo_BiasTControl
 *                  sdrplay_api_Update_RspDuo_Tuner1AmNotchControl
 *
 *   RSP1A / RSP1B:
 *     Single antenna input - no selection needed.
 *     Bias-T available.
 *     Update flag: sdrplay_api_Update_Rsp1a_BiasTControl
 * ========================================================================= */
static void apply_antenna_and_biast(AppState *state)
{
    Config *cfg = &state->cfg;
    sdrplay_api_ErrT err;
    unsigned char hw = state->device.hwVer;
    sdrplay_api_RxChannelParamsT *chA = state->ch_a_params;
    sdrplay_api_RxChannelParamsT *chB = state->ch_b_params;

    /* ── RSPdx / RSPdx R2 ─────────────────────────────────────────────── */
    /* Antenna is also set into dev_params before the very first Init in
     * setup_device_single, so that first call already has the right
     * antenna selected the moment the device starts. This call pushes
     * it live via sdrplay_api_Update using the Ext1 AntennaControl
     * reason documented above - which matters when Settings is saved
     * again while already listening (previously antenna silently had no
     * effect until Monitor was stopped and restarted, unlike gain/LNA).
     * Not yet bench-verified against real RSPdx hardware for the
     * post-Init case specifically - if the API rejects it or it doesn't
     * actually switch, that'll show up as a WARN below rather than
     * silently doing nothing, and live_antenna (and so the status line)
     * only updates once this call actually reports success.            */
    if (hw == SDRPLAY_RSPdx_ID || hw == SDRPLAY_RSPdxR2_ID) {
        sdrplay_api_DevParamsT *dp = state->dev_params->devParams;
        if      (!strcmp(cfg->antenna, "B")) dp->rspDxParams.antennaSel = sdrplay_api_RspDx_ANTENNA_B;
        else if (!strcmp(cfg->antenna, "C")) dp->rspDxParams.antennaSel = sdrplay_api_RspDx_ANTENNA_C;
        else                                 dp->rspDxParams.antennaSel = sdrplay_api_RspDx_ANTENNA_A;

        err = sdrplay_api_Update(state->device.dev, sdrplay_api_Tuner_A,
                                 sdrplay_api_Update_None,
                                 sdrplay_api_Update_RspDx_AntennaControl);
        if (err != sdrplay_api_Success) {
            /* Brief settling window right after a listening-to-recording
             * transition (e.g. an hourly/schedule repeat cycle, where the
             * device has been running continuously but just started a
             * fresh file) can spuriously report NotInitialised here even
             * though the device is genuinely streaming fine - same class
             * of SDRplay API timing quirk gui_apply_live_gain() already
             * retries around. One retry after a short sleep before
             * actually warning about it.                                 */
            Sleep(120);
            err = sdrplay_api_Update(state->device.dev, sdrplay_api_Tuner_A,
                                     sdrplay_api_Update_None,
                                     sdrplay_api_Update_RspDx_AntennaControl);
        }
        if (err != sdrplay_api_Success) {
            LOG_WARN("RSPdx antenna select failed: %s", sdrplay_api_GetErrorString(err));
        } else {
            if (cfg->verbose)
                LOG_INFO("RSPdx antenna set to: %s%s", cfg->antenna,
                         cfg->bias_t ? "  Bias-T: enabled" : "");
            strncpy(state->live_antenna, cfg->antenna, sizeof(state->live_antenna) - 1);
            state->live_antenna[sizeof(state->live_antenna) - 1] = '\0';
        }
        return;
    }

    /* ── RSP2 ──────────────────────────────────────────────────────────── */
    if (hw == SDRPLAY_RSP2_ID) {
        chA->rsp2TunerParams.antennaSel = (!strcmp(cfg->antenna, "B"))
            ? sdrplay_api_Rsp2_ANTENNA_B
            : sdrplay_api_Rsp2_ANTENNA_A;
        err = sdrplay_api_Update(state->device.dev, state->device.tuner,
                                 sdrplay_api_Update_Rsp2_AntennaControl,
                                 sdrplay_api_Update_Ext1_None);
        if (err != sdrplay_api_Success) {
            LOG_WARN("RSP2 antenna select failed: %s", sdrplay_api_GetErrorString(err));
        } else {
            LOG_INFO("RSP2 antenna set to: %s", cfg->antenna);
            strncpy(state->live_antenna, cfg->antenna, sizeof(state->live_antenna) - 1);
            state->live_antenna[sizeof(state->live_antenna) - 1] = '\0';
        }

        if (cfg->bias_t)
            LOG_WARN("RSP2 does not support Bias-T - bias_t ignored");
        return;
    }

    /* ── RSPduo ────────────────────────────────────────────────────────── */
    if (hw == SDRPLAY_RSPduo_ID) {

        /* Tuner 1 AM port (antenna) selection - and the Hi-Z notch below -
         * are genuinely Tuner-1-only hardware, not just a Tuner-A default:
         * the API's own field names (tuner1AmPortSel, tuner1AmNotchEnable)
         * say as much. Tuner 2 has a single fixed 50-ohm SMA input with no
         * port-select relay at all, so none of this applies when Tuner B
         * is the active single tuner - warn only if the user configured
         * something that assumes Tuner 1's Hi-Z port, rather than
         * silently ignoring it.                                          */
        if (state->device.tuner == sdrplay_api_Tuner_B) {
            int wanted_hiz = (!strcmp(cfg->antenna, "Hi-Z") ||
                              !strcmp(cfg->antenna, "hi-z") ||
                              !strcmp(cfg->antenna, "HIZ"));
            if (wanted_hiz)
                LOG_WARN("antenna=Hi-Z ignored - Tuner 2 (B) has no Hi-Z "
                         "port, only Tuner 1 (A) does.");
            if (cfg->hiz_notch)
                LOG_WARN("hiz_notch=1 ignored - only meaningful on Tuner "
                         "1 (A)'s Hi-Z port.");
            strncpy(state->live_antenna, "50ohm", sizeof(state->live_antenna) - 1);
            state->live_antenna[sizeof(state->live_antenna) - 1] = '\0';
        } else {
        int use_hiz = (!strcmp(cfg->antenna, "Hi-Z") ||
                       !strcmp(cfg->antenna, "hi-z") ||
                       !strcmp(cfg->antenna, "HIZ"));

        chA->rspDuoTunerParams.tuner1AmPortSel = use_hiz
            ? sdrplay_api_RspDuo_AMPORT_1   /* Hi-Z */
            : sdrplay_api_RspDuo_AMPORT_2;  /* 50 ohm (default) */

        err = sdrplay_api_Update(state->device.dev, state->device.tuner,
                                 sdrplay_api_Update_RspDuo_AmPortSelect,
                                 sdrplay_api_Update_Ext1_None);
        if (err != sdrplay_api_Success) {
            LOG_WARN("RSPduo AM port select failed: %s", sdrplay_api_GetErrorString(err));
        } else {
            if (cfg->verbose)
                LOG_INFO("RSPduo Tuner 1 port: %s", use_hiz ? "Hi-Z" : "50 ohm");
            strncpy(state->live_antenna, cfg->antenna, sizeof(state->live_antenna) - 1);
            state->live_antenna[sizeof(state->live_antenna) - 1] = '\0';
        }

        /* Hi-Z AM notch - only applies when Tuner 1 is on the Hi-Z port */
        if (cfg->hiz_notch) {
            if (!use_hiz) {
                LOG_WARN("hiz_notch=1 ignored - Tuner 1 is on 50 ohm port, not Hi-Z");
            } else {
                chA->rspDuoTunerParams.tuner1AmNotchEnable = 1;
                err = sdrplay_api_Update(state->device.dev, state->device.tuner,
                                         sdrplay_api_Update_RspDuo_Tuner1AmNotchControl,
                                         sdrplay_api_Update_Ext1_None);
                if (err != sdrplay_api_Success)
                    LOG_WARN("RSPduo Hi-Z notch failed: %s", sdrplay_api_GetErrorString(err));
                else
                    LOG_INFO("RSPduo Hi-Z AM notch enabled");
            }
        }
        }

        /* Bias-T on Tuner 2 (50 ohm port only) - reachable two ways: real
         * dual-channel mode (chB populated), or single-tuner mode with B
         * selected (chB stays NULL there regardless of which physical
         * tuner is active, same as everywhere else in this function -
         * Tuner B's real channel struct is chA, aliased, in that case). */
        if (cfg->bias_t) {
            if (chB != NULL) {
                chB->rspDuoTunerParams.biasTEnable = 1;
                err = sdrplay_api_Update(state->device.dev,
                                         sdrplay_api_Tuner_B,
                                         sdrplay_api_Update_RspDuo_BiasTControl,
                                         sdrplay_api_Update_Ext1_None);
                if (err != sdrplay_api_Success)
                    LOG_WARN("RSPduo Bias-T failed: %s", sdrplay_api_GetErrorString(err));
                else
                    if (cfg->verbose)
                        LOG_INFO("RSPduo Tuner 2 Bias-T enabled");
            } else if (state->device.tuner == sdrplay_api_Tuner_B) {
                chA->rspDuoTunerParams.biasTEnable = 1;
                err = sdrplay_api_Update(state->device.dev,
                                         state->device.tuner,
                                         sdrplay_api_Update_RspDuo_BiasTControl,
                                         sdrplay_api_Update_Ext1_None);
                if (err != sdrplay_api_Success)
                    LOG_WARN("RSPduo Bias-T failed: %s", sdrplay_api_GetErrorString(err));
                else
                    if (cfg->verbose)
                        LOG_INFO("RSPduo Tuner 2 Bias-T enabled");
            } else {
                LOG_WARN("RSPduo Bias-T requires dual-channel mode or Tuner 2 active - "
                         "Tuner 1 has no Bias-T capability.");
            }
        }
        return;
    }

    /* ── RSP1A / RSP1B ─────────────────────────────────────────────────── */
    if (hw == SDRPLAY_RSP1A_ID) {
        if (strcmp(cfg->antenna, "A") != 0)
            LOG_WARN("RSP1A has only one antenna input - antenna=%s ignored",
                     cfg->antenna);
        strncpy(state->live_antenna, "A", sizeof(state->live_antenna) - 1);
        state->live_antenna[sizeof(state->live_antenna) - 1] = '\0';
        if (cfg->bias_t) {
            chA->rsp1aTunerParams.biasTEnable = 1;
            err = sdrplay_api_Update(state->device.dev, state->device.tuner,
                                     sdrplay_api_Update_Rsp1a_BiasTControl,
                                     sdrplay_api_Update_Ext1_None);
            if (err != sdrplay_api_Success)
                LOG_WARN("RSP1A Bias-T failed: %s", sdrplay_api_GetErrorString(err));
            else
                LOG_INFO("RSP1A Bias-T enabled");
        }
        return;
    }

    /* RSP1 - no antenna selection, no Bias-T */
    if (strcmp(cfg->antenna, "A") != 0)
        LOG_WARN("This device has only one antenna input - antenna=%s ignored",
                 cfg->antenna);
    strncpy(state->live_antenna, "A", sizeof(state->live_antenna) - 1);
    state->live_antenna[sizeof(state->live_antenna) - 1] = '\0';
    if (cfg->bias_t)
        LOG_WARN("This device does not support Bias-T - bias_t ignored");
}

/* -------------------------------------------------------------------------
 * apply_slave_biast_b - RSPduo Slave (Tuner B) session only.
 *
 * Tuner B has no AM port selector or Hi-Z notch - those are Tuner 1-only
 * concepts (Tuner B is always the plain 50-ohm SMA port). The generic
 * apply_antenna_and_biast() unconditionally applies the Tuner 1 AM port
 * setting to whatever ch_a_params points at, which for a Slave session is
 * actually Tuner B's own channel struct - producing a harmless but
 * confusing sdrplay_api_OutOfRange warning. Only Bias-T applies here.
 * ------------------------------------------------------------------------- */
static void apply_slave_biast_b(AppState *state)
{
    Config *cfg = &state->cfg;
    if (cfg->bias_t) {
        sdrplay_api_ErrT err;
        state->ch_a_params->rspDuoTunerParams.biasTEnable = 1;
        err = sdrplay_api_Update(state->device.dev, sdrplay_api_Tuner_B,
                                 sdrplay_api_Update_RspDuo_BiasTControl,
                                 sdrplay_api_Update_Ext1_None);
        if (err != sdrplay_api_Success)
            LOG_WARN("Slave: RSPduo Tuner 2 Bias-T failed: %s",
                     sdrplay_api_GetErrorString(err));
        else
            LOG_INFO("Slave: RSPduo Tuner 2 Bias-T enabled");
    }
}

/* =========================================================================
 * Apply notch filters after streaming has started
 *
 * Field names and struct paths confirmed from installed API headers:
 *
 *   RSP1A  : devParams->rsp1aParams.rfNotchEnable / rfDabNotchEnable
 *             update flag: sdrplay_api_Update_Rsp1a_RfNotchControl
 *                          sdrplay_api_Update_Rsp1a_RfDabNotchControl
 *
 *   RSP2   : ch->rsp2TunerParams.rfNotchEnable (RF only, no DAB)
 *             update flag: sdrplay_api_Update_Rsp2_RfNotchControl
 *
 *   RSPduo : ch->rspDuoTunerParams.rfNotchEnable / rfDabNotchEnable
 *             update flags: sdrplay_api_Update_RspDuo_RfNotchControl
 *                           sdrplay_api_Update_RspDuo_RfDabNotchControl
 *
 *   RSPdx  : devParams->rspDxParams.rfNotchEnable / rfDabNotchEnable
 *             update flags (Ext1): sdrplay_api_Update_RspDx_RfNotchControl
 *                                  sdrplay_api_Update_RspDx_RfDabNotchControl
 * ========================================================================= */
static void apply_notch_filters(AppState *state)
{
    Config *cfg = &state->cfg;
    sdrplay_api_ErrT err;
    unsigned char hw = state->device.hwVer;
    sdrplay_api_DevParamsT *dp = state->dev_params->devParams;

    if (!cfg->notch_rf && !cfg->notch_dab)
        return;

    /* --- RSP1A ---
     * Both notch fields live in devParams->rsp1aParams (device-level) */
    if (hw == SDRPLAY_RSP1A_ID) {
        if (cfg->notch_rf) {
            dp->rsp1aParams.rfNotchEnable = 1;
            err = sdrplay_api_Update(state->device.dev, state->device.tuner,
                                     sdrplay_api_Update_Rsp1a_RfNotchControl,
                                     sdrplay_api_Update_Ext1_None);
            if (err != sdrplay_api_Success)
                LOG_WARN("RSP1A RF notch failed: %s", sdrplay_api_GetErrorString(err));
            else
                LOG_INFO("RSP1A RF notch enabled");
        }
        if (cfg->notch_dab) {
            dp->rsp1aParams.rfDabNotchEnable = 1;
            err = sdrplay_api_Update(state->device.dev, state->device.tuner,
                                     sdrplay_api_Update_Rsp1a_RfDabNotchControl,
                                     sdrplay_api_Update_Ext1_None);
            if (err != sdrplay_api_Success)
                LOG_WARN("RSP1A DAB notch failed: %s", sdrplay_api_GetErrorString(err));
            else
                LOG_INFO("RSP1A DAB notch enabled");
        }
        return;
    }

    /* --- RSP2 ---
     * RF notch only; lives in channel tuner params */
    if (hw == SDRPLAY_RSP2_ID) {
        if (cfg->notch_rf) {
            state->ch_a_params->rsp2TunerParams.rfNotchEnable = 1;
            err = sdrplay_api_Update(state->device.dev, state->device.tuner,
                                     sdrplay_api_Update_Rsp2_RfNotchControl,
                                     sdrplay_api_Update_Ext1_None);
            if (err != sdrplay_api_Success)
                LOG_WARN("RSP2 RF notch failed: %s", sdrplay_api_GetErrorString(err));
            else
                LOG_INFO("RSP2 RF notch enabled");
        }
        if (cfg->notch_dab)
            LOG_WARN("RSP2 does not support DAB notch filter - ignored");
        return;
    }

    /* --- RSPduo ---
     * Tuner A and Tuner B notch filters are independent.              */
    if (hw == SDRPLAY_RSPduo_ID) {
        /* Resolve Tuner B notch values (fall back to A if not set) */
        int rf_b  = (cfg->notch_rf_b  >= 0) ? cfg->notch_rf_b  : cfg->notch_rf;
        int dab_b = (cfg->notch_dab_b >= 0) ? cfg->notch_dab_b : cfg->notch_dab;

        /* Primary tuner's notches (ch_a_params) - Tuner B only when
         * single-tuner mode has selected it; Tuner A in every other case,
         * including dual mode, where the Tuner B block below handles the
         * other channel via its own explicit selector.                   */
        sdrplay_api_TunerSelectT prim_tuner =
            (state->device.tuner == sdrplay_api_Tuner_B)
                ? sdrplay_api_Tuner_B : sdrplay_api_Tuner_A;
        const char *prim_name = (prim_tuner == sdrplay_api_Tuner_B) ? "B" : "A";

        if (cfg->notch_rf) {
            state->ch_a_params->rspDuoTunerParams.rfNotchEnable = 1;
            err = sdrplay_api_Update(state->device.dev, prim_tuner,
                                     sdrplay_api_Update_RspDuo_RfNotchControl,
                                     sdrplay_api_Update_Ext1_None);
            if (err != sdrplay_api_Success)
                LOG_WARN("RSPduo Tuner %s RF notch failed: %s", prim_name, sdrplay_api_GetErrorString(err));
            else
                if (cfg->verbose)
                    LOG_INFO("RSPduo Tuner %s RF notch enabled", prim_name);
        }
        if (cfg->notch_dab) {
            state->ch_a_params->rspDuoTunerParams.rfDabNotchEnable = 1;
            err = sdrplay_api_Update(state->device.dev, prim_tuner,
                                     sdrplay_api_Update_RspDuo_RfDabNotchControl,
                                     sdrplay_api_Update_Ext1_None);
            if (err != sdrplay_api_Success)
                LOG_WARN("RSPduo Tuner %s DAB notch failed: %s", prim_name, sdrplay_api_GetErrorString(err));
            else
                if (cfg->verbose)
                    LOG_INFO("RSPduo Tuner %s DAB notch enabled", prim_name);
        }
        /* Tuner B notches (only meaningful in dual-channel mode) */
        if (cfg->dual_channel && state->ch_b_params) {
            if (rf_b) {
                state->ch_b_params->rspDuoTunerParams.rfNotchEnable = 1;
                err = sdrplay_api_Update(state->device.dev, sdrplay_api_Tuner_B,
                                         sdrplay_api_Update_RspDuo_RfNotchControl,
                                         sdrplay_api_Update_Ext1_None);
                if (err != sdrplay_api_Success)
                    LOG_WARN("RSPduo T2 RF notch failed: %s", sdrplay_api_GetErrorString(err));
                else
                    if (cfg->verbose)
                        LOG_INFO("RSPduo Tuner B RF notch enabled");
            }
            if (dab_b) {
                state->ch_b_params->rspDuoTunerParams.rfDabNotchEnable = 1;
                err = sdrplay_api_Update(state->device.dev, sdrplay_api_Tuner_B,
                                         sdrplay_api_Update_RspDuo_RfDabNotchControl,
                                         sdrplay_api_Update_Ext1_None);
                if (err != sdrplay_api_Success)
                    LOG_WARN("RSPduo T2 DAB notch failed: %s", sdrplay_api_GetErrorString(err));
                else
                    if (cfg->verbose)
                        LOG_INFO("RSPduo Tuner B DAB notch enabled");
            }
        }
        return;
    }

    /* --- RSPdx / RSPdx R2 ---
     * Both notch fields in devParams->rspDxParams (device-level)
     * Update flags are in the Ext1 parameter, not the main flags word */
    if (hw == SDRPLAY_RSPdx_ID || hw == SDRPLAY_RSPdxR2_ID) {
        if (cfg->notch_rf) {
            dp->rspDxParams.rfNotchEnable = 1;
            err = sdrplay_api_Update(state->device.dev, state->device.tuner,
                                     sdrplay_api_Update_None,
                                     sdrplay_api_Update_RspDx_RfNotchControl);
            if (err != sdrplay_api_Success)
                LOG_WARN("RSPdx RF notch failed: %s", sdrplay_api_GetErrorString(err));
            else
                LOG_INFO("RSPdx RF notch enabled");
        }
        if (cfg->notch_dab) {
            dp->rspDxParams.rfDabNotchEnable = 1;
            err = sdrplay_api_Update(state->device.dev, state->device.tuner,
                                     sdrplay_api_Update_None,
                                     sdrplay_api_Update_RspDx_RfDabNotchControl);
            if (err != sdrplay_api_Success)
                LOG_WARN("RSPdx DAB notch failed: %s", sdrplay_api_GetErrorString(err));
            else
                LOG_INFO("RSPdx DAB notch enabled");
        }
        return;
    }

    LOG_WARN("Notch filters not supported on this device (hwVer=%u)", hw);
}

/* =========================================================================
 * Scheduled start - wait until a specified UTC time before proceeding.
 * Displays a countdown line updated every second.
 * Returns 0 if cancelled (Ctrl+C), 1 if target time reached.
 * ========================================================================= */
/* Parses "HH:MM:SS" into total seconds. Returns 0 (unlimited) for blank,
 * "00:00:00", or anything that doesn't parse as three colon-separated
 * numbers - a malformed value failing safe to "unlimited" is far less
 * surprising than it failing safe to some arbitrary numeric duration. */
static int parse_duration_hms(const char *str)
{
    int h = 0, m = 0, s = 0;
    if (!str || !str[0]) return 0;
    if (sscanf(str, "%d:%d:%d", &h, &m, &s) != 3) return 0;
    if (h < 0) h = 0;
    if (m < 0) m = 0;
    if (s < 0) s = 0;
    return h * 3600 + m * 60 + s;
}

/* Formats total seconds back into canonical "HH:MM:SS", used to write
 * the duration/schedule_N_duration ini keys and to populate the
 * Settings dialog's Duration field from the loaded config. */
static void format_duration_hms(int total_sec, char *out, size_t out_size)
{
    int h, m, s;
    if (total_sec < 0) total_sec = 0;
    h = total_sec / 3600;
    m = (total_sec % 3600) / 60;
    s = total_sec % 60;
    snprintf(out, out_size, "%02d:%02d:%02d", h, m, s);
}

static int time_already_passed(const char *time_str)
{
    int target_h, target_m, target_s;
    if (sscanf(time_str, "%d:%d:%d", &target_h, &target_m, &target_s) != 3)
        return 0;
    SYSTEMTIME st;
    get_timestamp(&st);
    int now_secs    = st.wHour * 3600 + st.wMinute * 60 + st.wSecond;
    int target_secs = target_h * 3600 + target_m  * 60 + target_s;
    return (now_secs > target_secs);
}

/* wait_until_utc - waits for a UTC time.
 * between_recordings=0: never start immediately, always wait for tomorrow
 *                        if the time has passed (used for first scheduled start).
 * between_recordings=1: start immediately if time passed within 5 minutes
 *                        (previous recording slightly overran). Otherwise
 *                        wait until tomorrow.                               */
static int wait_until_time(const char *time_str, int between_recordings)
{
    int target_h, target_m, target_s;
    if (sscanf(time_str, "%d:%d:%d", &target_h, &target_m, &target_s) != 3) {
        LOG_ERROR("Invalid start_time format '%s' - expected HH:MM:SS", time_str);
        return 0;
    }
    if (target_h < 0 || target_h > 23 || target_m < 0 || target_m > 59 ||
            target_s < 0 || target_s > 59) {
        LOG_ERROR("Invalid start_time '%s' - hours must be 0-23, minutes "
                  "and seconds 0-59.", time_str);
        return 0;
    }
    LOG_INFO("Scheduled start: waiting until %02d:%02d:%02d",
             target_h, target_m, target_s);

    while (g_running && !g_cancel_listening && !g_downgrade_to_listening) {
        SYSTEMTIME st;
        int now_secs, target_secs, diff;
        get_timestamp(&st);
        now_secs    = st.wHour * 3600 + st.wMinute * 60 + st.wSecond;
        target_secs = target_h * 3600 + target_m  * 60 + target_s;
        diff = target_secs - now_secs;

        /* If time has passed: between recordings allow a 5-minute grace
         * period for slight overruns; otherwise always wait until tomorrow. */
        if (diff < 0) {
            if (between_recordings && diff >= -300)
                diff = 0;       /* slight overrun - start now */
            else
                diff += 86400;  /* wait until tomorrow        */
        }

        if (diff <= 0) {
            LOG_INFO("Scheduled start time reached");
            return 1;
        }
        if (g_record_now) {
            /* Not the scheduled time - the user forced an early start.
             * "Record Now: ad-hoc recording..." (logged by the caller)
             * already explains what happened; no need to also claim the
             * schedule was reached when it wasn't.                       */
            return 1;
        }

        /* GUI build: countdown shown via g_state.next_start in status bar */
        snprintf(g_state.next_start, sizeof(g_state.next_start),
                 "%02d:%02d:%02d", target_h, target_m, target_s);
        if (!g_worker_active) {
            snprintf(g_ui.sched, sizeof(g_ui.sched),
                     "Waiting: scheduled start %02d:%02d:%02d",
                     target_h, target_m, target_s);
            if (g_hwnd) {
                RECT cr; GetClientRect(g_hwnd, &cr);
                RECT strip = { 0, cr.bottom - 36, cr.right, cr.bottom - 10 };
                InvalidateRect(g_hwnd, &strip, FALSE);
            }
        }
        Sleep(1000);
    }
    return 0;
}

/* =========================================================================
 * Main
 * ========================================================================= */
/* =========================================================================
 * Apply a schedule entry to the live hardware
 *
 * Updates frequency (and Tuner B frequency in dual mode) via
 * sdrplay_api_Update() without stopping the stream. Called between
 * recordings in a multi-recording schedule.
 * ========================================================================= */
static void apply_schedule_entry(AppState *state, const ScheduleEntry *e)
{
    sdrplay_api_ErrT err;

    if (e->frequency_hz > 0.0) {
        const char *tname = (state->device.tuner == sdrplay_api_Tuner_B) ? "B" : "A";
        state->cfg.frequency_hz = e->frequency_hz;
        if (state->stream_running) {
            state->ch_a_params->tunerParams.rfFreq.rfHz = e->frequency_hz;
            err = sdrplay_api_Update(state->device.dev, state->device.tuner,
                                     sdrplay_api_Update_Tuner_Frf,
                                     sdrplay_api_Update_Ext1_None);
            if (err == sdrplay_api_Success)
                LOG_INFO("Schedule: Tuner %s frequency set to %.6f MHz",
                         tname, e->frequency_hz / 1e6);
            else
                LOG_WARN("Schedule: Tuner %s frequency update failed: %s",
                         tname, sdrplay_api_GetErrorString(err));
        } else {
            LOG_INFO("Schedule: Tuner %s frequency will be %.6f MHz",
                     tname, e->frequency_hz / 1e6);
        }
    }

    if (state->cfg.dual_channel && e->freq_b_hz > 0.0
            && state->ch_b_params) {
        state->cfg.freq_b_hz = e->freq_b_hz;
        if (state->stream_running) {
            state->ch_b_params->tunerParams.rfFreq.rfHz = e->freq_b_hz;
            err = sdrplay_api_Update(state->device.dev, sdrplay_api_Tuner_B,
                                     sdrplay_api_Update_Tuner_Frf,
                                     sdrplay_api_Update_Ext1_None);
            if (err == sdrplay_api_Success)
                LOG_INFO("Schedule: Tuner B frequency set to %.6f MHz",
                         e->freq_b_hz / 1e6);
            else
                LOG_WARN("Schedule: Tuner B frequency update failed: %s",
                         sdrplay_api_GetErrorString(err));
        } else {
            LOG_INFO("Schedule: Tuner B frequency will be %.6f MHz",
                     e->freq_b_hz / 1e6);
        }
    }

    /* Update antenna if specified */
    if (e->antenna[0]) {
        strncpy(state->cfg.antenna, e->antenna, 7);
        LOG_INFO("Schedule: antenna set to %s", e->antenna);
        /* cfg->antenna is applied at the next sdrplay_api_Init call via
         * apply_antenna_and_biast(). Calling Update here would fail with
         * NotInitialised as the stream is stopped between schedule entries. */
    }

    /* Update duration for this recording */
    if (e->duration_sec > 0)
        state->cfg.duration_sec = e->duration_sec;

    /* Update output file name */
    if (e->output_file[0])
        strncpy(state->cfg.output_file, e->output_file, MAX_PATH_LEN - 1);
    else
        strncpy(state->cfg.output_file, DEFAULT_OUTPUT_FILE, MAX_PATH_LEN - 1);
}

static void verify_recording(const Config *cfg, LONG64 samples_written)
{
    if (cfg->output_format == FORMAT_WAVVIEWDX) {
        LOG_INFO("Post-recording verification skipped (WavViewDX format has "
                 "no header).");
        return;
    }

    /* SDRuno, Winrad, and SDR Connect: RIFF ChunkSize was patched before
     * CloseHandle. Read it back and compare to expected from sample count. */
    if (cfg->output_format == FORMAT_SDRUNO ||
            cfg->output_format == FORMAT_WINRAD ||
            cfg->output_format == FORMAT_SDRCONNECT) {
        HANDLE fh2 = CreateFileA(cfg->output_file, GENERIC_READ, FILE_SHARE_READ,
                                 NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
        if (fh2 == INVALID_HANDLE_VALUE) {
            LOG_WARN("Verification: could not open '%s' (error %lu).",
                     cfg->output_file, GetLastError());
            return;
        }
        LARGE_INTEGER fsz;
        GetFileSizeEx(fh2, &fsz);
        CloseHandle(fh2);

        /* Compare against the actual measured file size rather than the
         * 32-bit riff_size field in the header - that field wraps around
         * on files >4GB, which previously caused a false "SHORT" warning
         * even though the file itself was complete and correct.          */
        int64_t hdr_sz;
        if (cfg->output_format == FORMAT_SDRUNO)
            hdr_sz = 216; /* never RF64 - see the OutputFormat enum comment */
        else if (cfg->output_format == FORMAT_WINRAD)
            hdr_sz = (cfg->large_file_mode == LARGE_FILE_RF64)
                     ? (int64_t)sizeof(SDRunoRF64Header) : 216;
        else /* FORMAT_SDRCONNECT */
            hdr_sz = (cfg->large_file_mode == LARGE_FILE_RF64)
                     ? (int64_t)sizeof(SDRConnectRF64Header) : 80;
        int64_t expected_data  = samples_written * 4;
        int64_t expected_total = hdr_sz + expected_data;
        int64_t actual_total   = fsz.QuadPart;
        int64_t diff           = actual_total - expected_total;
        double  dur = (cfg->expected_output_rate_hz > 0)
                      ? (double)(expected_data / 4) / cfg->expected_output_rate_hz : 0.0;
        LOG_INFO("Verification: file=%lld bytes  data=%lld bytes  "
                 "duration=%02d:%02d:%02d",
                 (long long)fsz.QuadPart,
                 (long long)(fsz.QuadPart - hdr_sz),
                 (int)dur/3600, ((int)dur%3600)/60, (int)dur%60);
        if      (diff == 0) LOG_OK("Verification PASSED (%lld bytes).",
                                   (long long)expected_total);
        else if (diff < 0)  LOG_WARN("Verification WARNING - file is %lld bytes "
                                     "SHORT of expected %lld.",
                                     (long long)-diff, (long long)expected_total);
        else                LOG_WARN("Verification: file is %lld bytes larger than "
                                     "expected (possible filesystem padding).",
                                     (long long)diff);
        return;
    }

    HANDLE fh = CreateFileA(cfg->output_file, GENERIC_READ, FILE_SHARE_READ,
                            NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (fh == INVALID_HANDLE_VALUE) {
        LOG_WARN("Verification: could not open '%s' (error %lu).",
                 cfg->output_file, GetLastError());
        return;
    }

    /* Get actual file size */
    LARGE_INTEGER file_size;
    if (!GetFileSizeEx(fh, &file_size)) {
        LOG_WARN("Verification: could not get file size (error %lu).",
                 GetLastError());
        CloseHandle(fh);
        return;
    }

    /* Read the 41-byte Linrad header */
    LinradRawHeader hdr;
    DWORD nread;
    BOOL ok = ReadFile(fh, &hdr, sizeof(hdr), &nread, NULL);
    CloseHandle(fh);

    if (!ok || nread != sizeof(hdr)) {
        LOG_WARN("Verification: could not read header (%lu bytes read).",
                 nread);
        return;
    }

    if (hdr.remember_proprietary_chunk != -1) {
        LOG_WARN("Verification: header sentinel invalid - file may be corrupt.");
        return;
    }

    /* bytes_per_frame: rx_ad_channels * sizeof(int16_t)
     * Single = 2 channels (I,Q)   * 2 = 4 bytes
     * Dual   = 4 channels (IA,QA,IB,QB) * 2 = 8 bytes              */
    int     bytes_per_frame  = hdr.rx_ad_channels * 2;
    int64_t header_bytes     = (int64_t)sizeof(LinradRawHeader);
    int64_t data_bytes       = file_size.QuadPart - header_bytes;
    int64_t actual_samples   = (bytes_per_frame > 0)
                               ? data_bytes / bytes_per_frame : 0;
    double  actual_duration  = (hdr.rx_ad_speed > 0)
                               ? (double)actual_samples / hdr.rx_ad_speed : 0.0;

    /* Expected file size from the writer's sample count */
    int64_t expected_data    = samples_written * bytes_per_frame;
    int64_t expected_total   = header_bytes + expected_data;
    int64_t diff             = file_size.QuadPart - expected_total;

    /* Round (not truncate) the reported total: a 40 s recording captures very
     * slightly under 40 s of samples (the loop stops at the threshold), so
     * truncation would display 39. Rounding restores the requested figure. */
    long total_s = (long)(actual_duration + 0.5);
    int exp_h = (int)(total_s / 3600);
    int exp_m = (int)((total_s % 3600) / 60);
    int exp_s = (int)(total_s % 60);

    if (cfg->verbose)
        LOG_INFO("Verifying file: %s", cfg->output_file);

    if (diff == 0) {
        LOG_OK("Verification PASSED - Total duration: %02d:%02d:%02d.",
               exp_h, exp_m, exp_s);
    } else if (diff < 0) {
        LOG_WARN("Verification WARNING - file is %lld bytes SHORT of expected "
                 "(%lld). Recording may be truncated.",
                 (long long)-diff, (long long)expected_total);
    } else {
        LOG_WARN("Verification: file is %lld bytes larger than expected "
                 "(possible filesystem padding).",
                 (long long)diff);
    }
}

/* Closes and verifies output_file_b (Tuner B's separate file, dual-separate-
 * files mode only). verify_recording() is fully self-describing from the
 * file's own header, so it just needs cfg->output_file pointed at file B
 * momentarily - restored afterwards. Safe to call unconditionally; no-ops
 * when out_file_b isn't open. */
static void close_and_verify_file_b(AppState *state)
{
    if (state->out_file_b == INVALID_HANDLE_VALUE)
        return;

    CloseHandle(state->out_file_b);
    state->out_file_b = INVALID_HANDLE_VALUE;

    if (state->samples_written > 0) {
        char save_path[MAX_PATH_LEN];
        strncpy(save_path, state->cfg.output_file, MAX_PATH_LEN - 1);
        save_path[MAX_PATH_LEN - 1] = '\0';
        strncpy(state->cfg.output_file, state->output_file_b, MAX_PATH_LEN - 1);
        state->cfg.output_file[MAX_PATH_LEN - 1] = '\0';
        verify_recording(&state->cfg, state->samples_written);
        strncpy(state->cfg.output_file, save_path, MAX_PATH_LEN - 1);
        state->cfg.output_file[MAX_PATH_LEN - 1] = '\0';
    }
}


/* =========================================================================
 * HTTP status server
 * ========================================================================= */

typedef struct { SOCKET sock; AppState *state; } HttpConn;

static DWORD WINAPI http_worker(LPVOID param)
{
    HttpConn *conn  = (HttpConn *)param;
    SOCKET    sock  = conn->sock;
    AppState *state = conn->state;
    free(conn);

    { DWORD tv = 2000; setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv)); }

    char req[512];
    memset(req, 0, sizeof(req));
    if (recv(sock, req, sizeof(req) - 1, 0) <= 0) { closesocket(sock); return 0; }

    int want_json = strstr(req, "GET /status") != NULL;
    int not_found = !want_json && !strstr(req, "GET / ") && !strstr(req, "GET /\r");

    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    double elapsed = 0.0;
    if (state->session_complete)
        elapsed = state->frozen_elapsed_sec;
    else if (state->start_time.QuadPart > 0)
        elapsed = (double)(now.QuadPart - state->start_time.QuadPart)
                  / (double)state->perf_freq.QuadPart;

    LONG64 samples_rx = state->samples_received;
    LONG64 samples_wr = state->samples_written;
    LONG   ovf        = state->overflows;
    LONG64 zf         = state->zero_frames_written;
    float  pk_a       = state->peak_dbfs;
    float  pk_b       = state->peak_dbfs_b;
    int    ovl_a      = state->overload_tuner_a;
    int    ovl_b      = state->overload_tuner_b;
    int    werr       = state->writer_error;
    int    dwarn      = state->disk_warn_issued;
    int    dstop      = state->disk_stop;
    LONG64 disk_free  = state->disk_free_mb;
    int    dual       = state->cfg.dual_channel;

    LONG64 file_bytes = 0;
    if (state->stream_running) {
        /* Use the live samples_written counter (GetFileSizeEx lags badly
         * during recording due to OS write caching).                       */
        int frame_bytes = state->cfg.dual_channel ? 8 : 4;
        file_bytes = state->samples_written * frame_bytes;
    } else if (state->frozen_file_mb > 0) {
        file_bytes = state->frozen_file_mb * 1024LL * 1024LL;
    } else if (state->out_file != INVALID_HANDLE_VALUE) {
        LARGE_INTEGER fs;
        if (GetFileSizeEx(state->out_file, &fs)) file_bytes = fs.QuadPart;
    }
    double file_mb = (double)file_bytes / (1024.0 * 1024.0);

    int agc_on = state->ch_a_params != NULL &&
                 (state->ch_a_params->ctrlParams.agc.enable != sdrplay_api_AGC_DISABLE);

    if (not_found) {
        const char *h = "HTTP/1.0 404 Not Found\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\n404 Not Found\n";
        send(sock, h, (int)strlen(h), 0);
        shutdown(sock, SD_SEND); closesocket(sock); return 0;
    }

    if (want_json) {
        const char *body;
        char live_body[4096];

        if (state->session_complete) {
            /* Serve the pre-built frozen JSON - safe after SDRplay cleanup */
            body = state->frozen_json;
        } else {
            snprintf(live_body, sizeof(live_body),
                "{\"elapsed_sec\":%.1f,\"file_mb\":%.1f,\"disk_free_mb\":%lld,"
                "\"overflows\":%ld,\"zero_frames\":%lld,"
                "\"peak_dbfs_a\":%.1f,\"peak_dbfs_b\":%.1f,"
                "\"overload_a\":%d,\"overload_b\":%d,\"agc_on\":%d,\"dual_channel\":%d,"
                "\"disk_warn\":%d,\"disk_stop\":%d,\"writer_error\":%d,\"running\":1,"
                "\"recording\":%d,\"hdr_on\":%d,\"session_complete\":0,"
                "\"waiting\":%d,\"next_start\":\"%s\","
                "\"samples_rx\":%lld,\"samples_written\":%lld}",
                elapsed, file_mb, (long long)disk_free, ovf, (long long)zf,
                (double)pk_a, (double)pk_b, ovl_a, ovl_b, agc_on, dual,
                dwarn, dstop, werr,
                state->stream_running,
                state->cfg.hdr_enable,
                (state->next_start[0] && !state->stream_running) ? 1 : 0,
                state->next_start,
                (long long)samples_rx, (long long)samples_wr);
            body = live_body;
        }
        const char *hdr = "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\n"
                          "Access-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n";
        send(sock, hdr, (int)strlen(hdr), 0);
        send(sock, body, (int)strlen(body), 0);
    } else {
        const char *html =
"<!DOCTYPE html><html lang='en'><head>\n"
"<meta charset='UTF-8'>\n"
"<meta name='viewport' content='width=device-width,initial-scale=1'>\n"
"<title>DuoDX Monitor</title>\n"
"<style>\n"
"body{font-family:monospace;background:#111;color:#ccc;margin:0;padding:16px;}\n"
"h1{color:#4af;margin:0 0 16px;font-size:1.3em;letter-spacing:.05em;}\n"
".grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(200px,1fr));gap:10px;}\n"
".card{background:#1c1c1c;border:1px solid #333;border-radius:6px;padding:14px 16px;}\n"
".label{font-size:.85em;color:#aaa;text-transform:uppercase;letter-spacing:.06em;margin-bottom:6px;}\n"
".val{font-size:1.6em;color:#eee;font-weight:bold;}\n"
".ok{color:#4f4;}.warn{color:#fa0;}.err{color:#f44;}\n"
".bar-wrap{background:#222;border-radius:3px;height:8px;margin-top:6px;}\n"
".bar{height:8px;border-radius:3px;transition:width .4s;}\n"
"#status-dot{display:inline-block;width:9px;height:9px;border-radius:50%;"
"background:#4f4;margin-right:6px;vertical-align:middle;}\n"
"footer{margin-top:18px;font-size:.82em;color:#666;}\n"
"</style></head><body>\n"
"<h1><span id='status-dot'></span>DuoDX Monitor</h1>\n"
"<div class='grid'>\n"
"<div class='card'><div class='label'>Elapsed</div><div class='val' id='elapsed'>--</div></div>\n"
"<div class='card'><div class='label'>File size</div><div class='val' id='filesize'>--</div></div>\n"
"<div class='card'><div class='label'>Disk free</div><div class='val' id='diskfree'>--</div></div>\n"
"<div class='card'><div class='label'>Signal A (dBFS)</div><div class='val' id='sigA'>--</div>"
"<div class='bar-wrap'><div class='bar' id='barA' style='width:0%;background:#4af;'></div></div></div>\n"
"<div class='card' id='cardB' style='display:none'><div class='label'>Signal B (dBFS)</div>"
"<div class='val' id='sigB'>--</div>"
"<div class='bar-wrap'><div class='bar' id='barB' style='width:0%;background:#4af;'></div></div></div>\n"
"<div class='card'><div class='label'>Overflows</div><div class='val' id='ovf'>--</div></div>\n"
"<div class='card'><div class='label'>Zero-fill frames</div><div class='val' id='zf'>--</div></div>\n"
"<div class='card'><div class='label' id='agcLabel'>AGC</div><div class='val' id='agc'>--</div></div>\n"
"<div class='card'><div class='label'>Alerts</div><div class='val' id='alerts'>--</div></div>\n"
"</div>\n"
"<footer id='updated'>Waiting for first update...</footer>\n"
"<script>\n"
"function fmt_elapsed(s){var h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sec=Math.floor(s%60);\n"
"  return String(h).padStart(2,'0')+':'+String(m).padStart(2,'0')+':'+String(sec).padStart(2,'0');}\n"
"function dbfs_bar(v){return Math.max(0,Math.min(100,((v+90)/90)*100));}\n"
"function dbfs_col(v){if(v>-6)return'#f44';if(v>-20)return'#fa0';if(v>-60)return'#4af';return'#555';}\n"
"function poll(){\n"
"  fetch('/status').then(r=>r.json()).then(d=>{\n"
"    if(d.recording||d.session_complete){\n"
"      var elEl=document.getElementById('elapsed');\n"
"      if(d.session_complete)\n"
"        elEl.innerHTML=fmt_elapsed(d.elapsed_sec)+'<span style=\"color:#ffd700;margin-left:8px;\">FINISHED</span>';\n"
"      else if(d.next_start&&d.next_start.length>0){\n"
"        var t=d.next_start.substring(0,5);\n"
"        elEl.innerHTML=fmt_elapsed(d.elapsed_sec)+'<span style=\"color:#fa0;margin-left:8px;\">NEXT AT '+t+'Z</span>';}\n"
"      else\n"
"        elEl.textContent=fmt_elapsed(d.elapsed_sec);\n"
"      var mb=d.file_mb;\n"
"      document.getElementById('filesize').textContent=mb>=1024?(mb/1024).toFixed(2)+' GB':mb.toFixed(0)+' MB';\n"
"    } else if(d.waiting){\n"
"      var t=d.next_start.substring(0,5);\n"
"      document.getElementById('elapsed').innerHTML='--'+'<span style=\"color:#fa0;margin-left:8px;\">NEXT AT '+t+'Z</span>';\n"
"    }\n"
"    var df=d.disk_free_mb,dfel=document.getElementById('diskfree');\n"
"    dfel.textContent=df>=1024?(df/1024).toFixed(1)+' GB':df+' MB';\n"
"    dfel.className='val'+(d.disk_warn?' warn':'')+(d.disk_stop?' err':'');\n"
"    var pa=d.peak_dbfs_a,saEl=document.getElementById('sigA');\n"
"    if(pa>-90){saEl.textContent=pa.toFixed(1)+' dBFS';saEl.style.color=dbfs_col(pa);\n"
"      document.getElementById('barA').style.width=dbfs_bar(pa)+'%';\n"
"      document.getElementById('barA').style.background=dbfs_col(pa);}\n"
"    else{saEl.textContent='------';saEl.style.color='#555';}\n"
"    var cardB=document.getElementById('cardB');\n"
"    if(d.dual_channel){cardB.style.display='';\n"
"      var pb=d.peak_dbfs_b,sbEl=document.getElementById('sigB');\n"
"      if(pb>-90){sbEl.textContent=pb.toFixed(1)+' dBFS';sbEl.style.color=dbfs_col(pb);\n"
"        document.getElementById('barB').style.width=dbfs_bar(pb)+'%';\n"
"        document.getElementById('barB').style.background=dbfs_col(pb);}\n"
"      else{sbEl.textContent='------';sbEl.style.color='#555';}\n"
"    }else{cardB.style.display='none';}\n"
"    var oEl=document.getElementById('ovf');\n"
"    oEl.textContent=d.overflows;oEl.className='val '+(d.overflows>0?'warn':'ok');\n"
"    var zEl=document.getElementById('zf');\n"
"    zEl.textContent=d.zero_frames.toLocaleString();zEl.className='val '+(d.zero_frames>0?'warn':'ok');\n"
"    var agcEl=document.getElementById('agc');\n"
"    var agcLabel=document.getElementById('agcLabel');\n"
"    if(d.hdr_on){\n"
"      agcLabel.textContent='HDR';\n"
"      agcEl.textContent='HDR enabled';agcEl.className='val ok';\n"
"    }else{\n"
"      agcLabel.textContent='AGC';\n"
"      agcEl.textContent=d.agc_on?'ON':'OFF';agcEl.className='val '+(d.agc_on?'warn':'ok');\n"
"    }\n"
"    var alerts=[];\n"
"    if(d.overload_a)alerts.push('OVL-A');\n"
"    if(d.overload_b)alerts.push('OVL-B');\n"
"    if(d.writer_error)alerts.push('WRITE ERR');\n"
"    if(d.disk_stop)alerts.push('DISK FULL');\n"
"    var aEl=document.getElementById('alerts');\n"
"    aEl.textContent=alerts.length?alerts.join(' | '):'None';\n"
"    aEl.className='val '+(alerts.length?'err':'ok');\n"
"    document.getElementById('status-dot').style.background='#4f4';\n"
"    document.getElementById('updated').textContent='Last update: '+new Date().toLocaleTimeString();\n"
"  }).catch(()=>{document.getElementById('status-dot').style.background='#f44';});}\n"
"poll();setInterval(poll,";
        const char *html2 = ");\n</script></body></html>\n";
        char interval_str[16];
        snprintf(interval_str, sizeof(interval_str), "%d", state->cfg.http_interval_ms);
        const char *hdr = "HTTP/1.0 200 OK\r\nContent-Type: text/html; charset=UTF-8\r\nConnection: close\r\n\r\n";
        send(sock, hdr, (int)strlen(hdr), 0);
        send(sock, html, (int)strlen(html), 0);
        send(sock, interval_str, (int)strlen(interval_str), 0);
        send(sock, html2, (int)strlen(html2), 0);
    }
    shutdown(sock, SD_SEND);
    closesocket(sock);
    return 0;
}

static DWORD WINAPI http_status_thread_func(LPVOID param)
{
    AppState *state = (AppState *)param;
    SOCKET listen_sock;
    struct sockaddr_in addr;
    WSADATA wsa;
    int port = state->cfg.http_port;

    if (WSAStartup(MAKEWORD(2,2), &wsa) != 0) {
        LOG_WARN("[HTTP] WSAStartup failed (%d)", WSAGetLastError()); return 1;
    }
    listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_sock == INVALID_SOCKET) {
        LOG_WARN("[HTTP] socket() failed (%d)", WSAGetLastError()); WSACleanup(); return 1;
    }
    { int yes=1; setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&yes, sizeof(yes)); }
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons((u_short)port);
    if (bind(listen_sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        LOG_WARN("[HTTP] bind() on port %d failed (%d)", port, WSAGetLastError());
        fflush(stdout); closesocket(listen_sock); WSACleanup(); return 1;
    }
    if (listen(listen_sock, 8) != 0) {
        LOG_WARN("[HTTP] listen() failed (%d)", WSAGetLastError());
        fflush(stdout); closesocket(listen_sock); WSACleanup(); return 1;
    }
    { u_long nb=1; ioctlsocket(listen_sock, FIONBIO, &nb); }

    Sleep(100);  /* brief delay so message doesn't interrupt countdown line */
    LOG_INFO("[HTTP] Status server listening on port %d", port);
    g_http_ready = 1;

    while (g_http_running) {
        struct sockaddr_in cli_addr;
        int cli_len = sizeof(cli_addr);
        SOCKET client_sock = accept(listen_sock, (struct sockaddr *)&cli_addr, &cli_len);
        if (client_sock == INVALID_SOCKET) {
            int err = WSAGetLastError();
            if (err != WSAEWOULDBLOCK) {
                LOG_WARN("[HTTP] accept() error %d - listen socket may have closed", err);
                fflush(stdout);
                break;
            }
            Sleep(20); continue;
        }
        HttpConn *conn = (HttpConn *)malloc(sizeof(HttpConn));
        if (conn) {
            conn->sock = client_sock; conn->state = state;
            HANDLE wt = CreateThread(NULL, 0, http_worker, conn, 0, NULL);
            if (wt) CloseHandle(wt);
            else { free(conn); closesocket(client_sock); }
        } else closesocket(client_sock);
    }
    closesocket(listen_sock);
    WSACleanup();
    return 0;
}

/* =========================================================================
 * Hourly recording mode helpers
 * ========================================================================= */

/* Parse HH:MM string into total minutes since midnight. Returns -1 on error */
static int hhmm_to_min(const char *s)
{
    int h, m;
    if (sscanf(s, "%d:%d", &h, &m) != 2) return -1;
    if (h < 0 || h > 23 || m < 0 || m > 59) return -1;
    return h * 60 + m;
}

/* Current time as minutes since midnight */
/* Get current time, using UTC or local depending on config */
static void get_timestamp(SYSTEMTIME *st)
{
    if (g_state.cfg.use_utc)
        GetSystemTime(st);
    else
        GetLocalTime(st);
}

static int now_min(void)
{
    SYSTEMTIME st;
    get_timestamp(&st);
    return st.wHour * 60 + st.wMinute;
}

/* Local constant rather than relying on math.h's M_PI, which isn't
 * guaranteed to be defined on MinGW without _USE_MATH_DEFINES set before
 * the #include - simpler to just not depend on that.                    */
#define SUN_PI 3.14159265358979323846

/* =========================================================================
 * Sunrise/sunset calculation - NOAA's General Solar Position algorithm
 * (the same formulas behind NOAA's published Solar Calculator spreadsheet).
 * Accurate to roughly a minute, which is plenty for an informational
 * display tile - this is not intended for precision astronomical use.
 *
 * Returns 1 with *sunrise_min_utc/*sunset_min_utc set to minutes-since-
 * midnight-UTC for the given calendar date (can be negative or >1440;
 * the caller wraps to the correct clock time), or 0 if the sun doesn't
 * rise/set at all that day at this latitude (polar day or polar night),
 * in which case both outputs are set to -1.
 * ========================================================================= */
static int calc_sun_times(int year, int month, int day,
                           double lat_deg, double lon_deg,
                           double *sunrise_min_utc, double *sunset_min_utc)
{
    static const int cum_days[12] = {0,31,59,90,120,151,181,212,243,273,304,334};
    int leap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
    int doy  = cum_days[month - 1] + day + ((leap && month > 2) ? 1 : 0);

    double gamma = 2.0 * SUN_PI / 365.0 * (double)(doy - 1);
    double eqtime = 229.18 * (0.000075 + 0.001868 * cos(gamma) - 0.032077 * sin(gamma)
                               - 0.014615 * cos(2 * gamma) - 0.040849 * sin(2 * gamma));
    double decl = 0.006918 - 0.399912 * cos(gamma) + 0.070257 * sin(gamma)
                  - 0.006758 * cos(2 * gamma) + 0.000907 * sin(2 * gamma)
                  - 0.002697 * cos(3 * gamma) + 0.00148  * sin(3 * gamma);

    double lat_rad    = lat_deg * SUN_PI / 180.0;
    double zenith_rad = 90.833 * SUN_PI / 180.0; /* atmospheric refraction + solar disk radius */
    double cos_ha = (cos(zenith_rad) / (cos(lat_rad) * cos(decl))) - (tan(lat_rad) * tan(decl));

    if (cos_ha > 1.0 || cos_ha < -1.0) {
        *sunrise_min_utc = -1.0;
        *sunset_min_utc  = -1.0;
        return 0; /* sun never rises (cos_ha>1) or never sets (cos_ha<-1) today here */
    }

    {
        double ha_deg = acos(cos_ha) * 180.0 / SUN_PI;
        *sunrise_min_utc = 720.0 - 4.0 * (lon_deg + ha_deg) - eqtime;
        *sunset_min_utc  = 720.0 - 4.0 * (lon_deg - ha_deg) - eqtime;
    }
    return 1;
}

/* Formats today's sunrise/sunset as "HH:MM" strings for the SUN tile,
 * displayed in local or UTC time matching cfg->use_utc - the same
 * convention the rest of the app's clock/logging already uses. Falls
 * back to "--:--" for polar day/night at extreme latitudes.             */
static void get_sun_times_str(const Config *cfg,
                               char *sunrise_str, size_t sunrise_len,
                               char *sunset_str,  size_t sunset_len)
{
    SYSTEMTIME st;
    double sunrise_min, sunset_min;

    get_timestamp(&st); /* today's date, in whichever zone cfg->use_utc selects */

    /* calc_sun_times() works from a UTC calendar date; using "today" in
     * the display's own zone rather than converting to a true UTC date is
     * a deliberate simplification - right at a local midnight this could
     * shift the result by under a minute, which doesn't matter for a
     * display-only feature and avoids a full calendar-date conversion.   */
    if (!calc_sun_times(st.wYear, st.wMonth, st.wDay, cfg->latitude, cfg->longitude,
                         &sunrise_min, &sunset_min)) {
        snprintf(sunrise_str, sunrise_len, "--:--");
        snprintf(sunset_str,  sunset_len,  "--:--");
        return;
    }

    if (!cfg->use_utc) {
        TIME_ZONE_INFORMATION tzi;
        DWORD r = GetTimeZoneInformation(&tzi);
        /* Win32 convention: UTC = local + Bias, so local = UTC - Bias.   */
        double bias_min = (double)tzi.Bias +
                           (double)((r == TIME_ZONE_ID_DAYLIGHT) ? tzi.DaylightBias : tzi.StandardBias);
        sunrise_min -= bias_min;
        sunset_min  -= bias_min;
    }

    {
        int sr = ((int)floor(sunrise_min) % 1440 + 1440) % 1440;
        int ss = ((int)floor(sunset_min)  % 1440 + 1440) % 1440;
        snprintf(sunrise_str, sunrise_len, "%02d:%02d", sr / 60, sr % 60);
        snprintf(sunset_str,  sunset_len,  "%02d:%02d", ss / 60, ss % 60);
    }
}

/* Current time as seconds since midnight */
static int now_sec(void)
{
    SYSTEMTIME st;
    get_timestamp(&st);
    return st.wHour * 3600 + st.wMinute * 60 + st.wSecond;
}

/* Return 1 if current local time is within the hourly session window
 * [start_min, stop_min). Handles overnight windows (stop < start).  */
static int hourly_in_session(int start_min, int stop_min)
{
    int now = now_min();
    if (stop_min > start_min)
        return (now >= start_min && now < stop_min);
    else  /* overnight: e.g. 17:00 to 05:00 */
        return (now >= start_min || now < stop_min);
}

/* Wait until the next hourly recording start (HH:00 - half_win_sec).
 * Returns 1 when the time has arrived, 0 if g_running went to 0.
 *
 * Rules:
 *  - If we are currently inside a pre-record window AND it has just
 *    started (i.e. we haven't missed more than 30 seconds of it),
 *    start immediately.
 *  - Otherwise wait until the next HH:00 - half_win_sec.
 */
static int hourly_wait_for_next(int half_win_sec)
{
    while (g_running && !g_downgrade_to_listening) {
        int cur_sec  = now_sec();
        int now_min_val  = cur_sec / 60;
        int sec_past = cur_sec % 3600;   /* seconds past the last hour mark */
        int pre_sec  = 3600 - half_win_sec; /* seconds past hour when pre-rec starts */

        /* Are we inside the pre-record window right now?
         * Window runs from (HH:00 - half_win_sec) to (HH:00 + half_win_sec)
         * i.e. sec_past >= pre_sec  OR  sec_past < half_win_sec            */
        if (sec_past >= pre_sec) {
            /* In the pre-window (e.g. 12:55-13:00) - start immediately */
            LOG_INFO("Hourly: starting recording window at %02d:%02d",
                     now_min_val / 60, now_min_val % 60);
            return 1;
        }
        if (sec_past < half_win_sec) {
            /* In the post-window (e.g. 13:00-13:05) - start immediately */
            LOG_INFO("Hourly: starting recording window at %02d:%02d",
                     now_min_val / 60, now_min_val % 60);
            return 1;
        }
        /* Between windows - wait for next pre-record start */

        /* Calculate seconds until next pre-record start */
        int secs_to_next;
        if (sec_past < pre_sec)
            secs_to_next = pre_sec - sec_past;
        else
            secs_to_next = 3600 - sec_past + pre_sec;

        /* Calculate the pre-record window start time (HH:MM:SS) - this is
         * when recording actually begins, the same thing the very first
         * hourly wait already shows (see the "Hourly: waiting until"
         * message above). Previously this got computed correctly here
         * and then discarded, replaced by a "hour_boundary" value that
         * added the half-window back - undoing the whole point - and
         * displayed with seconds hardcoded to :00 regardless of the real
         * value. That meant the first hourly window showed something
         * like 09:58:30 while every subsequent one showed a bare rounded
         * hour like 11:00 - two different things being called the same
         * "next" time.                                                   */
        int top_sec = (cur_sec + secs_to_next) % 86400;
        int disp_h  = top_sec / 3600;
        int disp_m  = (top_sec % 3600) / 60;
        int disp_s  = top_sec % 60;

        snprintf(g_state.next_start, sizeof(g_state.next_start),
                 "%02d:%02d:%02d", disp_h, disp_m, disp_s);

        /* Update the GUI scheduling text directly so it shows while the
         * monitor thread is not running (between hourly recordings).     */
        if (!g_worker_active) {
            snprintf(g_ui.sched, sizeof(g_ui.sched),
                     "Hourly %d min  next %s",
                     g_state.cfg.hourly_window_min,
                     g_state.next_start);
            if (g_hwnd) {
                RECT cr; GetClientRect(g_hwnd, &cr);
                RECT strip = { 0, cr.bottom - 36, cr.right, cr.bottom - 10 };
                InvalidateRect(g_hwnd, &strip, FALSE);
            }
        }

        Sleep(1000);
    }
    return 0;
}

/* One-shot version of hourly_wait_for_next()'s time math, for the idle
 * preview near the main window's Timer button (Section 3) - computes
 * where the next hourly window opens without looping or sleeping, since
 * nothing is actually waiting yet when this runs (no session started).
 * Writes "HH:MM" into out, or "now" if a window is already open.        */
static void hourly_next_window_text(char *out, size_t out_sz, int window_min)
{
    int half_win_sec = (window_min * 60) / 2;
    int cur_sec  = now_sec();
    int sec_past = cur_sec % 3600;
    int pre_sec  = 3600 - half_win_sec;
    int secs_to_next, top_sec, disp_h, disp_m, hour_boundary;

    if (sec_past >= pre_sec || sec_past < half_win_sec) {
        snprintf(out, out_sz, "now");
        return;
    }
    secs_to_next = (sec_past < pre_sec) ? (pre_sec - sec_past)
                                         : (3600 - sec_past + pre_sec);
    top_sec = (cur_sec + secs_to_next) % 86400;
    disp_h  = top_sec / 3600;
    disp_m  = (top_sec % 3600) / 60;
    hour_boundary = ((disp_h * 60 + disp_m + (half_win_sec / 60)) % (24 * 60));
    snprintf(out, out_sz, "%02d:%02d", hour_boundary / 60, hour_boundary % 60);
}

/* Shared Timer status text, shown near the Timer button (Section 3) -
 * used both for the idle preview (before Record is ever pressed, current
 * config only) and could equally be reused for the live/active-wait case
 * in future, though that path currently keeps its own longer-standing
 * wording (see wait_until_time()/hourly_wait_for_next()/the periodic
 * monitor thread) rather than being unified here, to avoid touching
 * those working call sites' countdown behaviour along with this.        */
static void gui_format_timer_idle_text(char *out, size_t out_sz)
{
    Config *cfg = &g_state.cfg;
    if (cfg->hourly_enable) {
        char next[16];
        hourly_next_window_text(next, sizeof(next), cfg->hourly_window_min);
        snprintf(out, out_sz, "HOURLY (next): %s", next);
    } else if (cfg->schedule_only) {
        if (cfg->schedule_count > 0) {
            const ScheduleEntry *e = &cfg->schedule[0];
            char start[6];
            snprintf(start, sizeof(start), "%.5s", e->start_time);
            if (e->duration_sec > 0) {
                int start_min = hhmm_to_min(e->start_time);
                int end_min = start_min >= 0
                    ? (start_min + e->duration_sec / 60) % (24 * 60) : -1;
                if (end_min >= 0) {
                    snprintf(out, out_sz, "SCHEDULED: %s-%02d:%02d  (%d entr%s)",
                             start, end_min / 60, end_min % 60,
                             cfg->schedule_count,
                             cfg->schedule_count == 1 ? "y" : "ies");
                    return;
                }
            }
            snprintf(out, out_sz, "SCHEDULED: %s  (%d entr%s)", start,
                     cfg->schedule_count, cfg->schedule_count == 1 ? "y" : "ies");
        } else {
            snprintf(out, out_sz, "SCHEDULED: no entries configured");
        }
    } else {
        out[0] = '\0';
    }
}

/* Refreshes the idle Timer preview and repaints it - called from the
 * clock timer while idle (WM_TIMER, so "next: HH:MM" doesn't go stale
 * sitting idle across an hour boundary), and immediately after anything
 * that could change what it should say (the Timer button, Settings
 * Save). Only meaningful while idle: once a session actually starts,
 * gui_monitor_thread_func()'s own g_ui.sched calculation - which has the
 * authoritative, already-armed next_start rather than a fresh guess -
 * takes over instead.                                                   */
static void gui_refresh_idle_timer_text(void)
{
    if (g_worker_active) return;
    gui_format_timer_idle_text(g_ui.sched, sizeof(g_ui.sched));
    if (g_hwnd) {
        RECT cr; GetClientRect(g_hwnd, &cr);
        RECT strip = { 0, cr.bottom - 36, cr.right, cr.bottom - 10 };
        InvalidateRect(g_hwnd, &strip, FALSE);
    }
}

static DWORD WINAPI recording_worker(LPVOID param)
{
    (void)param;
    sdrplay_api_ErrT          err;
    sdrplay_api_DeviceT       devices[6];
    unsigned int              num_devices = 0;
    sdrplay_api_CallbackFnsT  callbacks;
    char                     *config_file = g_config_file;
    HANDLE                    http_thread     = NULL;
    int                       sched_idx       = 0;   /* current schedule entry */
    int                       orig_sched_count = 0; /* total entries for display */
    int                       rc = 0;
    SIZE_T                    ring_size;
    int                       num_channels;
    char                      device_name[64] = "Unknown";
    unsigned int              i;

    /* Fresh run: clear the stop flags so repeated Start presses work. */
    g_running      = 1;
    g_http_running = 1;
    g_http_ready   = 0;

    /* Initialise global state */
    memset(&g_state, 0, sizeof(g_state));
    g_state.out_file         = INVALID_HANDLE_VALUE;
    g_state.out_file_b       = INVALID_HANDLE_VALUE;
    g_state.pipe_handle      = INVALID_HANDLE_VALUE;
    g_state.slave_process    = NULL;
    g_state.slave_job        = NULL;
    g_state.slave_stop_event = NULL;
    g_state.slave_monitor_thread  = NULL;
    g_state.slave_monitor_running = 0;
    g_state.frozen_file_mb   = -1;
    g_state.last_display_file_mb = -1;
    g_state.last_display_elapsed = 0.0;
    InitializeCriticalSection(&g_state.dual_lock);

    QueryPerformanceFrequency(&g_state.perf_freq);

    /* Load defaults then INI config (no CLI args in the GUI build). */
    config_set_defaults(&g_state.cfg);
    config_load_ini(&g_state.cfg, config_file);

    LOG_INFO("DuoDX GUI v%s  (c) 2026 Dave Headland", VERSION);
    if (g_state.cfg.verbose)
        LOG_INFO("Config file: %s", config_file);

    /* Keep the GUI clock in step with this session's config. */
    g_clock_show  = g_state.cfg.show_clock;
    g_clock_utc   = g_state.cfg.use_utc;
    g_meter_style = g_state.cfg.meter_style;

    /* Report free space on the target drive up front, so the display shows a
     * real value immediately instead of 0 MB until the first writer check.  */
    {
        ULARGE_INTEGER free_bytes;
        char dir_path[MAX_PATH] = ".";
        const char *src = g_state.cfg.recording_path[0]
                          ? g_state.cfg.recording_path
                          : g_state.cfg.output_file;
        const char *last_sep = NULL, *p = src;
        for (; *p; p++) if (*p == '\\' || *p == '/') last_sep = p;
        if (g_state.cfg.recording_path[0]) {
            strncpy(dir_path, g_state.cfg.recording_path, sizeof(dir_path) - 1);
        } else if (last_sep) {
            size_t len = (size_t)(last_sep - src) + 1;
            if (len < sizeof(dir_path)) { memcpy(dir_path, src, len); dir_path[len] = '\0'; }
        }
        if (GetDiskFreeSpaceExA(dir_path, &free_bytes, NULL, NULL)) {
            LONG64 free_mb = (LONG64)(free_bytes.QuadPart / (1024ULL * 1024ULL));
            InterlockedExchange64(&g_state.disk_free_mb, free_mb);
            if (free_mb >= 1024) {
                if (g_state.cfg.verbose)
                    LOG_INFO("Free space on target drive: %.1f GB", free_mb / 1024.0);
            } else
                if (g_state.cfg.verbose)
                    LOG_INFO("Free space on target drive: %lld MB", (long long)free_mb);
        }
    }

    /* Open log file if specified, or auto-save a timestamped one instead
     * if that's enabled and no manual path was set - independent
     * options, but a manually-set log_file always takes precedence if
     * both happen to be configured at once.                              */
    if (g_state.cfg.log_file[0]) {
        g_state.log_fp = fopen(g_state.cfg.log_file, "a");
        if (!g_state.log_fp)
            LOG_WARN("Could not open log file '%s'", g_state.cfg.log_file);
    } else if (g_state.cfg.log_auto_save) {
        char log_path[MAX_PATH_LEN];
        char dir[MAX_PATH_LEN] = "";
        SYSTEMTIME st;
        if (g_state.cfg.use_utc) GetSystemTime(&st); else GetLocalTime(&st);
        if (g_state.cfg.recording_path[0]) {
            size_t plen = strlen(g_state.cfg.recording_path);
            int has_sep = plen > 0 &&
                (g_state.cfg.recording_path[plen - 1] == '\\' ||
                 g_state.cfg.recording_path[plen - 1] == '/');
            snprintf(dir, sizeof(dir), "%s%s", g_state.cfg.recording_path,
                     has_sep ? "" : "\\");
        }
        snprintf(log_path, sizeof(log_path),
                 "%s%04d%02d%02d_%02d%02d%02d%s_session.log", dir,
                 st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond,
                 g_state.cfg.use_utc ? "Z" : "");
        g_state.log_fp = fopen(log_path, "w");
        if (g_state.log_fp)
            LOG_INFO("Session log: %s", log_path);
        else
            LOG_WARN("Could not open auto-save session log '%s'", log_path);
    }

    /* Print effective configuration */
    if (g_state.cfg.verbose)
        LOG_INFO("Configuration:");
    if (g_state.cfg.recording_path[0])
        LOG_INFO("  Recording path : %s", g_state.cfg.recording_path);
    if (g_state.cfg.verbose)
        LOG_INFO("  Output file    : %s", g_state.cfg.output_file);
    if (g_state.cfg.verbose)
        LOG_INFO("  Sample rate    : %.3f Msps (ADC rate; actual output may differ with IF mode)",
             g_state.cfg.sample_rate_hz / 1e6);
    if (g_state.cfg.verbose)
        LOG_INFO("  IF frequency   : %d kHz", g_state.cfg.if_khz);
    if (g_state.cfg.verbose)
        LOG_INFO("  IF bandwidth   : %d kHz", g_state.cfg.bw_khz);
    if (g_state.cfg.dual_channel) {
        int gr_b  = (g_state.cfg.gain_reduction_b >= 0) ? g_state.cfg.gain_reduction_b : g_state.cfg.gain_reduction;
        int lna_b = (g_state.cfg.lna_state_b      >= 0) ? g_state.cfg.lna_state_b      : g_state.cfg.lna_state;
        int indep = (g_state.cfg.gain_reduction_b >= 0 || g_state.cfg.lna_state_b >= 0);
        /* Use LOG_OK (green) only when the hardware is confirmed RSPduo */
        if (g_state.device.hwVer == SDRPLAY_RSPduo_ID)
            LOG_OK(  "  Dual channel   : YES (RSPduo)");
        else
            if (g_state.cfg.verbose)
                LOG_INFO("  Dual channel   : YES");
        if (g_state.cfg.verbose)
            LOG_INFO("  Tuner A        : %.6f MHz  Gain: %d dB  LNA: %d",
                 g_state.cfg.frequency_hz / 1e6,
                 g_state.cfg.gain_reduction,
                 g_state.cfg.lna_state);
        if (g_state.cfg.verbose)
            LOG_INFO("  Tuner B        : %.6f MHz  Gain: %d dB  LNA: %d  (%s)",
                 g_state.cfg.freq_b_hz / 1e6, gr_b, lna_b,
                 indep ? "independent" : "same as A");
    } else {
        if (g_state.cfg.verbose)
            LOG_INFO("  Frequency      : %.6f MHz", g_state.cfg.frequency_hz / 1e6);
        if (g_state.cfg.verbose)
            LOG_INFO("  Gain reduction : %d dB  LNA: %d",
                 g_state.cfg.gain_reduction, g_state.cfg.lna_state);
    }
    {
        int ds = g_state.cfg.duration_sec;
        if (ds > 0) {
            if (g_state.cfg.verbose)
                LOG_INFO("  Duration       : limited (%02d:%02d:%02d)",
                     ds / 3600, (ds % 3600) / 60, ds % 60);
        } else {
            if (g_state.cfg.verbose)
                LOG_INFO("  Duration       : unlimited (Stop to end)");
        }
    }
    if (g_state.cfg.start_time[0])
        if (g_state.cfg.verbose)
            LOG_INFO("  Scheduled start: %s", g_state.cfg.start_time);
    if (g_state.cfg.dual_channel) {
        /* Tuner 1 can be Hi-Z or 50 ohm; Tuner 2 is always 50 ohm, no selection */
        const char *t1_port =
            (!strcmp(g_state.cfg.antenna, "Hi-Z") ||
             !strcmp(g_state.cfg.antenna, "hi-z") ||
             !strcmp(g_state.cfg.antenna, "HIZ"))
            ? "Hi-Z" : "50 ohm";
        if (g_state.cfg.verbose)
            LOG_INFO("  Antenna        : T1=%s  T2=50 ohm (fixed)", t1_port);
    } else {
        if (g_state.cfg.verbose)
            LOG_INFO("  Antenna        : %s", g_state.cfg.antenna);
    }
    if (g_state.cfg.bias_t)
        LOG_OK(  "  Bias-T         : ENABLED");
    else
        if (g_state.cfg.verbose)
            LOG_INFO("  Bias-T         : Off");
    if (g_state.cfg.agc_enable) {
        LOG_INFO("  AGC            : ENABLED");
    } else
        if (g_state.cfg.verbose)
            LOG_INFO("  AGC            : Off");
    /* HDR validation runs later in apply_hdr_mode(). Display as white here
     * regardless of hdr_enable; apply_hdr_mode() will log a green confirmation
     * if the configuration is valid, or an error if not.                   */
    if (g_state.cfg.hdr_enable) {
        if (g_state.cfg.verbose)
            LOG_INFO("  HDR mode       : ENABLED (BW=%d kHz)", g_state.cfg.hdr_bw_khz);
    } else {
        if (g_state.cfg.verbose)
            LOG_INFO("  HDR mode       : Off");
    }
    if (g_state.cfg.hiz_notch)
        LOG_INFO("  Hi-Z AM notch  : ENABLED");
    if (g_state.cfg.ppm != 0.0)
        LOG_INFO("  PPM correction : %.2f ppm", g_state.cfg.ppm);
    if (g_state.cfg.device_serial[0])
        LOG_INFO("  Device serial  : %s", g_state.cfg.device_serial);
    if (g_state.cfg.decimation > 1)
        LOG_INFO("  SW decimation  : /%d", g_state.cfg.decimation);
    if (g_state.cfg.verbose)
        LOG_INFO("  Ring buffer    : %d seconds", g_state.cfg.ring_buffer_sec);
    if (g_state.cfg.verbose)
        LOG_INFO("  Time mode      : %s (scheduler, log and filenames)",
             g_state.cfg.use_utc ? "UTC" : "Local time");

    /* Validate parameter combination before touching hardware */
    if (!validate_config(&g_state.cfg)) {
        rc = 1;
        goto cleanup_no_api;
    }

    /* If schedule_only is not set, ignore any schedule entries entirely —
     * they are only processed when schedule_only = 1.                    */
    if (!g_state.cfg.schedule_only) {
        LOG_INFO("schedule_only=0: ignoring %d schedule entries.",
                 g_state.cfg.schedule_count);
        g_state.cfg.schedule_count = 0;
    }

    /* If schedule_only is set, copy schedule_1 settings into the top-level
     * config and run normally from sched_idx=0. This avoids calling
     * apply_schedule_entry before the device is initialised, and ensures
     * the start time is handled by the normal top-level wait path.      */
    if (g_state.cfg.schedule_only && g_state.cfg.schedule_count > 0) {
        /* Warn if schedule entries are not in chronological order — only
         * useful when schedule_1 hasn't passed (if it has, we skip it
         * anyway and the "order" is irrelevant).                        */
        if (!time_already_passed(g_state.cfg.schedule[0].start_time)) {
            for (int si = 0; si < g_state.cfg.schedule_count - 1; si++) {
                const char *t1 = g_state.cfg.schedule[si].start_time;
                const char *t2 = g_state.cfg.schedule[si+1].start_time;
                if (t1[0] && t2[0] && strcmp(t1, t2) > 0) {
                    LOG_WARN("Schedule entries are not in chronological order: "
                             "schedule_%d (%s) is after schedule_%d (%s). "
                             "Re-number them chronologically.",
                             si+1, t1, si+2, t2);
                }
            }
        }
        ScheduleEntry *e = &g_state.cfg.schedule[0];
        LOG_INFO("schedule_only=1: using schedule_1 as first recording.");
        orig_sched_count = g_state.cfg.schedule_count; /* save total before shift */
        /* Save original top-level values (pre-promotion) for ad-hoc recordings. */
        g_state.adhoc_frequency_hz  = g_state.cfg.frequency_hz;
        g_state.adhoc_freq_b_hz     = g_state.cfg.freq_b_hz;
        g_state.adhoc_duration_sec  = g_state.cfg.duration_sec;
        if (e->frequency_hz > 0.0)
            g_state.cfg.frequency_hz = e->frequency_hz;
        if (e->freq_b_hz > 0.0)
            g_state.cfg.freq_b_hz = e->freq_b_hz;
        if (e->duration_sec > 0)
            g_state.cfg.duration_sec = e->duration_sec;
        /* Save schedule_1 values post-promotion for restoration after ad-hoc. */
        g_state.sched1_frequency_hz = g_state.cfg.frequency_hz;
        g_state.sched1_freq_b_hz    = g_state.cfg.freq_b_hz;
        g_state.sched1_duration_sec = g_state.cfg.duration_sec;
        if (e->output_file[0])
            strncpy(g_state.cfg.output_file, e->output_file, MAX_PATH_LEN-1);
        if (e->antenna[0])
            strncpy(g_state.cfg.antenna, e->antenna, sizeof(g_state.cfg.antenna)-1);
        if (e->start_time[0]) {
            if (!time_already_passed(e->start_time)) {
                strncpy(g_state.cfg.start_time, e->start_time,
                        sizeof(g_state.cfg.start_time)-1);
            }
            /* else: handled after the shift below */
        } else {
            LOG_WARN("schedule_only: schedule_1 has no start_time set - "
                     "recording will start immediately.");
        }
        /* Save schedule_1 start_time before the shift overwrites schedule[0]. */
        char orig_start[16] = {0};
        strncpy(orig_start, e->start_time, sizeof(orig_start)-1);

        /* Remove schedule_1 from the list - remaining entries shift down */
        int i;
        for (i = 0; i < g_state.cfg.schedule_count - 1; i++)
            g_state.cfg.schedule[i] = g_state.cfg.schedule[i+1];
        g_state.cfg.schedule_count--;

        /* Now handle the case where schedule_1 time has passed. */
        if (orig_start[0] && time_already_passed(orig_start)) {
            int found = 0;
            for (int si = 0; si < g_state.cfg.schedule_count; si++) {
                ScheduleEntry *se = &g_state.cfg.schedule[si];
                if (se->start_time[0] && !time_already_passed(se->start_time)) {
                    LOG_WARN("schedule_only: schedule_1 time %s has passed. "
                             "Skipping to next entry at %s.",
                             orig_start, se->start_time);
                    strncpy(g_state.cfg.start_time, se->start_time,
                            sizeof(g_state.cfg.start_time)-1);
                    sched_idx = si + 1;
                    found = 1;
                    break;
                }
            }
            if (!found) {
                LOG_INFO("schedule_only: all schedule times have passed today "
                         "- waiting until schedule_1 (%s) tomorrow.", orig_start);
                strncpy(g_state.cfg.start_time, orig_start,
                        sizeof(g_state.cfg.start_time)-1);
            }
        }

        if (g_state.cfg.start_time[0]) {
            strncpy(g_state.next_start, g_state.cfg.start_time,
                    sizeof(g_state.next_start)-1);
            LOG_INFO("schedule_only: will wait for start time %s",
                     g_state.cfg.start_time);
        }
    }

    /* GUI build: Stop button / window close set g_running=0 (no signals). */

    /* ------------------------------------------------------------------ */
    /* Step 1: Open the SDRplay API                                        */
    /* ------------------------------------------------------------------ */
    err = sdrplay_api_Open();
    if (err != sdrplay_api_Success) {
        LOG_ERROR("sdrplay_api_Open failed: %s", sdrplay_api_GetErrorString(err));
        LOG_ERROR("Is the SDRplay API service running?");
        rc = 1;
        goto cleanup_no_api;
    }
    if (g_state.cfg.verbose)
        LOG_INFO("SDRplay API opened successfully");

    /* Check API version */
    {
        float ver = 0.0f;
        sdrplay_api_ErrT ver_err;
        int ver_attempts = 0;
        /* A handful of retries here specifically: if the previous session
         * ended via the StopPending "continuing anyway" path, the API
         * *service* (a separate Windows process, not just this one) can
         * still be settling for a moment even though sdrplay_api_Close()
         * already returned on our end - a version query issued into that
         * window can fail transiently. Previously this call's return
         * value wasn't checked at all, so a failure here left ver
         * uninitialized and got compared anyway - "API version 3.0 or
         * later required" was actually stack garbage being less than
         * 3.0, not a real version mismatch, which is why it kept
         * misreporting the same wrong cause on every immediate retry.  */
        for (;;) {
            ver_err = sdrplay_api_ApiVersion(&ver);
            if (ver_err == sdrplay_api_Success) break;
            if (++ver_attempts > 5) break;
            Sleep(400);
        }
        if (ver_err != sdrplay_api_Success) {
            LOG_ERROR("sdrplay_api_ApiVersion failed: %s",
                      sdrplay_api_GetErrorString(ver_err));
            LOG_ERROR("The SDRplay API service may still be busy finishing "
                      "the previous session - try again in a few seconds.");
            rc = 1;
            goto cleanup_api;
        }
        if (g_state.cfg.verbose)
            LOG_INFO("SDRplay API version: %.2f", ver);
        if (ver < 3.0f) {
            LOG_ERROR("API version 3.0 or later required");
            rc = 1;
            goto cleanup_api;
        }
    }

    /* ------------------------------------------------------------------ */
    /* Step 2: Enumerate devices                                           */
    /* ------------------------------------------------------------------ */
    err = sdrplay_api_LockDeviceApi();
    if (err != sdrplay_api_Success) {
        LOG_ERROR("LockDeviceApi: %s", sdrplay_api_GetErrorString(err));
        rc = 1;
        goto cleanup_api;
    }

    /* A session that just released this same device (Monitor turned off
     * then straight back on, or similar) can leave the driver needing a
     * brief moment to actually settle before it's ready to be re-
     * enumerated, even though DuoDX's own g_device_busy guard already
     * correctly waited for sdrplay_api_Close() to return. "err=Success
     * but zero devices" is the tell - a genuine "nothing connected"
     * failure would normally show a real error code instead. Retry
     * briefly rather than immediately reporting this as fatal.           */
    {
        int enum_attempts = 0;
        for (;;) {
            err = sdrplay_api_GetDevices(devices, &num_devices, 6);
            if (err == sdrplay_api_Success && num_devices > 0) break;
            if (++enum_attempts > 25) break;
            if (enum_attempts == 1)
                LOG_INFO("No devices found yet - the driver may still be "
                         "releasing a device from a session that just "
                         "ended. Retrying for a few seconds...");
            Sleep(200);
        }
    }
    if (err != sdrplay_api_Success || num_devices == 0) {
        LOG_ERROR("No SDRplay devices found (err=%s)", sdrplay_api_GetErrorString(err));
        sdrplay_api_UnlockDeviceApi();
        rc = 1;
        goto cleanup_api;
    }

    if (g_state.cfg.verbose)
        LOG_INFO("Found %u SDRplay device(s):", num_devices);
    for (i = 0; i < num_devices; i++) {
        if (g_state.cfg.verbose)
            LOG_INFO("  [%u] hwVer=%u  SerNo=%s  tuner=%d",
                 i, devices[i].hwVer, devices[i].SerNo, devices[i].tuner);
    }

    /* Select device - by serial number if specified, otherwise first found */
    {
        unsigned int selected = 0;
        if (g_state.cfg.device_serial[0]) {
            int found = 0;
            for (i = 0; i < num_devices; i++) {
                if (strncmp(devices[i].SerNo,
                            g_state.cfg.device_serial, 63) == 0) {
                    selected = i;
                    found = 1;
                    break;
                }
            }
            if (!found) {
                LOG_ERROR("Device with serial '%s' not found.",
                          g_state.cfg.device_serial);
                LOG_ERROR("Available devices:");
                for (i = 0; i < num_devices; i++)
                    LOG_ERROR("  [%u] SerNo=%s hwVer=%u",
                              i, devices[i].SerNo, devices[i].hwVer);
                sdrplay_api_UnlockDeviceApi();
                rc = 1;
                goto cleanup_api;
            }
        }
        g_state.device = devices[selected];
        g_last_known_hwVer = g_state.device.hwVer;
    }

    /* Identify device name - needed for validation error messages below */
    switch (g_state.device.hwVer) {
    case SDRPLAY_RSP1_ID:     strncpy(device_name, "RSP1",    63); break;
    case SDRPLAY_RSP1A_ID:    strncpy(device_name, "RSP1A",   63); break;
    case SDRPLAY_RSP2_ID:     strncpy(device_name, "RSP2",    63); break;
    case SDRPLAY_RSPduo_ID:   strncpy(device_name, "RSPduo",  63); break;
    case SDRPLAY_RSPdx_ID:    strncpy(device_name, "RSPdx",   63); break;
    case SDRPLAY_RSPdxR2_ID:  strncpy(device_name, "RSPdxR2", 63); break;
    default:                   strncpy(device_name, "RSP?",    63); break;
    }
    LOG_INFO("Using device: %s (SerNo: %s)", device_name, g_state.device.SerNo);

    /* Freeze the antenna and sample rate for this session now, before any
     * later Settings save (while listening) can change cfg out from
     * under the status line or the monitor's DSP math - see the
     * live_antenna and live_expected_output_rate_hz field comments in
     * AppState.                                                         */
    strncpy(g_state.live_antenna, g_state.cfg.antenna, sizeof(g_state.live_antenna) - 1);
    g_state.live_antenna[sizeof(g_state.live_antenna) - 1] = '\0';
    g_state.live_expected_output_rate_hz = g_state.cfg.expected_output_rate_hz;
    g_state.live_expected_usable_bw_hz = g_state.cfg.expected_usable_bw_hz;

    /* Pre-populate infobar so it shows immediately, before monitor starts */
    if (g_state.cfg.dual_channel)
        snprintf(g_ui.infobar, sizeof(g_ui.infobar),
                 "%s  \xB7  Ant: %s  \xB7  GR: %d/%d dB  LNA: %d/%d  \xB7  SR: %.3g Msps",
                 device_name,
                 g_state.live_antenna[0] ? g_state.live_antenna : "-",
                 g_state.cfg.gain_reduction,
                 g_state.cfg.gain_reduction_b >= 0 ? g_state.cfg.gain_reduction_b : g_state.cfg.gain_reduction,
                 g_state.cfg.lna_state,
                 g_state.cfg.lna_state_b >= 0 ? g_state.cfg.lna_state_b : g_state.cfg.lna_state,
                 g_state.cfg.expected_output_rate_hz / 1e6);
    else
        snprintf(g_ui.infobar, sizeof(g_ui.infobar),
                 "%s  \xB7  Ant: %s  \xB7  GR: %d dB  LNA: %d  \xB7  SR: %.3g Msps",
                 device_name,
                 g_state.live_antenna[0] ? g_state.live_antenna : "-",
                 g_state.cfg.gain_reduction,
                 g_state.cfg.lna_state,
                 g_state.cfg.expected_output_rate_hz / 1e6);

    /* Check for RSPduo dual mode request */
    if (g_state.cfg.dual_channel) {
        if (g_state.device.hwVer != SDRPLAY_RSPduo_ID) {
            LOG_ERROR("dual_channel=1 requires an RSPduo - found %s. "
                      "Set dual_channel=0 in duodx.ini.",
                      device_name);
            sdrplay_api_UnlockDeviceApi();
            rc = 1;
            goto cleanup_api;
        }

        /* Now that Record Now can no longer retroactively swap frequencies
         * mid-wait (replaced by the Schedule Enable/Disable toggle), this
         * is a direct, single check - g_state.cfg.frequency_hz/freq_b_hz
         * at this point are exactly what this session will actually use,
         * whether that's schedule_1's promoted values (schedule_only=1)
         * or the plain top-level values (schedule_only=0).                */
        g_state.master_slave_active =
            fabs(g_state.cfg.frequency_hz - g_state.cfg.freq_b_hz) > 100.0;

        if (g_state.master_slave_active && g_state.cfg.schedule_count > 1) {
            /* Multi-entry schedules can advance to schedule_2, schedule_3,
             * etc. with their own frequency pairs, which aren't checked
             * above (only schedule_1's promoted pair is) - re-deciding
             * Master/Slave on every schedule advance isn't supported yet,
             * so play it safe here. A single schedule_1 entry is fully
             * covered by the check above and doesn't hit this.           */
            LOG_WARN("Master/Slave mode (different Tuner A/B frequencies) "
                     "is not yet supported together with multi-entry "
                     "scheduling - falling back to Dual_Tuner mode for "
                     "this session. Tuner B's frequency may not apply "
                     "correctly (this is the known issue being tracked). "
                     "A single scheduled entry, or a plain immediate "
                     "Record, supports Master/Slave fully.");
            g_state.master_slave_active = 0;
        }

        if (g_state.master_slave_active) {
            /* Different CFs: Dual_Tuner mode does not reliably apply an
             * independent RF frequency to Tuner B on this hardware/driver
             * (confirmed by direct IQ analysis, not just the API's return
             * code). Run this session as Master (Tuner A only) - Tuner B
             * is recorded by a separate Slave process, launched further
             * below once the output filenames are known. From here on
             * this session behaves exactly like an ordinary single-tuner
             * recording of Tuner A.                                     */
            g_state.device.rspDuoMode = sdrplay_api_RspDuoMode_Master;
            g_state.device.tuner      = sdrplay_api_Tuner_A;
            g_state.device.rspDuoSampleFreq = g_state.cfg.sample_rate_hz;
            g_state.cfg.dual_channel = 0;
            LOG_INFO("RSPduo Master/Slave mode: Tuner A (Master, this "
                     "process) %.4f MHz, Tuner B (Slave, separate "
                     "process) %.4f MHz.",
                     g_state.cfg.frequency_hz / 1e6,
                     g_state.cfg.freq_b_hz / 1e6);
        } else {
            /* Same CF on both tuners - phase-coherent Dual_Tuner mode,
             * unaffected by the Tuner B frequency bug since both tuners
             * want the same frequency anyway.                          */
            g_state.device.rspDuoMode      = sdrplay_api_RspDuoMode_Dual_Tuner;
            g_state.device.tuner           = sdrplay_api_Tuner_Both;
            g_state.device.rspDuoSampleFreq = g_state.cfg.sample_rate_hz;
        }
    } else if (g_state.device.hwVer == SDRPLAY_RSPduo_ID) {
        /* RSPduo in single-tuner mode: rspDuoMode and tuner must be set
         * explicitly before SelectDevice, otherwise the API leaves them at
         * their zero-initialised defaults (RspDuoMode_Unknown / Tuner_Neither)
         * and subsequent sdrplay_api_Update calls for gain, LNA state, and AGC
         * silently fail - the hardware does not respond to any parameter changes
         * during recording.                                                      */
        g_state.device.rspDuoMode      = sdrplay_api_RspDuoMode_Single_Tuner;
        if (!strcmp(g_state.cfg.rspduo_single_tuner, "B")) {
            g_state.device.tuner = sdrplay_api_Tuner_B;
            LOG_INFO("RSPduo single-tuner mode: Tuner B (Tuner 2) selected");
        } else {
            g_state.device.tuner = sdrplay_api_Tuner_A;
            LOG_INFO("RSPduo single-tuner mode: Tuner A (Tuner 1) selected");
        }
        /* rspDuoSampleFreq is only used in dual-tuner master mode;
         * leave it at 0 for single-tuner mode.                    */
    }

    /* Validate antenna value for non-RSPduo devices.
     * 50ohm and Hi-Z are RSPduo Tuner 1 port names - meaningless elsewhere.
     * Warn and fall back to antenna A so recording can still proceed.       */
    if (g_state.device.hwVer != SDRPLAY_RSPduo_ID) {
        if (!strcmp(g_state.cfg.antenna, "50ohm") ||
            !strcmp(g_state.cfg.antenna, "Hi-Z")  ||
            !strcmp(g_state.cfg.antenna, "hi-z")  ||
            !strcmp(g_state.cfg.antenna, "HIZ")) {
            LOG_WARN("Antenna '%s' is only valid for RSPduo Tuner 1 - "
                     "defaulting to antenna A.", g_state.cfg.antenna);
            strncpy(g_state.cfg.antenna, "A", 7);
        }
    }

    err = sdrplay_api_SelectDevice(&g_state.device);
    sdrplay_api_UnlockDeviceApi();
    if (err != sdrplay_api_Success) {
        LOG_ERROR("SelectDevice: %s", sdrplay_api_GetErrorString(err));
        rc = 1;
        goto cleanup_api;
    }

    /* ------------------------------------------------------------------ */
    /* Step 3: Get device parameters                                       */
    /* ------------------------------------------------------------------ */
    err = sdrplay_api_GetDeviceParams(g_state.device.dev, &g_state.dev_params);
    if (err != sdrplay_api_Success) {
        LOG_ERROR("GetDeviceParams: %s", sdrplay_api_GetErrorString(err));
        rc = 1;
        goto cleanup_device;
    }

    /* In single-tuner mode with Tuner B selected, ch_a_params is aliased
     * onto the real Tuner B channel struct rather than Tuner A's - the
     * same trick the Master/Slave Slave process already uses for Tuner B
     * (see run_slave_b_session()). rxChannelB itself stays unpopulated/
     * unused in single-tuner mode regardless of which physical tuner is
     * active - the API only allocates it for dual-tuner modes - so every
     * per-tuner setting (gain, LNA, notch, Bias-T, tuning) needs to go
     * through ch_a_params here, exactly as single-tuner-A already does.  */
    if (g_state.device.hwVer == SDRPLAY_RSPduo_ID &&
            g_state.device.tuner == sdrplay_api_Tuner_B) {
        g_state.ch_a_params = g_state.dev_params->rxChannelB;
    } else {
        g_state.ch_a_params = g_state.dev_params->rxChannelA;
    }
    g_state.ch_b_params = g_state.dev_params->rxChannelB;

    /* ------------------------------------------------------------------ */
    /* Step 4: Configure device parameters                                 */
    /* ------------------------------------------------------------------ */

    /* Device-specific parameter validation now that hwVer is known.      */
    {
        unsigned char hw = g_state.device.hwVer;
        int max_lna;
        const char *dev_name;

        if      (hw == SDRPLAY_RSP1_ID)   { max_lna = 3;  dev_name = "RSP1"; }
        else if (hw == SDRPLAY_RSP1A_ID)  { max_lna = 9;  dev_name = "RSP1A"; }
        else if (hw == SDRPLAY_RSP1B_ID)  { max_lna = 9;  dev_name = "RSP1B"; }
        else if (hw == SDRPLAY_RSP2_ID)   { max_lna = 8;  dev_name = "RSP2"; }
        else if (hw == SDRPLAY_RSPduo_ID) { max_lna = 9;  dev_name = "RSPduo"; }
        else if (hw == SDRPLAY_RSPdx_ID)  { max_lna = 27; dev_name = "RSPdx"; }
        else if (hw == SDRPLAY_RSPdxR2_ID){ max_lna = 27; dev_name = "RSPdxR2"; }
        else                               { max_lna = 9;  dev_name = "RSP"; }

        if (g_state.cfg.lna_state < 0 || g_state.cfg.lna_state > max_lna) {
            LOG_ERROR("lna_state=%d is out of range for %s (valid: 0-%d).",
                      g_state.cfg.lna_state, dev_name, max_lna);
            LOG_ERROR("0 = maximum LNA gain (most sensitive).");
            LOG_ERROR("%d = maximum LNA attenuation for %s.", max_lna, dev_name);
            LOG_ERROR("For MW DX with a good antenna, lna_state=0 is usually correct.");
            goto cleanup_device;
        }

        if (g_state.cfg.dual_channel) {
            int max_lna_b = max_lna;   /* same device, same limit */
            /* -1 means "inherit from Tuner A" - resolve before validating,
             * matching the inheritance logic used when applying gain.    */
            int lna_b = (g_state.cfg.lna_state_b >= 0)
                        ? g_state.cfg.lna_state_b : g_state.cfg.lna_state;
            if (lna_b < 0 || lna_b > max_lna_b) {
                LOG_ERROR("lna_state_b=%d is out of range for %s (valid: 0-%d).",
                          lna_b, dev_name, max_lna_b);
                goto cleanup_device;
            }
        }

        /* duration: 0 (00:00:00) = unlimited, otherwise must be positive.
         * parse_duration_hms() already clamps negative components to 0,
         * so this mainly guards against duration_sec being set to a
         * negative value through some other internal path.             */
        if (g_state.cfg.duration_sec < 0) {
            LOG_ERROR("duration is invalid. Use 00:00:00 for unlimited or "
                      "a positive HH:MM:SS value.");
            goto cleanup_device;
        }

        /* ring_buffer_sec: must be at least 1 second */
        if (g_state.cfg.ring_buffer_sec < 1) {
            LOG_ERROR("ring_buffer_sec=%d is invalid (minimum 1).",
                      g_state.cfg.ring_buffer_sec);
            goto cleanup_device;
        }

        /* ppm: sanity check against fat-fingered entries - the SDRplay API
         * documentation (checked across multiple spec versions, R3.0
         * through 3.15) states no minimum or maximum for this field at
         * all, just "double ppm; // default: 0.0" - there's no real
         * hardware limit to respect here. The previous ±100 cap was an
         * arbitrary guess, not a documented one, and turned out to be too
         * tight for real-world manual calibration (~385 ppm reported
         * needed on an actual RSPdx without GPS discipline, via SDRuno).
         * ±1000 still catches genuinely absurd entries (a stray extra
         * digit, wrong units) while comfortably covering any real
         * calibration value.                                            */
        if (g_state.cfg.ppm < -1000.0 || g_state.cfg.ppm > 1000.0) {
            LOG_ERROR("ppm=%.1f is outside the expected range (-1000 to +1000 ppm). "
                      "Check your ppm correction value.", g_state.cfg.ppm);
            goto cleanup_device;
        }

        /* hdr_bw_khz: must be one of the four valid HDR bandwidths */
        if (g_state.cfg.hdr_enable) {
            int bw = g_state.cfg.hdr_bw_khz;
            if (bw != 200 && bw != 500 && bw != 1200 && bw != 1700) {
                LOG_ERROR("hdr_bw_khz=%d is invalid. "
                          "Valid HDR bandwidths: 200, 500, 1200, 1700 kHz.", bw);
                goto cleanup_device;
            }
        }

        /* frequency_mhz: with HDR on, must land on one of the ten centre
         * frequencies HDR mode actually supports. Previously only the
         * Settings dialog hint caught this (cosmetic, non-blocking) -
         * apply_hdr_mode() would silently fall back to a normal non-HDR
         * recording with an easy-to-miss log line. Now enforced here too,
         * so a mismatched frequency stops the session instead of quietly
         * recording without HDR. */
        if (g_state.cfg.hdr_enable) {
            double freq_khz = g_state.cfg.frequency_hz / 1000.0;
            int i, valid = 0;
            for (i = 0; i < NUM_HDR_VALID_KHZ; i++) {
                if (fabs(freq_khz - HDR_VALID_KHZ[i]) < 0.5) { valid = 1; break; }
            }
            if (!valid) {
                LOG_ERROR("frequency=%.6g MHz is not a valid HDR centre "
                          "frequency. Valid: 135/175/220/250/340/475/516/"
                          "875/1125/1900 kHz.", g_state.cfg.frequency_hz / 1e6);
                goto cleanup_device;
            }
        }

        /* monitor_interval_ms range */
        if (g_state.cfg.monitor_interval_ms < 100 ||
            g_state.cfg.monitor_interval_ms > 5000) {
            LOG_WARN("monitor_interval_ms=%d is outside recommended range "
                     "(100-5000). Using 500 ms.",
                     g_state.cfg.monitor_interval_ms);
            g_state.cfg.monitor_interval_ms = 500;
        }

        if (g_state.cfg.verbose)
            LOG_INFO("Device parameters validated for %s "
                 "(GR=%d dB, LNA=%d/%d).",
                 dev_name, g_state.cfg.gain_reduction,
                 g_state.cfg.lna_state, max_lna);
    }

    if (g_state.cfg.dual_channel) {
        num_channels = 2;
        setup_device_rspduo_dual(&g_state);
    } else {
        num_channels = 1;
        setup_device_single(&g_state);
    }

    /* Different CFs on the two tuners can't be represented in one Linrad
     * dual-channel file (it has only one centre-frequency field), so write
     * two separate single-channel files instead. Capture/ring/device setup
     * above is unchanged - both tuners still stream together as normal;
     * only the file-writing step differs (see writer_thread_func).        */
    g_state.dual_separate_files =
        g_state.cfg.dual_channel &&
        g_state.cfg.output_format == FORMAT_LINRAD &&
        fabs(g_state.cfg.frequency_hz - g_state.cfg.freq_b_hz) > 100.0;


    /* Apply HDR mode to device params before Init (RSPdx only) */
    apply_hdr_mode(&g_state);
    if (g_state.cfg.hdr_enable)
        LOG_OK("HDR mode active: CF=%.0f kHz  BW=%d kHz",
               g_state.cfg.frequency_hz / 1000.0, g_state.cfg.hdr_bw_khz);

    /* ------------------------------------------------------------------ */
    /* Step 5: Allocate ring buffer                                        */
    /* ------------------------------------------------------------------ */
    {
        /* bytes per second = sample_rate * bytes_per_frame
         * bytes_per_frame  = 4 (single) or 8 (dual), 16-bit I+Q */
        SIZE_T bytes_per_sec = (SIZE_T)(g_state.cfg.sample_rate_hz)
                               * (num_channels == 2 ? 8 : 4);
        ring_size = bytes_per_sec * (SIZE_T)g_state.cfg.ring_buffer_sec;

        if (ring_size < RING_BUFFER_MIN_BYTES) {
            LOG_WARN("ring_buffer_sec too small - clamped to minimum %d MB.",
                     RING_BUFFER_MIN_BYTES / (1024 * 1024));
            ring_size = RING_BUFFER_MIN_BYTES;
        }

        if (g_state.cfg.verbose)
            LOG_INFO("Allocating ring buffer: %zu MB",
                 ring_size / (1024 * 1024));

        if (ring_init(&g_state.ring, ring_size) != 0) {
            LOG_ERROR("Failed to allocate ring buffer (%zu MB)",
                      ring_size / (1024 * 1024));
            rc = 1;
            goto cleanup_device;
        }
    }

    /* ------------------------------------------------------------------ */
    /* Start HTTP status server before the recording loop so it is       */
    /* available during the scheduled wait and all subsequent recordings. */
    /* ------------------------------------------------------------------ */
    if (g_state.cfg.http_port > 0) {
        http_thread = CreateThread(NULL, 0, http_status_thread_func,
                                   &g_state, 0, NULL);
        if (!http_thread)
            LOG_WARN("HTTP: CreateThread failed - status server disabled.");
        else {
            if (g_state.cfg.verbose)
                LOG_INFO("Open http://<this-pc-ip>:%d/ in a browser to monitor.",
                     g_state.cfg.http_port);
            /* Wait for the HTTP thread to finish binding so its listening
             * message prints before the scheduled countdown begins.     */
            int wait_ms = 0;
            while (!g_http_ready && wait_ms < 2000) {
                Sleep(50);
                wait_ms += 50;
            }
        }
    }

    /* ------------------------------------------------------------------ */
    /* Listening-only start: start the device streaming right now, before */
    /* any of the waiting below, so the live monitor and level meters     */
    /* work while the user adjusts gain settings or simply waits for a    */
    /* recording to begin. Covers three cases: Monitor pressed while idle */
    /* (g_enter_listening_req); hourly mode entered via Record             */
    /* (g_state.cfg.hourly_enable); and any pending start_time wait        */
    /* entered via Record (g_state.cfg.start_time[0]) - which covers both */
    /* a plain top-level start_time delay and schedule_only=1, since the  */
    /* schedule promotion step above copies schedule_1's start_time into  */
    /* this same field before we ever reach this point. All three         */
    /* previously skipped device open entirely until the wait ended,      */
    /* since the various wait loops below predate the Listening feature   */
    /* and were never wired up to it: pressing Monitor during any of      */
    /* these waits turned the button on with nothing actually streaming   */
    /* to feed it, so it produced no audio even though it looked active.  */
    /* The wait logic that follows is completely unchanged - it doesn't   */
    /* know or care that the device happens to already be running; the    */
    /* existing "already initialised" guard at Step 8 below skips the     */
    /* normal Init call once recording actually starts. Covers            */
    /* dual_channel now too, not just single-tuner - see the callback     */
    /* selection just below.                                              */
    /* ------------------------------------------------------------------ */
    if (g_enter_listening_req ||
            g_state.cfg.hourly_enable ||
            g_state.cfg.start_time[0]) {
        g_enter_listening_req = 0;
        memset(&callbacks, 0, sizeof(callbacks));
        callbacks.EventCbFn = event_callback;
        if (g_state.cfg.dual_channel) {
            callbacks.StreamACbFn = stream_callback_dual_a;
            callbacks.StreamBCbFn = stream_callback_dual_b;
        } else {
            callbacks.StreamACbFn = stream_callback_single;
        }

        err = sdrplay_api_Init(g_state.device.dev, &callbacks, &g_state);
        if (err != sdrplay_api_Success) {
            LOG_ERROR("sdrplay_api_Init (listening): %s",
                      sdrplay_api_GetErrorString(err));
            rc = 1;
            goto cleanup_device;
        }
        g_state.listening = 1;

        if (g_state.cfg.dual_channel) {
            /* Same RSPduo Tuner B frequency quirk as the main recording
             * Init() below - see its own comment there for the full
             * explanation (setting rfFreq.rfHz before Init() isn't
             * reliably enough for Tuner B). Needed here too now that
             * Listening can open a genuine dual-tuner stream, not just a
             * single one - both tuners are live from this point, so
             * switching which one the monitor listens to (right-click)
             * is instant from here on, no restart required either way.  */
            sdrplay_api_ErrT ferr;
            g_state.ch_b_params->tunerParams.rfFreq.rfHz = g_state.cfg.freq_b_hz;
            ferr = sdrplay_api_Update(g_state.device.dev, sdrplay_api_Tuner_B,
                                       sdrplay_api_Update_Tuner_Frf,
                                       sdrplay_api_Update_Ext1_None);
            if (ferr != sdrplay_api_Success)
                LOG_WARN("Tuner B frequency update failed: %s",
                         sdrplay_api_GetErrorString(ferr));
            else
                LOG_INFO("Tuner B frequency confirmed: %.6f MHz",
                         g_state.cfg.freq_b_hz / 1e6);
            Sleep(400);
            g_state.ch_a_params->tunerParams.rfFreq.rfHz = g_state.cfg.frequency_hz;
            ferr = sdrplay_api_Update(g_state.device.dev, sdrplay_api_Tuner_A,
                                       sdrplay_api_Update_Tuner_Frf,
                                       sdrplay_api_Update_Ext1_None);
            if (ferr != sdrplay_api_Success)
                LOG_WARN("Tuner A frequency update failed: %s",
                         sdrplay_api_GetErrorString(ferr));
            else
                LOG_INFO("Tuner A frequency confirmed: %.6f MHz",
                         g_state.cfg.frequency_hz / 1e6);
        }
        if (g_state.master_slave_active) {
            /* Same reasoning as the dual_channel branch above - Listening
             * previously never started the Tuner B slave at all, since
             * that only ever happened from the main recording setup
             * further below. Reuses the same, already-working launch
             * function and its pipe-reader thread - listen_only=1 means
             * no file is created (see run_slave_b_session), matching how
             * this process's own Tuner A stream never writes anything
             * during Listening either.                                   */
            if (!launch_slave_b_process(&g_state, NULL, 0, 1)) {
                LOG_WARN("Master/Slave: failed to start the Tuner B slave "
                         "process - Tuner B will not be monitorable this "
                         "session (Tuner A is unaffected).");
                g_state.master_slave_active = 0;
            }
        }
        {
            /* gui_start_listening() (Monitor pressed while idle) already
             * force-enables the monitor's audio, since pressing Monitor is
             * itself a request to hear it. This path also runs for Record
             * pressed with an hourly/schedule/start_time wait pending
             * (g_toggle_btn_recording is 1 in that case, not 0) - there,
             * the device is only being pre-opened so Monitor *can* be
             * turned on if the user chooses to, not so it starts playing
             * audio nobody asked for the moment an unattended recording's
             * wait begins. Leave g_monitor.enabled at whatever it already
             * was (normally 0, reset at the end of the previous session)
             * in that case, and only force it on for a genuine Monitor
             * press.                                                     */
            if (!g_toggle_btn_recording)
                g_monitor.enabled = 1;
            /* Normally applied post-Init at Step 8, which listening mode
             * skips (see the guard there) - without this, listening would
             * monitor whatever antenna port/notch state the device
             * happened to power on with, not the configured one, and
             * then glitch to the correct settings the moment Record was
             * pressed. Applying it here means what you hear while
             * listening is what you'll actually get when you record.    */
            apply_antenna_and_biast(&g_state);
            apply_notch_filters(&g_state);
            if (g_state.cfg.hourly_enable) {
                if (g_state.cfg.verbose)
                    LOG_OK("Receiver on early so Monitor works during the wait "
                           "for the hourly window - recording will start "
                           "automatically when it opens.");
            } else if (g_state.cfg.start_time[0]) {
                if (g_state.cfg.verbose)
                    LOG_OK("Receiver on early so Monitor works during the wait "
                           "for the scheduled start time (%s) - recording will "
                           "start automatically when it arrives.",
                           g_state.cfg.start_time);
            } else {
                LOG_OK("Listening - receiver on, not recording. "
                       "Press Record to start writing a file.");
            }
        }
    }

    /* ------------------------------------------------------------------ */
    /* Step 5b: Scheduled start / pre-recording delay + recording loop    */
    /* ------------------------------------------------------------------ */

hourly_next:
    if (g_state.cfg.hourly_enable) {
        int start_min = hhmm_to_min(g_state.cfg.hourly_start);
        int stop_min  = hhmm_to_min(g_state.cfg.hourly_stop);
        if (start_min < 0 || stop_min < 0) {
            LOG_ERROR("hourly_start or hourly_stop invalid format "
                      "(expected HH:MM).");
            rc = 1;
            goto cleanup_device;
        }
        if (g_state.cfg.hourly_window_min < 2 ||
                g_state.cfg.hourly_window_min > 59) {
            LOG_ERROR("hourly_window_min must be between 2 and 59.");
            rc = 1;
            goto cleanup_device;
        }
        /* Hourly and schedule_only are mutually exclusive - hourly always
         * wins here since it runs after the schedule_only promotion above
         * and unconditionally overwrites what that promotion set up. That
         * was happening silently before; both start_time (from the
         * promoted schedule_1) and the remaining schedule entries are
         * about to be discarded, so say so.                              */
        if (g_state.cfg.schedule_only)
            LOG_WARN("Both hourly_enable=1 and schedule_only=1 are set - "
                     "hourly mode takes priority; the schedule is being "
                     "ignored for this session. Enable only one.");
        /* Override duration_sec and clear schedule entries - always, on
         * every pass through this point, not just the first. Without
         * this running every time, a recording reached via a goto that
         * jumps straight to this label (rather than falling through from
         * above) would keep whatever duration_sec happened to already be
         * set to - e.g. the general "unlimited" setting - instead of the
         * window-sized duration an hourly recording actually needs.      */
        g_state.cfg.duration_sec    = g_state.cfg.hourly_window_min * 60;
        g_state.cfg.schedule_count  = 0;
        g_state.cfg.start_time[0] = '\0';
        LOG_INFO("Hourly mode: %d minute window, session %s-%s",
                 g_state.cfg.hourly_window_min,
                 g_state.cfg.hourly_start,
                 g_state.cfg.hourly_stop);

        int half_win_sec = (g_state.cfg.hourly_window_min * 60) / 2;

        /* Check if we are within the nightly session window.
         * Recordings start half_win_sec before the hour, so the effective
         * session open time is hourly_start minus half the window.
         * e.g. session 13:00, 10-min window: first recording starts 12:55 */
        /* Use seconds arithmetic to avoid rounding with odd window sizes
         * e.g. 5-min window: half=150s, effective start = HH:57:30 not HH:58 */
        int start_sec_midnight = start_min * 60 - half_win_sec;
        if (start_sec_midnight < 0) start_sec_midnight += 86400;
        int eff_start_min = start_sec_midnight / 60; /* floor to minute for session check */
        int eff_stop_min  = stop_min;

        if (!hourly_in_session(eff_start_min, eff_stop_min)) {
            /* Outside session - wait using seconds precision for accuracy.
             * This is exactly when recording actually begins (the start
             * of the pre-record window), not a separate "session open"
             * moment before it.                                          */
            int eff_start_sec = start_min * 60 - half_win_sec;
            if (eff_start_sec < 0) eff_start_sec += 86400;
            LOG_INFO("Hourly: waiting until %02d:%02d:%02d",
                     eff_start_sec / 3600, (eff_start_sec % 3600) / 60,
                     eff_start_sec % 60);
            while (g_running && !g_cancel_listening && !g_downgrade_to_listening) {
                int cur  = now_sec();
                int diff = eff_start_sec - cur;
                if (diff < 0) diff += 86400;
                if (diff == 0) break;
                snprintf(g_state.next_start, sizeof(g_state.next_start),
                         "%02d:%02d:%02d",
                         eff_start_sec / 3600, (eff_start_sec % 3600) / 60,
                         eff_start_sec % 60);
                Sleep(1000);
            }
            if (!g_running || g_cancel_listening) { rc = 0; goto cleanup_device; }
            if (g_downgrade_to_listening) goto hourly_downgraded;
        }

        /* Wait for next hour pre-record window.
         * If we just entered the session at the top of an hour and are
         * already within the pre-record window, start immediately.     */
        if (!hourly_wait_for_next(half_win_sec)) {
            if (g_downgrade_to_listening) goto hourly_downgraded;
            rc = 0;
            /* Stop the monitor thread before cleanup so it doesn't read
             * freed resources. Writer is already stopped (done in Step 11),
             * but the file and ring still need closing.                   */
            g_worker_active = 0;
            if (g_gui_mon_thread) {
                WaitForSingleObject(g_gui_mon_thread, 2000);
                CloseHandle(g_gui_mon_thread);
                g_gui_mon_thread = NULL;
            }
            goto cleanup_file;
        }
        if (0) {
hourly_downgraded:
            /* Timer was turned off while waiting - cancel just the wait,
             * not the whole session. g_state.cfg.hourly_enable is already
             * 0 (the Timer button set it before signalling this), so
             * falling through past this whole hourly block to the plain
             * "Listening - waiting for Record" check further down is
             * exactly the right next state - it already handles "device
             * open, nothing scheduled, wait for the user" correctly.      */
            g_downgrade_to_listening = 0;
            g_toggle_btn_recording = 0;
            g_state.next_start[0] = '\0';
            if (g_hwnd) PostMessageA(g_hwnd, WM_APP_DOWNGRADED_TO_LISTENING, 0, 0);
            LOG_OK("Timer disabled - wait cancelled, still listening.");
        }
    }

repeat_schedule:
    { int was_adhoc = 0;
    do {
        /* Apply schedule entry if we have one */
        if (g_state.cfg.schedule_count > 0 && sched_idx > 0) {
            ScheduleEntry *e = &g_state.cfg.schedule[sched_idx - 1];
            LOG_INFO("Schedule entry %d of %d",
                     sched_idx + 1, orig_sched_count);
            apply_schedule_entry(&g_state, e);
            /* Wait for this entry's start time if specified */
            if (e->start_time[0]) {
                if (!wait_until_time(e->start_time, 1)) {
                    rc = 0;
                    goto cleanup_writer;
                }
            }
            /* If Record Now was pressed during the wait, restore top-level
             * frequency/duration so the ad-hoc uses original INI settings. */
            if (g_record_now) {
                if (g_state.adhoc_frequency_hz > 0.0)
                    g_state.cfg.frequency_hz = g_state.adhoc_frequency_hz;
                if (g_state.adhoc_freq_b_hz > 0.0)
                    g_state.cfg.freq_b_hz = g_state.adhoc_freq_b_hz;
                if (g_state.adhoc_duration_sec > 0)
                    g_state.cfg.duration_sec = g_state.adhoc_duration_sec;
                g_state.adhoc_recording = 1;
                g_record_now = 0;
                LOG_INFO("Record Now: ad-hoc recording at %.3f MHz for %d sec.",
                         g_state.cfg.frequency_hz / 1e6, g_state.cfg.duration_sec);
            }
        } else {
            /* First (or only) recording - use top-level scheduled start.
             * Loop so Record Now can run an ad-hoc recording then resume
             * waiting for the original scheduled time.                   */
            if (g_state.listening && !g_toggle_btn_recording &&
                    !g_state.cfg.start_time[0]) {
                /* Listening with nothing scheduled - wait indefinitely for
                 * either the Record button (g_record_now) or the listening
                 * session being cancelled, instead of starting immediately
                 * the way a normal blank start_time otherwise would.      */
                LOG_INFO("Listening - waiting for Record (no schedule set).");
                g_in_generic_listen_wait = 1;
                while (g_running && !g_record_now && !g_cancel_listening)
                    Sleep(200);
                g_in_generic_listen_wait = 0;
                if (g_cancel_listening || !g_running) {
                    rc = 0;
                    g_worker_active = 0;
                    if (g_gui_mon_thread) {
                        WaitForSingleObject(g_gui_mon_thread, 2000);
                        CloseHandle(g_gui_mon_thread);
                        g_gui_mon_thread = NULL;
                    }
                    goto cleanup_device;
                }
                if (g_state.cfg.hourly_enable) {
                    /* Start was pressed, but Timer has since been re-armed
                     * (hourly) - e.g. turned off, then back on, before
                     * Start was pressed. Honour the current Timer state by
                     * jumping back into the same hourly-wait entry point a
                     * normal hourly start already uses (session-hours
                     * check, then the per-hour window wait), rather than
                     * silently recording immediately regardless of what
                     * Timer currently says is armed.                       */
                    LOG_INFO("Timer is armed (hourly) - waiting for the "
                             "actual hourly window instead of recording "
                             "immediately.");
                    g_toggle_btn_recording = 1;
                    g_downgrade_to_listening = 0;   /* clear any stale
                        signal from a Timer-off click that happened while
                        this loop (which doesn't watch for it) was running -
                        otherwise the resumed wait sees it immediately and
                        downgrades again with no new click involved.        */
                    g_record_now = 0;   /* clear the signal from the Record
                        press that triggered this branch - the resumed
                        hourly wait doesn't consume it (only the plain
                        "no schedule at all" loop does), so left set it
                        would sit stale and get wrongly consumed by a
                        later, unrelated wait after a subsequent downgrade -
                        triggering a recording with no new press at all.    */
                    if (g_hwnd) PostMessageA(g_hwnd, WM_APP_RECORDING_STARTED, 0, 0);
                    goto hourly_next;
                } else if (g_state.cfg.schedule_only && g_state.cfg.start_time[0]) {
                    /* Same situation, schedule mode - the downgrade above
                     * never actually clears g_state.cfg.start_time (only
                     * next_start, the display value), so the promoted
                     * schedule_1 time is still sitting there intact. No
                     * goto needed here the way hourly needs one - just
                     * reset g_record_now and let execution fall straight
                     * through into the while(start_time[0]) wait loop
                     * immediately below, which resumes the wait exactly
                     * as it would on a normal schedule start. Previously
                     * this fell all the way down to the plain "consumed -
                     * record now" case below instead, since nothing here
                     * checked schedule_only at all - only hourly_enable -
                     * silently starting an ad-hoc recording regardless of
                     * what Timer actually said was armed.                  */
                    LOG_INFO("Timer is armed (schedule) - waiting for the "
                             "scheduled start time instead of recording "
                             "immediately.");
                    g_toggle_btn_recording = 1;
                    g_downgrade_to_listening = 0;   /* same reasoning as
                                                        the hourly case above */
                    if (g_hwnd) PostMessageA(g_hwnd, WM_APP_RECORDING_STARTED, 0, 0);
                    g_record_now = 0;
                } else {
                    g_record_now = 0;   /* consumed - fall through and record
                                          * with whatever settings are current */
                }
            }
            while (g_state.cfg.start_time[0] && g_running) {
                if (!wait_until_time(g_state.cfg.start_time, 0)) {
                    if (g_downgrade_to_listening) {
                        /* Timer turned off while waiting - cancel just the
                         * wait, not the whole session. Falls through to
                         * the same "just listening" loop used above for
                         * the no-schedule-at-all case, since that's
                         * exactly what this now is.                       */
                        g_downgrade_to_listening = 0;
                        g_toggle_btn_recording = 0;
                        g_state.next_start[0] = '\0';
                        if (g_hwnd) PostMessageA(g_hwnd, WM_APP_DOWNGRADED_TO_LISTENING, 0, 0);
                        LOG_OK("Timer disabled - wait cancelled, still listening.");
                        g_in_generic_listen_wait = 1;
                        while (g_running && !g_record_now && !g_cancel_listening)
                            Sleep(200);
                        g_in_generic_listen_wait = 0;
                        if (g_cancel_listening || !g_running) {
                            rc = 0;
                            g_worker_active = 0;
                            if (g_gui_mon_thread) {
                                WaitForSingleObject(g_gui_mon_thread, 2000);
                                CloseHandle(g_gui_mon_thread);
                                g_gui_mon_thread = NULL;
                            }
                            goto cleanup_device;
                        }
                        if (g_state.cfg.hourly_enable) {
                            /* Timer re-armed as hourly while downgraded
                             * from this schedule wait - resume via the
                             * hourly entry point instead of falling
                             * through to an ad-hoc recording.            */
                            LOG_INFO("Timer is armed (hourly) - waiting "
                                     "for the actual hourly window instead "
                                     "of recording immediately.");
                            g_toggle_btn_recording = 1;
                            g_downgrade_to_listening = 0;   /* clear any
                                stale signal, same reasoning as the other
                                two resume points above.                    */
                            g_record_now = 0;   /* same reasoning as the
                                                    first hourly resume
                                                    point above.             */
                            if (g_hwnd) PostMessageA(g_hwnd, WM_APP_RECORDING_STARTED, 0, 0);
                            goto hourly_next;
                        }
                        if (g_state.cfg.schedule_only && g_state.cfg.start_time[0]) {
                            /* Timer re-armed as schedule again - resume
                             * the same wait by re-entering the enclosing
                             * while loop rather than breaking out of it.
                             * g_record_now must be cleared first so the
                             * next iteration takes the normal "scheduled
                             * time reached" path, not the Record Now /
                             * ad-hoc path. This is the second of two
                             * places that needed this exact check - the
                             * other is the plain "no schedule at all"
                             * wait above, reached only when the session
                             * started with nothing armed at all; this one
                             * is reached when a schedule wait was already
                             * running from the very start, which is a
                             * different code path with its own separate
                             * copy of the same downgrade-then-Start gap.  */
                            LOG_INFO("Timer is armed (schedule) - waiting "
                                     "for the scheduled start time instead "
                                     "of recording immediately.");
                            g_toggle_btn_recording = 1;
                            g_downgrade_to_listening = 0;   /* same
                                                                reasoning */
                            if (g_hwnd) PostMessageA(g_hwnd, WM_APP_RECORDING_STARTED, 0, 0);
                            g_record_now = 0;
                            continue;
                        }
                        g_record_now = 0;
                        break;
                    }
                    /* Stop pressed — clean up and exit. */
                    rc = 0;
                    g_worker_active = 0;
                    if (g_gui_mon_thread) {
                        WaitForSingleObject(g_gui_mon_thread, 2000);
                        CloseHandle(g_gui_mon_thread);
                        g_gui_mon_thread = NULL;
                    }
                    goto cleanup_device;
                }
                if (!g_record_now) break;   /* normal scheduled start — proceed */
                /* Record Now: restore original top-level freq/duration so
                 * ad-hoc uses INI settings, not the promoted schedule_1 values. */
                if (g_state.adhoc_frequency_hz > 0.0)
                    g_state.cfg.frequency_hz = g_state.adhoc_frequency_hz;
                if (g_state.adhoc_freq_b_hz > 0.0)
                    g_state.cfg.freq_b_hz = g_state.adhoc_freq_b_hz;
                if (g_state.adhoc_duration_sec > 0)
                    g_state.cfg.duration_sec = g_state.adhoc_duration_sec;
                g_state.adhoc_recording = 1;
                g_record_now = 0;
                LOG_INFO("Record Now: ad-hoc recording at %.3f MHz for %d sec.",
                         g_state.cfg.frequency_hz / 1e6, g_state.cfg.duration_sec);
                break;
            }
            if (!g_running) goto cleanup_device;
        }


        if (!g_running) {
            rc = 0;
            goto cleanup_writer;
        }

    /* ------------------------------------------------------------------ */
    /* Step 6: Generate output filename if not explicitly set, then open   */
    /* ------------------------------------------------------------------ */

    /* Resolve the output file path.
     *
     * Priority order:
     *   1. If output_file is not the default, use it as-is (explicit name).
     *      recording_path is still prepended if output_file has no directory.
     *   2. If output_file is the default "recording.raw", auto-generate a
     *      timestamped filename and prepend recording_path if set.
     *
     * recording_path is always prepended when output_file has no directory
     * component of its own, giving a clean separation between path and name. */
    {
        const char *fname = g_state.cfg.output_file;
        const char *base;
        int need_autogen = 0;

        /* Check if output_file is the default (trigger auto-name) */
        base = strrchr(fname, '\\');
        if (!base) base = strrchr(fname, '/');
        if (base) base++; else base = fname;
        if (strcmp(base, DEFAULT_OUTPUT_FILE) == 0)
            need_autogen = 1;

        if (need_autogen) {
            if (g_state.dual_separate_files || g_state.master_slave_active)
                generate_dual_separate_filenames(&g_state);
            else
                generate_output_filename(&g_state.cfg, num_channels);
        } else if (g_state.dual_separate_files || g_state.master_slave_active) {
            /* User gave an explicit filename - still need two distinct
             * files, so tag both off that same base name. Each file's
             * own header still carries its own tuner's correct frequency
             * regardless of what the filename says.                    */
            strncpy(g_state.output_file_b, g_state.cfg.output_file,
                    MAX_PATH_LEN - 1);
            g_state.output_file_b[MAX_PATH_LEN - 1] = '\0';
            insert_tuner_tag(g_state.output_file_b,
                              sizeof(g_state.output_file_b), "_TunerB");
            insert_tuner_tag(g_state.cfg.output_file,
                              sizeof(g_state.cfg.output_file), "_TunerA");
        }

        /* Prepend recording_path if set and output_file has no directory */
        if (g_state.cfg.recording_path[0]) {
            const char *f = g_state.cfg.output_file;
            int has_dir = (strrchr(f, '\\') != NULL || strrchr(f, '/') != NULL);
            if (!has_dir) {
                char full_path[MAX_PATH_LEN];
                char *rp = g_state.cfg.recording_path;
                int rlen = (int)strlen(rp);
                /* Ensure trailing separator */
                if (rlen > 0 && rp[rlen-1] != '\\' && rp[rlen-1] != '/')
                    snprintf(full_path, MAX_PATH_LEN, "%s\\%s", rp, f);
                else
                    snprintf(full_path, MAX_PATH_LEN, "%s%s", rp, f);
                strncpy(g_state.cfg.output_file, full_path, MAX_PATH_LEN-1);
            }
            if (g_state.dual_separate_files || g_state.master_slave_active) {
                const char *fb = g_state.output_file_b;
                int has_dir_b = (strrchr(fb, '\\') != NULL || strrchr(fb, '/') != NULL);
                if (!has_dir_b) {
                    char full_path_b[MAX_PATH_LEN];
                    char *rp = g_state.cfg.recording_path;
                    int rlen = (int)strlen(rp);
                    if (rlen > 0 && rp[rlen-1] != '\\' && rp[rlen-1] != '/')
                        snprintf(full_path_b, MAX_PATH_LEN, "%s\\%s", rp, fb);
                    else
                        snprintf(full_path_b, MAX_PATH_LEN, "%s%s", rp, fb);
                    strncpy(g_state.output_file_b, full_path_b, MAX_PATH_LEN-1);
                }
            }
        }
    }

    if (g_state.master_slave_active) {
        if (g_state.slave_process) {
            /* A listen-only slave from a preceding Listening session is
             * already running - stop it cleanly before starting the real
             * recording one below, otherwise both would try to use the
             * same physical Tuner B at once. Brief gap in Tuner B's
             * stream during this handoff is expected and harmless -
             * nothing was being written to a file yet anyway.
             * stop_slave_b_process() unconditionally clears
             * master_slave_active as part of its normal "session truly
             * over" cleanup - correct there, wrong here, since we're
             * about to relaunch within the same still-active Master/
             * Slave session. Restore it immediately after, or everything
             * downstream that checks this flag (the meter, the audio
             * feed, the right-click tuner switch) silently starts
             * behaving as if Master/Slave mode had ended - which is
             * exactly the bug this fixes: Tuner B's process was still
             * genuinely running and streaming correctly the whole time,
             * only the flag was wrong.                                   */
            LOG_INFO("Master/Slave: stopping the listen-only Tuner B "
                     "process so recording can start on a fresh one.");
            stop_slave_b_process(&g_state);
            g_state.master_slave_active = 1;
        }
        if (!launch_slave_b_process(&g_state, g_state.output_file_b,
                                     g_state.cfg.duration_sec, 0)) {
            LOG_ERROR("Master/Slave: failed to start the Tuner B slave "
                      "process - Tuner B will not be recorded this session.");
            g_state.master_slave_active = 0;
        }
    }

    {
        const char *fmt_name =
            g_state.cfg.output_format == FORMAT_WAVVIEWDX  ? "WavViewDX-raw" :
            g_state.cfg.output_format == FORMAT_SDRUNO     ? "SDRuno WAV (216-byte header)" :
            g_state.cfg.output_format == FORMAT_WINRAD     ? "Winrad WAV (216-byte header)" :
            g_state.cfg.output_format == FORMAT_SDRCONNECT ? "SDR Connect WAV (80-byte header)" :
                                                              "Linrad (41-byte header)";
        LOG_INFO("Output format : %s", fmt_name);
        if (g_state.cfg.dual_channel && !g_state.master_slave_active &&
                (g_state.cfg.output_format == FORMAT_SDRUNO ||
                 g_state.cfg.output_format == FORMAT_WINRAD ||
                 g_state.cfg.output_format == FORMAT_SDRCONNECT)) {
            LOG_ERROR("Output format '%s' does not support interleaved "
                      "dual-channel mode (same CF on both tuners).", fmt_name);
            LOG_ERROR("Set dual_channel=0 and specify which tuner to record:");
            LOG_ERROR("  Tuner 1 (A): dual_channel=0  antenna=Hi-Z  (or 50ohm)");
            LOG_ERROR("  Tuner 2 (B): dual_channel=0  (connect antenna to Tuner 2 port)");
            LOG_ERROR("(Different CFs on the two tuners record fine as two "
                      "separate files in any format - only same-CF "
                      "interleaved dual needs Linrad.)");
            rc = 1;
            goto cleanup_ring;
        }
        if (g_state.master_slave_active &&
                (g_state.cfg.output_format == FORMAT_SDRUNO ||
                 g_state.cfg.output_format == FORMAT_WINRAD ||
                 g_state.cfg.output_format == FORMAT_SDRCONNECT)) {
            LOG_INFO("Different CFs on Tuner A/B: recording as two separate "
                     "%s files (one per tuner), not an interleaved dual file.",
                     fmt_name);
        }
    }

    /* ------------------------------------------------------------------ */
    /* Step 6b: Drive spin-up                                              */
    /* Write a 1 MB block to a temporary file on the recording drive to   */
    /* wake a spinning disk from idle before the ring buffer starts        */
    /* filling. Skipped automatically if the output is on C: (typically   */
    /* an SSD) or if spinup_enable=0.                                      */
    /* ------------------------------------------------------------------ */
    if (g_state.cfg.spinup_enable) {
        /* Build the spinup temp file path in the recording directory.
         * We use the recording directory (or output file's directory)
         * rather than the volume root, which requires admin rights.     */
        char spinup_path[MAX_PATH_LEN];
        const char *rp = g_state.cfg.recording_path;

        if (rp[0]) {
            /* recording_path is set - use it directly */
            size_t rplen = strlen(rp);
            if (rp[rplen-1] == '\\' || rp[rplen-1] == '/')
                snprintf(spinup_path, MAX_PATH_LEN,
                         "%s_duodx_spinup.tmp", rp);
            else
                snprintf(spinup_path, MAX_PATH_LEN,
                         "%s\\_duodx_spinup.tmp", rp);
        } else {
            /* No recording_path - use same directory as output file,
             * falling back to system temp if output_file has no dir.  */
            const char *op = g_state.cfg.output_file;
            char dir[MAX_PATH_LEN];
            strncpy(dir, op, MAX_PATH_LEN - 1);
            dir[MAX_PATH_LEN - 1] = '\0';
            char *last = strrchr(dir, '\\');
            if (!last) last = strrchr(dir, '/');
            if (last) {
                last[1] = '\0';
                snprintf(spinup_path, MAX_PATH_LEN,
                         "%s_duodx_spinup.tmp", dir);
            } else {
                /* Bare filename - use system temp directory */
                char tmp_dir[MAX_PATH_LEN];
                if (GetTempPathA(MAX_PATH_LEN, tmp_dir))
                    snprintf(spinup_path, MAX_PATH_LEN,
                             "%s_duodx_spinup.tmp", tmp_dir);
                else
                    snprintf(spinup_path, MAX_PATH_LEN,
                             "_duodx_spinup.tmp");
            }
        }

        HANDLE hsp = CreateFileA(spinup_path, GENERIC_WRITE, 0, NULL,
                                 CREATE_ALWAYS,
                                 FILE_ATTRIBUTE_TEMPORARY |
                                 FILE_FLAG_DELETE_ON_CLOSE,
                                 NULL);
        if (hsp != INVALID_HANDLE_VALUE) {
            static uint8_t spinup_buf[4096];
            DWORD written, remaining = (DWORD)g_state.cfg.spinup_bytes;
            if (g_state.cfg.verbose)
                LOG_INFO("Spinning up drive (writing %d kB to '%s') ...",
                     g_state.cfg.spinup_bytes / 1024, spinup_path);
            while (remaining > 0) {
                DWORD chunk = remaining < sizeof(spinup_buf)
                              ? remaining : (DWORD)sizeof(spinup_buf);
                if (!WriteFile(hsp, spinup_buf, chunk, &written, NULL))
                    break;
                remaining -= written;
            }
            FlushFileBuffers(hsp);
            CloseHandle(hsp);
            if (g_state.cfg.verbose)
                LOG_INFO("Drive spin-up complete.");
        } else {
            LOG_WARN("Drive spin-up: could not create temp file '%s' "
                     "(error %lu) - continuing without spin-up.",
                     spinup_path, GetLastError());
        }
    }

    g_state.out_file = CreateFileA(
        g_state.cfg.output_file,
        GENERIC_WRITE,
        0,              /* No sharing */
        NULL,
        CREATE_ALWAYS,
        FILE_ATTRIBUTE_NORMAL | FILE_FLAG_SEQUENTIAL_SCAN,
        NULL);

    if (g_state.out_file == INVALID_HANDLE_VALUE) {
        LOG_ERROR("Cannot open output file '%s': error %lu",
                  g_state.cfg.output_file, GetLastError());
        rc = 1;
        g_running = 0;   /* prevent retry / schedule continuation on error */
        goto cleanup_ring;
    }

    /* Split mode (SDRuno always; Winrad/SDR Connect when so configured):
     * make sure the part counter starts clean (it's also reset alongside
     * samples_written between recordings, but a first-ever recording in
     * this process needs it set here too).                               */
    if (g_state.cfg.output_format == FORMAT_SDRUNO ||
            ((g_state.cfg.output_format == FORMAT_WINRAD ||
              g_state.cfg.output_format == FORMAT_SDRCONNECT) &&
             g_state.cfg.large_file_mode == LARGE_FILE_SPLIT)) {
        g_state.output_part_number = 1;
    }
    g_state.segment_samples_written = 0;

    /* Writing two separate single-channel files (different CFs) rather
     * than one interleaved dual file - each header says 1 channel. */
    {
        int hdr_channels = g_state.dual_separate_files ? 1 : num_channels;

        if (g_state.cfg.output_format == FORMAT_LINRAD) {
            if (!write_linrad_header(g_state.out_file, &g_state.cfg,
                                      hdr_channels, 0.0)) {
                rc = 1; goto cleanup_file;
            }
            FlushFileBuffers(g_state.out_file);
        } else if (g_state.cfg.output_format == FORMAT_SDRUNO) {
            /* SDRuno never gets RF64 - it can't play those back (see the
             * OutputFormat enum comment) - so this is unconditional.     */
            if (!write_sdruno_header(g_state.out_file, &g_state.cfg)) {
                rc = 1; goto cleanup_file;
            }
            FlushFileBuffers(g_state.out_file);
        } else if (g_state.cfg.output_format == FORMAT_WINRAD) {
            int hdr_ok = (g_state.cfg.large_file_mode == LARGE_FILE_RF64)
                         ? write_sdruno_header_rf64(g_state.out_file, &g_state.cfg)
                         : write_sdruno_header(g_state.out_file, &g_state.cfg);
            if (!hdr_ok) {
                rc = 1; goto cleanup_file;
            }
            FlushFileBuffers(g_state.out_file);
        } else if (g_state.cfg.output_format == FORMAT_SDRCONNECT) {
            int hdr_ok = (g_state.cfg.large_file_mode == LARGE_FILE_RF64)
                         ? write_sdrconnect_header_rf64(g_state.out_file, &g_state.cfg)
                         : write_sdrconnect_header(g_state.out_file, &g_state.cfg);
            if (!hdr_ok) {
                rc = 1; goto cleanup_file;
            }
            FlushFileBuffers(g_state.out_file);
        }
    }

    if (g_state.dual_separate_files) {
        g_state.out_file_b = CreateFileA(
            g_state.output_file_b,
            GENERIC_WRITE,
            0,
            NULL,
            CREATE_ALWAYS,
            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_SEQUENTIAL_SCAN,
            NULL);

        if (g_state.out_file_b == INVALID_HANDLE_VALUE) {
            LOG_ERROR("Cannot open output file '%s': error %lu",
                      g_state.output_file_b, GetLastError());
            rc = 1;
            g_running = 0;
            goto cleanup_file;
        }

        if (!write_linrad_header(g_state.out_file_b, &g_state.cfg, 1,
                                  g_state.cfg.freq_b_hz)) {
            CloseHandle(g_state.out_file_b);
            g_state.out_file_b = INVALID_HANDLE_VALUE;
            rc = 1; goto cleanup_file;
        }
        FlushFileBuffers(g_state.out_file_b);

        LOG_INFO("Output file A : %s (1 channel, 16-bit, %.4f MHz)",
                 g_state.cfg.output_file, g_state.cfg.frequency_hz / 1e6);
        LOG_INFO("Output file B : %s (1 channel, 16-bit, %.4f MHz)",
                 g_state.output_file_b, g_state.cfg.freq_b_hz / 1e6);
    } else {
        LOG_INFO("Output file   : %s (%d channel(s), 16-bit)",
                 g_state.cfg.output_file, num_channels);
    }

    /* ------------------------------------------------------------------ */
    /* Pre-recording disk space check                                       *
     * Calculate available recording time from free space and warn if it   *
     * is less than the requested duration (or less than 1 hour if         *
     * recording indefinitely).                                             *
     * ------------------------------------------------------------------ */
    {
        /* Use recording_path for the disk check — output_file may not
         * have been generated yet and still holds the default name.     */
        const char *dir_path = g_state.cfg.recording_path[0]
                               ? g_state.cfg.recording_path : ".";

        ULARGE_INTEGER free_bytes;
        if (GetDiskFreeSpaceExA(dir_path, &free_bytes, NULL, NULL)) {
            /* bytes per second of recording */
            double bps = g_state.cfg.expected_output_rate_hz
                         * (g_state.cfg.dual_channel ? 8.0 : 4.0);
            LONGLONG free_mb = (LONGLONG)(free_bytes.QuadPart / (1024ULL * 1024ULL));
            double avail_sec = (bps > 0.0)
                               ? (double)free_bytes.QuadPart / bps : 0.0;
            int av_h = (int)avail_sec / 3600;
            int av_m = ((int)avail_sec % 3600) / 60;
            int av_s = (int)avail_sec % 60;

            LOG_INFO("Disk free     : %lld MB  (%.0f sec = %02d:%02d:%02d of recording)",
                     free_mb, avail_sec, av_h, av_m, av_s);

            /* Warn if less than requested duration will fit */
            if (g_state.cfg.duration_sec > 0) {
                if (avail_sec < (double)g_state.cfg.duration_sec) {
                    int req_h = g_state.cfg.duration_sec / 3600;
                    int req_m = (g_state.cfg.duration_sec % 3600) / 60;
                    int req_s = g_state.cfg.duration_sec % 60;
                    LOG_WARN("WARNING: requested duration %02d:%02d:%02d but only "
                             "%02d:%02d:%02d of space available -- recording will "
                             "stop early.",
                             req_h, req_m, req_s, av_h, av_m, av_s);
                }
            } else {
                /* Unlimited duration -- warn if less than 1 hour available */
                if (avail_sec < 3600.0) {
                    LOG_WARN("WARNING: less than 1 hour of recording space "
                             "available (%02d:%02d:%02d).", av_h, av_m, av_s);
                }
            }
        } else {
            LOG_WARN("Could not determine free disk space for '%s' (error %lu).",
                     dir_path, GetLastError());
        }
    }

    /* ------------------------------------------------------------------ */
    /* Step 7: Start writer thread                                         */
    /* ------------------------------------------------------------------ */
    g_state.writer_running     = 1;
    g_state.writer_error       = 0;
    g_state.disk_stop          = 0;
    g_state.writer_ready_event = CreateEvent(NULL, FALSE, FALSE, NULL);

    g_state.writer_thread = CreateThread(NULL, 0,
                                          writer_thread_func,
                                          &g_state, 0, NULL);
    if (!g_state.writer_thread) {
        LOG_ERROR("Failed to create writer thread: %lu", GetLastError());
        rc = 1;
        goto cleanup_file;
    }

    /* Wait for writer thread to be ready */
    WaitForSingleObject(g_state.writer_ready_event, 5000);
    if (g_state.cfg.verbose)
        LOG_INFO("Writer thread started");

    /* ------------------------------------------------------------------ */
    /* Step 8: Set up and start streaming                                  */
    /* ------------------------------------------------------------------ */
    if (g_state.listening) {
        /* Device is already running from the listening phase above - do
         * NOT call sdrplay_api_Init again (the API doesn't expect a second
         * Init on a device that's already streaming). Discard whatever
         * accumulated in the ring buffer while nothing was draining it;
         * the monitor and level meters read straight from the callback,
         * not from the ring, so this has never affected them - it only
         * matters here, so the file writer starts from a clean point
         * rather than including a backlog of pre-Record audio.
         * Listening now opens the device matching whatever dual_channel
         * was set to at the time (see the Listening-only start block
         * above), and settings_save() already forces a clean stop if
         * dual_channel changes while a listening session is running - so
         * by this point the device is guaranteed to already be open in
         * whichever mode g_state.cfg.dual_channel currently says.        */
        ring_reset(&g_state.ring);
        g_state.listening = 0;
        LOG_OK("Recording started (receiver was already running).");
        if (g_hwnd) PostMessageA(g_hwnd, WM_APP_RECORDING_STARTED, 0, 0);
    } else {
        memset(&callbacks, 0, sizeof(callbacks));
        callbacks.EventCbFn = event_callback;

        if (g_state.cfg.dual_channel) {
            callbacks.StreamACbFn = stream_callback_dual_a;
            callbacks.StreamBCbFn = stream_callback_dual_b;
        } else {
            callbacks.StreamACbFn = stream_callback_single;
        }

        if (g_state.cfg.verbose)
            LOG_INFO("Starting stream...");
        err = sdrplay_api_Init(g_state.device.dev, &callbacks, &g_state);
        if (err != sdrplay_api_Success) {
            LOG_ERROR("sdrplay_api_Init: %s", sdrplay_api_GetErrorString(err));
            rc = 1;
            goto cleanup_writer;
        }
    }

    g_state.stream_running = 1;
    g_recording = 1;  /* legacy flag; harmless in GUI build */
    QueryPerformanceCounter(&g_state.start_time);
    g_state.next_start[0] = '\0';  /* clear waiting indicator now recording */

    /* RSPduo dual-tuner mode: force-apply both tuners' RF frequency via an
     * explicit post-Init Update. Setting rfFreq.rfHz in the channel struct
     * before sdrplay_api_Init() is not reliably enough for Tuner B.
     * Independent per-tuner frequency updates in RSPduo dual-tuner mode are
     * a known troublesome area of this API (reports of the same symptom -
     * an Update() that reports success but the RF frequency of the
     * non-master tuner doesn't actually move - exist for other RSPduo
     * client libraries too). Tuner B is updated first, with a short settle
     * delay before Tuner A, in case the driver needs the non-master tuner
     * addressed before the master "re-confirms" its own frequency.        */
    if (g_state.cfg.dual_channel) {
        sdrplay_api_ErrT ferr;

        g_state.ch_b_params->tunerParams.rfFreq.rfHz = g_state.cfg.freq_b_hz;
        ferr = sdrplay_api_Update(g_state.device.dev, sdrplay_api_Tuner_B,
                                   sdrplay_api_Update_Tuner_Frf,
                                   sdrplay_api_Update_Ext1_None);
        if (ferr != sdrplay_api_Success)
            LOG_WARN("Tuner B frequency update failed: %s",
                     sdrplay_api_GetErrorString(ferr));
        else
            LOG_INFO("Tuner B frequency confirmed: %.6f MHz",
                     g_state.cfg.freq_b_hz / 1e6);

        Sleep(400);

        g_state.ch_a_params->tunerParams.rfFreq.rfHz = g_state.cfg.frequency_hz;
        ferr = sdrplay_api_Update(g_state.device.dev, sdrplay_api_Tuner_A,
                                   sdrplay_api_Update_Tuner_Frf,
                                   sdrplay_api_Update_Ext1_None);
        if (ferr != sdrplay_api_Success)
            LOG_WARN("Tuner A frequency update failed: %s",
                     sdrplay_api_GetErrorString(ferr));
        else
            LOG_INFO("Tuner A frequency confirmed: %.6f MHz",
                     g_state.cfg.frequency_hz / 1e6);

        Sleep(400);

        /* Re-assert Tuner B's frequency once more - if Tuner A's own
         * update (as the RSPduo master tuner) causes some internal
         * re-sync that disturbs Tuner B's setting, this gives B the
         * last word. */
        g_state.ch_b_params->tunerParams.rfFreq.rfHz = g_state.cfg.freq_b_hz;
        ferr = sdrplay_api_Update(g_state.device.dev, sdrplay_api_Tuner_B,
                                   sdrplay_api_Update_Tuner_Frf,
                                   sdrplay_api_Update_Ext1_None);
        if (ferr != sdrplay_api_Success)
            LOG_WARN("Tuner B frequency re-assert failed: %s",
                     sdrplay_api_GetErrorString(ferr));
        else
            LOG_INFO("Tuner B frequency re-confirmed: %.6f MHz",
                     g_state.cfg.freq_b_hz / 1e6);

        Sleep(400);
    }

    /* Apply antenna, Bias-T, Hi-Z notch, then RF/DAB notch filters */
    apply_antenna_and_biast(&g_state);
    apply_notch_filters(&g_state);

    /* Create named pipe for real-time monitoring if enabled.
     * PIPE_NOWAIT: writes never block the writer thread - if no client is
     * connected or the client's buffer is full, WriteFile returns
     * immediately without writing (all-or-nothing for a given chunk).
     * NOTE: deliberately NOT combined with FILE_FLAG_OVERLAPPED - every
     * ReadFile/WriteFile against this handle below is issued with a NULL
     * OVERLAPPED pointer, which is undefined behaviour on an overlapped
     * handle and was observed to return partial byte counts instead of
     * the documented all-or-nothing PIPE_NOWAIT result, corrupting I/Q
     * sample alignment downstream. Plain PIPE_NOWAIT (no OVERLAPPED flag)
     * is well-defined for this synchronous, non-blocking use.            */
    if (g_state.cfg.pipe_enable) {
        g_state.pipe_handle = CreateNamedPipeA(
            g_state.cfg.pipe_name,
            PIPE_ACCESS_OUTBOUND,
            PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_NOWAIT,
            1,              /* max instances */
            256 * 1024,     /* out buffer 256 KB */
            0,              /* in buffer (write-only, not needed) */
            0,              /* default timeout */
            NULL);
        if (g_state.pipe_handle == INVALID_HANDLE_VALUE) {
            LOG_WARN("Could not create named pipe '%s' (error %lu) - "
                     "pipe monitoring disabled.",
                     g_state.cfg.pipe_name, GetLastError());
            g_state.cfg.pipe_enable = 0;
        } else {
            /* CreateNamedPipe only creates the instance - a client's
             * CreateFile can't actually attach to it until the server
             * puts it in a listening state by calling ConnectNamedPipe.
             * With PIPE_NOWAIT this returns immediately either way (no
             * client yet: FALSE/ERROR_PIPE_LISTENING, which is normal and
             * expected here, not an error to report), so it's safe to
             * call once right away rather than from a separate thread.  */
            if (!ConnectNamedPipe(g_state.pipe_handle, NULL) &&
                    GetLastError() != ERROR_PIPE_LISTENING &&
                    GetLastError() != ERROR_PIPE_CONNECTED)
                LOG_WARN("Named pipe '%s': ConnectNamedPipe error %lu - "
                         "a client may not be able to attach.",
                         g_state.cfg.pipe_name, GetLastError());
            LOG_OK("Named pipe ready: %s", g_state.cfg.pipe_name);
            LOG_INFO("Connect a compatible IQ client to monitor in real time.");
        }
    }

    g_record_now = 0;   /* clear Start Now override once streaming begins */
    LOG_INFO("Streaming started. Use Stop to end, AGC to toggle gain control.");

    /* ------------------------------------------------------------------ */
    /* Step 9: GUI status labels are updated by gui_monitor_thread_func,   */
    /* which is started once in WinMain for the whole session. AGC is      */
    /* toggled via the AGC button (serviced in the wait loop below).       */
    /* ------------------------------------------------------------------ */

    /* ------------------------------------------------------------------ */
    /* Step 10: Main wait loop                                             */
    /* ------------------------------------------------------------------ */
    while (g_running && !g_state.writer_error) {
        if (g_agc_toggle_req) {
            g_agc_toggle_req = 0;
            gui_apply_agc_toggle(&g_state);
        }
        if (g_state.cfg.duration_sec > 0) {
            LARGE_INTEGER now;
            double elapsed;
            QueryPerformanceCounter(&now);
            elapsed = (double)(now.QuadPart - g_state.start_time.QuadPart)
                      / (double)g_state.perf_freq.QuadPart;
            if (elapsed >= (double)g_state.cfg.duration_sec) {
                LOG_INFO("Duration reached (%.1f seconds)", elapsed);
                break;
            }
        }
        Sleep(100);
    }

    /* ------------------------------------------------------------------ */
    /* Step 11: Clean shutdown                                             */
    /* ------------------------------------------------------------------ */
    g_recording = 0;  /* re-enable console LOG output */
    g_state.stream_running = 0;
    /* Freeze display values for HTTP monitor */
    if (g_state.out_file != INVALID_HANDLE_VALUE) {
        LARGE_INTEGER fs;
        if (GetFileSizeEx(g_state.out_file, &fs))
            g_state.frozen_file_mb = (LONG64)(fs.QuadPart / (1024 * 1024));
    }
    {
        LARGE_INTEGER now;
        QueryPerformanceCounter(&now);
        if (g_state.start_time.QuadPart > 0 && g_state.perf_freq.QuadPart > 0)
            g_state.frozen_elapsed_sec =
                (double)(now.QuadPart - g_state.start_time.QuadPart)
                / (double)g_state.perf_freq.QuadPart;
    }
    /* GUI display copies that survive the between-entry resets, so the
     * elapsed/file readouts keep the last completed recording's values. */
    if (g_state.frozen_elapsed_sec > 0.0)
        g_state.last_display_elapsed = g_state.frozen_elapsed_sec;
    if (g_state.frozen_file_mb >= 0)
        g_state.last_display_file_mb = g_state.frozen_file_mb;
    /* Only mark session complete if this is the last recording —
     * i.e. no more schedule entries remain after this one.        */
    g_state.session_complete = (g_state.samples_received > 0 &&
                                 sched_idx >= g_state.cfg.schedule_count) ? 1 : 0;

    /* Next scheduled start time for dashboard (empty if last recording) */
    memset(g_state.next_start, 0, sizeof(g_state.next_start));
    if (sched_idx == 0 && g_state.cfg.start_time[0]) {
        /* Top-level or ad-hoc path: next start is the top-level start_time. */
        strncpy(g_state.next_start, g_state.cfg.start_time,
                sizeof(g_state.next_start) - 1);
    } else if (sched_idx < g_state.cfg.schedule_count &&
            g_state.cfg.schedule[sched_idx].start_time[0]) {
        strncpy(g_state.next_start,
                g_state.cfg.schedule[sched_idx].start_time,
                sizeof(g_state.next_start) - 1);
    }
    /* Note: start_time is zeroed AFTER the stats block below so the
     * duration calculation remains correct. frozen_json is built here
     * before ring_free / ReleaseDevice / sdrplay_api_Close so all
     * state fields are still valid.                                    */
    snprintf(g_state.frozen_json, sizeof(g_state.frozen_json),
        "{\"elapsed_sec\":%.1f,\"file_mb\":%.1f,\"disk_free_mb\":%lld,"
        "\"overflows\":%ld,\"zero_frames\":%lld,"
        "\"peak_dbfs_a\":%.1f,\"peak_dbfs_b\":%.1f,"
        "\"overload_a\":0,\"overload_b\":0,\"agc_on\":0,\"dual_channel\":%d,"
        "\"disk_warn\":0,\"disk_stop\":0,\"writer_error\":0,\"running\":1,"
        "\"recording\":0,\"hdr_on\":%d,\"session_complete\":%d,"
        "\"waiting\":0,\"next_start\":\"%s\","
        "\"samples_rx\":%lld,\"samples_written\":%lld}",
        g_state.frozen_elapsed_sec,
        (double)g_state.frozen_file_mb,
        (long long)g_state.disk_free_mb,
        g_state.overflows,
        (long long)g_state.zero_frames_written,
        (double)g_state.peak_dbfs,
        (double)g_state.peak_dbfs_b,
        g_state.cfg.dual_channel,
        g_state.cfg.hdr_enable,
        g_state.session_complete,
        g_state.next_start,
        (long long)g_state.samples_received,
        (long long)g_state.samples_written);
    {
        /* Same conditions the hourly/schedule loop-back decisions further
         * down use for "are we about to record again" - keep these two
         * checks in sync, since disagreeing about it is exactly what
         * caused this bug (Step 8 assuming the device was still running
         * when this code had already unconditionally torn it down).      */
        int will_repeat = g_running && !g_state.writer_error && !g_state.disk_stop
                           && (g_state.cfg.hourly_enable || g_state.cfg.schedule_only);
        if (!will_repeat) {
            if (g_state.cfg.verbose)
                LOG_INFO("Stopping stream...");

            /* RSPduo Master/Slave: the master cannot fully un-initialise while
             * the slave is still attached - the API returns
             * sdrplay_api_StopPending and won't proceed until the slave has
             * stopped. Signal and wait for the slave FIRST, then retry Uninit
             * if it's still pending. Calling Uninit while the slave was
             * still running was almost certainly the cause of the crash on
             * Stop - the ordering was backwards.                          */
            if (g_state.master_slave_active)
                stop_slave_b_process(&g_state);
            {
                sdrplay_api_ErrT uerr;
                int uninit_attempts = 0;
                for (;;) {
                    uerr = sdrplay_api_Uninit(g_state.device.dev);
                    if (uerr != sdrplay_api_StopPending) break;
                    if (++uninit_attempts > 25) {
                        LOG_WARN("sdrplay_api_Uninit: still StopPending after 5s - "
                                 "continuing anyway.");
                        /* The device may still be mid-stop internally even
                         * though we're giving up on waiting - tell every
                         * streaming callback to go inert immediately, then
                         * give any already-in-flight callback a moment to
                         * see the flag and return, before anything below
                         * touches the ring/device again.                  */
                        InterlockedExchange(&g_state.teardown_forced, 1);
                        Sleep(100);
                        break;
                    }
                    Sleep(200);
                }
            }
        } else if (g_state.cfg.verbose) {
            LOG_INFO("Recording finished - hourly/schedule repeat is armed, "
                     "so the receiver stays running (Monitor keeps working) "
                     "rather than stopping and reinitialising for the next one.");
        }
    }

    /* Let writer drain the remaining ring buffer data */
    if (g_state.cfg.verbose)
        LOG_INFO("Draining ring buffer (%zu KB remaining)...",
             ring_available(&g_state.ring) / 1024);
    g_state.writer_running = 0;
    WaitForSingleObject(g_state.writer_thread, 30000);
    CloseHandle(g_state.writer_thread);
    CloseHandle(g_state.writer_ready_event);

    /* Print final statistics */
    {
        LARGE_INTEGER end_time;
        double elapsed;
        QueryPerformanceCounter(&end_time);
        elapsed = (double)(end_time.QuadPart - g_state.start_time.QuadPart)
                  / (double)g_state.perf_freq.QuadPart;

        LOG_INFO("Recording complete:");
        if (g_state.cfg.verbose)
            LOG_INFO("  Duration       : %.2f seconds", elapsed);
        if (g_state.cfg.verbose)
            LOG_INFO("  Samples rx     : %lld", (long long)g_state.samples_received);
        if (g_state.cfg.verbose)
            LOG_INFO("  Samples written: %lld (real + zero-fill)",
                 (long long)g_state.samples_written);

        /* Zero-fill and overflow lines: green when clean, orange when not */
        if (g_state.zero_frames_written == 0)
            LOG_OK(  "  Zero-fill frames: 0");
        else
            LOG_WARN("  Zero-fill frames: %lld (dropped samples replaced with silence)",
                     (long long)g_state.zero_frames_written);

        if (g_state.overflows == 0)
            LOG_OK(  "  Overflows      : 0");
        else
            LOG_WARN("  Overflows      : %ld", g_state.overflows);
        if (g_state.cfg.recording_path[0])
        LOG_INFO("  Recording path : %s", g_state.cfg.recording_path);
    if (g_state.dual_separate_files) {
        LOG_INFO("  Output file A  : %s", g_state.cfg.output_file);
        LOG_INFO("  Output file B  : %s", g_state.output_file_b);
    } else {
        LOG_INFO("  Output file    : %s", g_state.cfg.output_file);
    }

        if (g_state.overflows > 0 && g_state.zero_frames_written > 0) {
            LOG_WARN("There were %ld overflow(s). %lld frames of dropped samples "
                     "have been replaced with silence (zeros) to preserve time "
                     "synchronisation. The recording duration is correct but those "
                     "moments will appear as silence in WavViewDX.",
                     g_state.overflows,
                     (long long)g_state.zero_frames_written);
            LOG_WARN("To prevent future overflows: increase ring_buffer_sec "
                     "in the config or record to a faster drive (SSD).");
        } else if (g_state.overflows > 0) {
            LOG_WARN("There were %ld overflow(s). "
                     "Increase ring_buffer_sec or use a faster drive.",
                     g_state.overflows);
        }
        if (g_state.disk_stop)
            LOG_WARN("Recording stopped cleanly due to low disk space.");
        else if (g_state.writer_error)
            LOG_ERROR("Recording may be incomplete due to write errors.");
    }

    /* Freeze elapsed on HTTP monitor now that stats have been printed */
    g_state.start_time.QuadPart = 0;

    /* Close and verify the completed recording file now, so it is
     * immediately accessible to other applications (e.g. WavViewDX, HxD)
     * without needing to stop DuoDX. The file handle was previously left
     * open until the end of the entire session.                           */
    if (g_state.out_file != INVALID_HANDLE_VALUE) {
        /* segment_samples_written, not samples_written: if split mode
         * rolled over to a new part file, samples_written is the
         * cumulative session total but this file only holds the current
         * segment - segment_samples_written tracks that correctly and is
         * identical to samples_written whenever no rollover happened.    */
        if (g_state.segment_samples_written > 0 &&
                (g_state.cfg.output_format == FORMAT_SDRUNO ||
                 g_state.cfg.output_format == FORMAT_WINRAD ||
                 g_state.cfg.output_format == FORMAT_SDRCONNECT))
            finalize_output_header(g_state.out_file, &g_state.cfg,
                            g_state.segment_samples_written);
        CloseHandle(g_state.out_file);
        g_state.out_file = INVALID_HANDLE_VALUE;
        if (g_state.segment_samples_written > 0)
            verify_recording(&g_state.cfg, g_state.segment_samples_written);
    }
    close_and_verify_file_b(&g_state);
    if (g_state.master_slave_active)
        stop_slave_b_process(&g_state);

    /* ── Between-recording reset ─────────────────────────────────────────
     * If there are more schedule entries and no errors, stop the stream,
     * reset counters, reopen the output file, and restart the stream.   */
    int was_adhoc = g_state.adhoc_recording;
    g_state.adhoc_recording = 0;
    if (was_adhoc) {
        strncpy(g_state.cfg.output_file, DEFAULT_OUTPUT_FILE, MAX_PATH_LEN - 1);
        LOG_INFO("Ad-hoc recording complete.");
        /* Restore schedule_1 settings so it records at correct frequency. */
        if (sched_idx == 0 && g_state.sched1_frequency_hz > 0.0) {
            g_state.cfg.frequency_hz = g_state.sched1_frequency_hz;
            g_state.cfg.freq_b_hz    = g_state.sched1_freq_b_hz;
            if (g_state.sched1_duration_sec > 0)
                g_state.cfg.duration_sec = g_state.sched1_duration_sec;
        }
        strncpy(g_state.next_start, g_state.cfg.start_time,
                sizeof(g_state.next_start) - 1);
        snprintf(g_ui.sched, sizeof(g_ui.sched),
                 "Schedule: %d entr%s  starts %s",
                 g_state.cfg.schedule_count,
                 g_state.cfg.schedule_count == 1 ? "y" : "ies",
                 g_state.cfg.start_time);
    } else {
        sched_idx++;
    }
    if (g_running && !g_state.writer_error && rc == 0
            && (was_adhoc || sched_idx <= g_state.cfg.schedule_count)) {
        if (g_state.cfg.verbose)
            LOG_INFO("Preparing for schedule entry %d of %d ...",
                 sched_idx + 1, orig_sched_count);

        /* Stop current stream */
        g_recording = 0;
        g_state.stream_running = 0;
        if (g_state.master_slave_active)
            stop_slave_b_process(&g_state);
        {
            sdrplay_api_ErrT uerr;
            int uninit_attempts = 0;
            for (;;) {
                uerr = sdrplay_api_Uninit(g_state.device.dev);
                if (uerr != sdrplay_api_StopPending) break;
                if (++uninit_attempts > 25) {
                    LOG_WARN("sdrplay_api_Uninit (schedule repeat): still "
                             "StopPending after 5s - continuing anyway.");
                    InterlockedExchange(&g_state.teardown_forced, 1);
                    Sleep(100);
                    break;
                }
                Sleep(200);
            }
        }

        /* Drain and reset ring buffer */
        g_state.writer_running = 0;
        WaitForSingleObject(g_state.writer_thread, 30000);
        CloseHandle(g_state.writer_thread);
        CloseHandle(g_state.writer_ready_event);
        g_state.writer_thread = NULL;

        /* Close and verify the completed recording */
        if (g_state.pipe_handle != INVALID_HANDLE_VALUE) {
            CloseHandle(g_state.pipe_handle);
            g_state.pipe_handle = INVALID_HANDLE_VALUE;
        }
        if (g_state.out_file != INVALID_HANDLE_VALUE) {
            /* Freeze file size for HTTP monitor before closing */
            LARGE_INTEGER fs;
            if (GetFileSizeEx(g_state.out_file, &fs))
                g_state.frozen_file_mb = (LONG64)(fs.QuadPart / (1024 * 1024));
            /* segment_samples_written: see comment at the other close site
             * above - matches samples_written unless a split occurred.   */
            if (g_state.segment_samples_written > 0 &&
                    (g_state.cfg.output_format == FORMAT_SDRUNO ||
                     g_state.cfg.output_format == FORMAT_WINRAD ||
                     g_state.cfg.output_format == FORMAT_SDRCONNECT))
                finalize_output_header(g_state.out_file, &g_state.cfg,
                                g_state.segment_samples_written);
            CloseHandle(g_state.out_file);
            g_state.out_file = INVALID_HANDLE_VALUE;
            if (g_state.segment_samples_written > 0)
                verify_recording(&g_state.cfg, g_state.segment_samples_written);
        }
        close_and_verify_file_b(&g_state);
        if (g_state.master_slave_active)
            stop_slave_b_process(&g_state);

        /* Reset per-recording counters */
        g_state.samples_received    = 0;
        g_state.samples_written     = 0;
        g_state.segment_samples_written = 0;
        g_state.output_part_number  = 1;
        g_state.zero_frames_written = 0;
        g_state.overflows           = 0;
        g_state.peak_dbfs           = -90.0f;
        g_state.peak_dbfs_b         = -90.0f;
        g_state.writer_error        = 0;
        g_state.disk_stop           = 0;
        g_state.teardown_forced     = 0;   /* callbacks live again for the next session */
        g_state.frozen_file_mb      = -1;  /* cleared - new recording starting */
        g_state.frozen_elapsed_sec  = 0.0;
        g_state.session_complete    = 0;
        g_state.start_time.QuadPart = 0;   /* reset elapsed for HTTP monitor */

        /* Reset ring buffer for reuse - writer thread and sdrplay_api_Init
         * are restarted at the top of the next do-loop iteration.        */
        ring_reset(&g_state.ring);
    }

    } while (g_running && !g_state.writer_error && rc == 0
             && (was_adhoc || sched_idx <= g_state.cfg.schedule_count));
    } /* end repeat_schedule scope */

    /* ------------------------------------------------------------------ */
    /* Hourly mode: loop back for next recording if still in session      */
    /* ------------------------------------------------------------------ */
    if (g_running && !g_state.writer_error && !g_state.disk_stop
            && g_state.cfg.hourly_enable) {

        /* Reset counters for next recording */
        g_state.samples_received    = 0;
        g_state.samples_written     = 0;
        g_state.segment_samples_written = 0;
        g_state.output_part_number  = 1;
        g_state.zero_frames_written = 0;
        g_state.overflows           = 0;
        g_state.peak_dbfs           = -90.0f;
        g_state.peak_dbfs_b         = -90.0f;
        g_state.writer_error        = 0;
        g_state.disk_stop           = 0;
        g_state.teardown_forced     = 0;   /* callbacks live again for the next session */
        g_state.disk_warn_issued    = 0;
        g_state.disk_free_mb        = 0;
        g_state.frozen_file_mb      = -1;
        g_state.frozen_elapsed_sec  = 0.0;
        g_state.session_complete    = 0;
        g_state.start_time.QuadPart = 0;
        ring_reset(&g_state.ring);
        /* Reset output filename so a fresh timestamped name is generated
         * for each hourly recording rather than reusing the previous one. */
        strncpy(g_state.cfg.output_file, DEFAULT_OUTPUT_FILE, MAX_PATH_LEN - 1);
        /* The device is still genuinely running continuously through this
         * loop-back (goto hourly_next below jumps to a point well after
         * the actual device-open code, which never re-runs here) - just
         * not writing a file between recordings. Step 8 further down
         * decides whether to call sdrplay_api_Init() again based on this
         * flag; leaving it at 0 (set when the first recording began, never
         * restored) made every second-and-later hourly cycle wrongly
         * re-Init an already-streaming device, which the API doesn't
         * expect and was what silently broke live Monitor audio after the
         * first recording, even though the recording itself kept working. */
        g_state.listening = 1;

        goto hourly_next;
    }

    /* ------------------------------------------------------------------ */
    /* Schedule mode always repeats nightly once all entries complete,     */
    /* the same way hourly mode does above - no separate flag needed.      */
    /* ------------------------------------------------------------------ */
    if (g_running && !g_state.writer_error && !g_state.disk_stop
            && g_state.cfg.schedule_only) {

        /* Close the current output file and writer thread cleanly */
        if (g_state.writer_thread && g_state.writer_running) {
            g_state.writer_running = 0;
            WaitForSingleObject(g_state.writer_thread, 5000);
            CloseHandle(g_state.writer_thread);
            g_state.writer_thread = NULL;
        }
        if (g_state.out_file != INVALID_HANDLE_VALUE) {
            CloseHandle(g_state.out_file);
            g_state.out_file = INVALID_HANDLE_VALUE;
        }
        if (g_state.out_file_b != INVALID_HANDLE_VALUE) {
            CloseHandle(g_state.out_file_b);
            g_state.out_file_b = INVALID_HANDLE_VALUE;
        }

        LOG_INFO("All schedule entries complete - reloading schedule "
                 "for next day.");

        /* Reload config from INI to restore the original schedule entries */
        config_set_defaults(&g_state.cfg);
        config_load_ini(&g_state.cfg, config_file);

        /* Re-apply schedule_only promotion */
        if (!g_state.cfg.schedule_only)
            g_state.cfg.schedule_count = 0;

        if (g_state.cfg.schedule_only && g_state.cfg.schedule_count > 0) {
            ScheduleEntry *e = &g_state.cfg.schedule[0];
            if (e->frequency_hz > 0.0)
                g_state.cfg.frequency_hz = e->frequency_hz;
            if (e->freq_b_hz > 0.0)
                g_state.cfg.freq_b_hz = e->freq_b_hz;
            if (e->duration_sec > 0)
                g_state.cfg.duration_sec = e->duration_sec;
            if (e->output_file[0])
                strncpy(g_state.cfg.output_file, e->output_file, MAX_PATH_LEN-1);
            if (e->antenna[0])
                strncpy(g_state.cfg.antenna, e->antenna,
                        sizeof(g_state.cfg.antenna)-1);
            if (e->start_time[0]) {
                if (time_already_passed(e->start_time))
                    LOG_INFO("Schedule repeat: waiting until tomorrow for %s",
                             e->start_time);
                strncpy(g_state.cfg.start_time, e->start_time,
                        sizeof(g_state.cfg.start_time)-1);
                strncpy(g_state.next_start, e->start_time,
                        sizeof(g_state.next_start)-1);
            }
            int ri;
            for (ri = 0; ri < g_state.cfg.schedule_count - 1; ri++)
                g_state.cfg.schedule[ri] = g_state.cfg.schedule[ri+1];
            g_state.cfg.schedule_count--;
        }

        /* Reset all per-session counters */
        sched_idx                   = 0;
        g_state.samples_received    = 0;
        g_state.samples_written     = 0;
        g_state.segment_samples_written = 0;
        g_state.output_part_number  = 1;
        g_state.zero_frames_written = 0;
        g_state.overflows           = 0;
        g_state.peak_dbfs           = -90.0f;
        g_state.peak_dbfs_b         = -90.0f;
        g_state.writer_error        = 0;
        g_state.disk_stop           = 0;
        g_state.teardown_forced     = 0;   /* callbacks live again for the next session */
        g_state.disk_warn_issued    = 0;
        g_state.disk_free_mb        = 0;
        g_state.frozen_file_mb      = -1;
        g_state.frozen_elapsed_sec  = 0.0;
        g_state.session_complete    = 0;
        g_state.start_time.QuadPart = 0;
        /* next_start already set above - keep it so dashboard shows NEXT AT */
        ring_reset(&g_state.ring);
        /* Device stays open continuously through this loop-back too - see
         * the identical comment on the hourly repeat path above for why
         * this needs restoring before Step 8 is reached again.            */
        g_state.listening = 1;

        goto repeat_schedule;
    }

cleanup_writer:
    /* Stop the GUI status monitor BEFORE freeing engine resources below.
     * The monitor reads g_state.ring etc.; if it keeps running while
     * ring_free()/DeleteCriticalSection() execute it can touch freed memory
     * and crash the process (which looks like the window closing on finish). */
    g_worker_active = 0;
    if (g_gui_mon_thread) {
        WaitForSingleObject(g_gui_mon_thread, 2000);
        CloseHandle(g_gui_mon_thread);
        g_gui_mon_thread = NULL;
    }

    if (g_state.writer_thread && g_state.writer_running) {
        g_state.writer_running = 0;
        WaitForSingleObject(g_state.writer_thread, 5000);
        CloseHandle(g_state.writer_thread);
    }

cleanup_file:
    if (g_state.pipe_handle != INVALID_HANDLE_VALUE) {
        CloseHandle(g_state.pipe_handle);
        g_state.pipe_handle = INVALID_HANDLE_VALUE;
    }
    if (g_state.out_file != INVALID_HANDLE_VALUE) {
        /* segment_samples_written: see comment at the other close sites -
         * matches samples_written unless a split occurred.               */
        if (g_state.segment_samples_written > 0 &&
                (g_state.cfg.output_format == FORMAT_SDRUNO ||
                 g_state.cfg.output_format == FORMAT_WINRAD ||
                 g_state.cfg.output_format == FORMAT_SDRCONNECT))
            finalize_output_header(g_state.out_file, &g_state.cfg,
                            g_state.segment_samples_written);
        CloseHandle(g_state.out_file);
        g_state.out_file = INVALID_HANDLE_VALUE;
        if (g_state.segment_samples_written > 0)
            verify_recording(&g_state.cfg, g_state.segment_samples_written);
    }
    close_and_verify_file_b(&g_state);
    if (g_state.master_slave_active)
        stop_slave_b_process(&g_state);

cleanup_ring:
    /* Stop monitor thread if still running (skipped cleanup_writer path) */
    if (g_gui_mon_thread) {
        g_worker_active = 0;
        WaitForSingleObject(g_gui_mon_thread, 2000);
        CloseHandle(g_gui_mon_thread);
        g_gui_mon_thread = NULL;
    }
    if (g_state.listening) {
        /* Same reasoning as the identical block in cleanup_device below -
         * a goto can land here (e.g. a file-creation failure) while still
         * mid-transition from listening to recording, with the device
         * already streaming from the earlier listening start-up. Must
         * happen before ring_free() just below for the same use-after-
         * free reason. Duplicated rather than restructured into a single
         * checkpoint because other pre-Step-5 error paths jump directly
         * to cleanup_device and must keep working unchanged; this way
         * every route to either label is safe on its own, and the second
         * check below is a harmless no-op once this one has already run. */
        sdrplay_api_ErrT uerr;
        int uninit_attempts = 0;
        for (;;) {
            uerr = sdrplay_api_Uninit(g_state.device.dev);
            if (uerr != sdrplay_api_StopPending) break;
            if (++uninit_attempts > 25) {
                LOG_WARN("sdrplay_api_Uninit (listening): still StopPending "
                         "after 5s - continuing anyway.");
                InterlockedExchange(&g_state.teardown_forced, 1);
                Sleep(100);
                break;
            }
            Sleep(200);
        }
        g_state.listening  = 0;
        g_record_now       = 0;
        g_cancel_listening  = 0;
        g_downgrade_to_listening = 0;
        g_in_generic_listen_wait = 0;
    }
    ring_free(&g_state.ring);

cleanup_device:
    /* Stop the Tuner B slave process if this was a Master/Slave session -
     * cleanup_device is reached directly (skipping cleanup_file, the only
     * other place this is called) by every plain listening-cancel path,
     * including the ordinary "Monitor with no schedule, then Stop"
     * case - by far the most common way a Master/Slave listening
     * session ends. Without this, the slave process was left running
     * indefinitely after Stop, still holding its named monitor pipe
     * open, so the next session's slave would fail to create its own
     * pipe of the same name with ERROR_PIPE_BUSY. Guarded exactly like
     * the cleanup_file call, so if that one already ran (the recording
     * path falls through this label too) master_slave_active is already
     * 0 and this is a safe no-op, not a double-stop.                    */
    if (g_state.master_slave_active)
        stop_slave_b_process(&g_state);

    /* Stop monitor if still running (may have skipped cleanup_writer/ring) */
    if (g_gui_mon_thread) {
        g_worker_active = 0;
        WaitForSingleObject(g_gui_mon_thread, 2000);
        CloseHandle(g_gui_mon_thread);
        g_gui_mon_thread = NULL;
    }
    if (g_state.listening) {
        /* Listening was cancelled (or the app is stopping) before any file
         * was ever opened. The device was successfully Init'd earlier by
         * the listening start-up and is still streaming, so it needs a
         * proper Uninit here - this label is normally reached only when
         * Init never happened at all, so it doesn't otherwise call it.
         * CRITICAL: this must happen before ring_free() below - the
         * streaming callback can still be writing into the ring right up
         * until Uninit actually stops it, so freeing the ring first would
         * be a use-after-free the moment the callback fires again.       */
        sdrplay_api_ErrT uerr;
        int uninit_attempts = 0;
        for (;;) {
            uerr = sdrplay_api_Uninit(g_state.device.dev);
            if (uerr != sdrplay_api_StopPending) break;
            if (++uninit_attempts > 25) {
                LOG_WARN("sdrplay_api_Uninit (listening): still StopPending "
                         "after 5s - continuing anyway.");
                InterlockedExchange(&g_state.teardown_forced, 1);
                Sleep(100);
                break;
            }
            Sleep(200);
        }
        g_state.listening = 0;
        g_record_now       = 0;   /* clear here too, not just defensively
                                   * at the next start - see the reset in
                                   * gui_start_listening() for why this
                                   * flag needs to be reliably clean.      */
        g_cancel_listening  = 0;
        g_downgrade_to_listening = 0;
        g_in_generic_listen_wait = 0;
    }
    /* Safe no-op if the ring was never allocated (rb->buf is NULL from the
     * memset at the top of this function). Needed here because the
     * listening-cancel paths jump straight to this label, skipping over
     * cleanup_ring above even though Step 5 already allocated the ring.  */
    ring_free(&g_state.ring);
    sdrplay_api_ReleaseDevice(&g_state.device);
    g_state.ch_a_params = NULL;
    g_state.ch_b_params = NULL;

cleanup_api:
    sdrplay_api_Close();

cleanup_no_api:
    if (g_state.log_fp) {
        fclose(g_state.log_fp);
        g_state.log_fp = NULL;
    }

    if (g_state.dual_merge_buf) free(g_state.dual_merge_buf);
    if (g_state.pending_a_i)    free(g_state.pending_a_i);
    if (g_state.pending_a_q)    free(g_state.pending_a_q);

    DeleteCriticalSection(&g_state.dual_lock);

    /* Mark the HTTP dashboard session complete before tearing it down. */
    if (g_state.cfg.http_port > 0) {
        g_state.session_complete = 1;
        snprintf(g_state.frozen_json, sizeof(g_state.frozen_json),
            "{\"elapsed_sec\":%.1f,\"file_mb\":%.1f,\"disk_free_mb\":%lld,"
            "\"overflows\":%ld,\"zero_frames\":%lld,"
            "\"peak_dbfs_a\":%.1f,\"peak_dbfs_b\":%.1f,"
            "\"overload_a\":0,\"overload_b\":0,\"agc_on\":0,\"dual_channel\":0,"
            "\"disk_warn\":0,\"disk_stop\":0,\"writer_error\":0,\"running\":1,"
            "\"recording\":0,\"hdr_on\":0,\"session_complete\":1,"
            "\"waiting\":0,\"next_start\":\"\","
            "\"samples_rx\":%lld,\"samples_written\":%lld}",
            g_state.frozen_elapsed_sec,
            (double)(g_state.frozen_file_mb > 0 ? g_state.frozen_file_mb : 0),
            (long long)g_state.disk_free_mb,
            g_state.overflows,
            (long long)g_state.zero_frames_written,
            (double)g_state.peak_dbfs,
            (double)g_state.peak_dbfs_b,
            (long long)g_state.samples_received,
            (long long)g_state.samples_written);
    }

    /* Stop HTTP server. */
    g_running = 0;
    g_http_running = 0;
    if (http_thread) {
        WaitForSingleObject(http_thread, 2000);
        CloseHandle(http_thread);
        http_thread = NULL;
    }

    LOG_INFO("Session ended.");
    if (rc != 0)
        LOG_ERROR("RECORDING FAILED - check errors above");

    /* Tell the UI the worker has finished so it can re-enable Start.
     * g_device_busy is cleared here and ONLY here - unlike g_worker_active
     * above (cleared earlier, in several places, purely to let the GUI
     * monitor thread exit promptly), this is the one point guaranteed to
     * run after sdrplay_api_ReleaseDevice()/sdrplay_api_Close() have
     * actually completed, on every path through this function. That's
     * what makes it safe for gui_start_session()/gui_start_listening() to
     * gate a new session on - see the g_device_busy comment near its
     * declaration for the crash this fixes.                              */
    g_worker_active = 0;
    g_device_busy   = 0;
    if (g_hwnd) PostMessageA(g_hwnd, WM_APP_DONE, (WPARAM)rc, 0);
    return (DWORD)rc;
}

/* =========================================================================
 * GUI: window procedure and WinMain
 * ========================================================================= */

/* Record/Stop button label while not actually recording: "Start" when
 * the Timer is armed (Schedule or Hourly) - pressing it then begins the
 * wait-for-the-armed-time sequence, not an immediate recording - versus
 * plain "Record" for genuine ad-hoc, right-now recording when Timer is
 * off. Doesn't affect the "Stop" label once actually recording.         */
static const char *gui_record_btn_idle_label(void)
{
    return (g_state.cfg.schedule_only || g_state.cfg.hourly_enable)
           ? "Start" : "Record";
}

static void gui_set_recording_ui(int recording)
{
    g_toggle_btn_recording = recording;
    SetWindowTextA(g_hBtnToggle, recording ? "Stop" : gui_record_btn_idle_label());
    EnableWindow(g_hBtnToggle, TRUE);
    EnableWindow(g_hBtnAgc,     recording);
    if (g_hBtnSettings) EnableWindow(g_hBtnSettings, !recording);
    if (g_hBtnSchedToggle) EnableWindow(g_hBtnSchedToggle, TRUE);
    InvalidateRect(g_hBtnToggle, NULL, FALSE);
}

/* While listening (device running, no file open yet), the toggle button
 * stays labelled Record/Start rather than Stop - pressing it is meant to
 * start recording, not end the session. Settings stays enabled and open -
 * saving it pushes gain changes live (see gui_apply_live_gain) so the
 * whole point of listening - tuning gain against the meters - works.
 * Schedule stays disabled, same reasoning as during a real recording.   */
static void gui_set_listening_ui(void)
{
    g_toggle_btn_recording = 0;
    SetWindowTextA(g_hBtnToggle, gui_record_btn_idle_label());
    EnableWindow(g_hBtnToggle, TRUE);
    EnableWindow(g_hBtnAgc,     TRUE);
    if (g_hBtnSettings) EnableWindow(g_hBtnSettings, TRUE);
    if (g_hBtnSchedToggle) EnableWindow(g_hBtnSchedToggle, TRUE);
    InvalidateRect(g_hBtnToggle, NULL, FALSE);
}

static void gui_start_session(void)
{
    if (g_worker_active || g_device_busy) return;

    /* Clear finished state immediately so the window doesn't briefly flash
     * FINISHED on the next repaint before the memset runs below.         */
    g_ui.finished   = 0;
    g_ui.next[0]    = '\0';
    g_ui.infobar[0] = '\0';
    strncpy(g_ui.state, "STARTING", sizeof(g_ui.state) - 1);
    g_state.session_complete = 0;

    if (g_hLog) {
        if (g_log_freeze) {
            LOG_WARN("--- New session started ---");
        } else {
            SetWindowTextA(g_hLog, "");
        }
    }
    g_log_freeze = 0;
    double keep_disk = g_ui.disk_free_mb;
    memset(&g_ui, 0, sizeof(g_ui));
    g_ui.disk_free_mb = keep_disk;
    g_state.last_display_elapsed = 0.0;   /* clear so stop-while-waiting shows IDLE */
    g_state.last_display_file_mb = -1;
    g_ui.peak_a = -90.0f;
    g_ui.peak_b = -90.0f;
    strncpy(g_ui.state, "STARTING", sizeof(g_ui.state) - 1);
    g_record_now = 0;   /* defensive reset - see the matching reset in
                          * gui_start_listening() for why this matters:
                          * wait_until_time() checks it on every call, so
                          * a stuck value here would skip a scheduled
                          * wait immediately instead of waiting for it.   */
    /* Reset the periodic button-sync tracking too, so the first tick of
     * this new session always resyncs correctly instead of possibly
     * comparing against a stale value left over from a previous one.   */
    g_last_agc_on = g_last_agc_enabled = -1;
    g_last_sched_on = g_last_sched_enabled = -1;
    g_last_has_tuner_b = g_last_tuner_sel = g_last_lock_coherent = -1;
    g_last_sched_text[0] = '\0';
    g_last_infobar_text[0] = '\0';
    g_last_infostrip_text[0] = '\0';

    g_worker_active = 1;
    g_device_busy   = 1;
    gui_set_recording_ui(1);
    if (g_hwnd) InvalidateRect(g_hwnd, NULL, FALSE);

    g_worker_thread = CreateThread(NULL, 0, recording_worker, NULL, 0, NULL);
    if (!g_worker_thread) {
        g_worker_active = 0;
        g_device_busy   = 0;
        gui_set_recording_ui(0);
        LOG_ERROR("Failed to start recording worker thread (%lu)",
                  GetLastError());
        return;
    }

    g_gui_mon_thread = CreateThread(NULL, 0, gui_monitor_thread_func,
                                    &g_state, 0, NULL);
}

static void gui_stop_session(int wait_for_exit)
{
    if (!g_worker_active && !g_worker_thread) return;

    /* Signal the engine to stop. */
    g_running      = 0;
    g_http_running = 0;
    g_state.stop_requested = 1;
    g_state.stream_running = 0;

    if (wait_for_exit && g_worker_thread) {
        WaitForSingleObject(g_worker_thread, 35000);
    }
}

/* Monitor pressed while fully idle: start the device streaming (feeding
 * the live monitor and level meters) without opening a file, single-tuner
 * only. Mirrors gui_start_session() but sets g_enter_listening_req so
 * recording_worker takes the listening branch instead of recording
 * immediately - see the comment above that branch for the full picture. */
static void gui_start_listening(void)
{
    if (g_worker_active || g_device_busy) return;

    g_ui.finished   = 0;
    g_ui.next[0]    = '\0';
    g_ui.infobar[0] = '\0';
    strncpy(g_ui.state, "STARTING", sizeof(g_ui.state) - 1);
    g_state.session_complete = 0;

    if (g_hLog) {
        if (g_log_freeze) {
            LOG_WARN("--- Listening ---");
        } else {
            SetWindowTextA(g_hLog, "");
        }
    }
    g_log_freeze = 0;
    double keep_disk = g_ui.disk_free_mb;
    memset(&g_ui, 0, sizeof(g_ui));
    g_ui.disk_free_mb = keep_disk;
    g_state.last_display_elapsed = 0.0;
    g_state.last_display_file_mb = -1;
    g_ui.peak_a = -90.0f;
    g_ui.peak_b = -90.0f;
    strncpy(g_ui.state, "STARTING", sizeof(g_ui.state) - 1);

    g_cancel_listening    = 0;   /* defensive reset before a new session */
    g_downgrade_to_listening = 0; /* same reasoning */
    g_in_generic_listen_wait = 0; /* same reasoning */
    g_record_now          = 0;   /* defensive reset - a Record-then-cancel
                                   * race (or similar) can leave this stuck
                                   * at 1 from an earlier listening session,
                                   * which would make the very next Monitor
                                   * press skip straight into recording.   */
    g_enter_listening_req = 1;
    g_monitor.enabled     = 1;   /* the point of listening is to hear it */

    /* Full reset of the carrier tracker's state, unconditionally, every
     * time a fresh Monitor session actually starts - not relying on the
     * per-sample DSP thread's "did mode/bandwidth change" check, which
     * can find nothing changed (and so skip its own reset) if this
     * session happens to reuse the same settings as the previous one.
     * Confirmed as a real bug: a log's very first line already showed
     * "window #10" only 4 seconds into a session, which the window
     * counter's own accepted-windows-only counting makes impossible
     * unless it started already elevated - carrying over stale state
     * (including the smoothed estimate itself) from an earlier test run,
     * so the display started from an old leftover value and only slowly
     * corrected using new, otherwise-accurate measurements, rather than
     * starting genuinely fresh.                                          */
    g_monitor.carrier_prev_valid          = 0;
    g_monitor.carrier_lpf_state.re        = 0.0f;
    g_monitor.carrier_lpf_state.im        = 0.0f;
    g_monitor.carrier_dbm_baseline_valid  = 0;
    g_monitor.carrier_phase_accum         = 0.0;
    g_monitor.carrier_weight_accum        = 0.0;
    g_monitor.carrier_accum_samples       = 0;
    g_monitor.carrier_offset_valid_pub    = 0;
    g_monitor.carrier_window_count        = 0;
    g_monitor.carrier_last_raw_valid      = 0;
    g_monitor.carrier_consec_agree        = 0;
    g_monitor.carrier_locked_pub          = 0;
    g_monitor.carrier_last_published_valid = 0;
    g_monitor.carrier_settled_count        = 0;
    g_monitor.carrier_settled_pub          = 0;
    g_monitor.carrier_agc_guard_remaining = CARRIER_AGC_GUARD_SAMPLES;
    g_monitor.carrier_window_target       = CARRIER_WINDOW_SAMPLES_MIN;
    g_monitor.carrier_agc_gain_at_start   = 0.0f;

    if (g_ab_auto_pending && g_state.cfg.dual_channel &&
            fabs(g_state.cfg.frequency_hz - g_state.cfg.freq_b_hz) < 1.0) {
        /* Tuner A/B are set to the same frequency and this is the first
         * Monitor press since app start (or since the last Settings Save) -
         * turn A=B on automatically so the two stay in sync from the
         * start, without overriding a later deliberate manual toggle. */
        g_monitor.freq_locked = 1;
        if (g_hBtnFreqLock) InvalidateRect(g_hBtnFreqLock, NULL, TRUE);
    }
    g_ab_auto_pending = 0;   /* consumed either way - only ever fires once
                              * per epoch, whether or not it actually
                              * applied this time. */
    g_last_agc_on = g_last_agc_enabled = -1;
    g_last_sched_on = g_last_sched_enabled = -1;
    g_last_has_tuner_b = g_last_tuner_sel = g_last_lock_coherent = -1;
    g_last_sched_text[0] = '\0';
    g_last_infobar_text[0] = '\0';
    g_last_infostrip_text[0] = '\0';
    g_worker_active = 1;
    g_device_busy   = 1;
    gui_set_listening_ui();
    if (g_hwnd) InvalidateRect(g_hwnd, NULL, FALSE);

    g_worker_thread = CreateThread(NULL, 0, recording_worker, NULL, 0, NULL);
    if (!g_worker_thread) {
        g_worker_active = 0;
        g_device_busy   = 0;
        g_enter_listening_req = 0;
        gui_set_recording_ui(0);
        LOG_ERROR("Failed to start listening (worker thread failed to "
                  "launch, %lu)", GetLastError());
        return;
    }

    g_gui_mon_thread = CreateThread(NULL, 0, gui_monitor_thread_func,
                                    &g_state, 0, NULL);
}

/* Monitor pressed again while listening: tear the device down cleanly
 * without ever having opened a file. Deliberately narrower than
 * gui_stop_session() - only signals the specific wait loops a listening
 * session can be in, rather than the broader g_running flag which has
 * reach into the HTTP server and other threads not relevant here.       */
static void gui_cancel_listening(void)
{
    if (!g_worker_active || !g_state.listening) return;
    g_cancel_listening = 1;
}

/* =========================================================================
 * Custom drawing helpers
 * ========================================================================= */

/* Filled rounded-ish panel with a 1px border. */
static void draw_panel(HDC dc, RECT r)
{
    HBRUSH fill = CreateSolidBrush(COL_PANEL);
    HPEN   pen  = CreatePen(PS_SOLID, 1, COL_PANEL_EDGE);
    HGDIOBJ ob = SelectObject(dc, pen);
    HGDIOBJ of = SelectObject(dc, fill);
    RoundRect(dc, r.left, r.top, r.right, r.bottom, 8, 8);
    SelectObject(dc, ob);
    SelectObject(dc, of);
    DeleteObject(pen);
    DeleteObject(fill);
}

static void draw_text(HDC dc, int x, int y, const char *s,
                      COLORREF col, HFONT font)
{
    HGDIOBJ of = SelectObject(dc, font);
    SetTextColor(dc, col);
    SetBkMode(dc, TRANSPARENT);
    TextOutA(dc, x, y, s, (int)strlen(s));
    SelectObject(dc, of);
}

/* Draw text so its BASELINE sits at y_base. Lets different font sizes line
 * up along the bottom of the glyphs rather than the top. Returns the pixel
 * width drawn (so callers can place following text).                       */
static int draw_text_base(HDC dc, int x, int y_base, const char *s,
                          COLORREF col, HFONT font)
{
    HGDIOBJ of = SelectObject(dc, font);
    TEXTMETRICA tm;
    GetTextMetricsA(dc, &tm);
    SIZE sz;
    GetTextExtentPoint32A(dc, s, (int)strlen(s), &sz);
    SetTextColor(dc, col);
    SetBkMode(dc, TRANSPARENT);
    TextOutA(dc, x, y_base - tm.tmAscent, s, (int)strlen(s));
    SelectObject(dc, of);
    return sz.cx;
}

/* Simple round LED - just the lit centre, no glow ring or highlight. */
static void draw_led(HDC dc, int cx, int cy, int radius, COLORREF col, int glow)
{
    (void)glow;
    HBRUSH b = CreateSolidBrush(col);
    HPEN   p = CreatePen(PS_SOLID, 1, RGB(0, 0, 0));
    HGDIOBJ ob = SelectObject(dc, b);
    HGDIOBJ op = SelectObject(dc, p);
    Ellipse(dc, cx - radius, cy - radius, cx + radius, cy + radius);
    SelectObject(dc, ob);
    SelectObject(dc, op);
    DeleteObject(b);
    DeleteObject(p);
}

/* Horizontal segmented signal meter driven by dBFS (-60..0).
 * Returns nothing; draws inside the rect r.
 *
 * Colour zone thresholds - chosen deliberately rather than guessed:
 *   AMBER at -18 dBFS: this is the EBU R68 / SMPTE RP155 "0 VU" reference
 *     level used throughout broadcast and pro-audio metering as the
 *     boundary between "comfortably operating" and "getting loud, keep an
 *     eye on it" - a widely recognised, externally-defined reference point
 *     rather than a number picked to look right on this specific meter.
 *   RED at -6 dBFS: the standard "6 dB of headroom before clipping"
 *     warning threshold used the same way on broadcast peak meters - close
 *     enough to 0 dBFS (true digital full scale, real clipping) to mean
 *     something actionable, far enough from it to give real warning
 *     before a transient between updates could clip outright.
 * Both are independent of the SDRplay hardware's own ADC overload flag
 * (Section 3.3 of the User Guide) - that fires on the analog front end in
 * real time and can trip without ever pushing this meter into red, for
 * the reasons explained there. This meter answers "how loud is what's
 * arriving," not "is the front end currently overloaded."               */
#define METER_RANGE_DB   60.0f   /* meter spans -60 dBFS (0%) to 0 dBFS (100%) */
#define METER_AMBER_DB  -18.0f
#define METER_RED_DB     -6.0f

static void draw_meter(HDC dc, RECT r, float dbfs, int overload, int graduated)
{
    /* Track background */
    HBRUSH bg = CreateSolidBrush(COL_BAR_BG);
    HPEN   pen = CreatePen(PS_SOLID, 1, COL_PANEL_EDGE);
    HGDIOBJ ob = SelectObject(dc, bg);
    HGDIOBJ op = SelectObject(dc, pen);
    Rectangle(dc, r.left, r.top, r.right, r.bottom);
    SelectObject(dc, ob);
    SelectObject(dc, op);
    DeleteObject(bg);
    DeleteObject(pen);

    const int segs = 20;
    int active;
    if (dbfs <= -90.0f) {
        active = 0;                 /* silence */
    } else {
        float norm = (dbfs + METER_RANGE_DB) / METER_RANGE_DB;  /* -60 dB -> 0 .. 0 dB -> 1 */
        if (norm < 0.0f) norm = 0.0f;
        if (norm > 1.0f) norm = 1.0f;
        active = (int)(norm * segs + 0.5f);
    }
    if (active > segs) active = segs;

    /* Same -60..0 dBFS -> 0..1 normalisation as the level itself, applied
     * to the two threshold dB values above so every meter style reads
     * off the same underlying scale as what's actually lit.             */
    const float amber_frac = (METER_AMBER_DB + METER_RANGE_DB) / METER_RANGE_DB;
    const float red_frac   = (METER_RED_DB   + METER_RANGE_DB) / METER_RANGE_DB;

    int pad = 2;
    int innerW = (r.right - r.left) - pad * 2;
    int innerH = (r.bottom - r.top) - pad * 2;
    int gap = 2;
    /* The overload indicator is a half-width slot at the end, derived from
     * the same division as the 20 level segments rather than sized from
     * whatever pixels integer division left over. Solving
     *   innerW = segs*segW + segs*gap + segW/2
     * for segW keeps it locked to exactly half a level segment's width at
     * every window size, so it scales right along with them and is never
     * zero except at the same pathologically narrow widths that would
     * already squash the main segments too.                               */
    int segW = (2 * (innerW - gap * segs)) / (2 * segs + 1);
    if (segW < 2) segW = 2;
    int ovldW = segW / 2;
    int x = r.left + pad;
    int y = r.top + pad;

    for (int i = 0; i < segs; i++) {
        /* Segment colour: zone, graduated blend, or greyscale by style. */
        float t = (float)i / (float)(segs - 1);
        COLORREF c;
        if (graduated == 1) {
            if (t < amber_frac) {
                float s = t / amber_frac;
                c = RGB((int)(40  + (255-40 ) * s * 0.75f),
                        (int)(220 + (190-220) * s),
                        (int)(90  + ( 40- 90) * s));
            } else {
                float s = (t - amber_frac) / (1.0f - amber_frac);
                c = RGB(255,
                        (int)(190 + ( 60-190) * s),
                        (int)( 40 + ( 50- 40) * s));
            }
        } else if (graduated == 2) {
            /* Greyscale: dark grey at left, bright white-grey at right. */
            int v = (int)(60 + t * 180);
            c = RGB(v, v, v);
        } else {
            if (t >= red_frac)        c = COL_SEG_RED;
            else if (t >= amber_frac) c = COL_SEG_AMBER;
            else                       c = COL_SEG_GREEN;
        }

        if (i >= active) {
            /* dim unlit segment */
            c = RGB(GetRValue(c) / 5 + 10, GetGValue(c) / 5 + 10,
                    GetBValue(c) / 5 + 10);
        }
        HBRUSH sb = CreateSolidBrush(c);
        HGDIOBJ os = SelectObject(dc, sb);
        HGDIOBJ pp = SelectObject(dc, GetStockObject(NULL_PEN));
        Rectangle(dc, x, y, x + segW, y + innerH);
        SelectObject(dc, os);
        SelectObject(dc, pp);
        DeleteObject(sb);
        x += segW + gap;
    }

    /* Dedicated overload indicator: a small gap then a bright red segment
     * that only lights when the hardware reports ADC overload. It's half
     * the width of a level segment (ovldW, derived above), scaling with
     * the meter exactly like they do rather than being sized from
     * rounding leftovers.                                                 */
    {
        int ovld_x = x;   /* x already sits just after the last segment's
                              trailing gap, from the loop above           */
        COLORREF oc = overload ? COL_SEG_RED : RGB(60, 14, 12);
        HBRUSH ob2 = CreateSolidBrush(oc);
        HGDIOBJ os2 = SelectObject(dc, ob2);
        HGDIOBJ op2 = SelectObject(dc, GetStockObject(NULL_PEN));
        Rectangle(dc, ovld_x, y, ovld_x + ovldW, y + innerH);
        SelectObject(dc, os2);
        SelectObject(dc, op2);
        DeleteObject(ob2);
    }
}

/* Small "LED counter" tile: label above, big value, in a panel. */
static void draw_counter(HDC dc, int x, int y, int w, int h,
                         const char *label, const char *value, COLORREF valcol)
{
    RECT r = { x, y, x + w, y + h };
    draw_panel(dc, r);
    draw_text(dc, x + 10, y + 6, label, COL_TEXT_DIM, g_hFontUI);
    draw_text(dc, x + 10, y + 24, value, valcol, g_hFontBig);
}

/* Two-row variant for the SUN tile - "SUNRISE  HH:MM" on the top line,
 * "SUNSET  HH:MM" on the line below, rather than draw_counter's usual
 * single label + single value layout. SUNRISE and SUNSET share the same
 * (dim, small) label font so the two rows read consistently; the times
 * use the slightly larger/brighter value font, matching how every other
 * tile emphasises its value over its label.                              */
static void draw_suntile(HDC dc, int x, int y, int w, int h,
                          const char *sunrise, const char *sunset)
{
    RECT r = { x, y, x + w, y + h };
    draw_panel(dc, r);
    draw_text(dc, x + 10, y + 8,  "SUNRISE", COL_TEXT_DIM, g_hFontUI);
    draw_text(dc, x + 76, y + 6,  sunrise,   COL_SUNRISE, g_hFontVal);
    draw_text(dc, x + 10, y + 32, "SUNSET",  COL_TEXT_DIM, g_hFontUI);
    draw_text(dc, x + 76, y + 30, sunset,    COL_SUNSET, g_hFontVal);
}

/* =========================================================================
 * Live monitor - implementation
 *
 * Signal path (per incoming IQ sample, at the native recording rate):
 *   NCO mix (shift monitor freq_hz to 0 Hz)
 *     -> decimate stage 1 -> decimate stage 2   (cheap 2-stage anti-alias)
 *     -> IF notch (complex single pole/zero, +/-10 kHz around 0 Hz)
 *     -> selective filter (real low-pass for AM/FM; frequency-shifted
 *        "one-sided" complex low-pass for USB/LSB/CW - this is what gives
 *        real sideband rejection rather than just a narrower low-pass)
 *     -> demod (envelope / discriminator / real-part)
 *     -> linear-interpolation resample to 48 kHz
 *     -> soft AGC + limiter -> int16 -> waveOut
 *
 * Everything from "decimate stage 1" onward runs at a much lower rate than
 * the native IQ rate, so the whole chain is cheap even on a modest CPU.
 * ========================================================================= */

static inline MCplx mcplx_mul(MCplx a, MCplx b)
{
    MCplx r;
    r.re = a.re * b.re - a.im * b.im;
    r.im = a.re * b.im + a.im * b.re;
    return r;
}

/* Hamming-windowed sinc low-pass, unity DC gain. cutoff_hz/fs must be < 0.5. */
static void mon_design_lowpass(double cutoff_hz, double fs, float *coeffs, int ntaps)
{
    int    m = ntaps - 1;
    double fc = cutoff_hz / fs;
    double sum = 0.0;
    int    n;

    if (fc > 0.499) fc = 0.499;
    if (fc < 0.0005) fc = 0.0005;

    for (n = 0; n < ntaps; n++) {
        double x = (double)n - (double)m / 2.0;
        double sinc = (x == 0.0) ? 2.0 * fc : sin(2.0 * MON_PI * fc * x) / (MON_PI * x);
        double w = 0.54 - 0.46 * cos(2.0 * MON_PI * (double)n / (double)m);
        double c = sinc * w;
        coeffs[n] = (float)c;
        sum += c;
    }
    if (sum > 1e-9 || sum < -1e-9)
        for (n = 0; n < ntaps; n++)
            coeffs[n] = (float)(coeffs[n] / sum);
}

static void mon_decim_stage_design(MonDecimStage *s, double cutoff_hz, double fs_in, int decim)
{
    mon_design_lowpass(cutoff_hz, fs_in, s->coeffs, MON_DECIM_TAPS);
    memset(s->hist, 0, sizeof(s->hist));
    s->head = 0;
    s->decim = (decim < 1) ? 1 : decim;
    s->counter = 0;
}

/* Feed one input sample. Returns 1 and fills *out when a decimated output
 * sample is produced, 0 otherwise (this stage is still accumulating). */
static int mon_decim_stage_process(MonDecimStage *s, MCplx in, MCplx *out)
{
    int k;

    s->hist[s->head] = in;
    s->counter++;
    if (s->counter < s->decim) {
        s->head = (s->head + 1) % MON_DECIM_TAPS;
        return 0;
    }
    s->counter = 0;

    {
        MCplx acc = { 0.0f, 0.0f };
        int idx = s->head;
        for (k = 0; k < MON_DECIM_TAPS; k++) {
            acc.re += s->coeffs[k] * s->hist[idx].re;
            acc.im += s->coeffs[k] * s->hist[idx].im;
            idx = (idx == 0) ? MON_DECIM_TAPS - 1 : idx - 1;
        }
        *out = acc;
    }
    s->head = (s->head + 1) % MON_DECIM_TAPS;
    return 1;
}

/* Selective filter design.
 * AM/AM-N/FM-N/FM-W : symmetric low-pass, cutoff = bw/2 (no frequency shift)
 * USB               : low-pass prototype shifted up   by  bw/2 -> passes [0, bw]
 * LSB               : low-pass prototype shifted down by  bw/2 -> passes [-bw, 0]
 * CW                : narrow low-pass shifted to sit at the CW pitch (~700 Hz),
 *                     so taking the real part afterwards gives an audible tone.
 */
static void mon_sel_filter_design(MonSelFilter *f, MonMode mode, double bw_khz, double work_rate)
{
    double bw_hz = bw_khz * 1000.0;
    double proto_cutoff, shift_hz;
    float  proto[MON_SEL_TAPS];
    int    n;

    switch (mode) {
    case MON_MODE_USB: proto_cutoff = bw_hz / 2.0; shift_hz =  bw_hz / 2.0; break;
    case MON_MODE_LSB: proto_cutoff = bw_hz / 2.0; shift_hz = -bw_hz / 2.0; break;
    case MON_MODE_CW:  proto_cutoff = bw_hz / 2.0; shift_hz =  0.0; break;
    default:           proto_cutoff = bw_hz / 2.0; shift_hz =  0.0; break;
    }

    mon_design_lowpass(proto_cutoff, work_rate, proto, MON_SEL_TAPS);

    for (n = 0; n < MON_SEL_TAPS; n++) {
        double theta = 2.0 * MON_PI * shift_hz * (double)n / work_rate;
        f->coeffs_re[n] = (float)(proto[n] * cos(theta));
        f->coeffs_im[n] = (float)(proto[n] * sin(theta));
    }
    memset(f->hist, 0, sizeof(f->hist));
    f->head = 0;
}

static MCplx mon_sel_filter_process(MonSelFilter *f, MCplx in)
{
    MCplx acc = { 0.0f, 0.0f };
    int   idx = f->head;
    int   k;

    f->hist[f->head] = in;

    for (k = 0; k < MON_SEL_TAPS; k++) {
        float cr = f->coeffs_re[k], ci = f->coeffs_im[k];
        float xr = f->hist[idx].re, xi = f->hist[idx].im;
        acc.re += cr * xr - ci * xi;
        acc.im += cr * xi + ci * xr;
        idx = (idx == 0) ? MON_SEL_TAPS - 1 : idx - 1;
    }
    f->head = (f->head + 1) % MON_SEL_TAPS;
    return acc;
}

/* Single complex pole/zero notch. f0_hz is the offset from 0 Hz (i.e. from
 * the monitor's tuned frequency) - can be positive or negative, +/-10 kHz.
 * enabled is the explicit IF Notch button state - the slider only sets
 * where the notch would sit, not whether it's actually applied.        */
static void mon_notch_design(MonNotch *ns, double f0_hz, double fs, double r,
                              int enabled)
{
    double w = 2.0 * MON_PI * f0_hz / fs;
    ns->z_re = (float)cos(w);
    ns->z_im = (float)sin(w);
    ns->r    = (float)r;
    ns->x1.re = ns->x1.im = 0.0f;
    ns->y1.re = ns->y1.im = 0.0f;
    ns->active = enabled && (f0_hz > 50.0 || f0_hz < -50.0);  /* ~0 Hz = no-op anyway */
}

static MCplx mon_notch_process(MonNotch *ns, MCplx x)
{
    MCplx y, zx1, rzy1;
    float rz_re, rz_im;

    if (!ns->active) return x;

    zx1.re = ns->z_re * ns->x1.re - ns->z_im * ns->x1.im;
    zx1.im = ns->z_re * ns->x1.im + ns->z_im * ns->x1.re;

    rz_re = ns->r * ns->z_re;
    rz_im = ns->r * ns->z_im;
    rzy1.re = rz_re * ns->y1.re - rz_im * ns->y1.im;
    rzy1.im = rz_re * ns->y1.im + rz_im * ns->y1.re;

    y.re = x.re - zx1.re + rzy1.re;
    y.im = x.im - zx1.im + rzy1.im;

    ns->x1 = x;
    ns->y1 = y;
    return y;
}

/* -------------------------------------------------------------------------
 * monitor_feed - called from the SDRplay streaming callback (real-time
 * thread). Same contract as the callback itself: no malloc, no locks, no
 * blocking. A disabled monitor costs one interlocked read.
 * ------------------------------------------------------------------------- */
static void monitor_feed(const int16_t *xi, const int16_t *xq, unsigned int n)
{
    int16_t tmp[512];
    unsigned int off = 0;

    if (!g_monitor.enabled || !g_monitor.ring_ready)
        return;

    while (off < n) {
        unsigned int batch = n - off;
        unsigned int i;
        if (batch > 256) batch = 256;

        for (i = 0; i < batch; i++) {
            tmp[i * 2]     = xi[off + i];
            tmp[i * 2 + 1] = xq[off + i];
        }
        ring_write(&g_monitor.ring, tmp, batch * 4);
        off += batch;
    }
}

static void mon_clear_decim_hist(MonDecimStage *s)
{
    memset(s->hist, 0, sizeof(s->hist));
    s->head = 0;
    s->counter = 0;
}

static void mon_clear_sel_hist(MonSelFilter *f)
{
    memset(f->hist, 0, sizeof(f->hist));
    f->head = 0;
}

/* -------------------------------------------------------------------------
 * Rebuild the NCO / decimators / notch / selective filter whenever the
 * relevant settings (or the recording's native sample rate) change. Cheap
 * to call every block - it only does real work when something changed.
 * ------------------------------------------------------------------------- */
static void monitor_update_params(void)
{
    double freq, bw, notch_khz;
    LONG   mode_i, tuner_i;
    double native, center;
    int    need_filters, need_nco, need_notch, tuner_changed;

    EnterCriticalSection(&g_monitor.settings_lock);
    freq      = *monitor_active_freq_ptr();
    bw        = g_monitor.bw_khz;
    notch_khz = g_monitor.notch_khz;
    LeaveCriticalSection(&g_monitor.settings_lock);

    mode_i  = g_monitor.mode;
    tuner_i = g_monitor.tuner_sel;
    tuner_changed = (tuner_i != g_monitor.last_tuner);

    native = g_state.live_expected_output_rate_hz;  /* frozen for this session - see AppState */
    if (native < 1000.0) native = DEFAULT_SAMPLE_RATE_HZ;

    center = (tuner_i == 1 && g_state.cfg.freq_b_hz > 0.0)
                ? g_state.cfg.freq_b_hz : g_state.cfg.frequency_hz;
    if (center <= 0.0) center = freq;   /* idle (not yet recording) - assume on-freq */

    need_filters = (mode_i != g_monitor.last_mode) ||
                   (fabs(bw - g_monitor.last_bw) > 1e-6) ||
                   (fabs(native - g_monitor.last_native) > 1.0);
    need_nco = need_filters || tuner_changed ||
               (fabs(freq - g_monitor.last_freq) > 1e-6) ||
               (fabs(center - g_monitor.last_center) > 1e-6);
    need_notch = need_filters || (fabs(notch_khz - g_monitor.last_notch) > 1e-6) ||
                 ((int)g_monitor.notch_enabled != g_monitor.last_notch_enabled);

    {
        /* Decimation stages (st1/st2) depend only on mode (via work_rate)
         * and the native input rate - never on bw. Redesigning them (and
         * clearing their history) on every bandwidth tweak was needless:
         * it's the dominant cost of the mute you hear when adjusting BW,
         * since they run at the (much higher) native/intermediate rate,
         * so their history takes far longer to usefully refill than the
         * selectivity filter's does. Splitting the two triggers means a
         * bandwidth-only change now only touches the one filter that
         * actually depends on bandwidth.                                */
        int need_decim = (mode_i != g_monitor.last_mode) ||
                          (fabs(native - g_monitor.last_native) > 1.0);
        int need_sel   = need_decim || (fabs(bw - g_monitor.last_bw) > 1e-6);

        need_filters = need_decim || need_sel;   /* used below for NCO/reset-state */

        if (need_decim) {
            double work_rate = ((MonMode)mode_i == MON_MODE_FMW)
                                    ? MON_WORK_RATE_WIDE : MON_WORK_RATE_NARROW;
            int dtotal = (int)llround(native / work_rate);
            int d1, d2;
            double actual_work_rate;

            if (dtotal < 1) dtotal = 1;
            d1 = (int)floor(sqrt((double)dtotal));
            if (d1 < 1) d1 = 1;
            d2 = dtotal / d1;
            if (d2 < 1) d2 = 1;
            actual_work_rate = native / (double)(d1 * d2);

            mon_decim_stage_design(&g_monitor.st1, 0.42 * (native / d1) / 2.0, native, d1);
            mon_decim_stage_design(&g_monitor.st2, 0.42 * actual_work_rate / 2.0,
                                    native / d1, d2);
            g_monitor.work_rate_hz = actual_work_rate;
            g_monitor.last_mode    = (int)mode_i;
        }
        if (need_sel) {
            mon_sel_filter_design(&g_monitor.sel, (MonMode)mode_i, bw,
                                   g_monitor.work_rate_hz);
            g_monitor.last_bw = bw;
        }
    }
    if (!need_filters && tuner_changed) {
        /* Coefficients are unchanged, but a different tuner's RF content is
         * now flowing in - stale filter/demod history from the previous
         * tuner would otherwise bleed in as noise/glitching for a while. */
        mon_clear_decim_hist(&g_monitor.st1);
        mon_clear_decim_hist(&g_monitor.st2);
        mon_clear_sel_hist(&g_monitor.sel);
        g_monitor.notch.x1.re = g_monitor.notch.x1.im = 0.0f;
        g_monitor.notch.y1.re = g_monitor.notch.y1.im = 0.0f;
    }

    if (need_filters || tuner_changed) {
        g_monitor.resamp_acc    = 0.0;
        g_monitor.resamp_prev   = 0.0f;
        g_monitor.resamp_cur    = 0.0f;
        g_monitor.fm_prev.re    = 1.0f;
        g_monitor.fm_prev.im    = 0.0f;
        g_monitor.dc_prev_in    = 0.0f;
        g_monitor.dc_prev_out   = 0.0f;
        g_monitor.dc_primed     = 0;
        g_monitor.hpf_prev_in   = 0.0f;
        g_monitor.hpf_prev_out  = 0.0f;
        g_monitor.hpf_primed    = 0;
        g_monitor.hpf_last_hz   = 0.0;   /* forces the coefficient to be
                                          * recomputed for the new work rate */
        g_monitor.deemph_state  = 0.0f;
        g_monitor.agc_gain       = 3.0f;
        g_monitor.agc_envelope   = 0.0f;
        g_monitor.carrier_prev_valid       = 0;
        g_monitor.carrier_lpf_state.re     = 0.0f;
        g_monitor.carrier_lpf_state.im     = 0.0f;
        g_monitor.carrier_dbm_baseline_valid = 0;
        g_monitor.carrier_phase_accum      = 0.0;
        g_monitor.carrier_weight_accum     = 0.0;
        g_monitor.carrier_accum_samples    = 0;
        g_monitor.carrier_last_raw_valid   = 0;
        g_monitor.carrier_consec_agree     = 0;
        if (tuner_changed && !g_monitor.freq_locked) {
            /* Genuinely a different tuner/frequency - no continuity makes
             * sense, clear the display too, same as before. Skipped when
             * A=B is locked: both tuners share the same frequency then,
             * so switching which one is active/displayed doesn't change
             * what's actually being measured any more than a bandwidth
             * change does - same reasoning as the mode/bandwidth case
             * below applies here too.                                    */
            g_monitor.carrier_offset_valid_pub = 0;
            g_monitor.carrier_window_count     = 0;
            g_monitor.carrier_locked_pub       = 0;
            g_monitor.carrier_last_published_valid = 0;
        }
        /* else: a pure mode/bandwidth change on the same tuner (or a
         * tuner switch while A=B is locked) - the accumulator internals
         * above still need a clean restart (the DSP chain, and this
         * filter's own state, just got rebuilt), but the carrier
         * frequency itself hasn't moved, so there's no reason to blank
         * the display and force a full re-lock from scratch. Leaving
         * offset_valid_pub/window_count/locked_pub alone keeps showing
         * the last known reading uninterrupted while a fresh
         * accumulation quietly re-verifies it in the background -
         * window_count staying wherever it already was (typically at the
         * smoothing floor by then) means the next few genuine
         * measurements blend in gradually rather than snapping to
         * whatever the very first new window happens to read.           */
        /* Settle status ("(lock)") resets either way - a fresh
         * accumulation is starting regardless of which branch above ran,
         * so it's honest to say "still confirming" until it actually has,
         * rather than keep showing "(lock)" from before the change.     */
        g_monitor.carrier_settled_count = 0;
        g_monitor.carrier_settled_pub   = 0;
        g_monitor.carrier_agc_guard_remaining = CARRIER_AGC_GUARD_SAMPLES;
        g_monitor.carrier_window_target = CARRIER_WINDOW_SAMPLES_MIN;
        g_monitor.carrier_agc_gain_at_start = 0.0f;
        /* Mute just long enough to cover whichever filters actually got
         * reset above, rather than always assuming the worst case. A
         * bandwidth-only change only reset the ~127-tap selectivity
         * filter, which settles in a few ms at the monitor's work rate;
         * a mode or sample-rate change also reset both decimation
         * stages, which needs the fuller, original margin to settle
         * cleanly at the (much higher) native rate.                    */
        if (mode_i != g_monitor.last_mode_for_mute ||
                fabs(native - g_monitor.last_native) > 1.0) {
            g_monitor.mute_samples_left = MON_AUDIO_RATE_HZ / 8;  /* ~125 ms */
        } else {
            double settle_sec = (MON_SEL_TAPS * 3.0) / g_monitor.work_rate_hz;
            int    settle_samples = (int)(MON_AUDIO_RATE_HZ * settle_sec);
            g_monitor.mute_samples_left = settle_samples < 1 ? 1 : settle_samples;
        }
        g_monitor.last_mode_for_mute = (int)mode_i;

        /* CW audio-shift NCO: turns a filtered, on-frequency carrier
         * (which sits at 0Hz after the tuning NCO + selective filter)
         * into an audible tone at MON_CW_PITCH_HZ.                    */
        {
            double w = 2.0 * MON_PI * MON_CW_PITCH_HZ / g_monitor.work_rate_hz;
            g_monitor.cw_nco_step.re = (float)cos(w);
            g_monitor.cw_nco_step.im = (float)sin(w);
            g_monitor.cw_nco_rot.re  = 1.0f;
            g_monitor.cw_nco_rot.im  = 0.0f;
        }
    }
    g_monitor.last_tuner = (int)tuner_i;

    if (need_notch) {
        mon_notch_design(&g_monitor.notch, notch_khz * 1000.0,
                          g_monitor.work_rate_hz, 0.97,
                          (int)g_monitor.notch_enabled);
        g_monitor.last_notch = notch_khz;
        g_monitor.last_notch_enabled = (int)g_monitor.notch_enabled;
    }

    if (need_nco) {
        double delta = freq - center;
        double w = 2.0 * MON_PI * delta / native;
        g_monitor.nco_step.re = (float)cos(-w);
        g_monitor.nco_step.im = (float)sin(-w);
        g_monitor.nco_rot.re  = 1.0f;
        g_monitor.nco_rot.im  = 0.0f;
        g_monitor.last_freq   = freq;
        g_monitor.last_center = center;
    }

    g_monitor.last_native = native;
}

/* -------------------------------------------------------------------------
 * Audio output - waveOut, double/triple buffered, filled sample-by-sample
 * by the resampler below.
 * ------------------------------------------------------------------------- */
static void monitor_audio_open(void)
{
    WAVEFORMATEX wfx;
    int i;

    if (g_monitor.audio_open) return;

    memset(&wfx, 0, sizeof(wfx));
    wfx.wFormatTag      = WAVE_FORMAT_PCM;
    wfx.nChannels        = 1;
    wfx.nSamplesPerSec   = MON_AUDIO_RATE_HZ;
    wfx.wBitsPerSample   = 16;
    wfx.nBlockAlign      = (WORD)(wfx.nChannels * wfx.wBitsPerSample / 8);
    wfx.nAvgBytesPerSec  = wfx.nSamplesPerSec * wfx.nBlockAlign;

    if (!g_monitor.audio_done_event)
        g_monitor.audio_done_event = CreateEventA(NULL, FALSE, FALSE, NULL);

    if (waveOutOpen(&g_monitor.hwo, WAVE_MAPPER, &wfx,
                     (DWORD_PTR)g_monitor.audio_done_event, 0,
                     CALLBACK_EVENT) != MMSYSERR_NOERROR) {
        LOG_WARN("Monitor: could not open an audio output device.");
        g_monitor.hwo = NULL;
        return;
    }

    for (i = 0; i < MON_AUDIO_NUM_BUFS; i++) {
        memset(&g_monitor.hdr[i], 0, sizeof(WAVEHDR));
        g_monitor.hdr[i].lpData         = (LPSTR)g_monitor.audio_buf[i];
        g_monitor.hdr[i].dwBufferLength = MON_AUDIO_BUF_SAMPLES * sizeof(int16_t);
        waveOutPrepareHeader(g_monitor.hwo, &g_monitor.hdr[i], sizeof(WAVEHDR));
    }
    g_monitor.cur_buf = 0;
    g_monitor.cur_buf_fill = 0;
    g_monitor.agc_gain = 3.0f;
    g_monitor.agc_envelope = 0.0f;
    g_monitor.audio_open = 1;

    {
        WORD lvl = (WORD)((g_monitorVolPercent * 0xFFFF) / 100);
        waveOutSetVolume(g_monitor.hwo, MAKELONG(lvl, lvl));
    }
}

static void monitor_audio_close(void)
{
    int i;
    if (!g_monitor.audio_open) return;

    waveOutReset(g_monitor.hwo);
    for (i = 0; i < MON_AUDIO_NUM_BUFS; i++)
        waveOutUnprepareHeader(g_monitor.hwo, &g_monitor.hdr[i], sizeof(WAVEHDR));
    waveOutClose(g_monitor.hwo);
    g_monitor.hwo = NULL;
    g_monitor.audio_open = 0;
}

/* Push one finished 48 kHz sample into the current waveOut buffer,
 * submitting and rotating to the next buffer once full. If every buffer is
 * still in flight (playback can't keep up - shouldn't normally happen),
 * the sample is dropped rather than blocking the DSP loop. */
static void monitor_audio_push(int16_t sample)
{
    WAVEHDR *hdr = &g_monitor.hdr[g_monitor.cur_buf];

    if (!g_monitor.audio_open) return;

    if (hdr->dwFlags & WHDR_INQUEUE)
        return;  /* buffer still playing - drop this sample (rare) */

    g_monitor.audio_buf[g_monitor.cur_buf][g_monitor.cur_buf_fill++] = sample;

    if (g_monitor.cur_buf_fill >= MON_AUDIO_BUF_SAMPLES) {
        hdr->dwBufferLength = MON_AUDIO_BUF_SAMPLES * sizeof(int16_t);
        waveOutWrite(g_monitor.hwo, hdr, sizeof(WAVEHDR));
        g_monitor.cur_buf = (g_monitor.cur_buf + 1) % MON_AUDIO_NUM_BUFS;
        g_monitor.cur_buf_fill = 0;
    }
}

/* Linear-interpolation resample (working rate -> 48 kHz) + soft AGC/limiter,
 * then hand off to waveOut. Called once per demodulated audio sample. */
static void monitor_audio_process(float audio)
{
    double out_per_in = (double)MON_AUDIO_RATE_HZ / g_monitor.work_rate_hz;

    g_monitor.resamp_prev = g_monitor.resamp_cur;
    g_monitor.resamp_cur  = audio;
    g_monitor.resamp_acc += out_per_in;

    while (g_monitor.resamp_acc >= 1.0) {
        float frac = (float)(g_monitor.resamp_acc - 1.0);
        float out_sample = g_monitor.resamp_prev +
                            (g_monitor.resamp_cur - g_monitor.resamp_prev) * (1.0f - frac);
        float gain, leveled;
        int16_t s16;

        if (g_monitor.mute_samples_left > 0) {
            g_monitor.mute_samples_left--;
            monitor_audio_push(0);
            g_monitor.resamp_acc -= 1.0;
            continue;
        }

        {
            /* Feedback-loop AGC: fast attack (gain pulled down quickly
             * when something gets loud, protecting against clipping),
             * slow release (gain eased back up gradually when it's
             * quiet, avoiding the "pumping" that's fatiguing for
             * weak-signal DX listening). Target is now fixed - see
             * MON_AGC_TARGET for why the old adjustable version was
             * removed in favour of the Volume slider.                   */
            float target = MON_AGC_TARGET;
            float mag, desired_gain;
            if (target > 32000.0f) target = 32000.0f;

            mag = fabsf(out_sample);
            if (mag > g_monitor.agc_envelope)
                g_monitor.agc_envelope += (mag - g_monitor.agc_envelope) * MON_AGC_ENV_ATTACK;
            else
                g_monitor.agc_envelope += (mag - g_monitor.agc_envelope) * MON_AGC_ENV_DECAY;

            desired_gain = target / (g_monitor.agc_envelope > 1.0f ? g_monitor.agc_envelope : 1.0f);
            if (desired_gain > 2000.0f) desired_gain = 2000.0f;
            if (desired_gain < 0.02f)   desired_gain = 0.02f;

            if (desired_gain < g_monitor.agc_gain)
                g_monitor.agc_gain += (desired_gain - g_monitor.agc_gain) * 0.003f;   /* fast attack  */
            else
                g_monitor.agc_gain += (desired_gain - g_monitor.agc_gain) * 0.00004f; /* slow release */

            if (g_monitor.agc_gain > 2000.0f) g_monitor.agc_gain = 2000.0f;
            if (g_monitor.agc_gain < 0.02f)   g_monitor.agc_gain = 0.02f;

            gain = g_monitor.agc_gain;
        }

        leveled = out_sample * gain;
        /* Soft-knee limiter: rounds off any residual overshoot smoothly
         * instead of a harsh brick-wall clamp - much less "raspy" while
         * the AGC is still settling after a sudden loud transient.     */
        {
            float x = leveled / 32000.0f;
            if (x >  3.0f) x =  3.0f;
            if (x < -3.0f) x = -3.0f;
            leveled = tanhf(x) * 32000.0f;
        }
        s16 = (int16_t)leveled;

        monitor_audio_push(s16);
        g_monitor.resamp_acc -= 1.0;
    }
}

/* -------------------------------------------------------------------------
 * monitor_thread_func - runs for the lifetime of the app. Idles (cheap
 * Sleep) whenever the monitor is disabled or there's no data to process.
 * ------------------------------------------------------------------------- */
static DWORD WINAPI monitor_thread_func(LPVOID param)
{
    int16_t rdbuf[4096];  /* 1024 IQ frames per pass */
    int     was_enabled = 0;

    (void)param;

    while (!g_monitor.thread_stop_req) {
        int now_enabled = (g_monitor.enabled != 0);

        if (now_enabled != was_enabled) {
            if (now_enabled) monitor_audio_open();
            else             monitor_audio_close();
            was_enabled = now_enabled;
        }

        if (!now_enabled) {
            Sleep(20);
            continue;
        }

        {
            SIZE_T avail = ring_available(&g_monitor.ring);
            SIZE_T want, got;
            unsigned int nframes, i;

            if (avail < 4) { Sleep(2); continue; }

            want = avail;
            if (want > sizeof(rdbuf)) want = sizeof(rdbuf);
            want &= ~(SIZE_T)3;   /* align to 4-byte IQ frames */

            got = ring_read(&g_monitor.ring, rdbuf, want);
            nframes = (unsigned int)(got / 4);
            if (nframes == 0) continue;

            monitor_update_params();

            for (i = 0; i < nframes; i++) {
                MCplx raw, mixed, d1, d2, notched, sel;
                float mag;

                raw.re = (float)rdbuf[i * 2];
                raw.im = (float)rdbuf[i * 2 + 1];

                mixed = mcplx_mul(raw, g_monitor.nco_rot);
                g_monitor.nco_rot = mcplx_mul(g_monitor.nco_rot, g_monitor.nco_step);

                /* Renormalise the NCO rotator periodically - repeated complex
                 * multiplies slowly drift off the unit circle otherwise. */
                mag = sqrtf(g_monitor.nco_rot.re * g_monitor.nco_rot.re +
                            g_monitor.nco_rot.im * g_monitor.nco_rot.im);
                if (mag > 1e-6f) {
                    g_monitor.nco_rot.re /= mag;
                    g_monitor.nco_rot.im /= mag;
                }

                if (!mon_decim_stage_process(&g_monitor.st1, mixed, &d1))
                    continue;
                if (!mon_decim_stage_process(&g_monitor.st2, d1, &d2))
                    continue;

                notched = mon_notch_process(&g_monitor.notch, d2);
                sel     = mon_sel_filter_process(&g_monitor.sel, notched);

                /* Narrowband S-meter - measured here specifically because
                 * `sel` is the tuned station's own signal, already NCO-
                 * shifted, decimated, notched, and selectivity-filtered -
                 * not the wideband RF level the main A/B meters show.
                 * Both accumulators stay current regardless of which mode
                 * is selected, so switching the setting never needs a
                 * reset - it just starts reading from the other one.      */
                {
                    float mag2 = sel.re * sel.re + sel.im * sel.im;
                    float mag = sqrtf(mag2);
                    DWORD now_tick;

                    if (mag > g_monitor.smeter_peak_accum)
                        g_monitor.smeter_peak_accum = mag;
                    if (!g_monitor.smeter_avg_primed) {
                        g_monitor.smeter_avg_pow = mag2;
                        g_monitor.smeter_avg_primed = 1;
                    } else {
                        /* Exponential moving average - short enough to
                         * track a real fade, long enough to actually
                         * average out fluctuation rather than just
                         * tracking peak with extra steps.                 */
                        g_monitor.smeter_avg_pow += 0.001f *
                            (mag2 - g_monitor.smeter_avg_pow);
                    }

                    now_tick = GetTickCount();
                    if (now_tick - g_monitor.smeter_publish_tick >= 150) {
                        float use_mag = g_state.cfg.monitor_smeter_mode
                                       ? sqrtf(g_monitor.smeter_avg_pow)
                                       : g_monitor.smeter_peak_accum;
                        float dbfs = (use_mag > 1.0f)
                                   ? 20.0f * log10f(use_mag / 32767.0f)
                                   : -120.0f;
                        /* Gain-compensated approximate dBm - the same
                         * approach SDRplay's own Spectrum Analyzer tool
                         * uses (dBFS minus the actual applied gain, since
                         * the same RF signal reads completely differently
                         * at different GR/LNA settings, which raw dBFS
                         * alone can't account for). currGain comes from
                         * whichever tuner the monitor is actually reading
                         * from. The calibration offset defaults to 0 -
                         * the exact ADC full-scale-to-dBm reference isn't
                         * published, so this needs setting against a
                         * known signal source to be properly accurate;
                         * until then, treat it as approximate.            */
                        double curr_gain = (g_monitor.tuner_sel == 1)
                                          ? g_curr_gain_b : g_curr_gain_a;
                        float dbm = dbfs - (float)curr_gain
                                  + (float)g_state.cfg.monitor_smeter_cal_offset;
                        g_monitor.smeter_publish_tick = now_tick;
                        g_monitor.smeter_dbm_pub = dbm;
                        g_monitor.smeter_peak_accum = 0.0f;
                    }
                }

                /* Carrier frequency offset - see the field comments in
                 * AppMonitor for the method. AM only: SSB/CW have no
                 * steady carrier to track, and FM's "carrier" is the
                 * deliberately-varying thing being demodulated.          */
                {
                MonMode mode = (MonMode)g_monitor.mode;
                if (mode == MON_MODE_AM6 || mode == MON_MODE_AM4 ||
                        mode == MON_MODE_AM24) {
                    /* Narrowband tracking filter - runs unconditionally,
                     * even during the AGC guard, so its own (much
                     * shorter, ~40ms) settling transient is long over by
                     * the time anything downstream starts trusting data.
                     * See CARRIER_LPF_CUTOFF_HZ and the field comment on
                     * carrier_lpf_state for why this exists.             */
                    {
                        float lpf_alpha = (float)(2.0 * MON_PI * CARRIER_LPF_CUTOFF_HZ /
                                                   g_monitor.work_rate_hz);
                        g_monitor.carrier_lpf_state.re +=
                            lpf_alpha * (sel.re - g_monitor.carrier_lpf_state.re);
                        g_monitor.carrier_lpf_state.im +=
                            lpf_alpha * (sel.im - g_monitor.carrier_lpf_state.im);
                    }
                    if (g_monitor.carrier_agc_guard_remaining > 0) {
                        g_monitor.carrier_agc_guard_remaining--;
                    } else if (g_monitor.carrier_prev_valid) {
                        if (g_monitor.carrier_accum_samples == 0) {
                            g_monitor.carrier_window_target = carrier_window_samples_for_signal();
                            g_monitor.carrier_agc_gain_at_start = g_monitor.agc_gain;
                        }
                        float pr = g_monitor.carrier_lpf_state.re * g_monitor.carrier_prev_sample.re +
                                   g_monitor.carrier_lpf_state.im * g_monitor.carrier_prev_sample.im;
                        float pi = g_monitor.carrier_lpf_state.im * g_monitor.carrier_prev_sample.re -
                                   g_monitor.carrier_lpf_state.re * g_monitor.carrier_prev_sample.im;
                        float weight = sqrtf(pr * pr + pi * pi);
                        float dphase = atan2f(pi, pr);
                        g_monitor.carrier_phase_accum  += (double)dphase * (double)weight;
                        g_monitor.carrier_weight_accum += (double)weight;
                        g_monitor.carrier_accum_samples++;

                        if (g_monitor.carrier_accum_samples >= g_monitor.carrier_window_target) {
                            if (g_monitor.carrier_weight_accum > 0.0) {
                                double mean_dphase = g_monitor.carrier_phase_accum /
                                                      g_monitor.carrier_weight_accum;
                                double off_hz = mean_dphase * g_monitor.work_rate_hz /
                                                (2.0 * MON_PI);

                                float agc_rel_change =
                                    (g_monitor.carrier_agc_gain_at_start > 0.0f)
                                    ? fabsf(g_monitor.agc_gain - g_monitor.carrier_agc_gain_at_start)
                                      / g_monitor.carrier_agc_gain_at_start
                                    : 1.0f;
                                int agc_stable = agc_rel_change <= (float)CARRIER_AGC_STABILITY_FRAC;

                                /* Fade check - see the field/constant
                                 * comments. Independent of agc_stable:
                                 * a sustained fade can produce windows
                                 * that agree closely with each other
                                 * while both are wrong, which the
                                 * agreement check alone can't catch.    */
                                int not_fading = !g_monitor.carrier_dbm_baseline_valid ||
                                    (g_monitor.smeter_dbm_pub >=
                                     g_monitor.carrier_dbm_baseline - (float)CARRIER_FADE_DROP_DB);
                                int accepted = agc_stable && not_fading;

                                if (accepted) {
                                    /* Lock check - see the field/constant
                                     * comments. Runs off the raw per-window
                                     * value, not the smoothed one, since the
                                     * whole point is to catch disagreement
                                     * before smoothing has a chance to hide it. */
                                    if (g_monitor.carrier_last_raw_valid &&
                                            fabs(off_hz - g_monitor.carrier_last_raw_hz) <=
                                                CARRIER_LOCK_TOLERANCE_HZ) {
                                        g_monitor.carrier_consec_agree++;
                                    } else {
                                        g_monitor.carrier_consec_agree = 0;
                                    }
                                    g_monitor.carrier_locked_pub =
                                        (g_monitor.carrier_consec_agree >= CARRIER_LOCK_MIN_AGREE);
                                    g_monitor.carrier_last_raw_hz   = off_hz;
                                    g_monitor.carrier_last_raw_valid = 1;

                                    /* Baseline dbm - slow EMA, updated only
                                     * from windows that already passed
                                     * every other check, so a fade can't
                                     * drag its own detection threshold
                                     * down with it.                      */
                                    if (!g_monitor.carrier_dbm_baseline_valid) {
                                        g_monitor.carrier_dbm_baseline = g_monitor.smeter_dbm_pub;
                                        g_monitor.carrier_dbm_baseline_valid = 1;
                                    } else {
                                        g_monitor.carrier_dbm_baseline +=
                                            0.1f * (g_monitor.smeter_dbm_pub - g_monitor.carrier_dbm_baseline);
                                    }
                                }

                                if (!accepted) {
                                    /* Discard entirely - a window ridden
                                     * through active AGC hunting, or a
                                     * genuine fade, doesn't get to touch
                                     * the lock state (already skipped
                                     * above) or the smoothed estimate
                                     * either. Confirmed with real data:
                                     * both a fluctuating-AGC signal and a
                                     * genuine ~20 dB fade can otherwise
                                     * drag the smoothed estimate onto a
                                     * wrong value for a long time.       */
                                } else if (!g_monitor.carrier_offset_valid_pub) {
                                    g_monitor.carrier_offset_hz_pub = (float)off_hz;
                                    g_monitor.carrier_offset_valid_pub = 1;
                                    g_monitor.carrier_window_count = 1;
                                } else {
                                    /* Fast-then-settling EMA: alpha starts
                                     * at 1 (first window, handled above),
                                     * halves the gap on the 2nd, a third
                                     * on the 3rd, and so on - a running
                                     * average - continuing to shrink well
                                     * past the initial few windows, down
                                     * to a 0.1 floor. Windows contaminated
                                     * by active AGC hunting or a fade are
                                     * already filtered out above, so what
                                     * reaches here should be genuinely
                                     * trustworthy measurement noise around
                                     * a value that - for a real broadcast
                                     * carrier - is essentially constant,
                                     * worth continuing to average down
                                     * rather than stopping at an arbitrary
                                     * point. Not as low as the very first
                                     * attempt at this (0.02): that assumed
                                     * every accepted window was equally
                                     * trustworthy, which turned out not to
                                     * be true until these filters existed -
                                     * keeping a bit more responsiveness
                                     * here is a reasonable safety margin
                                     * against whatever still slips through
                                     * undetected.                        */
                                    float alpha = 1.0f / (float)(g_monitor.carrier_window_count + 1);
                                    if (alpha < 0.1f) alpha = 0.1f;
                                    g_monitor.carrier_offset_hz_pub +=
                                        alpha * ((float)off_hz - g_monitor.carrier_offset_hz_pub);
                                    g_monitor.carrier_window_count++;
                                }

                                /* Settle indicator ("(lock)") - see the
                                 * field/constant comments. Checks how
                                 * much the PUBLISHED value itself just
                                 * moved, not the raw measurement -
                                 * carrier_locked_pub already covers "is
                                 * this trustworthy enough to show",  this
                                 * covers "has it actually stopped
                                 * changing yet". Gated on accepted: a
                                 * discarded window leaves offset_hz_pub
                                 * completely untouched, which would
                                 * otherwise look like "zero change" and
                                 * count toward settling for entirely the
                                 * wrong reason - nothing was actually
                                 * confirmed that window.                  */
                                if (accepted) {
                                    if (g_monitor.carrier_last_published_valid &&
                                            fabsf(g_monitor.carrier_offset_hz_pub -
                                                  g_monitor.carrier_last_published_hz) <=
                                                (float)CARRIER_SETTLE_TOLERANCE_HZ) {
                                        g_monitor.carrier_settled_count++;
                                    } else {
                                        g_monitor.carrier_settled_count = 0;
                                    }
                                    g_monitor.carrier_settled_pub =
                                        (g_monitor.carrier_settled_count >= CARRIER_SETTLE_MIN_COUNT);
                                    g_monitor.carrier_last_published_hz   = g_monitor.carrier_offset_hz_pub;
                                    g_monitor.carrier_last_published_valid = 1;
                                }
                            }
                            g_monitor.carrier_phase_accum  = 0.0;
                            g_monitor.carrier_weight_accum = 0.0;
                            g_monitor.carrier_accum_samples = 0;
                        }
                    }
                    g_monitor.carrier_prev_sample = g_monitor.carrier_lpf_state;
                    g_monitor.carrier_prev_valid  = 1;
                } else {
                    g_monitor.carrier_prev_valid       = 0;
                    g_monitor.carrier_lpf_state.re     = 0.0f;
                    g_monitor.carrier_lpf_state.im     = 0.0f;
                    g_monitor.carrier_dbm_baseline_valid = 0;
                    g_monitor.carrier_offset_valid_pub = 0;
                    g_monitor.carrier_window_count     = 0;
                    g_monitor.carrier_last_raw_valid   = 0;
                    g_monitor.carrier_consec_agree     = 0;
                    g_monitor.carrier_locked_pub       = 0;
                    g_monitor.carrier_last_published_valid = 0;
                    g_monitor.carrier_settled_count        = 0;
                    g_monitor.carrier_settled_pub          = 0;
                    g_monitor.carrier_agc_guard_remaining = CARRIER_AGC_GUARD_SAMPLES;
                    g_monitor.carrier_window_target = CARRIER_WINDOW_SAMPLES_MIN;
                    g_monitor.carrier_agc_gain_at_start = 0.0f;
                }
                }

                {
                    float audio;
                    MonMode mode = (MonMode)g_monitor.mode;

                    switch (mode) {
                    case MON_MODE_AM6:
                    case MON_MODE_AM4:
                    case MON_MODE_AM24: {
                        float env = sqrtf(sel.re * sel.re + sel.im * sel.im);
                        float y;
                        if (!g_monitor.dc_primed) {
                            g_monitor.dc_prev_in = env;   /* avoid a huge first-sample step */
                            g_monitor.dc_primed = 1;
                        }
                        /* one-pole DC blocker: y = x - x1 + a*y1 */
                        y = env - g_monitor.dc_prev_in + 0.995f * g_monitor.dc_prev_out;
                        g_monitor.dc_prev_in  = env;
                        g_monitor.dc_prev_out = y;
                        audio = y;
                        break;
                    }
                    case MON_MODE_FMN:
                    case MON_MODE_FMW: {
                        float cross = g_monitor.fm_prev.re * sel.im - g_monitor.fm_prev.im * sel.re;
                        float dot   = g_monitor.fm_prev.re * sel.re + g_monitor.fm_prev.im * sel.im;
                        float ang   = atan2f(cross, dot);
                        float dev_hz = ang * (float)(g_monitor.work_rate_hz / (2.0 * MON_PI));
                        float maxdev = (mode == MON_MODE_FMW) ? (float)MON_FM_W_MAXDEV_HZ
                                                               : (float)MON_FM_N_MAXDEV_HZ;
                        audio = (dev_hz / maxdev) * 16000.0f;
                        if (mode == MON_MODE_FMW) {
                            /* light de-emphasis low-pass, ~50 us equivalent */
                            g_monitor.deemph_state += 0.25f * (audio - g_monitor.deemph_state);
                            audio = g_monitor.deemph_state;
                        }
                        g_monitor.fm_prev = sel;
                        break;
                    }
                    case MON_MODE_CW: {
                        /* sel's carrier sits at 0Hz (filtered right where a
                         * directly-tuned signal actually is) - shift it up
                         * to an audible pitch before taking the real part. */
                        MCplx shifted = mcplx_mul(sel, g_monitor.cw_nco_rot);
                        g_monitor.cw_nco_rot = mcplx_mul(g_monitor.cw_nco_rot,
                                                          g_monitor.cw_nco_step);
                        {
                            float mag = sqrtf(g_monitor.cw_nco_rot.re * g_monitor.cw_nco_rot.re +
                                               g_monitor.cw_nco_rot.im * g_monitor.cw_nco_rot.im);
                            if (mag > 1e-6f) {
                                g_monitor.cw_nco_rot.re /= mag;
                                g_monitor.cw_nco_rot.im /= mag;
                            }
                        }
                        audio = shifted.re;
                        break;
                    }
                    default:  /* USB, LSB - already selected one-sided */
                        audio = sel.re;
                        break;
                    }

                    /* Optional low-cut filter, applied after demod so it
                     * covers every mode uniformly - separate from AM's
                     * DC blocker above, which stays on regardless since
                     * it's structural to envelope detection, not a tone
                     * control. One-pole high-pass, same difference
                     * equation as the DC blocker but with a coefficient
                     * computed from the user's chosen cutoff instead of
                     * a fixed constant.                                 */
                    if (g_state.cfg.monitor_hpf_enable) {
                        float y;
                        if (g_monitor.hpf_last_hz != g_state.cfg.monitor_hpf_hz) {
                            double fc = g_state.cfg.monitor_hpf_hz;
                            if (fc < 1.0) fc = 1.0;
                            g_monitor.hpf_coeff = (float)exp(-2.0 * MON_PI * fc /
                                                              g_monitor.work_rate_hz);
                            g_monitor.hpf_last_hz = g_state.cfg.monitor_hpf_hz;
                        }
                        if (!g_monitor.hpf_primed) {
                            g_monitor.hpf_prev_in = audio;
                            g_monitor.hpf_primed = 1;
                        }
                        y = g_monitor.hpf_coeff * (g_monitor.hpf_prev_out + audio -
                                                    g_monitor.hpf_prev_in);
                        g_monitor.hpf_prev_in  = audio;
                        g_monitor.hpf_prev_out = y;
                        audio = y;
                    }

                    monitor_audio_process(audio);
                }
            }
        }
    }

    monitor_audio_close();
    return 0;
}

/* -------------------------------------------------------------------------
 * monitor_global_init / monitor_shutdown - app lifetime start/stop
 * ------------------------------------------------------------------------- */
static void monitor_global_init(void)
{
    memset(&g_monitor, 0, sizeof(g_monitor));
    InitializeCriticalSection(&g_monitor.settings_lock);

    g_monitor.freq_hz   = DEFAULT_FREQUENCY_HZ;
    g_monitor.freq_hz_b = 0.0;   /* unset - auto-centres on first use of Tuner B */
    g_monitor.bw_khz    = MON_MODE_INFO[MON_MODE_AM6].bw_khz_default;
    g_monitor.notch_khz = 0.0;
    g_monitor.mode      = MON_MODE_AM6;
    g_monitor.last_mode = -1;   /* force first-pass filter build */
    g_monitor.last_tuner = -1;  /* force first-pass history reset */
    g_monitor.last_mode_for_mute = -1;  /* force full-length mute first pass */

    if (ring_init(&g_monitor.ring, MON_RING_BYTES) == 0)
        g_monitor.ring_ready = 1;
    else
        LOG_WARN("Monitor: failed to allocate its IQ ring buffer - disabled.");

    g_monitor.thread = CreateThread(NULL, 0, monitor_thread_func, NULL, 0, NULL);
}

static void monitor_shutdown(void)
{
    g_monitor.thread_stop_req = 1;
    if (g_monitor.thread) {
        WaitForSingleObject(g_monitor.thread, 2000);
        CloseHandle(g_monitor.thread);
        g_monitor.thread = NULL;
    }
    if (g_monitor.audio_done_event) {
        CloseHandle(g_monitor.audio_done_event);
        g_monitor.audio_done_event = NULL;
    }
    if (g_monitor.ring_ready)
        ring_free(&g_monitor.ring);
    DeleteCriticalSection(&g_monitor.settings_lock);
}

/* =========================================================================
 * Live monitor - GUI: controls, layout, owner-draw, input handlers
 * ========================================================================= */
static void monitor_create_controls(HWND parent, HINSTANCE hInst)
{
    int i;
    char buf[32];

    g_hBtnMonitor = mk_button(parent, IDC_BTN_MONITOR,
                               (!g_state.cfg.dual_channel &&
                                !strcmp(g_state.cfg.rspduo_single_tuner, "B"))
                               ? "Tuner B" : "Tuner A");
    g_hBtnFreqLock = mk_button(parent, IDC_BTN_FREQ_LOCK, "A=B");
    EnableWindow(g_hBtnFreqLock, FALSE);   /* Monitor is off at creation
        * time - monitor_sync_button_label() enables it once a session
        * actually meets the criteria (Monitor on, matching CFs).         */

    g_hFreqDigits = freqdigits_create(parent, hInst);
    g_hMonHzLbl = CreateWindowExA(0, "STATIC", "Hz", WS_CHILD | WS_VISIBLE | SS_LEFT,
                    0, 0, 10, 10, parent, NULL, hInst, NULL);

    g_hMonModeLbl = CreateWindowExA(0, "STATIC", "Mode", WS_CHILD | WS_VISIBLE | SS_LEFT,
                    0, 0, 10, 10, parent, NULL, hInst, NULL);
    g_hMonMode = CreateWindowExA(0, "COMBOBOX", "",
                    WS_CHILD | WS_VISIBLE | WS_BORDER | CBS_DROPDOWNLIST | WS_VSCROLL,
                    0, 0, 10, 10, parent, (HMENU)(INT_PTR)IDC_COMBO_MON_MODE,
                    hInst, NULL);
    for (i = 0; i < MON_MODE_COUNT; i++)
        SendMessageA(g_hMonMode, CB_ADDSTRING, 0, (LPARAM)MON_MODE_INFO[MON_MODE_DISPLAY_ORDER[i]].name);
    SendMessageA(g_hMonMode, CB_SETCURSEL, (WPARAM)0, 0); /* display index 0 = MON_MODE_AM6 */
    {
        HMODULE hUx = LoadLibraryA("uxtheme.dll");
        if (hUx) {
            typedef HRESULT (WINAPI *PFN_SWT)(HWND, LPCWSTR, LPCWSTR);
            PFN_SWT pSwt = (PFN_SWT)GetProcAddress(hUx, "SetWindowTheme");
            if (pSwt) pSwt(g_hMonMode, L"DarkMode_Explorer", NULL);
            FreeLibrary(hUx);
        }
    }

    g_hBwDigits = bwdigits_create(parent, hInst);
    g_hMonKhzLbl = CreateWindowExA(0, "STATIC", "kHz", WS_CHILD | WS_VISIBLE | SS_LEFT,
                    0, 0, 10, 10, parent, NULL, hInst, NULL);

    g_hSMeter = smeter_create(parent, hInst);

    g_hBtnNotchEnable = mk_button(parent, IDC_BTN_NOTCH_ENABLE, "IF Notch");
    g_hNotchDigits = notchdigits_create(parent, hInst);
    g_hNotchKhzLbl = CreateWindowExA(0, "STATIC", "kHz", WS_CHILD | WS_VISIBLE | SS_LEFT,
                    0, 0, 10, 10, parent, NULL, hInst, NULL);

    g_hMonVolLbl = CreateWindowExA(0, "STATIC", "Vol", WS_CHILD | WS_VISIBLE | SS_LEFT,
                    0, 0, 10, 10, parent, NULL, hInst, NULL);
    g_hMonVol = CreateWindowExA(0, TRACKBAR_CLASSA, "",
                    WS_CHILD | WS_VISIBLE | TBS_HORZ | TBS_NOTICKS,
                    0, 0, 10, 10, parent, (HMENU)(INT_PTR)IDC_SLIDER_MON_VOL,
                    hInst, NULL);
    /* Direct 0-100% control of the monitor's Windows audio output volume
     * (waveOutSetVolume) - a real, always-audible level control, rather
     * than the old monitor_gain AGC-target multiplier, which fed into an
     * adaptive feedback loop that often masked its effect.               */
    g_monitorVolPercent = g_state.cfg.monitor_volume_percent;
    SendMessageA(g_hMonVol, TBM_SETRANGE, TRUE, MAKELPARAM(0, 100));
    SendMessageA(g_hMonVol, TBM_SETPOS, TRUE, g_monitorVolPercent);
    SendMessageA(g_hMonVol, TBM_SETPAGESIZE, 0, 10);
    apply_reversed_wheel_subclass(g_hMonVol);
    {
        HMODULE hUx = LoadLibraryA("uxtheme.dll");
        if (hUx) {
            typedef HRESULT (WINAPI *PFN_SWT)(HWND, LPCWSTR, LPCWSTR);
            PFN_SWT pSwt = (PFN_SWT)GetProcAddress(hUx, "SetWindowTheme");
            if (pSwt) pSwt(g_hMonVol, L"DarkMode_Explorer", NULL);
            FreeLibrary(hUx);
        }
    }
    snprintf(buf, sizeof(buf), "%d%%", g_monitorVolPercent);
    g_hMonVolVal = CreateWindowExA(0, "STATIC", buf,
                    WS_CHILD | WS_VISIBLE | SS_LEFT,
                    0, 0, 10, 10, parent, NULL, hInst, NULL);

    /* Low-cut filter - moved here from the Settings dialog (Monitor tab)
     * since it's a monitor-audio-only control (it never touches the
     * recorded IQ data) but Settings itself is fully disabled the moment
     * a recording starts, which meant there was no way to reach it while
     * actively recording - exactly the same reasoning that already put
     * Vol and Notch here instead of in Settings.                         */
    g_hBtnHpfEnable = mk_button(parent, IDC_BTN_HPF_ENABLE, "Low Cut");
    g_hHpfSlider = CreateWindowExA(0, TRACKBAR_CLASSA, "",
                    WS_CHILD | WS_VISIBLE | TBS_HORZ | TBS_NOTICKS,
                    0, 0, 10, 10, parent, (HMENU)(INT_PTR)IDC_SLIDER_HPF_HZ,
                    hInst, NULL);
    SendMessageA(g_hHpfSlider, TBM_SETRANGE, TRUE, MAKELPARAM(20, 300));
    SendMessageA(g_hHpfSlider, TBM_SETPOS, TRUE, (int)g_state.cfg.monitor_hpf_hz);
    SendMessageA(g_hHpfSlider, TBM_SETPAGESIZE, 0, 10);
    apply_reversed_wheel_subclass(g_hHpfSlider);
    {
        HMODULE hUx = LoadLibraryA("uxtheme.dll");
        if (hUx) {
            typedef HRESULT (WINAPI *PFN_SWT)(HWND, LPCWSTR, LPCWSTR);
            PFN_SWT pSwt = (PFN_SWT)GetProcAddress(hUx, "SetWindowTheme");
            if (pSwt) pSwt(g_hHpfSlider, L"DarkMode_Explorer", NULL);
            FreeLibrary(hUx);
        }
    }
    snprintf(buf, sizeof(buf), "%d Hz", (int)g_state.cfg.monitor_hpf_hz);
    g_hHpfVal = CreateWindowExA(0, "STATIC", buf,
                    WS_CHILD | WS_VISIBLE | SS_LEFT,
                    0, 0, 10, 10, parent, NULL, hInst, NULL);

    if (g_hFontUI) {
        HWND ctls[] = { g_hMonHzLbl, g_hMonModeLbl, g_hMonMode, g_hMonKhzLbl,
                         g_hNotchKhzLbl, g_hMonVolLbl, g_hMonVolVal, g_hHpfVal };
        size_t k;
        for (k = 0; k < sizeof(ctls) / sizeof(ctls[0]); k++)
            SendMessageA(ctls[k], WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
    }
}

/* Lays out the monitor bar along [bar_y, bar_y+bar_h) from x=12 to
 * right_edge - now two rows: row 1 keeps the original controls (Vol
 * removed from the end), row 2 holds Vol plus the low-cut filter
 * controls moved here from Settings.                                     */
static void monitor_layout(HWND hwnd, int right_edge, int bar_y, int bar_h)
{
    int x = 14;
    int w_btn = 86, w_lock = 44, w_freq = 160, w_hz = 20;
    int w_mode_lbl = 38, w_mode = 105;
    int w_bw_digits = 70, w_khz = 28;
    int w_notch_lbl = 58, w_notch_digits = 86, w_notch_khz = 28;
    int w_vol_lbl = 26, w_vol_val = 50;
    int w_hpf_lbl = 76, w_hpf_val = 54;
    /* Tight gaps sit between a control and something that reads as part
     * of the same unit with it - a digit display and its own unit label
     * (e.g. "kHz"), but also Monitor/A=B, Mode/Bandwidth, and the Notch
     * button/its own digits, which are each a single logical control
     * pair rather than two separate groups. The three gaps that remain
     * between genuinely separate groups (A=B -> Frequency, Frequency's Hz
     * label -> Mode, Bandwidth's kHz label -> Notch) are computed to fill
     * whatever width is left over, so the row reads as evenly spread
     * across the full bar rather than left-packed with a gap at the end. */
    int gap_tight = 4;
    int row1_h = BOTTOM_MON_BAR_H, row2_h = BOTTOM_MON_BAR2_H;
    int row2_y = bar_y + row1_h + BOTTOM_MON_ROW_GAP;
    int slider_x, slider_w, slider_right, slider_h;
    int fixed_w, n_group_gaps = 3, gap_group;

    (void)hwnd; (void)bar_h;

    fixed_w = w_btn + gap_tight + w_lock
            + w_freq + gap_tight + w_hz
            + w_mode_lbl + gap_tight + w_mode + gap_tight + w_bw_digits + gap_tight + w_khz
            + w_notch_lbl + gap_tight + w_notch_digits + gap_tight + w_notch_khz;
    gap_group = ((right_edge - 16) - x - fixed_w) / n_group_gaps;
    if (gap_group < 10) gap_group = 10;
    if (gap_group > 26) gap_group = 26;   /* a wide window shouldn't turn
        * this into visible empty gaps - anything left over past a
        * comfortable spacing is used to centre the row instead, below. */
    {
        int row_w = fixed_w + gap_group * n_group_gaps;
        int leftover = (right_edge - 16) - x - row_w;
        if (leftover > 0) x += leftover / 2;
    }

    /* Row 1 */
    MoveWindow(g_hBtnMonitor, x, bar_y, w_btn, row1_h, TRUE);
    x += w_btn + gap_tight;

    MoveWindow(g_hBtnFreqLock, x, bar_y, w_lock, row1_h, TRUE);
    x += w_lock + gap_group;

    MoveWindow(g_hFreqDigits, x, bar_y + 2, w_freq, row1_h - 4, TRUE);
    x += w_freq + gap_tight;
    MoveWindow(g_hMonHzLbl, x, bar_y + (row1_h - 16) / 2, w_hz, 16, TRUE);
    x += w_hz + gap_group;

    /* A combo box's height parameter covers the closed box PLUS the open
     * drop-down list - giving it only row1_h leaves no room to drop down,
     * so nothing appears to happen when another mode is picked. */
    MoveWindow(g_hMonModeLbl, x, bar_y + (row1_h - 16) / 2, w_mode_lbl, 16, TRUE);
    x += w_mode_lbl + gap_tight;
    MoveWindow(g_hMonMode, x, bar_y + 2, w_mode, row1_h - 4 + 160, TRUE);
    x += w_mode + gap_tight;

    MoveWindow(g_hBwDigits, x, bar_y + 2, w_bw_digits, row1_h - 4, TRUE);
    x += w_bw_digits + gap_tight;
    MoveWindow(g_hMonKhzLbl, x, bar_y + (row1_h - 16) / 2, w_khz, 16, TRUE);
    x += w_khz + gap_group;

    MoveWindow(g_hBtnNotchEnable, x, bar_y, w_notch_lbl, row1_h, TRUE);
    x += w_notch_lbl + gap_tight;

    MoveWindow(g_hNotchDigits, x, bar_y + 2, w_notch_digits, row1_h - 4, TRUE);
    x += w_notch_digits + gap_tight;
    MoveWindow(g_hNotchKhzLbl, x, bar_y + (row1_h - 16) / 2, w_notch_khz, 16, TRUE);

    /* Row 2: Vol, then Low Cut, then the S-meter - same tight/group gap
     * pattern as row 1. Low Cut's slider gets a fixed width now rather
     * than stretching to the right edge, since the S-meter needs to sit
     * after it and fill the remaining space itself instead.              */
    x = 14;
    MoveWindow(g_hMonVolLbl, x, row2_y + (row2_h - 16) / 2, w_vol_lbl, 16, TRUE);
    x += w_vol_lbl + gap_tight;

    slider_right = x + 180;
    slider_x = x;
    slider_w = slider_right - slider_x;
    slider_h = row2_h - 8;
    MoveWindow(g_hMonVol, slider_x, row2_y + 4, slider_w, slider_h, TRUE);
    x = slider_x + slider_w + 8;
    MoveWindow(g_hMonVolVal, x, row2_y + (row2_h - 16) / 2, w_vol_val, 16, TRUE);
    x += w_vol_val + gap_group + 8;

    MoveWindow(g_hBtnHpfEnable, x, row2_y, w_hpf_lbl, row2_h, TRUE);
    x += w_hpf_lbl + 8;

    slider_w = 140;
    slider_h = row2_h - 8;
    MoveWindow(g_hHpfSlider, x, row2_y + 4, slider_w, slider_h, TRUE);
    x += slider_w + 8;
    MoveWindow(g_hHpfVal, x, row2_y + (row2_h - 16) / 2, w_hpf_val, 16, TRUE);
    x += w_hpf_val + gap_group;

    slider_right = right_edge - 16;
    slider_w = slider_right - x;
    if (slider_w < 80) slider_w = 80;
    {
        int smeter_h = 14;
        int smeter_y = row2_y + (row2_h - smeter_h) / 2;
        MoveWindow(g_hSMeter, x, smeter_y, slider_w, smeter_h, TRUE);
    }
}

/* -------------------------------------------------------------------------
 * Scrollable digit frequency display - replaces the old free-text edit
 * box. Hover over a digit and scroll the mouse wheel to step that digit's
 * place value up or down (e.g. scrolling on the hundreds digit steps by
 * 100 Hz), without needing to type anything. Nine digits covers up to
 * 999,999,999 Hz - the whole MW/HF/VHF range this app is built for.
 * ------------------------------------------------------------------------- */
static int freqdigits_cell_width(HDC dc)
{
    SIZE sz;
    HGDIOBJ of = SelectObject(dc, g_hFontFreqDigits);
    GetTextExtentPoint32A(dc, "0", 1, &sz);
    SelectObject(dc, of);
    return sz.cx + 4;
}

/* Width of the "." separator shown after digits 3 and 6, grouping the
 * 9-digit display into MHz.kHz.Hz triplets (e.g. 001.560.000) rather
 * than one undifferentiated string of digits.                           */
static int freqdigits_sep_width(HDC dc)
{
    SIZE sz;
    HGDIOBJ of = SelectObject(dc, g_hFontFreqDigits);
    GetTextExtentPoint32A(dc, ".", 1, &sz);
    SelectObject(dc, of);
    /* No extra padding here, unlike a digit cell's sz.cx+4 - a wider
     * separator was pushing the MHz/kHz/Hz groups further apart than
     * intended; this keeps the "." tight against its neighbouring
     * digits instead.                                                    */
    return sz.cx;
}

/* How many separators sit before digit index i (0-based) - one after
 * index 2, another after index 5, for the 3-3-3 grouping.                */
static int freqdigits_seps_before(int i)
{
    int n = 0;
    if (i >= 3) n++;
    if (i >= 6) n++;
    return n;
}

/* x position of digit i's cell, given the display's total starting x and
 * per-cell/separator widths - shared by painting and hit-testing so they
 * can never disagree about where a digit actually sits.                  */
static int freqdigits_cell_x(int i, int x0, int cell_w, int sep_w)
{
    return x0 + i * cell_w + freqdigits_seps_before(i) * sep_w;
}

static int freqdigits_total_width(HDC dc)
{
    return freqdigits_cell_width(dc) * FREQ_DIGITS_COUNT
         + freqdigits_sep_width(dc) * 2;
}

static int freqdigits_hit_test(HWND hwnd, int client_x)
{
    RECT rc;
    HDC dc;
    int cell_w, sep_w, total_w, x0, i;

    GetClientRect(hwnd, &rc);
    dc = GetDC(hwnd);
    cell_w = freqdigits_cell_width(dc);
    sep_w = freqdigits_sep_width(dc);
    total_w = freqdigits_total_width(dc);
    ReleaseDC(hwnd, dc);
    if (cell_w < 1) cell_w = 1;

    x0 = (rc.right - total_w) / 2;
    if (x0 < 0) x0 = 0;

    if (client_x < x0) return 0;
    for (i = 0; i < FREQ_DIGITS_COUNT; i++) {
        int left = freqdigits_cell_x(i, x0, cell_w, sep_w);
        int right = left + cell_w;
        /* Extend the last digit's zone rightward, and every other digit's
         * zone through the separator (if any) that follows it, so there's
         * no dead click zone sitting in the gap around a decimal point.   */
        if (i == FREQ_DIGITS_COUNT - 1) {
            right += sep_w;
        } else if (freqdigits_seps_before(i + 1) > freqdigits_seps_before(i)) {
            right += sep_w;
        }
        if (client_x < right) return i;
        (void)left;
    }
    return FREQ_DIGITS_COUNT - 1;
}

/* Returns the configured centre frequency for the given tuner selection
 * (0=A, 1=B) - the same fallback logic used everywhere a tuner's centre
 * matters, so the digit control, the NCO shift, and auto-centring all
 * agree on what "centre" means for a given tuner.                      */
static double monitor_center_for_tuner(int tuner_sel)
{
    double center = (tuner_sel == 1 && g_state.cfg.freq_b_hz > 0.0)
                         ? g_state.cfg.freq_b_hz : g_state.cfg.frequency_hz;
    if (center <= 0.0) center = DEFAULT_FREQUENCY_HZ;
    return center;
}

/* Clamps freq to the receiver's actual Nyquist-limited coverage around
 * center - tuning beyond this just NCO-shifts noise or out-of-band
 * content to baseband and demodulates it, sounding like a real signal
 * even though there isn't one there.                                   */
static double monitor_clamp_to_coverage(double freq, double center)
{
    double native = g_state.live_expected_output_rate_hz;  /* frozen for this session - see AppState */
    double half_span;
    if (native < 1000.0) native = DEFAULT_SAMPLE_RATE_HZ;
    half_span = native / 2.0;
    if (freq < center - half_span) freq = center - half_span;
    if (freq > center + half_span) freq = center + half_span;
    if (freq < 0.0) freq = 0.0;
    return freq;
}

/* Returns a pointer to whichever tuner's remembered frequency is
 * currently active (freq_hz for A, freq_hz_b for B), auto-centring it
 * first if it's never been set (0.0) or has drifted outside that
 * tuner's current coverage - e.g. after a Settings change, or the
 * first time a tuner is ever selected. Caller must already hold
 * g_monitor.settings_lock, matching every other access to these
 * fields - this does not lock internally.                              */
static double *monitor_active_freq_ptr(void)
{
    int tuner_sel = (int)g_monitor.tuner_sel;
    double *p = (tuner_sel == 1) ? &g_monitor.freq_hz_b : &g_monitor.freq_hz;
    double center = monitor_center_for_tuner(tuner_sel);

    if (*p <= 0.0) {
        *p = center;
    } else {
        double clamped = monitor_clamp_to_coverage(*p, center);
        if (clamped != *p) *p = center;   /* out of range - re-centre rather
                                            * than just clamp to the edge,
                                            * which would otherwise silently
                                            * land on the very boundary of
                                            * the new coverage.            */
    }
    return p;
}

static LRESULT CALLBACK freqdigits_wndproc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp)
{
    switch (msg) {
    case WM_ERASEBKGND:
        return 1;   /* WM_PAINT fills the whole client area - avoids flicker */

    case WM_PAINT: {
        PAINTSTRUCT ps;
        HDC dc;
        RECT rc;
        char buf[FREQ_DIGITS_COUNT + 1];
        double freq;
        LONG64 ifreq;
        int i, cell_w, sep_w, total_w, x0;
        HGDIOBJ of;
        HPEN pen;
        HGDIOBJ ob, op;

        dc = BeginPaint(hwnd, &ps);
        GetClientRect(hwnd, &rc);
        FillRect(dc, &rc, g_hbrPanel);

        EnterCriticalSection(&g_monitor.settings_lock);
        freq = *monitor_active_freq_ptr();
        LeaveCriticalSection(&g_monitor.settings_lock);
        ifreq = (LONG64)(freq + 0.5);
        if (ifreq < 0) ifreq = 0;
        if (ifreq > 999999999LL) ifreq = 999999999LL;
        snprintf(buf, sizeof(buf), "%09lld", (long long)ifreq);

        cell_w = freqdigits_cell_width(dc);
        sep_w = freqdigits_sep_width(dc);
        total_w = freqdigits_total_width(dc);
        x0 = (rc.right - total_w) / 2;
        if (x0 < 0) x0 = 0;

        SetBkMode(dc, TRANSPARENT);
        of = SelectObject(dc, g_hFontFreqDigits);
        for (i = 0; i < FREQ_DIGITS_COUNT; i++) {
            RECT cellr;
            char ch[2];
            int cx = freqdigits_cell_x(i, x0, cell_w, sep_w);
            cellr.left = cx;
            cellr.top = rc.top;
            cellr.right = cx + cell_w;
            cellr.bottom = rc.bottom;
            ch[0] = buf[i];
            ch[1] = '\0';
            if (i == g_freqDigitsHover) {
                HBRUSH hb = CreateSolidBrush(COL_BTN_HOT);
                FillRect(dc, &cellr, hb);
                DeleteObject(hb);
            }
            SetTextColor(dc, i == g_freqDigitsHover ? RGB(255, 210, 90) : COL_TEXT);
            DrawTextA(dc, ch, 1, &cellr, DT_CENTER | DT_VCENTER | DT_SINGLELINE);

            /* A "." right after this digit, if a separator belongs here
             * (after digit 3 and after digit 6 - MHz.kHz.Hz grouping).    */
            if (i < FREQ_DIGITS_COUNT - 1 &&
                    freqdigits_seps_before(i + 1) > freqdigits_seps_before(i)) {
                RECT sepr;
                sepr.left = cellr.right;
                sepr.top = rc.top;
                sepr.right = cellr.right + sep_w;
                sepr.bottom = rc.bottom;
                SetTextColor(dc, COL_TEXT_DIM);
                DrawTextA(dc, ".", 1, &sepr, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
            }
        }
        SelectObject(dc, of);

        pen = CreatePen(PS_SOLID, 1, COL_PANEL_EDGE);
        op = SelectObject(dc, pen);
        ob = SelectObject(dc, GetStockObject(NULL_BRUSH));
        Rectangle(dc, rc.left, rc.top, rc.right, rc.bottom);
        SelectObject(dc, ob);
        SelectObject(dc, op);
        DeleteObject(pen);

        EndPaint(hwnd, &ps);
        return 0;
    }

    case WM_MOUSEMOVE: {
        int digit = freqdigits_hit_test(hwnd, (int)(short)LOWORD(lp));
        TRACKMOUSEEVENT tme;
        if (digit != g_freqDigitsHover) {
            g_freqDigitsHover = digit;
            InvalidateRect(hwnd, NULL, FALSE);
        }
        tme.cbSize = sizeof(tme);
        tme.dwFlags = TME_LEAVE;
        tme.hwndTrack = hwnd;
        tme.dwHoverTime = 0;
        TrackMouseEvent(&tme);
        return 0;
    }

    case WM_MOUSELEAVE:
        if (g_freqDigitsHover != -1) {
            g_freqDigitsHover = -1;
            InvalidateRect(hwnd, NULL, FALSE);
        }
        return 0;

    case WM_MOUSEWHEEL: {
        int notches = (int)(short)HIWORD(wp) / WHEEL_DELTA;
        POINT pt;
        int digit, p;
        double place, center, *pfreq;

        pt.x = (short)LOWORD(lp);
        pt.y = (short)HIWORD(lp);
        ScreenToClient(hwnd, &pt);
        digit = freqdigits_hit_test(hwnd, pt.x);

        place = 1.0;
        for (p = 0; p < (FREQ_DIGITS_COUNT - 1 - digit); p++)
            place *= 10.0;

        /* Clamp to the receiver's actual coverage (configured centre
         * frequency +/- half the output sample rate) instead of an
         * arbitrary 9-digit bound with no relation to what's really
         * being captured. Tuning beyond that just NCO-shifts whatever
         * noise or out-of-band content the ADC happened to capture
         * down to baseband and demodulates it - which sounds exactly
         * like a genuine signal even though there's nothing there.
         * Tuner-aware: Tuner A and B remember separate frequencies.   */
        EnterCriticalSection(&g_monitor.settings_lock);
        pfreq = monitor_active_freq_ptr();
        center = monitor_center_for_tuner((int)g_monitor.tuner_sel);
        *pfreq = monitor_clamp_to_coverage(*pfreq + place * notches, center);
        if (g_monitor.freq_locked) {
            /* Clamp to the OTHER tuner's own coverage, not just copy the
             * raw value - with two receiver CFs far apart (e.g. genuine
             * Master/Slave, CF1=1MHz, CF2=5MHz), the active tuner's
             * frequency can be completely outside what the other one's
             * ADC ever captured. Clamping means the lock always lands on
             * a real, valid frequency for that tuner - the closest thing
             * to "the same" it can actually receive - rather than
             * silently tuning it somewhere it has no signal at all.      */
            int other_sel = g_monitor.tuner_sel == 0 ? 1 : 0;
            double other_center = monitor_center_for_tuner(other_sel);
            double clamped = monitor_clamp_to_coverage(*pfreq, other_center);
            if (other_sel == 1)
                g_monitor.freq_hz_b = clamped;
            else
                g_monitor.freq_hz = clamped;
        }
        LeaveCriticalSection(&g_monitor.settings_lock);

        /* Retuned - the carrier readout (if showing) was measuring the
         * OLD frequency, so it needs to clear immediately rather than
         * keep showing what looks like a live reading of wherever the
         * dial has now moved to. Only the published display state is
         * touched here (safe from the GUI thread, these are exactly the
         * volatile flags meant for that); the phase accumulator and
         * narrowband filter are DSP-thread-owned and are left alone -
         * they'll naturally converge on the new frequency's data within
         * the next window or two without needing a hard reset here.    */
        g_monitor.carrier_offset_valid_pub = 0;
        g_monitor.carrier_locked_pub       = 0;
        g_monitor.carrier_last_raw_valid   = 0;
        g_monitor.carrier_consec_agree     = 0;
        g_monitor.carrier_last_published_valid = 0;
        g_monitor.carrier_settled_count        = 0;
        g_monitor.carrier_settled_pub          = 0;

        InvalidateRect(hwnd, NULL, FALSE);
        return 0;
    }
    }
    return DefWindowProcA(hwnd, msg, wp, lp);
}

static HWND freqdigits_create(HWND parent, HINSTANCE hInst)
{
    static int class_registered = 0;
    if (!class_registered) {
        WNDCLASSA wc;
        memset(&wc, 0, sizeof(wc));
        wc.lpfnWndProc   = freqdigits_wndproc;
        wc.hInstance     = hInst;
        wc.lpszClassName = "DuoDXFreqDigits";
        wc.hCursor       = LoadCursorA(NULL, (LPCSTR)IDC_ARROW);
        RegisterClassA(&wc);
        class_registered = 1;
    }
    if (!g_hFontFreqDigits) {
        g_hFontFreqDigits = CreateFontA(-18, 0, 0, 0, FW_NORMAL, 0, 0, 0,
                        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                        CLEARTYPE_QUALITY, FIXED_PITCH | FF_MODERN, "Consolas");
    }
    return CreateWindowExA(0, "DuoDXFreqDigits", "",
                WS_CHILD | WS_VISIBLE,
                0, 0, 10, 10, parent, (HMENU)(INT_PTR)IDC_FREQ_DIGITS,
                hInst, NULL);
}

/* -------------------------------------------------------------------------
 * Scrollable digit bandwidth display - fixed format "DDD.D" kHz (000.1 to
 * 500.0), one digit cell per character position except the decimal point,
 * which is drawn but not interactive. Same per-digit scroll-to-adjust idea
 * as the frequency digits above: scrolling the hundreds cell steps by 100
 * kHz, the tens cell by 10 kHz, and so on down to 0.1 kHz on the tenths
 * cell - covers every mode's bandwidth (0.5 kHz CW default up to 180 kHz
 * FM-W default) from one fixed layout, no per-mode adaptation needed.
 * ------------------------------------------------------------------------- */
#define BW_STR_LEN 5   /* "DDD.D" -> positions 0,1,2 = digits, 3 = '.', 4 = digit */

static int bwdigits_cell_width(HDC dc, int is_dot)
{
    SIZE sz;
    HGDIOBJ of = SelectObject(dc, g_hFontFreqDigits);
    GetTextExtentPoint32A(dc, is_dot ? "." : "0", 1, &sz);
    SelectObject(dc, of);
    return sz.cx + (is_dot ? 2 : 4);
}

/* place value for each string position; 0.0 for the dot (non-interactive) */
static double bwdigits_place_value(int pos)
{
    switch (pos) {
    case 0: return 100.0;
    case 1: return 10.0;
    case 2: return 1.0;
    case 4: return 0.1;
    default: return 0.0;   /* pos 3 = the decimal point */
    }
}

static int bwdigits_hit_test(HWND hwnd, int client_x)
{
    RECT rc;
    HDC dc;
    int cell_w[BW_STR_LEN], total_w, x0, i, x;

    GetClientRect(hwnd, &rc);
    dc = GetDC(hwnd);
    for (i = 0; i < BW_STR_LEN; i++)
        cell_w[i] = bwdigits_cell_width(dc, i == 3);
    ReleaseDC(hwnd, dc);

    total_w = 0;
    for (i = 0; i < BW_STR_LEN; i++) total_w += cell_w[i];
    x0 = (rc.right - total_w) / 2;
    if (x0 < 0) x0 = 0;

    x = x0;
    for (i = 0; i < BW_STR_LEN; i++) {
        if (client_x >= x && client_x < x + cell_w[i])
            return i;
        x += cell_w[i];
    }
    return BW_STR_LEN - 1;
}

static LRESULT CALLBACK bwdigits_wndproc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp)
{
    switch (msg) {
    case WM_ERASEBKGND:
        return 1;

    case WM_PAINT: {
        PAINTSTRUCT ps;
        HDC dc;
        RECT rc;
        char buf[BW_STR_LEN + 1];
        double bw;
        int i, cell_w[BW_STR_LEN], total_w, x0, x;
        HGDIOBJ of;
        HPEN pen;
        HGDIOBJ ob, op;

        dc = BeginPaint(hwnd, &ps);
        GetClientRect(hwnd, &rc);
        FillRect(dc, &rc, g_hbrPanel);

        EnterCriticalSection(&g_monitor.settings_lock);
        bw = g_monitor.bw_khz;
        LeaveCriticalSection(&g_monitor.settings_lock);
        if (bw < 0.1) bw = 0.1;
        if (bw > 500.0) bw = 500.0;
        snprintf(buf, sizeof(buf), "%05.1f", bw);

        for (i = 0; i < BW_STR_LEN; i++)
            cell_w[i] = bwdigits_cell_width(dc, i == 3);
        total_w = 0;
        for (i = 0; i < BW_STR_LEN; i++) total_w += cell_w[i];
        x0 = (rc.right - total_w) / 2;
        if (x0 < 0) x0 = 0;

        SetBkMode(dc, TRANSPARENT);
        of = SelectObject(dc, g_hFontFreqDigits);
        x = x0;
        for (i = 0; i < BW_STR_LEN; i++) {
            RECT cellr;
            char ch[2];
            cellr.left = x;
            cellr.top = rc.top;
            cellr.right = x + cell_w[i];
            cellr.bottom = rc.bottom;
            ch[0] = buf[i];
            ch[1] = '\0';
            if (i == g_bwDigitsHover && i != 3) {
                HBRUSH hb = CreateSolidBrush(COL_BTN_HOT);
                FillRect(dc, &cellr, hb);
                DeleteObject(hb);
            }
            SetTextColor(dc, (i == g_bwDigitsHover && i != 3) ? RGB(255, 210, 90) : COL_TEXT);
            DrawTextA(dc, ch, 1, &cellr, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
            x += cell_w[i];
        }
        SelectObject(dc, of);

        pen = CreatePen(PS_SOLID, 1, COL_PANEL_EDGE);
        op = SelectObject(dc, pen);
        ob = SelectObject(dc, GetStockObject(NULL_BRUSH));
        Rectangle(dc, rc.left, rc.top, rc.right, rc.bottom);
        SelectObject(dc, ob);
        SelectObject(dc, op);
        DeleteObject(pen);

        EndPaint(hwnd, &ps);
        return 0;
    }

    case WM_MOUSEMOVE: {
        int cell = bwdigits_hit_test(hwnd, (int)(short)LOWORD(lp));
        TRACKMOUSEEVENT tme;
        if (cell != g_bwDigitsHover) {
            g_bwDigitsHover = cell;
            InvalidateRect(hwnd, NULL, FALSE);
        }
        tme.cbSize = sizeof(tme);
        tme.dwFlags = TME_LEAVE;
        tme.hwndTrack = hwnd;
        tme.dwHoverTime = 0;
        TrackMouseEvent(&tme);
        return 0;
    }

    case WM_MOUSELEAVE:
        if (g_bwDigitsHover != -1) {
            g_bwDigitsHover = -1;
            InvalidateRect(hwnd, NULL, FALSE);
        }
        return 0;

    case WM_MOUSEWHEEL: {
        int notches = (int)(short)HIWORD(wp) / WHEEL_DELTA;
        POINT pt;
        int cell;
        double place, newbw;

        pt.x = (short)LOWORD(lp);
        pt.y = (short)HIWORD(lp);
        ScreenToClient(hwnd, &pt);
        cell = bwdigits_hit_test(hwnd, pt.x);
        place = bwdigits_place_value(cell);
        if (place == 0.0) return 0;   /* hit the decimal point - no-op */

        EnterCriticalSection(&g_monitor.settings_lock);
        newbw = g_monitor.bw_khz + place * notches;
        if (newbw < 0.1) newbw = 0.1;
        if (newbw > 500.0) newbw = 500.0;
        g_monitor.bw_khz = newbw;
        LeaveCriticalSection(&g_monitor.settings_lock);

        InvalidateRect(hwnd, NULL, FALSE);
        return 0;
    }
    }
    return DefWindowProcA(hwnd, msg, wp, lp);
}

static HWND bwdigits_create(HWND parent, HINSTANCE hInst)
{
    static int class_registered = 0;
    if (!class_registered) {
        WNDCLASSA wc;
        memset(&wc, 0, sizeof(wc));
        wc.lpfnWndProc   = bwdigits_wndproc;
        wc.hInstance     = hInst;
        wc.lpszClassName = "DuoDXBwDigits";
        wc.hCursor       = LoadCursorA(NULL, (LPCSTR)IDC_ARROW);
        RegisterClassA(&wc);
        class_registered = 1;
    }
    if (!g_hFontFreqDigits) {
        g_hFontFreqDigits = CreateFontA(-18, 0, 0, 0, FW_NORMAL, 0, 0, 0,
                        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                        CLEARTYPE_QUALITY, FIXED_PITCH | FF_MODERN, "Consolas");
    }
    return CreateWindowExA(0, "DuoDXBwDigits", "",
                WS_CHILD | WS_VISIBLE,
                0, 0, 10, 10, parent, (HMENU)(INT_PTR)IDC_BW_DIGITS,
                hInst, NULL);
}

/* -------------------------------------------------------------------------
 * Scrollable digit notch display - fixed format "DD.DD" kHz magnitude
 * (0.00 to 10.00), plus a separate sign cell to its left. The sign is
 * click-to-toggle only (not scrollable) - keeping polarity and magnitude
 * as two independent controls avoids the ambiguity of "what does scrolling
 * past zero do" that an odometer-style signed digit would raise. Scrolling
 * a magnitude digit always moves the magnitude up or down, clamped to
 * 0.00-10.00; the sign cell alone decides which side of zero that sits on.
 * ------------------------------------------------------------------------- */
#define NOTCH_STR_LEN 5   /* "DD.DD" -> 0,1 = digits, 2 = '.', 3,4 = digits */
#define NOTCH_SIGN_CELL (-2)

static int notchdigits_sign_width(HDC dc)
{
    SIZE sz;
    HGDIOBJ of = SelectObject(dc, g_hFontFreqDigits);
    GetTextExtentPoint32A(dc, "-", 1, &sz);
    SelectObject(dc, of);
    return sz.cx + 6;
}

static int notchdigits_cell_width(HDC dc, int is_dot)
{
    SIZE sz;
    HGDIOBJ of = SelectObject(dc, g_hFontFreqDigits);
    GetTextExtentPoint32A(dc, is_dot ? "." : "0", 1, &sz);
    SelectObject(dc, of);
    return sz.cx + (is_dot ? 2 : 4);
}

static double notchdigits_place_value(int pos)
{
    switch (pos) {
    case 0: return 10.0;
    case 1: return 1.0;
    case 3: return 0.1;
    case 4: return 0.01;
    default: return 0.0;   /* pos 2 = the decimal point */
    }
}

/* Returns NOTCH_SIGN_CELL, or 0..NOTCH_STR_LEN-1 for a digit/dot position. */
static int notchdigits_hit_test(HWND hwnd, int client_x)
{
    RECT rc;
    HDC dc;
    int sign_w, cell_w[NOTCH_STR_LEN], total_w, x0, i, x;

    GetClientRect(hwnd, &rc);
    dc = GetDC(hwnd);
    sign_w = notchdigits_sign_width(dc);
    for (i = 0; i < NOTCH_STR_LEN; i++)
        cell_w[i] = notchdigits_cell_width(dc, i == 2);
    ReleaseDC(hwnd, dc);

    total_w = sign_w;
    for (i = 0; i < NOTCH_STR_LEN; i++) total_w += cell_w[i];
    x0 = (rc.right - total_w) / 2;
    if (x0 < 0) x0 = 0;

    if (client_x < x0 + sign_w) return NOTCH_SIGN_CELL;

    x = x0 + sign_w;
    for (i = 0; i < NOTCH_STR_LEN; i++) {
        if (client_x >= x && client_x < x + cell_w[i])
            return i;
        x += cell_w[i];
    }
    return NOTCH_STR_LEN - 1;
}

static LRESULT CALLBACK notchdigits_wndproc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp)
{
    switch (msg) {
    case WM_ERASEBKGND:
        return 1;

    case WM_PAINT: {
        PAINTSTRUCT ps;
        HDC dc;
        RECT rc;
        char buf[NOTCH_STR_LEN + 1];
        double mag;
        int i, sign_w, cell_w[NOTCH_STR_LEN], total_w, x0, x;
        HGDIOBJ of;
        HPEN pen;
        HGDIOBJ ob, op;

        dc = BeginPaint(hwnd, &ps);
        GetClientRect(hwnd, &rc);
        FillRect(dc, &rc, g_hbrPanel);

        EnterCriticalSection(&g_monitor.settings_lock);
        mag = fabs(g_monitor.notch_khz);
        LeaveCriticalSection(&g_monitor.settings_lock);
        if (mag > 10.0) mag = 10.0;
        snprintf(buf, sizeof(buf), "%05.2f", mag);

        sign_w = notchdigits_sign_width(dc);
        for (i = 0; i < NOTCH_STR_LEN; i++)
            cell_w[i] = notchdigits_cell_width(dc, i == 2);
        total_w = sign_w;
        for (i = 0; i < NOTCH_STR_LEN; i++) total_w += cell_w[i];
        x0 = (rc.right - total_w) / 2;
        if (x0 < 0) x0 = 0;

        SetBkMode(dc, TRANSPARENT);
        of = SelectObject(dc, g_hFontFreqDigits);

        /* Sign cell */
        {
            RECT signr;
            signr.left = x0;
            signr.top = rc.top;
            signr.right = x0 + sign_w;
            signr.bottom = rc.bottom;
            if (g_notchDigitsHover == NOTCH_SIGN_CELL) {
                HBRUSH hb = CreateSolidBrush(COL_BTN_HOT);
                FillRect(dc, &signr, hb);
                DeleteObject(hb);
            }
            SetTextColor(dc, g_notchDigitsHover == NOTCH_SIGN_CELL
                              ? RGB(255, 210, 90) : COL_TEXT);
            DrawTextA(dc, g_notchNegative ? "-" : "+", 1, &signr,
                      DT_CENTER | DT_VCENTER | DT_SINGLELINE);
        }

        x = x0 + sign_w;
        for (i = 0; i < NOTCH_STR_LEN; i++) {
            RECT cellr;
            char ch[2];
            cellr.left = x;
            cellr.top = rc.top;
            cellr.right = x + cell_w[i];
            cellr.bottom = rc.bottom;
            ch[0] = buf[i];
            ch[1] = '\0';
            if (i == g_notchDigitsHover && i != 2) {
                HBRUSH hb = CreateSolidBrush(COL_BTN_HOT);
                FillRect(dc, &cellr, hb);
                DeleteObject(hb);
            }
            SetTextColor(dc, (i == g_notchDigitsHover && i != 2) ? RGB(255, 210, 90) : COL_TEXT);
            DrawTextA(dc, ch, 1, &cellr, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
            x += cell_w[i];
        }
        SelectObject(dc, of);

        pen = CreatePen(PS_SOLID, 1, COL_PANEL_EDGE);
        op = SelectObject(dc, pen);
        ob = SelectObject(dc, GetStockObject(NULL_BRUSH));
        Rectangle(dc, rc.left, rc.top, rc.right, rc.bottom);
        SelectObject(dc, ob);
        SelectObject(dc, op);
        DeleteObject(pen);

        EndPaint(hwnd, &ps);
        return 0;
    }

    case WM_MOUSEMOVE: {
        int cell = notchdigits_hit_test(hwnd, (int)(short)LOWORD(lp));
        TRACKMOUSEEVENT tme;
        if (cell != g_notchDigitsHover) {
            g_notchDigitsHover = cell;
            InvalidateRect(hwnd, NULL, FALSE);
        }
        tme.cbSize = sizeof(tme);
        tme.dwFlags = TME_LEAVE;
        tme.hwndTrack = hwnd;
        tme.dwHoverTime = 0;
        TrackMouseEvent(&tme);
        return 0;
    }

    case WM_MOUSELEAVE:
        if (g_notchDigitsHover != -1) {
            g_notchDigitsHover = -1;
            InvalidateRect(hwnd, NULL, FALSE);
        }
        return 0;

    case WM_LBUTTONDOWN: {
        int cell = notchdigits_hit_test(hwnd, (int)(short)LOWORD(lp));
        if (cell == NOTCH_SIGN_CELL) {
            double mag;
            g_notchNegative = !g_notchNegative;
            EnterCriticalSection(&g_monitor.settings_lock);
            mag = fabs(g_monitor.notch_khz);
            g_monitor.notch_khz = g_notchNegative ? -mag : mag;
            LeaveCriticalSection(&g_monitor.settings_lock);
            InvalidateRect(hwnd, NULL, FALSE);
        }
        return 0;
    }

    case WM_MOUSEWHEEL: {
        int notches = (int)(short)HIWORD(wp) / WHEEL_DELTA;
        POINT pt;
        int cell;
        double place, mag, newmag;

        pt.x = (short)LOWORD(lp);
        pt.y = (short)HIWORD(lp);
        ScreenToClient(hwnd, &pt);
        cell = notchdigits_hit_test(hwnd, pt.x);
        if (cell == NOTCH_SIGN_CELL) return 0;   /* sign is click-only */
        place = notchdigits_place_value(cell);
        if (place == 0.0) return 0;              /* hit the decimal point */

        EnterCriticalSection(&g_monitor.settings_lock);
        mag = fabs(g_monitor.notch_khz);
        newmag = mag + place * notches;
        if (newmag < 0.0) newmag = 0.0;
        if (newmag > 10.0) newmag = 10.0;
        g_monitor.notch_khz = g_notchNegative ? -newmag : newmag;
        LeaveCriticalSection(&g_monitor.settings_lock);

        InvalidateRect(hwnd, NULL, FALSE);
        return 0;
    }
    }
    return DefWindowProcA(hwnd, msg, wp, lp);
}

static HWND notchdigits_create(HWND parent, HINSTANCE hInst)
{
    static int class_registered = 0;
    if (!class_registered) {
        WNDCLASSA wc;
        memset(&wc, 0, sizeof(wc));
        wc.lpfnWndProc   = notchdigits_wndproc;
        wc.hInstance     = hInst;
        wc.lpszClassName = "DuoDXNotchDigits";
        wc.hCursor       = LoadCursorA(NULL, (LPCSTR)IDC_ARROW);
        RegisterClassA(&wc);
        class_registered = 1;
    }
    if (!g_hFontFreqDigits) {
        g_hFontFreqDigits = CreateFontA(-18, 0, 0, 0, FW_NORMAL, 0, 0, 0,
                        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                        CLEARTYPE_QUALITY, FIXED_PITCH | FF_MODERN, "Consolas");
    }
    return CreateWindowExA(0, "DuoDXNotchDigits", "",
                WS_CHILD | WS_VISIBLE,
                0, 0, 10, 10, parent, (HMENU)(INT_PTR)IDC_NOTCH_DIGITS,
                hInst, NULL);
}

/* -------------------------------------------------------------------------
 * Narrowband S-meter - a classic bar-graph S-meter for the live monitor's
 * own tuned signal (see the smeter_* fields' comments in the AppMonitor
 * struct for where and how it's measured). Green up to S9, orange above.
 * The digital readout shows an approximate dBm value - gain-compensated
 * using currGain, the same approach SDRplay's own Spectrum Analyzer tool
 * uses, but the final ADC-to-dBm reference isn't published anywhere, so
 * accuracy depends on monitor_smeter_cal_offset being set against a known
 * signal source (Settings > Monitor). Uninitialised (offset = 0), this is
 * a relative-but-gain-compensated reading, not a calibrated absolute one.
 * ------------------------------------------------------------------------- */
#define SMETER_SEGS       40
/* S9 isn't one fixed dBm figure - it's a band-dependent convention (IARU
 * Region 1 VHF Managers' Handbook): S9 = 50 uV across 50 ohm = -73 dBm on
 * HF/MW, but only 5 uV = -93 dBm at 30 MHz and above, since VHF/UHF noise
 * floors are routinely 15-25 dB lower than HF's atmospheric/man-made
 * noise (e.g. an RSPdx typically shows roughly -120 dBm HF vs. -143 dBm
 * at 100 MHz with no antenna). Using the HF figure everywhere would make
 * genuine VHF/UHF signals read a full S-unit-and-then-some low. S0 and
 * the S9+60 ceiling are then defined relative to whichever S9 is active,
 * keeping the same "9 x 6 dB S-units below S9" and "60 dB of headroom
 * above S9" spans in both bands, rather than being independently fixed
 * numbers that would need their own separate band-awareness.            */
#define SMETER_S9_HF_DBM       (-73.0f)   /* below 30 MHz */
#define SMETER_S9_VHF_DBM      (-93.0f)   /* 30 MHz and above */
#define SMETER_VHF_THRESHOLD_HZ  30000000.0
/* A straight linear dBm-to-position mapping puts S9 at (54 dB of S0-S9)
 * / (114 dB of S0-S9+60) = ~47% along the bar - so S9 and above (all
 * "strong signal" territory) claims over half the meter, while ordinary
 * S1-S9 reception, which is most real-world reception, is squeezed into
 * less than half. Traditional analog S-meter faceplates deliberately
 * don't map linearly for this reason: S0-S9 gets the majority of the
 * scale's resolution, S9-and-up gets a smaller remainder just enough to
 * show roughly how far over. SMETER_S9_TARGET_FRAC is where S9 should
 * land along the bar (0..1).                                            */
#define SMETER_S9_TARGET_FRAC  0.75f

/* Picks the S9 reference for whichever tuner the live monitor is
 * currently on, per the band convention above.                         */
static float smeter_s9_dbm_for_current_tuner(void)
{
    double freq_hz = (g_monitor.tuner_sel == 1) ? g_monitor.freq_hz_b
                                                 : g_monitor.freq_hz;
    return (freq_hz >= SMETER_VHF_THRESHOLD_HZ) ? SMETER_S9_VHF_DBM
                                                 : SMETER_S9_HF_DBM;
}

/* Picks how long the carrier tracker should integrate before it will call
 * a window "done" - see CARRIER_WINDOW_SAMPLES_MAX for the reasoning.
 * Tiered in S-unit-ish steps (6 dB each) below S9, doubling the window
 * roughly every couple of S-units weaker, capped at 12s so even very
 * weak DX doesn't wait forever for a reading that may never fully settle. */
static int carrier_window_samples_for_signal(void)
{
    float s9_dbm = smeter_s9_dbm_for_current_tuner();
    float below_s9 = s9_dbm - g_monitor.smeter_dbm_pub;  /* + = weaker than S9 */
    if (below_s9 <= 0.0f)  return  32000;   /* >= S9          : 1s  */
    if (below_s9 <= 12.0f) return  64000;   /* S7 - S9        : 2s  */
    if (below_s9 <= 24.0f) return 128000;   /* S5 - S7        : 4s  */
    if (below_s9 <= 36.0f) return 256000;   /* S3 - S5        : 8s  */
    return CARRIER_WINDOW_SAMPLES_MAX;      /* weaker than S3 : 12s */
}

/* Maps a dBm reading to a 0..1 bar-fill fraction via two straight-line
 * segments meeting at s9_dbm, rather than one continuous curve -
 * deliberately, so each end can be reasoned about and tuned
 * independently:
 *   S0..S9  -> 0 .. SMETER_S9_TARGET_FRAC, linear. Each 6 dB S-unit gets
 *              equal width this way, the same as how S-units are
 *              actually defined and how analog faceplates tick them -
 *              a near-floor reading (e.g. an RSPdx's typical ~-120 dBm
 *              with no antenna on HF, only just above S0) lands close to
 *              the left edge, around where S1 sits, rather than a third
 *              of the way across as a single global curve (e.g. a
 *              power/log-style curve fitted only to hit S9 in the right
 *              place) would leave it - that shape has no independent
 *              control over the low end at all, only the one point it's
 *              fitted to.
 *   S9..S9+60 -> SMETER_S9_TARGET_FRAC .. 1, linear. Coarse "how far
 *              over S9" indication in the remaining quarter.            */
static float smeter_warp(float dbm, float s9_dbm)
{
    float min_dbm = s9_dbm - 54.0f;   /* S0: 9 S-units of 6 dB below S9 */
    float max_dbm = s9_dbm + 60.0f;   /* S9+60 */
    float norm;
    if (dbm <= min_dbm) return 0.0f;
    if (dbm >= max_dbm) return 1.0f;
    if (dbm <= s9_dbm) {
        norm = (dbm - min_dbm) / (s9_dbm - min_dbm);
        return norm * SMETER_S9_TARGET_FRAC;
    } else {
        norm = (dbm - s9_dbm) / (max_dbm - s9_dbm);
        return SMETER_S9_TARGET_FRAC + norm * (1.0f - SMETER_S9_TARGET_FRAC);
    }
}

static LRESULT CALLBACK smeter_wndproc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp)
{
    if (msg == WM_ERASEBKGND)
        return 1;   /* WM_PAINT fills the whole client area - avoids flicker */

    if (msg == WM_PAINT) {
        PAINTSTRUCT ps;
        HDC dc;
        RECT rc, meter_r, readout_r;
        HGDIOBJ of;
        HPEN pen;
        HGDIOBJ ob, op;
        int is_active, active_segs, s9_seg, readout_w, pad, gap;
        int innerW, segW, x, y, h, i;
        float dbm, norm, s9_dbm;
        char readout[16];

        dc = BeginPaint(hwnd, &ps);
        GetClientRect(hwnd, &rc);
        FillRect(dc, &rc, g_hbrPanel);

        is_active = g_monitor.enabled ? 1 : 0;
        s9_dbm    = smeter_s9_dbm_for_current_tuner();
        dbm       = is_active ? g_monitor.smeter_dbm_pub : -200.0f;  /* well below
                                                                      * any S0, so
                                                                      * this always
                                                                      * reads empty */

        norm = smeter_warp(dbm, s9_dbm);
        active_segs = (int)(norm * SMETER_SEGS + 0.5f);

        s9_seg = (int)(SMETER_S9_TARGET_FRAC * SMETER_SEGS + 0.5f);

        readout_w = 56;
        meter_r = rc;
        meter_r.right -= readout_w;
        readout_r = rc;
        readout_r.left = meter_r.right + 4;

        pad = 2;
        gap = 1;
        innerW = (meter_r.right - meter_r.left) - pad * 2;
        segW = (innerW - gap * (SMETER_SEGS - 1)) / SMETER_SEGS;
        if (segW < 1) segW = 1;
        x = meter_r.left + pad;
        y = meter_r.top + 2;
        h = (meter_r.bottom - meter_r.top) - 4;
        if (h < 2) h = 2;

        for (i = 0; i < SMETER_SEGS; i++) {
            RECT segr;
            COLORREF c;
            HBRUSH b;
            segr.left = x;
            segr.top = y;
            segr.right = x + segW;
            segr.bottom = y + h;
            if (i < active_segs)
                /* Dark, muted lit colours - deliberately not bright/neon,
                 * so this reads as informative rather than distracting. */
                c = (i >= s9_seg) ? RGB(215, 130, 40) : RGB(60, 165, 85);
            else
                c = RGB(30, 33, 40);   /* unlit segment */
            b = CreateSolidBrush(c);
            FillRect(dc, &segr, b);
            DeleteObject(b);
            x += segW + gap;
        }

        SetBkMode(dc, TRANSPARENT);
        of = SelectObject(dc, g_hFontUI);
        if (is_active)
            snprintf(readout, sizeof(readout), "%ddBm", (int)(dbm + (dbm < 0 ? -0.5f : 0.5f)));
        else
            readout[0] = '\0';
        SetTextColor(dc, COL_TEXT_DIM);
        DrawTextA(dc, readout, -1, &readout_r, DT_LEFT | DT_VCENTER | DT_SINGLELINE);
        SelectObject(dc, of);

        pen = CreatePen(PS_SOLID, 1, COL_PANEL_EDGE);
        op = SelectObject(dc, pen);
        ob = SelectObject(dc, GetStockObject(NULL_BRUSH));
        Rectangle(dc, meter_r.left, meter_r.top, meter_r.right, meter_r.bottom);
        SelectObject(dc, ob);
        SelectObject(dc, op);
        DeleteObject(pen);

        EndPaint(hwnd, &ps);
        return 0;
    }

    return DefWindowProcA(hwnd, msg, wp, lp);
}

static HWND smeter_create(HWND parent, HINSTANCE hInst)
{
    static int class_registered = 0;
    if (!class_registered) {
        WNDCLASSA wc;
        memset(&wc, 0, sizeof(wc));
        wc.lpfnWndProc   = smeter_wndproc;
        wc.hInstance     = hInst;
        wc.lpszClassName = "DuoDXSMeter";
        wc.hCursor       = LoadCursorA(NULL, (LPCSTR)IDC_ARROW);
        RegisterClassA(&wc);
        class_registered = 1;
    }
    return CreateWindowExA(0, "DuoDXSMeter", "",
                WS_CHILD | WS_VISIBLE,
                0, 0, 10, 10, parent, (HMENU)(INT_PTR)IDC_SMETER,
                hInst, NULL);
}


static void monitor_apply_mode_from_combo(void)
{
    LRESULT sel;
    MonMode mode;
    if (!g_hMonMode || !g_hBwDigits) return;

    sel = SendMessageA(g_hMonMode, CB_GETCURSEL, 0, 0);
    if (sel == CB_ERR || sel < 0 || sel >= MON_MODE_COUNT) return;
    mode = MON_MODE_DISPLAY_ORDER[sel];
    g_monitor.mode = (LONG)mode;

    /* Auto-fill the bandwidth digits with this mode's default, as before. */
    EnterCriticalSection(&g_monitor.settings_lock);
    g_monitor.bw_khz = MON_MODE_INFO[mode].bw_khz_default;
    LeaveCriticalSection(&g_monitor.settings_lock);
    InvalidateRect(g_hBwDigits, NULL, FALSE);
}

static void monitor_apply_volume_from_slider(void)
{
    int pos;
    char buf[16];
    if (!g_hMonVol || !g_hMonVolVal) return;

    pos = (int)SendMessageA(g_hMonVol, TBM_GETPOS, 0, 0);   /* 0-100 */
    if (pos < 0) pos = 0;
    if (pos > 100) pos = 100;
    g_monitorVolPercent = pos;

    if (g_monitor.audio_open && g_monitor.hwo) {
        WORD lvl = (WORD)((pos * 0xFFFF) / 100);
        waveOutSetVolume(g_monitor.hwo, MAKELONG(lvl, lvl));
    }

    snprintf(buf, sizeof(buf), "%d%%", pos);
    SetWindowTextA(g_hMonVolVal, buf);

    g_state.cfg.monitor_volume_percent = pos;
    {
        IniPatchEntry e[1];
        e[0].key = "monitor_volume_percent";
        snprintf(e[0].value, sizeof(e[0].value), "%d", pos);
        ini_patch_values("duodx.ini", e, 1);
    }
}

/* Unlike volume, this needs no explicit "apply" call to the audio system -
 * the monitor's processing loop already reads g_state.cfg.monitor_hpf_hz
 * directly on every sample (see the low-cut filter block in the audio
 * pipeline), so updating the config field here is all that's needed for
 * the change to take effect immediately. Persisted to the ini right away
 * too, since this was previously a Settings-managed value that survived
 * restarts, and moving it to the main window shouldn't lose that.        */
static void monitor_apply_hpf_hz_from_slider(void)
{
    int pos;
    char buf[16];
    IniPatchEntry e[1];
    if (!g_hHpfSlider || !g_hHpfVal) return;

    pos = (int)SendMessageA(g_hHpfSlider, TBM_GETPOS, 0, 0);
    if (pos < 20) pos = 20;
    if (pos > 300) pos = 300;
    g_state.cfg.monitor_hpf_hz = (double)pos;

    snprintf(buf, sizeof(buf), "%d Hz", pos);
    SetWindowTextA(g_hHpfVal, buf);

    e[0].key = "monitor_hpf_hz";
    snprintf(e[0].value, sizeof(e[0].value), "%d", pos);
    ini_patch_values("duodx.ini", e, 1);
}

/* Right-click on the Monitor button (RSPduo dual/Master-Slave only)
 * switches which tuner is being listened to. Previously this opened a
 * small popup menu with "Monitor Tuner A"/"Monitor Tuner B" items,
 * requiring a second click to pick one - since there are only ever two
 * tuners, a menu never had anything to offer beyond "the other one",
 * so right-click now toggles directly instead.                        */
static void monitor_toggle_tuner_sel(void)
{
    g_monitor.tuner_sel = g_monitor.tuner_sel ? 0 : 1;
    SetWindowTextA(g_hBtnMonitor, g_monitor.tuner_sel ? "Tuner B" : "Tuner A");
    InvalidateRect(g_hBtnMonitor, NULL, TRUE);
    if (g_hFreqDigits) InvalidateRect(g_hFreqDigits, NULL, TRUE);
}

/* Remembers whether Record (armed for an immediate or timer-waiting
 * recording) or Monitor (plain listening) was active before a tuner
 * switch, so the restart in WM_APP_TUNER_SWITCH_RESTART brings back
 * exactly the same kind of session rather than always reverting to
 * plain listening regardless of what was actually running.               */
static volatile int g_tuner_switch_was_record = 0;

static DWORD WINAPI tuner_switch_wait_thread(LPVOID param)
{
    (void)param;
    /* Runs on its own thread specifically so this wait doesn't block the
     * GUI thread - the old session's device close (Uninit/Release/Close)
     * can take a real, if usually brief, amount of time.                 */
    if (g_worker_thread)
        WaitForSingleObject(g_worker_thread, 35000);
    if (g_hwnd) PostMessageA(g_hwnd, WM_APP_TUNER_SWITCH_RESTART, 0, 0);
    return 0;
}

/* Right-click tuner switch for plain single-tuner RSPduo mode (not
 * dual_channel, not Master/Slave - those already have their own working
 * switch via monitor_toggle_tuner_sel()). Switching which single tuner is
 * active requires a real device re-init, the same as any other tuner-
 * selection change - see settings_save()'s own handling of this. Three
 * cases: fully idle, just flip the config for next time; listening or
 * waiting for a timer (not yet actually recording), stop cleanly and
 * restart on the new tuner, preserving whether this was a Record or
 * Monitor press; actually recording (file open), refuse outright.        */
static void monitor_switch_single_tuner_live(void)
{
    int was_b = !strcmp(g_state.cfg.rspduo_single_tuner, "B");
    const char *new_tuner = was_b ? "A" : "B";
    IniPatchEntry e[1];

    if (g_worker_active && !g_state.listening) {
        LOG_WARN("Can't switch tuners while actively recording - stop "
                 "first, then switch.");
        return;
    }

    strncpy(g_state.cfg.rspduo_single_tuner, new_tuner,
            sizeof(g_state.cfg.rspduo_single_tuner) - 1);
    e[0].key = "rspduo_single_tuner";
    snprintf(e[0].value, sizeof(e[0].value), "%s", new_tuner);
    ini_patch_values("duodx.ini", e, 1);

    if (!g_worker_active) {
        /* Fully idle - takes effect next session; monitor_sync_button_
         * label()'s own change-detection wouldn't catch this (has_tuner_b/
         * tuner_sel/coherent are all unchanged), so update the label
         * directly here instead.                                        */
        if (g_hBtnMonitor) {
            SetWindowTextA(g_hBtnMonitor, was_b ? "Tuner A" : "Tuner B");
            InvalidateRect(g_hBtnMonitor, NULL, TRUE);
        }
        return;
    }

    /* Listening or waiting for a timer - stop cleanly and restart on the
     * new tuner. g_toggle_btn_recording distinguishes a Record press
     * (possibly still just waiting for a timer, not yet actually
     * recording) from a plain Monitor press, captured before the stop so
     * the restart below brings back the right one.                      */
    g_tuner_switch_was_record = g_toggle_btn_recording;
    LOG_INFO("Switching to Tuner %s - restarting the receiver on the new "
             "tuner...", new_tuner);
    gui_stop_session(0);
    {
        HANDLE th = CreateThread(NULL, 0, tuner_switch_wait_thread, NULL, 0, NULL);
        if (th) CloseHandle(th);
    }
}

/* Keeps the Monitor button's label matched to whether there's actually a
 * Tuner B to switch to - matches whichever physical tuner is actually
 * active ("Tuner A"/"Tuner B") either way, single-tuner mode included
 * (previously showed a bare "Monitor" there with no A/B indication at
 * all). Also forces tuner_sel back to
 * A in single-tuner mode, since there's no B data to listen to anyway.
 * Safe to call unconditionally/often - cheap, no stale-state tracking. */
static void monitor_sync_button_label(void)
{
    int has_tuner_b = g_state.cfg.dual_channel || g_state.master_slave_active;
    int tuner_sel   = g_monitor.tuner_sel;
    /* Same definition of "coherent" as the main window's own indicator -
     * see s.coherent above. Only meaningful for genuinely comparing the
     * same frequency on both tuners; with CFs far apart (different MW
     * segments, different bands entirely), there's no shared frequency
     * to lock to in the first place. Requires Monitor actually being on
     * (not just dual_channel/Master-Slave configured while idle), and
     * covers Master/Slave now too, not just dual_channel - both have two
     * genuinely independent, simultaneously-streaming tuners, which is
     * what A=B actually needs to mean anything.                          */
    int coherent = (g_monitor.enabled && has_tuner_b &&
                     fabs(g_state.cfg.frequency_hz - g_state.cfg.freq_b_hz) < 1.0)
                   ? 1 : 0;
    if (!g_hBtnMonitor) return;
    if (has_tuner_b == g_last_has_tuner_b && tuner_sel == g_last_tuner_sel
            && coherent == g_last_lock_coherent)
        return;   /* nothing changed since last tick - skip the repaint */
    if (!has_tuner_b) {
        g_monitor.tuner_sel = 0;
        tuner_sel = 0;
        SetWindowTextA(g_hBtnMonitor,
                        !strcmp(g_state.cfg.rspduo_single_tuner, "B")
                        ? "Tuner B" : "Tuner A");
    } else {
        SetWindowTextA(g_hBtnMonitor,
                        tuner_sel == 1 ? "Tuner B" : "Tuner A");
    }
    if (!has_tuner_b || !coherent) {
        if (g_monitor.freq_locked) {
            g_monitor.freq_locked = 0;
            if (g_hBtnFreqLock) InvalidateRect(g_hBtnFreqLock, NULL, TRUE);
        }
    }
    if (g_hBtnFreqLock) EnableWindow(g_hBtnFreqLock, has_tuner_b && coherent);
    InvalidateRect(g_hBtnMonitor, NULL, TRUE);
    if (g_hFreqDigits) InvalidateRect(g_hFreqDigits, NULL, TRUE);
    g_last_has_tuner_b   = has_tuner_b;
    g_last_tuner_sel     = tuner_sel;
    g_last_lock_coherent = coherent;
}

/* Owner-draw for the Monitor button - green when enabled, matching the
 * existing Start/Stop and AGC button style. Returns TRUE if handled. */
/* Owner-draw for the IF Notch enable/disable button - green when the
 * notch is actually applied, matching the Monitor button's style. */
static int notch_draw_button(LPDRAWITEMSTRUCT di)
{
    int dis  = (di->itemState & ODS_DISABLED) != 0;
    int down = (di->itemState & ODS_SELECTED) != 0;
    COLORREF face;
    HBRUSH b;
    HPEN p;
    HGDIOBJ ob, op, of;
    char txt[32];

    if (!dis && g_monitor.notch_enabled) {
        face = down ? RGB(40, 120, 80) : COL_BTN_START;
    } else {
        face = dis ? COL_BTN_DIS : (down ? COL_BTN_HOT : COL_BTN_FACE);
    }

    b = CreateSolidBrush(face);
    FillRect(di->hDC, &di->rcItem, g_hbrBg);
    p = CreatePen(PS_SOLID, 1, COL_PANEL_EDGE);
    ob = SelectObject(di->hDC, b);
    op = SelectObject(di->hDC, p);
    RoundRect(di->hDC, di->rcItem.left, di->rcItem.top,
              di->rcItem.right, di->rcItem.bottom, 5, 5);
    SelectObject(di->hDC, ob);
    SelectObject(di->hDC, op);
    DeleteObject(b);
    DeleteObject(p);

    GetWindowTextA(di->hwndItem, txt, sizeof(txt));
    SetBkMode(di->hDC, TRANSPARENT);
    SetTextColor(di->hDC, dis ? COL_TEXT_DIM : COL_TEXT);
    of = SelectObject(di->hDC, g_hFontUI);
    DrawTextA(di->hDC, txt, -1, &di->rcItem, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
    SelectObject(di->hDC, of);
    return TRUE;
}

/* Same styling as notch_draw_button() above - green when enabled, since
 * this button toggles g_state.cfg.monitor_hpf_enable rather than being a
 * momentary action, matching how Notch and Monitor both show their own
 * on/off state the same way.                                             */
static int hpf_draw_button(LPDRAWITEMSTRUCT di)
{
    int dis  = (di->itemState & ODS_DISABLED) != 0;
    int down = (di->itemState & ODS_SELECTED) != 0;
    COLORREF face;
    HBRUSH b;
    HPEN p;
    HGDIOBJ ob, op, of;
    char txt[32];

    if (!dis && g_state.cfg.monitor_hpf_enable) {
        face = down ? RGB(40, 120, 80) : COL_BTN_START;
    } else {
        face = dis ? COL_BTN_DIS : (down ? COL_BTN_HOT : COL_BTN_FACE);
    }

    b = CreateSolidBrush(face);
    FillRect(di->hDC, &di->rcItem, g_hbrBg);
    p = CreatePen(PS_SOLID, 1, COL_PANEL_EDGE);
    ob = SelectObject(di->hDC, b);
    op = SelectObject(di->hDC, p);
    RoundRect(di->hDC, di->rcItem.left, di->rcItem.top,
              di->rcItem.right, di->rcItem.bottom, 5, 5);
    SelectObject(di->hDC, ob);
    SelectObject(di->hDC, op);
    DeleteObject(b);
    DeleteObject(p);

    GetWindowTextA(di->hwndItem, txt, sizeof(txt));
    SetBkMode(di->hDC, TRANSPARENT);
    SetTextColor(di->hDC, dis ? COL_TEXT_DIM : COL_TEXT);
    of = SelectObject(di->hDC, g_hFontUI);
    DrawTextA(di->hDC, txt, -1, &di->rcItem, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
    SelectObject(di->hDC, of);
    return TRUE;
}

static int monitor_draw_button(LPDRAWITEMSTRUCT di)
{
    int dis  = (di->itemState & ODS_DISABLED) != 0;
    int down = (di->itemState & ODS_SELECTED) != 0;
    COLORREF face;
    HBRUSH b;
    HPEN p;
    HGDIOBJ ob, op, of;
    char txt[32];

    if (!dis && g_monitor.enabled) {
        face = down ? RGB(40, 120, 80) : COL_BTN_START;
    } else {
        face = dis ? COL_BTN_DIS : (down ? COL_BTN_HOT : COL_BTN_FACE);
    }

    b = CreateSolidBrush(face);
    FillRect(di->hDC, &di->rcItem, g_hbrBg);
    p = CreatePen(PS_SOLID, 1, COL_PANEL_EDGE);
    ob = SelectObject(di->hDC, b);
    op = SelectObject(di->hDC, p);
    RoundRect(di->hDC, di->rcItem.left, di->rcItem.top,
              di->rcItem.right, di->rcItem.bottom, 5, 5);
    SelectObject(di->hDC, ob);
    SelectObject(di->hDC, op);
    DeleteObject(b);
    DeleteObject(p);

    GetWindowTextA(di->hwndItem, txt, sizeof(txt));
    SetBkMode(di->hDC, TRANSPARENT);
    SetTextColor(di->hDC, dis ? COL_TEXT_DIM : COL_TEXT);
    of = SelectObject(di->hDC, g_hFontUI);
    DrawTextA(di->hDC, txt, -1, &di->rcItem, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
    SelectObject(di->hDC, of);
    return TRUE;
}

static void paint_window(HWND hwnd)
{
    PAINTSTRUCT ps;
    HDC wdc = BeginPaint(hwnd, &ps);

    RECT cr;
    GetClientRect(hwnd, &cr);

    /* Double buffer to avoid flicker */
    HDC dc = CreateCompatibleDC(wdc);
    HBITMAP bmp = CreateCompatibleBitmap(wdc, cr.right, cr.bottom);
    HGDIOBJ obmp = SelectObject(dc, bmp);

    /* Background */
    FillRect(dc, &cr, g_hbrBg);

    UiSnapshot s = g_ui;   /* snapshot */

    /* ---- Title bar strip (baseline-aligned) ---- */
    {
        int baseline = 34;
        int x = 14;
        x += draw_text_base(dc, x, baseline, "DuoDX", COL_ACCENT, g_hFontBig);
        draw_text_base(dc, x + 12, baseline, "RSP Dual Channel IQ Recorder  v" VERSION,
                       COL_TEXT_DIM, g_hFontUI);

        /* Recording LED + state. The LED sits at a FIXED position and the
         * state word is drawn left-justified to its right, so the LED does
         * not move when the word changes length (RECORDING -> FINISHED etc). */
        int waiting = !s.recording && !s.finished && s.next[0];
        COLORREF lc = s.recording ? COL_LED_ON
                    : s.finished  ? RGB(10, 245, 25)
                    : waiting     ? RGB(200, 160, 0)
                    :               RGB(160, 160, 160);
        const char *st = s.state[0] ? s.state : "IDLE";
        COLORREF stc = s.recording ? COL_LED_ON
                     : s.finished  ? RGB(10, 245, 25)
                     :               COL_TEXT_DIM;
        /* Reserve room for the longest state word so the LED clears the
         * version text on the left. ~110px holds "RECORDING".              */
        int led_x  = cr.right - 14 - 110;   /* fixed LED centre x           */
        int text_x = led_x + 14;            /* fixed text start, right of LED */
        HGDIOBJ of = SelectObject(dc, g_hFontVal);
        TEXTMETRICA tm; GetTextMetricsA(dc, &tm);
        SelectObject(dc, of);
        int mid = baseline - tm.tmAscent + tm.tmHeight / 2;
        draw_led(dc, led_x, mid, 6, lc, 0);
        draw_text_base(dc, text_x, baseline, st, stc, g_hFontVal);
    }

    /* ---- Frequency line (baseline-aligned) ---- */
    {
        int baseline = 60;
        char fb[128];
        snprintf(fb, sizeof(fb), "CF: %s", s.freq[0] ? s.freq : "-");
        int w = draw_text_base(dc, 14, baseline, fb, COL_TEXT, g_hFontVal);

        if (s.span[0]) {
            char sb[112];
            snprintf(sb, sizeof(sb), "Usable Coverage: %s", s.span);
            draw_text_base(dc, 14 + w + 24, baseline, sb,
                           COL_TEXT_DIM, g_hFontUI);
        }

        /* Live clock, right-aligned, small and dim. Follows the same UTC/local
         * choice as the rest of the app (use_utc). Optional via show_clock.   */
        int clock_left = cr.right;
        if (g_clock_show) {
            SYSTEMTIME ct;
            if (g_clock_utc) GetSystemTime(&ct); else GetLocalTime(&ct);
            char cb[40];
            snprintf(cb, sizeof(cb), "%02d:%02d:%02d %s",
                     ct.wHour, ct.wMinute, ct.wSecond,
                     g_clock_utc ? "UTC" : "local");
            HGDIOBJ of = SelectObject(dc, g_hFontUI);
            SIZE csz; GetTextExtentPoint32A(dc, cb, (int)strlen(cb), &csz);
            SelectObject(dc, of);
            clock_left = cr.right - 110; /* aligns with state text left edge */
            draw_text_base(dc, clock_left, baseline, cb,
                           COL_TEXT_DIM, g_hFontUI);
        }

        /* Next scheduled start, placed to the left of the clock. Hourly
         * mode populates this exact same display (g_state.next_start) as
         * the multi-entry schedule does, so without distinguishing them
         * here, an hourly wait and a genuine schedule wait look
         * identical - "(hourly)" only appears when that's actually what's
         * being waited for.                                              */
        if (s.next[0]) {
            char nb[64];
            snprintf(nb, sizeof(nb), "Scheduled%s: %s",
                     g_state.cfg.hourly_enable ? " (hourly)" : "", s.next);
            HGDIOBJ of = SelectObject(dc, g_hFontUI);
            SIZE nsz; GetTextExtentPoint32A(dc, nb, (int)strlen(nb), &nsz);
            SelectObject(dc, of);
            draw_text_base(dc, clock_left - 20 - nsz.cx, baseline, nb,
                           COL_ACCENT, g_hFontUI);
        }
    }

    /* ---- Signal meters panel ---- */
    int panelTop = 74;
    {
        RECT p = { 12, panelTop, cr.right - 12, panelTop + 86 };
        draw_panel(dc, p);

        draw_text(dc, 22, panelTop + 8, "SIGNAL", COL_TEXT_DIM, g_hFontUI);

        /* Single-tuner data always lands in the "A slot" (s.peak_a/
         * s.overload_a) regardless of which physical tuner it actually
         * came from - stream_callback_single() doesn't know or care
         * which one it is. So when single-tuner mode has selected B, the
         * live data needs to be drawn on the row labelled B instead of
         * the row labelled A, rather than relabelling a row - keeps "A"
         * and "B" meaning what they say, with only which one shows a
         * signal changing.                                               */
        int single_b_active = !g_state.cfg.dual_channel &&
                               !strcmp(g_state.cfg.rspduo_single_tuner, "B");

        /* Channel A */
        draw_text(dc, 22, panelTop + 30, "A", COL_TEXT, g_hFontVal);
        RECT ma = { 44, panelTop + 30, cr.right - 120, panelTop + 48 };
        draw_meter(dc, ma,
                   (!single_b_active && (s.recording || s.listening)) ? s.peak_a : -90.0f,
                   !single_b_active && s.overload_a, g_meter_style);
        {
            char db[24];
            if (single_b_active)
                snprintf(db, sizeof(db), "  (unused)");
            else if (s.peak_a <= -90.0f || !(s.recording || s.listening))
                snprintf(db, sizeof(db), "  --- dBFS");
            else
                snprintf(db, sizeof(db), "%+5.1f dBFS", s.peak_a);
            draw_text(dc, cr.right - 110, panelTop + 30, db,
                      (!single_b_active && s.overload_a) ? COL_SEG_RED : COL_TEXT, g_hFontVal);
        }

        /* Channel B - live when either genuinely dual/Master-Slave, or
         * single-tuner mode has selected B (see single_b_active above).  */
        draw_text(dc, 22, panelTop + 56, "B", COL_TEXT, g_hFontVal);
        RECT mb = { 44, panelTop + 56, cr.right - 120, panelTop + 74 };
        draw_meter(dc, mb,
                   single_b_active ? ((s.recording || s.listening) ? s.peak_a : -90.0f)
                                    : (((s.dual || s.master_slave) && (s.recording || s.listening)) ? s.peak_b : -90.0f),
                   s.overload_b, g_meter_style);
        {
            char db[24];
            if (single_b_active) {
                if (s.peak_a <= -90.0f || !(s.recording || s.listening))
                    snprintf(db, sizeof(db), "  --- dBFS");
                else
                    snprintf(db, sizeof(db), "%+5.1f dBFS", s.peak_a);
            } else if (!s.dual && !s.master_slave) {
                snprintf(db, sizeof(db), "  (single)");
            } else if (s.peak_b <= -90.0f || !(s.recording || s.listening)) {
                snprintf(db, sizeof(db), "  --- dBFS");
            } else {
                snprintf(db, sizeof(db), "%+5.1f dBFS", s.peak_b);
            }
            draw_text(dc, cr.right - 110, panelTop + 56, db,
                      s.overload_b ? COL_SEG_RED : COL_TEXT,
                      g_hFontVal);
        }
    }

    /* ---- Counter tiles row ---- */
    int ctrTop = panelTop + 96;
    {
        int gap = 10;
        int tiles = g_state.cfg.show_sun_times ? 6 : 5;
        int totalW = cr.right - 24;
        int tw = (totalW - gap * (tiles - 1)) / tiles;
        int th = 56;
        int x = 12;
        char v[48];

        int eh = (int)s.elapsed_sec / 3600;
        int em = ((int)s.elapsed_sec % 3600) / 60;
        int es = (int)s.elapsed_sec % 60;
        snprintf(v, sizeof(v), "%02d:%02d:%02d", eh, em, es);
        draw_counter(dc, x, ctrTop, tw, th, "ELAPSED", v, COL_ACCENT);
        x += tw + gap;

        if (s.file_mb >= 1024.0)
            snprintf(v, sizeof(v), "%.2f GB", s.file_mb / 1024.0);
        else
            snprintf(v, sizeof(v), "%.0f MB", s.file_mb);
        draw_counter(dc, x, ctrTop, tw, th, "FILE SIZE", v, COL_TEXT);
        x += tw + gap;

        snprintf(v, sizeof(v), "%ld", s.overflows);
        draw_counter(dc, x, ctrTop, tw, th, "OVERFLOWS", v,
                     s.overflows > 0 ? COL_SEG_RED : COL_SEG_GREEN);
        x += tw + gap;

        snprintf(v, sizeof(v), "%lld", s.dropped);
        draw_counter(dc, x, ctrTop, tw, th, "DROPPED", v,
                     s.dropped > 0 ? COL_SEG_AMBER : COL_SEG_GREEN);
        x += tw + gap;

        snprintf(v, sizeof(v), "%.1f%%", s.ring_pct);
        COLORREF rc = s.ring_pct > 80.0f ? COL_SEG_RED
                    : s.ring_pct > 50.0f ? COL_SEG_AMBER : COL_SEG_GREEN;
        draw_counter(dc, x, ctrTop, tw, th, "RING BUFFER", v, rc);
        x += tw + gap;

        if (g_state.cfg.show_sun_times) {
            char sunrise_str[16], sunset_str[16];
            get_sun_times_str(&g_state.cfg, sunrise_str, sizeof(sunrise_str),
                                             sunset_str,  sizeof(sunset_str));
            draw_suntile(dc, x, ctrTop, tw, th, sunrise_str, sunset_str);
        }
    }

    /* ---- Disk line: free space + AGC / HDR / overload indicators ---- */
    int diskY = ctrTop + 64;
    {
        int baseline = diskY + 16;
        char db[80];
        if (s.disk_free_mb >= 1024.0)
            snprintf(db, sizeof(db), "Disk free: %.1f GB", s.disk_free_mb / 1024.0);
        else
            snprintf(db, sizeof(db), "Disk free: %.0f MB", s.disk_free_mb);
        draw_text_base(dc, 14, baseline, db, COL_TEXT_DIM, g_hFontUI);

        /* AGC: ON/OFF (ON green, OFF dim-grey). In HDR mode AGC is not
         * available (fixed gain path), so show N/A in amber to make clear it
         * cannot be toggled.                                                 */
        COLORREF on_col  = RGB(70, 220, 110);
        COLORREF off_col = RGB(120, 140, 165);
        COLORREF na_col  = RGB(255, 190, 40);
        int ix = 220;
        ix += draw_text_base(dc, ix, baseline, "AGC: ", COL_TEXT_DIM, g_hFontUI);
        if (s.hdr_on) {
            ix += draw_text_base(dc, ix, baseline, "N/A", na_col, g_hFontUI);
        } else {
            ix += draw_text_base(dc, ix, baseline, s.agc_on ? "ON" : "OFF",
                                 s.agc_on ? on_col : off_col, g_hFontUI);
        }
        ix += 18;
        ix += draw_text_base(dc, ix, baseline, "HDR: ", COL_TEXT_DIM, g_hFontUI);
        ix += draw_text_base(dc, ix, baseline, s.hdr_on ? "ON" : "OFF",
                             s.hdr_on ? on_col : off_col, g_hFontUI);

        /* Duration indicator — shown while recording or finished.
         * Reads the configured duration from g_state.cfg (safe since it
         * is only written at config-load time, before recording starts). */
        if (s.recording || s.finished) {
            ix += 18;
            char dur_buf[32];
            int ds = g_state.cfg.duration_sec;
            if (ds <= 0) {
                snprintf(dur_buf, sizeof(dur_buf), "DUR: unlimited");
            } else {
                int h = ds / 3600;
                int m = (ds % 3600) / 60;
                int s2 = ds % 60;
                if (h > 0 && m > 0 && s2 == 0)
                    snprintf(dur_buf, sizeof(dur_buf), "DUR: %dh%02dm", h, m);
                else if (h > 0 && m == 0 && s2 == 0)
                    snprintf(dur_buf, sizeof(dur_buf), "DUR: %dh", h);
                else if (h > 0)
                    snprintf(dur_buf, sizeof(dur_buf), "DUR: %dh%02dm%02ds", h, m, s2);
                else if (m > 0 && s2 == 0)
                    snprintf(dur_buf, sizeof(dur_buf), "DUR: %dm", m);
                else if (m > 0)
                    snprintf(dur_buf, sizeof(dur_buf), "DUR: %dm%02ds", m, s2);
                else
                    snprintf(dur_buf, sizeof(dur_buf), "DUR: %ds", s2);
            }
            ix += draw_text_base(dc, ix, baseline, dur_buf, COL_TEXT_DIM, g_hFontUI);
        }

        /* COHERENT indicator: only shown for RSPduo dual-channel recording
         * with both tuners on the same frequency (phase-coherent diversity
         * condition). Same green as the FINISHED state word. Positioned
         * well clear of AGC/HDR so it reads as a distinct, occasional cue
         * rather than competing with the always-on indicators.            */
        if (s.coherent) {
            int cx = 560;
            HGDIOBJ of2 = SelectObject(dc, g_hFontUI);
            TEXTMETRICA tm2; GetTextMetricsA(dc, &tm2);
            SelectObject(dc, of2);
            int mid2 = baseline - tm2.tmAscent + tm2.tmHeight / 2;
            draw_led(dc, cx, mid2, 5, RGB(10, 245, 25), 0);
            draw_text_base(dc, cx + 12, baseline, "COHERENT",
                           RGB(10, 245, 25), g_hFontUI);
        }

        /* MASTER/SLAVE indicator: lit whenever Tuner B is being recorded
         * by the separate hidden slave process rather than the normal
         * Dual_Tuner path (i.e. the two tuners are on different
         * frequencies this session). Amber to read as "different mode",
         * not a straightforward pass/fail cue like the others. */
        if (s.master_slave) {
            int cx = 560;
            HGDIOBJ of3 = SelectObject(dc, g_hFontUI);
            TEXTMETRICA tm3; GetTextMetricsA(dc, &tm3);
            SelectObject(dc, of3);
            int mid3 = baseline - tm3.tmAscent + tm3.tmHeight / 2;
            draw_led(dc, cx, mid3, 5, RGB(255, 176, 32), 0);
            draw_text_base(dc, cx + 12, baseline, "MASTER/SLAVE",
                           RGB(255, 176, 32), g_hFontUI);
        }
    }

    /* Info strip: device, antenna, gain — shown only while recording.
     * Must match layout_children()'s button-row y, or it ends up hidden
     * behind the monitor bar's child controls (or floating in the wrong
     * place when the monitor bar is hidden). When there's no recording
     * in progress to describe, the Timer status (s.sched - "SCHEDULED:
     * ..." / "HOURLY (next): ...") takes the same spot instead, so
     * there's always something here telling you whether an automatic
     * recording is armed, rather than only finding out once it starts.
     * The measured AM carrier frequency takes priority over both when
     * available - this is the "left of Record/Start, replacing the
     * status info" spot Carrier moved into, rather than the frequency
     * line (see the AppMonitor field comments for the measurement
     * method, and CARRIER_LOCK_MIN_AGREE for what "available" requires). */
    {
        int bbh2 = BOTTOM_BTN_ROW_H;
        int monH2 = g_monitor_bar_visible_eff
                  ? (BOTTOM_MON_BAR_H + BOTTOM_MON_ROW_GAP + BOTTOM_MON_BAR2_H)
                  : 0;
        int monY2 = cr.bottom - monH2 - BOTTOM_MON_GAP;
        int bbY2 = g_monitor_bar_visible_eff ? (monY2 - bbh2 - BOTTOM_BTN_GAP)
                                              : (cr.bottom - bbh2 - BOTTOM_MON_GAP);
        /* Device-info text (RSPdx / Ant / GR / LNA / SR) removed here -
         * the Scheduled status still uses this row when armed and
         * waiting (time-exclusive with Carrier below, so no collision). */
        if (s.sched[0])
            draw_text_base(dc, 14, bbY2 + 17, s.sched, COL_ACCENT, g_hFontUI);

        /* Carrier readout - back on the info-strip row, aligned
         * horizontally above the frequency dial (g_hFreqDigits). No
         * longer collides with anything here now that the device-info
         * text is gone, so it moved back up from the gap below (which
         * frees that gap for the divider line to resume always showing).
         * Sized to approximate the dial's own font (18pt), marginally
         * smaller (16pt) - see g_hFontCarrier. Queries the dial's actual
         * current position rather than a hardcoded offset, since that
         * row's layout shifts with window width.                        */
        if (g_monitor.enabled && g_monitor.carrier_offset_valid_pub &&
                g_monitor.carrier_locked_pub && g_hFreqDigits) {
            RECT fr;
            GetWindowRect(g_hFreqDigits, &fr);
            MapWindowPoints(NULL, hwnd, (POINT *)&fr, 2);

            double dial_hz = (g_monitor.tuner_sel == 1) ? g_monitor.freq_hz_b
                                                          : g_monitor.freq_hz;
            double carrier_hz = dial_hz + (double)g_monitor.carrier_offset_hz_pub;
            char cb[32];
            snprintf(cb, sizeof(cb), "%.4f", carrier_hz / 1000.0);

            int cx = fr.left;
            int cy = bbY2 + 20;
            cx += draw_text_base(dc, cx, cy, "OFFSET: ", COL_TEXT_DIM, g_hFontUI);
            cx += draw_text_base(dc, cx, cy, cb, COL_SEG_AMBER, g_hFontCarrier);
            cx += draw_text_base(dc, cx, cy, " kHz", COL_TEXT_DIM, g_hFontUI);
            if (g_monitor.carrier_settled_pub)
                draw_text_base(dc, cx + 6, cy, "(lock)", COL_SEG_GREEN, g_hFontUI);
        }

        /* Low-profile divider between the button row above and the
         * monitor section below - sits at the midpoint of the gap
         * between them, spanning the same left/right margins as the
         * controls in both areas. Always shown now that carrier no
         * longer needs this gap for itself.                             */
        if (g_monitor_bar_visible_eff) {
            int div_y = bbY2 + bbh2 + BOTTOM_BTN_GAP / 2;
            HPEN divPen = CreatePen(PS_SOLID, 1, RGB(120, 120, 120));
            HGDIOBJ divOld = SelectObject(dc, divPen);
            MoveToEx(dc, 14, div_y, NULL);
            LineTo(dc, cr.right - 16, div_y);
            SelectObject(dc, divOld);
            DeleteObject(divPen);
        }
    }

    BitBlt(wdc, 0, 0, cr.right, cr.bottom, dc, 0, 0, SRCCOPY);

    SelectObject(dc, obmp);
    DeleteObject(bmp);
    DeleteDC(dc);
    EndPaint(hwnd, &ps);
}

/* Picks up any pending change to cfg.monitor_bar_visible and applies it -
 * called at app startup and whenever a session ends (WM_APP_DONE), and
 * immediately from the Settings save handler if nothing is running right
 * now. Never called while g_worker_active, by design - see the comment
 * on g_monitor_bar_visible_eff.                                          */
static void gui_refresh_monitor_bar_visibility(void)
{
    int vis = g_state.cfg.monitor_bar_visible ? 1 : 0;
    int cmd = vis ? SW_SHOW : SW_HIDE;
    g_monitor_bar_visible_eff = vis;

    if (g_hBtnMonitor)      ShowWindow(g_hBtnMonitor,      cmd);
    if (g_hBtnFreqLock)     ShowWindow(g_hBtnFreqLock,     cmd);
    if (g_hFreqDigits)      ShowWindow(g_hFreqDigits,      cmd);
    if (g_hMonHzLbl)        ShowWindow(g_hMonHzLbl,        cmd);
    if (g_hMonModeLbl)      ShowWindow(g_hMonModeLbl,      cmd);
    if (g_hMonMode)         ShowWindow(g_hMonMode,         cmd);
    if (g_hBwDigits)        ShowWindow(g_hBwDigits,        cmd);
    if (g_hMonKhzLbl)       ShowWindow(g_hMonKhzLbl,       cmd);
    if (g_hSMeter)          ShowWindow(g_hSMeter,          cmd);
    if (g_hBtnNotchEnable)  ShowWindow(g_hBtnNotchEnable,  cmd);
    if (g_hNotchDigits)     ShowWindow(g_hNotchDigits,     cmd);
    if (g_hNotchKhzLbl)     ShowWindow(g_hNotchKhzLbl,     cmd);
    if (g_hMonVolLbl)       ShowWindow(g_hMonVolLbl,       cmd);
    if (g_hMonVol)          ShowWindow(g_hMonVol,          cmd);
    if (g_hMonVolVal)       ShowWindow(g_hMonVolVal,       cmd);
    if (g_hBtnHpfEnable)    ShowWindow(g_hBtnHpfEnable,    cmd);
    if (g_hHpfSlider)       ShowWindow(g_hHpfSlider,       cmd);
    if (g_hHpfVal)          ShowWindow(g_hHpfVal,          cmd);

    if (g_hwnd) {
        layout_children(g_hwnd);
        InvalidateRect(g_hwnd, NULL, TRUE);
    }
}

/* Position child controls (buttons + log) relative to client size. */
static void layout_children(HWND hwnd)
{
    RECT cr;
    GetClientRect(hwnd, &cr);

    /* Geometry mirrors the painter. */
    int panelTop = 74;
    int ctrTop   = panelTop + 96;
    int diskY    = ctrTop + 64;

    /* Live monitor bar sits along the very bottom edge - unless hidden,
     * in which case it takes up no space at all and the button row below
     * sits flush with the bottom edge instead, using the same margin.   */
    int monH = g_monitor_bar_visible_eff
             ? (BOTTOM_MON_BAR_H + BOTTOM_MON_ROW_GAP + BOTTOM_MON_BAR2_H)
             : 0;
    int monY = cr.bottom - monH - BOTTOM_MON_GAP;

    /* Bottom button bar: Start/Stop and AGC sit directly above the monitor bar
     * (or flush with the bottom edge, if the monitor bar is hidden).       */
    int bbh = BOTTOM_BTN_ROW_H;
    int bbY = g_monitor_bar_visible_eff ? (monY - bbh - BOTTOM_BTN_GAP)
                                         : (cr.bottom - bbh - BOTTOM_MON_GAP);
    int sbw = 90, abw = 64, nw = 110, setw = 76, bgap = 8;
    /* Buttons right-aligned; scheduling text painted to their left. */
    int agc_x = cr.right - 12 - abw;
    int tog_x = agc_x - bgap - sbw;
    int now_x = tog_x - bgap - nw;
    int set_x = now_x - bgap - setw;
    MoveWindow(g_hBtnToggle, tog_x, bbY, sbw, bbh, TRUE);
    MoveWindow(g_hBtnAgc,    agc_x, bbY, abw, bbh, TRUE);
    if (g_hBtnSchedToggle) MoveWindow(g_hBtnSchedToggle, now_x, bbY, nw, bbh, TRUE);
    if (g_hBtnSettings) MoveWindow(g_hBtnSettings, set_x, bbY, setw, bbh, TRUE);

    if (g_monitor_bar_visible_eff)
        monitor_layout(hwnd, cr.right, monY, monH);

    /* Log fills the area between the disk line and the bottom button bar. */
    int logTop    = diskY + 26 + 6;
    int logBottom = bbY - 4;
    MoveWindow(g_hLog, 12, logTop, cr.right - 24, logBottom - logTop, TRUE);
}

static LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp)
{
    switch (msg) {

    case WM_CTLCOLOREDIT:
    case WM_CTLCOLORSTATIC: {
        HDC dc = (HDC)wp;
        SetTextColor(dc, COL_TEXT);
        SetBkColor(dc, COL_BG);
        return (LRESULT)g_hbrBg;
    }

    case WM_DRAWITEM: {
        LPDRAWITEMSTRUCT di = (LPDRAWITEMSTRUCT)lp;
        if (di->CtlType == ODT_BUTTON && di->CtlID == IDC_BTN_MONITOR)
            return monitor_draw_button(di);
        if (di->CtlType == ODT_BUTTON && di->CtlID == IDC_BTN_NOTCH_ENABLE)
            return notch_draw_button(di);
        if (di->CtlType == ODT_BUTTON && di->CtlID == IDC_BTN_HPF_ENABLE)
            return hpf_draw_button(di);
        if (di->CtlType == ODT_BUTTON) {
            int dis = (di->itemState & ODS_DISABLED) != 0;
            int down = (di->itemState & ODS_SELECTED) != 0;
            COLORREF face;
            if (di->CtlID == IDC_BTN_TOGGLE) {
                /* Same pattern as Monitor/AGC/Schedule: plain base colour
                 * when active, brightened slightly only when pressed.
                 * Grey = disabled (e.g. already armed and waiting).      */
                if (dis) {
                    face = COL_BTN_DIS;
                } else {
                    COLORREF base = g_toggle_btn_recording
                                    ? COL_BTN_STOP : COL_BTN_RECORD;
                    if (down) {
                        int r = GetRValue(base) + 25; if (r > 255) r = 255;
                        int g = GetGValue(base) + 25; if (g > 255) g = 255;
                        int bl = GetBValue(base) + 25; if (bl > 255) bl = 255;
                        face = RGB(r, g, bl);
                    } else {
                        face = base;
                    }
                }
            } else if (di->CtlID == IDC_BTN_AGC && !dis && g_ui.agc_on) {
                /* Green when AGC is currently enabled. */
                face = down ? RGB(40, 120, 80) : COL_BTN_START;
            } else if (di->CtlID == IDC_BTN_FREQ_LOCK && !dis && g_monitor.freq_locked) {
                /* Green when Monitor A/B frequencies are locked together. */
                face = down ? RGB(40, 120, 80) : COL_BTN_START;
            } else if (di->CtlID == IDC_BTN_SCHED_TOGGLE &&
                       (g_state.cfg.schedule_only || g_state.cfg.hourly_enable)) {
                /* Green whenever either timer mode is enabled - deliberately not
                 * gated on !dis like the other toggle highlights above.
                 * This button is disabled while a session is running,
                 * which is exactly when an armed schedule is most likely
                 * to actually be waiting on something (see "Scheduled:
                 * HH:MM:SS" at the top of the window) - gating the
                 * highlight on "also enabled" meant the button showed
                 * "Schedule: ON" in text while looking grey/off in that
                 * exact situation, which reads as a contradiction rather
                 * than "on, but you can't toggle it right now".         */
                face = down ? RGB(40, 120, 80) : COL_BTN_START;
            } else {
                face = dis ? COL_BTN_DIS : (down ? COL_BTN_HOT : COL_BTN_FACE);
            }
            HBRUSH b = CreateSolidBrush(face);

            /* Fill the whole item rect with the window background first, so the
             * rounded corners reveal navy, not the default white button face. */
            FillRect(di->hDC, &di->rcItem, g_hbrBg);

            HPEN   p = CreatePen(PS_SOLID, 1, COL_PANEL_EDGE);
            HGDIOBJ ob = SelectObject(di->hDC, b);
            HGDIOBJ op = SelectObject(di->hDC, p);
            RoundRect(di->hDC, di->rcItem.left, di->rcItem.top,
                      di->rcItem.right, di->rcItem.bottom, 5, 5);
            SelectObject(di->hDC, ob);
            SelectObject(di->hDC, op);
            DeleteObject(b);
            DeleteObject(p);

            char txt[32];
            GetWindowTextA(di->hwndItem, txt, sizeof(txt));
            SetBkMode(di->hDC, TRANSPARENT);
            SetTextColor(di->hDC, dis ? COL_TEXT_DIM : COL_TEXT);
            HGDIOBJ of = SelectObject(di->hDC, g_hFontUI);
            DrawTextA(di->hDC, txt, -1, &di->rcItem,
                      DT_CENTER | DT_VCENTER | DT_SINGLELINE);
            SelectObject(di->hDC, of);
            return TRUE;
        }
        return FALSE;
    }

    case WM_ERASEBKGND:
        return 1;   /* handled in WM_PAINT (double-buffered) */

    case WM_PAINT:
        paint_window(hwnd);
        return 0;

    case WM_TIMER:
        if (wp == ID_TIMER_CLOCK) {
            if (g_clock_show) {
                /* Tick the clock once a second. Repaint only the top strip
                 * where the clock lives (cheap; the monitor handles the
                 * rest during a recording, and this keeps the clock smooth
                 * at idle too).                                           */
                RECT cr; GetClientRect(hwnd, &cr);
                RECT top = { 0, 0, cr.right, 74 };
                InvalidateRect(hwnd, &top, FALSE);
            }
            if (!g_worker_active) {
                /* Only meaningful at minute granularity - once a minute is
                 * enough to keep "next: HH:MM" from going stale sitting
                 * idle across an hour boundary, without repainting every
                 * second for text that hasn't actually changed.           */
                SYSTEMTIME st;
                get_timestamp(&st);
                if (st.wSecond == 0) gui_refresh_idle_timer_text();
            }
        }
        return 0;

    case WM_COMMAND:
        switch (LOWORD(wp)) {
        case IDC_BTN_TOGGLE: {
            int genuinely_nothing_scheduled = !(g_state.cfg.schedule_only ||
                                                 g_state.cfg.hourly_enable ||
                                                 g_state.cfg.start_time[0]);
            if (g_worker_active && g_state.listening && !g_toggle_btn_recording &&
                    genuinely_nothing_scheduled) {
                /* Genuinely just listening (Monitor pressed, nothing at all
                 * scheduled) - this press means "start recording now"
                 * rather than "stop". Reuses the same g_record_now signal
                 * a scheduled wait already respects, so the transition
                 * falls through recording_worker's existing wait-then-
                 * record logic with no special case needed there.
                 * gui_set_recording_ui(1) commits this visually too -
                 * immediately shows Stop and marks g_toggle_btn_recording,
                 * the same as a cold Record/Start press already does -
                 * previously this branch only set g_record_now with no
                 * button update at all, so pressing Start here gave no
                 * feedback whatsoever and looked like it had done
                 * nothing, even on the rare case it actually had.        */
                g_record_now = 1;
                gui_set_recording_ui(1);
            } else if (g_worker_active && g_state.listening && !g_toggle_btn_recording) {
                /* Monitor-initiated listening, but a schedule/hourly/
                 * start_time wait is genuinely already active (Timer was
                 * armed before Monitor was pressed) - checking only
                 * g_toggle_btn_recording here, as the branch above used
                 * to do alone, meant this looked identical to "nothing
                 * scheduled" and pressing Start jumped straight into an
                 * immediate ad-hoc recording that bypassed the real wait
                 * entirely. Commit to the existing wait instead - the
                 * same thing a cold Record/Start press already does for
                 * this exact wait - and let it continue governing when
                 * recording actually begins, rather than short-circuiting
                 * it with g_record_now.
                 *
                 * Exception: if the session has already fallen through to
                 * the generic "no schedule at all" loop (g_toggle_btn_
                 * recording having been reset to 0 by an earlier Timer-off
                 * downgrade, then Timer turned back on before Start was
                 * pressed) - there is no real wait left underneath to
                 * notice g_toggle_btn_recording on its own; that loop only
                 * responds to g_record_now. Not setting it there wouldn't
                 * bypass a real wait, it would just leave the button
                 * showing Stop forever with nothing actually happening
                 * until Stop is pressed - worse than the original bug,
                 * not better. So g_record_now is also set in that specific
                 * case, trading a possibly-mistimed ad-hoc recording for a
                 * session that actually progresses instead of one that's
                 * silently stuck.                                         */
                if (g_in_generic_listen_wait) g_record_now = 1;
                gui_set_recording_ui(1);
            } else if (g_worker_active) {
                /* Covers both an actual recording in progress, and Record
                 * having been pressed for an hourly/schedule/start_time
                 * wait that pre-opened the device for Monitor's benefit
                 * (g_toggle_btn_recording is 1 in that case too, even
                 * though g_state.listening is also 1) - the button already
                 * correctly reads "Stop" here, and clicking it should
                 * actually stop, not silently skip ahead into recording.  */
                gui_stop_session(0);
            } else {
                gui_start_session();
            }
            InvalidateRect(hwnd, NULL, FALSE);
            return 0;
        }
        case IDC_BTN_SCHED_TOGGLE: {
            /* Unified Timer button: Schedule and Hourly are two mutually
             * exclusive automatic-recording modes, but previously only
             * Schedule had a main-window control at all - Hourly could
             * only be seen or changed from Settings, with nothing on the
             * main window even hinting it was active. This button now
             * arms/disarms whichever mode is configured, and
             * timer_last_mode remembers which one to re-arm on the next
             * OFF-to-ON press, since turning both off loses that
             * information otherwise (see the Config field comment).      */
            int currently_on = g_state.cfg.schedule_only || g_state.cfg.hourly_enable;
            IniPatchEntry entries[2];
            int n = 0;

            if (currently_on) {
                const char *which = g_state.cfg.schedule_only ? "schedule_only" : "hourly_enable";
                entries[n].key = which;
                entries[n].applied = 0;
                snprintf(entries[n].value, sizeof(entries[n].value), "0");
                n++;
            } else {
                int want_hourly = !strcmp(g_state.cfg.timer_last_mode, "hourly");
                entries[n].key = want_hourly ? "hourly_enable" : "schedule_only";
                entries[n].applied = 0;
                snprintf(entries[n].value, sizeof(entries[n].value), "1");
                n++;
                entries[n].key = "timer_last_mode";
                entries[n].applied = 0;
                snprintf(entries[n].value, sizeof(entries[n].value), "%s",
                         want_hourly ? "hourly" : "schedule");
                n++;
            }

            if (!ini_patch_values("duodx.ini", entries, n)) {
                LOG_ERROR("Timer toggle: could not write duodx.ini "
                          "(error %lu) - not changed.", GetLastError());
                return 0;
            }

            if (currently_on) {
                int was_schedule = g_state.cfg.schedule_only;
                if (was_schedule) g_state.cfg.schedule_only = 0;
                else              g_state.cfg.hourly_enable = 0;
                LOG_INFO("Timer disabled (%s).", was_schedule ? "schedule" : "hourly");
            } else {
                int want_hourly = !strcmp(g_state.cfg.timer_last_mode, "hourly");
                if (want_hourly) g_state.cfg.hourly_enable = 1;
                else             g_state.cfg.schedule_only = 1;
                LOG_INFO("Timer enabled (%s).", want_hourly ? "hourly" : "schedule");
            }
            SetWindowTextA(g_hBtnSchedToggle, currently_on ? "Timer: OFF" : "Timer: ON");
            InvalidateRect(g_hBtnSchedToggle, NULL, TRUE);
            gui_refresh_idle_timer_text();
            if (!g_toggle_btn_recording) {
                SetWindowTextA(g_hBtnToggle, gui_record_btn_idle_label());
                InvalidateRect(g_hBtnToggle, NULL, FALSE);
            }
            if (currently_on && g_worker_active && !g_state.stream_running) {
                /* Currently waiting for a scheduled/hourly time.
                 * currently_on being true already proves this really was
                 * an armed wait (checked above, before disabling it),
                 * regardless of whether Monitor or Record originally
                 * started it. If the device is open for listening - which
                 * it always is during this kind of wait, since it's
                 * pre-opened specifically so Monitor works - cancel just
                 * the scheduled plan and keep listening running, rather
                 * than tearing the whole session down and taking Monitor
                 * with it. Falls back to a full stop only if somehow not
                 * listening at all, which shouldn't normally happen here. */
                if (g_state.listening) {
                    g_downgrade_to_listening = 1;
                } else {
                    gui_stop_session(0);
                }
            } else if (!currently_on && g_worker_active && g_state.listening &&
                       g_in_generic_listen_wait && !g_toggle_btn_recording) {
                /* Turning Timer back ON, but this listening session had
                 * already fallen through to the generic "no schedule"
                 * loop from an earlier Timer-off downgrade (see the
                 * branch above). That loop only ever watches
                 * g_record_now, so simply flipping the ini flag back on
                 * here does nothing on its own - the just re-armed
                 * schedule would silently be ignored and Start would
                 * begin an immediate ad-hoc recording instead of waiting
                 * (see the g_in_generic_listen_wait comment in
                 * IDC_BTN_TOGGLE for that fallback). Restart listening
                 * cleanly instead, so recording_worker re-reads the
                 * config fresh and enters a genuine wait for the
                 * schedule/hourly time, exactly as a brand new Monitor
                 * press already would.                                   */
                gui_stop_session(1);
                if (g_hwnd) PostMessageA(g_hwnd, WM_APP_RESTART_LISTENING, 0, 0);
            }
            return 0;
        }
        case IDC_BTN_SETTINGS:
            open_settings_dialog(hwnd);
            return 0;
        case IDC_BTN_AGC: {
            /* Debounce: ignore presses within 1.5 s of the last one so the
             * API has time to settle. Rapid toggles caused NotInitialised
             * errors because each Update() raced the previous one.          */
            static DWORD last_agc_click = 0;
            DWORD now = GetTickCount();
            if (now - last_agc_click >= 1500) {
                last_agc_click = now;
                if (g_state.listening) {
                    /* g_agc_toggle_req is only serviced in the main
                     * recording loop (Step 10), which listening mode
                     * never reaches - deferring it here would silently
                     * do nothing now and then fire unexpectedly the
                     * moment Record is later pressed. Apply directly
                     * instead, the same way gui_apply_live_gain does for
                     * gain changes while listening.                     */
                    gui_apply_agc_toggle(&g_state);
                } else {
                    g_agc_toggle_req = 1;
                }
            }
            return 0;
        }
        case IDC_BTN_MONITOR:
            if (!g_worker_active) {
                /* Fully idle - Monitor starts the receiver without
                 * recording, so gain and tuning can be adjusted with the
                 * live meters and audio active but no file being written.
                 * Works for dual_channel too now - the worker opens both
                 * tuners' streams simultaneously in that case, same as a
                 * real recording would, so the monitor can switch between
                 * them (right-click) with no restart required.           */
                gui_start_listening();
            } else if (g_state.listening &&
                       !(g_state.cfg.hourly_enable || g_state.cfg.schedule_only ||
                         g_state.cfg.start_time[0])) {
                /* Genuinely just listening - nothing armed at all, this
                 * session only exists because Monitor was pressed with no
                 * schedule/hourly/start_time configured - pressing it
                 * again means stop listening entirely, same as always.
                 * Checking hourly_enable/schedule_only/start_time here
                 * rather than g_toggle_btn_recording deliberately - the
                 * latter only reflects which button was pressed, not
                 * whether an hourly/schedule wait is actually in progress.
                 * recording_worker()'s wait-then-auto-record logic engages
                 * purely on hourly_enable/schedule_only being true, with
                 * no idea whether Monitor or Record started the session -
                 * so pressing Monitor alone while Timer is armed silently
                 * enters that same wait, and needs this button to
                 * recognise that and toggle audio instead of cancelling
                 * the wait it doesn't know is running.                   */
                gui_cancel_listening();
            } else {
                /* Either an actual recording is already running, or the
                 * device was pre-opened for live monitoring during an
                 * hourly/schedule/start_time wait after Record was
                 * pressed (g_toggle_btn_recording is 1 in that case,
                 * since Record - not Monitor - is what's driving this
                 * session). Toggle live audio monitoring on/off either
                 * way, rather than cancelling a recording the user
                 * already committed to by pressing Record - that used to
                 * be reachable here too (g_state.listening is 1 for
                 * exactly this pre-opened wait as well), and pressing
                 * Monitor to hear the wait would silently abort the
                 * whole planned recording instead.                       */
                g_monitor.enabled = !g_monitor.enabled;
                InvalidateRect(g_hBtnMonitor, NULL, TRUE);
            }
            return 0;
        case IDC_BTN_FREQ_LOCK:
            /* Keeps Monitor A/B's frequencies in sync for quick side-by-side
             * antenna/tuner comparisons at the same frequency - only means
             * anything with two genuinely independent, simultaneously-
             * streaming tuners (dual-channel or Master/Slave sessions),
             * and only while Monitor is actually on.                      */
            if (!g_monitor.enabled ||
                    !(g_state.cfg.dual_channel || g_state.master_slave_active) ||
                    fabs(g_state.cfg.frequency_hz - g_state.cfg.freq_b_hz) >= 1.0)
                return 0;
            g_monitor.freq_locked = !g_monitor.freq_locked;
            if (g_monitor.freq_locked) {
                /* Sync immediately rather than waiting for the next tune,
                 * so turning the lock on doesn't leave them mismatched
                 * until one of them happens to move. Clamped to the
                 * target tuner's own coverage for the same reason as the
                 * mouse-wheel case above.                                 */
                EnterCriticalSection(&g_monitor.settings_lock);
                if (g_monitor.tuner_sel == 0)
                    g_monitor.freq_hz_b = monitor_clamp_to_coverage(
                        g_monitor.freq_hz, monitor_center_for_tuner(1));
                else
                    g_monitor.freq_hz = monitor_clamp_to_coverage(
                        g_monitor.freq_hz_b, monitor_center_for_tuner(0));
                LeaveCriticalSection(&g_monitor.settings_lock);
            }
            InvalidateRect(g_hBtnFreqLock, NULL, TRUE);
            return 0;
        case IDC_COMBO_MON_MODE:
            if (HIWORD(wp) == CBN_SELCHANGE) monitor_apply_mode_from_combo();
            return 0;
        case IDC_BTN_NOTCH_ENABLE:
            g_monitor.notch_enabled = !g_monitor.notch_enabled;
            InvalidateRect(g_hBtnNotchEnable, NULL, TRUE);
            return 0;
        case IDC_BTN_HPF_ENABLE: {
            IniPatchEntry e[1];
            g_state.cfg.monitor_hpf_enable = !g_state.cfg.monitor_hpf_enable;
            InvalidateRect(g_hBtnHpfEnable, NULL, TRUE);
            e[0].key = "monitor_hpf_enable";
            snprintf(e[0].value, sizeof(e[0].value), "%d",
                     g_state.cfg.monitor_hpf_enable);
            ini_patch_values("duodx.ini", e, 1);
            return 0;
        }
        }
        return 0;

    case WM_HSCROLL:
        if ((HWND)lp == g_hMonVol) {
            monitor_apply_volume_from_slider();
            return 0;
        }
        if ((HWND)lp == g_hHpfSlider) {
            monitor_apply_hpf_hz_from_slider();
            return 0;
        }
        return 0;

    case WM_CONTEXTMENU:
        if ((HWND)wp == g_hBtnMonitor) {
            if (g_state.cfg.dual_channel || g_state.master_slave_active)
                monitor_toggle_tuner_sel();
            else if (g_last_known_hwVer == SDRPLAY_RSPduo_ID)
                /* Plain single-tuner RSPduo mode - dual_channel/Master-
                 * Slave already have their own switch above; this is the
                 * one previously missing entirely, requiring a trip into
                 * Settings (and a full session restart either way) just
                 * to check the other tuner.                              */
                monitor_switch_single_tuner_live();
            /* Any other device (RSPdx etc.) genuinely has only one tuner -
             * nothing to switch to.                                      */
            return 0;
        }
        return 0;

    case WM_APP_LOG: {
        char *txt = (char *)lp;
        if (txt) {
            COLORREF col;
            switch ((int)wp) {
            case 1:  col = RGB(70, 220, 110); break;   /* green  */
            case 2:  col = RGB(255, 170, 40); break;   /* orange */
            case 3:  col = RGB(255, 80, 70);  break;   /* red    */
            default: col = RGB(195, 205, 220); break;  /* white, slightly dimmed */
            }

            CHARFORMAT2A cf;
            memset(&cf, 0, sizeof(cf));
            cf.cbSize = sizeof(cf);
            cf.dwMask = CFM_COLOR;
            cf.crTextColor = col;

            /* Move caret to end, set colour for the new text, insert it. */
            GETTEXTLENGTHEX gtl = { GTL_NUMCHARS, 1200 };
            LONG len = (LONG)SendMessageA(g_hLog, EM_GETTEXTLENGTHEX,
                                          (WPARAM)&gtl, 0);
            SendMessageA(g_hLog, EM_SETSEL, (WPARAM)len, (LPARAM)len);
            SendMessageA(g_hLog, EM_SETCHARFORMAT, SCF_SELECTION, (LPARAM)&cf);
            SendMessageA(g_hLog, EM_REPLACESEL, FALSE, (LPARAM)txt);
            if (!g_log_freeze) {
                SendMessageA(g_hLog, EM_SCROLL, SB_BOTTOM, 0);
                SendMessageA(g_hLog, EM_SCROLLCARET, 0, 0);
            }
            free(txt);
        }
        return 0;
    }

    case WM_APP_RECORDING_STARTED:
        /* The worker thread just transitioned seamlessly from listening
         * to actually recording (Record pressed, or an armed schedule
         * fired, while listening). It can't touch g_hBtnToggle itself -
         * only the thread that created a window may call Win32 control
         * functions on it - so it posts this and the GUI thread applies
         * the label change here instead.                                 */
        gui_set_recording_ui(1);
        return 0;

    case WM_APP_DOWNGRADED_TO_LISTENING:
        /* The reverse transition - Timer was turned off while waiting for
         * a scheduled/hourly window, cancelling just the wait rather than
         * the whole session, so the device (and Monitor) stay running.
         * Same cross-thread reasoning as WM_APP_RECORDING_STARTED above.  */
        gui_set_listening_ui();
        return 0;

    case WM_APP_TUNER_SWITCH_RESTART:
        /* The old session (on the previous tuner) has now fully stopped -
         * see monitor_switch_single_tuner_live() and tuner_switch_wait_
         * thread(). g_worker_thread may or may not have already been
         * closed by the normal WM_APP_DONE path, depending on message
         * ordering - guard rather than assume, since gui_start_session()/
         * gui_start_listening() below will unconditionally overwrite it,
         * which would leak the handle if it's still open.                */
        if (g_worker_thread) {
            CloseHandle(g_worker_thread);
            g_worker_thread = NULL;
        }
        if (g_tuner_switch_was_record)
            gui_start_session();
        else
            gui_start_listening();
        return 0;

    case WM_APP_RESTART_LISTENING:
        /* Posted right after gui_stop_session(1) blocked until the old
         * worker thread's routine actually returned - its own WM_APP_DONE
         * was therefore already posted before this one, so by normal
         * FIFO message ordering it has already run by the time this is
         * processed. Guard on g_worker_thread anyway rather than assume,
         * same reasoning as WM_APP_TUNER_SWITCH_RESTART just above.      */
        if (g_worker_thread) {
            CloseHandle(g_worker_thread);
            g_worker_thread = NULL;
        }
        gui_start_listening();
        return 0;

    case WM_APP_DONE:
        if ((int)wp != 0) g_log_freeze = 1;
        if (g_worker_thread) {
            WaitForSingleObject(g_worker_thread, 2000);
            CloseHandle(g_worker_thread);
            g_worker_thread = NULL;
        }
        if (g_gui_mon_thread) {
            WaitForSingleObject(g_gui_mon_thread, 2000);
            CloseHandle(g_gui_mon_thread);
            g_gui_mon_thread = NULL;
        }
        /* Pick up any monitor_bar_visible change that arrived while this
         * session was running - see g_monitor_bar_visible_eff.           */
        gui_refresh_monitor_bar_visibility();
        /* Leave the final FINISHED snapshot on screen; do NOT auto-close.
         * Pull the final elapsed time and file size from the frozen values
         * so the display keeps the last recording's length instead of
         * resetting to 00:00 once the monitor thread has stopped.          */
        g_ui.recording = 0;
        /* These are only ever meaningful while a session is actually
         * running - the gui_monitor_thread_func loop stops publishing new
         * values for them the moment the worker exits, so without this
         * they'd keep showing whatever the last live session happened to
         * be (e.g. the MASTER/SLAVE indicator staying lit, or COHERENT
         * never re-arming) until the next session overwrote them.        */
        g_ui.listening     = 0;
        g_ui.master_slave  = 0;
        g_ui.coherent      = 0;
        /* Whatever set this (a real recording ending, or a listening
         * session being cancelled/stopped) - once the worker thread has
         * genuinely finished, the device is gone, so there's nothing left
         * to monitor. Without this the button stayed green permanently
         * after the very first time it was ever pressed, since nothing
         * else ever reset it back to 0.                                   */
        g_monitor.enabled = 0;
        if (g_hBtnMonitor) InvalidateRect(g_hBtnMonitor, NULL, TRUE);
        if (g_state.last_display_elapsed > 0.0)
            g_ui.elapsed_sec = g_state.last_display_elapsed;
        if (g_state.last_display_file_mb >= 0)
            g_ui.file_mb = (double)g_state.last_display_file_mb;
        if (!g_toggle_btn_recording || g_state.samples_received == 0) {
            /* Monitor-only session (no file ever opened, regardless of how
             * much was streamed for listening), or Record was pressed but
             * nothing ever actually streamed (immediate failure) - either
             * way, nothing was genuinely recorded, so this is idle, not
             * finished.                                                    */
            strncpy(g_ui.state, "IDLE", sizeof(g_ui.state) - 1);
        } else if (g_state.session_complete) {
            g_ui.finished = 1;
            strncpy(g_ui.state, "FINISHED", sizeof(g_ui.state) - 1);
        } else {
            strncpy(g_ui.state, "FINISHED", sizeof(g_ui.state) - 1);
        }
        g_ui.next[0] = '\0';   /* clear scheduled time on stop/finish */
        gui_set_recording_ui(0);
        InvalidateRect(hwnd, NULL, FALSE);
        return 0;

    case WM_SIZE:
        layout_children(hwnd);
        InvalidateRect(hwnd, NULL, FALSE);
        return 0;

    case WM_GETMINMAXINFO: {
        MINMAXINFO *mm = (MINMAXINFO *)lp;
        mm->ptMinTrackSize.x = 900;   /* below this the monitor bar (Section
                                       * 13) no longer has room for its
                                       * fixed controls plus a usable
                                       * volume slider - see monitor_layout */
        mm->ptMinTrackSize.y = 480;
        return 0;
    }

    case WM_CLOSE:
        if (g_worker_active) {
            const char *msg;
            if (g_state.listening && !g_toggle_btn_recording)
                msg = "Still listening (not recording). Stop and exit?";
            else if (g_state.listening)
                msg = "A recording is armed and waiting to start "
                      "(hourly/scheduled). Cancel it and exit?";
            else
                msg = "A recording is in progress. Stop and exit?";
            if (MessageBoxA(hwnd, msg, "DuoDX",
                    MB_YESNO | MB_ICONQUESTION) != IDYES)
                return 0;
            gui_stop_session(1);
        }
        /* Remember window size/position for next launch. Uses
         * GetWindowPlacement rather than GetWindowRect so a maximized
         * window still saves its restored (un-maximized) rectangle -
         * otherwise closing while maximized would silently grow the
         * saved size to the full screen every time.                    */
        {
            WINDOWPLACEMENT wp;
            wp.length = sizeof(wp);
            if (GetWindowPlacement(hwnd, &wp)) {
                IniPatchEntry entries[5];
                char buf[5][16];
                int maximized = (wp.showCmd == SW_SHOWMAXIMIZED);
                snprintf(buf[0], sizeof(buf[0]), "%ld", wp.rcNormalPosition.left);
                snprintf(buf[1], sizeof(buf[1]), "%ld", wp.rcNormalPosition.top);
                snprintf(buf[2], sizeof(buf[2]), "%ld",
                         wp.rcNormalPosition.right - wp.rcNormalPosition.left);
                snprintf(buf[3], sizeof(buf[3]), "%ld",
                         wp.rcNormalPosition.bottom - wp.rcNormalPosition.top);
                snprintf(buf[4], sizeof(buf[4]), "%d", maximized);
                entries[0].key = "window_x";          strncpy(entries[0].value, buf[0], sizeof(entries[0].value)-1); entries[0].value[sizeof(entries[0].value)-1]='\0';
                entries[1].key = "window_y";          strncpy(entries[1].value, buf[1], sizeof(entries[1].value)-1); entries[1].value[sizeof(entries[1].value)-1]='\0';
                entries[2].key = "window_w";          strncpy(entries[2].value, buf[2], sizeof(entries[2].value)-1); entries[2].value[sizeof(entries[2].value)-1]='\0';
                entries[3].key = "window_h";          strncpy(entries[3].value, buf[3], sizeof(entries[3].value)-1); entries[3].value[sizeof(entries[3].value)-1]='\0';
                entries[4].key = "window_maximized";  strncpy(entries[4].value, buf[4], sizeof(entries[4].value)-1); entries[4].value[sizeof(entries[4].value)-1]='\0';
                entries[0].applied = entries[1].applied = entries[2].applied
                                    = entries[3].applied = entries[4].applied = 0;
                ini_patch_values("duodx.ini", entries, 5);
            }
        }
        DestroyWindow(hwnd);
        return 0;

    case WM_DESTROY:
        KillTimer(hwnd, ID_TIMER_CLOCK);
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcA(hwnd, msg, wp, lp);
}

static HWND mk_button(HWND parent, int id, const char *text)
{
    HWND c = CreateWindowExA(0, "BUTTON", text,
                WS_CHILD | WS_VISIBLE | BS_OWNERDRAW,
                0, 0, 10, 10, parent, (HMENU)(INT_PTR)id,
                (HINSTANCE)GetWindowLongPtrA(parent, GWLP_HINSTANCE), NULL);
    return c;
}

/* =========================================================================
 * RSPduo Master/Slave mode
 *
 * sdrplay_api_Update(..., sdrplay_api_Tuner_B, sdrplay_api_Update_Tuner_Frf,
 * ...) reports success but does not reliably move Tuner B's actual RF
 * frequency in Dual_Tuner mode on this hardware/driver combination (three
 * different call orderings/timings were tried and all failed identically -
 * confirmed by directly analysing recorded IQ data, not just trusting the
 * API's return code). The one mechanism documented to give a tuner its own
 * independent frequency reliably is RSPduo Master/Slave mode - which the
 * SDRplay API models as two separate application sessions, not one dual
 * session. So Tuner A records normally in this process (Master), and
 * Tuner B is recorded by a second, hidden instance of this same exe
 * (Slave), launched automatically. Both write their own single-channel
 * Linrad file, each with its own correct header.
 * ========================================================================= */

static int cmdline_is_slave_b(const char *cmdline)
{
    return cmdline && strstr(cmdline, "--slave-b") != NULL;
}

static int cmdline_is_listen_only(const char *cmdline)
{
    return cmdline && strstr(cmdline, "--listen-only") != NULL;
}

static void cmdline_get_quoted_value(const char *cmdline, const char *key,
                                      char *out, size_t out_sz)
{
    const char *p = cmdline ? strstr(cmdline, key) : NULL;
    out[0] = '\0';
    if (!p) return;
    p += strlen(key);
    if (*p == '"') {
        const char *end;
        size_t len;
        p++;
        end = strchr(p, '"');
        len = end ? (size_t)(end - p) : strlen(p);
        if (len >= out_sz) len = out_sz - 1;
        memcpy(out, p, len);
        out[len] = '\0';
    }
}

static long cmdline_get_int_value(const char *cmdline, const char *key)
{
    const char *p = cmdline ? strstr(cmdline, key) : NULL;
    if (!p) return 0;
    p += strlen(key);
    return strtol(p, NULL, 10);
}

/* -------------------------------------------------------------------------
 * launch_slave_b_process
 *
 * Starts the hidden slave instance for Tuner B. A named, auto-reset event
 * (keyed by this process's PID) lets Stop signal the slave to end
 * cleanly; a Job Object ties the slave's lifetime to this process so it
 * can never be left running as an orphan if the master exits or crashes.
 * ------------------------------------------------------------------------- */
static int launch_slave_b_process(AppState *state, const char *outfile_b,
                                   int duration_sec, int listen_only)
{
    char exe_path[MAX_PATH];
    char cmdline[MAX_PATH_LEN + 160];
    char event_name[64];
    STARTUPINFOA si;
    PROCESS_INFORMATION pi;
    DWORD my_pid = GetCurrentProcessId();

    if (!GetModuleFileNameA(NULL, exe_path, sizeof(exe_path))) {
        LOG_ERROR("Master/Slave: could not resolve own exe path (error %lu).",
                  GetLastError());
        return 0;
    }

    snprintf(event_name, sizeof(event_name), "Local\\DuoDXSlaveStop_%lu",
             (unsigned long)my_pid);
    state->slave_stop_event = CreateEventA(NULL, TRUE, FALSE, event_name);
    if (!state->slave_stop_event) {
        LOG_ERROR("Master/Slave: could not create stop event (error %lu).",
                  GetLastError());
        return 0;
    }

    if (listen_only)
        snprintf(cmdline, sizeof(cmdline),
                 "\"%s\" --slave-b --listen-only --masterpid=%lu",
                 exe_path, (unsigned long)my_pid);
    else
        snprintf(cmdline, sizeof(cmdline),
                 "\"%s\" --slave-b --outfile=\"%s\" --duration=%d --masterpid=%lu",
                 exe_path, outfile_b, duration_sec, (unsigned long)my_pid);

    memset(&si, 0, sizeof(si));
    si.cb = sizeof(si);
    si.dwFlags = STARTF_USESHOWWINDOW;
    si.wShowWindow = SW_HIDE;
    memset(&pi, 0, sizeof(pi));

    if (!CreateProcessA(exe_path, cmdline, NULL, NULL, FALSE,
                         CREATE_NO_WINDOW | CREATE_SUSPENDED,
                         NULL, NULL, &si, &pi)) {
        LOG_ERROR("Master/Slave: could not start slave process (error %lu).",
                  GetLastError());
        CloseHandle(state->slave_stop_event);
        state->slave_stop_event = NULL;
        return 0;
    }

    state->slave_job = CreateJobObjectA(NULL, NULL);
    if (state->slave_job) {
        JOBOBJECT_EXTENDED_LIMIT_INFORMATION jeli;
        memset(&jeli, 0, sizeof(jeli));
        jeli.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
        SetInformationJobObject(state->slave_job, JobObjectExtendedLimitInformation,
                                 &jeli, sizeof(jeli));
        AssignProcessToJobObject(state->slave_job, pi.hProcess);
    }

    state->slave_process = pi.hProcess;
    state->slave_pid     = pi.dwProcessId;
    ResumeThread(pi.hThread);
    CloseHandle(pi.hThread);

    LOG_INFO("Master/Slave: slave process started (PID %lu) for Tuner B -> %s",
             (unsigned long)state->slave_pid, outfile_b);

    /* Live monitor / B meter reader - connects to the slave's own monitor
     * pipe (named using THIS process's PID, matching what the slave will
     * construct from --masterpid). Safe to fail: recording is unaffected
     * either way, this thread only feeds the optional live monitor.     */
    state->slave_monitor_running = 1;
    state->slave_monitor_thread = CreateThread(NULL, 0,
                                                slave_b_monitor_reader_thread,
                                                state, 0, NULL);
    if (!state->slave_monitor_thread) {
        LOG_WARN("Master/Slave: could not start the Tuner B monitor reader "
                 "thread - Monitor B and the B level meter will be "
                 "unavailable this session (recording is unaffected).");
        state->slave_monitor_running = 0;
    }

    return 1;
}

/* Signals the slave to stop and waits (briefly) for it to exit cleanly,
 * falling back to a hard terminate if it doesn't. Safe to call even if
 * no slave was ever started. */
static void stop_slave_b_process(AppState *state)
{
    /* Signal the slave to start shutting down FIRST, before waiting on
     * the monitor reader thread below - that thread is blocked in a
     * synchronous ReadFile() on the slave's pipe, which only unblocks
     * once the pipe actually breaks, which only happens once the slave
     * process itself starts exiting. Waiting on the reader thread before
     * telling the slave to stop meant that wait was essentially
     * guaranteed to time out every single time, silently orphaning the
     * reader thread (still blocked, still running) well past this
     * function's return - right as a restart (e.g. a Tuner B frequency
     * change) could be about to spin up a brand new slave process and
     * reader thread of its own. Signalling first gives the pipe a real
     * chance to break within the wait below instead of none at all.    */
    if (state->slave_stop_event)
        SetEvent(state->slave_stop_event);

    if (state->slave_monitor_running) {
        state->slave_monitor_running = 0;
        if (state->slave_monitor_thread) {
            WaitForSingleObject(state->slave_monitor_thread, 2000);
            CloseHandle(state->slave_monitor_thread);
            state->slave_monitor_thread = NULL;
        }
    }
    if (state->slave_stop_event) {
        CloseHandle(state->slave_stop_event);
        state->slave_stop_event = NULL;
    }
    if (state->slave_process) {
        if (WaitForSingleObject(state->slave_process, 5000) == WAIT_TIMEOUT) {
            LOG_WARN("Master/Slave: slave process did not exit in time - "
                     "terminating it.");
            TerminateProcess(state->slave_process, 1);
        }
        CloseHandle(state->slave_process);
        state->slave_process = NULL;
    }
    if (state->slave_job) {
        CloseHandle(state->slave_job);
        state->slave_job = NULL;
    }
    state->master_slave_active = 0;
}

/* -------------------------------------------------------------------------
 * slave_b_monitor_reader_thread
 *
 * Connects to the Tuner B slave's monitor pipe (as a client) and
 * continuously reads its live IQ stream, purely for this process's own
 * use: updating the Tuner B level meter and, when the user has Monitor B
 * selected, feeding the live audio DSP chain. Entirely separate from the
 * recording itself, which the slave writes to disk on its own regardless
 * of whether anything is connected to this pipe - a stalled or absent
 * reader here can never affect the recording.
 * ------------------------------------------------------------------------- */
static DWORD WINAPI slave_b_monitor_reader_thread(LPVOID param)
{
    AppState *state = (AppState *)param;
    char pipe_name_mon[64];
    HANDLE pipe = INVALID_HANDLE_VALUE;
    uint8_t buf[8192];
    int16_t di[2048], dq[2048];
    int connect_attempts = 0;

    snprintf(pipe_name_mon, sizeof(pipe_name_mon),
             "\\\\.\\pipe\\DuoDXMonB_%lu", (unsigned long)GetCurrentProcessId());

    while (state->slave_monitor_running && pipe == INVALID_HANDLE_VALUE) {
        pipe = CreateFileA(pipe_name_mon, GENERIC_READ, 0, NULL,
                            OPEN_EXISTING, 0, NULL);
        if (pipe == INVALID_HANDLE_VALUE) {
            connect_attempts++;
            if (connect_attempts > 100) {
                LOG_WARN("Monitor: could not connect to Tuner B's live "
                         "stream after 20s - Monitor B and the B level "
                         "meter will be unavailable this session "
                         "(recording is unaffected).");
                return 0;
            }
            Sleep(200);
        }
    }
    if (!state->slave_monitor_running) {
        if (pipe != INVALID_HANDLE_VALUE) CloseHandle(pipe);
        return 0;
    }
    LOG_OK("Monitor: connected to Tuner B's live stream (%s) - "
           "Monitor B and the B level meter are now live.", pipe_name_mon);

    while (state->slave_monitor_running) {
        DWORD nread = 0;
        BOOL ok = ReadFile(pipe, buf, sizeof(buf), &nread, NULL);
        if (!ok || nread < 4) {
            DWORD gle = GetLastError();
            if (!ok && (gle == ERROR_NO_DATA || gle == ERROR_PIPE_LISTENING)) {
                Sleep(5);
                continue;
            }
            break;   /* slave stopped / pipe broke - normal at session end */
        }

        {
            unsigned int nframes = nread / 4;
            const int16_t *samp = (const int16_t *)buf;
            unsigned int k;
            int peak = 0;

            if (nframes > 2048) nframes = 2048;
            for (k = 0; k < nframes; k++) {
                int vi = samp[k * 2];     if (vi < 0) vi = -vi;
                int vq = samp[k * 2 + 1]; if (vq < 0) vq = -vq;
                if (vi > peak) peak = vi;
                if (vq > peak) peak = vq;
                di[k] = samp[k * 2];
                dq[k] = samp[k * 2 + 1];
            }
            if (peak > 0) {
                float db = 20.0f * log10f((float)peak / 32767.0f);
                if (db > state->peak_dbfs_b)
                    state->peak_dbfs_b = db;
            }

            if (g_monitor.tuner_sel == 1)
                monitor_feed(di, dq, nframes);
        }
    }

    CloseHandle(pipe);
    return 0;
}

/* -------------------------------------------------------------------------
 * run_slave_b_session - the hidden slave process's entire lifetime.
 * Loads the same duodx.ini as the master for shared settings, resolves
 * Tuner B's own values (falling back to Tuner A's - identical resolution
 * rule to the old Dual_Tuner setup), then records a single-channel Linrad
 * file until the duration elapses, the master signals stop, or the master
 * process itself disappears.
 * ------------------------------------------------------------------------- */
static int run_slave_b_session(const char *outfile, int duration_sec,
                                DWORD master_pid, int listen_only)
{
    sdrplay_api_ErrT err;
    sdrplay_api_DeviceT devices[6];
    unsigned int num_devices = 0, di;
    int rc = 1;
    HANDLE master_handle = NULL;
    char event_name[64];
    sdrplay_api_CallbackFnsT callbacks;
    int init_attempts;
    char device_name[64] = "RSP?";
    SIZE_T ring_size;

    memset(&g_state, 0, sizeof(g_state));
    g_state.out_file    = INVALID_HANDLE_VALUE;
    g_state.out_file_b  = INVALID_HANDLE_VALUE;
    g_state.pipe_handle = INVALID_HANDLE_VALUE;

    config_set_defaults(&g_state.cfg);
    config_load_ini(&g_state.cfg, "duodx.ini");

    g_state.log_fp = fopen("duodx_slave_b.log", "a");
    LOG_INFO("=== DuoDX Slave (Tuner B) starting, master PID %lu ===",
             (unsigned long)master_pid);

    /* Resolve Tuner B's effective settings - same fallback-to-A rule the
     * old Dual_Tuner setup used - then make this session look, to every
     * other part of the engine, like an ordinary single-tuner recording. */
    {
        int gr_b  = (g_state.cfg.gain_reduction_b >= 0) ? g_state.cfg.gain_reduction_b : g_state.cfg.gain_reduction;
        int lna_b = (g_state.cfg.lna_state_b      >= 0) ? g_state.cfg.lna_state_b      : g_state.cfg.lna_state;
        int agc_b = (g_state.cfg.agc_enable_b     >= 0) ? g_state.cfg.agc_enable_b     : g_state.cfg.agc_enable;
        int dc_b  = (g_state.cfg.dc_correct_b     >= 0) ? g_state.cfg.dc_correct_b     : g_state.cfg.dc_correct;
        int iq_b  = (g_state.cfg.iq_correct_b     >= 0) ? g_state.cfg.iq_correct_b     : g_state.cfg.iq_correct;
        if (g_state.cfg.notch_rf_b  >= 0) g_state.cfg.notch_rf  = g_state.cfg.notch_rf_b;
        if (g_state.cfg.notch_dab_b >= 0) g_state.cfg.notch_dab = g_state.cfg.notch_dab_b;

        g_state.cfg.frequency_hz   = g_state.cfg.freq_b_hz;
        g_state.cfg.gain_reduction = gr_b;
        g_state.cfg.lna_state      = lna_b;
        g_state.cfg.agc_enable     = agc_b;
        g_state.cfg.dc_correct     = dc_b;
        g_state.cfg.iq_correct     = iq_b;
    }
    g_state.cfg.dual_channel = 0;
    strncpy(g_state.cfg.output_file, outfile, MAX_PATH_LEN - 1);
    g_state.cfg.output_file[MAX_PATH_LEN - 1] = '\0';

    if (!validate_config(&g_state.cfg)) {
        LOG_ERROR("Slave: config validation failed - aborting.");
        goto slave_done;
    }

    snprintf(event_name, sizeof(event_name), "Local\\DuoDXSlaveStop_%lu",
             (unsigned long)master_pid);
    g_state.slave_stop_event = OpenEventA(SYNCHRONIZE, FALSE, event_name);
    if (!g_state.slave_stop_event)
        LOG_WARN("Slave: could not open master's stop event (error %lu) - "
                 "will only stop on duration or master exit.", GetLastError());

    master_handle = OpenProcess(SYNCHRONIZE, FALSE, master_pid);
    if (!master_handle)
        LOG_WARN("Slave: could not open master process (error %lu) - "
                 "will not detect the master exiting early.", GetLastError());

    err = sdrplay_api_Open();
    if (err != sdrplay_api_Success) {
        LOG_ERROR("Slave: sdrplay_api_Open: %s", sdrplay_api_GetErrorString(err));
        goto slave_done;
    }

    err = sdrplay_api_LockDeviceApi();
    if (err != sdrplay_api_Success) {
        LOG_ERROR("Slave: LockDeviceApi: %s", sdrplay_api_GetErrorString(err));
        goto slave_close_api;
    }
    err = sdrplay_api_GetDevices(devices, &num_devices, 6);
    sdrplay_api_UnlockDeviceApi();
    if (err != sdrplay_api_Success || num_devices == 0) {
        LOG_ERROR("Slave: no SDRplay devices found (%s).",
                  sdrplay_api_GetErrorString(err));
        goto slave_close_api;
    }

    {
        unsigned int selected = 0;
        int found = 0;
        for (di = 0; di < num_devices; di++) {
            if (devices[di].hwVer == SDRPLAY_RSPduo_ID &&
                    (!g_state.cfg.device_serial[0] ||
                     strncmp(devices[di].SerNo, g_state.cfg.device_serial, 63) == 0)) {
                selected = di;
                found = 1;
                break;
            }
        }
        if (!found) {
            LOG_ERROR("Slave: no RSPduo found among %u device(s).", num_devices);
            goto slave_close_api;
        }
        g_state.device = devices[selected];
        g_last_known_hwVer = g_state.device.hwVer;
    }

    if (g_state.device.hwVer == SDRPLAY_RSPduo_ID)
        strncpy(device_name, "RSPduo", 63);
    LOG_INFO("Slave: using device %s (SerNo %s) as Tuner B / Slave",
             device_name, g_state.device.SerNo);

    g_state.device.tuner            = sdrplay_api_Tuner_B;
    g_state.device.rspDuoMode       = sdrplay_api_RspDuoMode_Slave;
    g_state.device.rspDuoSampleFreq = g_state.cfg.sample_rate_hz;

    err = sdrplay_api_SelectDevice(&g_state.device);
    if (err != sdrplay_api_Success) {
        LOG_ERROR("Slave: SelectDevice failed: %s", sdrplay_api_GetErrorString(err));
        goto slave_close_api;
    }

    err = sdrplay_api_GetDeviceParams(g_state.device.dev, &g_state.dev_params);
    if (err != sdrplay_api_Success) {
        LOG_ERROR("Slave: GetDeviceParams failed: %s", sdrplay_api_GetErrorString(err));
        goto slave_release_device;
    }
    /* Channel structs are keyed by physical tuner, not by Master/Slave
     * role - since this session requested Tuner B, its params come back
     * in rxChannelB (rxChannelA is NULL here, confirmed by direct log
     * evidence: a Slave/Tuner_B session leaves rxChannelA unpopulated).
     * ch_a_params is reused purely as a generic "the channel struct we're
     * configuring" pointer - setup_device_single() doesn't care which
     * physical tuner it represents.                                     */
    g_state.ch_a_params = g_state.dev_params->rxChannelB;
    if (!g_state.ch_a_params) {
        LOG_ERROR("Slave: rxChannelB (Tuner B's channel) is NULL - "
                  "GetDeviceParams did not return it for this session.");
        goto slave_release_device;
    }

    setup_slave_channel_b(&g_state);
    QueryPerformanceFrequency(&g_state.perf_freq);

    ring_size = (SIZE_T)(g_state.cfg.sample_rate_hz) * 4
                * (SIZE_T)g_state.cfg.ring_buffer_sec;
    if (ring_size < RING_BUFFER_MIN_BYTES)
        ring_size = RING_BUFFER_MIN_BYTES;
    if (ring_init(&g_state.ring, ring_size) != 0) {
        LOG_ERROR("Slave: ring buffer allocation failed.");
        goto slave_release_device;
    }

    if (!listen_only) {
        g_state.out_file = CreateFileA(g_state.cfg.output_file, GENERIC_WRITE, 0,
                                        NULL, CREATE_ALWAYS,
                                        FILE_ATTRIBUTE_NORMAL | FILE_FLAG_SEQUENTIAL_SCAN,
                                        NULL);
        if (g_state.out_file == INVALID_HANDLE_VALUE) {
            LOG_ERROR("Slave: cannot open output file '%s' (error %lu).",
                      g_state.cfg.output_file, GetLastError());
            goto slave_free_ring;
        }
        if (g_state.cfg.output_format == FORMAT_SDRUNO ||
                ((g_state.cfg.output_format == FORMAT_WINRAD ||
                  g_state.cfg.output_format == FORMAT_SDRCONNECT) &&
                 g_state.cfg.large_file_mode == LARGE_FILE_SPLIT)) {
            g_state.output_part_number = 1;
        }
        g_state.segment_samples_written = 0;

        if (g_state.cfg.output_format == FORMAT_SDRUNO) {
            /* SDRuno never gets RF64 - see the OutputFormat enum comment. */
            if (!write_sdruno_header(g_state.out_file, &g_state.cfg)) {
                LOG_ERROR("Slave: header write failed.");
                goto slave_close_file;
            }
        } else if (g_state.cfg.output_format == FORMAT_WINRAD) {
            int hdr_ok = (g_state.cfg.large_file_mode == LARGE_FILE_RF64)
                         ? write_sdruno_header_rf64(g_state.out_file, &g_state.cfg)
                         : write_sdruno_header(g_state.out_file, &g_state.cfg);
            if (!hdr_ok) {
                LOG_ERROR("Slave: header write failed.");
                goto slave_close_file;
            }
        } else if (g_state.cfg.output_format == FORMAT_SDRCONNECT) {
            int hdr_ok = (g_state.cfg.large_file_mode == LARGE_FILE_RF64)
                         ? write_sdrconnect_header_rf64(g_state.out_file, &g_state.cfg)
                         : write_sdrconnect_header(g_state.out_file, &g_state.cfg);
            if (!hdr_ok) {
                LOG_ERROR("Slave: header write failed.");
                goto slave_close_file;
            }
        } else if (g_state.cfg.output_format == FORMAT_LINRAD) {
            if (!write_linrad_header(g_state.out_file, &g_state.cfg, 1, 0.0)) {
                LOG_ERROR("Slave: header write failed.");
                goto slave_close_file;
            }
        }
        /* FORMAT_WAVVIEWDX has no header at all - nothing to write. */
        FlushFileBuffers(g_state.out_file);
        LOG_INFO("Slave: recording to %s (%.4f MHz)",
                 g_state.cfg.output_file, g_state.cfg.frequency_hz / 1e6);
    } else {
        LOG_INFO("Slave: listen-only, no file - streaming Tuner B live to "
                 "the master (%.4f MHz)", g_state.cfg.frequency_hz / 1e6);
    }

    /* Dedicated named pipe so the master process can monitor Tuner B live
     * (audio + level meter) - Tuner B's real IQ data only exists in this
     * process, so this is the only way the master can see it at all. This
     * reuses the writer thread's existing, already-working pipe-write
     * path (the same one used for the user-facing external IQ monitoring
     * feature) - no new code needed there, just point pipe_handle at it. */
    {
        char pipe_name_mon[64];
        snprintf(pipe_name_mon, sizeof(pipe_name_mon),
                 "\\\\.\\pipe\\DuoDXMonB_%lu", (unsigned long)master_pid);
        /* PIPE_NOWAIT only, no FILE_FLAG_OVERLAPPED - see the identical
         * comment on the general monitor pipe above for why: this handle
         * is written with a NULL OVERLAPPED throughout, which is only
         * well-defined (all-or-nothing per write) without the overlapped
         * flag. Root cause of the Tuner B monitor's pulsing/glitching. */
        g_state.pipe_handle = CreateNamedPipeA(
            pipe_name_mon,
            PIPE_ACCESS_OUTBOUND,
            PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_NOWAIT,
            1, 256 * 1024, 0, 0, NULL);
        if (g_state.pipe_handle == INVALID_HANDLE_VALUE) {
            LOG_WARN("Slave: could not create monitor pipe '%s' (error %lu) "
                     "- live monitoring of Tuner B will be unavailable this "
                     "session (recording is unaffected).",
                     pipe_name_mon, GetLastError());
        } else {
            /* Puts the pipe into a listening state so the master's
             * CreateFile can actually attach - see the identical comment
             * on the general monitor pipe above. Without this, whether
             * the master's connect attempt succeeds at all was down to
             * unspecified/leftover pipe state rather than anything this
             * code actually arranged - the intermittent "could not
             * connect after 20s" was this, not a slow master.           */
            if (!ConnectNamedPipe(g_state.pipe_handle, NULL) &&
                    GetLastError() != ERROR_PIPE_LISTENING &&
                    GetLastError() != ERROR_PIPE_CONNECTED)
                LOG_WARN("Slave: monitor pipe '%s': ConnectNamedPipe error "
                         "%lu - the master may not be able to attach.",
                         pipe_name_mon, GetLastError());
            LOG_INFO("Slave: monitor pipe ready: %s", pipe_name_mon);
        }
    }

    g_state.writer_ready_event = CreateEventA(NULL, TRUE, FALSE, NULL);
    g_state.writer_running = 1;
    g_state.writer_thread = CreateThread(NULL, 0, writer_thread_func,
                                          &g_state, 0, NULL);
    if (!g_state.writer_thread) {
        LOG_ERROR("Slave: could not start writer thread.");
        goto slave_close_file;
    }
    WaitForSingleObject(g_state.writer_ready_event, 5000);

    memset(&callbacks, 0, sizeof(callbacks));
    callbacks.StreamACbFn = stream_callback_single;
    callbacks.EventCbFn   = event_callback;

    init_attempts = 0;
    for (;;) {
        err = sdrplay_api_Init(g_state.device.dev, &callbacks, &g_state);
        if (err == sdrplay_api_Success)
            break;
        if (err == sdrplay_api_StartPending) {
            init_attempts++;
            if (init_attempts == 1 || init_attempts % 10 == 0)
                LOG_INFO("Slave: waiting for master to start (attempt %d)...",
                         init_attempts);
            if (init_attempts > 150) {
                LOG_ERROR("Slave: master never became ready - giving up.");
                goto slave_stop_writer;
            }
            Sleep(200);
            continue;
        }
        LOG_ERROR("Slave: sdrplay_api_Init failed: %s", sdrplay_api_GetErrorString(err));
        goto slave_stop_writer;
    }

    g_state.stream_running = 1;
    QueryPerformanceCounter(&g_state.start_time);
    LOG_OK("Slave: streaming started.");

    apply_slave_biast_b(&g_state);
    apply_notch_filters(&g_state);

    {
        HANDLE waits[2];
        int n_waits = 0;
        DWORD start_tick = GetTickCount();
        if (g_state.slave_stop_event) waits[n_waits++] = g_state.slave_stop_event;
        if (master_handle)            waits[n_waits++] = master_handle;

        for (;;) {
            DWORD elapsed_ms = GetTickCount() - start_tick;
            if (duration_sec > 0 && elapsed_ms >= (DWORD)duration_sec * 1000)
                break;
            if (g_state.writer_error) {
                LOG_ERROR("Slave: writer error - stopping.");
                break;
            }
            if (n_waits > 0) {
                DWORD wr = WaitForMultipleObjects((DWORD)n_waits, waits, FALSE, 250);
                if (wr == WAIT_OBJECT_0 || wr == WAIT_OBJECT_0 + 1) {
                    LOG_INFO("Slave: stop signalled (%s).",
                             wr == WAIT_OBJECT_0 ? "stop event" : "master exited");
                    break;
                }
            } else {
                Sleep(250);
            }
        }
    }

    LOG_INFO("Slave: stopping stream.");
    sdrplay_api_Uninit(g_state.device.dev);
    g_state.stream_running = 0;

slave_stop_writer:
    g_state.writer_running = 0;
    if (g_state.writer_thread) {
        WaitForSingleObject(g_state.writer_thread, 10000);
        CloseHandle(g_state.writer_thread);
    }
    if (g_state.writer_ready_event)
        CloseHandle(g_state.writer_ready_event);

    if (g_state.out_file != INVALID_HANDLE_VALUE) {
        /* segment_samples_written: see comment at the other close sites -
         * matches samples_written unless a split occurred.               */
        if (g_state.segment_samples_written > 0 &&
                (g_state.cfg.output_format == FORMAT_SDRUNO ||
                 g_state.cfg.output_format == FORMAT_WINRAD ||
                 g_state.cfg.output_format == FORMAT_SDRCONNECT))
            finalize_output_header(g_state.out_file, &g_state.cfg, g_state.segment_samples_written);
        CloseHandle(g_state.out_file);
        g_state.out_file = INVALID_HANDLE_VALUE;
    }
    if (g_state.segment_samples_written > 0)
        verify_recording(&g_state.cfg, g_state.segment_samples_written);
    rc = 0;

slave_close_file:
    if (g_state.pipe_handle != INVALID_HANDLE_VALUE) {
        CloseHandle(g_state.pipe_handle);
        g_state.pipe_handle = INVALID_HANDLE_VALUE;
    }
    if (g_state.out_file != INVALID_HANDLE_VALUE) {
        CloseHandle(g_state.out_file);
        g_state.out_file = INVALID_HANDLE_VALUE;
    }
slave_free_ring:
    ring_free(&g_state.ring);
slave_release_device:
    sdrplay_api_ReleaseDevice(&g_state.device);
slave_close_api:
    sdrplay_api_Close();
slave_done:
    LOG_INFO("=== DuoDX Slave (Tuner B) exiting (rc=%d) ===", rc);
    if (g_state.slave_stop_event) CloseHandle(g_state.slave_stop_event);
    if (master_handle) CloseHandle(master_handle);
    if (g_state.log_fp) fclose(g_state.log_fp);
    return rc;
}

/* =========================================================================
 * Settings dialog - GUI editor for the most commonly changed duodx.ini
 * values (centre frequencies, gain/LNA, sample rate/IF/bandwidth, dual
 * channel, AGC, duration, antenna, output format, recording path).
 *
 * Saving patches the INI file in place - only the lines for values
 * actually shown here are touched; every comment, blank line, unrelated
 * setting, and schedule entry is left exactly as it was. A key missing
 * from the file entirely is appended under a clearly marked header rather
 * than silently dropped. Writes go via a temp file + atomic rename so a
 * failure partway through can never corrupt the original file.
 *
 * Everything not listed above (scheduling, notch filters, HTTP server,
 * disk spin-up, etc.) stays INI-only for now - a natural follow-up once
 * this first pass proves out.
 * ========================================================================= */
static HWND   g_hSettingsWnd  = NULL;
static HWND   g_hSetRateCombo = NULL;
static HWND   g_hSetRangeStart = NULL;
static HWND   g_hSetRangeEnd   = NULL;
static HWND   g_hBtnRangeCalc  = NULL;
static HWND   g_hSetRangeHint  = NULL;
static HWND   g_hSetAgc       = NULL;
static HWND   g_hSetTuner1En  = NULL;
static HWND   g_hSetTuner2En  = NULL;
static HWND   g_hSetCoherentLbl = NULL;
static HWND   g_hSetFreqB     = NULL;
static HWND   g_hSetGrB       = NULL;
static HWND   g_hSetGrBSame   = NULL;
static HWND   g_hSetLnaB      = NULL;
static HWND   g_hSetLnaBSame  = NULL;
static HWND   g_hSetDuration  = NULL;
static HWND   g_hSetFormat    = NULL;
static HWND   g_hSetLargeModeLbl = NULL;
static HWND   g_hSetLargeMode    = NULL;
static HWND   g_hSetLatitude     = NULL;
static HWND   g_hSetLongitude    = NULL;
static HWND   g_hSetShowSun      = NULL;
static HWND   g_hSetPath      = NULL;
static HWND   g_hSetHdr       = NULL;
static HWND   g_hSetHdrHint   = NULL;
static HWND   g_hSetHourlyHint = NULL;
static HWND   g_hSetMonVisible = NULL;
static HWND   g_hSetDecim     = NULL;
static HWND   g_hSetTabBtn[7] = { NULL, NULL, NULL, NULL, NULL, NULL, NULL };
static HWND   g_hSetColorScheme = NULL;
static HWND   g_hSetVerbose     = NULL;
static HWND   g_hSetLogAutoSave = NULL;
static int    g_settings_active_tab = 0;

/* -- Schedule tab controls -------------------------------------------- */
static HWND g_hSetSchedOnly    = NULL;
static HWND g_hSetSchedPrev    = NULL;
static HWND g_hSetSchedNext    = NULL;
static HWND g_hSetSchedAdd     = NULL;
static HWND g_hSetSchedDel     = NULL;
static HWND g_hSetSchedIdxLbl  = NULL;
static HWND g_hSetSchedStart   = NULL;
static HWND g_hSetSchedDuration = NULL;
static HWND g_hSetSchedFreq    = NULL;
static HWND g_hSetSchedFreqB   = NULL;
static HWND g_hSetSchedAntenna = NULL;
static HWND g_hSetSchedOutfile = NULL;
static HWND g_hSetHourlyEn     = NULL;
static HWND g_hSetHourlyWin    = NULL;
static HWND g_hSetHourlyStart  = NULL;
static HWND g_hSetHourlyStop   = NULL;
static HWND g_hSetDualT1Freq    = NULL;
static HWND g_hSetDualT1Gr      = NULL;
static HWND g_hSetDualT1Lna     = NULL;
static HWND g_hSetDualT1Antenna = NULL;

/* In-memory copy of the schedule entries being edited, separate from
 * g_settings_cfg.schedule[] until Save - the entry navigator (Prev/Next/
 * Add/Delete) mutates this freely while the dialog is open, exactly like
 * every other Settings field already does via g_settings_cfg, so Cancel
 * still discards everything cleanly by just not writing it back.        */
static ScheduleEntry g_settings_schedule[MAX_SCHEDULE_ENTRIES];
static int            g_settings_schedule_count = 0;
static int            g_settings_schedule_idx   = 0;   /* currently shown entry, 0-based */
static HWND   g_hSetPpm       = NULL;
static HWND   g_hSetDc        = NULL;
static HWND   g_hSetIq        = NULL;
static HWND   g_hSetNotchRf   = NULL;
static HWND   g_hSetNotchDab  = NULL;
static HWND   g_hSetBiasT     = NULL;
static HWND   g_hSetHiz       = NULL;
static HWND   g_hSetHdrBw     = NULL;
static HWND   g_hSetBSameCorr = NULL;
static HWND   g_hSetAgcB      = NULL;
static HWND   g_hSetDcB       = NULL;
static HWND   g_hSetIqB       = NULL;
static HWND   g_hSetNotchRfB  = NULL;
static HWND   g_hSetNotchDabB = NULL;
static HWND   g_hSetRingSec   = NULL;
static HWND   g_hSetSpinupEn  = NULL;
static HWND   g_hSetSpinupBytes = NULL;
static HWND   g_hSetMonInterval = NULL;
static HWND   g_hSetSMeterMode  = NULL;
static HWND   g_hSetSMeterCal   = NULL;
static HWND   g_hSetUseUtc    = NULL;
static HWND   g_hSetShowClock = NULL;
static HWND   g_hSetMeterStyle = NULL;
static HWND   g_hSetHttpPort  = NULL;
static HWND   g_hSetHttpInterval = NULL;
static HWND   g_hSetPipeEn    = NULL;
static HWND   g_hSetPipeName  = NULL;

/* Which tab (0=Receiver, 1=Recording, 2=Monitor, 3=Network) newly-created
 * controls belong to, while the dialog is being built. settings_mk_label/
 * mk_edit/mk_check tag every control they create with this value so
 * settings_select_tab() can show/hide a whole page at once without a
 * hand-maintained list of which HWND belongs to which tab. Controls
 * created outside those three helpers (combo boxes, the HPF slider, the
 * Save/Cancel buttons, the tab control itself) are tagged explicitly with
 * settings_tag_tab() right after creation - Save/Cancel and the tab
 * control are deliberately left untagged, which settings_select_tab()
 * treats as "always visible". */
static int    g_settings_build_tab = 0;
#define SETTINGS_TAB_PROP "DuoDXTab"

static void settings_tag_tab(HWND h)
{
    SetPropA(h, SETTINGS_TAB_PROP, (HANDLE)(INT_PTR)(g_settings_build_tab + 1));
}
static int    g_settings_hdr_freq_valid = 1;
static Config g_settings_cfg;

/* Patches key=value pairs into an INI file in place - see the block
 * comment above for the exact guarantees. Returns 1 on success, 0 on
 * failure (original file untouched on failure). */
static int ini_patch_values(const char *path, IniPatchEntry *entries, int n_entries)
{
    FILE *fp = fopen(path, "r");
    char **lines = NULL;
    int n_lines = 0, cap_lines = 0;
    char linebuf[1024];
    char tmp_path[MAX_PATH_LEN];
    FILE *out;
    int i, ok = 1;

    if (!fp) return 0;

    while (fgets(linebuf, sizeof(linebuf), fp)) {
        size_t len = strlen(linebuf);
        char *copy = (char *)malloc(len + 1);
        if (!copy) { ok = 0; break; }
        memcpy(copy, linebuf, len + 1);
        if (n_lines >= cap_lines) {
            int newcap = cap_lines ? cap_lines * 2 : 256;
            char **nl = (char **)realloc(lines, (size_t)newcap * sizeof(char *));
            if (!nl) { free(copy); ok = 0; break; }
            lines = nl;
            cap_lines = newcap;
        }
        lines[n_lines++] = copy;
    }
    fclose(fp);
    if (!ok) goto fail_free;

    for (i = 0; i < n_lines; i++) {
        char *p = lines[i];
        char *eq;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == ';' || *p == '#' || *p == '\n' || *p == '\0') continue;

        eq = strchr(p, '=');
        if (!eq) continue;
        {
            char keybuf[64];
            size_t klen = (size_t)(eq - p);
            char *kend;
            int j;
            if (klen >= sizeof(keybuf)) continue;
            memcpy(keybuf, p, klen);
            keybuf[klen] = '\0';
            kend = keybuf + strlen(keybuf);
            while (kend > keybuf && (kend[-1] == ' ' || kend[-1] == '\t')) *--kend = '\0';

            for (j = 0; j < n_entries; j++) {
                char *val_start, *comment, *scan;
                char newline_buf[1200];

                if (entries[j].applied) continue;
                if (strcmp(keybuf, entries[j].key) != 0) continue;

                val_start = eq + 1;
                comment = NULL;
                for (scan = val_start; *scan; scan++) {
                    if (*scan == ';' || *scan == '#') { comment = scan; break; }
                }
                if (comment)
                    snprintf(newline_buf, sizeof(newline_buf), "%s = %s  %s",
                             keybuf, entries[j].value, comment);
                else
                    snprintf(newline_buf, sizeof(newline_buf), "%s = %s\n",
                             keybuf, entries[j].value);

                free(lines[i]);
                lines[i] = (char *)malloc(strlen(newline_buf) + 1);
                if (!lines[i]) { ok = 0; goto fail_free; }
                memcpy(lines[i], newline_buf, strlen(newline_buf) + 1);
                entries[j].applied = 1;
                break;
            }
        }
    }

    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);
    out = fopen(tmp_path, "w");
    if (!out) goto fail_free;

    for (i = 0; i < n_lines; i++)
        fputs(lines[i], out);

    {
        int any_unapplied = 0;
        for (i = 0; i < n_entries; i++)
            if (!entries[i].applied) any_unapplied = 1;
        if (any_unapplied) {
            fprintf(out, "\n; --- Added by DuoDX Settings dialog ---\n");
            for (i = 0; i < n_entries; i++) {
                if (!entries[i].applied) {
                    fprintf(out, "%s = %s\n", entries[i].key, entries[i].value);
                    entries[i].applied = 1;
                }
            }
        }
    }
    fclose(out);

    for (i = 0; i < n_lines; i++) free(lines[i]);
    free(lines);

    if (!MoveFileExA(tmp_path, path, MOVEFILE_REPLACE_EXISTING)) {
        DeleteFileA(tmp_path);
        return 0;
    }
    return 1;

fail_free:
    for (i = 0; i < n_lines; i++) free(lines[i]);
    free(lines);
    return 0;
}

/* True for schedule_<N>_<field> keys specifically (indexed entries),
 * false for schedule_only (a plain scalar key handled separately via
 * ini_patch_values) - distinguished by whether the character right
 * after "schedule_" is a digit.                                         */
static int is_schedule_entry_key(const char *key)
{
    if (strncmp(key, "schedule_", 9) != 0) return 0;
    return isdigit((unsigned char)key[9]) != 0;
}

/* Rewrites the whole multi-entry schedule section: every existing
 * schedule_<N>_<field> line is stripped, regardless of what N was, and a
 * fresh block for entries[0..count-1] is appended at the end (matching
 * where the shipped duodx.ini template already keeps them). Needed
 * because ini_patch_values() only patches or appends fixed keys - it has
 * no way to shrink a variable-length list, so deleting an entry in the
 * dialog would otherwise leave its old schedule_N_* lines behind,
 * silently reappearing on the next load (config_load_ini() derives
 * schedule_count from the highest N it finds a key for, regardless of
 * that key's value - see the parsing code). Comments and every other
 * setting elsewhere in the file are left untouched.                     */
static int ini_rewrite_schedule(const char *path, ScheduleEntry *entries, int count)
{
    FILE *fp = fopen(path, "r");
    char **lines = NULL;
    int n_lines = 0, cap_lines = 0;
    char linebuf[1024];
    char tmp_path[MAX_PATH_LEN];
    FILE *out;
    int i, ok = 1;

    if (!fp) return 0;

    while (fgets(linebuf, sizeof(linebuf), fp)) {
        size_t len = strlen(linebuf);
        char *copy;
        char *p = linebuf;
        char *eq;
        int is_sched_line = 0;

        while (*p == ' ' || *p == '\t') p++;
        if (*p != ';' && *p != '#' && *p != '\n' && *p != '\0' &&
                (eq = strchr(p, '=')) != NULL) {
            char keybuf[64];
            size_t klen = (size_t)(eq - p);
            char *kend;
            if (klen < sizeof(keybuf)) {
                memcpy(keybuf, p, klen);
                keybuf[klen] = '\0';
                kend = keybuf + strlen(keybuf);
                while (kend > keybuf && (kend[-1] == ' ' || kend[-1] == '\t')) *--kend = '\0';
                is_sched_line = is_schedule_entry_key(keybuf);
            }
        }
        if (is_sched_line) continue;  /* drop - a fresh block is appended below */

        copy = (char *)malloc(len + 1);
        if (!copy) { ok = 0; break; }
        memcpy(copy, linebuf, len + 1);
        if (n_lines >= cap_lines) {
            int newcap = cap_lines ? cap_lines * 2 : 256;
            char **nl = (char **)realloc(lines, (size_t)newcap * sizeof(char *));
            if (!nl) { free(copy); ok = 0; break; }
            lines = nl;
            cap_lines = newcap;
        }
        lines[n_lines++] = copy;
    }
    fclose(fp);
    if (!ok) goto fail_free2;

    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);
    out = fopen(tmp_path, "w");
    if (!out) goto fail_free2;

    for (i = 0; i < n_lines; i++)
        fputs(lines[i], out);

    if (count > 0) {
        fprintf(out, "\n; --- Schedule entries (Settings dialog, Schedule tab) ---\n");
        for (i = 0; i < count; i++) {
            ScheduleEntry *e = &entries[i];
            char dur[32];
            format_duration_hms(e->duration_sec, dur, sizeof(dur));
            fprintf(out, "schedule_%d_start_time  = %s\n", i + 1, e->start_time);
            fprintf(out, "schedule_%d_duration    = %s\n", i + 1, dur);
            if (e->frequency_hz > 0.0)
                fprintf(out, "schedule_%d_frequency   = %.6g\n", i + 1, e->frequency_hz / 1e6);
            if (e->freq_b_hz > 0.0)
                fprintf(out, "schedule_%d_freq_b      = %.6g\n", i + 1, e->freq_b_hz / 1e6);
            if (e->antenna[0])
                fprintf(out, "schedule_%d_antenna     = %s\n", i + 1, e->antenna);
            if (e->output_file[0])
                fprintf(out, "schedule_%d_output_file = %s\n", i + 1, e->output_file);
            fprintf(out, "\n");
        }
    }
    fclose(out);

    for (i = 0; i < n_lines; i++) free(lines[i]);
    free(lines);

    if (!MoveFileExA(tmp_path, path, MOVEFILE_REPLACE_EXISTING)) {
        DeleteFileA(tmp_path);
        return 0;
    }
    return 1;

fail_free2:
    for (i = 0; i < n_lines; i++) free(lines[i]);
    free(lines);
    return 0;
}

/* Populates the Sample Rate/IF/Bandwidth combo from VALID_COMBOS - the
 * exact same table validate_config() checks against, so nothing sold
 * here can ever fail that check. When dual_only is set, only entries
 * valid for RSPduo dual/Master-Slave mode are listed. */
/* Populates the Sample Rate/IF/Bandwidth combo from VALID_COMBOS - the
 * exact same table validate_config() checks against, so nothing shown
 * here can ever fail that check. dual_only restricts to RSPduo dual/
 * Master-Slave-compatible entries; hdr_only restricts to the one entry
 * HDR mode requires (6 Msps, 1620 kHz IF) - apply_hdr_mode() silently
 * disables HDR at Start time if any other combo is picked, so filtering
 * here up front avoids that surprise. */
static void settings_populate_ratecombo(int dual_only, int hdr_only)
{
    int i;
    SendMessageA(g_hSetRateCombo, CB_RESETCONTENT, 0, 0);
    for (i = 0; i < NUM_VALID_COMBOS; i++) {
        const ValidCombo *c = &VALID_COMBOS[i];
        if (dual_only && !c->dual_ok) continue;
        if (hdr_only && !(c->if_khz == 1620 &&
                           fabs(c->adc_rate_hz - 6000000.0) < 1.0)) continue;
        SendMessageA(g_hSetRateCombo, CB_ADDSTRING, 0, (LPARAM)c->note);
    }
}

/* Selects the combo entry matching the working config's current
 * sample_rate_hz/if_khz/bw_khz, or index 0 if nothing matches exactly
 * (e.g. after switching the dual-only filter). */
static void settings_select_ratecombo_for_current(void)
{
    int i, list_idx = 0, found = 0;
    for (i = 0; i < NUM_VALID_COMBOS; i++) {
        const ValidCombo *c = &VALID_COMBOS[i];
        LRESULT n = SendMessageA(g_hSetRateCombo, CB_FINDSTRINGEXACT, (WPARAM)-1,
                                  (LPARAM)c->note);
        if (n == CB_ERR) continue;   /* filtered out of the current list */
        if (!found &&
                fabs(c->adc_rate_hz - g_settings_cfg.sample_rate_hz) < 1.0 &&
                c->if_khz == g_settings_cfg.if_khz &&
                c->bw_khz == g_settings_cfg.bw_khz) {
            list_idx = (int)n;
            found = 1;
        }
    }
    SendMessageA(g_hSetRateCombo, CB_SETCURSEL, (WPARAM)list_idx, 0);
}

/* HDR mode only works at ten specific centre frequencies - apply_hdr_mode()
 * silently falls back to a normal non-HDR recording (after an easy-to-miss
 * log error) if the frequency doesn't match. Checked live here instead so
 * it's obvious in the dialog before you ever press Start. */
static void settings_update_hdr_hint(void)
{
    LRESULT hdr = SendMessageA(g_hSetHdr, BM_GETCHECK, 0, 0) == BST_CHECKED;
    char buf[32];
    double freq_khz;
    int i, valid = 0;

    if (!g_hSetHdrHint) return;
    if (!hdr) {
        SetWindowTextA(g_hSetHdrHint, "");
        g_settings_hdr_freq_valid = 1;
        InvalidateRect(g_hSetHdrHint, NULL, TRUE);
        return;
    }

    GetWindowTextA(g_hSetDualT1Freq, buf, sizeof(buf));
    freq_khz = atof(buf) * 1000.0;
    for (i = 0; i < NUM_HDR_VALID_KHZ; i++) {
        if (fabs(freq_khz - HDR_VALID_KHZ[i]) < 0.5) { valid = 1; break; }
    }
    g_settings_hdr_freq_valid = valid;
    SetWindowTextA(g_hSetHdrHint,
        valid ? "Valid HDR frequency"
              : "Not a valid HDR CF - needs 135/175/220/250/340/475/"
                "516/875/1125/1900 kHz");
    InvalidateRect(g_hSetHdrHint, NULL, TRUE);
}

/* Shows the actual first-recording time given the current Session Start
 * and Window - the pre-record buffer means recording genuinely begins
 * window/2 minutes before the hour shown in Session Start, not at that
 * hour itself (e.g. Session Start 08:00 with a 10-minute window actually
 * starts recording at 07:55). Session Start itself is just when the
 * overnight session becomes eligible to open, not a recording time - see
 * Section 8.3 of the guide - and this makes that gap visible up front
 * rather than only discoverable by comparing the log against what was
 * typed.                                                                 */
static void settings_update_hourly_hint(void)
{
    char start_buf[16], win_buf[16];
    int start_h, start_m, window_min, half_win_sec, start_sec, first_sec;
    int first_h_disp, first_m_disp, first_s_disp;

    if (!g_hSetHourlyHint) return;

    GetWindowTextA(g_hSetHourlyStart, start_buf, sizeof(start_buf));
    GetWindowTextA(g_hSetHourlyWin, win_buf, sizeof(win_buf));
    window_min = atoi(win_buf);
    if (sscanf(start_buf, "%d:%d", &start_h, &start_m) != 2 ||
            start_h < 0 || start_h > 23 || window_min <= 0) {
        SetWindowTextA(g_hSetHourlyHint, "");
        InvalidateRect(g_hSetHourlyHint, NULL, TRUE);
        return;
    }

    /* Same seconds-precision math as hourly_wait_for_next() itself uses,
     * so this hint can never show an answer that disagrees with what
     * actually happens. window_min*60 is always even (60 is even), so
     * this never loses precision even for odd window lengths like 3
     * minutes - half_win_sec comes out to exactly 90 (1m30s), not
     * rounded down to a whole 1 or 2 minutes the way integer-minute
     * division would.                                                   */
    half_win_sec = (window_min * 60) / 2;
    start_sec = (start_h * 60 + start_m) * 60;
    first_sec = start_sec - half_win_sec;
    if (first_sec < 0) first_sec += 24 * 3600;
    first_h_disp = first_sec / 3600;
    first_m_disp = (first_sec % 3600) / 60;
    first_s_disp = first_sec % 60;

    {
        char buf[112];
        if (half_win_sec % 60 == 0) {
            snprintf(buf, sizeof(buf),
                     "First recording actually begins at %02d:%02d:%02d "
                     "(%d min before the hour, half the window)",
                     first_h_disp, first_m_disp, first_s_disp,
                     half_win_sec / 60);
        } else {
            snprintf(buf, sizeof(buf),
                     "First recording actually begins at %02d:%02d:%02d "
                     "(%dm %02ds before the hour, half the window)",
                     first_h_disp, first_m_disp, first_s_disp,
                     half_win_sec / 60, half_win_sec % 60);
        }
        SetWindowTextA(g_hSetHourlyHint, buf);
    }
    InvalidateRect(g_hSetHourlyHint, NULL, TRUE);
}

/* Shows/hides the COHERENT label next to the frequency row - same
 * definition as the main window's own indicator (Section 7.3 of the
 * guide): both tuners active and tuned to the same frequency.           */
static void settings_update_coherent_indicator(void)
{
    LRESULT t1 = SendMessageA(g_hSetTuner1En, BM_GETCHECK, 0, 0) == BST_CHECKED;
    LRESULT t2 = SendMessageA(g_hSetTuner2En, BM_GETCHECK, 0, 0) == BST_CHECKED;
    char f1[32], f2[32];
    int coherent;

    GetWindowTextA(g_hSetDualT1Freq, f1, sizeof(f1));
    GetWindowTextA(g_hSetFreqB, f2, sizeof(f2));
    coherent = t1 && t2 && f1[0] && f2[0] && fabs(atof(f1) - atof(f2)) < 1e-6;

    ShowWindow(g_hSetCoherentLbl, coherent ? SW_SHOW : SW_HIDE);
}

/* Explicit dim-state tracking for the Tuner 2 / Same as T1 (GR, LNA) /
 * Tuner 2: same AGC... checkboxes - checked directly by HWND in
 * settings_wndproc's WM_CTLCOLORBTN handler. A genuinely disabled
 * checkbox's label text ignores the app's custom colour entirely and
 * falls back to Windows' own bright, hard-to-read "disabled" text style -
 * the same class of problem the edit fields ran into. Keeping the
 * checkbox enabled (so the custom colour actually applies) but tracking
 * a separate "active" flag here lets the label be coloured properly,
 * with the click itself blocked in the WM_COMMAND handler below instead
 * of relying on EnableWindow - see the BN_CLICKED cases for
 * IDC_SET_TUNER2_EN / IDC_SET_GRB_SAME / IDC_SET_LNAB_SAME /
 * IDC_SET_B_SAME_CORR.                                                    */
static HWND g_set_check_dim_hwnd[4];
static int  g_set_check_dim_state[4];

static int settings_check_is_dim(HWND h)
{
    int i;
    for (i = 0; i < 4; i++)
        if (g_set_check_dim_hwnd[i] == h) return g_set_check_dim_state[i];
    return 0;
}

static void settings_set_check_dim(HWND h, int active)
{
    int i;
    EnableWindow(h, TRUE);
    for (i = 0; i < 4; i++) {
        if (g_set_check_dim_hwnd[i] == h || g_set_check_dim_hwnd[i] == NULL) {
            g_set_check_dim_hwnd[i] = h;
            g_set_check_dim_state[i] = !active;
            break;
        }
    }
    /* The state above only affects colour at the next repaint - without
     * this, a checkbox could sit showing its old colour indefinitely
     * even after its actual state had changed, since nothing else here
     * ever prompts Windows to repaint it.                                */
    InvalidateRect(h, NULL, TRUE);
}

/* Reverted to plain EnableWindow() - the read-only/custom-colour approach
 * tried here was aimed at the wrong controls; what needed changing was
 * the checkbox labels (Tuner 2, Same as T1, Tuner 2: same AGC/DC/IQ/
 * notch...), not these edit fields, which the user wants left exactly as
 * they originally were. See settings_set_check_dim() above for the
 * actual fix.                                                            */
static void settings_set_edit_readonly(HWND h, int enabled)
{
    EnableWindow(h, enabled);
}

/* Shows the "large file handling" combo when Winrad or SDR Connect output
 * format is selected AND the Recording tab (where it lives) is the active
 * one. Not shown for Linrad/WavViewDX (no 4 GiB WAV header limit to work
 * around) or for SDRuno - SDRuno itself doesn't support RF64 playback, so
 * that format always splits at 4 GiB with no user-facing choice; Winrad
 * exists specifically as the RF64-capable alternative. Format selection
 * alone isn't enough to gate visibility: this control is tagged for tab 2
 * like everything else on Recording, so switching to a different tab must
 * still hide it even if the format condition would otherwise say "show". */
static void settings_update_format_dependent_state(void)
{
    LRESULT fsel = g_hSetFormat ? SendMessageA(g_hSetFormat, CB_GETCURSEL, 0, 0) : CB_ERR;
    int show = (fsel == FORMAT_WINRAD || fsel == FORMAT_SDRCONNECT)
               && (g_settings_active_tab == 2);
    if (g_hSetLargeModeLbl) ShowWindow(g_hSetLargeModeLbl, show ? SW_SHOW : SW_HIDE);
    if (g_hSetLargeMode)    ShowWindow(g_hSetLargeMode,    show ? SW_SHOW : SW_HIDE);
}

static void settings_update_dual_enable_state(void)
{
    LRESULT t1  = SendMessageA(g_hSetTuner1En, BM_GETCHECK, 0, 0) == BST_CHECKED;
    LRESULT t2  = SendMessageA(g_hSetTuner2En, BM_GETCHECK, 0, 0) == BST_CHECKED;
    LRESULT dual = t1 && t2;
    LRESULT hdr  = SendMessageA(g_hSetHdr, BM_GETCHECK, 0, 0) == BST_CHECKED;
    LRESULT gr_same = SendMessageA(g_hSetGrBSame, BM_GETCHECK, 0, 0) == BST_CHECKED;
    LRESULT lna_same = SendMessageA(g_hSetLnaBSame, BM_GETCHECK, 0, 0) == BST_CHECKED;

    /* Tuner 2 can only be selected on an RSPduo - hwVer is only known
     * once a session has run at least once in this app instance (device
     * enumeration doesn't happen just from opening Settings). If it's
     * still unknown (fresh launch, nothing recorded yet), default to
     * enabled rather than guessing wrong; it's harmless either way.     */
    {
        int known_non_duo = (g_last_known_hwVer != 0 &&
                              g_last_known_hwVer != SDRPLAY_RSPduo_ID);
        settings_set_check_dim(g_hSetTuner2En, !known_non_duo);
        if (known_non_duo && t2) {
            SendMessageA(g_hSetTuner2En, BM_SETCHECK, BST_UNCHECKED, 0);
            SendMessageA(g_hSetTuner1En, BM_SETCHECK, BST_CHECKED, 0);
            t2 = 0; t1 = 1; dual = 0;
        }
    }

    /* Dual Channel and HDR are mutually exclusive - no device does both.
     * Dual is now derived from both tuner checkboxes rather than its own
     * checkbox, but the conflict is the same one HDR already resolves
     * against dual_channel at session start (validate_config()) - this
     * just surfaces it in the dialog instead of leaving it to a runtime
     * warning.                                                          */
    EnableWindow(g_hSetHdr, (BOOL)!dual);
    if (dual && hdr) {
        SendMessageA(g_hSetHdr, BM_SETCHECK, BST_UNCHECKED, 0);
        hdr = 0;
    }

    /* Tuner 1's fields: relevant whenever Tuner 1 is checked, whether
     * alone (single-tuner-A) or alongside Tuner 2 (dual).               */
    settings_set_edit_readonly(g_hSetDualT1Freq, t1);
    settings_set_edit_readonly(g_hSetDualT1Gr, t1);
    settings_set_edit_readonly(g_hSetDualT1Lna, t1);
    EnableWindow(g_hSetDualT1Antenna, (BOOL)t1);

    /* Tuner 2's fields: relevant whenever Tuner 2 is checked, whether
     * alone (single-tuner-B - its fields are then the primary, directly-
     * editable values, with no "Tuner 1" to copy from) or alongside
     * Tuner 1 (dual - where "Same as T1" becomes available).            */
    settings_set_edit_readonly(g_hSetFreqB, t2);
    settings_set_check_dim(g_hSetGrBSame, dual);
    settings_set_check_dim(g_hSetLnaBSame, dual);
    settings_set_edit_readonly(g_hSetGrB, t2 && !(dual && gr_same));
    settings_set_edit_readonly(g_hSetLnaB, t2 && !(dual && lna_same));
    settings_set_check_dim(g_hSetBSameCorr, dual);
    {
        int same_corr = SendMessageA(g_hSetBSameCorr, BM_GETCHECK, 0, 0) == BST_CHECKED;
        int cmd_b = (dual && !same_corr) ? SW_SHOW : SW_HIDE;
        if (g_hSetAgcB)      ShowWindow(g_hSetAgcB,      cmd_b);
        if (g_hSetDcB)       ShowWindow(g_hSetDcB,       cmd_b);
        if (g_hSetIqB)       ShowWindow(g_hSetIqB,       cmd_b);
        if (g_hSetNotchRfB)  ShowWindow(g_hSetNotchRfB,  cmd_b);
        if (g_hSetNotchDabB) ShowWindow(g_hSetNotchDabB, cmd_b);
    }

    settings_populate_ratecombo((int)dual, (int)hdr);
    settings_select_ratecombo_for_current();
    settings_update_coherent_indicator();
}

/* settings_update_duration_hms() removed - the Duration field itself now
 * directly holds HH:MM:SS, so the separate converted readout beside it
 * that this used to maintain is no longer needed.                       */

/* Reads duodx.ini fresh (not g_state.cfg) so the dialog always reflects
 * what's really on disk, then pushes those values into the controls. */
/* Refreshes the "Entry N of M" label and Prev/Next/Add/Delete enabled
 * state for the current position in g_settings_schedule[].              */
static void settings_schedule_update_nav_ui(void)
{
    char buf[32];
    int count = g_settings_schedule_count;
    int idx1 = count > 0 ? g_settings_schedule_idx + 1 : 0;
    snprintf(buf, sizeof(buf), "Entry %d of %d", idx1, count);
    SetWindowTextA(g_hSetSchedIdxLbl, buf);
    EnableWindow(g_hSetSchedPrev, count > 0 && g_settings_schedule_idx > 0);
    EnableWindow(g_hSetSchedNext, count > 0 && g_settings_schedule_idx < count - 1);
    EnableWindow(g_hSetSchedAdd, count < MAX_SCHEDULE_ENTRIES);
    EnableWindow(g_hSetSchedDel, count > 0);
    EnableWindow(g_hSetSchedStart, count > 0);
    EnableWindow(g_hSetSchedDuration, count > 0);
    EnableWindow(g_hSetSchedFreq, count > 0);
    EnableWindow(g_hSetSchedFreqB, count > 0);
    EnableWindow(g_hSetSchedAntenna, count > 0);
    EnableWindow(g_hSetSchedOutfile, count > 0);
    InvalidateRect(g_hSetSchedPrev, NULL, TRUE);
    InvalidateRect(g_hSetSchedNext, NULL, TRUE);
    InvalidateRect(g_hSetSchedAdd, NULL, TRUE);
    InvalidateRect(g_hSetSchedDel, NULL, TRUE);
}

/* Loads g_settings_schedule[g_settings_schedule_idx] into the visible
 * fields, or blanks them all if there are no entries.                   */
static void settings_schedule_load_current_from_array(void)
{
    char buf[32];
    if (g_settings_schedule_count == 0) {
        SetWindowTextA(g_hSetSchedStart, "");
        SetWindowTextA(g_hSetSchedDuration, "");
        SetWindowTextA(g_hSetSchedFreq, "");
        SetWindowTextA(g_hSetSchedFreqB, "");
        SetWindowTextA(g_hSetSchedOutfile, "");
        SetWindowTextA(g_hSetSchedAntenna, "");
    } else {
        ScheduleEntry *e = &g_settings_schedule[g_settings_schedule_idx];
        SetWindowTextA(g_hSetSchedStart, e->start_time);
        format_duration_hms(e->duration_sec, buf, sizeof(buf));
        SetWindowTextA(g_hSetSchedDuration, buf);
        if (e->frequency_hz > 0.0) {
            snprintf(buf, sizeof(buf), "%.6g", e->frequency_hz / 1e6);
            SetWindowTextA(g_hSetSchedFreq, buf);
        } else {
            SetWindowTextA(g_hSetSchedFreq, "");
        }
        if (e->freq_b_hz > 0.0) {
            snprintf(buf, sizeof(buf), "%.6g", e->freq_b_hz / 1e6);
            SetWindowTextA(g_hSetSchedFreqB, buf);
        } else {
            SetWindowTextA(g_hSetSchedFreqB, "");
        }
        SetWindowTextA(g_hSetSchedOutfile, e->output_file);
        SetWindowTextA(g_hSetSchedAntenna, e->antenna);
    }
    settings_schedule_update_nav_ui();
}

/* Saves the visible fields back into g_settings_schedule[idx] - called
 * before navigating away from an entry (Prev/Next/Delete) and again
 * before writing to disk on Save, so in-progress edits are never lost
 * just because the user didn't step off the entry first.                */
static void settings_schedule_save_current_to_array(void)
{
    char buf[MAX_PATH_LEN];
    ScheduleEntry *e;
    if (g_settings_schedule_count == 0) return;
    e = &g_settings_schedule[g_settings_schedule_idx];

    GetWindowTextA(g_hSetSchedStart, buf, sizeof(buf));
    strncpy(e->start_time, buf, sizeof(e->start_time) - 1);
    e->start_time[sizeof(e->start_time) - 1] = '\0';

    GetWindowTextA(g_hSetSchedDuration, buf, sizeof(buf));
    e->duration_sec = parse_duration_hms(buf);

    GetWindowTextA(g_hSetSchedFreq, buf, sizeof(buf));
    e->frequency_hz = buf[0] ? atof(buf) * 1e6 : 0.0;

    GetWindowTextA(g_hSetSchedFreqB, buf, sizeof(buf));
    e->freq_b_hz = buf[0] ? atof(buf) * 1e6 : 0.0;

    GetWindowTextA(g_hSetSchedOutfile, buf, sizeof(buf));
    strncpy(e->output_file, buf, sizeof(e->output_file) - 1);
    e->output_file[sizeof(e->output_file) - 1] = '\0';

    GetWindowTextA(g_hSetSchedAntenna, buf, sizeof(buf));
    strncpy(e->antenna, buf, sizeof(e->antenna) - 1);
    e->antenna[sizeof(e->antenna) - 1] = '\0';
}

static void settings_load_controls(void)
{
    char buf[64];
    int single_b;

    config_set_defaults(&g_settings_cfg);
    config_load_ini(&g_settings_cfg, "duodx.ini");

    single_b = !g_settings_cfg.dual_channel &&
               !strcmp(g_settings_cfg.rspduo_single_tuner, "B");

    /* Tuner 1 column: always the primary config values, whether Tuner 1
     * is actually the active one right now or not (harmless when not -
     * the fields are simply greyed out in that case).                   */
    snprintf(buf, sizeof(buf), "%.6g", g_settings_cfg.frequency_hz / 1e6);
    SetWindowTextA(g_hSetDualT1Freq, buf);
    snprintf(buf, sizeof(buf), "%d", g_settings_cfg.gain_reduction);
    SetWindowTextA(g_hSetDualT1Gr, buf);
    snprintf(buf, sizeof(buf), "%d", g_settings_cfg.lna_state);
    SetWindowTextA(g_hSetDualT1Lna, buf);

    /* Tuner 2 column: the primary config values when Tuner 2 is the sole
     * active tuner (single-tuner-B - there is no separate "Tuner 2" key
     * in that case, frequency_hz/gain_reduction/lna_state already are
     * Tuner 2's real settings), otherwise the separate _b values used in
     * true dual mode.                                                   */
    if (single_b) {
        snprintf(buf, sizeof(buf), "%.6g", g_settings_cfg.frequency_hz / 1e6);
        SetWindowTextA(g_hSetFreqB, buf);
        snprintf(buf, sizeof(buf), "%d", g_settings_cfg.gain_reduction);
        SetWindowTextA(g_hSetGrB, buf);
        snprintf(buf, sizeof(buf), "%d", g_settings_cfg.lna_state);
        SetWindowTextA(g_hSetLnaB, buf);
    } else {
        snprintf(buf, sizeof(buf), "%.6g", g_settings_cfg.freq_b_hz / 1e6);
        SetWindowTextA(g_hSetFreqB, buf);
        snprintf(buf, sizeof(buf), "%d", g_settings_cfg.gain_reduction_b);
        SetWindowTextA(g_hSetGrB, buf);
        snprintf(buf, sizeof(buf), "%d", g_settings_cfg.lna_state_b);
        SetWindowTextA(g_hSetLnaB, buf);
    }

    SendMessageA(g_hSetAgc, BM_SETCHECK,
                 g_settings_cfg.agc_enable ? BST_CHECKED : BST_UNCHECKED, 0);
    SendMessageA(g_hSetTuner1En, BM_SETCHECK,
                 (g_settings_cfg.dual_channel || !single_b) ? BST_CHECKED : BST_UNCHECKED, 0);
    SendMessageA(g_hSetTuner2En, BM_SETCHECK,
                 (g_settings_cfg.dual_channel || single_b) ? BST_CHECKED : BST_UNCHECKED, 0);
    SendMessageA(g_hSetHdr, BM_SETCHECK,
                 g_settings_cfg.hdr_enable ? BST_CHECKED : BST_UNCHECKED, 0);

    snprintf(buf, sizeof(buf), "%.6g", g_settings_cfg.ppm);
    SetWindowTextA(g_hSetPpm, buf);
    SendMessageA(g_hSetDc, BM_SETCHECK,
                 g_settings_cfg.dc_correct ? BST_CHECKED : BST_UNCHECKED, 0);
    SendMessageA(g_hSetIq, BM_SETCHECK,
                 g_settings_cfg.iq_correct ? BST_CHECKED : BST_UNCHECKED, 0);
    SendMessageA(g_hSetNotchRf, BM_SETCHECK,
                 g_settings_cfg.notch_rf ? BST_CHECKED : BST_UNCHECKED, 0);
    SendMessageA(g_hSetNotchDab, BM_SETCHECK,
                 g_settings_cfg.notch_dab ? BST_CHECKED : BST_UNCHECKED, 0);
    SendMessageA(g_hSetBiasT, BM_SETCHECK,
                 g_settings_cfg.bias_t ? BST_CHECKED : BST_UNCHECKED, 0);
    SendMessageA(g_hSetHiz, BM_SETCHECK,
                 g_settings_cfg.hiz_notch ? BST_CHECKED : BST_UNCHECKED, 0);
    {
        int bw = g_settings_cfg.hdr_bw_khz;
        int idx = 3; /* default 1700, matches config_set_defaults() */
        if (bw <= 200) idx = 0;
        else if (bw <= 500) idx = 1;
        else if (bw <= 1200) idx = 2;
        SendMessageA(g_hSetHdrBw, CB_SETCURSEL, (WPARAM)idx, 0);
    }
    /* "Same as A" for Tuner B corrections/notches/AGC: true only if every
     * one of the five underlying _b fields is still at its -1 (inherit)
     * sentinel. If any of them was set independently (e.g. by hand-editing
     * duodx.ini), leave the box unchecked so re-saving doesn't silently
     * collapse a deliberately different Tuner B setting back onto A.     */
    SendMessageA(g_hSetBSameCorr, BM_SETCHECK,
                 (g_settings_cfg.agc_enable_b   < 0 &&
                  g_settings_cfg.dc_correct_b   < 0 &&
                  g_settings_cfg.iq_correct_b   < 0 &&
                  g_settings_cfg.notch_rf_b     < 0 &&
                  g_settings_cfg.notch_dab_b    < 0) ? BST_CHECKED : BST_UNCHECKED, 0);

    /* The actual Tuner B values - default to matching Tuner A's own
     * loaded value when still at the -1 inherit sentinel, so unchecking
     * "same as T1" starts from a sensible, already-familiar state rather
     * than everything defaulting off regardless of what A is set to.     */
    SendMessageA(g_hSetAgcB, BM_SETCHECK,
                 (g_settings_cfg.agc_enable_b >= 0 ? g_settings_cfg.agc_enable_b
                                                    : g_settings_cfg.agc_enable)
                 ? BST_CHECKED : BST_UNCHECKED, 0);
    SendMessageA(g_hSetDcB, BM_SETCHECK,
                 (g_settings_cfg.dc_correct_b >= 0 ? g_settings_cfg.dc_correct_b
                                                    : g_settings_cfg.dc_correct)
                 ? BST_CHECKED : BST_UNCHECKED, 0);
    SendMessageA(g_hSetIqB, BM_SETCHECK,
                 (g_settings_cfg.iq_correct_b >= 0 ? g_settings_cfg.iq_correct_b
                                                    : g_settings_cfg.iq_correct)
                 ? BST_CHECKED : BST_UNCHECKED, 0);
    SendMessageA(g_hSetNotchRfB, BM_SETCHECK,
                 (g_settings_cfg.notch_rf_b >= 0 ? g_settings_cfg.notch_rf_b
                                                  : g_settings_cfg.notch_rf)
                 ? BST_CHECKED : BST_UNCHECKED, 0);
    SendMessageA(g_hSetNotchDabB, BM_SETCHECK,
                 (g_settings_cfg.notch_dab_b >= 0 ? g_settings_cfg.notch_dab_b
                                                   : g_settings_cfg.notch_dab)
                 ? BST_CHECKED : BST_UNCHECKED, 0);

    SendMessageA(g_hSetMonVisible, BM_SETCHECK,
                 g_settings_cfg.monitor_bar_visible ? BST_CHECKED : BST_UNCHECKED, 0);
    {
        int ms = g_settings_cfg.monitor_interval_ms;
        int idx = 1; /* default 500 ms */
        if (ms <= 200) idx = 0;
        else if (ms >= 1000) idx = 2;
        SendMessageA(g_hSetMonInterval, CB_SETCURSEL, (WPARAM)idx, 0);
    }
    SendMessageA(g_hSetSMeterMode, CB_SETCURSEL,
                 (WPARAM)(g_settings_cfg.monitor_smeter_mode ? 1 : 0), 0);
    snprintf(buf, sizeof(buf), "%.6g", g_settings_cfg.monitor_smeter_cal_offset);
    SetWindowTextA(g_hSetSMeterCal, buf);
    SendMessageA(g_hSetUseUtc, BM_SETCHECK,
                 g_settings_cfg.use_utc ? BST_CHECKED : BST_UNCHECKED, 0);
    SendMessageA(g_hSetShowClock, BM_SETCHECK,
                 g_settings_cfg.show_clock ? BST_CHECKED : BST_UNCHECKED, 0);
    SendMessageA(g_hSetVerbose, BM_SETCHECK,
                 g_settings_cfg.verbose ? BST_CHECKED : BST_UNCHECKED, 0);
    SendMessageA(g_hSetLogAutoSave, BM_SETCHECK,
                 g_settings_cfg.log_auto_save ? BST_CHECKED : BST_UNCHECKED, 0);
    snprintf(buf, sizeof(buf), "%.6f", g_settings_cfg.latitude);
    SetWindowTextA(g_hSetLatitude, buf);
    snprintf(buf, sizeof(buf), "%.6f", g_settings_cfg.longitude);
    SetWindowTextA(g_hSetLongitude, buf);
    SendMessageA(g_hSetShowSun, BM_SETCHECK,
                 g_settings_cfg.show_sun_times ? BST_CHECKED : BST_UNCHECKED, 0);
    {
        int ms = g_settings_cfg.meter_style;
        SendMessageA(g_hSetMeterStyle, CB_SETCURSEL, (WPARAM)(ms >= 0 && ms <= 2 ? ms : 0), 0);
    }

    snprintf(buf, sizeof(buf), "%d", g_settings_cfg.http_port);
    SetWindowTextA(g_hSetHttpPort, buf);
    snprintf(buf, sizeof(buf), "%d", g_settings_cfg.http_interval_ms);
    SetWindowTextA(g_hSetHttpInterval, buf);
    SendMessageA(g_hSetPipeEn, BM_SETCHECK,
                 g_settings_cfg.pipe_enable ? BST_CHECKED : BST_UNCHECKED, 0);
    SetWindowTextA(g_hSetPipeName, g_settings_cfg.pipe_name);

    snprintf(buf, sizeof(buf), "%d", g_settings_cfg.ring_buffer_sec);
    SetWindowTextA(g_hSetRingSec, buf);
    SendMessageA(g_hSetSpinupEn, BM_SETCHECK,
                 g_settings_cfg.spinup_enable ? BST_CHECKED : BST_UNCHECKED, 0);
    {
        int bytes = g_settings_cfg.spinup_bytes;
        int idx = 0; /* default 1 MB if the stored value is unrecognised */
        if (bytes >= 8 * 1024 * 1024) idx = 2;
        else if (bytes >= 4 * 1024 * 1024) idx = 1;
        SendMessageA(g_hSetSpinupBytes, CB_SETCURSEL, (WPARAM)idx, 0);
    }

    SendMessageA(g_hSetGrBSame, BM_SETCHECK,
                 g_settings_cfg.gain_reduction_b < 0 ? BST_CHECKED : BST_UNCHECKED, 0);
    snprintf(buf, sizeof(buf), "%d",
             g_settings_cfg.gain_reduction_b >= 0 ? g_settings_cfg.gain_reduction_b
                                                   : g_settings_cfg.gain_reduction);
    SetWindowTextA(g_hSetGrB, buf);

    SendMessageA(g_hSetLnaBSame, BM_SETCHECK,
                 g_settings_cfg.lna_state_b < 0 ? BST_CHECKED : BST_UNCHECKED, 0);
    snprintf(buf, sizeof(buf), "%d",
             g_settings_cfg.lna_state_b >= 0 ? g_settings_cfg.lna_state_b
                                              : g_settings_cfg.lna_state);
    SetWindowTextA(g_hSetLnaB, buf);

    format_duration_hms(g_settings_cfg.duration_sec, buf, sizeof(buf));
    SetWindowTextA(g_hSetDuration, buf);

    {
        int d = g_settings_cfg.decimation;
        int idx = 0; /* default to 1 (off) if the stored value is unrecognised */
        switch (d) {
            case 1:  idx = 0; break;
            case 2:  idx = 1; break;
            case 4:  idx = 2; break;
            case 8:  idx = 3; break;
            case 16: idx = 4; break;
            case 32: idx = 5; break;
        }
        SendMessageA(g_hSetDecim, CB_SETCURSEL, (WPARAM)idx, 0);
    }

    SetWindowTextA(g_hSetDualT1Antenna, g_settings_cfg.antenna);

    SendMessageA(g_hSetFormat, CB_SETCURSEL, (WPARAM)g_settings_cfg.output_format, 0);
    if (g_hSetLargeMode)
        SendMessageA(g_hSetLargeMode, CB_SETCURSEL,
                     (WPARAM)g_settings_cfg.large_file_mode, 0);
    settings_update_format_dependent_state();
    SetWindowTextA(g_hSetPath, g_settings_cfg.recording_path);

    {
        int want_hourly = !strcmp(g_settings_cfg.timer_last_mode, "hourly");
        SendMessageA(g_hSetSchedOnly, BM_SETCHECK,
                     want_hourly ? BST_UNCHECKED : BST_CHECKED, 0);
        SendMessageA(g_hSetHourlyEn, BM_SETCHECK,
                     want_hourly ? BST_CHECKED : BST_UNCHECKED, 0);
    }
    snprintf(buf, sizeof(buf), "%d", g_settings_cfg.hourly_window_min);
    SetWindowTextA(g_hSetHourlyWin, buf);
    SetWindowTextA(g_hSetHourlyStart, g_settings_cfg.hourly_start);
    SetWindowTextA(g_hSetHourlyStop, g_settings_cfg.hourly_stop);

    /* Copy into the separate in-memory array the navigator edits, rather
     * than editing g_settings_cfg.schedule[] directly - keeps this dialog
     * consistent with every other field (Cancel discards cleanly), and
     * the on-disk entry count is only ever written explicitly on Save
     * rather than tracked implicitly through array mutations mid-edit.  */
    memcpy(g_settings_schedule, g_settings_cfg.schedule, sizeof(g_settings_schedule));
    g_settings_schedule_count = g_settings_cfg.schedule_count;
    g_settings_schedule_idx = 0;
    settings_schedule_load_current_from_array();

    settings_update_dual_enable_state();
    settings_update_hdr_hint();
    settings_update_hourly_hint();

    SendMessageA(g_hSetColorScheme, CB_SETCURSEL,
                 (WPARAM)(!strcmp(g_settings_cfg.color_scheme, "grey") ? 1 : 0), 0);
}

/* Reads the controls, patches duodx.ini in place, and reloads g_state.cfg
 * so the change is live immediately without needing to restart the app. */
static void settings_save(void)
{
    /* Sized with generous headroom for the Tab 1-4 migration (24 new
     * fields on top of the ~20 that were already here), plus the
     * Schedule tab's scalar fields (the entries themselves are handled
     * separately below, via ini_rewrite_schedule() rather than this
     * fixed-size array - a 32-entry list doesn't fit a bounded array
     * sized for a handful of scalar settings). */
    IniPatchEntry entries[96];
    int n = 0;
    char buf[80];
    LRESULT sel;
    /* Captured before this save touches g_state.cfg, so the tuner
     * selection actually in effect on the running device can be
     * compared against what's about to be saved - see the "tuner
     * selection changed while listening" handling further down.         */
    int old_dual = g_state.cfg.dual_channel;
    int old_single_b = !strcmp(g_state.cfg.rspduo_single_tuner, "B");
    double old_freq_b_hz = g_state.cfg.freq_b_hz;
    double old_frequency_hz = g_state.cfg.frequency_hz;
    int old_if_khz = g_state.cfg.if_khz;
    int old_bw_khz = g_state.cfg.bw_khz;
    double old_sample_rate_hz = g_state.cfg.sample_rate_hz;
    int old_master_slave_active = g_state.master_slave_active;
    int t1 = SendMessageA(g_hSetTuner1En, BM_GETCHECK, 0, 0) == BST_CHECKED;
    int t2 = SendMessageA(g_hSetTuner2En, BM_GETCHECK, 0, 0) == BST_CHECKED;
    int dual = t1 && t2;
    /* Tuner 2 selected without Tuner 1 - its column holds the primary
     * frequency_hz/gain_reduction/lna_state/antenna values directly,
     * there is no separate "Tuner 2" key for this case the way dual
     * mode has freq_b_hz/gain_reduction_b/lna_state_b.                  */
    int single_b = t2 && !t1;

    GetWindowTextA(single_b ? g_hSetFreqB : g_hSetDualT1Freq, buf, sizeof(buf));
    entries[n].key = "frequency_mhz";
    snprintf(entries[n].value, sizeof(entries[n].value), "%.6g", atof(buf));
    n++;

    GetWindowTextA(single_b ? g_hSetGrB : g_hSetDualT1Gr, buf, sizeof(buf));
    entries[n].key = "gain_reduction";
    snprintf(entries[n].value, sizeof(entries[n].value), "%d", atoi(buf));
    n++;

    GetWindowTextA(single_b ? g_hSetLnaB : g_hSetDualT1Lna, buf, sizeof(buf));
    entries[n].key = "lna_state";
    snprintf(entries[n].value, sizeof(entries[n].value), "%d", atoi(buf));
    n++;

    /* freq_b_mhz is only meaningful in true dual mode - in single-tuner-B
     * mode, Tuner 2's fields were just read above as the primary values,
     * so writing them again here as "Tuner B's" would duplicate/corrupt
     * the _b keys with primary-tuner data. Left untouched (whatever was
     * already on disk stays there) rather than written wrong.           */
    if (!single_b) {
        GetWindowTextA(g_hSetFreqB, buf, sizeof(buf));
        entries[n].key = "freq_b_mhz";
        snprintf(entries[n].value, sizeof(entries[n].value), "%.6g", atof(buf));
        n++;
    }

    entries[n].key = "agc_enable";
    snprintf(entries[n].value, sizeof(entries[n].value), "%d",
             SendMessageA(g_hSetAgc, BM_GETCHECK, 0, 0) == BST_CHECKED ? 1 : 0);
    n++;

    entries[n].key = "dual_channel";
    snprintf(entries[n].value, sizeof(entries[n].value), "%d", dual ? 1 : 0);
    n++;

    entries[n].key = "hdr_enable";
    snprintf(entries[n].value, sizeof(entries[n].value), "%d",
             SendMessageA(g_hSetHdr, BM_GETCHECK, 0, 0) == BST_CHECKED ? 1 : 0);
    n++;

    entries[n].key = "rspduo_single_tuner";
    snprintf(entries[n].value, sizeof(entries[n].value), "%s", single_b ? "B" : "A");
    n++;

    /* monitor_hpf_enable/monitor_hpf_hz are no longer saved from here -
     * the controls moved to the main window (next to Vol) and write
     * their own ini entries directly and immediately when changed, since
     * they need to take effect live rather than waiting for Save.        */

    entries[n].key = "monitor_bar_visible";
    snprintf(entries[n].value, sizeof(entries[n].value), "%d",
             SendMessageA(g_hSetMonVisible, BM_GETCHECK, 0, 0) == BST_CHECKED ? 1 : 0);
    n++;

    if (!single_b) {
        if (SendMessageA(g_hSetGrBSame, BM_GETCHECK, 0, 0) == BST_CHECKED) {
            entries[n].key = "gain_reduction_b";
            snprintf(entries[n].value, sizeof(entries[n].value), "-1");
        } else {
            GetWindowTextA(g_hSetGrB, buf, sizeof(buf));
            entries[n].key = "gain_reduction_b";
            snprintf(entries[n].value, sizeof(entries[n].value), "%d", atoi(buf));
        }
        n++;

        if (SendMessageA(g_hSetLnaBSame, BM_GETCHECK, 0, 0) == BST_CHECKED) {
            entries[n].key = "lna_state_b";
            snprintf(entries[n].value, sizeof(entries[n].value), "-1");
        } else {
            GetWindowTextA(g_hSetLnaB, buf, sizeof(buf));
            entries[n].key = "lna_state_b";
            snprintf(entries[n].value, sizeof(entries[n].value), "%d", atoi(buf));
        }
        n++;
    }

    /* Sample rate / IF / bandwidth - all three come from one selected
     * VALID_COMBOS entry, so they can never disagree with each other. */
    sel = SendMessageA(g_hSetRateCombo, CB_GETCURSEL, 0, 0);
    if (sel != CB_ERR) {
        char sel_text[128];
        int i;
        SendMessageA(g_hSetRateCombo, CB_GETLBTEXT, (WPARAM)sel, (LPARAM)sel_text);
        for (i = 0; i < NUM_VALID_COMBOS; i++) {
            if (strcmp(VALID_COMBOS[i].note, sel_text) != 0) continue;
            entries[n].key = "sample_rate_msps";
            snprintf(entries[n].value, sizeof(entries[n].value), "%.3g",
                     VALID_COMBOS[i].adc_rate_hz / 1e6);
            n++;
            entries[n].key = "if_khz";
            snprintf(entries[n].value, sizeof(entries[n].value), "%d",
                     VALID_COMBOS[i].if_khz);
            n++;
            entries[n].key = "bw_khz";
            snprintf(entries[n].value, sizeof(entries[n].value), "%d",
                     VALID_COMBOS[i].bw_khz);
            n++;
            break;
        }
    }

    GetWindowTextA(g_hSetDuration, buf, sizeof(buf));
    entries[n].key = "duration";
    format_duration_hms(parse_duration_hms(buf), entries[n].value,
                         sizeof(entries[n].value));
    n++;

    {
        static const int decim_values[6] = { 1, 2, 4, 8, 16, 32 };
        LRESULT dsel = SendMessageA(g_hSetDecim, CB_GETCURSEL, 0, 0);
        int dval = (dsel != CB_ERR && dsel >= 0 && dsel < 6)
                       ? decim_values[dsel] : 1;
        entries[n].key = "decimation";
        snprintf(entries[n].value, sizeof(entries[n].value), "%d", dval);
        n++;
    }

    if (single_b) {
        /* Tuner 2 has no selectable port - always a fixed 50-ohm SMA
         * input, see apply_antenna_and_biast()'s handling of this same
         * hardware constraint.                                          */
        entries[n].key = "antenna";
        snprintf(entries[n].value, sizeof(entries[n].value), "50ohm");
        n++;
    } else {
        GetWindowTextA(g_hSetDualT1Antenna, buf, sizeof(buf));
        entries[n].key = "antenna";
        snprintf(entries[n].value, sizeof(entries[n].value), "%s", buf);
        n++;
    }

    {
        static const char *fmt_names[5] = { "linrad", "wavviewdx", "sdruno", "sdrconnect", "winrad" };
        LRESULT fsel = SendMessageA(g_hSetFormat, CB_GETCURSEL, 0, 0);
        if (fsel == CB_ERR) fsel = 0;
        entries[n].key = "output_format";
        snprintf(entries[n].value, sizeof(entries[n].value), "%s",
                 fmt_names[fsel < 5 ? fsel : 0]);
        n++;
    }

    {
        LRESULT lsel = g_hSetLargeMode
                       ? SendMessageA(g_hSetLargeMode, CB_GETCURSEL, 0, 0)
                       : LARGE_FILE_SPLIT;
        if (lsel == CB_ERR) lsel = LARGE_FILE_SPLIT;
        entries[n].key = "large_file_mode";
        snprintf(entries[n].value, sizeof(entries[n].value), "%d", (int)lsel);
        n++;
    }

    GetWindowTextA(g_hSetPath, buf, sizeof(buf));
    entries[n].key = "recording_path";
    snprintf(entries[n].value, sizeof(entries[n].value), "%s", buf);
    n++;

    GetWindowTextA(g_hSetPpm, buf, sizeof(buf));
    entries[n].key = "ppm";
    snprintf(entries[n].value, sizeof(entries[n].value), "%.6g", atof(buf));
    n++;

    entries[n].key = "dc_correct";
    snprintf(entries[n].value, sizeof(entries[n].value), "%d",
             SendMessageA(g_hSetDc, BM_GETCHECK, 0, 0) == BST_CHECKED ? 1 : 0);
    n++;

    entries[n].key = "iq_correct";
    snprintf(entries[n].value, sizeof(entries[n].value), "%d",
             SendMessageA(g_hSetIq, BM_GETCHECK, 0, 0) == BST_CHECKED ? 1 : 0);
    n++;

    entries[n].key = "notch_rf";
    snprintf(entries[n].value, sizeof(entries[n].value), "%d",
             SendMessageA(g_hSetNotchRf, BM_GETCHECK, 0, 0) == BST_CHECKED ? 1 : 0);
    n++;

    entries[n].key = "notch_dab";
    snprintf(entries[n].value, sizeof(entries[n].value), "%d",
             SendMessageA(g_hSetNotchDab, BM_GETCHECK, 0, 0) == BST_CHECKED ? 1 : 0);
    n++;

    entries[n].key = "bias_t";
    snprintf(entries[n].value, sizeof(entries[n].value), "%d",
             SendMessageA(g_hSetBiasT, BM_GETCHECK, 0, 0) == BST_CHECKED ? 1 : 0);
    n++;

    entries[n].key = "hiz_notch";
    snprintf(entries[n].value, sizeof(entries[n].value), "%d",
             SendMessageA(g_hSetHiz, BM_GETCHECK, 0, 0) == BST_CHECKED ? 1 : 0);
    n++;

    {
        static const int hdr_bw_values[4] = { 200, 500, 1200, 1700 };
        LRESULT hsel = SendMessageA(g_hSetHdrBw, CB_GETCURSEL, 0, 0);
        int hval = (hsel != CB_ERR && hsel >= 0 && hsel < 4)
                       ? hdr_bw_values[hsel] : 1700;
        entries[n].key = "hdr_bw_khz";
        snprintf(entries[n].value, sizeof(entries[n].value), "%d", hval);
        n++;
    }

    /* Tuner B: AGC/DC/IQ/notch "same as A" checkbox covers all five
     * underlying _b sentinel fields at once (-1 = inherit from A, same
     * pattern as the existing gain_reduction_b / lna_state_b fields).   */
    if (SendMessageA(g_hSetBSameCorr, BM_GETCHECK, 0, 0) == BST_CHECKED) {
        entries[n].key = "agc_enable_b";   snprintf(entries[n].value, sizeof(entries[n].value), "-1"); n++;
        entries[n].key = "dc_correct_b";   snprintf(entries[n].value, sizeof(entries[n].value), "-1"); n++;
        entries[n].key = "iq_correct_b";   snprintf(entries[n].value, sizeof(entries[n].value), "-1"); n++;
        entries[n].key = "notch_rf_b";     snprintf(entries[n].value, sizeof(entries[n].value), "-1"); n++;
        entries[n].key = "notch_dab_b";    snprintf(entries[n].value, sizeof(entries[n].value), "-1"); n++;
    } else {
        /* Unchecked: read Tuner B's own controls, now that they actually
         * exist - previously this copied Tuner A's current values here
         * instead, which is exactly what silently collapsed a
         * deliberately different Tuner B setting back onto A on every
         * save, even ones unrelated to this checkbox.                    */
        int agc_b  = SendMessageA(g_hSetAgcB,      BM_GETCHECK, 0, 0) == BST_CHECKED;
        int dc_b   = SendMessageA(g_hSetDcB,       BM_GETCHECK, 0, 0) == BST_CHECKED;
        int iq_b   = SendMessageA(g_hSetIqB,       BM_GETCHECK, 0, 0) == BST_CHECKED;
        int rf_b   = SendMessageA(g_hSetNotchRfB,  BM_GETCHECK, 0, 0) == BST_CHECKED;
        int dab_b  = SendMessageA(g_hSetNotchDabB, BM_GETCHECK, 0, 0) == BST_CHECKED;
        entries[n].key = "agc_enable_b"; snprintf(entries[n].value, sizeof(entries[n].value), "%d", agc_b); n++;
        entries[n].key = "dc_correct_b"; snprintf(entries[n].value, sizeof(entries[n].value), "%d", dc_b);  n++;
        entries[n].key = "iq_correct_b"; snprintf(entries[n].value, sizeof(entries[n].value), "%d", iq_b);  n++;
        entries[n].key = "notch_rf_b";   snprintf(entries[n].value, sizeof(entries[n].value), "%d", rf_b);  n++;
        entries[n].key = "notch_dab_b";  snprintf(entries[n].value, sizeof(entries[n].value), "%d", dab_b); n++;
    }

    GetWindowTextA(g_hSetRingSec, buf, sizeof(buf));
    entries[n].key = "ring_buffer_sec";
    snprintf(entries[n].value, sizeof(entries[n].value), "%d", atoi(buf));
    n++;

    entries[n].key = "spinup_enable";
    snprintf(entries[n].value, sizeof(entries[n].value), "%d",
             SendMessageA(g_hSetSpinupEn, BM_GETCHECK, 0, 0) == BST_CHECKED ? 1 : 0);
    n++;

    {
        static const int spinup_values[3] = { 1024 * 1024, 4 * 1024 * 1024, 8 * 1024 * 1024 };
        LRESULT ssel = SendMessageA(g_hSetSpinupBytes, CB_GETCURSEL, 0, 0);
        int sval = (ssel != CB_ERR && ssel >= 0 && ssel < 3)
                       ? spinup_values[ssel] : 1024 * 1024;
        entries[n].key = "spinup_bytes";
        snprintf(entries[n].value, sizeof(entries[n].value), "%d", sval);
        n++;
    }

    {
        static const int mon_int_values[3] = { 200, 500, 1000 };
        LRESULT msel = SendMessageA(g_hSetMonInterval, CB_GETCURSEL, 0, 0);
        int mval = (msel != CB_ERR && msel >= 0 && msel < 3)
                       ? mon_int_values[msel] : 500;
        entries[n].key = "monitor_interval_ms";
        snprintf(entries[n].value, sizeof(entries[n].value), "%d", mval);
        n++;
    }

    entries[n].key = "monitor_smeter_mode";
    snprintf(entries[n].value, sizeof(entries[n].value), "%d",
             SendMessageA(g_hSetSMeterMode, CB_GETCURSEL, 0, 0) == 1 ? 1 : 0);
    n++;

    GetWindowTextA(g_hSetSMeterCal, buf, sizeof(buf));
    entries[n].key = "monitor_smeter_cal_offset";
    snprintf(entries[n].value, sizeof(entries[n].value), "%.6g", atof(buf));
    n++;

    entries[n].key = "use_utc";
    snprintf(entries[n].value, sizeof(entries[n].value), "%d",
             SendMessageA(g_hSetUseUtc, BM_GETCHECK, 0, 0) == BST_CHECKED ? 1 : 0);
    n++;

    entries[n].key = "show_clock";
    snprintf(entries[n].value, sizeof(entries[n].value), "%d",
             SendMessageA(g_hSetShowClock, BM_GETCHECK, 0, 0) == BST_CHECKED ? 1 : 0);
    n++;

    entries[n].key = "verbose";
    snprintf(entries[n].value, sizeof(entries[n].value), "%d",
             SendMessageA(g_hSetVerbose, BM_GETCHECK, 0, 0) == BST_CHECKED ? 1 : 0);
    n++;

    entries[n].key = "log_auto_save";
    snprintf(entries[n].value, sizeof(entries[n].value), "%d",
             SendMessageA(g_hSetLogAutoSave, BM_GETCHECK, 0, 0) == BST_CHECKED ? 1 : 0);
    n++;

    GetWindowTextA(g_hSetLatitude, buf, sizeof(buf));
    entries[n].key = "latitude";
    snprintf(entries[n].value, sizeof(entries[n].value), "%.6f", atof(buf));
    n++;

    GetWindowTextA(g_hSetLongitude, buf, sizeof(buf));
    entries[n].key = "longitude";
    snprintf(entries[n].value, sizeof(entries[n].value), "%.6f", atof(buf));
    n++;

    entries[n].key = "show_sun_times";
    snprintf(entries[n].value, sizeof(entries[n].value), "%d",
             SendMessageA(g_hSetShowSun, BM_GETCHECK, 0, 0) == BST_CHECKED ? 1 : 0);
    n++;

    {
        LRESULT msel = SendMessageA(g_hSetMeterStyle, CB_GETCURSEL, 0, 0);
        entries[n].key = "meter_style";
        snprintf(entries[n].value, sizeof(entries[n].value), "%d",
                 (int)(msel == CB_ERR ? 0 : msel));
        n++;
    }

    GetWindowTextA(g_hSetHttpPort, buf, sizeof(buf));
    entries[n].key = "http_port";
    snprintf(entries[n].value, sizeof(entries[n].value), "%d", atoi(buf));
    n++;

    GetWindowTextA(g_hSetHttpInterval, buf, sizeof(buf));
    entries[n].key = "http_interval_ms";
    snprintf(entries[n].value, sizeof(entries[n].value), "%d", atoi(buf));
    n++;

    entries[n].key = "pipe_enable";
    snprintf(entries[n].value, sizeof(entries[n].value), "%d",
             SendMessageA(g_hSetPipeEn, BM_GETCHECK, 0, 0) == BST_CHECKED ? 1 : 0);
    n++;

    GetWindowTextA(g_hSetPipeName, buf, sizeof(buf));
    entries[n].key = "pipe_name";
    snprintf(entries[n].value, sizeof(entries[n].value), "%s", buf);
    n++;

    /* schedule_only / hourly_enable are deliberately never written here.
     * These two checkboxes are a pure mode selector now - exactly one is
     * always checked (see the radio-pair click handling in
     * settings_wndproc) - not an enable switch in either direction.
     * Arming and disarming is entirely the main window Timer button's
     * job; Settings only ever records which mode is selected, via
     * timer_last_mode below, for that button to read the next time it's
     * pressed from OFF. Both modes always repeat nightly once armed, so
     * there's no separate repeat flag to write here anymore either.       */

    GetWindowTextA(g_hSetHourlyWin, buf, sizeof(buf));
    entries[n].key = "hourly_window_min";
    snprintf(entries[n].value, sizeof(entries[n].value), "%d", atoi(buf));
    n++;

    GetWindowTextA(g_hSetHourlyStart, buf, sizeof(buf));
    entries[n].key = "hourly_start";
    snprintf(entries[n].value, sizeof(entries[n].value), "%s", buf);
    n++;

    GetWindowTextA(g_hSetHourlyStop, buf, sizeof(buf));
    entries[n].key = "hourly_stop";
    snprintf(entries[n].value, sizeof(entries[n].value), "%s", buf);
    n++;

    {
        int hourly_selected = SendMessageA(g_hSetHourlyEn, BM_GETCHECK, 0, 0) == BST_CHECKED;

        entries[n].key = "timer_last_mode";
        snprintf(entries[n].value, sizeof(entries[n].value), "%s",
                 hourly_selected ? "hourly" : "schedule");
        n++;

        /* Timer is already armed - switch which mode is actually active to
         * match the new selection, rather than leaving whichever one was
         * armed before Settings was opened. Settings Save still never
         * arms Timer from OFF (that stays the main window button's job
         * alone, unchanged) - this only applies when it's already on.   */
        if (g_state.cfg.schedule_only || g_state.cfg.hourly_enable) {
            entries[n].key = "schedule_only";
            snprintf(entries[n].value, sizeof(entries[n].value), "%d",
                     hourly_selected ? 0 : 1);
            n++;
            entries[n].key = "hourly_enable";
            snprintf(entries[n].value, sizeof(entries[n].value), "%d",
                     hourly_selected ? 1 : 0);
            n++;
        }
    }

    /* Flush whichever entry is currently on screen into the array before
     * writing it out - otherwise navigating Add/Prev/Next but never
     * clicking off the last-viewed entry would silently drop its edits. */
    settings_schedule_save_current_to_array();
    /* Schedule entries are a variable-length list (up to 32, each with
     * several fields) - doesn't fit the fixed-size entries[] array above,
     * which is sized for a bounded set of scalar settings. Rewritten via
     * its own dedicated pass instead, before the scalar fields below so
     * both land in the same Settings save.                              */
    ini_rewrite_schedule("duodx.ini", g_settings_schedule, g_settings_schedule_count);

    {
        LRESULT sel = SendMessageA(g_hSetColorScheme, CB_GETCURSEL, 0, 0);
        const char *new_scheme = sel == 1 ? "grey" : "navy";
        entries[n].key = "color_scheme";
        snprintf(entries[n].value, sizeof(entries[n].value), "%s", new_scheme);
        n++;
        if (strcmp(new_scheme, g_state.cfg.color_scheme) != 0)
            LOG_WARN("Colour scheme changed to '%s' - restart DuoDX for it "
                     "to take effect (applied once at startup, not live).",
                     new_scheme);
    }

    {
        int i;
        for (i = 0; i < n; i++) entries[i].applied = 0;
    }

    if (ini_patch_values("duodx.ini", entries, n)) {
        LOG_OK("Settings: duodx.ini updated.");
        /* Reflect the change immediately, without needing an app restart. */
        config_set_defaults(&g_state.cfg);
        config_load_ini(&g_state.cfg, "duodx.ini");
        /* config_load_ini() always repopulates schedule_count from however
         * many schedule_N_... entries are physically present in the ini,
         * regardless of schedule_only - recording_worker()'s startup only
         * zeroes it out once, at the very start of a session, specifically
         * so a disabled schedule's leftover entries are inert. Reloading
         * config here (which happens on every Settings save, including
         * while listening or recording) undoes that unless it's reapplied,
         * which meant any recording running after a mid-session Settings
         * save would find schedule_count back at its raw file value and
         * wrongly roll into "waiting for the next schedule entry" once it
         * finished - even with Schedule: OFF and nothing actually armed.
         * Same fix already applied at the nightly schedule-reload day
         * rollover, for the same reason.                                 */
        if (!g_state.cfg.schedule_only)
            g_state.cfg.schedule_count = 0;
        monitor_sync_button_label();
        if (g_hBtnSchedToggle) {
            SetWindowTextA(g_hBtnSchedToggle,
                (g_state.cfg.schedule_only || g_state.cfg.hourly_enable)
                    ? "Timer: ON" : "Timer: OFF");
            InvalidateRect(g_hBtnSchedToggle, NULL, TRUE);
        }
        gui_refresh_idle_timer_text();
        /* expected_output_rate_hz (and therefore the Coverage readout) is
         * normally only computed by validate_config() when a session
         * actually starts - meaning a changed IF/BW/sample-rate combo
         * wouldn't be reflected anywhere until Record or Monitor was
         * pressed. Validating here, and recomputing the coverage text
         * that depends on it, means the display catches up immediately
         * instead of silently showing the previous combo's numbers.    */
        validate_config(&g_state.cfg);
        if (!g_worker_active) {
            /* Fully idle right now - safe to refresh the coverage display
             * and show/hide the monitor bar immediately. If a session is
             * active, the monitor bar change is deliberately left alone:
             * the Monitor button lives inside that bar, and hiding it out
             * from under a running listening session would strand the
             * only way to cancel it. WM_APP_DONE picks up that change
             * once the session actually ends instead.                    */
            gui_compute_cf_text(g_ui.freq, sizeof(g_ui.freq));
            gui_compute_coverage_span(g_ui.span, sizeof(g_ui.span));
            gui_refresh_monitor_bar_visibility();
            /* HDR: ON/OFF is normally only recalculated by the periodic
             * monitor thread, which only runs during an active session -
             * without this, toggling HDR in Settings while idle left the
             * indicator showing the previous state until a session
             * actually started, even though the config itself was
             * already correct.                                          */
            g_ui.hdr_on = g_state.cfg.hdr_enable ? 1 : 0;
            if (g_hwnd) InvalidateRect(g_hwnd, NULL, FALSE);
        }
        if (g_state.listening || g_state.stream_running) {
            int new_dual = g_state.cfg.dual_channel;
            int new_single_b = !strcmp(g_state.cfg.rspduo_single_tuner, "B");
            int freq_b_changed = old_master_slave_active &&
                    fabs(g_state.cfg.freq_b_hz - old_freq_b_hz) > 0.5;
            int freq_a_changed =
                    fabs(g_state.cfg.frequency_hz - old_frequency_hz) > 0.5;
            int combo_changed = (g_state.cfg.if_khz != old_if_khz) ||
                    (g_state.cfg.bw_khz != old_bw_khz) ||
                    (fabs(g_state.cfg.sample_rate_hz - old_sample_rate_hz) > 0.5);
            if (new_dual != old_dual || new_single_b != old_single_b ||
                    freq_b_changed || freq_a_changed || combo_changed) {
                /* Which physical tuner(s) are active just changed - this
                 * needs the same device re-Init as sample rate, dual
                 * channel, and HDR already require, so the running
                 * session can't just carry on with gain/antenna nudges
                 * the way it does for changes that genuinely can be
                 * applied live. Stop cleanly here rather than silently
                 * leaving the old tuner's stream running under the new
                 * selection's label - Monitor needs pressing again to
                 * pick up the change.
                 *
                 * Tuner B's frequency is included here too: it lives
                 * entirely in the separate slave process, started once
                 * with that frequency on its command line - there's no
                 * live-retune path to it at all, unlike Tuner A's gain/
                 * antenna just below. Without this check, saving a new
                 * Tuner B frequency here updated the display and ini but
                 * left the slave silently still streaming its old one,
                 * with everything else (coverage text, level meter
                 * scaling) now assuming the new frequency instead.
                 *
                 * Same reasoning for Tuner A's own CF and the Sample
                 * Rate / IF / Bandwidth combo: neither has a live-retune
                 * path either (only gain/LNA/antenna do, just below).
                 * config_load_ini() a few lines up reloads g_state.cfg
                 * unconditionally, live session or not, so without this
                 * check a CF or combo edit saved while already listening
                 * silently changed what the software believed it was
                 * tuned to - shifting the monitor's demod point within
                 * the still-unmoved hardware capture - without the
                 * hardware, the on-screen absolute frequency, or the
                 * coverage text actually following it: exactly the
                 * "audio jumped to a different station at the same
                 * displayed frequency" symptom this was reported as.    */
                LOG_WARN("A setting that needs the device reopened (tuner "
                         "selection, frequency, or sample rate/IF/BW) just "
                         "changed - stopping the current Listening session "
                         "so it can restart correctly. Press the Tuner A / "
                         "Tuner B button again to resume.");
                gui_stop_session(0);
            } else {
                /* Gain/LNA and antenna are both worth pushing live here,
                 * since that's the whole point of listening. Sample rate,
                 * dual channel, and HDR still only take effect on the next
                 * Record/restart - those require a device re-Init, unlike
                 * gain and antenna which the SDRplay API supports updating
                 * on an already-running stream.                          */
                gui_apply_live_gain(&g_state);
                apply_antenna_and_biast(&g_state);
            }
        }
    } else {
        LOG_ERROR("Settings: could not write duodx.ini (error %lu).",
                  GetLastError());
        MessageBoxA(g_hSettingsWnd,
                    "Could not save duodx.ini. Check the file isn't "
                    "read-only or open in another program.",
                    "DuoDX Settings", MB_ICONERROR);
    }
}

static void settings_select_tab(int tab_idx);

/* Frequency-range helper for the Calculate button: given a start/end
 * range typed into g_hSetRangeStart/g_hSetRangeEnd (kHz), works out and
 * applies the centre frequency, IF bandwidth, and sample rate the same
 * way the User Guide's Appendix A.1 walks through by hand - smallest
 * valid bw_khz that's at least as wide as the span, then the lowest
 * sample rate that offers it. Zero-IF only; updates g_settings_cfg and
 * the on-screen Frequency/Sample-Rate-IF-BW fields directly, the same
 * fields Save later reads from, so nothing else needs to change to pick
 * this up. Leaves a one-line result (or error) in g_hSetRangeHint. */
static void settings_calc_range(void)
{
    char sbuf[32], ebuf[32], hint[260];
    double start_mhz, end_mhz, start_khz, end_khz, span_khz, centre_khz;
    static const int ZIF_BW_LIST[] = {200, 300, 600, 1536, 5000, 6000, 7000, 8000};
    int i, chosen_bw = 0, chosen_if = 0, dual;
    double chosen_adc_hz = 0.0, chosen_output_hz = 0.0;

    if (!g_hSetRangeStart || !g_hSetRangeEnd || !g_hSetRangeHint) return;

    /* Dual-tuner mode (both tuners checked) shares one ADC between the two
     * tuners, which restricts the valid combinations to the small
     * dual_ok=1 subset of VALID_COMBOS - all Low-IF (1620/2048 kHz), none
     * Zero-IF. Searching the normal Zero-IF list here would silently set
     * a combination that only fails later at Record, since validate_config
     * checks dual_ok separately from everything this function touches. */
    dual = (SendMessageA(g_hSetTuner1En, BM_GETCHECK, 0, 0) == BST_CHECKED) &&
           (SendMessageA(g_hSetTuner2En, BM_GETCHECK, 0, 0) == BST_CHECKED);

    GetWindowTextA(g_hSetRangeStart, sbuf, sizeof(sbuf));
    GetWindowTextA(g_hSetRangeEnd, ebuf, sizeof(ebuf));
    start_mhz = atof(sbuf);
    end_mhz = atof(ebuf);
    /* MHz to match the Frequency field's own units above, converted to
     * kHz here since everything below - BW_LIST, VALID_COMBOS - naturally
     * works in kHz and stays that way regardless of the UI's units. */
    start_khz = start_mhz * 1000.0;
    end_khz = end_mhz * 1000.0;

    if (start_mhz <= 0.0 || end_mhz <= 0.0 || end_mhz <= start_mhz) {
        SetWindowTextA(g_hSetRangeHint,
            "Enter a start and end frequency in MHz, with end greater than start.");
        InvalidateRect(g_hSetRangeHint, NULL, TRUE);
        return;
    }

    span_khz = end_khz - start_khz;
    centre_khz = (start_khz + end_khz) / 2.0;

    /* SDRplay hardware doesn't tune above ~2 GHz on any model, so a
     * computed centre beyond that is never a genuine request - almost
     * always it's kHz or Hz typed into this MHz field instead (e.g. 86000
     * for 86 MHz, rather than the 86.0 this field actually wants).
     * Catching it here gives a specific, actionable hint instead of just
     * the generic "span too wide" message a mis-scaled entry would
     * otherwise also trigger, since the resulting (bogus) span is
     * enormous either way. */
    if (centre_khz > 2000000.0) {
        snprintf(hint, sizeof(hint),
            "That's ~%.0f MHz - beyond any SDRplay tuning range. Did you enter kHz or Hz instead of "
            "MHz? For 86.0-88.7 MHz, enter 86.0 to 88.7.",
            centre_khz / 1000.0);
        SetWindowTextA(g_hSetRangeHint, hint);
        InvalidateRect(g_hSetRangeHint, NULL, TRUE);
        return;
    }

    if (dual) {
        /* Dual-tuner mode: search only the dual_ok combos directly,
         * rather than a fixed BW list - the smallest bw_khz among them
         * that's still >= span_khz, then the lowest ADC rate offering
         * that bw_khz. All dual_ok entries happen to be if_khz=1620 or
         * 2048; 1620 covers every bandwidth option (200/300/600/1536) at
         * 6 Msps, while 2048 only offers 1536 at 8 Msps - so for any
         * bandwidth both support, 1620's lower rate always wins on its
         * own via the "lowest ADC rate" comparison below, with no need
         * to special-case a preference between them. */
        double max_dual_bw = 0.0;
        for (i = 0; i < NUM_VALID_COMBOS; i++) {
            const ValidCombo *c = &VALID_COMBOS[i];
            if (!c->dual_ok) continue;
            if (c->bw_khz > max_dual_bw) max_dual_bw = c->bw_khz;
        }
        if (span_khz > max_dual_bw) {
            snprintf(hint, sizeof(hint),
                "%.3f MHz span exceeds dual-tuner mode's %.0f kHz max (Low-IF only - see Appendix "
                "A.2). Uncheck Tuner 2 for wider Zero-IF options, or split into sessions.",
                span_khz / 1000.0, max_dual_bw);
            SetWindowTextA(g_hSetRangeHint, hint);
            InvalidateRect(g_hSetRangeHint, NULL, TRUE);
            return;
        }
        for (i = 0; i < NUM_VALID_COMBOS; i++) {
            const ValidCombo *c = &VALID_COMBOS[i];
            if (!c->dual_ok || (double)c->bw_khz < span_khz) continue;
            if (chosen_adc_hz == 0.0 || c->bw_khz < chosen_bw ||
                    (c->bw_khz == chosen_bw && c->adc_rate_hz < chosen_adc_hz)) {
                chosen_bw = c->bw_khz;
                chosen_if = c->if_khz;
                chosen_adc_hz = c->adc_rate_hz;
                chosen_output_hz = c->output_rate_hz;
            }
        }
    } else {
        if (span_khz > 8000.0) {
            snprintf(hint, sizeof(hint),
                "%.3f MHz span exceeds the 8 MHz max - split into multiple sessions (Appendix A.1).",
                span_khz / 1000.0);
            SetWindowTextA(g_hSetRangeHint, hint);
            InvalidateRect(g_hSetRangeHint, NULL, TRUE);
            return;
        }

        /* Smallest valid bw_khz that is >= span_khz. */
        for (i = 0; i < (int)(sizeof(ZIF_BW_LIST) / sizeof(ZIF_BW_LIST[0])); i++) {
            if ((double)ZIF_BW_LIST[i] >= span_khz) {
                chosen_bw = ZIF_BW_LIST[i];
                break;
            }
        }
        chosen_if = 0;

        /* Lowest Zero-IF sample rate offering that bw_khz. */
        for (i = 0; i < NUM_VALID_COMBOS; i++) {
            const ValidCombo *c = &VALID_COMBOS[i];
            if (c->if_khz != 0 || c->bw_khz != chosen_bw) continue;
            if (chosen_adc_hz == 0.0 || c->adc_rate_hz < chosen_adc_hz) {
                chosen_adc_hz = c->adc_rate_hz;
                chosen_output_hz = c->output_rate_hz;
            }
        }
    }

    if (chosen_adc_hz == 0.0) {
        /* Shouldn't happen - every reachable bandwidth has at least one
         * matching combo - but fail safely rather than apply a zero rate. */
        SetWindowTextA(g_hSetRangeHint, "Could not find a matching combination - please check the range.");
        InvalidateRect(g_hSetRangeHint, NULL, TRUE);
        return;
    }

    g_settings_cfg.frequency_hz = centre_khz * 1000.0;
    g_settings_cfg.if_khz = chosen_if;
    g_settings_cfg.bw_khz = chosen_bw;
    g_settings_cfg.sample_rate_hz = chosen_adc_hz;

    /* Software decimation can shrink the file well below the base output
     * rate without losing any real content, as long as the decimated
     * output rate doesn't drop below the filter's own bandwidth (that
     * would start clipping the filter's own passband, not just trimming
     * oversampling headroom - see A.7's own explanation of this same
     * rule). Divides chosen_output_hz - the rate AFTER any internal Low-IF
     * decimation a dual-mode combo already applies - not chosen_adc_hz,
     * since that's the actual stream rate the decimation key divides
     * further; for Zero-IF the two are the same value anyway. Picks the
     * largest valid factor (1/2/4/8/16/32) that still keeps the result
     * >= chosen_bw, so a narrow filter on a wide base rate - the common
     * case here - ends up close to its own true size rather than
     * needlessly inheriting the full base rate. */
    {
        static const int DECIM_LIST[] = {32, 16, 8, 4, 2, 1};
        double bw_hz = (double)chosen_bw * 1000.0;
        int d, chosen_decim = 1;
        for (d = 0; d < (int)(sizeof(DECIM_LIST) / sizeof(DECIM_LIST[0])); d++) {
            if (chosen_output_hz / (double)DECIM_LIST[d] >= bw_hz) {
                chosen_decim = DECIM_LIST[d];
                break;
            }
        }
        g_settings_cfg.decimation = chosen_decim;
        if (g_hSetDecim) {
            int idx = 0;
            switch (chosen_decim) {
                case 1:  idx = 0; break;
                case 2:  idx = 1; break;
                case 4:  idx = 2; break;
                case 8:  idx = 3; break;
                case 16: idx = 4; break;
                case 32: idx = 5; break;
            }
            SendMessageA(g_hSetDecim, CB_SETCURSEL, (WPARAM)idx, 0);
        }
    }

    {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.6g", g_settings_cfg.frequency_hz / 1e6);
        SetWindowTextA(g_hSetDualT1Freq, buf);
    }
    settings_select_ratecombo_for_current();

    if (dual) {
        snprintf(hint, sizeof(hint),
            "Tuner A CF=%.6g MHz, IF=%d kHz, BW=%d kHz, SR=%.1f Msps, decim=%d -> %.0f kHz output, "
            "covers %.0f-%.0f kHz. Shared by both tuners - set Tuner B's own frequency separately if "
            "it differs.",
            g_settings_cfg.frequency_hz / 1e6, chosen_if, chosen_bw, chosen_adc_hz / 1e6,
            g_settings_cfg.decimation, chosen_output_hz / g_settings_cfg.decimation / 1e3,
            centre_khz - (double)chosen_bw / 2.0, centre_khz + (double)chosen_bw / 2.0);
    } else {
        snprintf(hint, sizeof(hint),
            "CF=%.6g MHz, BW=%d kHz, SR=%.1f Msps, decim=%d -> %.0f kHz output, covers %.0f-%.0f kHz.",
            g_settings_cfg.frequency_hz / 1e6, chosen_bw, chosen_adc_hz / 1e6,
            g_settings_cfg.decimation, chosen_output_hz / g_settings_cfg.decimation / 1e3,
            centre_khz - (double)chosen_bw / 2.0, centre_khz + (double)chosen_bw / 2.0);
    }
    SetWindowTextA(g_hSetRangeHint, hint);
    InvalidateRect(g_hSetRangeHint, NULL, TRUE);
}

static LRESULT CALLBACK settings_wndproc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp)
{
    switch (msg) {
    case WM_CTLCOLOREDIT:
    case WM_CTLCOLORSTATIC:
    case WM_CTLCOLORBTN:
    case WM_CTLCOLORLISTBOX: {
        HDC dc = (HDC)wp;
        if ((HWND)lp == g_hSetHdrHint)
            SetTextColor(dc, g_settings_hdr_freq_valid ? COL_SEG_GREEN : COL_SEG_RED);
        else if ((HWND)lp == g_hSetCoherentLbl)
            SetTextColor(dc, RGB(10, 245, 25));   /* exact match to the main window's COHERENT text */
        else if (settings_check_is_dim((HWND)lp))
            /* Tuner 2 / Same as T1 (GR, LNA) / Tuner 2: same AGC.../ -
             * currently not clickable (see settings_set_check_dim()) but
             * kept enabled so this custom colour actually applies rather
             * than Windows' own bright disabled-text rendering, the same
             * class of problem the edit fields ran into. Matches the
             * system's own disabled-text colour - the same grey already
             * shown by the genuinely disabled, greyed-out edit fields
             * (Gain Reduction etc.) elsewhere in this dialog.             */
            SetTextColor(dc, GetSysColor(COLOR_GRAYTEXT));
        else
            SetTextColor(dc, COL_TEXT);
        SetBkColor(dc, COL_BG);
        return (LRESULT)g_hbrBg;
    }
    case WM_DRAWITEM: {
        LPDRAWITEMSTRUCT di = (LPDRAWITEMSTRUCT)lp;
        if (di->CtlType == ODT_BUTTON &&
                (di->CtlID == IDC_SET_SAVE || di->CtlID == IDC_SET_CANCEL ||
                 di->CtlID == IDC_BTN_RANGE_CALC)) {
            int dis  = (di->itemState & ODS_DISABLED) != 0;
            int down = (di->itemState & ODS_SELECTED) != 0;
            COLORREF face = (di->CtlID == IDC_SET_SAVE)
                ? (dis ? COL_BTN_DIS : (down ? RGB(40, 120, 80) : COL_BTN_START))
                : (dis ? COL_BTN_DIS : (down ? COL_BTN_HOT : COL_BTN_FACE));
            HBRUSH b = CreateSolidBrush(face);
            HPEN   p = CreatePen(PS_SOLID, 1, COL_PANEL_EDGE);
            HGDIOBJ ob, op, of;
            char txt[32];

            FillRect(di->hDC, &di->rcItem, g_hbrBg);
            ob = SelectObject(di->hDC, b);
            op = SelectObject(di->hDC, p);
            RoundRect(di->hDC, di->rcItem.left, di->rcItem.top,
                      di->rcItem.right, di->rcItem.bottom, 5, 5);
            SelectObject(di->hDC, ob);
            SelectObject(di->hDC, op);
            DeleteObject(b);
            DeleteObject(p);

            GetWindowTextA(di->hwndItem, txt, sizeof(txt));
            SetBkMode(di->hDC, TRANSPARENT);
            SetTextColor(di->hDC, dis ? COL_TEXT_DIM : COL_TEXT);
            of = SelectObject(di->hDC, g_hFontUI);
            DrawTextA(di->hDC, txt, -1, &di->rcItem,
                      DT_CENTER | DT_VCENTER | DT_SINGLELINE);
            SelectObject(di->hDC, of);
            return TRUE;
        }
        if (di->CtlType == ODT_BUTTON &&
                (di->CtlID == IDC_SET_TAB0 || di->CtlID == IDC_SET_TAB1 ||
                 di->CtlID == IDC_SET_TAB2 || di->CtlID == IDC_SET_TAB3 ||
                 di->CtlID == IDC_SET_TAB4 || di->CtlID == IDC_SET_TAB5 ||
                 di->CtlID == IDC_SET_TAB6)) {
            /* Custom-drawn in place of a native tab control: comctl32's
             * dark theming doesn't reliably cover the SysTabControl32
             * body, which left a plain light-grey strip across the top
             * of the dialog. Drawing these ourselves keeps every pixel
             * on the same navy/cyan palette as the rest of the app.    */
            int tab_idx;
            switch (di->CtlID) {
                case IDC_SET_TAB0: tab_idx = 0; break;
                case IDC_SET_TAB6: tab_idx = 1; break;   /* Device (new) */
                case IDC_SET_TAB1: tab_idx = 2; break;   /* Recording */
                case IDC_SET_TAB2: tab_idx = 3; break;   /* Monitor */
                case IDC_SET_TAB3: tab_idx = 4; break;   /* Network */
                case IDC_SET_TAB4: tab_idx = 5; break;   /* Schedule */
                default:           tab_idx = 6; break;   /* IDC_SET_TAB5 = Miscellaneous */
            }
            int active  = (tab_idx == g_settings_active_tab);
            COLORREF face = active ? COL_PANEL : COL_BG;
            HBRUSH b = CreateSolidBrush(face);
            HGDIOBJ of;
            RECT r = di->rcItem;
            char txt[32];

            FillRect(di->hDC, &r, b);
            DeleteObject(b);

            if (active) {
                /* Accent bar along the bottom edge marks the selected
                 * tab, echoing COL_ACCENT used for values elsewhere.   */
                RECT under = r;
                under.top = r.bottom - 3;
                HBRUSH ab = CreateSolidBrush(COL_ACCENT);
                FillRect(di->hDC, &under, ab);
                DeleteObject(ab);
            }

            GetWindowTextA(di->hwndItem, txt, sizeof(txt));
            SetBkMode(di->hDC, TRANSPARENT);
            SetTextColor(di->hDC, active ? COL_TEXT : COL_TEXT_DIM);
            of = SelectObject(di->hDC, g_hFontUI);
            DrawTextA(di->hDC, txt, -1, &r, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
            SelectObject(di->hDC, of);
            return TRUE;
        }
        if (di->CtlType == ODT_BUTTON &&
                (di->CtlID == IDC_SET_SCHED_PREV || di->CtlID == IDC_SET_SCHED_NEXT ||
                 di->CtlID == IDC_SET_SCHED_ADD  || di->CtlID == IDC_SET_SCHED_DEL)) {
            int dis  = (di->itemState & ODS_DISABLED) != 0;
            int down = (di->itemState & ODS_SELECTED) != 0;
            COLORREF face = dis ? COL_BTN_DIS : (down ? COL_BTN_HOT : COL_BTN_FACE);
            HBRUSH b = CreateSolidBrush(face);
            HPEN   p = CreatePen(PS_SOLID, 1, COL_PANEL_EDGE);
            HGDIOBJ ob, op, of;
            char txt[32];

            FillRect(di->hDC, &di->rcItem, g_hbrBg);
            ob = SelectObject(di->hDC, b);
            op = SelectObject(di->hDC, p);
            RoundRect(di->hDC, di->rcItem.left, di->rcItem.top,
                      di->rcItem.right, di->rcItem.bottom, 4, 4);
            SelectObject(di->hDC, ob);
            SelectObject(di->hDC, op);
            DeleteObject(b);
            DeleteObject(p);

            GetWindowTextA(di->hwndItem, txt, sizeof(txt));
            SetBkMode(di->hDC, TRANSPARENT);
            SetTextColor(di->hDC, dis ? COL_TEXT_DIM : COL_TEXT);
            of = SelectObject(di->hDC, g_hFontUI);
            DrawTextA(di->hDC, txt, -1, &di->rcItem,
                      DT_CENTER | DT_VCENTER | DT_SINGLELINE);
            SelectObject(di->hDC, of);
            return TRUE;
        }
        return 0;
    }
    case WM_HSCROLL:
        return 0;
    case WM_COMMAND:
        switch (LOWORD(wp)) {
        case IDC_SET_DUALT1_FREQ:
        case IDC_SET_FREQ_B:
            if (HIWORD(wp) == EN_CHANGE) {
                settings_update_hdr_hint();
                settings_update_coherent_indicator();
            }
            return 0;
        case IDC_SET_HOURLY_START:
        case IDC_SET_HOURLY_WIN:
            if (HIWORD(wp) == EN_CHANGE)
                settings_update_hourly_hint();
            return 0;
        case IDC_SET_TUNER1_EN:
            /* At least one tuner must always be selected - clicking the
             * only one currently checked re-checks it rather than
             * leaving neither selected, the same pattern used for the
             * Schedule/Hourly mode selector.                             */
            if (HIWORD(wp) == BN_CLICKED) {
                if (SendMessageA(g_hSetTuner1En, BM_GETCHECK, 0, 0) != BST_CHECKED &&
                        SendMessageA(g_hSetTuner2En, BM_GETCHECK, 0, 0) != BST_CHECKED)
                    SendMessageA(g_hSetTuner1En, BM_SETCHECK, BST_CHECKED, 0);
                settings_update_dual_enable_state();
                settings_update_hdr_hint();
            }
            return 0;
        case IDC_SET_TUNER2_EN:
            if (HIWORD(wp) == BN_CLICKED) {
                if (settings_check_is_dim(g_hSetTuner2En)) {
                    /* Non-RSPduo device - not actually selectable, revert
                     * the click rather than letting it take effect.       */
                    SendMessageA(g_hSetTuner2En, BM_SETCHECK, BST_UNCHECKED, 0);
                    return 0;
                }
                if (SendMessageA(g_hSetTuner2En, BM_GETCHECK, 0, 0) != BST_CHECKED &&
                        SendMessageA(g_hSetTuner1En, BM_GETCHECK, 0, 0) != BST_CHECKED)
                    SendMessageA(g_hSetTuner2En, BM_SETCHECK, BST_CHECKED, 0);
                settings_update_dual_enable_state();
                settings_update_hdr_hint();
            }
            return 0;
        case IDC_SET_HDR:
            if (HIWORD(wp) == BN_CLICKED) {
                settings_update_dual_enable_state();
                settings_update_hdr_hint();
            }
            return 0;
        case IDC_SET_FORMAT:
            if (HIWORD(wp) == CBN_SELCHANGE) {
                /* Winrad's whole reason for existing is RF64 support (SDRuno
                 * doesn't have it - see the OutputFormat enum comment), so
                 * default the choice to RF64 the moment the user actively
                 * picks Winrad here. This only fires on a live selection
                 * change, not when the dialog loads a previously-saved
                 * choice at open time, so it won't fight a value the user
                 * deliberately set back to Split for Winrad in the past.  */
                LRESULT fsel = SendMessageA(g_hSetFormat, CB_GETCURSEL, 0, 0);
                if (fsel == FORMAT_WINRAD && g_hSetLargeMode)
                    SendMessageA(g_hSetLargeMode, CB_SETCURSEL, LARGE_FILE_RF64, 0);
                settings_update_format_dependent_state();
            }
            return 0;
        case IDC_SET_RATECOMBO:
            if (HIWORD(wp) == CBN_SELCHANGE) {
                /* A different Sample Rate/IF/BW combo just got picked -
                 * any decimation left over from a previous combo no
                 * longer means what it used to (it can silently shrink
                 * the coverage well below what the new combo implies),
                 * so reset it back to "1 (off)" rather than carrying it
                 * over. This is a plain CB_SETCURSEL, not user input, so
                 * it doesn't itself generate another CBN_SELCHANGE and
                 * loop back here.                                        */
                if (g_hSetDecim) SendMessageA(g_hSetDecim, CB_SETCURSEL, 0, 0);
            }
            return 0;
        case IDC_SET_GR_B_SAME:
        case IDC_SET_LNA_B_SAME:
            if (HIWORD(wp) == BN_CLICKED) {
                HWND h = (LOWORD(wp) == IDC_SET_GR_B_SAME) ? g_hSetGrBSame : g_hSetLnaBSame;
                if (settings_check_is_dim(h)) {
                    /* Not in dual mode - "Same as T1" has nothing to mean
                     * yet, revert the click rather than letting it take
                     * effect.                                             */
                    SendMessageA(h, BM_SETCHECK,
                                 SendMessageA(h, BM_GETCHECK, 0, 0) == BST_CHECKED
                                     ? BST_UNCHECKED : BST_CHECKED, 0);
                    return 0;
                }
                settings_update_dual_enable_state();
            }
            return 0;
        case IDC_SET_B_SAME_CORR:
            if (HIWORD(wp) == BN_CLICKED) {
                if (settings_check_is_dim(g_hSetBSameCorr)) {
                    /* Same reasoning - not in dual mode, revert the click. */
                    SendMessageA(g_hSetBSameCorr, BM_SETCHECK,
                                 SendMessageA(g_hSetBSameCorr, BM_GETCHECK, 0, 0) == BST_CHECKED
                                     ? BST_UNCHECKED : BST_CHECKED, 0);
                } else {
                    settings_update_dual_enable_state();
                }
            }
            return 0;
        case IDC_SET_SCHED_ONLY:
            /* Pure mode selector now, not an enable switch (see
             * settings_save() - it never writes schedule_only/
             * hourly_enable from these anymore, only timer_last_mode).
             * Behaves like a two-option radio group: exactly one of the
             * two is always selected, so clicking the already-selected
             * one re-selects it rather than leaving both unchecked -
             * there's no third "neither" state to represent here.        */
            if (HIWORD(wp) == BN_CLICKED) {
                if (SendMessageA(g_hSetSchedOnly, BM_GETCHECK, 0, 0) == BST_CHECKED)
                    SendMessageA(g_hSetHourlyEn, BM_SETCHECK, BST_UNCHECKED, 0);
                else
                    SendMessageA(g_hSetSchedOnly, BM_SETCHECK, BST_CHECKED, 0);
            }
            return 0;
        case IDC_SET_HOURLY_EN:
            if (HIWORD(wp) == BN_CLICKED) {
                if (SendMessageA(g_hSetHourlyEn, BM_GETCHECK, 0, 0) == BST_CHECKED)
                    SendMessageA(g_hSetSchedOnly, BM_SETCHECK, BST_UNCHECKED, 0);
                else
                    SendMessageA(g_hSetHourlyEn, BM_SETCHECK, BST_CHECKED, 0);
            }
            return 0;
        case IDC_SET_SCHED_PREV:
            if (HIWORD(wp) == BN_CLICKED) {
                settings_schedule_save_current_to_array();
                if (g_settings_schedule_idx > 0) g_settings_schedule_idx--;
                settings_schedule_load_current_from_array();
            }
            return 0;
        case IDC_SET_SCHED_NEXT:
            if (HIWORD(wp) == BN_CLICKED) {
                settings_schedule_save_current_to_array();
                if (g_settings_schedule_idx < g_settings_schedule_count - 1)
                    g_settings_schedule_idx++;
                settings_schedule_load_current_from_array();
            }
            return 0;
        case IDC_SET_SCHED_ADD:
            if (HIWORD(wp) == BN_CLICKED) {
                settings_schedule_save_current_to_array();
                if (g_settings_schedule_count < MAX_SCHEDULE_ENTRIES) {
                    g_settings_schedule_idx = g_settings_schedule_count;
                    memset(&g_settings_schedule[g_settings_schedule_idx], 0,
                           sizeof(ScheduleEntry));
                    g_settings_schedule_count++;
                    settings_schedule_load_current_from_array();
                }
            }
            return 0;
        case IDC_SET_SCHED_DEL:
            if (HIWORD(wp) == BN_CLICKED && g_settings_schedule_count > 0) {
                int i;
                for (i = g_settings_schedule_idx; i < g_settings_schedule_count - 1; i++)
                    g_settings_schedule[i] = g_settings_schedule[i + 1];
                g_settings_schedule_count--;
                if (g_settings_schedule_idx >= g_settings_schedule_count &&
                        g_settings_schedule_idx > 0)
                    g_settings_schedule_idx--;
                settings_schedule_load_current_from_array();
            }
            return 0;
        case IDC_BTN_RANGE_CALC:
            if (HIWORD(wp) == BN_CLICKED) settings_calc_range();
            return 0;
        case IDC_SET_SAVE:
            settings_save();
            g_ab_auto_pending = 1;   /* re-arm: eligible to auto-enable A=B
                                      * once more, next time Monitor starts,
                                      * if the saved A/B frequencies match. */
            DestroyWindow(hwnd);
            return 0;
        case IDC_SET_CANCEL:
            DestroyWindow(hwnd);
            return 0;
        case IDC_SET_TAB0: case IDC_SET_TAB1: case IDC_SET_TAB2: case IDC_SET_TAB3: case IDC_SET_TAB4: case IDC_SET_TAB5: case IDC_SET_TAB6:
            if (HIWORD(wp) == BN_CLICKED) {
                int tab_idx;
                switch (LOWORD(wp)) {
                    case IDC_SET_TAB0: tab_idx = 0; break;
                    case IDC_SET_TAB6: tab_idx = 1; break;   /* Device (new) */
                    case IDC_SET_TAB1: tab_idx = 2; break;   /* Recording */
                    case IDC_SET_TAB2: tab_idx = 3; break;   /* Monitor */
                    case IDC_SET_TAB3: tab_idx = 4; break;   /* Network */
                    case IDC_SET_TAB4: tab_idx = 5; break;   /* Schedule */
                    default:           tab_idx = 6; break;   /* IDC_SET_TAB5 = Miscellaneous */
                }
                settings_select_tab(tab_idx);
            }
            return 0;
        }
        return 0;
    case WM_CLOSE:
        DestroyWindow(hwnd);
        return 0;
    case WM_DESTROY: {
        /* SetProp entries must be removed before the window goes away or
         * they leak - each tagged control had one added by
         * settings_tag_tab() while the dialog was being built. */
        HWND child = GetWindow(hwnd, GW_CHILD);
        while (child) {
            RemovePropA(child, SETTINGS_TAB_PROP);
            child = GetWindow(child, GW_HWNDNEXT);
        }
        g_hSettingsWnd = NULL;
        return 0;
    }
    }
    return DefWindowProcA(hwnd, msg, wp, lp);
}

/* Small helper: label + control on one row. */
static HWND settings_mk_label(HWND parent, HINSTANCE hInst, const char *text,
                               int x, int y, int w)
{
    HWND h = CreateWindowExA(0, "STATIC", text, WS_CHILD | WS_VISIBLE | SS_LEFT,
                              x, y, w, 20, parent, NULL, hInst, NULL);
    if (g_hFontUI) SendMessageA(h, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
    settings_tag_tab(h);
    return h;
}


/* Masked HH:MM:SS entry for the Duration field: digits overwrite in
 * place and the caret auto-advances past the two fixed colons, so
 * typing "094500" produces "09:45:00" without ever needing to type a
 * colon or move the caret by hand - the same idea as the scrollable
 * digit controls used elsewhere in the app, just applied to a plain
 * edit box instead of a custom-drawn control, since this field lives
 * in the Settings dialog alongside other plain edit fields.
 *
 * The field is always exactly 8 characters ("HH:MM:SS"); every input
 * path that could otherwise shrink or corrupt that (paste, cut, the
 * Delete key) is intercepted and re-implemented as an in-place reset
 * to '0' instead, so the fixed-width assumption always holds.        */
#define TIME_EDIT_LEN_PROP  "TimeEditFieldLen"
#define TIME_EDIT_PROC_PROP "TimeEditOrigProc"
#define TIME_EDIT_DUR_PROP  "TimeEditIsDuration"
#define TIME_EDIT_LOCKMIN_PROP "TimeEditLockMinZero"

/* Returns 1 if typing ch at logical position pos (0-based within the
 * field, after any colon has already been skipped over) would keep the
 * value sensible, 0 if it should be rejected outright. Time-of-day
 * fields (Start Time, Session Start/Stop) cap hours at 23, since those
 * are genuinely a time of day and the scheduler compares them against
 * the real clock - typing digits that produced something like 36:75:92
 * previously sailed straight through with no check at all, and the
 * scheduler's arithmetic then just treated it as an arbitrary offset in
 * seconds rather than any sensible time, waiting for a moment that
 * looked nothing like what was actually typed. Duration fields leave
 * hours unrestricted, since a continuous recording can genuinely run
 * past 24 hours. Minutes and seconds are always capped at 0-59 for
 * every field, time-of-day or duration alike, since "75 minutes" is
 * never meaningful within an HH:MM:SS breakdown either way. Some fields
 * (the Hourly Session Start/Stop) go further and lock minutes at 00
 * entirely, since Hourly recordings always happen on the hour regardless
 * of what minute is typed there - letting the field show anything else
 * implied a precision the feature doesn't actually have.                 */
static int time_edit_digit_ok(HWND hwnd, int pos, char ch)
{
    int is_duration = (int)(INT_PTR)GetPropA(hwnd, TIME_EDIT_DUR_PROP);
    int lock_min_zero = (int)(INT_PTR)GetPropA(hwnd, TIME_EDIT_LOCKMIN_PROP);
    int digit = ch - '0';

    if (pos == 0) {
        /* Tens of hours. */
        if (!is_duration && digit > 2) return 0;
        return 1;
    }
    if (pos == 1) {
        /* Units of hours - only limited (to 20-23) once the tens digit
         * already reads '2', and only for time-of-day fields.          */
        if (!is_duration) {
            char buf[16];
            GetWindowTextA(hwnd, buf, sizeof(buf));
            if (buf[0] == '2' && digit > 3) return 0;
        }
        return 1;
    }
    if (pos == 3 || pos == 4) {
        /* Minutes - locked to 00 for fields where it's never meaningful
         * (Hourly Session Start/Stop specifically).                     */
        if (lock_min_zero && digit != 0) return 0;
    }
    if (pos == 3 || pos == 6) {
        /* Tens of minutes / tens of seconds - caps the pair at 59. */
        if (digit > 5) return 0;
    }
    return 1;
}

/* Shared by every HH:MM:SS and HH:MM field in Settings (Duration, the
 * per-entry Schedule Start Time and Duration, and the Hourly Session
 * Start/Stop) - as you type digits, each one overwrites the digit under
 * the cursor and the cursor auto-advances, skipping over the colons,
 * rather than working like a normal free-form text field. field_len is
 * 8 for "HH:MM:SS" (colons at position 2 and 5) or 5 for "HH:MM" (colon
 * at position 2 only) - stored per-control via TIME_EDIT_LEN_PROP so one
 * subclass procedure can serve both formats without needing a copy each.
 * The original window proc is likewise stored per-control (TIME_EDIT_
 * PROC_PROP) rather than in a single global, since multiple fields with
 * this subclass are now active at once.                                  */
static LRESULT CALLBACK time_edit_subclass_proc(HWND hwnd, UINT msg,
                                                  WPARAM wp, LPARAM lp)
{
    WNDPROC orig = (WNDPROC)GetPropA(hwnd, TIME_EDIT_PROC_PROP);
    int field_len = (int)(INT_PTR)GetPropA(hwnd, TIME_EDIT_LEN_PROP);
    int colon_a = 2;                        /* always present */
    int colon_b = (field_len == 8) ? 5 : -1; /* only for HH:MM:SS */

    if (msg == WM_CHAR) {
        char ch = (char)wp;
        DWORD start, end;
        int pos;

        if (ch == '\r' || ch == '\t')
            return CallWindowProcA(orig, hwnd, msg, wp, lp);

        SendMessageA(hwnd, EM_GETSEL, (WPARAM)&start, (LPARAM)&end);
        pos = (int)start;

        if (ch >= '0' && ch <= '9') {
            char buf[2];
            int newpos;
            if (pos == colon_a || pos == colon_b) pos++;  /* typed "on" a colon = next slot */
            if (pos >= field_len) return 0;               /* field already full */
            if (!time_edit_digit_ok(hwnd, pos, ch)) return 0; /* would make an invalid time */
            buf[0] = ch; buf[1] = '\0';
            SendMessageA(hwnd, EM_SETSEL, pos, pos + 1);
            SendMessageA(hwnd, EM_REPLACESEL, TRUE, (LPARAM)buf);
            newpos = pos + 1;
            if (newpos == colon_a || newpos == colon_b) newpos++;
            SendMessageA(hwnd, EM_SETSEL, newpos, newpos);
            return 0;
        }
        if (ch == '\b') {
            int newpos;
            if (pos == 0) return 0;
            newpos = pos - 1;
            if (newpos == colon_a || newpos == colon_b) newpos--;
            SendMessageA(hwnd, EM_SETSEL, newpos, newpos + 1);
            SendMessageA(hwnd, EM_REPLACESEL, TRUE, (LPARAM)"0");
            SendMessageA(hwnd, EM_SETSEL, newpos, newpos);
            return 0;
        }
        return 0;   /* block anything else typed - letters, symbols, colon */
    }

    if (msg == WM_KEYDOWN && wp == VK_DELETE) {
        DWORD start, end;
        int pos;
        SendMessageA(hwnd, EM_GETSEL, (WPARAM)&start, (LPARAM)&end);
        pos = (int)start;
        if (pos == colon_a || pos == colon_b) pos++;
        if (pos < field_len) {
            SendMessageA(hwnd, EM_SETSEL, pos, pos + 1);
            SendMessageA(hwnd, EM_REPLACESEL, TRUE, (LPARAM)"0");
            SendMessageA(hwnd, EM_SETSEL, pos, pos);
        }
        return 0;
    }

    if (msg == WM_PASTE || msg == WM_CUT || msg == WM_CLEAR)
        return 0;   /* would risk shrinking/corrupting the fixed template */

    return CallWindowProcA(orig, hwnd, msg, wp, lp);
}

/* Applies the subclass above to one HH:MM:SS (field_len=8) or HH:MM
 * (field_len=5) edit control. is_duration should be 1 for a duration
 * field (hours unrestricted) or 0 for a time-of-day field (hours capped
 * at 23). lock_min_zero should be 1 to lock the minutes digits at 00
 * (Hourly Session Start/Stop specifically) or 0 to leave minutes freely
 * editable. Safe to call every time the Settings dialog is rebuilt,
 * since its controls (and their HWNDs) are destroyed and recreated
 * fresh each time it's opened.                                          */
static void apply_time_edit_subclass(HWND h, int field_len, int is_duration,
                                       int lock_min_zero)
{
    WNDPROC orig = (WNDPROC)SetWindowLongPtrA(h, GWLP_WNDPROC,
                                    (LONG_PTR)time_edit_subclass_proc);
    SetPropA(h, TIME_EDIT_PROC_PROP, (HANDLE)orig);
    SetPropA(h, TIME_EDIT_LEN_PROP, (HANDLE)(INT_PTR)field_len);
    SetPropA(h, TIME_EDIT_DUR_PROP, (HANDLE)(INT_PTR)is_duration);
    SetPropA(h, TIME_EDIT_LOCKMIN_PROP, (HANDLE)(INT_PTR)lock_min_zero);
}

#define WHEEL_REVERSED_PROC_PROP "WheelReversedOrigProc"

/* Reverses the direction of native mouse-wheel handling on a trackbar
 * (slider) control - scroll up now increases the value, scroll down
 * decreases it, matching the near-universal convention for volume and
 * similar controls. Standard Win32 trackbars do the opposite by default
 * once they have focus: scrolling up moves the thumb toward the LOWER
 * end of its range, which feels backwards for a control framed as "more
 * at the top of the wheel". Achieved by inverting the wheel delta and
 * letting the control's own default proc do the actual position math and
 * fire its usual WM_HSCROLL notification, rather than reimplementing
 * that here - applies equally to the Volume and Low-Cut sliders. */
static LRESULT CALLBACK wheel_reversed_subclass_proc(HWND hwnd, UINT msg,
                                                       WPARAM wp, LPARAM lp)
{
    WNDPROC orig = (WNDPROC)GetPropA(hwnd, WHEEL_REVERSED_PROC_PROP);

    if (msg == WM_MOUSEWHEEL) {
        short delta = (short)HIWORD(wp);
        WPARAM inverted = MAKEWPARAM(LOWORD(wp), (WORD)(short)(-delta));
        return CallWindowProcA(orig, hwnd, msg, inverted, lp);
    }

    return CallWindowProcA(orig, hwnd, msg, wp, lp);
}

static void apply_reversed_wheel_subclass(HWND h)
{
    WNDPROC orig = (WNDPROC)SetWindowLongPtrA(h, GWLP_WNDPROC,
                                    (LONG_PTR)wheel_reversed_subclass_proc);
    SetPropA(h, WHEEL_REVERSED_PROC_PROP, (HANDLE)orig);
}

static HWND settings_mk_edit(HWND parent, HINSTANCE hInst, int id,
                              int x, int y, int w)
{
    HWND h = CreateWindowExA(0, "EDIT", "",
                WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL,
                x, y, w, 22, parent, (HMENU)(INT_PTR)id, hInst, NULL);
    if (g_hFontUI) SendMessageA(h, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
    /* Same fix as settings_mk_check() below, and for the same reason -
     * an edit control under the default visual style ignores
     * WM_CTLCOLOREDIT's custom text colour entirely, which is why
     * neither the original disabled-grey text nor a read-only field's
     * custom dim colour ever actually showed up - the app's requested
     * colour was being silently overridden by the theme regardless of
     * enabled/disabled/read-only state.                                  */
    {
        HMODULE hUx = LoadLibraryA("uxtheme.dll");
        if (hUx) {
            typedef HRESULT (WINAPI *PFN_SWT)(HWND, LPCWSTR, LPCWSTR);
            PFN_SWT pSwt = (PFN_SWT)GetProcAddress(hUx, "SetWindowTheme");
            if (pSwt) pSwt(h, L"", L"");
            FreeLibrary(hUx);
        }
    }
    settings_tag_tab(h);
    return h;
}

static HWND settings_mk_check(HWND parent, HINSTANCE hInst, int id,
                               const char *text, int x, int y, int w)
{
    HWND h = CreateWindowExA(0, "BUTTON", text,
                WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX,
                x, y, w, 20, parent, (HMENU)(INT_PTR)id, hInst, NULL);
    if (g_hFontUI) SendMessageA(h, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
    /* Disabling visual styles on this one control is the standard fix for
     * a checkbox ignoring WM_CTLCOLORBTN's background brush and always
     * painting the bright default system square instead. */
    {
        HMODULE hUx = LoadLibraryA("uxtheme.dll");
        if (hUx) {
            typedef HRESULT (WINAPI *PFN_SWT)(HWND, LPCWSTR, LPCWSTR);
            PFN_SWT pSwt = (PFN_SWT)GetProcAddress(hUx, "SetWindowTheme");
            if (pSwt) pSwt(h, L"", L"");
            FreeLibrary(hUx);
        }
    }
    settings_tag_tab(h);
    return h;
}

/* Applies the OS dark visual style to a control - loaded dynamically so
 * the app still runs fine without uxtheme (just keeps default colours). */
static void settings_apply_dark_theme(HWND h)
{
    HMODULE hUx = LoadLibraryA("uxtheme.dll");
    if (hUx) {
        typedef HRESULT (WINAPI *PFN_SWT)(HWND, LPCWSTR, LPCWSTR);
        PFN_SWT pSwt = (PFN_SWT)GetProcAddress(hUx, "SetWindowTheme");
        if (pSwt) pSwt(h, L"DarkMode_Explorer", NULL);
        FreeLibrary(hUx);
    }
}

/* Shows every control tagged for tab_idx (0-based) and hides every other
 * tagged control. Untagged controls (Save/Cancel, the tab buttons
 * themselves) are never touched here, so they stay visible on every tab. */
static void settings_select_tab(int tab_idx)
{
    HWND child = GetWindow(g_hSettingsWnd, GW_CHILD);
    int i;
    while (child) {
        INT_PTR prop = (INT_PTR)GetPropA(child, SETTINGS_TAB_PROP);
        if (prop != 0)
            ShowWindow(child, (prop - 1 == tab_idx) ? SW_SHOW : SW_HIDE);
        child = GetWindow(child, GW_HWNDNEXT);
    }
    g_settings_active_tab = tab_idx;
    for (i = 0; i < 7; i++)
        if (g_hSetTabBtn[i]) InvalidateRect(g_hSetTabBtn[i], NULL, TRUE);
    if (tab_idx == 0) settings_update_coherent_indicator();
    /* Re-apply the format-dependent override on top of the generic tab
     * show/hide above - settings_select_tab() alone would show this
     * control on Recording regardless of output format, and hide it on
     * every other tab regardless of format too; the format check has to
     * run again after the tab switch to correct for both.               */
    settings_update_format_dependent_state();
}

static void open_settings_dialog(HWND parent)
{
    HINSTANCE hInst = (HINSTANCE)GetWindowLongPtrA(parent, GWLP_HINSTANCE);
    RECT pr;
    int win_w = 660, win_h = 700;
    int x, y0, row, label_w = 150, ctl_x, ctl_w = 130;
    int y_content0, y_r, y_dev, y_rec, y_mon, y_net, y_sched, y_misc, y_max;
    static int class_registered = 0;
    HWND hSave, hCancel;

    if (g_hSettingsWnd) {
        SetForegroundWindow(g_hSettingsWnd);
        return;
    }

    if (!g_worker_active && !g_device_busy) {
        /* Only safe to probe the API when nothing else has it open. If a
         * session (Monitor or Record) is already active, it already set
         * g_last_known_hwVer correctly when it started - re-detecting
         * here would mean two concurrent sdrplay_api_Open()/Close()
         * cycles on the same device from two different threads, which is
         * a much likelier explanation for later "no devices found"
         * failures on the next session start than any driver settling
         * delay. Skip entirely rather than risk that.
         *
         * g_device_busy is checked alongside g_worker_active, not
         * g_worker_active alone: there are several points in the worker
         * thread's teardown where g_worker_active already reads 0 before
         * the device is actually done being torn down (sdrplay_api_Close()
         * etc. still in progress) - g_device_busy is specifically the
         * flag meant to stay true for the device's whole lifecycle,
         * including that tail end. Confirmed as a real crash: pressing
         * Settings in that exact window raced this function's own
         * sdrplay_api_Open() against the worker thread's still-running
         * sdrplay_api_Close(), crashing inside the API DLL itself rather
         * than anywhere in DuoDX's own code.                             */
        refresh_known_device_type();
        if (g_last_known_hwVer != 0 && g_last_known_hwVer != SDRPLAY_RSPduo_ID &&
                (g_state.cfg.dual_channel || !strcmp(g_state.cfg.rspduo_single_tuner, "B"))) {
            IniPatchEntry duo_fix[2];
            int duo_fix_n = 0;
            g_state.cfg.dual_channel = 0;
        strncpy(g_state.cfg.rspduo_single_tuner, "A",
                sizeof(g_state.cfg.rspduo_single_tuner) - 1);
        duo_fix[duo_fix_n].key = "dual_channel";
        snprintf(duo_fix[duo_fix_n].value, sizeof(duo_fix[duo_fix_n].value), "0");
        duo_fix_n++;
        duo_fix[duo_fix_n].key = "rspduo_single_tuner";
        snprintf(duo_fix[duo_fix_n].value, sizeof(duo_fix[duo_fix_n].value), "A");
        duo_fix_n++;
        ini_patch_values("duodx.ini", duo_fix, duo_fix_n);
        LOG_WARN("dual_channel/rspduo_single_tuner=B in duodx.ini looked "
                 "left over from a different (RSPduo) device - reset to "
                 "single-tuner A for this one.");
        }
    }

    if (!class_registered) {
        WNDCLASSA wc;
        INITCOMMONCONTROLSEX icc;
        memset(&wc, 0, sizeof(wc));
        wc.lpfnWndProc   = settings_wndproc;
        wc.hInstance     = hInst;
        wc.lpszClassName = "DuoDXSettingsWindow";
        wc.hbrBackground = g_hbrBg;
        wc.hCursor       = LoadCursorA(NULL, (LPCSTR)IDC_ARROW);
        RegisterClassA(&wc);
        icc.dwSize = sizeof(icc);
        icc.dwICC  = ICC_BAR_CLASSES;
        InitCommonControlsEx(&icc);
        class_registered = 1;
    }

    GetWindowRect(parent, &pr);

    g_hSettingsWnd = CreateWindowExA(WS_EX_DLGMODALFRAME, "DuoDXSettingsWindow",
        "DuoDX Settings",
        WS_POPUP | WS_CAPTION | WS_SYSMENU,
        pr.left + 40, pr.top + 40, win_w, win_h,
        parent, NULL, hInst, NULL);
    if (!g_hSettingsWnd) return;

    x = 16; row = 34;
    ctl_x = x + label_w + 8;

    /* Tab strip across the top, custom owner-drawn (see WM_DRAWITEM) - a
     * native SysTabControl32 was tried first but comctl32's dark theming
     * only reaches the tab labels, not the strip's own body, leaving a
     * plain light-grey bar across the top of an otherwise dark dialog.
     * These are untagged, so settings_select_tab() never hides them.    */
    {
        static const char *tab_names[7] = { "Receiver", "Device", "Recording", "Monitor", "Network", "Schedule", "Miscellaneous" };
        static const int   tab_ids[7] = { IDC_SET_TAB0, IDC_SET_TAB6, IDC_SET_TAB1, IDC_SET_TAB2, IDC_SET_TAB3, IDC_SET_TAB4, IDC_SET_TAB5 };
        int tab_w = (win_w - 16) / 7;
        int i;
        for (i = 0; i < 7; i++) {
            g_hSetTabBtn[i] = CreateWindowExA(0, "BUTTON", tab_names[i],
                        WS_CHILD | WS_VISIBLE | BS_OWNERDRAW,
                        8 + i * tab_w, 8, tab_w, 28,
                        g_hSettingsWnd, (HMENU)(INT_PTR)tab_ids[i], hInst, NULL);
            if (g_hFontUI) SendMessageA(g_hSetTabBtn[i], WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
        }
    }
    y_content0 = 46;

    /* ---------------------------------------------------------------- */
    /* Tab 0: Receiver                                                   */
    /* ---------------------------------------------------------------- */
    g_settings_build_tab = 0;
    y0 = y_content0;

    /* ================================================================ */
    /* Tuners - covers both single- and dual-tuner configuration in one  */
    /* place. Checking one tuner is single-tuner mode; checking both is  */
    /* dual mode (there's no separate Dual Channel checkbox anymore -    */
    /* it's derived from this). Tuner 2 is only ever available on an     */
    /* RSPduo - see settings_update_dual_enable_state().                 */
    /* ================================================================ */
    settings_mk_label(g_hSettingsWnd, hInst, "TUNERS", x, y0, win_w - x - 16);
    y0 += row - 10;

    {
        int row_label_w = 130;
        int t1_x = x + row_label_w, t1_w = 90;
        int t2_x = t1_x + 100, t2_w = 90;
        int same_x = t2_x + 100;

        g_hSetTuner1En = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_TUNER1_EN,
                                            "Tuner 1", t1_x, y0, t1_w + 10);
        g_hSetTuner2En = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_TUNER2_EN,
                                            "Tuner 2", t2_x, y0, t2_w + 10);
        y0 += row;

        settings_mk_label(g_hSettingsWnd, hInst, "Frequency (MHz)", x, y0, row_label_w);
        g_hSetDualT1Freq = settings_mk_edit(g_hSettingsWnd, hInst, IDC_SET_DUALT1_FREQ, t1_x, y0 - 2, t1_w);
        g_hSetFreqB = settings_mk_edit(g_hSettingsWnd, hInst, IDC_SET_FREQ_B, t2_x, y0 - 2, t2_w);
        g_hSetCoherentLbl = settings_mk_label(g_hSettingsWnd, hInst, "COHERENT", same_x, y0 + 3, 110);
        ShowWindow(g_hSetCoherentLbl, SW_HIDE);
        y0 += row;

        settings_mk_label(g_hSettingsWnd, hInst, "Gain Reduction (dB)", x, y0, row_label_w);
        g_hSetDualT1Gr = settings_mk_edit(g_hSettingsWnd, hInst, IDC_SET_DUALT1_GR, t1_x, y0 - 2, t1_w);
        g_hSetGrB = settings_mk_edit(g_hSettingsWnd, hInst, IDC_SET_GR_B, t2_x, y0 - 2, t2_w);
        g_hSetGrBSame = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_GR_B_SAME,
                                           "Same as T1", same_x, y0, 110);
        y0 += row;

        settings_mk_label(g_hSettingsWnd, hInst, "LNA State", x, y0, row_label_w);
        g_hSetDualT1Lna = settings_mk_edit(g_hSettingsWnd, hInst, IDC_SET_DUALT1_LNA, t1_x, y0 - 2, t1_w);
        g_hSetLnaB = settings_mk_edit(g_hSettingsWnd, hInst, IDC_SET_LNA_B, t2_x, y0 - 2, t2_w);
        g_hSetLnaBSame = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_LNA_B_SAME,
                                            "Same as T1", same_x, y0, 110);
        y0 += row;

        /* Tuner 1's antenna field covers every compatible radio (A/B/C
         * for RSPdx/RSP2, 50ohm/Hi-Z for RSPduo Tuner 1) - it's the
         * general-purpose antenna selector regardless of which device is
         * actually connected, the same way it always has been. Tuner 2
         * has no selectable port on any device, so there's nothing to
         * put in a second combo for it.                                 */
        settings_mk_label(g_hSettingsWnd, hInst, "Antenna", x, y0, row_label_w);
        g_hSetDualT1Antenna = CreateWindowExA(0, "COMBOBOX", "",
                    WS_CHILD | WS_VISIBLE | WS_BORDER | CBS_DROPDOWN | WS_VSCROLL,
                    t1_x, y0 - 2, t1_w, 22 + 120,
                    g_hSettingsWnd, (HMENU)(INT_PTR)IDC_SET_DUALT1_ANTENNA, hInst, NULL);
        SendMessageA(g_hSetDualT1Antenna, CB_ADDSTRING, 0, (LPARAM)"A");
        SendMessageA(g_hSetDualT1Antenna, CB_ADDSTRING, 0, (LPARAM)"B");
        SendMessageA(g_hSetDualT1Antenna, CB_ADDSTRING, 0, (LPARAM)"C");
        SendMessageA(g_hSetDualT1Antenna, CB_ADDSTRING, 0, (LPARAM)"50ohm");
        SendMessageA(g_hSetDualT1Antenna, CB_ADDSTRING, 0, (LPARAM)"Hi-Z");
        if (g_hFontUI) SendMessageA(g_hSetDualT1Antenna, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
        settings_apply_dark_theme(g_hSetDualT1Antenna);
        settings_tag_tab(g_hSetDualT1Antenna);
        settings_mk_label(g_hSettingsWnd, hInst, "(Tuner 2 has no selectable port)", t2_x, y0 + 3, 220);
        y0 += row + 6;
    }

    /* AGC/DC/IQ/notch/Bias-T/Hi-Z describe Tuner 1 - whether it's running
     * alone or alongside Tuner 2 in dual mode, where the checkbox below
     * gives Tuner 2 the option to copy them rather than needing its own
     * full set.                                                          */
    g_hSetAgc = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_AGC,
                                   "AGC Enable", x, y0, 145);
    g_hSetDc = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_DC,
                                  "DC Correct", x + 150, y0, 145);
    g_hSetIq = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_IQ,
                                  "IQ Correct", x + 300, y0, 145);
    y0 += row;

    g_hSetNotchRf = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_NOTCH_RF,
                                       "RF Notch", x, y0, 145);
    g_hSetNotchDab = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_NOTCH_DAB,
                                        "DAB Notch", x + 150, y0, 145);
    g_hSetBiasT = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_BIAST,
                                     "Bias-T", x + 300, y0, 145);
    y0 += row;

    g_hSetHiz = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_HIZ,
                                   "Hi-Z Notch (RSPduo Tuner 1)", x, y0, 260);
    y0 += row;

    g_hSetBSameCorr = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_B_SAME_CORR,
        "Tuner 2: same AGC/DC/IQ/notch settings as Tuner 1", x, y0, 400);
    y0 += row;

    /* Tuner B's own AGC/DC/IQ/notch controls - only relevant (and only
     * ever shown) when the checkbox above is unchecked; see
     * settings_update_dual_enable_state() for the show/hide logic. Same
     * three-then-two layout as Tuner A's own controls above.             */
    g_hSetAgcB = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_AGC_B,
                                    "AGC Enable", x, y0, 145);
    g_hSetDcB = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_DC_B,
                                   "DC Correct", x + 150, y0, 145);
    g_hSetIqB = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_IQ_B,
                                   "IQ Correct", x + 300, y0, 145);
    y0 += row;
    g_hSetNotchRfB = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_NOTCH_RF_B,
                                        "RF Notch", x, y0, 145);
    g_hSetNotchDabB = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_NOTCH_DAB_B,
                                         "DAB Notch", x + 150, y0, 145);
    y0 += row + 6;

    y_r = y0;

    /* ---------------------------------------------------------------- */
    /* Tab 1: Device Settings - split out from Receiver into its own tab */
    /* so every tab ends up a similar height instead of all of them      */
    /* sharing one height sized for Receiver (which used to carry both   */
    /* the Tuners section and this).                                     */
    /* ---------------------------------------------------------------- */
    g_settings_build_tab = 1;
    y0 = y_content0;

    settings_mk_label(g_hSettingsWnd, hInst, "DEVICE SETTINGS", x, y0, win_w - x - 16);
    y0 += row - 10;

    settings_mk_label(g_hSettingsWnd, hInst, "Sample Rate / IF / BW", x, y0, label_w);
    g_hSetRateCombo = CreateWindowExA(0, "COMBOBOX", "",
                WS_CHILD | WS_VISIBLE | WS_BORDER | CBS_DROPDOWNLIST | WS_VSCROLL,
                ctl_x, y0 - 2, win_w - ctl_x - 16, 22 + 200,
                g_hSettingsWnd, (HMENU)(INT_PTR)IDC_SET_RATECOMBO, hInst, NULL);
    if (g_hFontUI) SendMessageA(g_hSetRateCombo, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
    /* The dropped-down list doesn't auto-widen beyond the closed control's
     * own width, unlike the closed box it's a popup and can safely extend
     * past the dialog's right edge without disturbing anything else -
     * give it extra room so the longest Low-IF entries never truncate. */
    SendMessageA(g_hSetRateCombo, CB_SETDROPPEDWIDTH, (WPARAM)(win_w - ctl_x - 16 + 80), 0);
    settings_apply_dark_theme(g_hSetRateCombo);
    settings_tag_tab(g_hSetRateCombo);
    y0 += row;

    /* Frequency-range helper: given a start/end range in MHz (matching the
     * Frequency field's own units above), computes and applies the
     * smallest sufficient bandwidth, the lowest sample rate that offers
     * it, and the resulting centre frequency - the same
     * smallest-sufficient-bandwidth method the User Guide's Appendix A
     * walks through by hand. Saves working that out manually for a
     * one-off capture of a specific range. Zero-IF only (if_khz=0); for
     * MW/LW work where Low-IF's DC-spike avoidance matters, the Sample
     * Rate / IF / BW dropdown above already has the recommended combo. */
    settings_mk_label(g_hSettingsWnd, hInst, "Frequency Range (MHz)", x, y0, label_w);
    g_hSetRangeStart = settings_mk_edit(g_hSettingsWnd, hInst, IDC_SET_RANGE_START,
                                          ctl_x, y0 - 2, 80);
    settings_mk_label(g_hSettingsWnd, hInst, "to", ctl_x + 86, y0 + 2, 20);
    g_hSetRangeEnd = settings_mk_edit(g_hSettingsWnd, hInst, IDC_SET_RANGE_END,
                                        ctl_x + 110, y0 - 2, 80);
    g_hBtnRangeCalc = CreateWindowExA(0, "BUTTON", "Calculate",
                WS_CHILD | WS_VISIBLE | BS_OWNERDRAW,
                ctl_x + 202, y0 - 2, 90, 24,
                g_hSettingsWnd, (HMENU)(INT_PTR)IDC_BTN_RANGE_CALC, hInst, NULL);
    if (g_hFontUI) SendMessageA(g_hBtnRangeCalc, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
    settings_tag_tab(g_hBtnRangeCalc);
    y0 += row - 4;

    /* Three lines tall (not settings_mk_label's fixed single-line height) -
     * plain SS_LEFT static text already word-wraps within its width, but
     * a shorter-than-needed control just clips anything past its own
     * height rather than showing it, so the control itself needs the
     * room for the longest message (the dual-tuner-mode one, which adds
     * a reminder about Tuner B) ever to be fully visible. */
    g_hSetRangeHint = CreateWindowExA(0, "STATIC", "", WS_CHILD | WS_VISIBLE | SS_LEFT,
                                        x, y0, win_w - x - 16, 54, g_hSettingsWnd, NULL, hInst, NULL);
    if (g_hFontUI) SendMessageA(g_hSetRangeHint, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
    settings_tag_tab(g_hSetRangeHint);
    y0 += row + 24;

    settings_mk_label(g_hSettingsWnd, hInst, "Freq. Correction (ppm)", x, y0, label_w);
    g_hSetPpm = settings_mk_edit(g_hSettingsWnd, hInst, IDC_SET_PPM, ctl_x, y0 - 2, ctl_w);
    y0 += row;

    g_hSetHdr = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_HDR,
                                   "HDR Enable (RSPdx)", x, y0, 220);
    y0 += row;

    settings_mk_label(g_hSettingsWnd, hInst, "HDR Bandwidth (kHz)", x, y0, label_w);
    g_hSetHdrBw = CreateWindowExA(0, "COMBOBOX", "",
                WS_CHILD | WS_VISIBLE | WS_BORDER | CBS_DROPDOWNLIST | WS_VSCROLL,
                ctl_x, y0 - 2, ctl_w, 22 + 100,
                g_hSettingsWnd, (HMENU)(INT_PTR)IDC_SET_HDR_BW, hInst, NULL);
    SendMessageA(g_hSetHdrBw, CB_ADDSTRING, 0, (LPARAM)"200");
    SendMessageA(g_hSetHdrBw, CB_ADDSTRING, 0, (LPARAM)"500");
    SendMessageA(g_hSetHdrBw, CB_ADDSTRING, 0, (LPARAM)"1200");
    SendMessageA(g_hSetHdrBw, CB_ADDSTRING, 0, (LPARAM)"1700");
    if (g_hFontUI) SendMessageA(g_hSetHdrBw, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
    settings_apply_dark_theme(g_hSetHdrBw);
    settings_tag_tab(g_hSetHdrBw);
    y0 += row;

    g_hSetHdrHint = settings_mk_label(g_hSettingsWnd, hInst, "", x, y0, win_w - x - 16);
    y0 += row - 12 + row;

    y_dev = y0;

    /* ---------------------------------------------------------------- */
    /* Tab 2: Recording                                                   */
    /* ---------------------------------------------------------------- */
    g_settings_build_tab = 2;
    y0 = y_content0;

    settings_mk_label(g_hSettingsWnd, hInst, "Duration (HH:MM:SS)", x, y0, label_w);
    g_hSetDuration = settings_mk_edit(g_hSettingsWnd, hInst, IDC_SET_DURATION, ctl_x, y0 - 2, 90);
    /* Re-subclass every time - this dialog's controls are destroyed and
     * recreated fresh each time it's opened (see g_hSettingsWnd = NULL
     * on close), so g_hSetDuration is a new HWND each time too.        */
    apply_time_edit_subclass(g_hSetDuration, 8, 1, 0);  /* duration - hours unrestricted */
    settings_mk_label(g_hSettingsWnd, hInst, "(00:00:00 = unlimited)",
                       ctl_x + 98, y0 + 1, 150);
    y0 += row;

    settings_mk_label(g_hSettingsWnd, hInst, "Decimation", x, y0, label_w);
    g_hSetDecim = CreateWindowExA(0, "COMBOBOX", "",
                WS_CHILD | WS_VISIBLE | WS_BORDER | CBS_DROPDOWNLIST | WS_VSCROLL,
                ctl_x, y0 - 2, ctl_w, 22 + 120,
                g_hSettingsWnd, (HMENU)(INT_PTR)IDC_SET_DECIM, hInst, NULL);
    SendMessageA(g_hSetDecim, CB_ADDSTRING, 0, (LPARAM)"1 (off)");
    SendMessageA(g_hSetDecim, CB_ADDSTRING, 0, (LPARAM)"2");
    SendMessageA(g_hSetDecim, CB_ADDSTRING, 0, (LPARAM)"4");
    SendMessageA(g_hSetDecim, CB_ADDSTRING, 0, (LPARAM)"8");
    SendMessageA(g_hSetDecim, CB_ADDSTRING, 0, (LPARAM)"16");
    SendMessageA(g_hSetDecim, CB_ADDSTRING, 0, (LPARAM)"32");
    if (g_hFontUI) SendMessageA(g_hSetDecim, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
    settings_apply_dark_theme(g_hSetDecim);
    settings_tag_tab(g_hSetDecim);
    y0 += row;

    settings_mk_label(g_hSettingsWnd, hInst, "Output Format", x, y0, label_w);
    g_hSetFormat = CreateWindowExA(0, "COMBOBOX", "",
                WS_CHILD | WS_VISIBLE | WS_BORDER | CBS_DROPDOWNLIST | WS_VSCROLL,
                ctl_x, y0 - 2, ctl_w, 22 + 100,
                g_hSettingsWnd, (HMENU)(INT_PTR)IDC_SET_FORMAT, hInst, NULL);
    SendMessageA(g_hSetFormat, CB_ADDSTRING, 0, (LPARAM)"Linrad");
    SendMessageA(g_hSetFormat, CB_ADDSTRING, 0, (LPARAM)"WavViewDX");
    SendMessageA(g_hSetFormat, CB_ADDSTRING, 0, (LPARAM)"SDRuno");
    SendMessageA(g_hSetFormat, CB_ADDSTRING, 0, (LPARAM)"SDRconnect");
    SendMessageA(g_hSetFormat, CB_ADDSTRING, 0, (LPARAM)"Winrad");
    if (g_hFontUI) SendMessageA(g_hSetFormat, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
    settings_apply_dark_theme(g_hSetFormat);
    settings_tag_tab(g_hSetFormat);
    y0 += row;

    /* SDRuno and SDR Connect only: how to handle a recording that would
     * exceed the 4 GiB WAV data-size limit. Hidden for every other output
     * format - shown/hidden live by settings_update_format_dependent_state(). */
    g_hSetLargeModeLbl = settings_mk_label(g_hSettingsWnd, hInst,
                                    ">4GB File Handling", x, y0, label_w);
    g_hSetLargeMode = CreateWindowExA(0, "COMBOBOX", "",
                WS_CHILD | WS_VISIBLE | WS_BORDER | CBS_DROPDOWNLIST | WS_VSCROLL,
                ctl_x, y0 - 2, win_w - ctl_x - 16, 22 + 60,
                g_hSettingsWnd, (HMENU)(INT_PTR)IDC_SET_LARGEMODE, hInst, NULL);
    SendMessageA(g_hSetLargeMode, CB_ADDSTRING, 0, (LPARAM)"Split at 4GB (recommended)");
    SendMessageA(g_hSetLargeMode, CB_ADDSTRING, 0, (LPARAM)"RF64 (single file)");
    if (g_hFontUI) SendMessageA(g_hSetLargeMode, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
    settings_apply_dark_theme(g_hSetLargeMode);
    settings_tag_tab(g_hSetLargeMode);
    y0 += row;

    settings_mk_label(g_hSettingsWnd, hInst, "Recording Path", x, y0, label_w);
    g_hSetPath = settings_mk_edit(g_hSettingsWnd, hInst, IDC_SET_PATH, ctl_x,
                                   y0 - 2, win_w - ctl_x - 16);
    y0 += row;

    settings_mk_label(g_hSettingsWnd, hInst, "Ring Buffer (sec)", x, y0, label_w);
    g_hSetRingSec = settings_mk_edit(g_hSettingsWnd, hInst, IDC_SET_RING_SEC, ctl_x, y0 - 2, ctl_w);
    y0 += row;

    g_hSetSpinupEn = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_SPINUP_EN,
                                        "Disk Spin-Up Before Recording", x, y0, 260);
    g_hSetSpinupBytes = CreateWindowExA(0, "COMBOBOX", "",
                WS_CHILD | WS_VISIBLE | WS_BORDER | CBS_DROPDOWNLIST | WS_VSCROLL,
                x + 270, y0 - 2, 100, 22 + 80,
                g_hSettingsWnd, (HMENU)(INT_PTR)IDC_SET_SPINUP_BYTES, hInst, NULL);
    SendMessageA(g_hSetSpinupBytes, CB_ADDSTRING, 0, (LPARAM)"1 MB");
    SendMessageA(g_hSetSpinupBytes, CB_ADDSTRING, 0, (LPARAM)"4 MB");
    SendMessageA(g_hSetSpinupBytes, CB_ADDSTRING, 0, (LPARAM)"8 MB");
    if (g_hFontUI) SendMessageA(g_hSetSpinupBytes, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
    settings_apply_dark_theme(g_hSetSpinupBytes);
    settings_tag_tab(g_hSetSpinupBytes);
    y0 += row;

    y_rec = y0;

    /* ---------------------------------------------------------------- */
    /* Tab 3: Monitor / Display                                           */
    /* ---------------------------------------------------------------- */
    g_settings_build_tab = 3;
    y0 = y_content0;

    /* Low-Cut Filter moved to the main window (next to Vol) - it's a
     * monitor-audio-only control, but this whole dialog is disabled while
     * actively recording, which meant there was no way to reach it then. */
    g_hSetMonVisible = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_MON_VISIBLE,
                                          "Show Monitor Controls", x, y0, 260);
    y0 += row;

    settings_mk_label(g_hSettingsWnd, hInst, "S-Meter Mode", x, y0, label_w);
    g_hSetSMeterMode = CreateWindowExA(0, "COMBOBOX", "",
                WS_CHILD | WS_VISIBLE | WS_BORDER | CBS_DROPDOWNLIST | WS_VSCROLL,
                ctl_x, y0 - 2, 220, 22 + 60,
                g_hSettingsWnd, (HMENU)(INT_PTR)IDC_SET_SMETER_MODE, hInst, NULL);
    SendMessageA(g_hSetSMeterMode, CB_ADDSTRING, 0, (LPARAM)"Peak (more responsive)");
    SendMessageA(g_hSetSMeterMode, CB_ADDSTRING, 0, (LPARAM)"Averaged (steadier)");
    if (g_hFontUI) SendMessageA(g_hSetSMeterMode, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
    settings_apply_dark_theme(g_hSetSMeterMode);
    settings_tag_tab(g_hSetSMeterMode);
    y0 += row;

    settings_mk_label(g_hSettingsWnd, hInst, "S-Meter Cal. Offset (dB)", x, y0, label_w);
    g_hSetSMeterCal = settings_mk_edit(g_hSettingsWnd, hInst, IDC_SET_SMETER_CAL,
                                        ctl_x, y0 - 2, 70);
    y0 += row;
    settings_mk_label(g_hSettingsWnd, hInst,
                       "Approximate dBm only - set against a known signal source to calibrate",
                       x, y0, 480);
    y0 += row;

    settings_mk_label(g_hSettingsWnd, hInst, "Status Update Interval", x, y0, label_w);
    g_hSetMonInterval = CreateWindowExA(0, "COMBOBOX", "",
                WS_CHILD | WS_VISIBLE | WS_BORDER | CBS_DROPDOWNLIST | WS_VSCROLL,
                ctl_x, y0 - 2, ctl_w, 22 + 80,
                g_hSettingsWnd, (HMENU)(INT_PTR)IDC_SET_MON_INTERVAL, hInst, NULL);
    SendMessageA(g_hSetMonInterval, CB_ADDSTRING, 0, (LPARAM)"200 ms");
    SendMessageA(g_hSetMonInterval, CB_ADDSTRING, 0, (LPARAM)"500 ms");
    SendMessageA(g_hSetMonInterval, CB_ADDSTRING, 0, (LPARAM)"1000 ms");
    if (g_hFontUI) SendMessageA(g_hSetMonInterval, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
    settings_apply_dark_theme(g_hSetMonInterval);
    settings_tag_tab(g_hSetMonInterval);
    y0 += row;

    y_mon = y0;

    /* ---------------------------------------------------------------- */
    /* Tab 4: Network                                                     */
    /* ---------------------------------------------------------------- */
    g_settings_build_tab = 4;
    y0 = y_content0;

    settings_mk_label(g_hSettingsWnd, hInst, "HTTP Port (0=off)", x, y0, label_w);
    g_hSetHttpPort = settings_mk_edit(g_hSettingsWnd, hInst, IDC_SET_HTTP_PORT, ctl_x, y0 - 2, ctl_w);
    y0 += row;

    settings_mk_label(g_hSettingsWnd, hInst, "HTTP Refresh (ms)", x, y0, label_w);
    g_hSetHttpInterval = settings_mk_edit(g_hSettingsWnd, hInst, IDC_SET_HTTP_INTERVAL, ctl_x, y0 - 2, ctl_w);
    y0 += row;

    g_hSetPipeEn = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_PIPE_EN,
                                      "Named Pipe (real-time IQ stream)", x, y0, 300);
    y0 += row;

    settings_mk_label(g_hSettingsWnd, hInst, "Pipe Name", x, y0, label_w);
    g_hSetPipeName = settings_mk_edit(g_hSettingsWnd, hInst, IDC_SET_PIPE_NAME, ctl_x,
                                       y0 - 2, win_w - ctl_x - 16);
    y0 += row;

    y_net = y0;

    /* ---------------------------------------------------------------- */
    /* Tab 5: Schedule                                                    */
    /* ---------------------------------------------------------------- */
    g_settings_build_tab = 5;
    y0 = y_content0;

    g_hSetSchedOnly = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_SCHED_ONLY,
                                         "Schedule Recording", x, y0, 200);
    y0 += row;

    g_hSetSchedIdxLbl = settings_mk_label(g_hSettingsWnd, hInst, "Entry 1 of 1",
                                           x, y0 + 3, 110);
    g_hSetSchedPrev = CreateWindowExA(0, "BUTTON", "<",
                WS_CHILD | WS_VISIBLE | BS_OWNERDRAW,
                x + 120, y0, 30, 26,
                g_hSettingsWnd, (HMENU)(INT_PTR)IDC_SET_SCHED_PREV, hInst, NULL);
    g_hSetSchedNext = CreateWindowExA(0, "BUTTON", ">",
                WS_CHILD | WS_VISIBLE | BS_OWNERDRAW,
                x + 155, y0, 30, 26,
                g_hSettingsWnd, (HMENU)(INT_PTR)IDC_SET_SCHED_NEXT, hInst, NULL);
    g_hSetSchedAdd = CreateWindowExA(0, "BUTTON", "+ Add",
                WS_CHILD | WS_VISIBLE | BS_OWNERDRAW,
                x + 200, y0, 70, 26,
                g_hSettingsWnd, (HMENU)(INT_PTR)IDC_SET_SCHED_ADD, hInst, NULL);
    g_hSetSchedDel = CreateWindowExA(0, "BUTTON", "- Delete",
                WS_CHILD | WS_VISIBLE | BS_OWNERDRAW,
                x + 280, y0, 80, 26,
                g_hSettingsWnd, (HMENU)(INT_PTR)IDC_SET_SCHED_DEL, hInst, NULL);
    if (g_hFontUI) {
        SendMessageA(g_hSetSchedPrev, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
        SendMessageA(g_hSetSchedNext, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
        SendMessageA(g_hSetSchedAdd,  WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
        SendMessageA(g_hSetSchedDel,  WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
    }
    settings_tag_tab(g_hSetSchedPrev);
    settings_tag_tab(g_hSetSchedNext);
    settings_tag_tab(g_hSetSchedAdd);
    settings_tag_tab(g_hSetSchedDel);
    y0 += row;

    settings_mk_label(g_hSettingsWnd, hInst, "Start Time (HH:MM:SS)", x, y0, label_w);
    g_hSetSchedStart = settings_mk_edit(g_hSettingsWnd, hInst, IDC_SET_SCHED_START, ctl_x, y0 - 2, 90);
    apply_time_edit_subclass(g_hSetSchedStart, 8, 0, 0);  /* time-of-day - hours capped at 23 */
    settings_mk_label(g_hSettingsWnd, hInst, "Duration (HH:MM:SS)", ctl_x + 100, y0, 140);
    g_hSetSchedDuration = settings_mk_edit(g_hSettingsWnd, hInst, IDC_SET_SCHED_DURATION, ctl_x + 250, y0 - 2, 90);
    apply_time_edit_subclass(g_hSetSchedDuration, 8, 1, 0);  /* duration - hours unrestricted */
    y0 += row;

    settings_mk_label(g_hSettingsWnd, hInst, "Frequency A (MHz)", x, y0, label_w);
    g_hSetSchedFreq = settings_mk_edit(g_hSettingsWnd, hInst, IDC_SET_SCHED_FREQ, ctl_x, y0 - 2, 90);
    settings_mk_label(g_hSettingsWnd, hInst, "Frequency B (MHz)", ctl_x + 100, y0, 140);
    g_hSetSchedFreqB = settings_mk_edit(g_hSettingsWnd, hInst, IDC_SET_SCHED_FREQ_B, ctl_x + 250, y0 - 2, 90);
    y0 += row;

    settings_mk_label(g_hSettingsWnd, hInst, "Antenna", x, y0, label_w);
    g_hSetSchedAntenna = CreateWindowExA(0, "COMBOBOX", "",
                WS_CHILD | WS_VISIBLE | WS_BORDER | CBS_DROPDOWN | WS_VSCROLL,
                ctl_x, y0 - 2, 90, 22 + 120,
                g_hSettingsWnd, (HMENU)(INT_PTR)IDC_SET_SCHED_ANTENNA, hInst, NULL);
    SendMessageA(g_hSetSchedAntenna, CB_ADDSTRING, 0, (LPARAM)"");
    SendMessageA(g_hSetSchedAntenna, CB_ADDSTRING, 0, (LPARAM)"A");
    SendMessageA(g_hSetSchedAntenna, CB_ADDSTRING, 0, (LPARAM)"B");
    SendMessageA(g_hSetSchedAntenna, CB_ADDSTRING, 0, (LPARAM)"C");
    SendMessageA(g_hSetSchedAntenna, CB_ADDSTRING, 0, (LPARAM)"50ohm");
    SendMessageA(g_hSetSchedAntenna, CB_ADDSTRING, 0, (LPARAM)"Hi-Z");
    if (g_hFontUI) SendMessageA(g_hSetSchedAntenna, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
    settings_apply_dark_theme(g_hSetSchedAntenna);
    settings_tag_tab(g_hSetSchedAntenna);
    settings_mk_label(g_hSettingsWnd, hInst, "Output File", ctl_x + 100, y0, 140);
    g_hSetSchedOutfile = settings_mk_edit(g_hSettingsWnd, hInst, IDC_SET_SCHED_OUTFILE,
                                           ctl_x + 250, y0 - 2, win_w - (ctl_x + 250) - 16);
    y0 += row;

    settings_mk_label(g_hSettingsWnd, hInst,
        "Blank = keep current/auto. Antenna/frequency blank = keep whatever was previously set.",
        x, y0, win_w - x - 16);
    y0 += row + 14;

    g_hSetHourlyEn = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_HOURLY_EN,
                                        "Hourly Recording", x, y0, 220);
    y0 += row;

    settings_mk_label(g_hSettingsWnd, hInst, "Window (min)", x, y0, label_w);
    g_hSetHourlyWin = settings_mk_edit(g_hSettingsWnd, hInst, IDC_SET_HOURLY_WIN, ctl_x, y0 - 2, 70);
    y0 += row;

    settings_mk_label(g_hSettingsWnd, hInst, "Session Start (HH:MM)", x, y0, label_w);
    g_hSetHourlyStart = settings_mk_edit(g_hSettingsWnd, hInst, IDC_SET_HOURLY_START, ctl_x, y0 - 2, 70);
    apply_time_edit_subclass(g_hSetHourlyStart, 5, 0, 1);  /* time-of-day, minutes locked at 00 - Hourly always fires on the hour */
    settings_mk_label(g_hSettingsWnd, hInst, "Session Stop (HH:MM)", ctl_x + 110, y0, 160);
    g_hSetHourlyStop = settings_mk_edit(g_hSettingsWnd, hInst, IDC_SET_HOURLY_STOP, ctl_x + 280, y0 - 2, 70);
    apply_time_edit_subclass(g_hSetHourlyStop, 5, 0, 1);  /* time-of-day, minutes locked at 00 - Hourly always fires on the hour */
    y0 += row;

    g_hSetHourlyHint = settings_mk_label(g_hSettingsWnd, hInst, "", x, y0, win_w - x - 16);
    y0 += row;

    y_sched = y0;

    g_settings_build_tab = 0; /* restore harmless default; no more controls created below */

    /* ---------------------------------------------------------------- */
    /* Tab 6: Miscellaneous                                               */
    /* ---------------------------------------------------------------- */
    g_settings_build_tab = 6;
    y0 = y_content0;

    settings_mk_label(g_hSettingsWnd, hInst, "Color Scheme", x, y0, label_w);
    g_hSetColorScheme = CreateWindowExA(0, "COMBOBOX", "",
                WS_CHILD | WS_VISIBLE | WS_BORDER | CBS_DROPDOWNLIST | WS_VSCROLL,
                ctl_x, y0 - 2, 220, 22 + 60,
                g_hSettingsWnd, (HMENU)(INT_PTR)IDC_SET_COLOR_SCHEME, hInst, NULL);
    SendMessageA(g_hSetColorScheme, CB_ADDSTRING, 0, (LPARAM)"Navy Blue");
    SendMessageA(g_hSetColorScheme, CB_ADDSTRING, 0, (LPARAM)"Dark Grey");
    if (g_hFontUI) SendMessageA(g_hSetColorScheme, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
    settings_apply_dark_theme(g_hSetColorScheme);
    settings_tag_tab(g_hSetColorScheme);
    y0 += row;

    settings_mk_label(g_hSettingsWnd, hInst,
        "Takes effect the next time DuoDX starts - the same as sample rate, "
        "dual channel, and HDR (Section 5, Section 7, Section 12.3).",
        x, y0, win_w - x - 16);
    y0 += row;

    settings_mk_label(g_hSettingsWnd, hInst, "VU Meter Style", x, y0, label_w);
    g_hSetMeterStyle = CreateWindowExA(0, "COMBOBOX", "",
                WS_CHILD | WS_VISIBLE | WS_BORDER | CBS_DROPDOWNLIST | WS_VSCROLL,
                ctl_x, y0 - 2, 220, 22 + 80,
                g_hSettingsWnd, (HMENU)(INT_PTR)IDC_SET_METER_STYLE, hInst, NULL);
    SendMessageA(g_hSetMeterStyle, CB_ADDSTRING, 0, (LPARAM)"Zone Colours");
    SendMessageA(g_hSetMeterStyle, CB_ADDSTRING, 0, (LPARAM)"Graduated Blend");
    SendMessageA(g_hSetMeterStyle, CB_ADDSTRING, 0, (LPARAM)"Greyscale");
    if (g_hFontUI) SendMessageA(g_hSetMeterStyle, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
    settings_apply_dark_theme(g_hSetMeterStyle);
    settings_tag_tab(g_hSetMeterStyle);
    y0 += row;

    g_hSetUseUtc = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_USE_UTC,
                                      "Use UTC Time", x, y0, 180);
    g_hSetShowClock = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_SHOW_CLOCK,
                                         "Show Clock", x + 190, y0, 180);
    y0 += row;

    g_hSetVerbose = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_VERBOSE,
                                       "Verbose Logging", x, y0, 220);
    y0 += row;

    g_hSetLogAutoSave = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_LOG_AUTOSAVE,
                                       "Auto-save session log to a file", x, y0, 260);
    y0 += row;

    settings_mk_label(g_hSettingsWnd, hInst, "Location (for sunrise/sunset)", x, y0, win_w - x - 16);
    y0 += row;

    settings_mk_label(g_hSettingsWnd, hInst, "Latitude", x, y0, label_w);
    g_hSetLatitude = settings_mk_edit(g_hSettingsWnd, hInst, IDC_SET_LATITUDE, ctl_x, y0 - 2, 120);
    settings_mk_label(g_hSettingsWnd, hInst, "Longitude", ctl_x + 130, y0, 80);
    g_hSetLongitude = settings_mk_edit(g_hSettingsWnd, hInst, IDC_SET_LONGITUDE,
                                        ctl_x + 210, y0 - 2, 120);
    y0 += row;

    settings_mk_label(g_hSettingsWnd, hInst,
        "Decimal degrees - positive latitude is north, positive longitude is east "
        "(e.g. -45.098, 170.966).", x, y0, win_w - x - 16);
    y0 += row;

    g_hSetShowSun = settings_mk_check(g_hSettingsWnd, hInst, IDC_SET_SHOW_SUN,
                                       "Show Sunrise/Sunset on main window", x, y0, 300);
    y0 += row;

    y_misc = y0;

    g_settings_build_tab = 0; /* restore harmless default; no more controls created below */

    /* ---------------------------------------------------------------- */
    /* Save / Cancel - untagged, so always visible regardless of tab.    */
    /* Positioned below whichever tab needs the most vertical space.     */
    /* ---------------------------------------------------------------- */
    y_max = y_r;
    if (y_dev   > y_max) y_max = y_dev;
    if (y_rec   > y_max) y_max = y_rec;
    if (y_mon   > y_max) y_max = y_mon;
    if (y_net   > y_max) y_max = y_net;
    if (y_sched > y_max) y_max = y_sched;
    if (y_misc  > y_max) y_max = y_misc;
    y0 = y_max + 16;

    hSave = CreateWindowExA(0, "BUTTON", "Save",
                WS_CHILD | WS_VISIBLE | BS_OWNERDRAW,
                win_w - 16 - 90 - 8 - 90, y0, 90, 28,
                g_hSettingsWnd, (HMENU)(INT_PTR)IDC_SET_SAVE, hInst, NULL);
    hCancel = CreateWindowExA(0, "BUTTON", "Cancel",
                WS_CHILD | WS_VISIBLE | BS_OWNERDRAW,
                win_w - 16 - 90, y0, 90, 28,
                g_hSettingsWnd, (HMENU)(INT_PTR)IDC_SET_CANCEL, hInst, NULL);
    if (g_hFontUI) {
        SendMessageA(hSave, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
        SendMessageA(hCancel, WM_SETFONT, (WPARAM)g_hFontUI, TRUE);
    }

    /* Auto-size the window to fit whichever tab needs the most vertical
     * space, rather than relying on a hand-maintained win_h constant
     * that silently goes stale (and cuts off Save/Cancel) every time a
     * field is added or removed. Happens before ShowWindow below, so
     * there's no visible resize flash. Now that Device Settings has its
     * own tab, every tab's content height is fairly close, so this ends
     * up sizing the window sensibly rather than to one outlier tab.     */
    {
        RECT rc;
        int needed_client_h = y0 + 28 + 16;   /* Save/Cancel height + bottom margin */
        rc.left = 0; rc.top = 0; rc.right = win_w; rc.bottom = needed_client_h;
        AdjustWindowRectEx(&rc, WS_POPUP | WS_CAPTION | WS_SYSMENU, FALSE,
                            WS_EX_DLGMODALFRAME);
        SetWindowPos(g_hSettingsWnd, NULL, 0, 0,
                     rc.right - rc.left, rc.bottom - rc.top,
                     SWP_NOMOVE | SWP_NOZORDER);
    }

    settings_select_tab(0);
    settings_load_controls();

    ShowWindow(g_hSettingsWnd, SW_SHOW);
    EnableWindow(parent, FALSE);

    /* Simple modal loop - runs until the settings window is destroyed. */
    {
        MSG msg;
        while (IsWindow(g_hSettingsWnd)) {
            if (!GetMessageA(&msg, NULL, 0, 0)) break;
            if (!IsDialogMessageA(g_hSettingsWnd, &msg)) {
                TranslateMessage(&msg);
                DispatchMessageA(&msg);
            }
        }
    }

    EnableWindow(parent, TRUE);
    SetForegroundWindow(parent);
}

/* Lightweight device detection - opens the SDRplay API just long enough
 * to enumerate connected devices and read the hwVer of whichever one
 * would actually be selected (by device_serial if set, otherwise the
 * first one found), then closes the API again without ever calling
 * SelectDevice/Init - no stream is started, nothing is locked in for the
 * session proper. Called once at startup so Settings can correctly grey
 * out Tuner 2 for a non-RSPduo device (Section 14.1) even before the
 * first session ever runs, and again every time Settings is opened, so
 * a device swapped for a different one after launch (without restarting
 * the app, and without a session having been started with it yet) is
 * also picked up correctly rather than leaving g_last_known_hwVer stuck
 * on whatever was connected at launch. Any failure here (API service not
 * running, no device connected) is silently ignored - g_last_known_hwVer
 * just keeps its previous value, and the real, fully-logged enumeration
 * still happens normally when a session actually starts.                */
static void refresh_known_device_type(void)
{
    sdrplay_api_ErrT err;
    sdrplay_api_DeviceT devices[6];
    unsigned int num_devices = 0, i;
    unsigned int selected = 0;

    err = sdrplay_api_Open();
    if (err != sdrplay_api_Success) return;

    err = sdrplay_api_LockDeviceApi();
    if (err != sdrplay_api_Success) {
        sdrplay_api_Close();
        return;
    }

    err = sdrplay_api_GetDevices(devices, &num_devices, 6);
    if (err == sdrplay_api_Success && num_devices > 0) {
        if (g_state.cfg.device_serial[0]) {
            for (i = 0; i < num_devices; i++) {
                if (strncmp(devices[i].SerNo, g_state.cfg.device_serial, 63) == 0) {
                    selected = i;
                    break;
                }
            }
        }
        g_last_known_hwVer = devices[selected].hwVer;
    }

    sdrplay_api_UnlockDeviceApi();
    sdrplay_api_Close();
}

int WINAPI WinMain(HINSTANCE hInst, HINSTANCE hPrev, LPSTR lpCmdLine, int nShow)
{
    (void)hPrev;

    /* Installed first, before anything else - covers both this process
     * and the RSPduo Master/Slave slave process below, since that's a
     * second invocation of this same WinMain (see cmdline_is_slave_b). */
    SetUnhandledExceptionFilter(crash_exception_filter);

    if (cmdline_is_slave_b(lpCmdLine)) {
        char outfile[MAX_PATH_LEN];
        long duration = cmdline_get_int_value(lpCmdLine, "--duration=");
        long master_pid = cmdline_get_int_value(lpCmdLine, "--masterpid=");
        cmdline_get_quoted_value(lpCmdLine, "--outfile=", outfile, sizeof(outfile));
        return run_slave_b_session(outfile, (int)duration, (DWORD)master_pid,
                                    cmdline_is_listen_only(lpCmdLine));
    }

    InitCommonControls();
    LoadLibraryA("Msftedit.dll");   /* registers RICHEDIT50W window class */

    /* Load duodx.ini now, before window creation below needs the saved
     * window geometry, and before any control further down reads
     * g_state.cfg for its initial state (the Schedule button's label,
     * the monitor bar's visibility, ...). Previously nothing populated
     * g_state.cfg until the first Record/Monitor press or Settings save,
     * so those initial reads were silently working off zero-initialised
     * memory instead of the actual configured values - happened to look
     * right for fields whose default is 0 (schedule_only), but not for
     * anything that defaults to on, like monitor_bar_visible.            */
    config_set_defaults(&g_state.cfg);
    config_load_ini(&g_state.cfg, "duodx.ini");

    /* Color scheme must be resolved before any brush is created below. */
    apply_color_scheme(g_state.cfg.color_scheme);

    g_hbrBg    = CreateSolidBrush(COL_BG);
    g_hbrPanel = CreateSolidBrush(COL_BAR_BG);

    /* Timer always starts OFF, regardless of what was saved last session -
     * a predictable "always idle at launch" experience rather than
     * silently picking up an armed schedule/hourly plan from before.
     * timer_last_mode (which one Timer will arm next time it's turned on)
     * is deliberately left alone; only whether it's currently armed
     * resets.                                                             */
    if (g_state.cfg.schedule_only || g_state.cfg.hourly_enable) {
        IniPatchEntry timer_off[2];
        int timer_off_n = 0;
        g_state.cfg.schedule_only = 0;
        g_state.cfg.hourly_enable = 0;
        timer_off[timer_off_n].key = "schedule_only";
        snprintf(timer_off[timer_off_n].value, sizeof(timer_off[timer_off_n].value), "0");
        timer_off_n++;
        timer_off[timer_off_n].key = "hourly_enable";
        snprintf(timer_off[timer_off_n].value, sizeof(timer_off[timer_off_n].value), "0");
        timer_off_n++;
        ini_patch_values("duodx.ini", timer_off, timer_off_n);
    }

    validate_config(&g_state.cfg);

    /* Best-effort device detection so Settings can correctly grey out
     * Tuner 2 for a non-RSPduo device from the very first time it's
     * opened, rather than only after a session has actually run once.  */
    refresh_known_device_type();

    /* Correct a stale dual_channel or rspduo_single_tuner=B setting left
     * over from a previous session on a different (RSPduo) device - both
     * only make sense there. Settings already prevents these from being
     * set going forward once the device is known, but a value saved
     * before that protection existed (or before switching to a
     * different device) would otherwise sit in duodx.ini unnoticed until
     * Monitor or Record was pressed and produced a confusing runtime
     * warning about a mode this hardware was never going to support.     */
    int startup_duo_fix_applied = 0;
    if (g_last_known_hwVer != 0 && g_last_known_hwVer != SDRPLAY_RSPduo_ID &&
            (g_state.cfg.dual_channel || !strcmp(g_state.cfg.rspduo_single_tuner, "B"))) {
        IniPatchEntry duo_fix[2];
        int duo_fix_n = 0;
        g_state.cfg.dual_channel = 0;
        strncpy(g_state.cfg.rspduo_single_tuner, "A",
                sizeof(g_state.cfg.rspduo_single_tuner) - 1);
        duo_fix[duo_fix_n].key = "dual_channel";
        snprintf(duo_fix[duo_fix_n].value, sizeof(duo_fix[duo_fix_n].value), "0");
        duo_fix_n++;
        duo_fix[duo_fix_n].key = "rspduo_single_tuner";
        snprintf(duo_fix[duo_fix_n].value, sizeof(duo_fix[duo_fix_n].value), "A");
        duo_fix_n++;
        ini_patch_values("duodx.ini", duo_fix, duo_fix_n);
        startup_duo_fix_applied = 1;
    }

    WNDCLASSA wc;
    memset(&wc, 0, sizeof(wc));
    wc.lpfnWndProc   = WndProc;
    wc.hInstance     = hInst;
    wc.hCursor       = LoadCursor(NULL, IDC_ARROW);
    wc.hbrBackground = g_hbrBg;
    wc.lpszClassName = "DuoDXWindow";
    wc.hIcon         = LoadIcon(hInst, MAKEINTRESOURCE(1));
    if (!RegisterClassA(&wc)) {
        MessageBoxA(NULL, "RegisterClass failed", "DuoDX", MB_ICONERROR);
        return 1;
    }

    /* Saved window size gets a sane floor - guards against a corrupted or
     * hand-edited ini leaving the window too small to use (or invisible,
     * at 0x0). Position is not clamped the same way: an off-screen saved
     * position is Windows' own problem to fix (it snaps back on-screen
     * automatically), and clamping it here could fight a legitimate
     * multi-monitor layout instead.                                     */
    {
        int ww = g_state.cfg.window_w, wh = g_state.cfg.window_h;
        if (ww < 500) ww = 930;
        if (wh < 400) wh = 660;
        g_hwnd = CreateWindowExA(0, "DuoDXWindow",
                    "DuoDX  v" VERSION "  -  RSP Dual Channel IQ Recorder",
                    WS_OVERLAPPEDWINDOW,
                    g_state.cfg.window_x, g_state.cfg.window_y, ww, wh,
                    NULL, NULL, hInst, NULL);
    }
    if (!g_hwnd) {
        MessageBoxA(NULL, "CreateWindow failed", "DuoDX", MB_ICONERROR);
        return 1;
    }

    gui_compute_cf_text(g_ui.freq, sizeof(g_ui.freq));
    gui_compute_coverage_span(g_ui.span, sizeof(g_ui.span));

    /* Fonts */
    g_hFontUI  = CreateFontA(-13, 0, 0, 0, FW_NORMAL, 0, 0, 0,
                    DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                    CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_SWISS, "Segoe UI");
    g_hFontVal = CreateFontA(-15, 0, 0, 0, FW_BOLD, 0, 0, 0,
                    DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                    CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_SWISS, "Segoe UI");
    g_hFontBig = CreateFontA(-22, 0, 0, 0, FW_BOLD, 0, 0, 0,
                    DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                    CLEARTYPE_QUALITY, FIXED_PITCH | FF_MODERN, "Consolas");
    g_hFontCarrier = CreateFontA(-16, 0, 0, 0, FW_NORMAL, 0, 0, 0,
                    DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                    CLEARTYPE_QUALITY, FIXED_PITCH | FF_MODERN, "Consolas");
    g_hFontLog = CreateFontA(-13, 0, 0, 0, FW_NORMAL, 0, 0, 0,
                    DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                    DEFAULT_QUALITY, FIXED_PITCH | FF_MODERN, "Consolas");

    /* Buttons (owner-drawn, positioned by layout_children) */
    g_hBtnToggle = mk_button(g_hwnd, IDC_BTN_TOGGLE, gui_record_btn_idle_label());
    g_hBtnAgc    = mk_button(g_hwnd, IDC_BTN_AGC,    "AGC");
    g_hBtnSchedToggle = mk_button(g_hwnd, IDC_BTN_SCHED_TOGGLE,
                                   (g_state.cfg.schedule_only || g_state.cfg.hourly_enable)
                                       ? "Timer: ON" : "Timer: OFF");
    g_hBtnSettings = mk_button(g_hwnd, IDC_BTN_SETTINGS, "Settings");

    /* Live monitor: ring buffer + demod/audio thread, then its controls. */
    monitor_global_init();
    monitor_create_controls(g_hwnd, hInst);
    gui_refresh_monitor_bar_visibility();

    /* Log control - RichEdit (per-line colour). Requires Msftedit.dll. */
    g_hLog = CreateWindowExA(0, "RICHEDIT50W", "",
                WS_CHILD | WS_VISIBLE | WS_BORDER | WS_VSCROLL | ES_MULTILINE |
                ES_AUTOVSCROLL | ES_READONLY,
                0, 0, 10, 10, g_hwnd, (HMENU)(INT_PTR)IDC_LOG,
                hInst, NULL);
    if (g_hLog) {
        /* Request dark-mode scrollbar from the OS theme engine (Win10+).
         * Loaded dynamically so the app still runs if uxtheme is absent. */
        HMODULE hUx = LoadLibraryA("uxtheme.dll");
        if (hUx) {
            typedef HRESULT (WINAPI *PFN_SWT)(HWND, LPCWSTR, LPCWSTR);
            PFN_SWT pSwt = (PFN_SWT)GetProcAddress(hUx, "SetWindowTheme");
            if (pSwt) pSwt(g_hLog, L"DarkMode_Explorer", NULL);
            FreeLibrary(hUx);
        }
        if (g_hFontLog)
            SendMessageA(g_hLog, WM_SETFONT, (WPARAM)g_hFontLog, TRUE);
        /* Dark background + default white-ish text. */
        SendMessageA(g_hLog, EM_SETBKGNDCOLOR, 0, (LPARAM)COL_BAR_BG);
        {
            CHARFORMAT2A cf;
            memset(&cf, 0, sizeof(cf));
            cf.cbSize = sizeof(cf);
            cf.dwMask = CFM_COLOR;
            cf.crTextColor = RGB(225, 235, 250);
            SendMessageA(g_hLog, EM_SETCHARFORMAT, SCF_ALL, (LPARAM)&cf);
        }
        /* Keep a generous backlog. */
        SendMessageA(g_hLog, EM_EXLIMITTEXT, 0, (LPARAM)(1024 * 1024));
    }

    gui_set_recording_ui(0);
    layout_children(g_hwnd);

    /* Seed the idle disk-free display and the clock settings by reading the
     * INI BEFORE the window is first shown, so the initial paint already
     * reflects show_clock and use_utc (otherwise the clock briefly appears
     * even when disabled, until the config loads). */
    {
        Config tmp;
        config_set_defaults(&tmp);
        config_load_ini(&tmp, g_config_file);
        ULARGE_INTEGER fb;
        char dir_path[MAX_PATH] = ".";
        const char *src = tmp.recording_path[0] ? tmp.recording_path
                                                : tmp.output_file;
        const char *last_sep = NULL, *p = src;
        for (; *p; p++) if (*p == '\\' || *p == '/') last_sep = p;
        if (tmp.recording_path[0])
            strncpy(dir_path, tmp.recording_path, sizeof(dir_path) - 1);
        else if (last_sep) {
            size_t len = (size_t)(last_sep - src) + 1;
            if (len < sizeof(dir_path)) { memcpy(dir_path, src, len); dir_path[len] = '\0'; }
        }
        if (GetDiskFreeSpaceExA(dir_path, &fb, NULL, NULL))
            g_ui.disk_free_mb = (double)(fb.QuadPart / (1024ULL * 1024ULL));

        /* Seed clock display settings from the same INI read. */
        g_clock_show  = tmp.show_clock;
        g_clock_utc   = tmp.use_utc;
        g_meter_style = tmp.meter_style;
    }

    ShowWindow(g_hwnd, g_state.cfg.window_maximized ? SW_MAXIMIZE : nShow);
    UpdateWindow(g_hwnd);

    LOG_INFO("DuoDX GUI ready. Press %s to begin recording.",
             gui_record_btn_idle_label());

    if (startup_duo_fix_applied)
        LOG_WARN("dual_channel/rspduo_single_tuner=B in duodx.ini looked "
                 "left over from a different (RSPduo) device - reset to "
                 "single-tuner A for this one.");

    /* 1-second timer to tick the live clock while idle. */
    SetTimer(g_hwnd, ID_TIMER_CLOCK, 1000, NULL);
    gui_refresh_idle_timer_text();
    g_ui.hdr_on = g_state.cfg.hdr_enable ? 1 : 0;

    MSG m;
    while (GetMessageA(&m, NULL, 0, 0) > 0) {
        TranslateMessage(&m);
        DispatchMessageA(&m);
    }

    /* Ensure worker is stopped on exit. */
    gui_stop_session(1);
    monitor_shutdown();

    if (g_hFontUI)  DeleteObject(g_hFontUI);
    if (g_hFontVal) DeleteObject(g_hFontVal);
    if (g_hFontBig) DeleteObject(g_hFontBig);
    if (g_hFontLog) DeleteObject(g_hFontLog);
    if (g_hFontFreqDigits) DeleteObject(g_hFontFreqDigits);
    if (g_hbrBg)    DeleteObject(g_hbrBg);
    if (g_hbrPanel) DeleteObject(g_hbrPanel);
    return (int)m.wParam;
}
