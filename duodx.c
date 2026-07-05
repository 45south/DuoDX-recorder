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
#include <stdint.h>
#include <inttypes.h>
#include <time.h>
#include <math.h>
#include <signal.h>
#include <commctrl.h>
#include <richedit.h>

#include "sdrplay_api.h"
/* RSP1B was added in API 3.14; older headers may not define this ID. */
#ifndef SDRPLAY_RSP1B_ID
#  define SDRPLAY_RSP1B_ID  6
#endif

/* =========================================================================
 * Constants and configuration defaults
 * ========================================================================= */

#define VERSION                 "2.1.2"

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
    FORMAT_SDRUNO    = 2,  /* RIFF/WAV with fmt+auxi chunks, 216-byte header */
    FORMAT_SDRCONNECT= 3   /* RIFF/WAV with JUNK padding, 80-byte header     */
} OutputFormat;

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
    int      dc_correct;
    int      iq_correct;
    int      notch_rf;
    int      notch_dab;
    int      dual_channel;      /* 1 = force RSPduo dual mode */
    double   freq_b_hz;         /* RSPduo tuner B frequency (dual mode) */
    int      ring_buffer_sec;   /* Override ring buffer duration */
    int      monitor_interval_ms; /* Status bar update interval (ms) */
    char     log_file[MAX_PATH_LEN];
    int      verbose;
    OutputFormat output_format;  /* FORMAT_LINRAD or FORMAT_WAVVIEWDX */


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
    int           schedule_repeat; /* 1 = repeat schedule nightly after all entries finish */

    int           use_utc;         /* 1=UTC timestamps (default), 0=local time */
    int           show_clock;      /* 1=show live clock in GUI (default), 0=hide */
    int           meter_style;     /* 0=zone colours (default), 1=graduated blend */
    int           hourly_enable;   /* 1 = enable hourly recording mode */
    int           hourly_window_min; /* recording window centred on the hour (minutes) */
    char          hourly_start[8]; /* window open time HH:MM */
    char          hourly_stop[8];  /* window close time HH:MM */
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

    /* Config */
    Config      cfg;

    /* Statistics (updated atomically by callback, read by monitor) */
    volatile LONG64 samples_received;    /* Total sample frames received */
    volatile LONG64 samples_written;     /* Total sample frames written */
    volatile LONG64 callback_count;      /* Number of callback invocations */
    volatile LONG   overflows;           /* Ring buffer overflow count */
    volatile LONG64 zero_frames_written;  /* Frames written as zero-fill gap compensation */
    volatile float  peak_dbfs;            /* Peak signal level dBFS, updated each callback */
    volatile float  peak_dbfs_b;          /* Peak dBFS for Tuner B (dual mode only)        */
    volatile int    overload_tuner_a;     /* 1=Tuner A overload active   */
    volatile int    overload_tuner_b;     /* 1=Tuner B overload active   */
    volatile int    stream_running;
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
#define IDC_BTN_NOW      1006

/* Private window messages */
#define WM_APP_LOG       (WM_APP + 1)   /* lParam = char* (heap), append to log  */
#define WM_APP_DONE      (WM_APP + 3)   /* worker thread finished                */

/* 1-second timer drives the live clock display. */
#define ID_TIMER_CLOCK   1

/* ---- Theme colours (deep navy / cyan / white) -------------------------- */
#define COL_BG        RGB(13, 27, 53)     /* window background  - deep navy    */
#define COL_PANEL     RGB(20, 40, 74)     /* meter/counter panel - lighter navy*/
#define COL_PANEL_EDGE RGB(45, 80, 130)   /* panel border                      */
#define COL_TEXT      RGB(232, 240, 255)  /* primary text - near white         */
#define COL_TEXT_DIM  RGB(150, 175, 210)  /* labels - muted blue-grey          */
#define COL_ACCENT    RGB(80, 200, 255)   /* cyan accent / values              */
#define COL_LED_OFF   RGB(60, 40, 40)     /* recording LED when idle           */
#define COL_LED_ON    RGB(255, 0, 0)      /* recording LED when active         */
#define COL_BAR_BG    RGB(8, 16, 32)      /* meter track background            */
#define COL_SEG_GREEN RGB(40, 220, 90)
#define COL_SEG_AMBER RGB(255, 190, 40)
#define COL_SEG_RED   RGB(255, 60, 50)
#define COL_BTN_FACE  RGB(30, 58, 100)
#define COL_BTN_HOT   RGB(45, 85, 140)
#define COL_BTN_DIS   RGB(22, 38, 62)
#define COL_BTN_START RGB(28, 96, 62)    /* green-ish: idle toggle = "Start"  */
#define COL_BTN_STOP  RGB(140, 44, 44)   /* red-ish:   recording toggle="Stop"*/

/* GUI globals */
static HWND   g_hwnd        = NULL;
static HWND   g_hLog        = NULL;
static HWND   g_hBtnToggle  = NULL;
static HWND   g_hBtnAgc     = NULL;
static HWND   g_hBtnNow     = NULL;   /* Start Now — visible only while waiting */
static HFONT  g_hFontLog    = NULL;
static HFONT  g_hFontUI     = NULL;   /* labels                              */
static HFONT  g_hFontVal    = NULL;   /* bold values / counters              */
static HFONT  g_hFontBig    = NULL;   /* big 7-seg-ish counters              */
static HBRUSH g_hbrBg       = NULL;
static HBRUSH g_hbrPanel    = NULL;

static HANDLE g_worker_thread  = NULL;   /* recording_worker thread             */
static HANDLE g_gui_mon_thread = NULL;   /* GUI status monitor thread           */
static volatile int g_worker_active = 0; /* 1 while a recording session runs    */
static volatile int g_agc_toggle_req = 0;/* set by AGC button, serviced in engine*/
/* (LED is static while recording) */

static char   g_config_file[MAX_PATH_LEN] = CONFIG_FILE;

/* Clock display: read from the INI at startup and on each Start, so the GUI
 * can show a live clock even while idle (before the worker loads config). */
static volatile int g_clock_show   = 1;   /* 1 = show live clock              */
static volatile int g_clock_utc    = 1;   /* 1 = UTC, 0 = local               */
static volatile int g_meter_style  = 0;   /* 0 = zone, 1 = graduated          */
static volatile int g_record_now   = 0;  /* set by Record Now button: run one ad-hoc recording then resume wait */

/* ---- Live UI snapshot ---------------------------------------------------
 * The GUI monitor thread fills this; WM_PAINT reads it. Plain scalars,
 * updated/read atomically enough for display purposes.                     */
typedef struct {
    int    recording;     /* 1 = stream live                                 */
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
    long   overflows;
    long long dropped;    /* zero-fill frames                                */
    float  ring_pct;      /* ring buffer fill 0..100 (percent, one decimal)  */
    char   state[48];
    char   next[32];
    char   freq[48];
    char   span[64];      /* coverage range, e.g. "150 - 1750 kHz"          */
    char   sched[96];     /* scheduling status line for the bottom bar       */
} UiSnapshot;

static UiSnapshot g_ui;   /* zero-initialised                                */

/* Forward declarations for the GUI front-end */
static DWORD WINAPI recording_worker(LPVOID param);
static DWORD WINAPI gui_monitor_thread_func(LPVOID param);
static void  gui_apply_agc_toggle(AppState *state);
static inline SIZE_T ring_available(const RingBuffer *rb);

/* -------------------------------------------------------------------------
 * log_write - GUI version. Formats the message and posts a heap-allocated
 * string to the main window, which appends it to the log edit control on
 * the UI thread. Also writes to the log file if one is open. Thread-safe.
 * ------------------------------------------------------------------------- */
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
}

#define LOG_INFO(...)  log_write("INFO ", __VA_ARGS__)
#define LOG_OK(...)    log_write("OK   ", __VA_ARGS__)
#define LOG_WARN(...)  log_write("WARN ", __VA_ARGS__)
#define LOG_ERROR(...) log_write("ERROR", __VA_ARGS__)

/* -------------------------------------------------------------------------
 * gui_apply_agc_toggle - toggle AGC on the live stream. Extracted from the
 * original keyboard handler's 'G' key. Called from the engine's main wait
 * loop when the AGC button has been pressed (g_agc_toggle_req).
 * ------------------------------------------------------------------------- */
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
        ? sdrplay_api_Tuner_Both : sdrplay_api_Tuner_A;

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
static DWORD WINAPI gui_monitor_thread_func(LPVOID param)
{
    AppState *state = (AppState *)param;

    while (g_worker_active) {
        UiSnapshot s;
        memset(&s, 0, sizeof(s));

        s.recording = state->stream_running ? 1 : 0;
        s.finished  = (!state->stream_running && state->session_complete) ? 1 : 0;
        s.dual      = state->cfg.dual_channel;
        /* Coherent dual-channel indicator: only meaningful on an RSPduo with
         * dual_channel enabled and both tuners on the same frequency (the
         * condition for phase-coherent diversity reception - see Section 7.3
         * of the user guide). Hidden for any other device or configuration. */
        s.coherent = (s.recording && state->cfg.dual_channel &&
                      state->device.hwVer == SDRPLAY_RSPduo_ID &&
                      fabs(state->cfg.frequency_hz - state->cfg.freq_b_hz) < 1.0)
                     ? 1 : 0;

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

        /* State text */
        if (s.finished)
            strncpy(s.state, "FINISHED", sizeof(s.state) - 1);
        else if (s.recording)
            strncpy(s.state, "RECORDING", sizeof(s.state) - 1);
        else if (state->next_start[0])
            strncpy(s.state, "WAITING", sizeof(s.state) - 1);
        else
            strncpy(s.state, "IDLE", sizeof(s.state) - 1);

        strncpy(s.next, state->next_start, sizeof(s.next) - 1);

        /* Scheduling status for the bottom info line.
         * Only shown while waiting, not during active recording
         * (the recording LED makes it obvious what state we're in). */
        if (s.recording) {
            s.sched[0] = '\0';
        } else if (state->cfg.hourly_enable) {
            snprintf(s.sched, sizeof(s.sched),
                     "Hourly %d min%s%s%s",
                     state->cfg.hourly_window_min,
                     state->next_start[0] ? "  next " : "",
                     state->next_start[0] ? state->next_start : "",
                     state->cfg.schedule_repeat ? "  (repeat)" : "");
        } else if (state->cfg.schedule_only && state->cfg.schedule_count > 0) {
            snprintf(s.sched, sizeof(s.sched),
                     "Schedule: %d entr%s%s%s%s",
                     state->cfg.schedule_count,
                     state->cfg.schedule_count == 1 ? "y" : "ies",
                     state->next_start[0] ? "  next " : "",
                     state->next_start[0] ? state->next_start : "",
                     state->cfg.schedule_repeat ? "  (repeat)" : "");
        } else if (state->next_start[0]) {
            snprintf(s.sched, sizeof(s.sched), "Start at %s",
                     state->next_start);
        } else {
            s.sched[0] = '\0';
        }

        if (state->cfg.dual_channel)
            snprintf(s.freq, sizeof(s.freq), "A %.3f / B %.3f MHz",
                     state->cfg.frequency_hz / 1e6,
                     state->cfg.freq_b_hz / 1e6);
        else
            snprintf(s.freq, sizeof(s.freq), "%.3f MHz",
                     state->cfg.frequency_hz / 1e6);

        /* Coverage span = centre frequency +/- half the recorded IQ bandwidth.
         * The recorded bandwidth equals the output sample rate.            */
        {
            double rate = state->cfg.expected_output_rate_hz;
            if (rate <= 0.0) rate = state->cfg.sample_rate_hz;
            double half = rate / 2.0;
            double lo = state->cfg.frequency_hz - half;
            double hi = state->cfg.frequency_hz + half;
            if (lo < 0.0) lo = 0.0;
            /* Show in kHz for MW, MHz once we are above ~3 MHz centre. */
            if (state->cfg.frequency_hz < 3.0e6)
                snprintf(s.span, sizeof(s.span), "%.0f - %.0f kHz",
                         lo / 1e3, hi / 1e3);
            else
                snprintf(s.span, sizeof(s.span), "%.3f - %.3f MHz",
                         lo / 1e6, hi / 1e6);
        }

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
                 * The buttons are repainted only when their state changes.  */
                int log_bot = tl.y + (int)(lr.bottom - lr.top);
                int btn_top = cr.bottom - 36; /* top of button row           */
                if (log_bot < btn_top) {
                    RECT sched_strip = { 0, log_bot, cr.right, btn_top };
                    InvalidateRect(g_hwnd, &sched_strip, FALSE);
                }
            } else {
                InvalidateRect(g_hwnd, NULL, FALSE);
            }
        }
        /* Update AGC button state only when it changes, to avoid flicker. */
        if (g_hBtnAgc) {
            static int last_agc_on = -1;
            static int last_agc_enabled = -1;
            int agc_enabled = s.recording && !s.hdr_on;
            if (s.agc_on != last_agc_on || agc_enabled != last_agc_enabled) {
                EnableWindow(g_hBtnAgc, agc_enabled);
                InvalidateRect(g_hBtnAgc, NULL, FALSE);
                last_agc_on      = s.agc_on;
                last_agc_enabled = agc_enabled;
            }
        }
        /* Show Start Now only while waiting for a scheduled start. */
        if (g_hBtnNow) {
            static int last_waiting = -1;
            int waiting = !s.recording && !s.finished && g_state.cfg.start_time[0];
            if (waiting != last_waiting) {
                ShowWindow(g_hBtnNow, waiting ? SW_SHOW : SW_HIDE);
                last_waiting = waiting;
            }
        }

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
    } else if (cfg->output_format == FORMAT_SDRUNO) {
        /* SDRuno WAV - keep Z regardless of use_utc for compatibility */
        long long freq_khz = (long long)(cfg->frequency_hz / 1000.0 + 0.5);
        char freq_str[32];
        if (fabs(cfg->frequency_hz - (double)(freq_khz * 1000)) < 1.0)
            snprintf(freq_str, sizeof(freq_str), "%lldkHz", freq_khz);
        else
            snprintf(freq_str, sizeof(freq_str), "%.1fkHz", cfg->frequency_hz / 1000.0);
        snprintf(cfg->output_file, MAX_PATH_LEN,
                 "SDRuno_%04d%02d%02d_%02d%02d%02dZ_%s.wav",
                 st.wYear, st.wMonth, st.wDay,
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

static int write_linrad_header(HANDLE fh, const Config *cfg, int num_channels)
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
    hdr.passband_center    = cfg->frequency_hz / 1.0e6;
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
 * RIFF size patcher -- seeks back and updates riff_size and data_size.
 * Called before CloseHandle for SDRuno and SDR Connect formats.
 * Both fields were written as 0 in the initial header.
 * ========================================================================= */
static void patch_wav_sizes(HANDLE fh, const Config *cfg, LONG64 samples_written)
{
    int64_t  data_bytes  = samples_written * 4;
    uint32_t data_size32 = (data_bytes > 0xFFFFFFFCLL)
                           ? 0xFFFFFFFC : (uint32_t)data_bytes;
    int64_t  header_size = (cfg->output_format == FORMAT_SDRUNO) ? 216 : 80;
    uint32_t riff_size32 = (uint32_t)((header_size - 8 + data_bytes) & 0xFFFFFFFF);
    DWORD written; LARGE_INTEGER li;
    li.QuadPart = 4;
    if (SetFilePointerEx(fh, li, NULL, FILE_BEGIN))
        WriteFile(fh, &riff_size32, 4, &written, NULL);
    li.QuadPart = header_size - 4;
    if (SetFilePointerEx(fh, li, NULL, FILE_BEGIN))
        WriteFile(fh, &data_size32, 4, &written, NULL);
    LOG_INFO("WAV sizes patched: riff_size=%u  data_size=%u", riff_size32, data_size32);
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

        ok = WriteFile(state->out_file, g_zero_chunk, (DWORD)chunk,
                       &written, NULL);
        if (!ok || written != (DWORD)chunk) {
            LOG_ERROR("Gap fill WriteFile failed: error %lu", GetLastError());
            return 0;
        }

        /* Count zero frames separately so we can report them distinctly.
         * They are also added to samples_written because they occupy
         * real time slots in the recording timeline. */
        InterlockedAdd64(&state->zero_frames_written,
                         (LONG64)(chunk / frame_size));
        InterlockedAdd64(&state->samples_written,
                         (LONG64)(chunk / frame_size));
        remaining -= chunk;
    }

    return 1;
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

        ok = WriteFile(state->out_file, g_write_chunk, (DWORD)to_read,
                       &written, NULL);

        if (!ok || written != (DWORD)to_read) {
            DWORD err = GetLastError();
            LOG_ERROR("Write failed: Windows error %lu (wrote %lu of %zu bytes)",
                      err, written, to_read);
            state->writer_error = 1;
            break;
        }

        /* Non-blocking pipe write - silently skip if no client connected
         * or client buffer full. Never blocks or errors the recording.   */
        if (state->pipe_handle != INVALID_HANDLE_VALUE) {
            DWORD pipe_written;
            WriteFile(state->pipe_handle, g_write_chunk, (DWORD)to_read,
                      &pipe_written, NULL);
            /* Ignore return value intentionally - pipe is best-effort.   */
        }

        InterlockedAdd64(&state->samples_written,
                         (LONG64)(to_read / (state->cfg.dual_channel ? 8 : 4)));
    }

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

        for (i = 0; i < batch; i++) {
            tmp[i * 2]     = (int16_t)xi[offset + i];
            tmp[i * 2 + 1] = (int16_t)xq[offset + i];
        }

        ring_write(&state->ring, tmp, batch * 4);
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

        ring_write(&state->ring, state->dual_merge_buf,
                   (SIZE_T)merge_count * 8);
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
        if (state->cfg.verbose) {
            LOG_INFO("Gain change [%s]: lnaGRdB=%d, grDb=%d, currGain=%.1f",
                     tuner == sdrplay_api_Tuner_A ? "T1" :
                     tuner == sdrplay_api_Tuner_B ? "T2" : "Both",
                     params->gainParams.lnaGRdB,
                     params->gainParams.gRdB,
                     params->gainParams.currGain);
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
 * Low-IF mode (if_khz = 1620 or 2048):
 *   Only specific ADC sample rates are valid (hardware requirement).
 *   Internal decimation is applied automatically by the API.
 *   Output sample rate = ADC rate / internal decimation factor.
 *
 *   IF=1620 kHz, BW=1536 kHz:
 *     ADC 6 Msps  -> decimation /3 -> output 2 Msps   **recommended for MW**
 *     ADC 8 Msps  -> decimation /4 -> output 2 Msps
 *
 *   IF=2048 kHz, BW=1536 kHz:
 *     ADC 8 Msps  -> decimation /4 -> output 2 Msps
 *
 *   RSPduo dual-tuner mode additionally restricts to:
 *     ADC 6 Msps  (IF=1620, BW=1536) -> output 2 Msps
 *     ADC 8 Msps  (IF=2048, BW=1536) -> output 2 Msps
 *
 * All other combinations will either be rejected by the API or produce
 * unpredictable output sample rates.
 * ========================================================================= */
typedef struct {
    int    if_khz;          /* IF frequency in kHz */
    int    bw_khz;          /* IF bandwidth in kHz */
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
    /* Low-IF 1620 kHz - internal decimation /3, output always 2 Msps */
    { 1620, 1536, 6000000.0, 3, 2000000.0, 1, "Low-IF 1620kHz: 6Msps ADC -> 2Msps out (RSPduo dual OK)"},
    { 1620, 1536, 8000000.0, 4, 2000000.0, 0, "Low-IF 1620kHz: 8Msps ADC -> 2Msps out"},
    /* Low-IF 2048 kHz - internal decimation /4, output always 2 Msps */
    { 2048, 1536, 8000000.0, 4, 2000000.0, 1, "Low-IF 2048kHz: 8Msps ADC -> 2Msps out (RSPduo dual OK)"},
};
#define NUM_VALID_COMBOS (int)(sizeof(VALID_COMBOS)/sizeof(VALID_COMBOS[0]))

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
             * correctly upfront without any mid-recording seek/patch.    */
            cfg->expected_output_rate_hz = c->output_rate_hz;

            LOG_INFO("Config validated: %s", c->note);
            if (c->decimation > 1)
                LOG_INFO("  Internal decimation /%d: output will be %.0f sps",
                         c->decimation, c->output_rate_hz);
            return 1;
        }
    }

    /* Validate decimation factor */
    {
        int d = cfg->decimation;
        if (d != 1 && d != 2 && d != 4 && d != 8 && d != 16 && d != 32) {
            LOG_ERROR("Invalid decimation=%d. Valid values: 1, 2, 4, 8, 16, 32", d);
            return 0;
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
    cfg->dc_correct     = DEFAULT_DC_CORRECT;
    cfg->iq_correct     = DEFAULT_IQ_CORRECT;
    cfg->notch_rf       = DEFAULT_NOTCH_RF;
    cfg->notch_dab      = DEFAULT_NOTCH_DAB;
    cfg->dual_channel   = 0;
    cfg->freq_b_hz      = DEFAULT_FREQUENCY_HZ;
    cfg->ring_buffer_sec      = RING_BUFFER_SECONDS;
    cfg->monitor_interval_ms  = DEFAULT_MONITOR_INTERVAL_MS;
    cfg->verbose        = 0;
    cfg->output_format  = FORMAT_LINRAD;

    strncpy(cfg->antenna, "A", 7);  /* default to Antenna A / 50 ohm */
    cfg->bias_t         = 0;
    cfg->hiz_notch      = 0;
    cfg->hdr_enable     = 0;
    cfg->hdr_bw_khz     = 1700;
    cfg->ppm            = 0.0;
    cfg->device_serial[0] = '\0'; /* empty = use first device found */
    cfg->decimation     = 1;      /* no additional decimation */

    cfg->expected_output_rate_hz = 0.0; /* set by validate_config */
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
    cfg->schedule_repeat = 0;
    cfg->use_utc         = 1;
    cfg->show_clock      = 1;
    cfg->meter_style     = 0;
    cfg->hourly_enable      = 0;
    cfg->hourly_window_min  = 10;
    strncpy(cfg->hourly_start, "18:00", 7);
    strncpy(cfg->hourly_stop,  "06:00", 7);
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
        else if (!strcmp(key, "duration_sec"))   cfg->duration_sec   = atoi(val);
        else if (!strcmp(key, "agc_enable"))     cfg->agc_enable     = atoi(val);
        else if (!strcmp(key, "dc_correct"))     cfg->dc_correct     = atoi(val);
        else if (!strcmp(key, "iq_correct"))     cfg->iq_correct     = atoi(val);
        else if (!strcmp(key, "notch_rf"))       cfg->notch_rf       = atoi(val);
        else if (!strcmp(key, "notch_dab"))      cfg->notch_dab      = atoi(val);
        else if (!strcmp(key, "dual_channel"))   cfg->dual_channel   = atoi(val);
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
        else if (!strcmp(key, "schedule_repeat"))    cfg->schedule_repeat = atoi(val);
        else if (!strcmp(key, "use_utc"))            cfg->use_utc = atoi(val);
        else if (!strcmp(key, "show_clock"))         cfg->show_clock = atoi(val);
        else if (!strcmp(key, "meter_style"))        cfg->meter_style = atoi(val);
        else if (!strcmp(key, "hourly_enable"))      cfg->hourly_enable = atoi(val);
        else if (!strcmp(key, "hourly_window_min"))  cfg->hourly_window_min = atoi(val);
        else if (!strcmp(key, "hourly_start")) strncpy(cfg->hourly_start, val, 7);
        else if (!strcmp(key, "hourly_stop"))  strncpy(cfg->hourly_stop,  val, 7);
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
                        e->duration_sec = atoi(val);
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
            else
                cfg->output_format = FORMAT_LINRAD;
        }
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
    (void)state->dev_params; /* dp not used in this function */

    /* ── RSPdx / RSPdx R2 ─────────────────────────────────────────────── */
    /* Antenna and Bias-T are set before Init in setup_device_single.
     * No post-Init Update call needed or supported for these fields. */
    if (hw == SDRPLAY_RSPdx_ID || hw == SDRPLAY_RSPdxR2_ID) {
        LOG_INFO("RSPdx antenna: %s%s", cfg->antenna,
                 cfg->bias_t ? "  Bias-T: enabled" : "");
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
        if (err != sdrplay_api_Success)
            LOG_WARN("RSP2 antenna select failed: %s", sdrplay_api_GetErrorString(err));
        else
            LOG_INFO("RSP2 antenna set to: %s", cfg->antenna);

        if (cfg->bias_t)
            LOG_WARN("RSP2 does not support Bias-T - bias_t ignored");
        return;
    }

    /* ── RSPduo ────────────────────────────────────────────────────────── */
    if (hw == SDRPLAY_RSPduo_ID) {

        /* Tuner 1 AM port (antenna) selection */
        int use_hiz = (!strcmp(cfg->antenna, "Hi-Z") ||
                       !strcmp(cfg->antenna, "hi-z") ||
                       !strcmp(cfg->antenna, "HIZ"));

        chA->rspDuoTunerParams.tuner1AmPortSel = use_hiz
            ? sdrplay_api_RspDuo_AMPORT_1   /* Hi-Z */
            : sdrplay_api_RspDuo_AMPORT_2;  /* 50 ohm (default) */

        err = sdrplay_api_Update(state->device.dev, state->device.tuner,
                                 sdrplay_api_Update_RspDuo_AmPortSelect,
                                 sdrplay_api_Update_Ext1_None);
        if (err != sdrplay_api_Success)
            LOG_WARN("RSPduo AM port select failed: %s", sdrplay_api_GetErrorString(err));
        else
            LOG_INFO("RSPduo Tuner 1 port: %s", use_hiz ? "Hi-Z" : "50 ohm");

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

        /* Bias-T on Tuner 2 (50 ohm port only) */
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
                    LOG_INFO("RSPduo Tuner 2 Bias-T enabled");
            } else {
                LOG_WARN("RSPduo Bias-T requires dual-channel mode or Tuner 2 active");
            }
        }
        return;
    }

    /* ── RSP1A / RSP1B ─────────────────────────────────────────────────── */
    if (hw == SDRPLAY_RSP1A_ID) {
        if (strcmp(cfg->antenna, "A") != 0)
            LOG_WARN("RSP1A has only one antenna input - antenna=%s ignored",
                     cfg->antenna);
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
    if (cfg->bias_t)
        LOG_WARN("This device does not support Bias-T - bias_t ignored");
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

        /* Tuner A notches */
        if (cfg->notch_rf) {
            state->ch_a_params->rspDuoTunerParams.rfNotchEnable = 1;
            err = sdrplay_api_Update(state->device.dev, sdrplay_api_Tuner_A,
                                     sdrplay_api_Update_RspDuo_RfNotchControl,
                                     sdrplay_api_Update_Ext1_None);
            if (err != sdrplay_api_Success)
                LOG_WARN("RSPduo T1 RF notch failed: %s", sdrplay_api_GetErrorString(err));
            else
                LOG_INFO("RSPduo Tuner A RF notch enabled");
        }
        if (cfg->notch_dab) {
            state->ch_a_params->rspDuoTunerParams.rfDabNotchEnable = 1;
            err = sdrplay_api_Update(state->device.dev, sdrplay_api_Tuner_A,
                                     sdrplay_api_Update_RspDuo_RfDabNotchControl,
                                     sdrplay_api_Update_Ext1_None);
            if (err != sdrplay_api_Success)
                LOG_WARN("RSPduo T1 DAB notch failed: %s", sdrplay_api_GetErrorString(err));
            else
                LOG_INFO("RSPduo Tuner A DAB notch enabled");
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
    LOG_INFO("Scheduled start: waiting until %02d:%02d:%02d",
             target_h, target_m, target_s);

    while (g_running) {
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

        if (diff <= 0 || g_record_now) {
            LOG_INFO("Scheduled start time reached");
            return 1;
        }

        /* GUI build: countdown shown via g_state.next_start in status bar */
        snprintf(g_state.next_start, sizeof(g_state.next_start),
                 "%02d:%02d:%02d", target_h, target_m, target_s);
        if (!g_worker_active) {
            snprintf(g_ui.sched, sizeof(g_ui.sched),
                     "Waiting for scheduled start: %02d:%02d:%02d",
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
        state->cfg.frequency_hz = e->frequency_hz;
        if (state->stream_running) {
            state->ch_a_params->tunerParams.rfFreq.rfHz = e->frequency_hz;
            err = sdrplay_api_Update(state->device.dev, sdrplay_api_Tuner_A,
                                     sdrplay_api_Update_Tuner_Frf,
                                     sdrplay_api_Update_Ext1_None);
            if (err == sdrplay_api_Success)
                LOG_INFO("Schedule: Tuner A frequency set to %.6f MHz",
                         e->frequency_hz / 1e6);
            else
                LOG_WARN("Schedule: Tuner A frequency update failed: %s",
                         sdrplay_api_GetErrorString(err));
        } else {
            LOG_INFO("Schedule: Tuner A frequency will be %.6f MHz",
                     e->frequency_hz / 1e6);
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

    /* SDRuno and SDR Connect: RIFF ChunkSize was patched before CloseHandle.
     * Read it back and compare to expected from sample count.             */
    if (cfg->output_format == FORMAT_SDRUNO ||
            cfg->output_format == FORMAT_SDRCONNECT) {
        HANDLE fh2 = CreateFileA(cfg->output_file, GENERIC_READ, FILE_SHARE_READ,
                                 NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
        if (fh2 == INVALID_HANDLE_VALUE) {
            LOG_WARN("Verification: could not open '%s' (error %lu).",
                     cfg->output_file, GetLastError());
            return;
        }
        LARGE_INTEGER fsz; uint32_t riff_sz = 0; DWORD nr;
        GetFileSizeEx(fh2, &fsz);
        LARGE_INTEGER li2; li2.QuadPart = 4;
        SetFilePointerEx(fh2, li2, NULL, FILE_BEGIN);
        ReadFile(fh2, &riff_sz, 4, &nr, NULL);
        CloseHandle(fh2);

        int64_t hdr_sz         = (cfg->output_format == FORMAT_SDRUNO) ? 216 : 80;
        int64_t expected_data  = samples_written * 4;
        int64_t expected_total = hdr_sz + expected_data;
        int64_t actual_total   = (int64_t)riff_sz + 8;
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
    while (g_running) {
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

        /* Calculate the pre-record window start time (HH:MM:SS) */
        int top_sec = (cur_sec + secs_to_next) % 86400;
        int disp_h  = top_sec / 3600;
        int disp_m  = (top_sec % 3600) / 60;
        int disp_s  = top_sec % 60;

        /* GUI build: progress shown via g_state.next_start in status bar */
        (void)disp_h; (void)disp_m; (void)disp_s;

        /* Update next_start for HTTP dashboard - show the hour boundary */
        int hour_boundary = ((disp_h * 60 + disp_m + (half_win_sec / 60)) % (24 * 60));
        snprintf(g_state.next_start, sizeof(g_state.next_start),
                 "%02d:%02d:00",
                 hour_boundary / 60,
                 hour_boundary % 60);

        /* Update the GUI scheduling text directly so it shows while the
         * monitor thread is not running (between hourly recordings).     */
        if (!g_worker_active) {
            snprintf(g_ui.sched, sizeof(g_ui.sched),
                     "Hourly %d min  next %s%s",
                     g_state.cfg.hourly_window_min,
                     g_state.next_start,
                     g_state.cfg.schedule_repeat ? "  (repeat)" : "");
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
    g_state.pipe_handle      = INVALID_HANDLE_VALUE;
    g_state.frozen_file_mb   = -1;
    g_state.last_display_file_mb = -1;
    g_state.last_display_elapsed = 0.0;
    InitializeCriticalSection(&g_state.dual_lock);

    QueryPerformanceFrequency(&g_state.perf_freq);

    /* Load defaults then INI config (no CLI args in the GUI build). */
    config_set_defaults(&g_state.cfg);
    config_load_ini(&g_state.cfg, config_file);

    LOG_INFO("DuoDX GUI v%s  (c) 2026 Dave Headland", VERSION);
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
            if (free_mb >= 1024)
                LOG_INFO("Free space on target drive: %.1f GB", free_mb / 1024.0);
            else
                LOG_INFO("Free space on target drive: %lld MB", (long long)free_mb);
        }
    }

    /* Open log file if specified */
    if (g_state.cfg.log_file[0]) {
        g_state.log_fp = fopen(g_state.cfg.log_file, "a");
        if (!g_state.log_fp)
            LOG_WARN("Could not open log file '%s'", g_state.cfg.log_file);
    }

    /* Print effective configuration */
    LOG_INFO("Configuration:");
    if (g_state.cfg.recording_path[0])
        LOG_INFO("  Recording path : %s", g_state.cfg.recording_path);
    LOG_INFO("  Output file    : %s", g_state.cfg.output_file);
    LOG_INFO("  Sample rate    : %.3f Msps (ADC rate; actual output may differ with IF mode)",
             g_state.cfg.sample_rate_hz / 1e6);
    LOG_INFO("  IF frequency   : %d kHz", g_state.cfg.if_khz);
    LOG_INFO("  IF bandwidth   : %d kHz", g_state.cfg.bw_khz);
    if (g_state.cfg.dual_channel) {
        int gr_b  = (g_state.cfg.gain_reduction_b >= 0) ? g_state.cfg.gain_reduction_b : g_state.cfg.gain_reduction;
        int lna_b = (g_state.cfg.lna_state_b      >= 0) ? g_state.cfg.lna_state_b      : g_state.cfg.lna_state;
        int indep = (g_state.cfg.gain_reduction_b >= 0 || g_state.cfg.lna_state_b >= 0);
        /* Use LOG_OK (green) only when the hardware is confirmed RSPduo */
        if (g_state.device.hwVer == SDRPLAY_RSPduo_ID)
            LOG_OK(  "  Dual channel   : YES (RSPduo)");
        else
            LOG_INFO("  Dual channel   : YES");
        LOG_INFO("  Tuner A        : %.6f MHz  Gain: %d dB  LNA: %d",
                 g_state.cfg.frequency_hz / 1e6,
                 g_state.cfg.gain_reduction,
                 g_state.cfg.lna_state);
        LOG_INFO("  Tuner B        : %.6f MHz  Gain: %d dB  LNA: %d  (%s)",
                 g_state.cfg.freq_b_hz / 1e6, gr_b, lna_b,
                 indep ? "independent" : "same as A");
    } else {
        LOG_INFO("  Frequency      : %.6f MHz", g_state.cfg.frequency_hz / 1e6);
        LOG_INFO("  Gain reduction : %d dB  LNA: %d",
                 g_state.cfg.gain_reduction, g_state.cfg.lna_state);
    }
    {
        int ds = g_state.cfg.duration_sec;
        if (ds > 0)
            LOG_INFO("  Duration       : limited (%02d:%02d:%02d)",
                     ds / 3600, (ds % 3600) / 60, ds % 60);
        else
            LOG_INFO("  Duration       : unlimited (Stop to end)");
    }
    if (g_state.cfg.start_time[0])
        LOG_INFO("  Scheduled start: %s", g_state.cfg.start_time);
    if (g_state.cfg.dual_channel) {
        /* Tuner 1 can be Hi-Z or 50 ohm; Tuner 2 is always 50 ohm, no selection */
        const char *t1_port =
            (!strcmp(g_state.cfg.antenna, "Hi-Z") ||
             !strcmp(g_state.cfg.antenna, "hi-z") ||
             !strcmp(g_state.cfg.antenna, "HIZ"))
            ? "Hi-Z" : "50 ohm";
        LOG_INFO("  Antenna        : T1=%s  T2=50 ohm (fixed)", t1_port);
    } else {
        LOG_INFO("  Antenna        : %s", g_state.cfg.antenna);
    }
    if (g_state.cfg.bias_t)
        LOG_OK(  "  Bias-T         : ENABLED");
    else
        LOG_INFO("  Bias-T         : Off");
    if (g_state.cfg.agc_enable) {
        LOG_INFO("  AGC            : ENABLED");
        LOG_WARN("AGC is enabled: the hardware AGC loop will continuously "
                 "override gain_reduction (%d dB) to hit its target level. "
                 "Changing gain_reduction will have little or no visible "
                 "effect while AGC is on. Set agc_enable=0 for manual gain "
                 "control to take effect.", g_state.cfg.gain_reduction);
    } else
        LOG_INFO("  AGC            : Off");
    /* HDR validation runs later in apply_hdr_mode(). Display as white here
     * regardless of hdr_enable; apply_hdr_mode() will log a green confirmation
     * if the configuration is valid, or an error if not.                   */
    if (g_state.cfg.hdr_enable)
        LOG_INFO("  HDR mode       : ENABLED (BW=%d kHz)", g_state.cfg.hdr_bw_khz);
    else
        LOG_INFO("  HDR mode       : Off");
    if (g_state.cfg.hiz_notch)
        LOG_INFO("  Hi-Z AM notch  : ENABLED");
    if (g_state.cfg.ppm != 0.0)
        LOG_INFO("  PPM correction : %.2f ppm", g_state.cfg.ppm);
    if (g_state.cfg.device_serial[0])
        LOG_INFO("  Device serial  : %s", g_state.cfg.device_serial);
    if (g_state.cfg.decimation > 1)
        LOG_INFO("  SW decimation  : /%d", g_state.cfg.decimation);
    LOG_INFO("  Ring buffer    : %d seconds", g_state.cfg.ring_buffer_sec);
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
        if (e->frequency_hz > 0.0)
            g_state.cfg.frequency_hz = e->frequency_hz;
        if (e->freq_b_hz > 0.0)
            g_state.cfg.freq_b_hz = e->freq_b_hz;
        if (e->duration_sec > 0)
            g_state.cfg.duration_sec = e->duration_sec;
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
    LOG_INFO("SDRplay API opened successfully");

    /* Check API version */
    {
        float ver;
        sdrplay_api_ApiVersion(&ver);
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

    err = sdrplay_api_GetDevices(devices, &num_devices, 6);
    if (err != sdrplay_api_Success || num_devices == 0) {
        LOG_ERROR("No SDRplay devices found (err=%s)", sdrplay_api_GetErrorString(err));
        sdrplay_api_UnlockDeviceApi();
        rc = 1;
        goto cleanup_api;
    }

    LOG_INFO("Found %u SDRplay device(s):", num_devices);
    for (i = 0; i < num_devices; i++) {
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
        /* Only set RSPduo-specific fields once device type is confirmed */
        g_state.device.rspDuoMode      = sdrplay_api_RspDuoMode_Dual_Tuner;
        g_state.device.tuner           = sdrplay_api_Tuner_Both;
        g_state.device.rspDuoSampleFreq = g_state.cfg.sample_rate_hz;
    } else if (g_state.device.hwVer == SDRPLAY_RSPduo_ID) {
        /* RSPduo in single-tuner mode: rspDuoMode and tuner must be set
         * explicitly before SelectDevice, otherwise the API leaves them at
         * their zero-initialised defaults (RspDuoMode_Unknown / Tuner_Neither)
         * and subsequent sdrplay_api_Update calls for gain, LNA state, and AGC
         * silently fail - the hardware does not respond to any parameter changes
         * during recording.                                                      */
        g_state.device.rspDuoMode      = sdrplay_api_RspDuoMode_Single_Tuner;
        g_state.device.tuner           = sdrplay_api_Tuner_A;
        /* rspDuoSampleFreq is only used in dual-tuner master mode;
         * leave it at 0 for single-tuner mode.                    */
        LOG_INFO("RSPduo single-tuner mode: Tuner A selected");
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

    g_state.ch_a_params = g_state.dev_params->rxChannelA;
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

        /* duration_sec: 0 = unlimited, otherwise must be positive */
        if (g_state.cfg.duration_sec < 0) {
            LOG_ERROR("duration_sec=%d is invalid. Use 0 for unlimited or a "
                      "positive number of seconds.", g_state.cfg.duration_sec);
            goto cleanup_device;
        }

        /* ring_buffer_sec: must be at least 1 second */
        if (g_state.cfg.ring_buffer_sec < 1) {
            LOG_ERROR("ring_buffer_sec=%d is invalid (minimum 1).",
                      g_state.cfg.ring_buffer_sec);
            goto cleanup_device;
        }

        /* ppm: sanity check — values beyond ±100 ppm are almost certainly wrong */
        if (g_state.cfg.ppm < -100.0 || g_state.cfg.ppm > 100.0) {
            LOG_ERROR("ppm=%.1f is outside the expected range (-100 to +100 ppm). "
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

        /* monitor_interval_ms range */
        if (g_state.cfg.monitor_interval_ms < 100 ||
            g_state.cfg.monitor_interval_ms > 5000) {
            LOG_WARN("monitor_interval_ms=%d is outside recommended range "
                     "(100-5000). Using 500 ms.",
                     g_state.cfg.monitor_interval_ms);
            g_state.cfg.monitor_interval_ms = 500;
        }

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
    /* Step 5b: Scheduled start / pre-recording delay + recording loop    */
    /* ------------------------------------------------------------------ */

    /* Hourly mode: validate settings and set duration */
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
        /* Override duration_sec and clear schedule entries */
        g_state.cfg.duration_sec    = g_state.cfg.hourly_window_min * 60;
        g_state.cfg.schedule_count  = 0;
        g_state.cfg.start_time[0] = '\0';
        LOG_INFO("Hourly mode: %d minute window, session %s-%s",
                 g_state.cfg.hourly_window_min,
                 g_state.cfg.hourly_start,
                 g_state.cfg.hourly_stop);
    }

hourly_next:
    if (g_state.cfg.hourly_enable) {
        int start_min    = hhmm_to_min(g_state.cfg.hourly_start);
        int stop_min     = hhmm_to_min(g_state.cfg.hourly_stop);
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
            /* Outside session - wait using seconds precision for accuracy */
            int eff_start_sec = start_min * 60 - half_win_sec;
            if (eff_start_sec < 0) eff_start_sec += 86400;
            LOG_INFO("Hourly: waiting for first recording window at %02d:%02d:%02d",
                     eff_start_sec / 3600, (eff_start_sec % 3600) / 60,
                     eff_start_sec % 60);
            while (g_running) {
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
            if (!g_running) { rc = 0; goto cleanup_device; }
        }

        /* Wait for next hour pre-record window.
         * If we just entered the session at the top of an hour and are
         * already within the pre-record window, start immediately.     */
        if (!hourly_wait_for_next(half_win_sec)) {
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
    }

repeat_schedule:
    { int was_adhoc = 0;
    do {
        /* Apply schedule entry if we have one */
        if (g_state.cfg.schedule_count > 0 && sched_idx > 0) {
            ScheduleEntry *e = &g_state.cfg.schedule[sched_idx - 1];
            LOG_INFO("Schedule entry %d of %d",
                     sched_idx, g_state.cfg.schedule_count);
            apply_schedule_entry(&g_state, e);
            /* Wait for this entry's start time if specified */
            if (e->start_time[0]) {
                if (!wait_until_time(e->start_time, 1)) {
                    rc = 0;
                    goto cleanup_writer;
                }
            }
        } else {
            /* First (or only) recording - use top-level scheduled start.
             * Loop so Record Now can run an ad-hoc recording then resume
             * waiting for the original scheduled time.                   */
            while (g_state.cfg.start_time[0] && g_running) {
                if (!wait_until_time(g_state.cfg.start_time, 0)) {
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
                /* Record Now: set flag so the do-while loop resets and waits again */
                g_state.adhoc_recording = 1;
                g_record_now = 0;
                LOG_INFO("Record Now: running ad-hoc recording, will resume wait for %s afterwards.",
                         g_state.cfg.start_time);
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

        if (need_autogen)
            generate_output_filename(&g_state.cfg, num_channels);

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
        }
    }

    {
        const char *fmt_name =
            g_state.cfg.output_format == FORMAT_WAVVIEWDX  ? "WavViewDX-raw" :
            g_state.cfg.output_format == FORMAT_SDRUNO     ? "SDRuno WAV (216-byte header)" :
            g_state.cfg.output_format == FORMAT_SDRCONNECT ? "SDR Connect WAV (80-byte header)" :
                                                              "Linrad (41-byte header)";
        LOG_INFO("Output format : %s", fmt_name);
        if (g_state.cfg.dual_channel &&
                (g_state.cfg.output_format == FORMAT_SDRUNO ||
                 g_state.cfg.output_format == FORMAT_SDRCONNECT)) {
            LOG_ERROR("Output format '%s' does not support dual-channel mode.",
                      fmt_name);
            LOG_ERROR("Set dual_channel=0 and specify which tuner to record:");
            LOG_ERROR("  Tuner 1 (A): dual_channel=0  antenna=Hi-Z  (or 50ohm)");
            LOG_ERROR("  Tuner 2 (B): dual_channel=0  (connect antenna to Tuner 2 port)");
            rc = 1;
            goto cleanup_ring;
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
        goto cleanup_ring;
    }

    if (g_state.cfg.output_format == FORMAT_LINRAD) {
        if (!write_linrad_header(g_state.out_file, &g_state.cfg, num_channels)) {
            rc = 1; goto cleanup_file;
        }
        FlushFileBuffers(g_state.out_file);
    } else if (g_state.cfg.output_format == FORMAT_SDRUNO) {
        if (!write_sdruno_header(g_state.out_file, &g_state.cfg)) {
            rc = 1; goto cleanup_file;
        }
        FlushFileBuffers(g_state.out_file);
    } else if (g_state.cfg.output_format == FORMAT_SDRCONNECT) {
        if (!write_sdrconnect_header(g_state.out_file, &g_state.cfg)) {
            rc = 1; goto cleanup_file;
        }
        FlushFileBuffers(g_state.out_file);
    }

    LOG_INFO("Output file   : %s (%d channel(s), 16-bit)",
             g_state.cfg.output_file, num_channels);

    /* ------------------------------------------------------------------ */
    /* Pre-recording disk space check                                       *
     * Calculate available recording time from free space and warn if it   *
     * is less than the requested duration (or less than 1 hour if         *
     * recording indefinitely).                                             *
     * ------------------------------------------------------------------ */
    {
        char dir_path[MAX_PATH] = ".";
        const char *last_sep = NULL;
        const char *p2 = g_state.cfg.output_file;
        for (; *p2; p2++)
            if (*p2 == '\\' || *p2 == '/') last_sep = p2;
        if (last_sep) {
            size_t len = (size_t)(last_sep - g_state.cfg.output_file) + 1;
            if (len < sizeof(dir_path)) {
                memcpy(dir_path, g_state.cfg.output_file, len);
                dir_path[len] = '\0';
            }
        }

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
    LOG_INFO("Writer thread started");

    /* ------------------------------------------------------------------ */
    /* Step 8: Set up and start streaming                                  */
    /* ------------------------------------------------------------------ */
    memset(&callbacks, 0, sizeof(callbacks));
    callbacks.EventCbFn = event_callback;

    if (g_state.cfg.dual_channel) {
        callbacks.StreamACbFn = stream_callback_dual_a;
        callbacks.StreamBCbFn = stream_callback_dual_b;
    } else {
        callbacks.StreamACbFn = stream_callback_single;
    }

    LOG_INFO("Starting stream...");
    err = sdrplay_api_Init(g_state.device.dev, &callbacks, &g_state);
    if (err != sdrplay_api_Success) {
        LOG_ERROR("sdrplay_api_Init: %s", sdrplay_api_GetErrorString(err));
        rc = 1;
        goto cleanup_writer;
    }

    g_state.stream_running = 1;
    g_recording = 1;  /* legacy flag; harmless in GUI build */
    QueryPerformanceCounter(&g_state.start_time);
    g_state.next_start[0] = '\0';  /* clear waiting indicator now recording */

    /* Apply antenna, Bias-T, Hi-Z notch, then RF/DAB notch filters */
    apply_antenna_and_biast(&g_state);
    apply_notch_filters(&g_state);

    /* Create named pipe for real-time monitoring if enabled.
     * FILE_FLAG_OVERLAPPED + PIPE_NOWAIT: writes never block the writer
     * thread. If no client is connected or the client buffer is full,
     * WriteFile to the pipe returns immediately with ERROR_PIPE_LISTENING
     * or ERROR_NO_DATA - both are silently ignored.
     * The pipe carries the raw IQ stream including the Linrad header,
     * identical to what is written to the file.                          */
    if (g_state.cfg.pipe_enable) {
        g_state.pipe_handle = CreateNamedPipeA(
            g_state.cfg.pipe_name,
            PIPE_ACCESS_OUTBOUND | FILE_FLAG_OVERLAPPED,
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
    g_state.session_complete = (sched_idx >= g_state.cfg.schedule_count) ? 1 : 0;

    /* Next scheduled start time for dashboard (empty if last recording) */
    memset(g_state.next_start, 0, sizeof(g_state.next_start));
    if (sched_idx < g_state.cfg.schedule_count &&
            g_state.cfg.schedule[sched_idx].start_time[0])
        strncpy(g_state.next_start,
                g_state.cfg.schedule[sched_idx].start_time,
                sizeof(g_state.next_start) - 1);
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
    LOG_INFO("Stopping stream...");

    sdrplay_api_Uninit(g_state.device.dev);

    /* Let writer drain the remaining ring buffer data */
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
        LOG_INFO("  Duration       : %.2f seconds", elapsed);
        LOG_INFO("  Samples rx     : %lld", (long long)g_state.samples_received);
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
    LOG_INFO("  Output file    : %s", g_state.cfg.output_file);

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
        if (g_state.samples_written > 0 &&
                (g_state.cfg.output_format == FORMAT_SDRUNO ||
                 g_state.cfg.output_format == FORMAT_SDRCONNECT))
            patch_wav_sizes(g_state.out_file, &g_state.cfg,
                            g_state.samples_written);
        CloseHandle(g_state.out_file);
        g_state.out_file = INVALID_HANDLE_VALUE;
        if (g_state.samples_written > 0)
            verify_recording(&g_state.cfg, g_state.samples_written);
    }

    /* ── Between-recording reset ─────────────────────────────────────────
     * If there are more schedule entries and no errors, stop the stream,
     * reset counters, reopen the output file, and restart the stream.   */
    int was_adhoc = g_state.adhoc_recording;
    g_state.adhoc_recording = 0;
    if (was_adhoc) {
        /* Ad-hoc recording done — don't advance sched_idx, loop back to
         * resume waiting for the original scheduled time.                */
        strncpy(g_state.cfg.output_file, DEFAULT_OUTPUT_FILE, MAX_PATH_LEN - 1);
        LOG_INFO("Ad-hoc recording complete. Resuming wait for %s.",
                 g_state.cfg.start_time);
    } else {
        sched_idx++;
    }
    if (g_running && !g_state.writer_error
            && (was_adhoc || sched_idx <= g_state.cfg.schedule_count)) {
        LOG_INFO("Preparing for schedule entry %d of %d ...",
                 sched_idx, g_state.cfg.schedule_count);

        /* Stop current stream */
        g_recording = 0;
        g_state.stream_running = 0;
        sdrplay_api_Uninit(g_state.device.dev);

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
            if (g_state.samples_written > 0 &&
                    (g_state.cfg.output_format == FORMAT_SDRUNO ||
                     g_state.cfg.output_format == FORMAT_SDRCONNECT))
                patch_wav_sizes(g_state.out_file, &g_state.cfg,
                                g_state.samples_written);
            CloseHandle(g_state.out_file);
            g_state.out_file = INVALID_HANDLE_VALUE;
            if (g_state.samples_written > 0)
                verify_recording(&g_state.cfg, g_state.samples_written);
        }

        /* Reset per-recording counters */
        g_state.samples_received    = 0;
        g_state.samples_written     = 0;
        g_state.zero_frames_written = 0;
        g_state.overflows           = 0;
        g_state.peak_dbfs           = -90.0f;
        g_state.peak_dbfs_b         = -90.0f;
        g_state.writer_error        = 0;
        g_state.disk_stop           = 0;
        g_state.frozen_file_mb      = -1;  /* cleared - new recording starting */
        g_state.frozen_elapsed_sec  = 0.0;
        g_state.session_complete    = 0;
        g_state.start_time.QuadPart = 0;   /* reset elapsed for HTTP monitor */

        /* Reset ring buffer for reuse - writer thread and sdrplay_api_Init
         * are restarted at the top of the next do-loop iteration.        */
        ring_reset(&g_state.ring);
    }

    } while (g_running && !g_state.writer_error
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
        g_state.zero_frames_written = 0;
        g_state.overflows           = 0;
        g_state.peak_dbfs           = -90.0f;
        g_state.peak_dbfs_b         = -90.0f;
        g_state.writer_error        = 0;
        g_state.disk_stop           = 0;
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

        goto hourly_next;
    }

    /* ------------------------------------------------------------------ */
    /* schedule_repeat: if enabled and no errors, reload schedule and      */
    /* restart from schedule_1 for the next day.                           */
    /* ------------------------------------------------------------------ */
    if (g_running && !g_state.writer_error && !g_state.disk_stop
            && g_state.cfg.schedule_repeat && g_state.cfg.schedule_only) {

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

        LOG_INFO("schedule_repeat=1: all entries complete, reloading schedule "
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
                    LOG_INFO("schedule_repeat: waiting until tomorrow for %s",
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
        g_state.zero_frames_written = 0;
        g_state.overflows           = 0;
        g_state.peak_dbfs           = -90.0f;
        g_state.peak_dbfs_b         = -90.0f;
        g_state.writer_error        = 0;
        g_state.disk_stop           = 0;
        g_state.disk_warn_issued    = 0;
        g_state.disk_free_mb        = 0;
        g_state.frozen_file_mb      = -1;
        g_state.frozen_elapsed_sec  = 0.0;
        g_state.session_complete    = 0;
        g_state.start_time.QuadPart = 0;
        /* next_start already set above - keep it so dashboard shows NEXT AT */
        ring_reset(&g_state.ring);

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
        if (g_state.samples_written > 0 &&
                (g_state.cfg.output_format == FORMAT_SDRUNO ||
                 g_state.cfg.output_format == FORMAT_SDRCONNECT))
            patch_wav_sizes(g_state.out_file, &g_state.cfg,
                            g_state.samples_written);
        CloseHandle(g_state.out_file);
        g_state.out_file = INVALID_HANDLE_VALUE;
        if (g_state.samples_written > 0)
            verify_recording(&g_state.cfg, g_state.samples_written);
    }

cleanup_ring:
    ring_free(&g_state.ring);

cleanup_device:
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

    /* Tell the UI the worker has finished so it can re-enable Start. */
    g_worker_active = 0;
    if (g_hwnd) PostMessageA(g_hwnd, WM_APP_DONE, (WPARAM)rc, 0);
    return (DWORD)rc;
}

/* =========================================================================
 * GUI: window procedure and WinMain
 * ========================================================================= */

static void gui_set_recording_ui(int recording)
{
    SetWindowTextA(g_hBtnToggle, recording ? "Stop" : "Start");
    EnableWindow(g_hBtnToggle, TRUE);
    EnableWindow(g_hBtnAgc,     recording);
    InvalidateRect(g_hBtnToggle, NULL, FALSE);
    /* Hide Start Now — it shows only during the scheduled wait. */
    if (g_hBtnNow) ShowWindow(g_hBtnNow, SW_HIDE);
}

static void gui_start_session(void)
{
    if (g_worker_active) return;

    /* Clear the log window for the new session. Schedule a one-shot scroll
     * to bottom 300 ms from now — by then the initial messages will have
     * been appended and there is content to scroll to.                    */
    if (g_hLog) SetWindowTextA(g_hLog, "");

    double keep_disk = g_ui.disk_free_mb;
    memset(&g_ui, 0, sizeof(g_ui));
    g_ui.disk_free_mb = keep_disk;
    g_ui.peak_a = -90.0f;
    g_ui.peak_b = -90.0f;
    strncpy(g_ui.state, "STARTING", sizeof(g_ui.state) - 1);

    g_worker_active = 1;
    gui_set_recording_ui(1);
    if (g_hwnd) InvalidateRect(g_hwnd, NULL, FALSE);

    g_worker_thread = CreateThread(NULL, 0, recording_worker, NULL, 0, NULL);
    if (!g_worker_thread) {
        g_worker_active = 0;
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
 * Returns nothing; draws inside the rect r.                                */
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
        float norm = (dbfs + 60.0f) / 60.0f;   /* -60 dB -> 0 .. 0 dB -> 1 */
        if (norm < 0.0f) norm = 0.0f;
        if (norm > 1.0f) norm = 1.0f;
        active = (int)(norm * segs + 0.5f);
    }
    if (active > segs) active = segs;

    int pad = 2;
    int innerW = (r.right - r.left) - pad * 2;
    int innerH = (r.bottom - r.top) - pad * 2;
    int gap = 2;
    int segW = (innerW - gap * (segs - 1)) / segs;
    int x = r.left + pad;
    int y = r.top + pad;

    for (int i = 0; i < segs; i++) {
        /* Segment colour: zone, graduated blend, or greyscale by style. */
        float t = (float)i / (float)(segs - 1);
        COLORREF c;
        if (graduated == 1) {
            if (t < 0.65f) {
                float s = t / 0.65f;
                c = RGB((int)(40  + (255-40 ) * s * 0.75f),
                        (int)(220 + (190-220) * s),
                        (int)(90  + ( 40- 90) * s));
            } else {
                float s = (t - 0.65f) / 0.35f;
                c = RGB(255,
                        (int)(190 + ( 60-190) * s),
                        (int)( 40 + ( 50- 40) * s));
            }
        } else if (graduated == 2) {
            /* Greyscale: dark grey at left, bright white-grey at right. */
            int v = (int)(60 + t * 180);
            c = RGB(v, v, v);
        } else {
            if (i >= (int)(segs * 0.85))      c = COL_SEG_RED;
            else if (i >= (int)(segs * 0.65)) c = COL_SEG_AMBER;
            else                               c = COL_SEG_GREEN;
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
     * that only lights when the hardware reports ADC overload. The remaining
     * pixels after the 20 segments (left over from integer division) form its
     * width naturally, keeping it flush with the right edge of the meter.   */
    {
        int ovld_x = x + gap;   /* just after the last segment + one gap    */
        int ovld_w = r.right - pad - ovld_x;
        if (ovld_w > 2) {
            COLORREF oc = overload ? COL_SEG_RED : RGB(60, 14, 12);
            HBRUSH ob2 = CreateSolidBrush(oc);
            HGDIOBJ os2 = SelectObject(dc, ob2);
            HGDIOBJ op2 = SelectObject(dc, GetStockObject(NULL_PEN));
            Rectangle(dc, ovld_x, y, ovld_x + ovld_w, y + innerH);
            SelectObject(dc, os2);
            SelectObject(dc, op2);
            DeleteObject(ob2);
        }
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
        draw_text_base(dc, x + 12, baseline, "RSP IQ Recorder  v" VERSION,
                       COL_TEXT_DIM, g_hFontUI);

        /* Recording LED + state. The LED sits at a FIXED position and the
         * state word is drawn left-justified to its right, so the LED does
         * not move when the word changes length (RECORDING -> FINISHED etc). */
        COLORREF lc = s.recording ? COL_LED_ON
                    : s.finished  ? RGB(10, 245, 25)
                    :               COL_LED_OFF;
        const char *st = s.state[0] ? s.state : "IDLE";
        COLORREF stc = s.recording ? COL_LED_ON
                      : s.finished ? RGB(10, 245, 25) : COL_TEXT_DIM;
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
            char sb[80];
            snprintf(sb, sizeof(sb), "Coverage: %s", s.span);
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

        /* Next scheduled start, placed to the left of the clock. */
        if (s.next[0]) {
            char nb[64];
            snprintf(nb, sizeof(nb), "Next: %s", s.next);
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

        /* Channel A */
        draw_text(dc, 22, panelTop + 30, "A", COL_TEXT, g_hFontVal);
        RECT ma = { 44, panelTop + 30, cr.right - 120, panelTop + 48 };
        draw_meter(dc, ma, s.recording ? s.peak_a : -90.0f, s.overload_a, g_meter_style);
        {
            char db[24];
            if (s.peak_a <= -90.0f || !s.recording)
                snprintf(db, sizeof(db), "  --- dBFS");
            else
                snprintf(db, sizeof(db), "%+5.1f dBFS", s.peak_a);
            draw_text(dc, cr.right - 110, panelTop + 30, db,
                      s.overload_a ? COL_SEG_RED : COL_TEXT, g_hFontVal);
        }

        /* Channel B (only meaningful in dual mode, shown greyed otherwise) */
        draw_text(dc, 22, panelTop + 56, "B", COL_TEXT, g_hFontVal);
        RECT mb = { 44, panelTop + 56, cr.right - 120, panelTop + 74 };
        draw_meter(dc, mb, (s.dual && s.recording) ? s.peak_b : -90.0f,
                   s.overload_b, g_meter_style);
        {
            char db[24];
            if (!s.dual)
                snprintf(db, sizeof(db), "  (single)");
            else if (s.peak_b <= -90.0f || !s.recording)
                snprintf(db, sizeof(db), "  --- dBFS");
            else
                snprintf(db, sizeof(db), "%+5.1f dBFS", s.peak_b);
            draw_text(dc, cr.right - 110, panelTop + 56, db,
                      s.overload_b ? COL_SEG_RED : COL_TEXT, g_hFontVal);
        }
    }

    /* ---- Counter tiles row ---- */
    int ctrTop = panelTop + 96;
    {
        int gap = 10;
        int tiles = 5;
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
    }

    /* ---- Bottom info line (scheduling), left of the buttons ---- */
    if (s.sched[0]) {
        draw_text_base(dc, 14, cr.bottom - 17, s.sched,
                       COL_TEXT_DIM, g_hFontUI);
    }

    BitBlt(wdc, 0, 0, cr.right, cr.bottom, dc, 0, 0, SRCCOPY);

    SelectObject(dc, obmp);
    DeleteObject(bmp);
    DeleteDC(dc);
    EndPaint(hwnd, &ps);
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

    /* Bottom button bar: Start/Stop and AGC sit along the bottom edge. */
    int bbh = 26;                       /* button height                    */
    int bbY = cr.bottom - bbh - 10;     /* button row y                     */
    int sbw = 90, abw = 64, nw = 80, bgap = 8;
    /* Buttons right-aligned; scheduling text painted to their left. */
    int agc_x = cr.right - 12 - abw;
    int tog_x = agc_x - bgap - sbw;
    int now_x = tog_x - bgap - nw;
    MoveWindow(g_hBtnToggle, tog_x, bbY, sbw, bbh, TRUE);
    MoveWindow(g_hBtnAgc,    agc_x, bbY, abw, bbh, TRUE);
    if (g_hBtnNow) MoveWindow(g_hBtnNow, now_x, bbY, nw, bbh, TRUE);

    /* Log fills the area between the disk line and the bottom button bar. */
    int logTop = diskY + 26 + 6;
    int logBottom = bbY - 8;
    MoveWindow(g_hLog, 12, logTop, cr.right - 24, logBottom - logTop, TRUE);
}

static LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp)
{
    switch (msg) {

    case WM_CTLCOLOREDIT:
    case WM_CTLCOLORSTATIC: {
        HDC dc = (HDC)wp;
        SetTextColor(dc, COL_TEXT);
        SetBkColor(dc, COL_BAR_BG);
        return (LRESULT)g_hbrPanel;
    }

    case WM_DRAWITEM: {
        LPDRAWITEMSTRUCT di = (LPDRAWITEMSTRUCT)lp;
        if (di->CtlType == ODT_BUTTON) {
            int dis = (di->itemState & ODS_DISABLED) != 0;
            int down = (di->itemState & ODS_SELECTED) != 0;
            COLORREF face;
            if (di->CtlID == IDC_BTN_TOGGLE) {
                /* Green = will start, Red = will stop. */
                COLORREF base = g_worker_active ? COL_BTN_STOP : COL_BTN_START;
                face = down ? RGB(GetRValue(base)+25, GetGValue(base)+25,
                                  GetBValue(base)+25) : base;
            } else if (di->CtlID == IDC_BTN_AGC && !dis && g_ui.agc_on) {
                /* Green when AGC is currently enabled. */
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
        if (wp == ID_TIMER_CLOCK && g_clock_show) {
            /* Tick the clock once a second. Repaint only the top strip where
             * the clock lives (cheap; the monitor handles the rest during a
             * recording, and this keeps the clock smooth at idle too).      */
            RECT cr; GetClientRect(hwnd, &cr);
            RECT top = { 0, 0, cr.right, 74 };
            InvalidateRect(hwnd, &top, FALSE);
        }
        return 0;

    case WM_COMMAND:
        switch (LOWORD(wp)) {
        case IDC_BTN_TOGGLE:
            if (g_worker_active) {
                gui_stop_session(0);
            } else {
                gui_start_session();
            }
            InvalidateRect(hwnd, NULL, FALSE);
            return 0;
        case IDC_BTN_NOW:
            /* Override the scheduled wait — start recording immediately. */
            g_record_now = 1;
            if (g_hBtnNow) ShowWindow(g_hBtnNow, SW_HIDE);
            return 0;
        case IDC_BTN_AGC: {
            /* Debounce: ignore presses within 1.5 s of the last one so the
             * API has time to settle. Rapid toggles caused NotInitialised
             * errors because each Update() raced the previous one.          */
            static DWORD last_agc_click = 0;
            DWORD now = GetTickCount();
            if (now - last_agc_click >= 1500) {
                last_agc_click = now;
                g_agc_toggle_req = 1;
            }
            return 0;
        }
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
            default: col = RGB(225, 235, 250); break;  /* white  */
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
            SendMessageA(g_hLog, EM_SCROLL, SB_BOTTOM, 0);
            SendMessageA(g_hLog, EM_SCROLLCARET, 0, 0);
            free(txt);
        }
        return 0;
    }

    case WM_APP_DONE:
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
        /* Leave the final FINISHED snapshot on screen; do NOT auto-close.
         * Pull the final elapsed time and file size from the frozen values
         * so the display keeps the last recording's length instead of
         * resetting to 00:00 once the monitor thread has stopped.          */
        g_ui.recording = 0;
        if (g_state.last_display_elapsed > 0.0)
            g_ui.elapsed_sec = g_state.last_display_elapsed;
        if (g_state.last_display_file_mb >= 0)
            g_ui.file_mb = (double)g_state.last_display_file_mb;
        if (g_state.session_complete) {
            g_ui.finished = 1;
            strncpy(g_ui.state, "FINISHED", sizeof(g_ui.state) - 1);
        } else {
            strncpy(g_ui.state, "STOPPED", sizeof(g_ui.state) - 1);
        }
        gui_set_recording_ui(0);
        InvalidateRect(hwnd, NULL, FALSE);
        return 0;

    case WM_SIZE:
        layout_children(hwnd);
        InvalidateRect(hwnd, NULL, FALSE);
        return 0;

    case WM_GETMINMAXINFO: {
        MINMAXINFO *mm = (MINMAXINFO *)lp;
        mm->ptMinTrackSize.x = 640;
        mm->ptMinTrackSize.y = 480;
        return 0;
    }

    case WM_CLOSE:
        if (g_worker_active) {
            if (MessageBoxA(hwnd,
                    "A recording is in progress. Stop and exit?",
                    "DuoDX", MB_YESNO | MB_ICONQUESTION) != IDYES)
                return 0;
            gui_stop_session(1);
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

int WINAPI WinMain(HINSTANCE hInst, HINSTANCE hPrev, LPSTR lpCmdLine, int nShow)
{
    (void)hPrev; (void)lpCmdLine;

    InitCommonControls();
    LoadLibraryA("Msftedit.dll");   /* registers RICHEDIT50W window class */

    g_hbrBg    = CreateSolidBrush(COL_BG);
    g_hbrPanel = CreateSolidBrush(COL_BAR_BG);

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

    g_hwnd = CreateWindowExA(0, "DuoDXWindow",
                "DuoDX  v" VERSION "  -  RSP IQ Recorder",
                WS_OVERLAPPEDWINDOW,
                CW_USEDEFAULT, CW_USEDEFAULT, 820, 660,
                NULL, NULL, hInst, NULL);
    if (!g_hwnd) {
        MessageBoxA(NULL, "CreateWindow failed", "DuoDX", MB_ICONERROR);
        return 1;
    }

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
    g_hFontLog = CreateFontA(-13, 0, 0, 0, FW_NORMAL, 0, 0, 0,
                    DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                    DEFAULT_QUALITY, FIXED_PITCH | FF_MODERN, "Consolas");

    /* Buttons (owner-drawn, positioned by layout_children) */
    g_hBtnToggle = mk_button(g_hwnd, IDC_BTN_TOGGLE, "Start");
    g_hBtnAgc    = mk_button(g_hwnd, IDC_BTN_AGC,    "AGC");
    g_hBtnNow    = mk_button(g_hwnd, IDC_BTN_NOW,    "Start Now");
    if (g_hBtnNow) ShowWindow(g_hBtnNow, SW_HIDE);

    /* Log control - RichEdit (per-line colour). Requires Msftedit.dll. */
    g_hLog = CreateWindowExA(WS_EX_CLIENTEDGE, "RICHEDIT50W", "",
                WS_CHILD | WS_VISIBLE | WS_VSCROLL | ES_MULTILINE |
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

    ShowWindow(g_hwnd, nShow);
    UpdateWindow(g_hwnd);

    LOG_INFO("DuoDX GUI ready. Press Start to begin recording using duodx.ini.");

    /* 1-second timer to tick the live clock while idle. */
    SetTimer(g_hwnd, ID_TIMER_CLOCK, 1000, NULL);

    MSG m;
    while (GetMessageA(&m, NULL, 0, 0) > 0) {
        TranslateMessage(&m);
        DispatchMessageA(&m);
    }

    /* Ensure worker is stopped on exit. */
    gui_stop_session(1);

    if (g_hFontUI)  DeleteObject(g_hFontUI);
    if (g_hFontVal) DeleteObject(g_hFontVal);
    if (g_hFontBig) DeleteObject(g_hFontBig);
    if (g_hFontLog) DeleteObject(g_hFontLog);
    if (g_hbrBg)    DeleteObject(g_hbrBg);
    if (g_hbrPanel) DeleteObject(g_hbrPanel);
    return (int)m.wParam;
}
