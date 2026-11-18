// MCU link task — see mcu_link.h. Owns UART0 @ 9600, heartbeats the MCU, decodes status
// into a mutex-protected snapshot, applies queued control commands, rings raw RX bytes,
// keeps a TX/RX timeline, and recovers the link on its own when it goes bad.
//
// TX pacing: every frame is followed by a 300 ms gap, so the MCU never has two replies in
// flight. We used to send hello + panel-light + time + command frames back-to-back; on
// 2026-11-17 the MCU was found answering every request with its reply shifted 10 bytes
// (5B B5 header never sent, frame tail repeated in its place) until power-cycled. Every
// frame now goes through one paced slot — except the deliberate pair below.
//
// Recovery ladder — runs once the link has not been UP for 5 s, then backs off 30 s → 5 min:
//   uart_reset — reinstall our UART driver + flush the parser (rules out the ESP side)
//   quiet      — send nothing for 3 s so the MCU can drain, then one hello
//   pair_poll  — two hellos back-to-back. With the shifted-reply fault the second reply is
//                queued behind the first, so its header goes out and it arrives intact
//                (seen in the 2026-11-17 capture). If that works we stay in pair mode
//                (degraded but live) and retry single polls every 5 min.
// "break" (a ~27 ms UART break) is manual-only: its effect on this MCU is untested.
#include "mcu_link.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include <time.h>

#define MCU_UART     UART_NUM_0
#define MCU_TX_GPIO  20          // ESP U0TXD -> MCU RX  (confirmed default)
#define MCU_RX_GPIO  19          // ESP U0RXD <- MCU TX
#define MCU_BAUD     9600
#define RX_BUF       512
#define SNIFF_CAP    1024

#define BYTE_US         1042       // 10 bits @ 9600
#define TX_GAP_US       300000     // gap after every frame: one MCU reply in flight
#define BOOT_QUIET_US   1000000    // let the MCU shrug off our ROM/bootloader log first
#define STALE_US        5000000    // no valid frame for this long = link not UP
#define BURST_GAP_US    25000      // RX idle this long closes a burst (read tick = 10 ms)
#define WATCH_US        2000000    // how long a recovery step waits for a valid frame
#define QUIET_US        3000000
#define BREAK_HOLD_US   500000
#define BACKOFF_MIN_US  30000000LL
#define BACKOFF_MAX_US  300000000LL
#define PAIR_EVERY_US   2000000
#define PAIR_RETEST_US  300000000LL

static fc_state_t       s_state;
static bool             s_valid;
static SemaphoreHandle_t s_lock;
static QueueHandle_t     s_cmdq, s_rawq;

static uint8_t          s_sniff[SNIFF_CAP];
static int              s_sniff_len;
static SemaphoreHandle_t s_sniff_lock;     // guards s_sniff and s_trace

// --- link health instrumentation (guarded by s_lock) ---
static fc_parser_t      s_parser;
static uint32_t         s_rx_bytes, s_rx_frames, s_tx_frames, s_rx_bursts, s_headerless;
static int64_t          s_start_us, s_last_rx_us, s_last_frame_us;
static volatile bool    s_ping_req;   // set by mcu_probe(): poke the MCU with a hello now
static volatile bool    s_connected;  // WiFi up → periodically send time-sync (0x11) to MCU
static volatile int     s_light = -1;  // MCU panel-light state (payload[0] of 0x0F); <0 = don't send

// --- TX scheduling (task-only) ---
static int64_t          s_next_tx_us;  // earliest time the next paced frame may go out
static mcu_cmd_t        s_pending = { MCU_NOCHANGE, MCU_NOCHANGE, MCU_NOCHANGE, MCU_NOCHANGE, MCU_NOCHANGE };
typedef struct { uint8_t len; uint8_t f[64]; } raw_frame_t;

// --- recovery (written by the task under s_lock; s_rec_req is the HTTP → task mailbox) ---
enum { STEP_UART_RESET, STEP_QUIET, STEP_PAIR, STEP_BREAK, STEP_SINGLE, STEP_N };
static const char *const STEP_NAME[STEP_N] = { "uart_reset", "quiet", "pair_poll", "break", "single_retest" };
static const int LADDER[] = { STEP_UART_RESET, STEP_QUIET, STEP_PAIR };
#define LADDER_LEN ((int)(sizeof LADDER / sizeof LADDER[0]))
#define REQ_NONE  (-1)
#define REQ_AUTO  (STEP_N)
static volatile int     s_rec_req = REQ_NONE;
static int              s_rec_step = -1;          // step in progress, -1 = none
static bool             s_rec_watching;           // false = quiet hold (no TX), true = waiting for a frame
static int64_t          s_rec_until;
static bool             s_rec_ladder;             // part of the auto ladder (advance on failure)
static bool             s_rec_was_down;           // link was not UP when the step started
static int              s_ladder_pos;
static int64_t          s_rec_next_us;            // next auto ladder run; 0 = as soon as the link drops
static int64_t          s_backoff_us = BACKOFF_MIN_US;
static uint32_t         s_frames_at_step;
static uint32_t         s_rec_attempts, s_recoveries;
static const char      *s_rec_last_ok;
static bool             s_pair_mode;
static int64_t          s_pair_retest_us;

// --- TX/RX timeline (guarded by s_sniff_lock) ---
#define TRACE_N   32
#define TRACE_CAP 96
typedef struct {
    uint32_t t_ms;
    uint16_t len;          // true length (bytes beyond TRACE_CAP are not stored)
    char     dir;          // 'T' frame sent, 'R' RX burst, '#' note (b = text)
    uint8_t  frames;       // RX: valid frames decoded inside this burst
    int16_t  lead;         // RX: offset of the first valid frame in the burst
    uint8_t  b[TRACE_CAP];
} trace_t;
static trace_t s_trace[TRACE_N];
static int     s_trace_head, s_trace_count;
static trace_t s_burst;                    // RX burst being assembled (task-only)
static bool    s_burst_open;
static uint32_t s_burst_frames0;

const char *mcu_link_state_name(mcu_link_state_t s) {
    switch (s) {
    case MCU_LINK_UP:     return "up";
    case MCU_LINK_DESYNC: return "desync";
    case MCU_LINK_SILENT: return "silent";
    default:              return "boot";
    }
}

static void trace_push(const trace_t *e) {
    xSemaphoreTake(s_sniff_lock, portMAX_DELAY);
    s_trace[s_trace_head] = *e;
    s_trace_head = (s_trace_head + 1) % TRACE_N;
    if (s_trace_count < TRACE_N) s_trace_count++;
    xSemaphoreGive(s_sniff_lock);
}

static void trace_note(int64_t now, const char *fmt, ...) {
    trace_t e = { .t_ms = (uint32_t)(now / 1000), .dir = '#' };
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf((char *)e.b, sizeof e.b, fmt, ap);
    va_end(ap);
    e.len = (uint16_t)(n < (int)sizeof e.b ? n : (int)sizeof e.b - 1);
    trace_push(&e);
}

static void sniff_push(const uint8_t *b, int n) {
    xSemaphoreTake(s_sniff_lock, portMAX_DELAY);
    for (int i = 0; i < n; i++) {
        if (s_sniff_len < SNIFF_CAP) s_sniff[s_sniff_len++] = b[i];
        else { memmove(s_sniff, s_sniff + 1, SNIFF_CAP - 1); s_sniff[SNIFF_CAP - 1] = b[i]; }
    }
    xSemaphoreGive(s_sniff_lock);
}

int mcu_sniffer_dump(char *out, int max) {
    xSemaphoreTake(s_sniff_lock, portMAX_DELAY);
    int n = 0;
    for (int i = 0; i < s_sniff_len && n + 4 < max; i++)
        n += snprintf(out + n, max - n, "%02x ", s_sniff[i]);
    xSemaphoreGive(s_sniff_lock);
    return n;
}

// snprintf-append that never runs past `max` (truncates instead).
static int app(char *out, int n, int max, const char *fmt, ...) {
    if (n >= max - 1) return n;
    va_list ap; va_start(ap, fmt);
    int w = vsnprintf(out + n, (size_t)(max - n), fmt, ap);
    va_end(ap);
    return w < 0 ? n : (n + w < max - 1 ? n + w : max - 1);
}

int mcu_trace_dump(char *out, int max) {
    static trace_t snap[TRACE_N];               // only the HTTP task calls this
    xSemaphoreTake(s_sniff_lock, portMAX_DELAY);
    int cnt = s_trace_count, first = (s_trace_head - s_trace_count + TRACE_N) % TRACE_N;
    for (int i = 0; i < cnt; i++) snap[i] = s_trace[(first + i) % TRACE_N];
    xSemaphoreGive(s_sniff_lock);

    int n = 0;
    for (int i = 0; i < cnt; i++) {
        const trace_t *e = &snap[i];
        if (e->dir == '#') {
            n = app(out, n, max, "%9lu  --  %s\n", (unsigned long)e->t_ms, (const char *)e->b);
            continue;
        }
        n = app(out, n, max, "%9lu  %s %3u ", (unsigned long)e->t_ms, e->dir == 'T' ? "TX" : "RX", e->len);
        if (e->dir == 'R')
            n = e->frames ? app(out, n, max, "[%u ok @%d] ", e->frames, e->lead)
                          : app(out, n, max, "[NO FRAME] ");
        int show = e->len < TRACE_CAP ? e->len : TRACE_CAP;
        for (int k = 0; k < show; k++) n = app(out, n, max, "%02x ", e->b[k]);
        n = app(out, n, max, e->len > TRACE_CAP ? "...\n" : "\n");
    }
    return n;
}

bool mcu_get_state(fc_state_t *out) {
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool v = s_valid;
    if (v) *out = s_state;
    xSemaphoreGive(s_lock);
    return v;
}

// Call with s_lock held.
static mcu_link_state_t link_state(int64_t now) {
    if (s_last_frame_us && now - s_last_frame_us < STALE_US) return MCU_LINK_UP;
    if (!s_last_frame_us && now - s_start_us < BOOT_QUIET_US + STALE_US) return MCU_LINK_BOOT;
    if (s_last_rx_us && now - s_last_rx_us < STALE_US) return MCU_LINK_DESYNC;
    return MCU_LINK_SILENT;
}

void mcu_get_health(mcu_health_t *out) {
    int64_t now = esp_timer_get_time();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    out->rx_bytes     = s_rx_bytes;
    out->rx_frames    = s_rx_frames;
    out->tx_frames    = s_tx_frames;
    out->status_valid = s_valid;
    out->link_up      = s_rx_bytes > 0;
    out->state        = link_state(now);
    out->pair_mode    = s_pair_mode;
    out->last_rx_ms    = s_last_rx_us    ? (int32_t)((now - s_last_rx_us)    / 1000) : -1;
    out->last_frame_ms = s_last_frame_us ? (int32_t)((now - s_last_frame_us) / 1000) : -1;
    out->ck_frames    = s_parser.frames;
    out->bad_ck       = s_parser.bad_ck;
    out->bad_len      = s_parser.bad_len;
    out->skipped      = s_parser.skipped;
    out->rx_bursts    = s_rx_bursts;
    out->headerless   = s_headerless;
    out->rec_attempts = s_rec_attempts;
    out->recoveries   = s_recoveries;
    out->rec_active   = s_rec_step >= 0 ? STEP_NAME[s_rec_step] : NULL;
    out->rec_last_ok  = s_rec_last_ok;
    out->rec_next_ms  = (s_rec_step < 0 && s_rec_next_us > now) ? (int32_t)((s_rec_next_us - now) / 1000) : -1;
    xSemaphoreGive(s_lock);
}

bool mcu_probe(int timeout_ms, int *rtt_ms) {
    xSemaphoreTake(s_lock, portMAX_DELAY);
    uint32_t b0 = s_rx_bytes, f0 = s_rx_frames;
    xSemaphoreGive(s_lock);

    int64_t t0 = esp_timer_get_time();
    s_ping_req = true;                                   // the task sends a hello next free slot
    while ((esp_timer_get_time() - t0) / 1000 < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(10));
        xSemaphoreTake(s_lock, portMAX_DELAY);
        uint32_t b = s_rx_bytes, f = s_rx_frames;
        xSemaphoreGive(s_lock);
        if (b != b0 || f != f0) {                        // any fresh byte/frame = MCU alive
            if (rtt_ms) *rtt_ms = (int)((esp_timer_get_time() - t0) / 1000);
            return true;
        }
    }
    return false;
}

void mcu_send_cmd(const mcu_cmd_t *cmd) {
    if (s_cmdq) xQueueSend(s_cmdq, cmd, 0);
}

void mcu_set_connected(bool c) { s_connected = c; }
void mcu_set_light(int s)      { s_light = s; }

bool mcu_recover(const char *step) {
    if (!strcmp(step, "auto")) { s_rec_req = REQ_AUTO; return true; }
    for (int i = 0; i < STEP_N; i++)
        if (i != STEP_SINGLE && !strcmp(step, STEP_NAME[i])) { s_rec_req = i; return true; }
    return false;
}

bool mcu_send_raw(uint8_t type, const uint8_t *payload, uint16_t len) {
    if (type == FC_T_UPGDATA) return false;                         // MCU firmware upload
    if (type == FC_T_UPGCTRL &&                                     // only the light values
        !(len == 2 && (payload[0] == 1 || payload[0] == 2) && payload[1] == 0)) return false;
    if (len > 56 || !s_rawq) return false;
    raw_frame_t r;
    r.len = (uint8_t)fc_encode(type, payload, len, 0, r.f);
    return xQueueSend(s_rawq, &r, 0) == pdTRUE;
}

// --- TX ---
static void tx_raw(const uint8_t *buf, size_t len) {
    if (!len) return;
    uart_write_bytes(MCU_UART, (const char *)buf, len);   // no TX ring: returns once in the FIFO
    int64_t now = esp_timer_get_time();
    s_next_tx_us = now + (int64_t)len * BYTE_US + TX_GAP_US;
    trace_t e = { .t_ms = (uint32_t)(now / 1000), .dir = 'T', .len = (uint16_t)len };
    memcpy(e.b, buf, len < TRACE_CAP ? len : TRACE_CAP);
    trace_push(&e);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_tx_frames++;
    xSemaphoreGive(s_lock);
}

static void tx_hello_pair(uint8_t *o) {
    size_t n = fc_hello(o);
    tx_raw(o, n);
    tx_raw(o, n);              // straight behind the first, deliberately unpaced
}

static void merge_cmd(const mcu_cmd_t *c) {
    if (c->setpoint != MCU_NOCHANGE) s_pending.setpoint = c->setpoint;
    if (c->mode     != MCU_NOCHANGE) s_pending.mode     = c->mode;
    if (c->fan      != MCU_NOCHANGE) s_pending.fan      = c->fan;
    if (c->lock     != MCU_NOCHANGE) s_pending.lock     = c->lock;
    if (c->power    != MCU_NOCHANGE) s_pending.power    = c->power;
}

// Send ONE pending command field (one 0x12 frame per paced slot). Returns false if none left.
static bool send_pending(uint8_t *o) {
    mcu_cmd_t *c = &s_pending;
    if (c->setpoint != MCU_NOCHANGE) { tx_raw(o, fc_set_temp(o, (uint8_t)c->setpoint)); c->setpoint = MCU_NOCHANGE; return true; }
    if (c->mode     != MCU_NOCHANGE) { tx_raw(o, fc_set_mode(o, (uint8_t)c->mode, 0));  c->mode     = MCU_NOCHANGE; return true; }
    if (c->fan      != MCU_NOCHANGE) { tx_raw(o, fc_set_fan(o, (uint8_t)c->fan));       c->fan      = MCU_NOCHANGE; return true; }
    if (c->lock     != MCU_NOCHANGE) { tx_raw(o, fc_set_keylock(o, c->lock != 0));      c->lock     = MCU_NOCHANGE; return true; }
    if (c->power    != MCU_NOCHANGE) { tx_raw(o, fc_set_power(o, c->power != 0));       c->power    = MCU_NOCHANGE; return true; }
    return false;
}

// --- UART (re)initialisation ---
static void uart_setup(void) {
    uart_config_t cfg = {
        .baud_rate  = MCU_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(MCU_UART, RX_BUF, 0, 0, NULL, 0);
    uart_param_config(MCU_UART, &cfg);
    uart_set_pin(MCU_UART, MCU_TX_GPIO, MCU_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void rx_resync(void) {           // forget any half-received bytes, ours and the driver's
    uart_flush_input(MCU_UART);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    fc_parser_flush(&s_parser);
    xSemaphoreGive(s_lock);
}

// --- recovery state machine (task-only) ---
static void set_rec_next(int64_t t) {           // int64 isn't atomic on RV32: write under lock
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_rec_next_us = t;
    xSemaphoreGive(s_lock);
}

static void step_start(int step, bool ladder, int64_t now, uint8_t *o) {
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_rec_step = step;
    s_rec_attempts++;
    s_frames_at_step = s_rx_frames;
    s_rec_was_down = link_state(now) != MCU_LINK_UP;
    xSemaphoreGive(s_lock);
    s_rec_ladder = ladder;
    trace_note(now, "recovery step: %s%s", STEP_NAME[step], ladder ? " (auto)" : "");
    switch (step) {
    case STEP_UART_RESET:
        uart_driver_delete(MCU_UART);
        uart_setup();
        rx_resync();
        tx_raw(o, fc_hello(o));
        s_rec_watching = true;  s_rec_until = now + WATCH_US;
        break;
    case STEP_QUIET:
        s_rec_watching = false; s_rec_until = now + QUIET_US;
        break;
    case STEP_BREAK: {
        uint8_t z = 0;
        uart_write_bytes_with_break(MCU_UART, &z, 1, 255);      // 1 byte + ~27 ms of low
        s_rec_watching = false; s_rec_until = now + BREAK_HOLD_US;
        break;
    }
    case STEP_PAIR:
        tx_hello_pair(o);
        s_rec_watching = true;  s_rec_until = now + WATCH_US;
        break;
    case STEP_SINGLE:
        tx_raw(o, fc_hello(o));
        s_rec_watching = true;  s_rec_until = now + WATCH_US;
        break;
    }
}

static void step_end(bool ok, int64_t now, uint8_t *o) {
    int step = s_rec_step;
    // Pair mode only when single polls were failing — a manual pair_poll on a healthy
    // link also gets a frame, and that must not demote the link.
    bool to_pair = ok && step == STEP_PAIR && s_rec_was_down;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_rec_step = -1;
    if (ok && s_rec_was_down && step != STEP_SINGLE) { s_recoveries++; s_rec_last_ok = STEP_NAME[step]; }
    if (to_pair)                     s_pair_mode = true;
    if (ok && step == STEP_SINGLE)   s_pair_mode = false;
    xSemaphoreGive(s_lock);

    if (ok) {
        if (step == STEP_SINGLE) trace_note(now, "single polls answer again: leaving pair mode");
        else if (to_pair)        trace_note(now, "pair_poll got a frame: staying in pair mode (degraded)");
        else                     trace_note(now, "%s: got a frame", STEP_NAME[step]);
        if (to_pair) s_pair_retest_us = now + PAIR_RETEST_US;
        s_backoff_us = BACKOFF_MIN_US;
        set_rec_next(0);
        s_ladder_pos = 0;
        return;
    }
    if (step == STEP_SINGLE) {                     // still shifted: keep pairing, retry later
        trace_note(now, "single poll still unanswered: staying in pair mode");
        s_pair_retest_us = now + PAIR_RETEST_US;
    } else if (s_rec_ladder && ++s_ladder_pos < LADDER_LEN) {
        step_start(LADDER[s_ladder_pos], true, now, o);
    } else if (s_rec_ladder) {
        trace_note(now, "recovery ladder failed; next try in %d s", (int)(s_backoff_us / 1000000));
        s_ladder_pos = 0;
        set_rec_next(now + s_backoff_us);
        s_backoff_us = s_backoff_us * 2 > BACKOFF_MAX_US ? BACKOFF_MAX_US : s_backoff_us * 2;
    } else {
        trace_note(now, "%s: no frame", STEP_NAME[step]);
    }
}

static void recovery_tick(int64_t now, mcu_link_state_t st, uint8_t *o) {
    if (s_rec_step >= 0) {
        if (!s_rec_watching) {
            if (now >= s_rec_until) {              // quiet/break hold over: clean slate + one hello
                rx_resync();
                tx_raw(o, fc_hello(o));
                s_rec_watching = true;
                s_rec_until = now + WATCH_US;
            }
            return;
        }
        xSemaphoreTake(s_lock, portMAX_DELAY);
        bool got = s_rx_frames != s_frames_at_step;
        xSemaphoreGive(s_lock);
        if (got) step_end(true, now, o);
        else if (now >= s_rec_until) step_end(false, now, o);
        return;
    }

    int req = s_rec_req;
    if (req != REQ_NONE) {
        s_rec_req = REQ_NONE;
        if (req == REQ_AUTO) {
            xSemaphoreTake(s_lock, portMAX_DELAY); s_pair_mode = false; xSemaphoreGive(s_lock);
            s_ladder_pos = 0; s_backoff_us = BACKOFF_MIN_US;
            step_start(LADDER[0], true, now, o);
        } else {
            step_start(req, false, now, o);
        }
        return;
    }

    if (st == MCU_LINK_DESYNC || st == MCU_LINK_SILENT) {
        if (now >= s_rec_next_us) {
            xSemaphoreTake(s_lock, portMAX_DELAY); s_pair_mode = false; xSemaphoreGive(s_lock);
            trace_note(now, "link %s: starting recovery ladder", mcu_link_state_name(st));
            s_ladder_pos = 0;
            step_start(LADDER[0], true, now, o);
        }
    } else if (s_pair_mode && now >= s_pair_retest_us && now >= s_next_tx_us) {
        step_start(STEP_SINGLE, false, now, o);
    }
}

// --- RX ---
static void burst_close(void) {
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_burst.frames = (uint8_t)(s_parser.frames - s_burst_frames0);
    s_rx_bursts++;
    if (!s_burst.frames && s_burst.len >= 8) s_headerless++;
    xSemaphoreGive(s_lock);
    trace_push(&s_burst);
    s_burst_open = false;
}

static void rx_feed(const uint8_t *rx, int n, int64_t now) {
    sniff_push(rx, n);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (!s_burst_open) {
        memset(&s_burst, 0, sizeof s_burst);
        s_burst.t_ms = (uint32_t)(now / 1000);
        s_burst.dir = 'R';
        s_burst.lead = -1;
        s_burst_frames0 = s_parser.frames;
        s_burst_open = true;
    }
    int base = s_burst.len;
    for (int i = 0; i < n; i++) {
        if (base + i < TRACE_CAP) s_burst.b[base + i] = rx[i];
        uint32_t before = s_parser.frames;
        fc_state_t st;
        if (fc_parse_byte(&s_parser, rx[i], &st)) {
            s_state = st; s_valid = true;
            s_rx_frames++; s_last_frame_us = now;
        }
        if (s_parser.frames != before && s_burst.frames == 0 && s_burst.lead < 0) {
            s_burst.lead = (int16_t)(base + i + 1 - s_parser.last_len);
            s_burst.frames = 1;   // marks the lead as set; recomputed on close
        }
    }
    s_burst.len = (uint16_t)(base + n);
    s_rx_bytes += (uint32_t)n; s_last_rx_us = now;
    xSemaphoreGive(s_lock);
}

static void mcu_task(void *arg) {
    uint8_t o[64], rx[128];
    int64_t last_hello = 0, last_time = 0, last_net = 0;
    esp_task_wdt_add(NULL);                  // watchdog: reset the chip if this task ever hangs
    for (;;) {
        esp_task_wdt_reset();
        mcu_cmd_t c;
        while (xQueueReceive(s_cmdq, &c, 0)) merge_cmd(&c);

        int n = uart_read_bytes(MCU_UART, rx, sizeof rx, pdMS_TO_TICKS(10));
        int64_t now = esp_timer_get_time();
        if (n > 0) rx_feed(rx, n, now);
        else if (s_burst_open && now - s_last_rx_us > BURST_GAP_US) burst_close();

        xSemaphoreTake(s_lock, portMAX_DELAY);
        mcu_link_state_t st = link_state(now);
        bool pair = s_pair_mode;
        xSemaphoreGive(s_lock);

        recovery_tick(now, st, o);
        if (s_rec_step >= 0 || now < s_next_tx_us) continue;    // recovery owns the line / pacing

        // One frame per paced slot, highest priority first.
        raw_frame_t r;
        if (send_pending(o)) continue;
        if (xQueueReceive(s_rawq, &r, 0)) { tx_raw(r.f, r.len); continue; }
        if (s_ping_req) {                                                // probe poke
            s_ping_req = false;
            if (pair) tx_hello_pair(o); else tx_raw(o, fc_hello(o));
            continue;
        }
        if (now - last_hello > (pair ? PAIR_EVERY_US : 1000000)) {      // heartbeat (~1 Hz)
            if (pair) tx_hello_pair(o); else tx_raw(o, fc_hello(o));
            last_hello = now;
            continue;
        }
        if ((s_light == 1 || s_light == 2) && now - last_net > 3000000) {  // MCU panel light: 2=solid, 1=flash.
            uint8_t p[2] = { (uint8_t)s_light, 0 };                        // HARD CLAMP to 1/2 — every other
            tx_raw(o, fc_encode(0x0F, p, 2, 0, o));                        // 0x0F payload is MCU upgrade-control.
            last_net = now;
            continue;
        }
        if (s_connected && now - last_time > 10000000) {    // ~10s: time-sync keeps the panel clock right
            time_t t = time(NULL);
            struct tm tmv; localtime_r(&t, &tmv);
            uint8_t wd = tmv.tm_wday == 0 ? 7 : (uint8_t)tmv.tm_wday;               // Sun=7, Mon..Sat=1..6
            tx_raw(o, fc_time(o, wd, (uint8_t)tmv.tm_hour, (uint8_t)tmv.tm_min, (uint8_t)tmv.tm_sec));
            last_time = now;
        }
    }
}

void mcu_link_start(void) {
    s_lock       = xSemaphoreCreateMutex();
    s_sniff_lock = xSemaphoreCreateMutex();
    s_cmdq       = xQueueCreate(8, sizeof(mcu_cmd_t));
    s_rawq       = xQueueCreate(4, sizeof(raw_frame_t));
    fc_parser_reset(&s_parser);

    uart_setup();
    uart_flush_input(MCU_UART);
    s_start_us   = esp_timer_get_time();
    s_next_tx_us = s_start_us + BOOT_QUIET_US;     // first frame only after the line settles
    trace_note(s_start_us, "link start (paced %d ms, boot quiet %d ms)", TX_GAP_US / 1000, BOOT_QUIET_US / 1000);

    xTaskCreate(mcu_task, "mcu", 4096, NULL, 10, NULL);
}
