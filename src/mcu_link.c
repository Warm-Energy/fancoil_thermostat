// MCU link task — see mcu_link.h. Owns UART0 @ 9600, heartbeats the MCU, decodes status
// into a mutex-protected snapshot, applies queued control commands, and rings raw RX bytes.
#include "mcu_link.h"
#include <string.h>
#include <stdio.h>
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

static fc_state_t       s_state;
static bool             s_valid;
static SemaphoreHandle_t s_lock;
static QueueHandle_t     s_cmdq;

static uint8_t          s_sniff[SNIFF_CAP];
static int              s_sniff_len;
static SemaphoreHandle_t s_sniff_lock;

// --- link health instrumentation (guarded by s_lock) ---
static uint32_t         s_rx_bytes, s_rx_frames, s_tx_frames;
static int64_t          s_last_rx_us, s_last_frame_us;
static volatile bool    s_ping_req;   // set by mcu_probe(): poke the MCU with a hello now
static volatile bool    s_connected;  // WiFi up → periodically send time-sync (0x11) to MCU
static volatile int     s_light = -1;  // MCU panel-light state (payload[0] of 0x0F); <0 = don't send

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

bool mcu_get_state(fc_state_t *out) {
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool v = s_valid;
    if (v) *out = s_state;
    xSemaphoreGive(s_lock);
    return v;
}

void mcu_get_health(mcu_health_t *out) {
    int64_t now = esp_timer_get_time();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    out->rx_bytes     = s_rx_bytes;
    out->rx_frames    = s_rx_frames;
    out->tx_frames    = s_tx_frames;
    out->status_valid = s_valid;
    out->link_up      = s_rx_bytes > 0;
    out->last_rx_ms    = s_last_rx_us    ? (int32_t)((now - s_last_rx_us)    / 1000) : -1;
    out->last_frame_ms = s_last_frame_us ? (int32_t)((now - s_last_frame_us) / 1000) : -1;
    xSemaphoreGive(s_lock);
}

bool mcu_probe(int timeout_ms, int *rtt_ms) {
    xSemaphoreTake(s_lock, portMAX_DELAY);
    uint32_t b0 = s_rx_bytes, f0 = s_rx_frames;
    xSemaphoreGive(s_lock);

    int64_t t0 = esp_timer_get_time();
    s_ping_req = true;                                   // the task sends a hello next loop
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

static void tx(uint8_t *buf, size_t len) {
    if (!len) return;
    uart_write_bytes(MCU_UART, (const char *)buf, len);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_tx_frames++;
    xSemaphoreGive(s_lock);
}

static void apply_cmd(const mcu_cmd_t *c) {
    uint8_t o[64];
    if (c->setpoint != MCU_NOCHANGE) tx(o, fc_set_temp(o, (uint8_t)c->setpoint));
    if (c->mode     != MCU_NOCHANGE) tx(o, fc_set_mode(o, (uint8_t)c->mode, 0));
    if (c->fan      != MCU_NOCHANGE) tx(o, fc_set_fan(o, (uint8_t)c->fan));
    if (c->lock     != MCU_NOCHANGE) tx(o, fc_set_keylock(o, c->lock != 0));
    if (c->power    != MCU_NOCHANGE) tx(o, fc_set_power(o, c->power != 0));
}

static void mcu_task(void *arg) {
    uint8_t o[64], rx[128];
    fc_parser_t parser; fc_parser_reset(&parser);
    fc_state_t st;
    int64_t last_hello = 0, last_time = 0, last_net = 0;
    esp_task_wdt_add(NULL);                  // watchdog: reset the chip if this task ever hangs
    for (;;) {
        esp_task_wdt_reset();
        mcu_cmd_t c;
        while (xQueueReceive(s_cmdq, &c, 0)) apply_cmd(&c);

        int64_t now = esp_timer_get_time();
        if (s_ping_req) { s_ping_req = false; tx(o, fc_hello(o)); }                 // probe poke
        if (now - last_hello > 1000000) { tx(o, fc_hello(o)); last_hello = now; }   // ~1 Hz heartbeat
        if ((s_light == 1 || s_light == 2) && now - last_net > 3000000) {  // MCU panel light: 2=solid, 1=flash.
            uint8_t p[2] = { (uint8_t)s_light, 0 };                        // HARD CLAMP to 1/2 — every other
            tx(o, fc_encode(0x0F, p, 2, 0, o));                            // 0x0F payload is MCU upgrade-control
            last_net = now;                                               // and knocks out status reporting.
        }
        if (s_connected && now - last_time > 10000000) {    // ~10s: time-sync keeps the panel clock right
            time_t t = time(NULL);
            struct tm tmv; localtime_r(&t, &tmv);
            uint8_t wd = tmv.tm_wday == 0 ? 7 : (uint8_t)tmv.tm_wday;               // Sun=7, Mon..Sat=1..6
            tx(o, fc_time(o, wd, (uint8_t)tmv.tm_hour, (uint8_t)tmv.tm_min, (uint8_t)tmv.tm_sec));
            last_time = now;
        }

        int n = uart_read_bytes(MCU_UART, rx, sizeof rx, pdMS_TO_TICKS(100));
        if (n > 0) {
            sniff_push(rx, n);
            int64_t rxnow = esp_timer_get_time();
            xSemaphoreTake(s_lock, portMAX_DELAY);
            s_rx_bytes += (uint32_t)n; s_last_rx_us = rxnow;
            xSemaphoreGive(s_lock);
            for (int i = 0; i < n; i++) {
                if (fc_parse_byte(&parser, rx[i], &st)) {
                    xSemaphoreTake(s_lock, portMAX_DELAY);
                    s_state = st; s_valid = true;
                    s_rx_frames++; s_last_frame_us = rxnow;
                    xSemaphoreGive(s_lock);
                }
            }
        }
    }
}

void mcu_link_start(void) {
    s_lock       = xSemaphoreCreateMutex();
    s_sniff_lock = xSemaphoreCreateMutex();
    s_cmdq       = xQueueCreate(8, sizeof(mcu_cmd_t));

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

    xTaskCreate(mcu_task, "mcu", 4096, NULL, 10, NULL);
}
