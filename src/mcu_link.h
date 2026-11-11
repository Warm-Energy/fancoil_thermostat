// MCU link: owns UART0 @ 9600 (GPIO20 TX / GPIO19 RX), drives the panel MCU with the
// validated 5C-C5 codec (mcu_protocol.c), decodes its status into a shared state, and
// keeps a ring of raw frames for the network sniffer.
#pragma once
#include <stdbool.h>
#include "mcu_protocol.h"

// Start the UART task. Safe to call once from app_main.
void mcu_link_start(void);

// Snapshot the latest decoded state. Returns false until the first status frame arrives.
bool mcu_get_state(fc_state_t *out);

// Queue a control change; any field left at MCU_NOCHANGE is not sent. Non-blocking.
#define MCU_NOCHANGE (-32768)
typedef struct {
    int power;     // 0/1, or MCU_NOCHANGE
    int lock;      // 0/1
    int mode;      // 0.., or MCU_NOCHANGE
    int fan;       // 0.., or MCU_NOCHANGE
    int setpoint;  // whole °C, or MCU_NOCHANGE
} mcu_cmd_t;
void mcu_send_cmd(const mcu_cmd_t *cmd);

// Tell the link whether WiFi/IP is up. While connected, the task periodically sends a
// time-sync (type 0x11) frame — the "I'm online" signal to the panel MCU, which
// is what drives the panel WiFi light solid. Safe to call from any task.
void mcu_set_connected(bool connected);

// Set the MCU panel WiFi-light state: it's the payload[0] byte of the type-0x0F status frame
// the task sends every ~3 s. Known: 2 = solid (connected). Other values TBD empirically.
// Pass a negative value to stop sending (the MCU then falls back to its own blink).
void mcu_set_light(int state);

// Copy up to `max` bytes of the most-recent raw UART frames (hex-dump source for /log).
// Returns the number of bytes copied.
int mcu_sniffer_dump(char *out, int max);

// --- Link health, for the "test MCU connection" diagnostic (GET /mcu/test) ---
typedef struct {
    bool     link_up;        // any bytes received from the MCU since boot
    bool     status_valid;   // at least one status frame decoded
    uint32_t rx_bytes;       // total bytes received from the MCU
    uint32_t rx_frames;      // valid status frames decoded
    uint32_t tx_frames;      // frames we have sent to the MCU
    int32_t  last_rx_ms;     // ms since the last byte from the MCU    (-1 = never)
    int32_t  last_frame_ms;  // ms since the last decoded status frame (-1 = never)
} mcu_health_t;
void mcu_get_health(mcu_health_t *out);

// Active probe: poke the MCU with an immediate hello and wait up to timeout_ms for a
// fresh reply. Returns true if the MCU answered; *rtt_ms (if non-NULL) gets the round
// trip in ms. Safe to call from another task (e.g. an HTTP handler).
bool mcu_probe(int timeout_ms, int *rtt_ms);
