// Panel MCU <-> ESP32-C2 UART protocol — transport-agnostic codec.
// Transport-agnostic frame codec for the panel-MCU UART link.
// UART0, 9600 8N1. Frame: 5C C5 | type | len(16 LE) | seq | payload | cksum(16 LE sum).
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Frame types
enum {
  FC_T_HELLO   = 0x01,  // handshake (sent at startup)
  FC_T_TIME    = 0x11,  // time sync
  FC_T_DP      = 0x12,  // set one runtime datapoint (payload 10B; 0 = no change)
  FC_T_CONFIG  = 0x13,  // set config block (payload 9B)
  FC_T_QUERY   = 0x1F,  // refresh/query (payload [0]=1) — purpose unconfirmed
  FC_T_UPGCTRL = 0x0F,  // MCU-upgrade control
  FC_T_UPGDATA = 0x16,  // MCU-upgrade data chunk
};

#define FC_MAGIC0_TX 0x5C   // ESP->MCU frame starts 5C C5
#define FC_MAGIC1    0xC5   // ESP->MCU second magic byte (the encoder emits this)
#define FC_MAGIC0_RX 0x5B   // MCU->ESP frame starts 5B B5  (CONFIRMED 2026-11-10 /log capture)
#define FC_MAGIC1_RX 0xB5   // MCU->ESP second magic byte — NOT 0xC5; demanding C5 was the RX-decode bug

// Decoded appliance state (status frame type 0x01/0x12).
typedef struct {
  bool    power;            // flags bit7 (1 = on)
  uint8_t mode;             // (statusByte & 7) - 1   (enum: CONFIRM which n = heat/cool/fan)
  float   current_temp;     // int + frac/10          (units: CONFIRM)
  int     temp_set;         // raw setpoint           (units: CONFIRM)
  uint8_t fan_speed;        // 0-based
  bool    is_heating;       // flags bit6
  bool    is_cooling;       // flags bit2 (is_refri)
  bool    key_lock;         // flags bit5
  uint8_t sys_lock;
  int     adj;              // calibration offset
  uint8_t t_mode;
  uint8_t tolr;             // hysteresis
  uint8_t antif;            // anti-freeze
  uint8_t heat_top;
  uint8_t cool_bot;
  uint8_t fan_delay;
  bool    reset_provision;  // flags bit0 — user held FAN ~8s: wipe WiFi creds + reprovision (AP).
  bool    reset_reinit;     // flags bit1 — MCU asks to re-init WiFi; do NOT wipe creds.
} fc_state_t;

// ---- Encoders: build a complete frame into `out` (>=64B). Return total length. ----
size_t fc_encode(uint8_t type, const uint8_t *payload, uint16_t len, uint8_t seq, uint8_t *out);

// Convenience builders (each sends one 0x12 datapoint; other fields 0 = "no change").
size_t fc_set_power   (uint8_t *out, bool on);
size_t fc_set_keylock (uint8_t *out, bool locked);
size_t fc_set_mode    (uint8_t *out, uint8_t mode, uint8_t sys_lock);
size_t fc_set_fan     (uint8_t *out, uint8_t fan_speed);
size_t fc_set_temp    (uint8_t *out, uint8_t temp_set);
size_t fc_set_config  (uint8_t *out, int adj, uint8_t tolr, uint8_t antif,
                       uint8_t heat_top, uint8_t cool_bot, uint8_t fan_delay);
size_t fc_hello       (uint8_t *out);                 // startup handshake (payload: CONFIRM)
size_t fc_time        (uint8_t *out, uint8_t weekday, // y/m/d/h/m/s — layout: CONFIRM
                       uint8_t hour, uint8_t minute, uint8_t second);

// ---- Streaming RX parser: feed bytes; returns 1 when a status frame decoded into `st`. ----
typedef struct {
  uint8_t  buf[64];
  uint16_t idx;
  uint16_t need;   // total frame length once known (len+8), else 0
} fc_parser_t;

void fc_parser_reset(fc_parser_t *p);
// Returns 1 and fills *st when a valid status frame (type 0x01/0x12) completes; else 0.
int  fc_parse_byte(fc_parser_t *p, uint8_t b, fc_state_t *st);

#ifdef __cplusplus
}
#endif
