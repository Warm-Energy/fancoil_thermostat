// MCU protocol codec — see mcu_protocol.h.
#include "mcu_protocol.h"
#include <string.h>

// 16-bit additive checksum over the first `n` bytes.
static uint16_t fc_cksum(const uint8_t *b, size_t n) {
  uint16_t s = 0;
  for (size_t i = 0; i < n; i++) s += b[i];
  return s;
}

// Control/datapoint/config/hello/time frames all use seq=0 — confirmed on a 2026-11-10
// UART capture (every 5C C5 control frame had byte[5]=0x00). Only
// MCU-upgrade-data frames (type 0x16, not built here) increment a sequence counter.

size_t fc_encode(uint8_t type, const uint8_t *payload, uint16_t len, uint8_t seq, uint8_t *out) {
  out[0] = FC_MAGIC0_TX;
  out[1] = FC_MAGIC1;
  out[2] = type;
  out[3] = (uint8_t)(len & 0xFF);
  out[4] = (uint8_t)(len >> 8);
  out[5] = seq;
  if (len && payload) memcpy(out + 6, payload, len);
  uint16_t ck = fc_cksum(out, 6 + len);   // header(6) + payload
  out[6 + len] = (uint8_t)(ck & 0xFF);
  out[7 + len] = (uint8_t)(ck >> 8);
  return (size_t)len + 8;
}

// --- datapoint (0x12) builders: 10-byte payload, one field set, rest 0 = no change ---
static size_t dp(uint8_t *out, uint8_t off, uint8_t val) {
  uint8_t p[10] = {0};
  p[off] = val;
  return fc_encode(FC_T_DP, p, sizeof(p), 0, out);
}
size_t fc_set_power  (uint8_t *out, bool on)        { return dp(out, 0, on ? 1 : 2); }       // 1=on,2=off
size_t fc_set_keylock(uint8_t *out, bool locked)    { return dp(out, 1, locked ? 2 : 1); }
size_t fc_set_fan    (uint8_t *out, uint8_t fan)    { return dp(out, 6, (uint8_t)(fan + 1)); }
size_t fc_set_temp   (uint8_t *out, uint8_t temp)   { return dp(out, 8, temp); }
size_t fc_set_mode   (uint8_t *out, uint8_t mode, uint8_t sys_lock) {
  return dp(out, 2, (uint8_t)(((mode + 1) & 0x1F) | ((sys_lock & 0x7) << 5)));
}
size_t fc_set_config(uint8_t *out, int adj, uint8_t tolr, uint8_t antif,
                     uint8_t heat_top, uint8_t cool_bot, uint8_t fan_delay) {
  uint8_t p[9] = {0};
  p[0] = (uint8_t)(adj + 0x14);
  p[1] = tolr;
  p[4] = antif;
  p[5] = heat_top;
  p[6] = cool_bot;
  p[7] = fan_delay;
  return fc_encode(FC_T_CONFIG, p, sizeof(p), 0, out);
}
size_t fc_hello(uint8_t *out) {
  // Hello payload confirmed by a UART capture: {0x01, 0x02}
  // (frame 5c c5 01 02 00 00 01 02 27 01). Earlier {0x01,'2'} was a wrong guess.
  uint8_t p[2] = {0x01, 0x02};
  return fc_encode(FC_T_HELLO, p, sizeof(p), 0, out);
}
size_t fc_time(uint8_t *out, uint8_t weekday, uint8_t hour, uint8_t minute, uint8_t second) {
  // Exact field order/epoch is unconfirmed; this is a plausible 8-byte layout to refine.
  uint8_t p[8] = {0};
  p[0] = weekday ? weekday : 7;
  p[1] = hour; p[2] = minute; p[3] = second;
  return fc_encode(FC_T_TIME, p, sizeof(p), 0, out);
}

// --------------------------- streaming RX parser ---------------------------
void fc_parser_reset(fc_parser_t *p) { p->idx = 0; p->need = 0; }

static int fc_decode_status(const uint8_t *f, uint16_t total, fc_state_t *st) {
  // f[2]=type; payload starts at f[6]; status fields are at FRAME offsets (per RE).
  uint8_t type = f[2];
  if (type != 0x01 && type != 0x12) return 0;
  if (total < 0x24) return 0;                 // need through offset 0x21 (+cksum)
  uint8_t flags = f[0x0C];
  memset(st, 0, sizeof(*st));
  st->current_temp   = (float)f[0x08] + (float)f[0x0A] / 10.0f;
  st->mode           = (uint8_t)((f[0x0B] & 0x07) ? (f[0x0B] & 0x07) - 1 : 0);
  st->sys_lock       = (uint8_t)(f[0x0B] >> 5);
  st->power          = (flags >> 7) & 1;
  st->is_heating     = (flags >> 6) & 1;
  st->key_lock       = (flags >> 5) & 1;
  st->is_cooling     = (flags >> 2) & 1;
  st->reset_provision = flags & 1;             // bit0 — provisioning reset (FAN held ~8s)
  st->reset_reinit    = (flags >> 1) & 1;      // bit1 — WiFi re-init request (keep creds)
  st->temp_set       = f[0x0D];
  st->adj            = (int)f[0x12] - 0x14;
  st->t_mode         = (uint8_t)(f[0x13] ? f[0x13] - 1 : 0);
  st->tolr           = f[0x14];
  st->antif          = (uint8_t)(f[0x15] & 0x1F);
  st->fan_speed      = (uint8_t)(f[0x1D] ? f[0x1D] - 1 : 0);
  st->heat_top       = f[0x1F];
  st->cool_bot       = f[0x20];
  st->fan_delay      = f[0x21];
  return 1;
}

int fc_parse_byte(fc_parser_t *p, uint8_t b, fc_state_t *st) {
  // Byte 0: a start magic (accept both 0x5B and 0x5C; checksum is the real guard).
  if (p->idx == 0) {
    if (b == FC_MAGIC0_RX || b == FC_MAGIC0_TX) { p->buf[p->idx++] = b; p->need = 0; }
    return 0;
  }
  if (p->idx == 1) {                       // byte 1: 0xB5 for MCU->ESP (RX), 0xC5 for ESP->MCU
    if (b == FC_MAGIC1_RX || b == FC_MAGIC1) { p->buf[p->idx++] = b; return 0; }
    // resync: maybe this byte is a fresh start magic
    p->idx = 0;
    if (b == FC_MAGIC0_RX || b == FC_MAGIC0_TX) p->buf[p->idx++] = b;
    return 0;
  }
  p->buf[p->idx++] = b;
  if (p->idx == 5) {                        // have len(16LE) at [3..4]
    uint16_t len = (uint16_t)p->buf[3] | ((uint16_t)p->buf[4] << 8);
    p->need = (uint16_t)len + 8;
    if (p->need > sizeof(p->buf)) { fc_parser_reset(p); return 0; }  // bogus length
  }
  if (p->need && p->idx >= p->need) {
    uint16_t len = p->need - 8;
    uint16_t ck  = fc_cksum(p->buf, 6 + len);
    uint16_t got = (uint16_t)p->buf[6 + len] | ((uint16_t)p->buf[7 + len] << 8);
    int ok = 0;
    if (ck == got) ok = fc_decode_status(p->buf, p->need, st);
    fc_parser_reset(p);
    return ok;
  }
  if (p->idx >= sizeof(p->buf)) fc_parser_reset(p);  // safety
  return 0;
}
