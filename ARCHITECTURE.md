# FanCoil.co.uk thermostat firmware — architecture

A lean ESP-IDF application for the **FanCoil.co.uk** fan-coil thermostat. It is
**local-first**: out of the box a unit needs only Wi-Fi, and the
[FanCoil.co.uk Home Assistant integration](https://github.com/Warm-Energy/fancoil_home_assistant)
discovers and controls it directly over a small HTTP/JSON API on the LAN. No broker and no
cloud account are required.

## Hardware

- **SoC:** ESP32-C2 (ESP8684, RISC-V), 2 MB flash, **26 MHz** crystal (not the 40 MHz IDF
  default — see `sdkconfig.defaults`).
- **Panel MCU:** a separate microcontroller drives the segment LCD, relays and buttons. The
  ESP talks to it over **UART0 @ 9600 8N1** (GPIO20 TX / GPIO19 RX). Because UART0 is the MCU
  link, the serial console is disabled; diagnostics are exposed over the network instead.

## Flash / partition layout

Dual-OTA (A/B) in 2 MB, defined in `partitions_2mb_ota.csv`:

| Region | Offset | Size |
|---|---|---|
| bootloader | `0x0` | — |
| partition table | `0x8000` | — |
| `nvs` | `0x9000` | `0x4000` |
| `otadata` | `0xd000` | `0x2000` |
| `phy_init` | `0xf000` | `0x1000` |
| `ota_0` (app) | `0x10000` | `0xE0000` (896 KB) |
| `ota_1` (app) | `0xf0000` | `0xE0000` (896 KB) |

The app must fit the **896 KB** slot; the build is `-Os` and CI gates the size.

## Source layout (`src/`)

| File | Responsibility |
|---|---|
| `main.c` | Wi-Fi STA + SoftAP provisioning, mDNS/DHCP identity, start-up, OTA rollback guard |
| `mcu_protocol.c/.h` | Transport-agnostic UART frame codec (encode/decode, checksum, status decode) |
| `mcu_link.c/.h` | UART0 task: owns the link, sends the heartbeat/set/config frames, decodes status into a shared `fc_state_t`, keeps a ring of raw frames for the sniffer |
| `http_api.c/.h` | `esp_http_server`: the local REST API + the on-device web UI (cJSON) |

## MCU link

A UART0 task @ 9600 owns the panel-MCU link. Frames are `5C C5 | type | len(16 LE) | seq |
payload | cksum(16 LE sum)`. While Wi-Fi is up the task periodically sends a time-sync frame
(the "I'm online" signal that drives the panel Wi-Fi light solid); it sends `set`/`config`
frames on command and decodes incoming status into the shared `fc_state_t` that the HTTP API
serves. A reset-request in the status flags is honoured as **reprovision** (drop to SoftAP),
clearing only the Wi-Fi credentials.

## Local HTTP API

`esp_http_server` on port 80. Routes:

| Route | Method | Purpose |
|---|---|---|
| `/` | GET | web UI — setup form in SoftAP mode, live status + firmware upload otherwise |
| `/state` | GET | full status as JSON (see below) |
| `/set` | POST | change control/config fields (partial JSON) |
| `/ota` | POST | flash a firmware image (raw `.bin` body) |
| `/wifi` | POST | set Wi-Fi credentials (provisioning) |
| `/reboot` | POST | reboot the device |
| `/log` | GET | raw MCU frame sniffer (diagnostics) |
| `/mcu/test`, `/light` | GET | MCU link diagnostics |

`GET /state` reports `power`, `mode`, `setpoint_c`, `fan`, `key_lock`, `room_temp_c`,
`heating`/`cooling`, limits (`heat_top`, `cool_bot`), tuning (`adj`, `tolr`, `antif`,
`fan_delay`), plus `mac`, `model`, `fw`, link health (`valid`, `link_up`, `rtt_ms`, frame
counters). When the unit is **off**, `mode`, `setpoint_c` and `fan` are reported as `null`
(they describe nothing it's doing — the panel only retains them), so a consumer never sees
a stale speed or setpoint; `room_temp_c`, the limits and `key_lock` stay reported.

`POST /set` accepts any subset of the writable fields (`power`, `mode`,
`setpoint`, `fan`, `lock`, `heat_top`, `cool_bot`, `adj`, `tolr`, `antif`, `fan_delay`).

The API is unauthenticated by design — the device is meant for a trusted LAN, and `POST /ota`
is intentionally open so the unit stays user-reflashable. Rollback (below) is the safety net.

## Discovery & identity

- **mDNS**: advertises `_fancoil._tcp` with a MAC-derived hostname `fancoil-<mac>`, so several
  units are distinguishable and the HA integration auto-discovers them.
- **DHCP**: sets the DHCP hostname to `fancoil-<mac>` too, so discovery also works on networks
  that prune multicast.

## Provisioning (SoftAP)

On first boot or after a reprovision, the device raises AP `fancoil-<mac>` at `192.168.4.1`
and serves a plain setup page (Wi-Fi SSID/password → `POST /wifi`). No app or BLE needed.

## OTA & rollback

Updates are a raw image `POST`ed to `/ota` (by the HA update entity, the web UI, or `curl`);
the device writes the inactive slot and reboots into it. With
`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`, a new image boots as *pending-verify* and is marked
valid only after ~30 s uptime — a boot-looping image auto-reverts to the previous slot. The
reported version comes from `version.txt` (→ `PROJECT_VER` → `/state.fw`).

## Build

```bash
pio run -e fancoil          # -> .pio/build/fancoil/firmware.bin  (must be <= 896 KB)
```

CI (`.github/workflows/build.yml`) builds on every push, gates the size, and on a `v*` tag
publishes `firmware.bin` + a `latest.json` manifest as a GitHub Release that the HA update
entity polls. Host unit tests for the codec: see `test/`.
