# FanCoil.co.uk thermostat firmware

Open ESP32-C2 firmware for the **FanCoil.co.uk** fan-coil thermostat. It is **local-first**:
controlled over a small local HTTP API by the
[FanCoil.co.uk Home Assistant integration](https://github.com/Warm-Energy/fancoil_home_assistant)
— no cloud account, no broker, nothing phoning home.

## Features

- Local HTTP/JSON API (`GET /state`, `POST /set`) plus an on-device web UI
- mDNS **and** DHCP auto-discovery (`fancoil-<mac>`) for Home Assistant
- SoftAP Wi-Fi provisioning (no app required)
- Dual-slot OTA with automatic rollback on a bad image
- Panel-MCU UART link: heat / cool / fan-only, fan Auto + 1–7, target temp, room temp,
  child-lock, and the heat/cool limits

See [`ARCHITECTURE.md`](ARCHITECTURE.md) for the full design.

## Build

```bash
pip install platformio
pio run -e fancoil
# -> .pio/build/fancoil/firmware.bin   (must be <= 896 KB)
```

## Flash

First flash over USB/serial (puts bootloader + partition table + app on the chip):

```bash
pio run -e fancoil -t upload --upload-port /dev/tty.usbserial-XXXX
```

After that, update over the air — no serial needed:

```bash
curl --data-binary @.pio/build/fancoil/firmware.bin http://<device-ip>/ota
```

…or just use the one-click update in the Home Assistant integration.

## Releases & OTA hosting

Pushing a `vX.Y.Z` tag triggers CI (`.github/workflows/build.yml`) to build, enforce the
896 KB size limit, and publish a **GitHub Release** containing `firmware.bin` and a
`latest.json` manifest:

```json
{ "version": "X.Y.Z",
  "url": "https://github.com/Warm-Energy/fancoil_thermostat/releases/download/vX.Y.Z/firmware.bin",
  "notes": "https://github.com/Warm-Energy/fancoil_thermostat/releases/tag/vX.Y.Z" }
```

The Home Assistant integration polls
`https://github.com/Warm-Energy/fancoil_thermostat/releases/latest/download/latest.json`
and offers the update automatically. To cut a release: bump [`version.txt`](version.txt),
commit, then `git tag vX.Y.Z && git push --tags`.

## Tests

Host-side unit tests for the MCU frame codec (no hardware needed) — see [`test/`](test/).

## License

[MIT](LICENSE) © FanCoil.co.uk (Warm-Energy)
