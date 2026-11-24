// Local HTTP/JSON API + dev push-OTA. Started once from app_main.
//   GET  /         info page, or the Wi-Fi setup form in SoftAP mode
//   GET  /state    thermostat state as JSON
//   POST /set      {"power":true,"mode":1,"setpoint":22,"fan":2,"lock":false}
//   POST /ota      raw firmware.bin body -> writes inactive slot, reboots   (dev: no auth)
//   POST /wifi     {"ssid":"...","password":"..."} -> save + reboot
//   POST /wifi/reset  forget the network, reboot into the SoftAP setup (= FAN held ~8 s)
//   GET  /log      recent raw MCU UART bytes (hex) — the network sniffer
#pragma once
#include <stdbool.h>
void http_api_start(bool ap_mode);
void enter_provisioning_and_reboot(void);   // main.c: wipe Wi-Fi creds, reboot into AP setup
