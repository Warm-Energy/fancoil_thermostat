// Fancoil custom firmware — app entry + WiFi provisioning state machine.
//
// Provisioning policy (secure by default — this firmware NEVER auto-opens an AP):
//   * creds present      -> STA; reconnect forever with 5 s backoff; never falls back to AP.
//   * no creds, no flag   -> adopt the previous firmware's STA network if the WiFi driver kept
//                            one (first boot after OTA); else IDLE: WiFi off, dormant.
//   * provision flag set  -> SoftAP for setup. The flag is set ONLY when the user holds the
//                            FAN button ~8 s, which makes the MCU raise reset bit0
//                            (reset_provision). We then wipe the `wifi` NVS namespace
//                            (keeping `cfg` = server/token) and reboot into AP.
//
// The panel WiFi light is driven by the MCU, not us; we will signal status to it over UART
// (see the TODO in on_wifi_evt). Reset detection is edge-triggered and only armed after the
// flag has been seen clear, so a stuck bit can't put us in a reprovision loop.
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_ota_ops.h"
#include "mdns.h"
#include "mcu_link.h"
#include "http_api.h"

static const char *TAG = "main";
static esp_timer_handle_t s_retry;     // STA reconnect backoff timer
static bool s_ap_mode;                 // true while serving SoftAP setup

// ---- NVS: "wifi" ns = STA creds (wiped on reset); "cfg" ns = flags/server (preserved) ----
static bool load_creds(char *ssid, size_t ssz, char *pass, size_t psz) {
    nvs_handle_t nv;
    if (nvs_open("wifi", NVS_READONLY, &nv) != ESP_OK) return false;
    size_t l = ssz;
    esp_err_t e = nvs_get_str(nv, "ssid", ssid, &l);
    size_t lp = psz;
    if (nvs_get_str(nv, "pass", pass, &lp) != ESP_OK) pass[0] = 0;
    nvs_close(nv);
    return e == ESP_OK && ssid[0];
}

// First boot after an OTA from other firmware: our `wifi` namespace is empty, but the
// previous firmware may have left its STA network in the WiFi driver's own NVS (nvs.net80211,
// the default WIFI_STORAGE_FLASH). Adopt it into `wifi` once, so a unit that was already on
// Wi-Fi comes up on the LAN instead of dormant. The FAN-hold reset wipes the driver copy
// too (esp_wifi_restore), so wiped creds never come back this way.
static bool adopt_driver_creds(char *ssid, size_t ssz, char *pass, size_t psz) {
    wifi_config_t wc = {0};
    if (esp_wifi_get_config(WIFI_IF_STA, &wc) != ESP_OK || !wc.sta.ssid[0]) return false;
    // driver fields are fixed-size and not NUL-terminated when full
    snprintf(ssid, ssz, "%.*s", (int)sizeof wc.sta.ssid, (const char *)wc.sta.ssid);
    snprintf(pass, psz, "%.*s", (int)sizeof wc.sta.password, (const char *)wc.sta.password);
    nvs_handle_t nv;
    if (nvs_open("wifi", NVS_READWRITE, &nv) == ESP_OK) {
        nvs_set_str(nv, "ssid", ssid);
        nvs_set_str(nv, "pass", pass);
        nvs_commit(nv);
        nvs_close(nv);
    }
    ESP_LOGI(TAG, "adopted STA creds for '%s' from the WiFi driver NVS", ssid);
    return true;
}

static bool provision_pending(void) {
    nvs_handle_t nv;
    uint8_t v = 0;
    if (nvs_open("cfg", NVS_READONLY, &nv) == ESP_OK) {
        nvs_get_u8(nv, "provision", &v);
        nvs_close(nv);
    }
    return v != 0;
}

// Called when the MCU requests a WiFi reset (user held FAN ~8 s). Wipes STA creds ONLY —
// the `cfg` namespace (custom server/token) is preserved — arms AP setup, and reboots.
static void enter_provisioning_and_reboot(void) {
    nvs_handle_t nv;
    if (nvs_open("wifi", NVS_READWRITE, &nv) == ESP_OK) { nvs_erase_all(nv); nvs_commit(nv); nvs_close(nv); }
    esp_wifi_restore();            // driver's persisted STA config too (see adopt_driver_creds)
    if (nvs_open("cfg",  NVS_READWRITE, &nv) == ESP_OK) { nvs_set_u8(nv, "provision", 1); nvs_commit(nv); nvs_close(nv); }
    ESP_LOGI(TAG, "MCU wifi-reset (FAN 8s): creds wiped, rebooting into AP setup");
    esp_restart();
}

// ---- SNTP: started once on first connect so the time-sync we push the MCU is real ----
static bool s_sntp_started;
static void start_sntp(void) {
    if (s_sntp_started) return;
    s_sntp_started = true;
    setenv("TZ", "GMT0BST,M3.5.0/1,M10.5.0", 1);   // UK (GMT/BST); TODO: make configurable
    tzset();
    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_netif_sntp_init(&cfg);                     // non-fatal; time just stays unset if it fails
    ESP_LOGI(TAG, "SNTP started");
}

// ---- mDNS: advertise _fancoil._tcp so the HA integration auto-discovers us ----
// Name is MAC-derived (fancoil-<mac4mac5>, same scheme as the SoftAP SSID) so several
// units on one LAN never clash; the full MAC rides in a TXT record as the stable id HA
// keys each device on. Started once, after we first get an IP.
static bool s_mdns_started;
static void start_mdns(void) {
    if (s_mdns_started) return;
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char id[16], macstr[13];
    snprintf(id, sizeof id, "fancoil-%02x%02x", mac[4], mac[5]);
    snprintf(macstr, sizeof macstr, "%02x%02x%02x%02x%02x%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    if (mdns_init() != ESP_OK) { ESP_LOGW(TAG, "mdns init failed"); return; }
    s_mdns_started = true;
    mdns_hostname_set(id);
    mdns_instance_name_set(id);
    mdns_txt_item_t txt[] = {
        { "mac",   macstr },
        { "model", "FanCoil.co.uk" },
    };
    mdns_service_add(NULL, "_fancoil", "_tcp", 80, txt, 2);   // TXT strings are copied here
    ESP_LOGI(TAG, "mDNS up: %s.local advertising _fancoil._tcp:80", id);
}

// ---- WiFi events: reconnect STA with backoff; NO automatic AP fallback ----
static void retry_cb(void *a) { esp_wifi_connect(); }

static void on_wifi_evt(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (s_ap_mode) return;         // setup: the STA side only scans (GET /scan), never connects
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        mcu_set_connected(false);
        mcu_set_light(1);          // flash: not connected
        esp_timer_stop(s_retry);
        esp_timer_start_once(s_retry, 5000000);   // retry in 5 s — never opens an AP
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "wifi connected");
        mcu_set_connected(true);
        mcu_set_light(2);          // solid: on the network (0x0F payload[0]=2, confirmed)
        start_sntp();
        start_mdns();              // advertise for HA discovery once we're on the LAN
    }
}

static void wifi_common_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&ic));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_evt, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi_evt, NULL, NULL));
    const esp_timer_create_args_t ta = { .callback = retry_cb, .name = "wifiretry" };
    ESP_ERROR_CHECK(esp_timer_create(&ta, &s_retry));
}

static void start_sta(const char *ssid, const char *pass) {
    esp_netif_t *sta = esp_netif_create_default_wifi_sta();
    // DHCP hostname = fancoil-<mac4mac5>. HA's DHCP discovery matches "fancoil-*" and
    // auto-adds us — this is the discovery path that survives multicast-filtered WiFi
    // (mDNS 224.x gets pruned by IGMP snooping; the DHCP broadcast is still flooded).
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char id[16];
    snprintf(id, sizeof id, "fancoil-%02x%02x", mac[4], mac[5]);
    esp_netif_set_hostname(sta, id);
    wifi_config_t wc = {0};
    strlcpy((char *)wc.sta.ssid, ssid, sizeof wc.sta.ssid);
    strlcpy((char *)wc.sta.password, pass, sizeof wc.sta.password);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "STA connecting to '%s'", ssid);
}

static void start_ap(void) {
    esp_netif_create_default_wifi_ap();   // gateway 192.168.4.1
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    wifi_config_t wc = {0};
    snprintf((char *)wc.ap.ssid, sizeof wc.ap.ssid, "fancoil-%02x%02x", mac[4], mac[5]);
    wc.ap.ssid_len       = strlen((char *)wc.ap.ssid);
    wc.ap.channel        = 1;
    wc.ap.max_connection = 4;
    wc.ap.authmode       = WIFI_AUTH_OPEN;
    // APSTA, not AP: the STA side lets the setup page scan for networks (GET /scan). It
    // never connects — on_wifi_evt ignores STA events while s_ap_mode is set.
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "SoftAP '%s' up — http://192.168.4.1/", wc.ap.ssid);
}

// Watches the MCU's provisioning-reset request (FAN held ~8 s -> flag bit0). Edge-triggered:
// we only act after first seeing the flag clear, so a flag still asserted at boot (e.g. just
// after a reset reboot) can't trigger a reprovision loop.
static void prov_guard_task(void *a) {
    esp_task_wdt_add(NULL);                      // watchdog-fed health task
    bool armed = false, marked = false;
    int  hits = 0, low_heap = 0;
    for (;;) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(250));
        // OTA rollback: once we've run ~30 s without crashing/boot-looping, confirm this
        // image is good so the bootloader keeps it. Uptime-gated only — a healthy build
        // that simply can't reach WiFi must NOT be rolled back. No-op if not pending.
        if (!marked && esp_timer_get_time() > 30000000) {
            esp_ota_mark_app_valid_cancel_rollback();
            marked = true;
        }
        // memory safety net: reboot if critically low on heap for ~2 s (guards against a leak)
        if (esp_get_free_heap_size() < 8192) { if (++low_heap >= 8) esp_restart(); }
        else low_heap = 0;
        // MCU provisioning-reset watch (FAN held ~8 s)
        if (s_ap_mode) continue;                 // already in setup — ignore
        fc_state_t st;
        if (!mcu_get_state(&st)) continue;       // no MCU status yet
        if (!st.reset_provision) { armed = true; hits = 0; continue; }   // clear -> arm baseline
        if (armed && ++hits >= 4) enter_provisioning_and_reboot();        // ~1 s sustained
    }
}

void app_main(void) {
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    wifi_common_init();

    char ssid[33] = {0}, pass[65] = {0};
    if (provision_pending()) {
        s_ap_mode = true;         // set first: STA_START fires inside start_ap()
        start_ap();
    } else if (load_creds(ssid, sizeof ssid, pass, sizeof pass) ||
               adopt_driver_creds(ssid, sizeof ssid, pass, sizeof pass)) {
        start_sta(ssid, pass);
    } else {
        ESP_LOGI(TAG, "no creds — idle, no AP (hold FAN ~8 s to set up WiFi)");
    }

    mcu_set_light(1);             // default = flash (not connected); GOT_IP flips it to solid (2)
    mcu_link_start();
    http_api_start(s_ap_mode);
    xTaskCreate(prov_guard_task, "prov", 4096, NULL, 5, NULL);
}
