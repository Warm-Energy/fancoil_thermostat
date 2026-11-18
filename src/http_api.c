// Local HTTP/JSON API + dev push-OTA — see http_api.h.
#include "http_api.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "nvs.h"
#include "cJSON.h"
#include "mcu_link.h"

static const char *TAG = "api";
static httpd_handle_t s_srv;

// "Test MCU connection" widget, appended to every page of the root UI. Pokes GET /mcu/test
// and shows the verdict + raw health JSON; "Resync" kicks the recovery ladder. No external deps.
static const char *MCU_TEST_HTML =
    "<h3>MCU link</h3>"
    "<button onclick=\"mt()\">Test MCU connection</button> "
    "<button onclick=\"fetch('/mcu/recover?step=auto',{method:'POST'}).then(_=>{mv.textContent=' resyncing...';"
    "setTimeout(mt,8000)})\">Resync now</button> "
    "<a href=/mcu/trace>timeline</a> <a href=/log>raw bytes</a> <b id=mv></b>"
    "<pre id=mo style=\"white-space:pre-wrap;background:#f4f4f4;padding:8px\"></pre>"
    "<script>function mt(){mv.textContent=' testing...';mo.textContent='';"
    "fetch('/mcu/test').then(r=>r.json()).then(j=>{"
    "mv.textContent=' ['+j.state.toUpperCase()+(j.pair_mode?', degraded':'')+']';"
    "mo.textContent=j.verdict+'\\n\\n'+JSON.stringify(j,null,2);})"
    ".catch(e=>{mv.textContent=' error';mo.textContent=''+e;})}</script>";

static esp_err_t h_root(httpd_req_t *r) {
    httpd_resp_set_type(r, "text/html");
    if ((bool)(intptr_t)r->user_ctx) {
        httpd_resp_sendstr_chunk(r,
            "<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
            "<h2>FanCoil.co.uk setup</h2>"
            "<form onsubmit=\"event.preventDefault();fetch('/wifi',{method:'POST',"
            "body:JSON.stringify({ssid:s.value,password:p.value})}).then(_=>b.textContent="
            "'Saved - rebooting, reconnect to your network.')\">"
            "SSID <input id=s><br>Password <input id=p type=password><br>"
            "<button>Save</button></form><p id=b></p>");
    } else {
        httpd_resp_sendstr_chunk(r,
            "<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
            "<h2>FanCoil.co.uk</h2><ul>"
            "<li><a href=/state>/state</a> (JSON)</li>"
            "<li>POST /set {power,mode,setpoint,fan,lock}</li>"
            "<li>POST /ota  (raw firmware.bin)</li>"
            "<li><a href=/log>/log</a> (MCU sniffer)</li></ul>"
            "<button onclick=\"if(confirm('Reboot the thermostat controller?'))"
            "fetch('/reboot',{method:'POST'}).then(_=>{rb.textContent=' rebooting...';"
            "setTimeout(()=>location.reload(),6000)})\">Reboot ESP32</button> <b id=rb></b>"
            "<h3>Firmware</h3><span id=fv>?</span><br>"
            "<input type=file id=fw accept=.bin> "
            "<button onclick=up()>Upload &amp; flash</button> "
            "<progress id=pg max=100 value=0 style=display:none></progress> <b id=us></b>"
            "<script>"
            "fetch('/state').then(r=>r.json()).then(j=>fv.textContent='Version: '+(j.fw||'?'));"
            "function up(){var f=fw.files[0];if(!f){us.textContent='pick a .bin';return;}"
            "var x=new XMLHttpRequest();x.open('POST','/ota');pg.style.display='';"
            "x.upload.onprogress=function(e){if(e.lengthComputable)pg.value=100*e.loaded/e.total;};"
            "x.onload=function(){us.textContent=x.responseText||'done';"
            "setTimeout(function(){location.reload();},9000);};"
            "x.onerror=function(){us.textContent='upload failed';};"
            "us.textContent=' uploading...';x.send(f);}"
            "</script>");
    }
    httpd_resp_sendstr_chunk(r, MCU_TEST_HTML);
    httpd_resp_sendstr_chunk(r, NULL);   // terminate the chunked response
    return ESP_OK;
}

static esp_err_t h_state(httpd_req_t *r) {
    fc_state_t st;
    cJSON *j = cJSON_CreateObject();

    // Stable device identity (ESP-side, always present even before the MCU replies).
    // The HA integration keys each unit on `mac`, so multiple thermostats never collide
    // and a DHCP IP change never strands the config entry.
    uint8_t mac[6];
    char macstr[13];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(macstr, sizeof macstr, "%02x%02x%02x%02x%02x%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    cJSON_AddStringToObject(j, "mac", macstr);
    cJSON_AddStringToObject(j, "model", "FanCoil.co.uk");
    const esp_app_desc_t *app = esp_app_get_description();
    cJSON_AddStringToObject(j, "fw", app ? app->version : "?");
    mcu_health_t h;
    mcu_get_health(&h);
    cJSON_AddStringToObject(j, "link", mcu_link_state_name(h.state));   // up/desync/silent/boot
    cJSON_AddBoolToObject(j, "degraded", h.pair_mode);

    if (mcu_get_state(&st)) {
        cJSON_AddBoolToObject(j, "valid", true);
        cJSON_AddBoolToObject(j, "power", st.power);
        // Powered off: mode/setpoint/fan describe nothing the unit is doing — the panel
        // just retains the last settings — so report null, not a stale value. room_temp
        // is a live reading and the limits/lock persist, so those stay reported.
        if (st.power) {
            cJSON_AddNumberToObject(j, "mode", st.mode);
            cJSON_AddNumberToObject(j, "setpoint_c", st.temp_set);
            cJSON_AddNumberToObject(j, "fan", st.fan_speed);
        } else {
            cJSON_AddNullToObject(j, "mode");
            cJSON_AddNullToObject(j, "setpoint_c");
            cJSON_AddNullToObject(j, "fan");
        }
        cJSON_AddNumberToObject(j, "room_temp_c", st.current_temp);
        cJSON_AddBoolToObject(j, "heating", st.is_heating);
        cJSON_AddBoolToObject(j, "cooling", st.is_cooling);
        cJSON_AddBoolToObject(j, "key_lock", st.key_lock);
        cJSON_AddNumberToObject(j, "heat_top", st.heat_top);
        cJSON_AddNumberToObject(j, "cool_bot", st.cool_bot);
        cJSON_AddNumberToObject(j, "adj", st.adj);
        cJSON_AddNumberToObject(j, "tolr", st.tolr);
        cJSON_AddNumberToObject(j, "antif", st.antif);
        cJSON_AddNumberToObject(j, "fan_delay", st.fan_delay);
    } else {
        cJSON_AddBoolToObject(j, "valid", false);  // no MCU status decoded yet
    }
    char *s = cJSON_PrintUnformatted(j);
    httpd_resp_set_type(r, "application/json");
    httpd_resp_sendstr(r, s ? s : "{}");
    if (s) cJSON_free(s);
    cJSON_Delete(j);
    return ESP_OK;
}

static int body_read(httpd_req_t *r, char *buf, int max) {
    int total = 0, got;
    while (total < max - 1 && (got = httpd_req_recv(r, buf + total, max - 1 - total)) > 0)
        total += got;
    buf[total > 0 ? total : 0] = 0;
    return total;
}

static esp_err_t h_set(httpd_req_t *r) {
    char buf[256];
    body_read(r, buf, sizeof buf);
    cJSON *j = cJSON_Parse(buf);
    if (!j) { httpd_resp_set_status(r, "400 Bad Request"); httpd_resp_sendstr(r, "bad json"); return ESP_OK; }
    mcu_cmd_t c = { MCU_NOCHANGE, MCU_NOCHANGE, MCU_NOCHANGE, MCU_NOCHANGE, MCU_NOCHANGE };
    cJSON *v;
    if ((v = cJSON_GetObjectItem(j, "power")))      c.power    = cJSON_IsBool(v) ? cJSON_IsTrue(v) : (int)cJSON_GetNumberValue(v);
    if ((v = cJSON_GetObjectItem(j, "lock")))       c.lock     = cJSON_IsTrue(v) ? 1 : 0;
    if ((v = cJSON_GetObjectItem(j, "mode")))       c.mode     = (int)cJSON_GetNumberValue(v);
    if ((v = cJSON_GetObjectItem(j, "fan")))        c.fan      = (int)cJSON_GetNumberValue(v);
    if ((v = cJSON_GetObjectItem(j, "setpoint")))   c.setpoint = (int)cJSON_GetNumberValue(v);
    if ((v = cJSON_GetObjectItem(j, "setpoint_c"))) c.setpoint = (int)cJSON_GetNumberValue(v);
    cJSON_Delete(j);
    mcu_send_cmd(&c);
    httpd_resp_sendstr(r, "ok");
    return ESP_OK;
}

static esp_err_t fail(httpd_req_t *r, const char *msg) {
    httpd_resp_set_status(r, "500 Internal Server Error");
    httpd_resp_sendstr(r, msg);
    return ESP_OK;
}

static esp_err_t h_ota(httpd_req_t *r) {
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    if (!part) return fail(r, "no ota partition");
    esp_ota_handle_t h = 0;
    if (esp_ota_begin(part, OTA_SIZE_UNKNOWN, &h) != ESP_OK) return fail(r, "ota begin failed");
    char buf[1024];
    int got, total = 0;
    while ((got = httpd_req_recv(r, buf, sizeof buf)) > 0) {
        if (esp_ota_write(h, buf, got) != ESP_OK) { esp_ota_abort(h); return fail(r, "ota write failed"); }
        total += got;
    }
    if (got < 0 || esp_ota_end(h) != ESP_OK) return fail(r, "ota end failed (incomplete?)");
    if (esp_ota_set_boot_partition(part) != ESP_OK) return fail(r, "set boot failed");
    ESP_LOGI(TAG, "OTA %d bytes -> %s, rebooting", total, part->label);
    httpd_resp_sendstr(r, "OTA OK, rebooting");
    vTaskDelay(pdMS_TO_TICKS(300));
    esp_restart();
    return ESP_OK;
}

static esp_err_t h_wifi(httpd_req_t *r) {
    char buf[256];
    body_read(r, buf, sizeof buf);
    cJSON *j = cJSON_Parse(buf);
    if (!j) { httpd_resp_set_status(r, "400 Bad Request"); httpd_resp_sendstr(r, "bad json"); return ESP_OK; }
    const char *ssid = cJSON_GetStringValue(cJSON_GetObjectItem(j, "ssid"));
    const char *pass = cJSON_GetStringValue(cJSON_GetObjectItem(j, "password"));
    if (ssid && ssid[0]) {
        nvs_handle_t nv;
        if (nvs_open("wifi", NVS_READWRITE, &nv) == ESP_OK) {
            nvs_set_str(nv, "ssid", ssid);
            nvs_set_str(nv, "pass", pass ? pass : "");
            nvs_commit(nv);
            nvs_close(nv);
        }
        if (nvs_open("cfg", NVS_READWRITE, &nv) == ESP_OK) {  // leaving AP setup -> STA on reboot
            nvs_erase_key(nv, "provision");
            nvs_commit(nv);
            nvs_close(nv);
        }
    }
    cJSON_Delete(j);
    httpd_resp_sendstr(r, "saved");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

static esp_err_t h_log(httpd_req_t *r) {
    char buf[3200];
    int n = mcu_sniffer_dump(buf, sizeof buf);
    httpd_resp_set_type(r, "text/plain");
    httpd_resp_send(r, buf, n);
    return ESP_OK;
}

// Active MCU connection test: poke the MCU and wait for a reply, then report link health.
static esp_err_t h_mcu_test(httpd_req_t *r) {
    int rtt = -1;
    bool ok = mcu_probe(2000, &rtt);
    mcu_health_t h;
    mcu_get_health(&h);

    cJSON *j = cJSON_CreateObject();
    cJSON_AddBoolToObject(j, "ok", ok);
    if (ok && rtt >= 0) cJSON_AddNumberToObject(j, "rtt_ms", rtt);
    else                cJSON_AddNullToObject(j, "rtt_ms");
    cJSON_AddBoolToObject(j, "link_up", h.link_up);
    cJSON_AddBoolToObject(j, "status_valid", h.status_valid);
    cJSON_AddNumberToObject(j, "rx_bytes", h.rx_bytes);
    cJSON_AddNumberToObject(j, "rx_frames", h.rx_frames);
    cJSON_AddNumberToObject(j, "tx_frames", h.tx_frames);
    if (h.last_rx_ms    >= 0) cJSON_AddNumberToObject(j, "last_rx_ms", h.last_rx_ms);       else cJSON_AddNullToObject(j, "last_rx_ms");
    if (h.last_frame_ms >= 0) cJSON_AddNumberToObject(j, "last_frame_ms", h.last_frame_ms); else cJSON_AddNullToObject(j, "last_frame_ms");
    cJSON_AddStringToObject(j, "state", mcu_link_state_name(h.state));
    cJSON_AddBoolToObject(j, "pair_mode", h.pair_mode);
    cJSON_AddNumberToObject(j, "ck_frames", h.ck_frames);
    cJSON_AddNumberToObject(j, "bad_ck", h.bad_ck);
    cJSON_AddNumberToObject(j, "bad_len", h.bad_len);
    cJSON_AddNumberToObject(j, "skipped", h.skipped);
    cJSON_AddNumberToObject(j, "rx_bursts", h.rx_bursts);
    cJSON_AddNumberToObject(j, "headerless", h.headerless);
    cJSON_AddNumberToObject(j, "rec_attempts", h.rec_attempts);
    cJSON_AddNumberToObject(j, "recoveries", h.recoveries);
    if (h.rec_active)       cJSON_AddStringToObject(j, "rec_active", h.rec_active);   else cJSON_AddNullToObject(j, "rec_active");
    if (h.rec_last_ok)      cJSON_AddStringToObject(j, "rec_last_ok", h.rec_last_ok); else cJSON_AddNullToObject(j, "rec_last_ok");
    if (h.rec_next_ms >= 0) cJSON_AddNumberToObject(j, "rec_next_ms", h.rec_next_ms); else cJSON_AddNullToObject(j, "rec_next_ms");
    cJSON_AddNumberToObject(j, "heap", esp_get_free_heap_size());

    const char *verdict;
    if (h.state == MCU_LINK_UP && h.pair_mode)
        verdict = "DEGRADED: the MCU's single replies arrive shifted (5B B5 header lost inside "
                  "the MCU), so we poll with back-to-back hello pairs to get intact frames. "
                  "Power-cycle the thermostat to clear it; single polls are retried every 5 min.";
    else if (h.state == MCU_LINK_UP)
        verdict = "MCU link OK - receiving and decoding status frames.";
    else if (h.state == MCU_LINK_BOOT)
        verdict = "Link starting (first few seconds after boot) - test again shortly.";
    else if (h.state == MCU_LINK_DESYNC && h.headerless > 4 * h.bad_ck)
        verdict = "MCU is answering, but its replies arrive WITHOUT the 5B B5 header - each "
                  "reply is shifted inside the MCU (not an ESP framing problem). Auto-recovery "
                  "is running (see rec_*); if it can't get frames, power-cycle the thermostat. "
                  "Timeline: /mcu/trace";
    else if (h.state == MCU_LINK_DESYNC && h.bad_ck)
        verdict = "Frames arrive with bad checksums - line noise, a loose wire or wrong baud.";
    else if (h.state == MCU_LINK_DESYNC)
        verdict = "MCU is sending bytes but no valid status frame decodes. Timeline: /mcu/trace";
    else if (h.link_up)
        verdict = "MCU has gone quiet (bytes were seen earlier). Auto-recovery is running.";
    else
        verdict = "No response from the MCU. Check wiring (ESP TX=GPIO20 -> MCU RX, "
                  "ESP RX=GPIO19 <- MCU TX - do not swap), MCU power, and 9600 8N1.";
    cJSON_AddStringToObject(j, "verdict", verdict);

    char *s = cJSON_PrintUnformatted(j);
    httpd_resp_set_type(r, "application/json");
    httpd_resp_sendstr(r, s ? s : "{}");
    if (s) cJSON_free(s);
    cJSON_Delete(j);
    return ESP_OK;
}

// TX/RX timeline: every frame we sent and every burst the MCU sent back, with ms timestamps
// and recovery notes. The thing to read when the link misbehaves.
static esp_err_t h_mcu_trace(httpd_req_t *r) {
    const int max = 11 * 1024;
    char *buf = malloc(max);
    if (!buf) return fail(r, "no memory");
    int n = mcu_trace_dump(buf, max);
    httpd_resp_set_type(r, "text/plain");
    httpd_resp_send(r, buf, n);
    free(buf);
    return ESP_OK;
}

// POST /mcu/recover?step=auto|uart_reset|quiet|pair_poll|break — run a recovery step now.
static esp_err_t h_mcu_recover(httpd_req_t *r) {
    char q[48], step[16] = "auto";
    if (httpd_req_get_url_query_str(r, q, sizeof q) == ESP_OK)
        httpd_query_key_value(q, "step", step, sizeof step);
    if (!mcu_recover(step)) {
        httpd_resp_set_status(r, "400 Bad Request");
        return httpd_resp_sendstr(r, "step must be auto, uart_reset, quiet, pair_poll or break");
    }
    char msg[48];
    snprintf(msg, sizeof msg, "started %s - see /mcu/test", step);
    return httpd_resp_sendstr(r, msg);
}

// Dev: POST /mcu/raw {"type":31,"payload":"01"} — send one frame (framed + checksummed
// here). MCU upgrade frames are refused (see mcu_send_raw).
static esp_err_t h_mcu_raw(httpd_req_t *r) {
    char buf[200];
    body_read(r, buf, sizeof buf);
    cJSON *j = cJSON_Parse(buf);
    cJSON *t = j ? cJSON_GetObjectItem(j, "type") : NULL;
    const char *hex = j ? cJSON_GetStringValue(cJSON_GetObjectItem(j, "payload")) : NULL;
    uint8_t p[56];
    int len = 0;
    bool ok = t && cJSON_IsNumber(t);
    for (const char *h = hex; ok && h && h[0] && h[1]; h += 2) {
        unsigned v;
        if (len >= (int)sizeof p || sscanf(h, "%2x", &v) != 1) ok = false;
        else p[len++] = (uint8_t)v;
    }
    ok = ok && mcu_send_raw((uint8_t)cJSON_GetNumberValue(t), p, (uint16_t)len);
    cJSON_Delete(j);
    if (!ok) {
        httpd_resp_set_status(r, "400 Bad Request");
        return httpd_resp_sendstr(r, "refused: need {type:n, payload:\"hex\"}; upgrade frames are blocked");
    }
    return httpd_resp_sendstr(r, "queued - see /mcu/trace");
}

// Dev: set the MCU panel-light state directly — GET /light?s=N sets payload[0] of the 0x0F
// status frame (2 = solid). Lets us map which values blink / turn off from a browser.
static esp_err_t h_light(httpd_req_t *r) {
    char q[32], v[8];
    int state = 2;
    if (httpd_req_get_url_query_str(r, q, sizeof q) == ESP_OK &&
        httpd_query_key_value(q, "s", v, sizeof v) == ESP_OK)
        state = atoi(v);
    char msg[72];
    int n;
    if (state == 1 || state == 2) {        // only safe panel-light states; other 0x0F = upgrade-control
        mcu_set_light(state);
        n = snprintf(msg, sizeof msg, "MCU light = %s", state == 2 ? "solid" : "flash");
    } else {
        n = snprintf(msg, sizeof msg, "ignored: only 1 (flash) / 2 (solid) are safe to send");
    }
    httpd_resp_set_type(r, "text/plain");
    httpd_resp_send(r, msg, n);
    return ESP_OK;
}

// Reboot the ESP (not the MCU). POST so a browser prefetch can't trip it.
static esp_err_t h_reboot(httpd_req_t *r) {
    httpd_resp_sendstr(r, "rebooting");
    ESP_LOGI(TAG, "reboot requested via /reboot");
    vTaskDelay(pdMS_TO_TICKS(300));
    esp_restart();
    return ESP_OK;
}

void http_api_start(bool ap_mode) {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size        = 8192;        // OTA handler needs headroom
    cfg.max_uri_handlers  = 16;
    cfg.lru_purge_enable  = true;
    cfg.recv_wait_timeout = 20;
    cfg.send_wait_timeout = 20;
    if (httpd_start(&s_srv, &cfg) != ESP_OK) return;

    httpd_uri_t u;
#define REG(p, m, fn, ctx) do { u = (httpd_uri_t){ .uri = p, .method = m, .handler = fn, .user_ctx = ctx }; \
                                httpd_register_uri_handler(s_srv, &u); } while (0)
    REG("/",      HTTP_GET,  h_root,  (void *)(intptr_t)ap_mode);
    REG("/state", HTTP_GET,  h_state, NULL);
    REG("/set",   HTTP_POST, h_set,   NULL);
    REG("/ota",   HTTP_POST, h_ota,   NULL);
    REG("/wifi",  HTTP_POST, h_wifi,  NULL);
    REG("/log",   HTTP_GET,  h_log,   NULL);
    REG("/mcu/test", HTTP_GET, h_mcu_test, NULL);
    REG("/mcu/trace",   HTTP_GET,  h_mcu_trace,   NULL);
    REG("/mcu/recover", HTTP_POST, h_mcu_recover, NULL);
    REG("/mcu/raw",     HTTP_POST, h_mcu_raw,     NULL);
    REG("/light",    HTTP_GET, h_light,    NULL);
    REG("/reboot",   HTTP_POST, h_reboot,  NULL);
#undef REG
}
