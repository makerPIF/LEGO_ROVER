/* ===========================================================================
 *  esp32cam_rover.ino  --  SPIKE Prime 탐사로버용 ESP32-CAM 영상 서버
 *  보드 : AI-Thinker ESP32-CAM (OV2640)
 *
 *  제공 엔드포인트
 *    http://<ip>/           간단한 안내 페이지
 *    http://<ip>/status     JSON 상태 (해상도/품질/RSSI/heap/fps/전압)
 *    http://<ip>/capture    JPEG 한 장  (GCS의 AI 분석은 이걸 폴링한다)
 *    http://<ip>/control?var=framesize&val=7   설정 변경
 *    http://<ip>/flash?val=0~255               플래시 LED 밝기
 *    http://<ip>:81/stream  MJPEG 실시간 스트림 (화면 표시용)
 *
 *  모든 응답에 Access-Control-Allow-Origin: * 를 붙이므로
 *  브라우저 GCS에서 canvas 로 픽셀 분석이 가능하다(=taint 되지 않음).
 *
 *  아두이노 IDE 설정
 *    보드      : "AI Thinker ESP32-CAM"
 *    Partition : Huge APP (3MB No OTA)
 *    업로드 시 : GPIO0 - GND 연결 후 리셋, 업로드 끝나면 반드시 분리
 *
 *  ★ 전원 : USB-TTL 어댑터의 5V 로는 스트리밍 중 브라운아웃이 잘 난다.
 *          5V / 1A 이상 별도 전원 + 5V-GND 사이 470uF 이상 전해 콘덴서 권장.
 *          (자세한 내용은 함께 제공한 전원 가이드 문서 참고)
 * =========================================================================== */

#include "esp_camera.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/ledc.h"

// ----------------------------- 사용자 설정 ---------------------------------
const char *WIFI_SSID = "여기에_와이파이_이름";
const char *WIFI_PASS = "여기에_와이파이_비밀번호";

// 공유기가 없을 때(운동장, 대회장) 자동으로 AP 모드로 전환한다.
const char *AP_SSID   = "ROVER-CAM";
const char *AP_PASS   = "rover1234";      // 8자 이상
const char *MDNS_NAME = "rover-cam";      // http://rover-cam.local
const uint32_t STA_TIMEOUT_MS = 15000;

// 시작 해상도: 로버 주행 + 실시간 분석에는 QVGA(320x240)가 가장 안정적이다.
#define START_FRAMESIZE   FRAMESIZE_QVGA  // QQVGA/QVGA/CIF/VGA/SVGA...
#define START_QUALITY     12              // 10(고화질,무거움) ~ 30(저화질,가벼움)
// ---------------------------------------------------------------------------

// AI-Thinker ESP32-CAM 핀맵
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define FLASH_LED_PIN      4      // 온보드 흰색 플래시 LED

static httpd_handle_t camera_httpd = NULL;
static httpd_handle_t stream_httpd = NULL;
static float g_fps = 0.0f;
static bool  g_ap_mode = false;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// --------------------------- 공통 CORS 헤더 --------------------------------
static void set_cors(httpd_req_t *req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "*");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET,OPTIONS");
}

// ------------------------------ /capture -----------------------------------
static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  set_cors(req);
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=cap.jpg");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");

  esp_err_t res = ESP_OK;
  if (fb->format == PIXFORMAT_JPEG) {
    res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  } else {
    uint8_t *jpg = NULL; size_t jpg_len = 0;
    bool ok = frame2jpg(fb, 80, &jpg, &jpg_len);
    if (ok) { res = httpd_resp_send(req, (const char *)jpg, jpg_len); free(jpg); }
    else    { res = ESP_FAIL; httpd_resp_send_500(req); }
  }
  esp_camera_fb_return(fb);
  return res;
}

// ------------------------------- /stream -----------------------------------
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  char part_buf[80];
  int64_t last = esp_timer_get_time();

  res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  set_cors(req);
  httpd_resp_set_hdr(req, "X-Framerate", "30");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }

    size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);
    res  = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, part_buf, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;

    int64_t now = esp_timer_get_time();
    float dt = (now - last) / 1000000.0f;
    last = now;
    if (dt > 0) g_fps = g_fps * 0.8f + (1.0f / dt) * 0.2f;
  }
  return res;
}

// ------------------------------- /control ----------------------------------
static esp_err_t control_handler(httpd_req_t *req) {
  char query[128] = {0};
  char var[32] = {0}, val[32] = {0};
  set_cors(req);

  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "var", var, sizeof(var)) != ESP_OK ||
      httpd_query_key_value(query, "val", val, sizeof(val)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "var/val required");
    return ESP_FAIL;
  }
  int v = atoi(val);
  sensor_t *s = esp_camera_sensor_get();
  int res = 0;

  if      (!strcmp(var, "framesize"))  res = s->set_framesize(s, (framesize_t)v);
  else if (!strcmp(var, "quality"))    res = s->set_quality(s, v);
  else if (!strcmp(var, "brightness")) res = s->set_brightness(s, v);
  else if (!strcmp(var, "contrast"))   res = s->set_contrast(s, v);
  else if (!strcmp(var, "saturation")) res = s->set_saturation(s, v);
  else if (!strcmp(var, "hmirror"))    res = s->set_hmirror(s, v);
  else if (!strcmp(var, "vflip"))      res = s->set_vflip(s, v);
  else if (!strcmp(var, "awb"))        res = s->set_whitebal(s, v);
  else if (!strcmp(var, "aec"))        res = s->set_exposure_ctrl(s, v);
  else if (!strcmp(var, "agc"))        res = s->set_gain_ctrl(s, v);
  else res = -1;

  httpd_resp_set_type(req, "application/json");
  char out[48];
  snprintf(out, sizeof(out), "{\"ok\":%s}", res == 0 ? "true" : "false");
  httpd_resp_send(req, out, strlen(out));
  return ESP_OK;
}

// -------------------------------- /flash -----------------------------------
static esp_err_t flash_handler(httpd_req_t *req) {
  char query[64] = {0}, val[16] = {0};
  set_cors(req);
  int duty = 0;
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
      httpd_query_key_value(query, "val", val, sizeof(val)) == ESP_OK) {
    duty = constrain(atoi(val), 0, 255);
  }
  // 플래시 LED는 전류를 많이 먹는다. 최대 50%로 제한(전원 안정성).
  ledcWrite(7, map(duty, 0, 255, 0, 128));
  httpd_resp_set_type(req, "application/json");
  char out[40];
  snprintf(out, sizeof(out), "{\"flash\":%d}", duty);
  httpd_resp_send(req, out, strlen(out));
  return ESP_OK;
}

// -------------------------------- /status ----------------------------------
static esp_err_t status_handler(httpd_req_t *req) {
  set_cors(req);
  httpd_resp_set_type(req, "application/json");
  sensor_t *s = esp_camera_sensor_get();
  char out[256];
  snprintf(out, sizeof(out),
           "{\"fps\":%.1f,\"framesize\":%d,\"quality\":%d,\"rssi\":%d,"
           "\"heap\":%u,\"psram\":%s,\"ap\":%s,\"ip\":\"%s\",\"up\":%lu}",
           g_fps, s->status.framesize, s->status.quality,
           g_ap_mode ? 0 : WiFi.RSSI(), (unsigned)ESP.getFreeHeap(),
           psramFound() ? "true" : "false", g_ap_mode ? "true" : "false",
           (g_ap_mode ? WiFi.softAPIP() : WiFi.localIP()).toString().c_str(),
           (unsigned long)(millis() / 1000));
  httpd_resp_send(req, out, strlen(out));
  return ESP_OK;
}

// --------------------------------- / --------------------------------------
static esp_err_t index_handler(httpd_req_t *req) {
  set_cors(req);
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  String ip = (g_ap_mode ? WiFi.softAPIP() : WiFi.localIP()).toString();
  String html =
      "<!doctype html><meta charset=utf-8><title>ROVER-CAM</title>"
      "<body style='font-family:sans-serif;background:#111;color:#eee;padding:24px'>"
      "<h2>ROVER-CAM online</h2><p>IP: <b>" + ip + "</b></p>"
      "<p>GCS의 카메라 주소 칸에 위 IP를 입력하세요.</p>"
      "<img src='http://" + ip + ":81/stream' style='max-width:100%;border:1px solid #444'>"
      "<ul><li><a style='color:#6cf' href='/capture'>/capture</a></li>"
      "<li><a style='color:#6cf' href='/status'>/status</a></li></ul></body>";
  return httpd_resp_send(req, html.c_str(), html.length());
}

// ------------------------------ 서버 시작 ----------------------------------
static void startServers() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port = 32768;
  config.max_uri_handlers = 8;

  httpd_uri_t index_uri   = {"/",        HTTP_GET, index_handler,   NULL};
  httpd_uri_t status_uri  = {"/status",  HTTP_GET, status_handler,  NULL};
  httpd_uri_t capture_uri = {"/capture", HTTP_GET, capture_handler, NULL};
  httpd_uri_t control_uri = {"/control", HTTP_GET, control_handler, NULL};
  httpd_uri_t flash_uri   = {"/flash",   HTTP_GET, flash_handler,   NULL};

  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &status_uri);
    httpd_register_uri_handler(camera_httpd, &capture_uri);
    httpd_register_uri_handler(camera_httpd, &control_uri);
    httpd_register_uri_handler(camera_httpd, &flash_uri);
  }

  // 스트림은 접속이 오래 유지되므로 별도 포트/서버로 분리한다.
  config.server_port = 81;
  config.ctrl_port = 32769;
  httpd_uri_t stream_uri = {"/stream", HTTP_GET, stream_handler, NULL};
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

// --------------------------------- setup -----------------------------------
void setup() {
  // 브라운아웃 리셋 비활성화: 전원이 약할 때 무한 재부팅을 막아준다.
  // ※ 근본 해결책이 아니라 임시 방편이다. 전원을 제대로 주는 것이 우선!
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  Serial.setDebugOutput(false);
  Serial.println();

  // 플래시 LED PWM (채널 7)
  ledcSetup(7, 5000, 8);
  ledcAttachPin(FLASH_LED_PIN, 7);
  ledcWrite(7, 0);

  camera_config_t cfg;
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0 = Y2_GPIO_NUM;   cfg.pin_d1 = Y3_GPIO_NUM;
  cfg.pin_d2 = Y4_GPIO_NUM;   cfg.pin_d3 = Y5_GPIO_NUM;
  cfg.pin_d4 = Y6_GPIO_NUM;   cfg.pin_d5 = Y7_GPIO_NUM;
  cfg.pin_d6 = Y8_GPIO_NUM;   cfg.pin_d7 = Y9_GPIO_NUM;
  cfg.pin_xclk = XCLK_GPIO_NUM;   cfg.pin_pclk = PCLK_GPIO_NUM;
  cfg.pin_vsync = VSYNC_GPIO_NUM; cfg.pin_href = HREF_GPIO_NUM;
  cfg.pin_sccb_sda = SIOD_GPIO_NUM; cfg.pin_sccb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn = PWDN_GPIO_NUM;   cfg.pin_reset = RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.frame_size   = START_FRAMESIZE;
  cfg.jpeg_quality = START_QUALITY;
  cfg.fb_count     = psramFound() ? 2 : 1;
  cfg.grab_mode    = psramFound() ? CAMERA_GRAB_LATEST : CAMERA_GRAB_WHEN_EMPTY;
  cfg.fb_location  = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) {
    Serial.printf("[CAM] init 실패 0x%x — 카메라 케이블/전원을 확인하세요\n", err);
    delay(3000);
    ESP.restart();
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, START_FRAMESIZE);
  s->set_quality(s, START_QUALITY);
  s->set_vflip(s, 0);        // 카메라를 뒤집어 달았다면 1
  s->set_hmirror(s, 0);

  // ---- Wi-Fi : 먼저 STA, 실패하면 AP ----
  WiFi.setSleep(false);                    // 전송 지연 감소 (전류는 조금 증가)
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[WiFi] 접속 시도");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < STA_TIMEOUT_MS) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    g_ap_mode = false;
    Serial.printf("[WiFi] STA 접속 완료  IP = %s  (RSSI %d)\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    g_ap_mode = true;
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("[WiFi] AP 모드  SSID=%s  PASS=%s  IP = %s\n",
                  AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
  }

  if (MDNS.begin(MDNS_NAME)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("[mDNS] http://%s.local\n", MDNS_NAME);
  }

  startServers();
  Serial.println("[HTTP] 80(제어/캡처), 81(스트림) 서버 시작");
  Serial.println("[READY] GCS에 위 IP를 입력하세요.");
}

void loop() {
  // Wi-Fi가 끊기면 재접속 시도 (AP 모드에서는 생략)
  static uint32_t last = 0;
  if (!g_ap_mode && millis() - last > 5000) {
    last = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] 재접속...");
      WiFi.reconnect();
    }
  }
  delay(100);
}
