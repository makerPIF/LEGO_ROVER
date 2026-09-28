// ESP32-CAM AP camera for SPIKE Rover GCS — AI-Thinker ESP32-CAM + OV2640
// Arduino-ESP32 3.x. Board: AI Thinker ESP32-CAM, PSRAM enabled.
// AP: ROVER-CAM / rovercam123. MJPEG: http://192.168.4.1:81/stream
// No connection to the LEGO hub is needed. Independent regulated 5V power.
#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"

static const char *AP_SSID = "LEGO_ROVER1";
static const char *AP_PASSWORD = "12345678"; // 8..63 characters
static const int AP_CHANNEL = 1; // try 1 or 11 if local interference is high
static const int TARGET_FPS = 10;
static const int JPEG_QUALITY = 14; // lower number = higher quality / more bytes
static const bool FLIP_VERTICAL = false;
static const bool MIRROR_HORIZONTAL = false;
static bool cameraReady = false;
static esp_err_t cameraError = ESP_OK;
static httpd_handle_t webServer = nullptr;
static httpd_handle_t streamServer = nullptr;

static esp_err_t indexHandler(httpd_req_t *req) {
  const char *html = R"HTML(<!doctype html><html><meta name="viewport" content="width=device-width,initial-scale=1"><title>ROVER-CAM</title><style>body{background:#101923;color:#eee;font-family:sans-serif;text-align:center}img{max-width:100%;height:auto}</style><h1>ROVER-CAM</h1><p>Close this page before connecting video in GCS. One viewer at a time.</p><img id="cam"><script>document.getElementById('cam').src='http://'+location.hostname+':81/stream';</script></html>)HTML";
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t statusHandler(httpd_req_t *req) {
  char body[160];
  snprintf(body, sizeof(body), "{\"camera_ready\":%s,\"camera_error\":%d,\"psram\":%s,\"target_fps\":%d}",
      cameraReady ? "true" : "false", (int)cameraError, psramFound() ? "true" : "false", TARGET_FPS);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t streamHandler(httpd_req_t *req) {
  if (!cameraReady) {
    httpd_resp_set_status(req, "503 Service Unavailable");
    return httpd_resp_send(req, "Camera initialization failed; check /status and Serial", HTTPD_RESP_USE_STRLEN);
  }
  httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=roverframe");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  esp_err_t result = ESP_OK;
  while (true) {
    uint32_t started = millis();
    camera_fb_t *frame = esp_camera_fb_get();
    if (!frame) return ESP_FAIL;
    if (frame->format != PIXFORMAT_JPEG) {
      esp_camera_fb_return(frame);
      return ESP_FAIL;
    }
    char header[128];
    int count = snprintf(header, sizeof(header),
        "\r\n--roverframe\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", (unsigned)frame->len);
    if (count <= 0 || count >= (int)sizeof(header)) {
      esp_camera_fb_return(frame);
      return ESP_FAIL;
    }
    result = httpd_resp_send_chunk(req, header, count);
    if (result == ESP_OK) result = httpd_resp_send_chunk(req, (const char *)frame->buf, frame->len);
    esp_camera_fb_return(frame); // return buffer on success AND failure
    if (result != ESP_OK) break;
    uint32_t elapsed = millis() - started;
    uint32_t interval = 1000 / TARGET_FPS;
    delay(elapsed < interval ? interval - elapsed : 1);
  }
  return result;
}

static bool startServers() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.stack_size = 8192;
  config.max_uri_handlers = 4;
  config.lru_purge_enable = true;
  config.send_wait_timeout = 2;
  config.recv_wait_timeout = 2;
  if (httpd_start(&webServer, &config) != ESP_OK) return false;
  httpd_uri_t root = {};
  root.uri = "/"; root.method = HTTP_GET; root.handler = indexHandler;
  httpd_uri_t status = {};
  status.uri = "/status"; status.method = HTTP_GET; status.handler = statusHandler;
  if (httpd_register_uri_handler(webServer, &root) != ESP_OK ||
      httpd_register_uri_handler(webServer, &status) != ESP_OK) return false;
  config.server_port = 81;
  config.ctrl_port += 1;
  config.max_open_sockets = 2;
  if (httpd_start(&streamServer, &config) != ESP_OK) return false;
  httpd_uri_t stream = {};
  stream.uri = "/stream"; stream.method = HTTP_GET; stream.handler = streamHandler;
  return httpd_register_uri_handler(streamServer, &stream) == ESP_OK;
}

void setup() {
  Serial.begin(115200);
  pinMode(4, OUTPUT); digitalWrite(4, LOW); // white flash LED off; SD card not used
  WiFi.mode(WIFI_AP);
  IPAddress address(192,168,4,1), mask(255,255,255,0);
  if (!WiFi.softAPConfig(address, address, mask) ||
      !WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, false, 2)) {
    Serial.println("AP start failed");
    return;
  }
  WiFi.setSleep(false);
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  // AI-Thinker pin mapping: D0..D7
  c.pin_d0 = 5; c.pin_d1 = 18; c.pin_d2 = 19; c.pin_d3 = 21;
  c.pin_d4 = 36; c.pin_d5 = 39; c.pin_d6 = 34; c.pin_d7 = 35;
  c.pin_xclk = 0; c.pin_pclk = 22; c.pin_vsync = 25; c.pin_href = 23;
  c.pin_sccb_sda = 26; c.pin_sccb_scl = 27;
  c.pin_pwdn = 32; c.pin_reset = -1;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = psramFound() ? FRAMESIZE_VGA : FRAMESIZE_QVGA;
  c.jpeg_quality = JPEG_QUALITY;
  c.fb_count = psramFound() ? 2 : 1;
  c.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  c.grab_mode = psramFound() ? CAMERA_GRAB_LATEST : CAMERA_GRAB_WHEN_EMPTY;
  cameraError = esp_camera_init(&c);
  cameraReady = cameraError == ESP_OK;
  if (cameraReady) {
    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor) {
      sensor->set_vflip(sensor, FLIP_VERTICAL ? 1 : 0);
      sensor->set_hmirror(sensor, MIRROR_HORIZONTAL ? 1 : 0);
    }
  } else {
    Serial.printf("Camera init failed: 0x%x\n", cameraError);
  }
  if (!startServers()) Serial.println("HTTP server start failed");
  Serial.printf("AP: %s\nIP: %s\nVideo: http://192.168.4.1:81/stream\n", AP_SSID, WiFi.softAPIP().toString().c_str());
}
void loop() { delay(1000); }
