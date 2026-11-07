/*
 * Water monitor server (ESP32, no microphone needed)
 * Receives messages from the sensor board via HTTP POST /event
 * and shows the state on a web page.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

const char* WIFI_SSID = "pf19wlan";
const char* WIFI_PASS = "Silupada2025!";
const char* TOKEN     = "change-me";              // gleich wie im Sensor

// Optional: feste IP (sonst im Router reservieren)
// IPAddress ip(192,168,1,50), gw(192,168,1,1), mask(255,255,255,0), dns(192,168,1,1);

WebServer server(80);

struct EventEntry { uint32_t ms; char text[32]; };
static EventEntry events[10];
static int eventCount = 0;

static bool     flowing = false, alarmActive = false;
static uint32_t flowStartMs = 0, lastMsgMs = 0;

static void addEvent(const char* text) {
  EventEntry &e = events[eventCount % 10];
  e.ms = millis();
  strncpy(e.text, text, sizeof(e.text) - 1);
  e.text[sizeof(e.text) - 1] = 0;
  eventCount++;
}

static void handleEvent() {
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "POST only");
    return;
  }
  if (server.header("X-Token") != TOKEN) {
    server.send(401, "text/plain", "unauthorized");
    return;
  }
  String body = server.arg("plain");
  body.trim();
  if (body.length() == 0) {
    server.send(400, "text/plain", "empty message");
    return;
  }

  uint32_t now = millis();
  lastMsgMs = now;
  if (body == "Alarm") {
    if (!flowing) { flowing = true; flowStartMs = now; }
    alarmActive = true;
  } else if (body == "Water is flowing") {
    flowing = true;
    alarmActive = false;
    flowStartMs = now;
  } else if (body == "Water stopped") {
    flowing = false;
    alarmActive = false;
  }
  addEvent(body.c_str());
  Serial.printf("[web] received: %s\n", body.c_str());
  server.send(200, "text/plain", "OK");
}

static void handleRoot() {
  uint32_t now = millis();
  String html;
  html.reserve(1500);
  html += F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<meta http-equiv='refresh' content='5'>"
            "<title>Water monitor</title>"
            "<style>body{font-family:sans-serif;max-width:480px;margin:20px auto;padding:0 12px}"
            ".s{padding:14px;border-radius:8px;color:#fff;font-size:1.3em}"
            ".ok{background:#2e7d32}.fl{background:#ef6c00}.al{background:#c62828}</style>"
            "</head><body><h2>Water monitor</h2>");
  if (alarmActive)  html += F("<div class='s al'>ALARM: water running too long</div>");
  else if (flowing) html += F("<div class='s fl'>Water is flowing</div>");
  else              html += F("<div class='s ok'>No flow</div>");
  if (flowing) html += "<p>Flowing for " + String((now - flowStartMs) / 1000) + " s</p>";
  if (eventCount > 0) html += "<p>Last message " + String((now - lastMsgMs) / 1000) + " s ago</p>";
  html += F("<h3>Events</h3><ul>");
  int n = eventCount < 10 ? eventCount : 10;
  for (int i = 0; i < n; i++) {
    const EventEntry &e = events[(eventCount - 1 - i) % 10];   // neueste zuerst
    html += "<li>" + String(e.text) + " (" + String((now - e.ms) / 1000) + " s ago)</li>";
  }
  if (n == 0) html += F("<li>none yet</li>");
  html += F("</ul><p><a href='/status'>JSON</a></p></body></html>");
  server.send(200, "text/html", html);
}

static void handleStatus() {
  String j = "{\"flowing\":";
  j += flowing ? "true" : "false";
  j += ",\"alarm\":";
  j += alarmActive ? "true" : "false";
  j += ",\"flow_seconds\":" + String(flowing ? (millis() - flowStartMs) / 1000 : 0);
  j += "}";
  server.send(200, "application/json", j);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  // WiFi.config(ip, gw, mask, dns);          // nur für feste IP
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\nServer: http://%s/\n", WiFi.localIP().toString().c_str());

  if (MDNS.begin("wassersensor")) MDNS.addService("http", "tcp", 80);

  const char* hdrs[] = {"X-Token"};
  server.collectHeaders(hdrs, 1);             // vor begin() aufrufen
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/event", handleEvent);
  server.begin();
}

void loop() {
  server.handleClient();
} 