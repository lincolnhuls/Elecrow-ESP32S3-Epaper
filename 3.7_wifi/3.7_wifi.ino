// Flash settings for this board are fairly specific
// 
// Board: ESP32S3 Dev Module 
// Flash Size: 8MB (64Mb)
// Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"
// PSRAM: "OPI PSRAM"
// Upload Speed: "460800" (Board will not upload if this is not set) 


// Import needed libraries
#include <PubSubClient.h>
#include <FastLED.h>
#include <QRCodeGenerator.h>
#include <WiFiManager.h>
#include <Arduino.h>
#include "EPD.h"
#include "EPD_GUI.h"
#include <WiFi.h>
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <Preferences.h>

// Declare the WiFiManager and Preferences(non-volitile storage) to use in code
WiFiManager wifiManager;
Preferences prefs;

// Flag for register state
bool registered = false;
bool registeringScreenShown = false;
bool connectedScreenShown = false;

// Led helpers
bool ledsActive = false;
uint32_t ledsOffAtMs = 0;
uint8_t lastBrightness = 255;

// Blink Aids
bool blinkActive = false;
bool blinkIsOn = false;
uint32_t blinkNextToggleMs = 0;

uint16_t blinkIntervalMs = 0;
uint16_t blinkRemaining = 0;

uint8_t blinkR, blinkG, blinkB;
uint8_t blinkBrightness = 255;
uint16_t blinkStart, blinkCount;

// Breate Aids
bool breatheActive = false;
uint32_t breatheStartMs = 0;
uint32_t breatheHalfPeriodMs = 1000;

uint8_t breatheR, breatheG, breatheB;
uint8_t breatheMaxBrightness = 255;
uint16_t breatheStart, breatheCount;

uint32_t breatheNextFrameMs = 0;
uint32_t breatheLastFrameMs = 0;
const uint16_t BREATHE_FRAME_MS = 15; // ~66 FPS (try 10–20)

uint16_t breathePhase16 = 0;
uint16_t breathePhaseStep = 0;
uint8_t lastBreatheBrightness = 255;


// Timers to help with state transitions
unsigned long lastMqttAttemptMs = 0;
unsigned long lastRegisterAttemptMs = 0;

// Timers for heartbeat
unsigned long lastHeartbeatMs = 0;
const unsigned long HEARTBEAT_PERIOD_MS = 30000;

// Declare strings
String SERVICE_UUID = "";
String ReportTopic;
String SubscribeTopic;
String HeartbeatTopic;
String apName;
String wifiID;

// Mqtt topics
const char *reportTopic ;
const char *subscribeTopic ;
const char *heartbeatTopic ;

// Setup WiFi Client
WiFiClient      Wifi_net;

// Led setup
#define NUM_LEDS 100
#define DATA_PIN 8
CRGB leds[NUM_LEDS];

// 240x416 buffer
uint8_t ImageBW[12480];
char buffer[200];

// Forward declarations
int rssiToBars(long rssi);
void displayQRCodeOnEPD(const char* text);
void drawWiFiIcon(int x, int y, int bars);

// Declare space for chipid and shortId
uint64_t chipid;
char chipIdStr[32];
char shortId[7];

// callback helpers
volatile bool mqttMsgReady = false;
char mqttTopicBuff[256];
char mqttPayloadBuff[1024];

// Declare space for secret from preferences
String secret;

///////////////// MQTT Broker Setup //////////////////////////
const char* mqttServer = "mqttbroker.tetontechnology.com";        
const char* brokerName = "fuelbroker";
const char* brokerPassword = "N3tJPFTHYYNcsHw";

////////////////////////////////////// Led Functions //////////////////////////////////////

void on(uint8_t red=255, uint8_t green=255, uint8_t blue=255, uint16_t duration_s=-1, uint8_t brightness=255, uint16_t start_index=0, uint16_t indexCount=1) {
  lastBrightness = brightness;
  FastLED.setBrightness(brightness);

  for (int i = 0; i < indexCount && (start_index + i) < NUM_LEDS; i++) {
    int idx = start_index + i;
    leds[idx].setRGB(red, green, blue); 
  }
  FastLED.show();

  ledsActive = true;

  if (duration_s < 0) {
    ledsOffAtMs = 0;
  } else {
    ledsOffAtMs = millis() + (uint32_t)duration_s * 1000UL;
  }
}

void blink(uint8_t red=255, uint8_t green=255, uint8_t blue=255, uint16_t interval_ms=500, uint8_t times=0, uint8_t brightness=255, uint16_t start_index=0, uint16_t indexCount=1) {
  blinkR = red;
  blinkG = green;
  blinkB = blue;
  blinkBrightness = brightness;
  blinkStart = start_index;
  blinkCount = indexCount;

  blinkIntervalMs = interval_ms;
  blinkRemaining = (times == 0) ? 0 : times * 2;

  blinkIsOn = false;
  blinkActive = true;
  blinkNextToggleMs = millis();
}

void breathe(uint8_t red=255, uint8_t green=255, uint8_t blue=255, uint16_t duration_s=1, 
uint8_t brightness=255, uint16_t start_index=0, uint16_t indexCount=1) {

  breatheR = red;
  breatheG = green;
  breatheB = blue;

  breatheMaxBrightness = brightness;
  breatheStart = start_index;
  breatheCount = indexCount;

  breatheHalfPeriodMs = (uint32_t)duration_s * 1000UL;

  uint32_t cycleMs = breatheHalfPeriodMs * 2UL;
  if (cycleMs == 0) cycleMs = 1;

  breathePhaseStep = (uint16_t)((65536UL * BREATHE_FRAME_MS) / cycleMs);
  if (breathePhaseStep == 0) breathePhaseStep = 1;

  breathePhase16 = (uint16_t)(64 << 8); // sin8(64) ~= 255 (start bright)
  breatheLastFrameMs = millis();
  lastBreatheBrightness = 255; // ok to keep, but optional

  breatheActive = true;

  // paint pixels once; ledTick modulates brightness
  on(breatheR, breatheG, breatheB, -1, breatheMaxBrightness, breatheStart, breatheCount);
}

void off(uint16_t start_index = 0, uint16_t count = NUM_LEDS) {
  for (uint16_t i = start_index; i < start_index + count && i < NUM_LEDS; i++) {
    leds[i] = CRGB::Black;
  }
  FastLED.show();

  ledsActive = false;   
  ledsOffAtMs = 0;     
}


// Helper for non-blocking
void ledTick() {
  uint32_t now = millis();

  // 1) Blink scheduler (highest priority)
  if (blinkActive && (int32_t)(now - blinkNextToggleMs) >= 0) {
    blinkNextToggleMs = now + blinkIntervalMs;

    if (blinkIsOn) {
      off(blinkStart, blinkCount);
    } else {
      on(blinkR, blinkG, blinkB,
         -1,               // blink controls timing
         blinkBrightness,
         blinkStart,
         blinkCount);
    }

    blinkIsOn = !blinkIsOn;

    if (blinkRemaining > 0) {
      blinkRemaining--;
      if (blinkRemaining == 0) {
        blinkActive = false;
        off(blinkStart, blinkCount);
      }
    }
    return; // prevent other schedulers from fighting this cycle
  }

  // 2) Breathe scheduler (next priority)
  if (breatheActive) {
    uint32_t elapsed = now - breatheLastFrameMs;
    if (elapsed < BREATHE_FRAME_MS) return;

    // number of frames to advance; cap catch-up to prevent jumps
    uint32_t frames = elapsed / BREATHE_FRAME_MS;
    if (frames > 3) frames = 3;

    breatheLastFrameMs += frames * BREATHE_FRAME_MS;
    breathePhase16 += (uint16_t)(frames * breathePhaseStep);

    // use high byte as 0..255 phase for sin8
    uint8_t phase8 = (uint8_t)(breathePhase16 >> 8);
    uint8_t wave = sin8(phase8); // smooth 0..255..0..255

    uint8_t curBrightness = (uint8_t)((uint32_t)breatheMaxBrightness * wave / 255UL);

    if (curBrightness != lastBreatheBrightness) {
      lastBreatheBrightness = curBrightness;
      FastLED.setBrightness(curBrightness);
      FastLED.show();
    }
    return;
  }

  // 3) Auto-off scheduler (for plain on(duration))
  if (ledsActive && ledsOffAtMs != 0 && (int32_t)(now - ledsOffAtMs) >= 0) {
    off();
  }
}

void stopLedEffects() {
  blinkActive = false;
  breatheActive = false;
  // (future effects go here)
}
///////////////////////////////////////////////////////////////////////////////////////////


// Wifi callback for digesting commands over Mqtt
void WiFi_callback(char* topic, byte* payload, unsigned int length) {
  // Copy topic
  strncpy(mqttTopicBuff, topic, sizeof(mqttTopicBuff));
  mqttTopicBuff[sizeof(mqttTopicBuff) - 1] = '\0';

  // Copy payload
  size_t n = (length < sizeof(mqttPayloadBuff) - 1) ? length : (sizeof(mqttPayloadBuff) - 1);
  memcpy(mqttPayloadBuff, payload, n);
  mqttPayloadBuff[n] = '\0';

  mqttMsgReady = true;
}

void handleMqttMessage() {
  if (!mqttMsgReady) return;
  mqttMsgReady = false;

  if (strcmp(mqttTopicBuff, subscribeTopic) != 0) return;

  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, mqttPayloadBuff);
  if (err) {
    Serial.print("MQTT JSON parse failed: ");
    Serial.println(err.c_str());
    return;
  }

  // If not registered accept the reply for the secret and store
  if (!registered) {
    const char* s = doc["secret"] | "";
    if (s[0] == '\0') return;

    prefs.begin("secret", false);
    prefs.putString("secret", s);
    prefs.end();

    secret = s;
    registered = true;
    connectedScreenShown = false;

    Serial.println("Registered, secret saved");
    return;
  }

  // If registered, accept full range of cmds
  const char* incomingSecret = doc["secret"] | "";
  if (incomingSecret[0] == '\0' || secret.length() == 0) {
    Serial.println("Rejected command: Missing secret");
    return;
  }

  if (secret != incomingSecret) {
    Serial.println("Rejected command: Bad secret");
    return;
  }

  const char* cmd = doc["cmd"] | "";
  if (cmd[0] == '\0') {
    Serial.println("No cmd provided");
    return;
  }

  if (strcmp(cmd, "deregister") == 0) {
    deregister();
    return;
  }

  if (strcmp(cmd, "test") == 0) {
    Serial.println("Recieved testing command");
    return;
  }

  if (strcmp(cmd, "on") == 0) {
    stopLedEffects();
    uint8_t r = doc["r"] | 255;
    uint8_t g = doc["g"] | 255;
    uint8_t b = doc["b"] | 255;

    uint8_t brightness = doc["brightness"] | 255;
    int16_t duration_s = doc["duration"] | -1;

    uint16_t start = doc["start"] | 0;
    uint16_t count = doc["count"] | NUM_LEDS;

    on(r, g, b, duration_s, brightness, start, count);
    return;
  }

  if (strcmp(cmd, "blink") == 0) {
    stopLedEffects();
    uint8_t r = doc["r"] | 255;
    uint8_t g = doc["g"] | 255;
    uint8_t b = doc["b"] | 255;

    uint16_t interval_ms = doc["interval"] | 500;   
    uint8_t times = doc["times"] | 0;              

    uint8_t brightness = doc["brightness"] | 255;
    uint16_t start = doc["start"] | 0;
    uint16_t count = doc["count"] | NUM_LEDS;

    blink(r, g, b, interval_ms, times, brightness, start, count);
          
    return;
  }

  if (strcmp(cmd, "breathe") == 0) {
    stopLedEffects();
    uint8_t r = doc["r"] | 255;
    uint8_t g = doc["g"] | 255;
    uint8_t b = doc["b"] | 255;

    int16_t duration_s = doc["duration"] | 1;   // HALF cycle seconds
    uint8_t brightness = doc["brightness"] | 255;

    uint16_t start = doc["start"] | 0;
    uint16_t count = doc["count"] | NUM_LEDS;

    breathe(r, g, b, duration_s, brightness, start, count);
    return;
  }

  if (strcmp(cmd, "off") == 0) {
    stopLedEffects();
    blinkActive = false;
    uint16_t start = doc["start"] | 0;
    uint16_t count = doc["count"] | NUM_LEDS;

    off(start, count);
    Serial.println("Executed OFF command");
  }

  Serial.print("Unknown cmd: ");
  Serial.println(cmd);
} 

// Set up Mqtt Client 
PubSubClient    mqttClient(mqttServer, 1883, WiFi_callback, Wifi_net);

void mqttConnectAttempt() {
  if (mqttClient.connected()) return;

  if (millis() - lastMqttAttemptMs < 5000) return;
  lastMqttAttemptMs = millis();

  Serial.println("Attempting MQTT connect...");
  if (mqttClient.connect(wifiID.c_str(), brokerName, brokerPassword)) {
    mqttClient.subscribe(subscribeTopic);
    mqttClient.subscribe(heartbeatTopic);
    Serial.println("MQTT connected + subscribed");
  } else {
    Serial.println("MQTT connected failed");
  }
}
void deregister() {
  Serial.println("Deregister command recieved");

  prefs.begin("secret", false);
  prefs.remove("secret");
  prefs.end();

  secret = "";
  registered = false;

  connectedScreenShown = false;
  registeringScreenShown = false;

  mqttClient.disconnect();

  showRegisteringScreen();
}


void registerAttempt() {
  if (registered) return;

  if (!mqttClient.connected()) return;

  if (millis() - lastRegisterAttemptMs < 3000) return;
  lastRegisterAttemptMs = millis();

  bool ok = requestRegister();
  Serial.print("Register Publish: ");
  Serial.println(ok ? "ok" : "failed");
}

// Function to call once a device has wifi and mqtt connection for registering it with the organization
bool requestRegister() {
  if (!mqttClient.connected()) return false;
  
  StaticJsonDocument<128> doc;
  doc["chipId"] = SERVICE_UUID;
  doc["code"] = shortId;

  char out[128];
  size_t n = serializeJson(doc, out, sizeof(out));

  return mqttClient.publish(reportTopic, out, n);
}

void heartbeat() {
  if (!registered) return;

  if (!mqttClient.connected()) return;

  if (millis() - lastHeartbeatMs < HEARTBEAT_PERIOD_MS) return;
  lastHeartbeatMs = millis();

  StaticJsonDocument<256> doc;
  doc["secret"] = secret;
  doc["code"] = shortId;
  doc["uuid"] = SERVICE_UUID;
  doc["ssid"] = WiFi.SSID().c_str();
  doc["ip"] = WiFi.localIP().toString();
  doc["rssi"] = WiFi.RSSI();

  char out[256];
  size_t n = serializeJson(doc, out, sizeof(out));

  bool ok = mqttClient.publish(heartbeatTopic, out, n);
  Serial.print("Heartbeat publish: ");
  Serial.println(ok ? "ok" : "failed");
}

// WiFi setup screen
void showWifiSetupScreen(const char* apName) {
  EPD_FastInit();
  EPD_Display_Clear();

  Paint_NewImage(ImageBW, EPD_W, EPD_H, 0, WHITE);
  Paint_Clear(WHITE);

  // Left side
  EPD_ShowString(10, 10,  "WIFI SETUP REQUIRED", 16, BLACK);
  EPD_ShowString(10, 30,  "Connect to AP:", 16, BLACK);
  EPD_ShowString(10, 50,  apName, 16, BLACK);
  EPD_ShowString(10, 75,  "Or scan QR to open", 16, BLACK);
  EPD_ShowString(10, 95,  "setup portal", 16, BLACK);
  EPD_ShowString(10, 125, "URL: 192.168.4.1", 16, BLACK);

  // Right side QR to portal
  displayQRCodeOnEPD("http://192.168.4.1");

  EPD_Display(ImageBW);
  EPD_Update();
}

// Stage two screen while connecting to mqtt and getting registered
void showRegisteringScreen() {
  EPD_FastInit();
  EPD_Display_Clear();

  Paint_NewImage(ImageBW, EPD_W, EPD_H, 0, WHITE);
  Paint_Clear(WHITE);

  EPD_ShowString(10, 10,  "WIFI CONNECTED", 16, BLACK);
  EPD_ShowString(10, 30,  "Registering device...", 16, BLACK);

  snprintf(buffer, sizeof(buffer), "Code: %s", shortId);
  EPD_ShowString(10, 55, buffer, 16, BLACK);

  EPD_ShowString(10, 80,  "Connecting MQTT...", 16, BLACK);
  EPD_ShowString(10, 100, "Waiting for secret", 16, BLACK);

  displayQRCodeOnEPD(shortId);

  EPD_Display(ImageBW);
  EPD_Update();
}


// ----------------------------------------------------
// SETUP
// ----------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
  chipid = ESP.getEfuseMac();

  // Create small code for wh connection
  snprintf(shortId, sizeof(shortId), "%06X", (uint32_t)(chipid & 0xFFFFFF));

  // Build service uuid using chip id
  SERVICE_UUID = "mqtt" + (String)chipid;

  // Make readable chip id string
  sprintf(chipIdStr, "%04X%08X",
          (uint16_t)(chipid >> 32),
          (uint32_t)chipid);
 

  
  apName = "Cart Setup ";
  apName += shortId;

  // Reset settings for testing
  // wifiManager.resetSettings();

  FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);

  // ----- EPD INIT -----
  EPD_GPIOInit();
  EPD_Init();

  // Get wifi id
  wifiID = shortId;

  Paint_NewImage(ImageBW, EPD_W, EPD_H, 0, WHITE);
  Paint_Clear(WHITE);
  
  // True means that namespace can be read but not written to, false is read and write
  // Ready from prefs to find if a secret key is stored
  prefs.begin("secret", true);
  secret = prefs.getString("secret", "");
  prefs.end();

  // If the secret is not empty then we know that the device has been registered
  if (secret != "") {
    registered = true;
  } 
  // If not we need to go through the register process
  else {
    registered = false;
  }

  // Setup the mqttClient
  mqttClient.setServer(mqttServer, 1883);
  mqttClient.setCallback(WiFi_callback);

  /////////////////// Setup MQTT topics ///////////////////////
  ReportTopic = "wh/register/" + SERVICE_UUID;
  SubscribeTopic = "wh/device/" + SERVICE_UUID;
  HeartbeatTopic = SubscribeTopic + "/heartbeat";

  reportTopic = ReportTopic.c_str();
  subscribeTopic = SubscribeTopic.c_str();
  heartbeatTopic = HeartbeatTopic.c_str();
  ////////////////////////////////////////////////////////////

  // BOOT SCREEN
  EPD_ShowString(10, 10, "Connecting to WiFi...", 16, BLACK);
  EPD_Display(ImageBW);
  EPD_Update();

  delay(1500); // allow panel to settle

  // ----- WIFI INIT (NON-BLOCKING) -----
  WiFi.mode(WIFI_STA);
  WiFi.begin();

  wifiManager.setConfigPortalTimeout(180);

  // Try quick reconnect first
  wifiManager.setConfigPortalTimeout(5);
  bool ok = wifiManager.autoConnect(apName.c_str());

  if (!ok) {
    // Show setup screen before starting the portal to ensure display
    showWifiSetupScreen(apName.c_str());

    wifiManager.setConfigPortalTimeout(180);
    while (!wifiManager.autoConnect(apName.c_str())) {
      // Loop until connected
    }
  }
}

// ----------------------------------------------------
// LOOP 
// ----------------------------------------------------
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectedScreenShown = false;
    registeringScreenShown = false;

    wifiManager.setConfigPortalTimeout(5);
    bool ok = wifiManager.autoConnect(apName.c_str());
    if (ok) return;

    showWifiSetupScreen(apName.c_str());

    wifiManager.setConfigPortalTimeout(180);
    while (!wifiManager.autoConnect(apName.c_str())) {}
    return;
  }
  
  mqttClient.loop();
  handleMqttMessage();

  mqttConnectAttempt();
  heartbeat();

  ledTick();

  if (!registered) {
    if (!registeringScreenShown) {
      registeringScreenShown = true;
      connectedScreenShown = false;
      showRegisteringScreen();
    }

    registerAttempt();
    return;
  }

  if (!connectedScreenShown) {
    connectedScreenShown = true;
    registeringScreenShown = false;
    showConnectedDashboard();
  }
}

// ----------------------------------------------------
// CONNECTED DASHBOARD
// ----------------------------------------------------
void showConnectedDashboard() {
  EPD_FastInit();
  EPD_Display_Clear();

  Paint_NewImage(ImageBW, EPD_W, EPD_H, 0, WHITE);
  Paint_Clear(WHITE);

  // Title
  EPD_ShowString(10, 10, "CONNECTED", 16, BLACK);

  // SSID
  snprintf(buffer, sizeof(buffer), "SSID: %s", WiFi.SSID().c_str());
  EPD_ShowString(10, 30, buffer, 16, BLACK);

  // IP Address
  snprintf(buffer, sizeof(buffer), "IP: %s", WiFi.localIP().toString().c_str());
  EPD_ShowString(10, 50, buffer, 16, BLACK);

  // RSSI Value
  long rssi = WiFi.RSSI();
  snprintf(buffer, sizeof(buffer), "Signal: %ld dBm", rssi);
  EPD_ShowString(10, 70, buffer, 16, BLACK);

  // Signal Bars
  int bars = rssiToBars(rssi);
  drawWiFiIcon(10, 95, bars);   // under RSSI text

  // Chip ID
  snprintf(buffer, sizeof(buffer), "ChipID: %s", SERVICE_UUID.c_str());
  EPD_ShowString(10, 140, buffer, 16, BLACK);

  // QR Code (right side)
  displayQRCodeOnEPD(SERVICE_UUID.c_str());

  EPD_Display(ImageBW);
  EPD_Update();
  EPD_DeepSleep();
}

// ----------------------------------------------------
// QR CODE
// ----------------------------------------------------
void displayQRCodeOnEPD(const char* text) {
  QRCode qrcode;
  uint8_t qrcodeData[qrcode_getBufferSize(4)];
  qrcode_initText(&qrcode, qrcodeData, 4, 0, text);

  const int scale = 5;
  int qrPixSize = qrcode.size * scale;
  int displayWidth = Paint.width;

  const int pxOffset = displayWidth - qrPixSize - 12;
  const int pyOffset = 40;

  for (uint8_t y = 0; y < qrcode.size; y++) {
    for (uint8_t x = 0; x < qrcode.size; x++) {
      bool module = qrcode_getModule(&qrcode, x, y);
      for (int dx = 0; dx < scale; dx++) {
        for (int dy = 0; dy < scale; dy++) {
          Paint_SetPixel(pxOffset + x * scale + dx,
                         pyOffset + y * scale + dy,
                         module ? BLACK : WHITE);
        }
      }
    }
  }
}


// ----------------------------------------------------
// RSSI → SIGNAL BARS
// ----------------------------------------------------
int rssiToBars(long rssi) {
  if (rssi >= -60) return 4;   // Excellent
  if (rssi >= -70) return 3;   // Good
  if (rssi >= -80) return 2;   // Fair
  if (rssi >= -90) return 1;   // Weak
  return 0;                    // No signal
}

void drawWiFiIcon(int x, int y, int bars) {
  const int barWidth = 4;
  const int gap = 3;

  for (int i = 0; i < 4; i++) {
    int barXStart = x + i * (barWidth + gap);
    int barXEnd   = barXStart + barWidth;
    int barHeight = 6 * (i + 1);      // 6, 12, 18, 24 px

    int barYStart = y + (24 - barHeight);
    int barYEnd   = y + 24;

    // 1) Always draw OUTLINE in BLACK (hollow bar)
    EPD_DrawRectangle(barXStart, barYStart,
                      barXEnd,   barYEnd,
                      BLACK, 0);   // 0 = outline only

    // 2) If this bar is "active", fill INSIDE the outline
    if (i < bars) {
      // Slightly inset rectangle so we don't overwrite the outline
      EPD_DrawRectangle(barXStart + 1, barYStart + 1,
                        barXEnd   - 1, barYEnd   - 1,
                        BLACK, 1);   // 1 = filled
    }
  }
}


