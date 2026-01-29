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
#include <EEPROM.h>
#include <Preferences.h>

// Declare the WiFiManager and Preferences(non-volitile storage) to use in code
WiFiManager wifiManager;
Preferences prefs;

// Flag for it registering needs to happen
bool registered = false;

// Declare strings
String SERVICE_UUID = "";
String ReportTopic;
String SubscribeTopic;
String WiFi_ID;
String str = "test";

// Mqtt topics
const char *wifiID;
const char *reportTopic ;
const char *subscribeTopic ;

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
void drawWifiIcon();

// Declare space for chipid and shortId
uint64_t chipid;
char chipIdStr[32];
char shortId[7];

// callback helpers
volatile bool mqttMsgReady = false;
char mqttTopicBuff[128];
char mqttPayloadBuff[256];

// Declare space for secret from preferences
String secret;

///////////////// MQTT Broker Setup //////////////////////////
const char* mqttServer = "mqttbroker.tetontechnology.com";        
const char* brokerName = "fuelbroker";
const char* brokerPassword = "N3tJPFTHYYNcsHw";

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

  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, mqttPayloadBuff);
  if (err) {
    Serial.print("MQTT JSON parse failed: ");
    Serial.println(err.c_str());
    return;
  }

  const char* s = doc["secret"] | "";
  if (s[0] == '\0') return;

  prefs.begin("secret", false);
  prefs.putString("secret", s);
  prefs.end();

  secret = s;
  registered = true;

  Serial.println("Registered, secret saved");
} 

// Set up Mqtt Client 
PubSubClient    mqttClient(mqttServer, 1883, WiFi_callback, Wifi_net);

// Initial mqtt connect function
bool mqttConnect() {
  mqttClient.setServer(mqttServer, 1883);
  mqttClient.setCallback(WiFi_callback);
  while (!mqttClient.connected()) {
    if (mqttClient.connect(wifiID, brokerName, brokerPassword)) {
      mqttClient.subscribe(subscribeTopic);
      Serial.print("Connected to MQTT on topic ");
      Serial.println(SubscribeTopic);
      mqttClient.publish(reportTopic, shortId);
      return true;
    } 
    else {
      delay(5000);
      Serial.println("Failed to connect on MQTT");
    }
  }
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
 
  // wifiManager.resetSettings();

  FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);

  // ----- EPD INIT -----
  EPD_GPIOInit();
  EPD_Init();

  // Get wifi id
  wifiID = WiFi_ID.c_str();

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

  /////////////////// Setup MQTT topics ///////////////////////
  ReportTopic = "wh/register/" + SERVICE_UUID;
  SubscribeTopic = "wh/device/" + SERVICE_UUID;

  reportTopic = ReportTopic.c_str();
  subscribeTopic = SubscribeTopic.c_str();
  ////////////////////////////////////////////////////////////

  // prefs.begin("secret", false);
  // prefs.putString("chipId", SERVICE_UUID);
  // prefs.end();

  // Serial.println(id);
  // delay(5000);

  // BOOT SCREEN
  EPD_ShowString(10, 10, "Connecting to WiFi...", 16, BLACK);
  EPD_Display(ImageBW);
  EPD_Update();

  delay(1500); // allow panel to settle

  // ----- WIFI INIT (NON-BLOCKING) -----
  WiFi.mode(WIFI_STA);
  WiFi.begin();

  wifiManager.setConfigPortalTimeout(5);

  bool result = wifiManager.autoConnect("Cart Setup");
  Serial.println(result);

  if (result == 0) {
    Serial.println("Showing setup screen");
    showPortalScreen();
  }

  wifiManager.setConfigPortalTimeout(180);
  while (result == 0) {
    result = wifiManager.autoConnect("Cart Setup");
  }
  
  // If not registered, request register
  if (!registered) {
    if (mqttConnect()) {
      requestRegister();
    }
  }
  // If registered, show connected dashboard
  if (registered) {
    if (mqttConnect()) {
      showConnectedDashboard();
    }
  }
}

// ----------------------------------------------------
// LOOP (FULLY NON-BLOCKING)
// ----------------------------------------------------
void loop() {
  ///////////////////// WiFi reconnect on disconnect ///////////////////////////
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Lost Connection");
    showLostConnectionScreen();
    int result = 0;
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::Green;
    }
    FastLED.show();
    while (result == 0) {
      result = wifiManager.autoConnect("Cart Setup");
    }
  }
  /////////////////////////////////////////////////////////////////////////////

  ///////////////////// Mqtt reconnect on disconnect //////////////////////////
  if (!mqttClient.connected()) {
    mqttConnect();
  }
  /////////////////////////////////////////////////////////////////////////////

  mqttClient.loop();
  handleMqttMessage();

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
  EPD_ShowString(10, 10, "WiFi CONNECTED", 16, BLACK);

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
  snprintf(buffer, sizeof(buffer), "ChipID: %s", chipIdStr);
  EPD_ShowString(10, 140, buffer, 16, BLACK);

  // QR Code (right side)
  displayQRCodeOnEPD(chipIdStr);

  EPD_Display(ImageBW);
  EPD_Update();
  EPD_DeepSleep();
}



// ----------------------------------------------------
// PORTAL SCREEN
// ----------------------------------------------------
void showPortalScreen() {
  EPD_FastInit();
  EPD_Display_Clear();

  Paint_NewImage(ImageBW, EPD_W, EPD_H, 0, WHITE);  // Create a new white background image.
  Paint_Clear(WHITE);  // Clear the canvas.

  EPD_ShowString(10, 10, "WIFI SETUP REQUIRED", 16, BLACK);
  EPD_ShowString(10, 30, "Connect to WiFi:", 16, BLACK);
  EPD_ShowString(10, 50, "Cart Setup", 16, BLACK);

  EPD_Display(ImageBW);
  EPD_Update();
}

// ----------------------------------------------------
// LOST CONNETION SCREEN
// ----------------------------------------------------
void showLostConnectionScreen() {
  EPD_FastInit();
  EPD_Display_Clear();

  Paint_NewImage(ImageBW, EPD_W, EPD_H, 0, WHITE);  // Create a new white background image.
  Paint_Clear(WHITE);  // Clear the canvas.

  EPD_ShowString(10, 10, "WIFI CONNECTION LOST", 16, BLACK);
  EPD_ShowString(10, 30, "Reconfigure WiFi at AP:", 16, BLACK);
  EPD_ShowString(10, 50, "Cart Setup", 16, BLACK);

  EPD_Display(ImageBW);
  EPD_Update();
}

// ----------------------------------------------------
// QR CODE
// ----------------------------------------------------
void displayQRCodeOnEPD(const char* text) {
  // Build "ptl-" + original text
  char qrText[64];
  snprintf(qrText, sizeof(qrText), "ptl-%s", text);

  QRCode qrcode;
  uint8_t qrcodeData[qrcode_getBufferSize(4)];
  qrcode_initText(&qrcode, qrcodeData, 4, 0, qrText);  // ✅ use qrText now

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


