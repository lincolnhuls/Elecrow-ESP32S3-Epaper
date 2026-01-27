// Flash settings for this board are fairly specific
// 
// Board: ESP32S3 Dev Module 
// Flash Size: 8MB (64Mb)
// Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"
// PSRAM: "OPI PSRAM"
// Upload Speed: "460800" (Board will not upload if this is not set) 


#include <FastLED.h>
#include <QRCodeGenerator.h>
#include <WiFiManager.h>
#include <Arduino.h>
#include "EPD.h"
#include "EPD_GUI.h"
#include <WiFi.h>

WiFiManager wifiManager;

#define NUM_LEDS 100
#define DATA_PIN 8
CRGB leds[NUM_LEDS];

// 240x416 buffer
uint8_t ImageBW[12480];
char buffer[200];

// Forward declarations
int rssiToBars(long rssi);
void drawWifiIcon();

uint64_t chipid;
char chipIdStr[32];

// ----------------------------------------------------
// SETUP
// ----------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);

  // wifiManager.resetSettings();

  FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);

  chipid = ESP.getEfuseMac();
  sprintf(chipIdStr, "%04X%08X",
          (uint16_t)(chipid >> 32),
          (uint32_t)chipid);

  // ----- EPD INIT -----
  EPD_GPIOInit();
  EPD_Init();

  Paint_NewImage(ImageBW, EPD_W, EPD_H, Rotation, WHITE);
  Paint_Clear(WHITE);

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
  showConnectedDashboard();
}

// ----------------------------------------------------
// LOOP (FULLY NON-BLOCKING)
// ----------------------------------------------------
void loop() {
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
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Green;
  }
  FastLED.show();
  delay(500);
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Black;
  }
  FastLED.show();
  delay(500);
}

// ----------------------------------------------------
// CONNECTED DASHBOARD
// ----------------------------------------------------
void showConnectedDashboard() {
  EPD_FastInit();
  EPD_Display_Clear();

  Paint_NewImage(ImageBW, EPD_W, EPD_H, 180, WHITE);
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

  Paint_NewImage(ImageBW, EPD_W, EPD_H, 180, WHITE);  // Create a new white background image.
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

  Paint_NewImage(ImageBW, EPD_W, EPD_H, 180, WHITE);  // Create a new white background image.
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


