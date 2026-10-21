#include "EpdManager.h"
#include <Arduino.h>
#include "EPD.h"
#include "EPD_GUI.h"
#include <QRCodeGenerator.h>
#include <Display_EPD_W21.h>

// ======================= Public Methods =======================

void EpdManager::init() {
  pinMode(SCREEN, OUTPUT);
  digitalWrite(SCREEN, HIGH);

  EPD_GPIOInit();
  EPD_Init();

  Paint_NewImage(ImageBW, EPD_W, EPD_H, ROTATION, WHITE);
  Paint_Clear(WHITE);
}

void EpdManager::showBootScreenBase() {
  Paint_NewImage(ImageBW, EPD_W, EPD_H, ROTATION, WHITE);
  Paint_Clear(WHITE);

  EPD_ShowString(10, 10, "Connecting to WiFi", 16, BLACK);

  // First draw after boot should be full refresh
  EPD_Display(ImageBW);

  // Seed the panel for later partial updates
  EPD_SetRAMValue_BaseMap(ImageBW);
}

void EpdManager::updateBootDots(uint8_t dotCount) {
  // Modify the existing full-screen buffer using the same coordinates
  // that already look correct in your normal drawing code.
  clearBootDotsInBuffer();

  if (dotCount >= 1)
    EPD_ShowString(154, 10, ".", 16, BLACK);
  if (dotCount >= 2)
    EPD_ShowString(161, 10, ".", 16, BLACK);
  if (dotCount >= 3)
    EPD_ShowString(168, 10, ".", 16, BLACK);

  // Partial-refresh using the full buffer
  EPD_Dis_PartAll(ImageBW);
}

void EpdManager::showUpdatingScreenBase() {
  EPD_FastInit();

  Paint_NewImage(ImageBW, EPD_W, EPD_H, ROTATION, WHITE);
  Paint_Clear(WHITE);

  EPD_ShowString(10, 10, "UPDATE DETECTED", 16, BLACK);
  EPD_ShowString(10, 34, "Updating device", 16, BLACK);

  // first draw of this screen should be full
  EPD_Display(ImageBW);

  // seed panel memory for repeated partial updates
  EPD_SetRAMValue_BaseMap(ImageBW);
}

void EpdManager::updateUpdatingDots(uint8_t dotCount) {
  clearUpdatingDotsInBuffer();

  if (dotCount >= 1)
    EPD_ShowString(132, 34, ".", 16, BLACK);
  if (dotCount >= 2)
    EPD_ShowString(138, 34, ".", 16, BLACK);
  if (dotCount >= 3)
    EPD_ShowString(144, 34, ".", 16, BLACK);

  EPD_Dis_PartAll(ImageBW);
}

void EpdManager::showConnectedDashboard(IPAddress ip, long rssi, const String& deviceId, const String& currentVersion, const String& status) {
  EPD_FastInit();

  Paint_NewImage(ImageBW, EPD_W, EPD_H, ROTATION, WHITE);
  Paint_Clear(WHITE);

  EPD_ShowString(10, 13, "WiFi Connected", 16, BLACK);

  EPD_ShowString(10, 41, "Mqtt Connected", 16, BLACK);

  constexpr uint16_t STATUS_X = 10;
  constexpr uint16_t STATUS_Y1 = 70;
  constexpr uint16_t STATUS_Y2 = 98;
  constexpr size_t STATUS_MAX_CHARS = 29;

  String line1;
  String line2;
  String remaining = status;

  remaining.trim();
  
  constexpr size_t PREFIX_LENGTH = 8;
  constexpr size_t FIRST_LINE_CHARS = STATUS_MAX_CHARS - PREFIX_LENGTH;

  if (remaining.length() <= FIRST_LINE_CHARS) { 
    line1 = remaining; 
  } else { 
    int splitAt = remaining.lastIndexOf(' ', FIRST_LINE_CHARS); 

    if (splitAt <= 0) { 
      splitAt = FIRST_LINE_CHARS;
    } 
  
    line1 = remaining.substring(0, splitAt); 
    remaining = remaining.substring( splitAt < FIRST_LINE_CHARS ? splitAt + 1 : splitAt ); 
    remaining.trim(); 
    if (remaining.length() > STATUS_MAX_CHARS) { 
      int secondSplit = remaining.lastIndexOf(' ', STATUS_MAX_CHARS - 3); 
      if (secondSplit > 0) { 
        line2 = remaining.substring(0, secondSplit); 
      } else { 
        line2 = remaining.substring(0, STATUS_MAX_CHARS - 3);
      } 
      line2 += "..."; 
    } else { 
      line2 = remaining;
    } 
  }
  snprintf(buffer, sizeof(buffer), "Status: %s", line1.c_str());
  EPD_ShowString(STATUS_X, STATUS_Y1, buffer, 16, BLACK);

  if (line2.length() > 0) {
    EPD_ShowString(STATUS_X, STATUS_Y2, line2.c_str(), 16, BLACK);
  }

  compareRSSI = rssi;
  snprintf(buffer, sizeof(buffer), "Signal: %ld dBm", rssi);
  EPD_ShowString(10, 127, buffer, 16, BLACK);

  int bars = rssiToBars(rssi);
  drawWiFiIcon(140, 117, bars);

  snprintf(buffer, sizeof(buffer), "ID: %s", deviceId.c_str());
  EPD_ShowString(10, 155, buffer, 16, BLACK);

  displayQRCodeOnEPD(deviceId.c_str(), -1, QR_LARGE_SCALE);

  EPD_WhiteScreen_ALL_Fast(ImageBW);
  EPD_DeepSleep();
}

void EpdManager::setConnectedScreenShown(bool value) {
  connectedScreenShown = value;
}

bool EpdManager::getConnectedScreenShown() {
  return connectedScreenShown;
}

void EpdManager::setCurrentRSSI(long rssi) {
  currentRSSI = rssi;
}

long EpdManager::getCurrentRSSI() {
  return currentRSSI;
}

void EpdManager::showClockScreen() {
  EPD_FastInit();

  Paint_NewImage(ImageBW, EPD_W, EPD_H, ROTATION, WHITE);
  Paint_Clear(WHITE);

  EPD_ShowString(10, 13, "Syncing Clock...", 16, BLACK);

  EPD_WhiteScreen_ALL_Fast(ImageBW);
}

void EpdManager::showMqttScreen() {
  EPD_FastInit();

  Paint_NewImage(ImageBW, EPD_W, EPD_H, ROTATION, WHITE);
  Paint_Clear(WHITE);

  EPD_ShowString(10, 13, "Connecting to Mqtt...", 16, BLACK);

  EPD_WhiteScreen_ALL_Fast(ImageBW);
}


// ======================= Private Methods =======================

void EpdManager::clearBootDotsInBuffer() {
  // Clear just the dots area in the full-screen buffer
  // Covers the three dots at x = 154, 161, 168
  for (int x = 154; x < 176; x++) {
    for (int y = 10; y < 26; y++) {
      Paint_SetPixel(x, y, WHITE);
    }
  }
}

void EpdManager::clearUpdatingDotsInBuffer() {
  for (int x = 132; x < 150; x++) {
    for (int y = 34; y < 50; y++) {
      Paint_SetPixel(x, y, WHITE);
    }
  }
}

void EpdManager::displayQRCodeOnEPD(const char *text, int yOffset, int qrScale) {
  QRCode qrcode;
  uint8_t qrcodeData[qrcode_getBufferSize(4)];
  qrcode_initText(&qrcode, qrcodeData, 4, 0, text);

  int scale = qrScale;
  if (scale < 1)
    scale = QR_DEFAULT_SCALE;

  int qrPixSize = qrcode.size * scale;

  // Use actual drawable dimensions from the GUI library
  int displayWidth = Paint.width;
  int displayHeight = Paint.height;

  // Place QR on the right with a small margin
  int pxOffset = displayWidth - qrPixSize - QR_MARGIN_RIGHT;

  // Vertically center unless caller gives a specific y
  int pyOffset;
  if (yOffset >= 0)
    pyOffset = yOffset;
  else
    pyOffset = (displayHeight - qrPixSize) / 2;

  // Clamp into visible area
  if (pxOffset < SCREEN_MARGIN_X)
    pxOffset = SCREEN_MARGIN_X;
  if (pyOffset < SCREEN_MARGIN_Y)
    pyOffset = SCREEN_MARGIN_Y;

  if (pxOffset + qrPixSize >= displayWidth) {
    pxOffset = displayWidth - qrPixSize - 1;
  }
  if (pyOffset + qrPixSize >= displayHeight) {
    pyOffset = displayHeight - qrPixSize - 1;
  }

  if (pxOffset < 0)
    pxOffset = 0;
  if (pyOffset < 0)
    pyOffset = 0;

  for (uint8_t y = 0; y < qrcode.size; y++) {
    for (uint8_t x = 0; x < qrcode.size; x++) {
      bool module = qrcode_getModule(&qrcode, x, y);

      for (int dx = 0; dx < scale; dx++) {
        for (int dy = 0; dy < scale; dy++) {
          int px = pxOffset + x * scale + dx;
          int py = pyOffset + y * scale + dy;

          if (px >= 0 && px < displayWidth && py >= 0 && py < displayHeight) {
            Paint_SetPixel(px, py, module ? BLACK : WHITE);
          }
        }
      }
    }
  }
}

int EpdManager::rssiToBars(long rssi) {
  if (rssi >= -60)
    return 4; // Excellent
  if (rssi >= -70)
    return 3; // Good
  if (rssi >= -80)
    return 2; // Fair
  if (rssi >= -90)
    return 1; // Weak
  return 0;   // No signal
}

void EpdManager::drawWiFiIcon(int x, int y, int bars) {
  const int barWidth = 4;
  const int gap = 3;

  for (int i = 0; i < 4; i++) {
    int barXStart = x + i * (barWidth + gap);
    int barXEnd = barXStart + barWidth;
    int barHeight = 6 * (i + 1); // 6, 12, 18, 24 px

    int barYStart = y + (24 - barHeight);
    int barYEnd = y + 24;

    // 1) Always draw OUTLINE in BLACK (hollow bar)
    EPD_DrawRectangle(barXStart, barYStart, barXEnd, barYEnd, BLACK,
                      0); // 0 = outline only

    // 2) If this bar is "active", fill INSIDE the outline
    if (i < bars) {
      // Slightly inset rectangle so we don't overwrite the outline
      EPD_DrawRectangle(barXStart + 1, barYStart + 1, barXEnd - 1, barYEnd - 1,
                        BLACK, 1); // 1 = filled
    }
  }
}
