#ifndef EPD_MANAGER_H
#define EPD_MANAGER_H

#include <Arduino.h>
#include "EPD.h"
#include "EPD_GUI.h"
#include <QRCodeGenerator.h>
#include <Display_EPD_W21.h>

class EpdManager {
public:
  // Methods
  void init();
  void showBootScreenBase();
  void updateBootDots(uint8_t dotCount);
  void showUpdatingScreenBase();
  void updateUpdatingDots(uint8_t dotCount);
  void showConnectedDashboard(const char* ssid, IPAddress ip, long rssi, String deviceId, String currentVersion, float v);
  void setConnectedScreenShown(bool value);
  bool getConnectedScreenShown();
  void setCurrentRSSI(long rssi);
  long getCurrentRSSI();

private:
  // Methods
  void clearBootDotsInBuffer();
  void clearUpdatingDotsInBuffer();
  void displayQRCodeOnEPD(const char *text, int yOffset, int qrScale);
  int rssiToBars(long rssi);
  void drawWiFiIcon(int x, int y, int bars);

  // Attributes
  int compareRSSI = 0;
  int currentRSSI = 0;

  int ROTATION = 0;
  bool connectedScreenShown = false;
  // display buffer
  uint8_t ImageBW[(EPD_W * EPD_H) / 8];
  char buffer[200];

  // Screen/layout helpers for the new panel
  const int SCREEN_MARGIN_X = 10;
  const int SCREEN_MARGIN_Y = 10;
  const int QR_MARGIN_RIGHT = 8;
  const int QR_DEFAULT_SCALE = 4;
  const int QR_LARGE_SCALE = 4;
  const int QR_SMALL_SCALE = 3;
  static constexpr uint8_t SCREEN = 7;
};

#endif