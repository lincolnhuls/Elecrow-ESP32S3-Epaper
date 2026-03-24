#include <Arduino.h>
#include <SPI.h>
#include "EPD.h"
#include "Display_EPD_W21.h"
#include "Display_EPD_W21_spi.h"

void EPD_GPIOInit(void)
{
  pinMode(EPD_PIN_CS, OUTPUT);
  pinMode(EPD_PIN_DC, OUTPUT);
  pinMode(EPD_PIN_RST, OUTPUT);
  pinMode(EPD_PIN_BUSY, INPUT);

  digitalWrite(EPD_PIN_CS, HIGH);
  digitalWrite(EPD_PIN_DC, HIGH);
  digitalWrite(EPD_PIN_RST, HIGH);

  // no MISO, CS handled manually in the display driver
  SPI.begin(EPD_PIN_SCK, -1, EPD_PIN_MOSI, EPD_PIN_CS);
}

void EPD_READBUSY(void)
{
  while (isEPD_W21_BUSY != 0) {
    delay(1);
  }
}

void EPD_HW_RESET(void)
{
  EPD_W21_RST_0;
  delay(10);
  EPD_W21_RST_1;
  delay(10);
}

// void EPD_Update(void)
// {
//   ::EPD_Update();
// }

void EPD_PartInit(void)
{
  // For now use normal init
  EPD_HW_Init();
}

void EPD_FastInit(void)
{
  EPD_HW_Init_Fast();
}

// void EPD_DeepSleep(void)
// {
//   ::EPD_DeepSleep();
// }

void EPD_Init(void)
{
  EPD_HW_Init();
}

void EPD_Display_Clear(void)
{
  EPD_WhiteScreen_White();
}

void EPD_Display(const uint8_t *image)
{
  EPD_WhiteScreen_ALL(image);
}
