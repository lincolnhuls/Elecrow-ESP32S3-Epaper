#include <Arduino.h>
#include <SPI.h>
#include "EPD.h"
#include "Display_EPD_W21.h"
#include "Display_EPD_W21_spi.h"

void EPD_GPIOInit(void)
{
  pinMode(45, OUTPUT); // CS
  pinMode(46, OUTPUT); // DC
  pinMode(47, OUTPUT); // RST
  pinMode(48, INPUT);  // BUSY

  digitalWrite(45, HIGH);
  digitalWrite(46, HIGH);
  digitalWrite(47, HIGH);

  // SCK=12, MOSI=11, no MISO, CS handled manually in driver
  SPI.begin(12, -1, 11, 45);
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