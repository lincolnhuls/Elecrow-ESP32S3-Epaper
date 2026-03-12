#ifndef _EPD_H_
#define _EPD_H_

#include <Arduino.h>

// New panel resolution
#define EPD_W 184
#define EPD_H 384

#ifndef WHITE
#define WHITE 0xFF
#endif

#ifndef BLACK
#define BLACK 0x00
#endif

void EPD_GPIOInit(void);
void EPD_READBUSY(void);
void EPD_HW_RESET(void);
void EPD_Update(void);
void EPD_PartInit(void);
void EPD_FastInit(void);
void EPD_DeepSleep(void);
void EPD_Init(void);
void EPD_Display_Clear(void);
void EPD_Display(const uint8_t *image);

#endif