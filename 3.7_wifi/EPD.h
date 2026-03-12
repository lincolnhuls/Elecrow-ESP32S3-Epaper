#ifndef _EPD_H_
#define _EPD_H_

#include "EPD_SPI.h"

// For default screen size
#define EPD_W 240 
#define EPD_H 416

// For new screen, different size
// #define EPD_W 184
// #define EPD_H 384

#define WHITE 0xFF
#define BLACK 0x00


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
