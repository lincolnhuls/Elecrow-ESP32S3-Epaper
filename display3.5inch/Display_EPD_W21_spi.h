#ifndef _DISPLAY_EPD_W21_SPI_
#define _DISPLAY_EPD_W21_SPI_
#include "Arduino.h"
#include "EPD.h"

//IO settings
//SDIN--GPIO23(MOSI)
//SCLK---GPIO18(SCLK)
#define isEPD_W21_BUSY digitalRead(EPD_PIN_BUSY)  //BUSY
#define EPD_W21_RST_0 digitalWrite(EPD_PIN_RST,LOW)  //RES
#define EPD_W21_RST_1 digitalWrite(EPD_PIN_RST,HIGH)
#define EPD_W21_DC_0  digitalWrite(EPD_PIN_DC,LOW) //DC
#define EPD_W21_DC_1  digitalWrite(EPD_PIN_DC,HIGH)
#define EPD_W21_CS_0 digitalWrite(EPD_PIN_CS,LOW) //CS
#define EPD_W21_CS_1 digitalWrite(EPD_PIN_CS,HIGH)


void SPI_Write(unsigned char value);
void EPD_W21_WriteDATA(unsigned char datas);
void EPD_W21_WriteCMD(unsigned char command);


#endif 
