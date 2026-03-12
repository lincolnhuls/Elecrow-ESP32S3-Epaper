#ifndef _DISPLAY_EPD_W21_SPI_
#define _DISPLAY_EPD_W21_SPI_
#include "Arduino.h"

//IO settings
//SDIN--GPIO23(MOSI)
//SCLK---GPIO18(SCLK)
#define isEPD_W21_BUSY digitalRead(48)  //BUSY
#define EPD_W21_RST_0 digitalWrite(47,LOW)  //RES
#define EPD_W21_RST_1 digitalWrite(47,HIGH)
#define EPD_W21_DC_0  digitalWrite(46,LOW) //DC
#define EPD_W21_DC_1  digitalWrite(46,HIGH)
#define EPD_W21_CS_0 digitalWrite(45,LOW) //CS
#define EPD_W21_CS_1 digitalWrite(45,HIGH)


void SPI_Write(unsigned char value);
void EPD_W21_WriteDATA(unsigned char datas);
void EPD_W21_WriteCMD(unsigned char command);


#endif 
