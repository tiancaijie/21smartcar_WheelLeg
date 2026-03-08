/*************************************************
Copyright (C), 2016-2017, TYUT JBD TEAM C.
File name: CH455.c
Author: TEAM  A B C
Version:0.0               Date: 2016.11.12
Description:    CH455
Others:      ÎÞ
Function List:    1.CH450_Write
                  2.CH450_Read
History:
<author>  <time>      <version > <desc>
JBD       2016.10.21  0.0      ³õÊ¼
**************************************************/
#include "OLED.h"

void CH455_Write(void)
{
    I2c_Start();
    I2c_Write_OneByte(0x48);
    I2C_Wait_Ask();
    I2c_Write_OneByte(0x01);
    I2C_Wait_Ask();
    I2c_Stop();
}

unsigned char CH455_Read(void)
{
    unsigned char keycode;
    I2c_Start();
    I2c_Write_OneByte(0x4f);
    I2C_Wait_Ask();
    keycode=I2c_Read_OneByte(1);
    I2c_Stop();   //ÐÞ¸Ä
    return(keycode);
}
