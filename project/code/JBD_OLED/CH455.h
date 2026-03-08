/*
 * CH455.h
 *
 *  Created on: 2023年12月30日
 *      Author: 86139
 */

#ifndef CODE_CH455_H_
#define CODE_CH455_H_

#include  "OLED.h"
#include "IIC.h"
/************************************宏定义************************************/

/*********************************全局变量声明*********************************/

extern unsigned char keycode;

/***********************************函数声明***********************************/
extern void CH455_Write(void);
extern unsigned char CH455_Read(void);

#endif /* CODE_CH455_H_ */
