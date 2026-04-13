/*
 * OLEDKeyboard.h
 *
 *  Created on: 2023年12月30日
 *      Author: 86139
 */
#ifndef __OLEDKEYBOARD_H
#define __OLEDKEYBOARD_H
#include "zf_common_headfile.h"
#include "OLEDFun.h"
#include "ssd1306.h"

#define Standby_Buffer  95

extern int gcstart;
extern uint8 UpperChoose;
extern uint32 Threshold;

/*-------键显要改的变量-------*/


/*----键显显示要用的变量----*/
extern uint32 Set_Sign_X[8];
extern uint32 Set_X_Row[8]; 
extern uint32 Set_Sign_Y[8];
extern uint32 Set_Y_Row[8];  
extern float Set_Angle[8];
extern double Set_Guidance_X[8];
extern double Set_Guidance_Y[8];
extern uint32 Set_Sign[8];
extern double Set_X[20] ;
extern double Set_Y[20] ;
extern int Left_Target_Speed[2];
extern int Right_Target_Speed[2];
extern int Left_Real_Speed;
extern int Right_Real_Speed;
extern float Pos_Error;          // 位置偏差
extern int Speed_Error;        // 位置环输出差速
extern uint16 Getpoint_LastFlag;

/***********************************鍑芥暟澹版槑***********************************/
extern void Oled_Input(void);
extern void Oled_Display(void);
extern void camera_flash_read(void);
extern void control_flash_read(void);
extern void speed_flash_read(void);
extern void speedPID_flash_read(void);
extern void run_flash_read(void);
extern void Set_flash_read(void);
extern void Hyperparameter_Init(void);
extern void Oled_Send(void);
extern void element_flash_read(void);
extern void Get_Point(void);

void Balance_flash_read(void);
void Control_flash_read(void);
void Run_flash_read(void);
#endif
