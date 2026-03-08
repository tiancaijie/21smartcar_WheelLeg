/*
 * fun.h
 *
 *  Created on: 2023年12月11日
 *      Author: 86139
 */

#ifndef CODE_FUN_H_
#define CODE_FUN_H_

#include "zf_common_headfile.h"
/*****功能声明*****/
#define Step_Limit(New,Old,Range)       (New > (Old + Range) ? (Old + Range) : (New < (Old - Range) ? (Old - Range) : (New)))
#define ABS(x)                          (((x) >= (0.0f)) ? (x) : (-(x)))
#define Amplitude_Limit(Value, Down, Up)    ( (Value) < (Down) ? (Down) : ( (Value) > (Up) ? (Up) : (Value) ) )
#define Value_Max(V1, V2)                (((V1) > (V2)) ? (V1) : (V2))
#define Value_Min(V1, V2)                (((V1) < (V2)) ? (V1) : (V2))
#define Deg2Rad(deg)                    ((double)(deg) * 0.0174532925)
#define Rad2Deg(rad)                    ((double)(rad) * 57.295779513)

typedef enum
{
    Set_Speed = 0x01,
    Get_Speed = 0x02,
    Zero_Init = 0x03,
}FOCcontrol_Type;

typedef union
{
    float floatValue;
    uint8_t uint8Value[4];
}float2u8_union;


void ClearArrayChar(unsigned char *PTemp, unsigned char *PEnd);
void ClearArraySignedChar(signed char *PTemp,signed char *PEnd);
void ClearArrayInt(int *PTemp, int *PEnd);
void ClearArrayShortInt(short int *PTemp,short int *PEnd);
void ClearArrayfloat(float *PTemp, float *PEnd);
unsigned int m_sqrt(unsigned int x);
int Square_int(int arg);
float Square_float(float arg);
void Linear_Interpolation(int *Ptemp,int End_Row);
void GetSpeed (void);
void ParameterSent(void);
extern void VOFA_ParameterSent(uart_index_enum uart_n);
extern int giRightWheelSpeed;
extern int giLeftWheelSpeed;
extern int giWheelSpeed;
extern int uartget;
//int IsStraight(int giRoadEdge[][2], int begin, int end, int limit);
//int IsStraight(int giRoadEdge[][2], int begin, int end);
//int IsStraight(int EdgeType int begin, int end, int limit);
extern int SpeedMax, SpeedMid, SpeedMin, SpeedHill, SpeedCircle;
extern void VOFA_Receive_callback (uart_index_enum uart_n);
void FOC_SendControl(FOCcontrol_Type FOC_Send,int16 Left_SpeedPWM,int16 Right_SpeedPWM);
//unsigned char TrackType (unsigned char LostRow);
//int ChangeSpeed(int SpeedP);
#endif /* CODE_FUN_H_ */
