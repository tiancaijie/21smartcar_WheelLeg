/*
 * control.h
 *
 *  Created on: 2023年12月11日
 *      Author: 86139
 */

#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

#include "pid.h"
#include "zf_common_headfile.h"
#include "Element.h"

#define N 3
extern float IMU_ax_output_a[N];
extern int Center_deviation2[5];
extern void SetMotorVoltage();

typedef struct
{
   uint8 Zebra;
   uint8 Circle;
   uint8 Hill;
   uint8 Birck;
   uint8 Wait;
}
Flag_ON_DEFINE;
extern Flag_ON_DEFINE Flag_ON;

typedef struct
{
    int Car;
    int Single_Bridge;
    int Right;
    int Zebra;
    int Circle;
    int Hill;
    int Birck;
}
SpeedExp_DEFINE;

typedef struct
{
    float Theta1;
    float Theta2;

    float OC;
    float Xc;
    float Yc;

    int PWM_Out1;
    int PWM_Out2;
}VMC_Data_Type;

typedef enum  // 键显状态更新
{
    NO  = 0,
    ING = 1,
    OK  = 2,
}OLEDState_enum;

typedef struct
{
     //-----键显示------//
    OLEDState_enum OLEDState;


    //-----速度------//
    int SpeedTarget;
    int RealSpeed;
    int Motor_Speed;
    int Run_PWM;
    int Acceleration;
    int Last_Motor_Speed;

    //-----方向------//
    float Last_High_Speed_Dir_Control_Out[6]; //方向pid输出历史值
    float Last_Low_Speed_Dir_Control_Out[6]; //方向pid输出历史值

    float AngleTarget ;             //角度
    float AngleDiffer ;
    int Dir_PWM;
    int Dir_PWM_Last;
    uint8 Com_finish;

    //-----跑车------//
    uint32 Subject_Set;             //科目选择

    struct
    {
        int ExSp_S1_H;
        int ExSp_S1_L;
        uint8 Sub1_finish;
    }Sub1;//科目一

    struct
    {
        int ExSp_S2_H;
        int ExSp_S2_M;
        int ExSp_S2_L;
        uint8 Sub2_finish;
    }Sub2;//科目二

    struct
    {
        int ExSp_S3_H;
        int ExSp_S3_M;
        int ExSp_S3_Back;
        int ExSp_S3_Grass;
        int ExSp_S3_Jolt;
        int ExSp_S3_Slope;
        int ExSp_S3_Bridge;
        uint8 Sub3_finish;
    }Sub3;//科目三

    struct
    {
        int ExSp_S4;
        uint8 Sub4_finish;
        uint8 Sub4_Command_finish;
        int Command_Line[6];
    }Sub4;//科目四
    /* ----Pitch control----*/
    struct
    {
        float ExpectSpeed_Exp;              // 期望车速
        float ExpectSpeed_Act;
        float ExpectAngle;                 // Pitch轴期望角度
        float ExpectOmega;                // Pitch轴期望角速度
        float BalancePoint;               // 平衡点
        uint8  BalanceSign;
        float Output;
        float LegOutput;
    }Pitch;
    struct
    {
        float ExpectAngle_Act;              // Yaw轴期望角度
        float ExpectOmega_Act;              // Yaw轴期望角速度
        float ExpectOmega_Exp;
        float ExpectOmega_Odo;              // 导航期望Yaw轴角速度
        float BalancePoint;                 // 平衡点
        uint8  BalanceSign;
        float Output;
    }Yaw;
    struct
    {
        float BalancePoint;
        float Output;
    }Roll;
}CtrlHandle_TypeDef;

typedef enum
{
    Find_Off = 0,
    Find_Normal = 1,
    Find_Init   = 2,
}FrameHead_Type;

extern SpeedExp_DEFINE SpeedExp;
extern CtrlHandle_TypeDef hCtrl;
extern float L_Height_Init;
extern float R_Height_Init;
extern float Swerve_Offset[2];
extern float Swerve_Step;
extern float Final_Speed;
extern float ay_last;
extern float gx_last;
extern float LegOutput;
extern int Slow_Flag;
extern float Hill_Height;
extern int PWM_Output_Flag;
float DirectionControl(int EleMode, int NewError);
extern void SpeedControl(void);
void PID_InitAll(void);
void filter(float input_new, float *input, float *output,double *filter_b, double *filter_a,uint8 M);
extern void GetAngle(void);
void Angle_cal(void);
void FiveLink_Opposite(VMC_Data_Type* leg, float Xc, float Yc);
void PWM_Output(VMC_Data_Type* leg1,VMC_Data_Type* leg2);
void Pitch_AngleCtrl(void);
void Hill_Stand(void);
void Pitch_LegCtrl(void);
void PitchOmegaCtrl(void);
void PitchAngleCtrl(void);
void YawOmegaCtrl(void);
void YawAngleCtrl(void);
void RollAngleCtrl(void);
void Speed_Output(void);
void Direction_Ctrl(void);
void SAFE_PROTECT(void);
void Swerve_RollOffset(void);
void Remote_Conctol(void);
#endif /* CODE_CONTROL_H_ */
