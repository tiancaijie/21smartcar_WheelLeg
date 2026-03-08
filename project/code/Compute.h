/*
 * Compute.h
 *
 *  Created on: 2024年12月6日
 *      Author: YWW
 */

#ifndef _Compute_H_
#define _Compute_H_

#include "zf_common_headfile.h"

/* ========= 全局变量声明 ========== */
//科三相关
extern uint32 Po_Sub3_Order[6];
extern uint32 Sub3_Order_Proj;
/* =========== 枚举 ============= */
typedef union floatu8data
{
    float floatdata;
    uint8 u8data[4];
}floatu8data;

typedef enum  // 跑车方式枚举
{
    GPS  = 1,
    INS  = 2,
}RunMode_enum;

typedef enum  // 掉头方式枚举
{
    Left  = 1,
    Right = 2,
}BackSide_enum;

//惯导
typedef struct INS_TypeDef
{
      double x;
      double y;
}INS_TypeDef;

//GPS
typedef struct GPS_TypeDef
{
    double  latitude;
    double  longitude;
}GPS_TypeDef;

typedef struct
{
     INS_TypeDef INS;
     GPS_TypeDef GPS;
}coordinate;

typedef struct
{
    coordinate Car;
    coordinate Car_Last;

    uint8 Get_GPS_State;
    struct
    {
        coordinate Po_Sub1[3];
        double latitude_Offset;
        double longitude_Offset;
        double N_Y_Angle_Offset;
        double Distance_Car2Goal;
        double Forward_Slow_Distance;
        double Back_Slow_Distance;
        double Speed_Change_K;

        uint8 Po_Goal;
        RunMode_enum Sub1_RunMode;
        BackSide_enum Sub1_BackSide;
    }Sub1;//科目一

    struct
    {
        coordinate Po_Sub2[197];
        uint32 Sub2_EndLine;
        uint8 Line_Goal;
        double Change_Distance;
        double Distance_Car2Goal;

        BackSide_enum Sub2_BackSide;
    }Sub2;//科目二

    struct
    {
        coordinate Po_Sub3[17];
        uint32 Sub3_EndLine;
        uint8 Line_Goal;
        uint32 Change_Distance;
        double Distance_Car2Goal;
        BackSide_enum Sub3_BackSide;

        uint8 Back_EL;
        uint8 Grass_EL;
        uint8 Jolt_EL;
        uint8 Slope_EL;
        uint8 Bridge_EL;

    }Sub3;//科目三

    struct
    {
        coordinate Po_Sub4[6];
        RunMode_enum Sub4_RunMode;
        double Distance_Car2Goal;

    }Sub4;//科目四

}CoCtrlHandle_TypeDef;

extern CoCtrlHandle_TypeDef CoCtrl;



/* =========== 函数声明 ============ */
extern void ComAngle(uint32 ExSubject);
extern void ComSpeed(uint32 ExSubject);
#endif /* _Compute_H_ */
