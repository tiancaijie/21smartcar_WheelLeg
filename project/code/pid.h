#ifndef __PID_H
#define __PID_H

#include "zf_common_headfile.h"

typedef enum
{
    PID_MODE_POSITION = 0,
    PID_MODE_POSITION_ISeparation,
    PID_MODE_ADD,
}PID_MODE_TypeDef;

typedef struct
{
    uint8               name[20];                                                       //对应结构体的名字
    uint8               index;                                                          //在flash中存储的引导值（从1开始）
    PID_MODE_TypeDef    mode;
    uint8               ZeroOutFlag;

    float               kp;
    float               ki;
    float               kd;
    float               factor_kp;                                                      //kp倍数
    float               factor_ki;                                                      //ki倍数
    float               factor_kd;                                                      //kd倍数

    float               iOutMax;
    float               outMax;
    float               ISeparation_error_max;
}PID_InitTypeDef;

typedef struct
{
    PID_MODE_TypeDef    mode;
    uint8               ZeroOutFlag;

    float               set;                                                            //希望值
    float               err3[3];                                                        //偏差值

    float               kp;
    float               ki;
    float               kd;

    float               pOut;
    float               iOut;
    float               dOut;

    float               out;            //储存每一次pid计算后的输出值
    float               out_terminal;

    float               iOutMax;
    float               outMax;
    float               ISeparation_error_max;

}PID_HandleTypeDef;

#define PID_Sector 5

void PID_init(PID_HandleTypeDef *pid, PID_InitTypeDef *PID);
void PID_loadparam(PID_HandleTypeDef *pid, float kp, float ki, float kd);
void PID_loadtarget(PID_HandleTypeDef *pid, float target);
float PID_calc(PID_HandleTypeDef *pid, float exp_data, float real_data);
void PID_cleardata(PID_HandleTypeDef *pid);
boolean PID_InputCheck(PID_InitTypeDef *target);

#endif/* __PID_H */


