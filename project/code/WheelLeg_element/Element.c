/*
 * Element.c
 *
 *  Created on: 2025�?5�?13�?
 *      Author: Zzio
 */
#include "zf_common_headfile.h"
#include "Element.h"
#include "isr.h"
#include "control.h"

// 期望�?向�??
float Expect_Angle = 0;
// 期望�?仰�??
float Expect_Angle_Pitch = 0;
// 行走腿偏移量
float Row_LegOffset[2] = {0};
// 单�?��?�度
float Single_Angle = 10;
// 当前�?标点编号
uint8 Now_Dot = 1;
// 总目标点�?
uint32 All_Dot = 8;
// 当前�?标点状态标�?
uint8 Now_Dot_Flag = 0;
// 上一�?GPS状�?
uint8 Last_GPS_State = 0;
// 状态�?�数
uint8 State_get_count = 0;
// INS�?向�?�准完成标志
uint8 INS_Heading_Align_Done = 0;
// INS�?向�?�准偏移
float INS_Heading_Align_Offset = 0;
// 单桥通过标志
int Single_Bridge = 0;
// 桥上�?/下降标志
int BridgeUP_Flag = 0,BridgeDOWN_Flag = 0;

uint8 Run_GPS = 0;
uint8 Run_INS = 1;
uint8 Spin_State = 0;             // 0: �?开�?  1: 旋转�?

// 桥通过计数与状态�?��?
void Single_BridgeCount(void)
{
    if (Single_Bridge == 1 && Bridge_Count > 0 && BridgeDOWN_Flag == 0 && BridgeUP_Flag == 0 &&(Stand_Height < 0.06 && (ABS(L_Height - R_Height) >= 0.025) || Stand_Height >= 0.06 && ABS(L_Height - R_Height) >= 0.045))
    {
        BridgeUP_Flag = 1;
    }
    if (Single_Bridge == 1 && Bridge_Count > 0 && BridgeDOWN_Flag == 0 && BridgeUP_Flag == 1 && (ABS(L_Height - R_Height) <= 0.01))
    {
        BridgeDOWN_Flag = 1;
    }

    if (BridgeUP_Flag && BridgeDOWN_Flag && Bridge_Count > 0 && Slow_Flag == 1)
    {
        Bridge_Count--;
        BridgeUP_Flag = 0;
        BridgeDOWN_Flag = 0;
    }
}

extern PID_HandleTypeDef PID_RollOmega;
extern PID_HandleTypeDef PID_RollAngle;
float Stand_Height = 0.055;
float L_Height_Init = 0.028;//0.052;
float R_Height_Init = 0.028;//0.052;
float Roll_LStep = 0.00018;
float Accumulation = 0;
// 桥通过状态控制（预留�?
void Single_BridgeStateCTRL(void)
{

}

int JumpTimeCount = 0;
int JumpOff = 0;
// int Jump_Finish_Flag = 1;        //遥控跳跃
int Jump_Finish_Flag = 0;           //�?跳一�?
int Jump_Finish_Flag1 = 0;
uint16 Extend_time = 100;
uint16 Shrink_time = 200;
uint16 Cushion_time = 240;
uint16 Over_time = 350;//320;
float Extend_Height = 2800;
float Shrink_Height = 0.075;
float Cushion_Height = 1000;
float Cushion_Speed  = 1000;
// 跳跃动作控制主函�?
void JumpCTRL(void)
{
    JumpTimeCount+=2;
    if (Extend_time >= JumpTimeCount)
    {
        //L_Height = Extend_Height;
       // R_Height = Extend_Height;
      Jump_Flag = 1;
    }
    else if (Shrink_time >= JumpTimeCount && JumpTimeCount > Extend_time)
    {
        //L_Height = Shrink_Height;
        //R_Height = Shrink_Height;
       // RXc[0] = LXc[0] = 0.03;
        JumpOff = 1;
        Jump_Flag = 2;
    }
    else if (JumpTimeCount > Shrink_time && JumpTimeCount <= Cushion_time)
    {
       // L_Height = Cushion_Height;
       // R_Height = Cushion_Height;
       // RXc[0] = LXc[0] = 0.03;
      Jump_Flag = 3;
    }
    else if (JumpTimeCount > Cushion_time && JumpTimeCount <= Over_time)
    {

       // L_Height -= (Cushion_Height - Height_Init)/(Over_time - Cushion_time);
       // R_Height -= (Cushion_Height - Height_Init)/(Over_time - Cushion_time);
       // RXc[0] = LXc[0] = 0.02;

      Cushion_Speed -= (float)Cushion_Height/(Over_time - Cushion_time);
      Jump_Flag = 4;
    }
    else if(JumpTimeCount >= Over_time)
    {
        JumpTimeCount = 0;
        Jump_Flag = 0;
        JumpOff = 0;
        Jump_Finish_Flag = 1;
        Cushion_Speed = 1000;
        // L_Height = Height_Init;
        // R_Height = Height_Init;
    }
}

// 恢�?�到初�?�高�?
void Recover(void)
{
    if (L_Height_Init > Height_Init)
    {
        L_Height_Init-=Cushion_Speed;
        R_Height_Init-=Cushion_Speed;
    }
    else
    {
        L_Height_Init = Height_Init;
        R_Height_Init = Height_Init;
    }
}

// 站立到指定高�?
void Stand(void)
{
    if (L_Height_Init < Stand_Height)
    {
        L_Height_Init+=Cushion_Speed;
        R_Height_Init+=Cushion_Speed;
    }
    else
    {
        L_Height_Init = Stand_Height;
        R_Height_Init = Stand_Height;
    }
}

//void Reset_StartPot(void)
//{
//    if(Last_GPS_State != gps_state && State_get_count < 5)
//    {
//        Last_GPS_State = gps_state;
//        Start_Latitude += gnss.latitude;
//        Start_Longitude += gnss.longitude;
//        State_get_count++;
//    }
//    if(State_get_count == 5)
//    {
//        Start_Latitude /= 5;
//        Start_Longitude /= 5;
//    }
//    Distance = get_two_points_distance(Start_Latitude, Start_Longitude, gnss.latitude, gnss.longitude);
//    if(Distance < 1 && State_get_count == 5)
//    {
//        State_get_count = 0;
//        Now_Dot_Flag++;
//    }
//    else if(Distance >= 1 && State_get_count == 5)
//    {
//        State_get_count = 0;
//        Start_Latitude = 0;
//        Start_Longitude = 0;
//    }
//}

// 重新设置起点（GPS/INS�?
void Reset_StartPot(void)
{
    if(Run_GPS)
    {
        if(Last_GPS_State != gps_state && State_get_count < 5)
        {
            Last_GPS_State = gps_state;
            Start_Latitude += gnss.latitude;
            Start_Longitude += gnss.longitude;
            State_get_count++;
        }
        if(State_get_count == 5)
        {
            Start_Latitude /= 5;
            Start_Longitude /= 5;
        }
        Distance = get_two_points_distance(Start_Latitude, Start_Longitude, gnss.latitude, gnss.longitude);
        if(Distance < 1 && State_get_count == 5)
        {
            State_get_count = 0;
            Now_Dot_Flag++;
        }
        else if(Distance >= 1 && State_get_count == 5)
        {
            State_get_count = 0;
            Start_Latitude = 0;
            Start_Longitude = 0;
        }
    }

    if(Run_INS)
    {
        // Start_X = INSData.Position_x;
        // Start_Y = INSData.Position_y;
        Now_Dot_Flag++;
    }
}

// 设置当前�?标�?�度
void Angle_Setting(void)
{
    // if(Set_Sign[7] == 2)// && ABS(Expect_Angle - IMUData.sum_yaw_mahony) < 0.5)
    // {
    //     Expect_Angle = Set_Angle[Now_Dot] + 720 * Now_Dot * (Set_Angle[Now_Dot] == 0 ? (1) : (-1));
    // }
    // else if(Set_Sign[7] != 2)
    // {
    //     Expect_Angle = Set_Angle[Now_Dot];
    // }
    Expect_Angle = Set_Angle[Now_Dot];
}

float Speed_Compensation = 0;
// 速度控制设置
void Speed_Set(void)
{
    // //速度1.0 减�?
    // if(Distance < 0.3 && fabsf(hCtrl.Pitch.ExpectSpeed_Act) > 550)
    // {
    //     hCtrl.Pitch.ExpectSpeed_Act = 0;
    //     Now_Dot_Flag++;
    // }
    // else if(Distance < 0.2)
    // {
    //     hCtrl.Pitch.ExpectSpeed_Act = 0;
    //     Now_Dot_Flag++;
    // }
    // else if(Distance < 2.0 && fabsf(hCtrl.Pitch.ExpectSpeed_Act) > 550)
    // {
    //     hCtrl.Pitch.ExpectSpeed_Act =  Distance / 2 * hCtrl.Pitch.ExpectSpeed_Exp;
    // }
    // else if(Distance < 1.0)
    // {
    //     hCtrl.Pitch.ExpectSpeed_Act =  Distance * hCtrl.Pitch.ExpectSpeed_Exp;
    // }

    //速度2.0 加反�?补偿
    // if(Distance < 0.2 && fabsf(hCtrl.Pitch.ExpectSpeed_Act) > 550)
    // {
    //     hCtrl.Pitch.ExpectSpeed_Act = 0;
    //     Now_Dot_Flag++;
    // }
    // else if(Distance < 0.1)
    // {
    //     hCtrl.Pitch.ExpectSpeed_Act = 0;
    //     Now_Dot_Flag++;
    // }
    // else if(Distance < 1.5 && fabsf(hCtrl.Pitch.ExpectSpeed_Act) > 550)
    // {
    //     hCtrl.Pitch.ExpectSpeed_Act =  Distance / 2 * hCtrl.Pitch.ExpectSpeed_Exp;
    //     Speed_Compensation = -hCtrl.Pitch.LegOutput * 0.8;
    // }
    // else if(Distance < 0.8)
    // {
    //     hCtrl.Pitch.ExpectSpeed_Act =  Distance * hCtrl.Pitch.ExpectSpeed_Exp;
    //     Speed_Compensation = -hCtrl.Pitch.LegOutput * 1.5;
    // }

    // //速度3.0 尝试ing
    // if(Distance < 1.0 && fabsf(hCtrl.Pitch.ExpectSpeed_Act) > 550)
    // {
    //     hCtrl.Pitch.ExpectSpeed_Act =  Distance * hCtrl.Pitch.ExpectSpeed_Exp;
    //     Speed_Compensation = -hCtrl.Pitch.LegOutput * 2.5;
    //     if(Distance < 0.2)
    //     {
    //         hCtrl.Pitch.ExpectSpeed_Act = 0;
    //         Now_Dot_Flag++;
    //     }
    // }

    //调试连续走，不停
        if(Distance < 0.3)
        {
            // if (Run_SET[1] == 2)
            // {
            //     hCtrl.Pitch.ExpectSpeed_Act = 0;
            // }
            // else
            // {
            //     hCtrl.Pitch.ExpectSpeed_Act = hCtrl.Pitch.ExpectSpeed_Exp / 2;
            // }
            if (Run_SET[1] == 2)
            {
                hCtrl.Pitch.ExpectSpeed_Act = 0;
            }
            else 
            {
                hCtrl.Pitch.ExpectSpeed_Act = hCtrl.Pitch.ExpectSpeed_Exp;
            }
            Now_Dot_Flag++;
        }
    // else if(Distance < 1.0)
    // {
    //     hCtrl.Pitch.ExpectSpeed_Act = hCtrl.Pitch.ExpectSpeed_Exp / 2;
    //     //Now_Dot_Flag++;
    // }
        else 
        {
            if (PWM_Output_Flag == 4)
            {
                hCtrl.Pitch.ExpectSpeed_Act = 420;
            }
            else if (PWM_Output_Flag == 3)
            {
                hCtrl.Pitch.ExpectSpeed_Act = 200;
            }
            else
            {
                hCtrl.Pitch.ExpectSpeed_Act = hCtrl.Pitch.ExpectSpeed_Exp;
            }
        
            // Now_Dot_Flag++;
        }
    // else if(Distance < 2.0 && fabsf(hCtrl.Pitch.ExpectSpeed_Act) > 550)
    // {
    //     hCtrl.Pitch.ExpectSpeed_Act =  Distance / 2 * hCtrl.Pitch.ExpectSpeed_Exp;
    // }
    // else if(Distance < 1.0)
    // {
    //     hCtrl.Pitch.ExpectSpeed_Act =  Distance * hCtrl.Pitch.ExpectSpeed_Exp;
    // }
        // else
        // {
        //     hCtrl.Pitch.ExpectSpeed_Act = hCtrl.Pitch.ExpectSpeed_Exp;
        // }

}

uint32 Element_Flag[8] = {0};
int Roll_Allance = 0;
void Element_Play(void)
{
    if (Element_Flag[Now_Dot - 1] == 2 && Element_Flag[Now_Dot] == 1 && Jump_Finish_Flag != 1)
    {
        JumpCTRL();
        PWM_Output_Flag = 2;
    }
    else if (Element_Flag[Now_Dot - 1] == 3 && Element_Flag[Now_Dot] == 1)
    {
        if (Roll_Allance == 0)
        {
            hCtrl.Roll.BalancePoint = IMUData.roll_mahony;
            Roll_Allance = 1;
        }
        PWM_Output_Flag = 3;
    }
    else if (Element_Flag[Now_Dot - 1] == 4 && Element_Flag[Now_Dot] == 1)
    {
        PWM_Output_Flag = 4;
    }
    else
    {
        PWM_Output_Flag = 1;
    }
}


// �?标点间�?�航与�?�度计算
void P_to_P(void)
{
    if(Run_INS)
    {
        if(Now_Dot < All_Dot)
        {
            float dx = (float)(Set_Guidance_X[Now_Dot] - INSData.Position_x);
            float dy = (float)(Set_Guidance_Y[Now_Dot] - INSData.Position_y);
            float coefficient = (IMUData.sum_yaw_mahony + (180/PI)*atan2(-(Set_Guidance_X[Now_Dot] - INSData.Position_x), Set_Guidance_Y[Now_Dot] - INSData.Position_y)) / fabs(IMUData.sum_yaw_mahony + (180/PI)*atan2(-(Set_Guidance_X[Now_Dot] - INSData.Position_x), Set_Guidance_Y[Now_Dot] - INSData.Position_y));
            Distance = sqrtf(dx * dx + dy * dy);

            //Set_Sign  [Now_Dot] = (180/PI)*atan2(Set_X[Now_Dot + 1] - INSData.Position_x, Set_Y[Now_Dot + 1] - INSData.Position_y) > 0 ? 1 : 0;
            if(Run_SET[1] == 2)
            {
                Set_Angle [Now_Dot] = 720 * (Now_Dot - 1) + ((180/PI)*atan2(-(Set_Guidance_X[Now_Dot] - INSData.Position_x), Set_Guidance_Y[Now_Dot] - INSData.Position_y)) ;
            }
            else
            {
                Set_Angle [Now_Dot] = ((180/PI)*atan2(-(Set_Guidance_X[Now_Dot] - INSData.Position_x), Set_Guidance_Y[Now_Dot] - INSData.Position_y)) ;

                if (fabs(IMUData.sum_yaw_mahony + Set_Angle [Now_Dot]) > 180)
                {
                    Set_Angle [Now_Dot] = -coefficient * 360 + Set_Angle [Now_Dot];
                }
                else
                {
                    Set_Angle [Now_Dot] = Set_Angle [Now_Dot];
                }
            }

            Angle_Setting();
        }
    }

}

// 主运行流程控�?
void Run_Start(void)
{
    static uint8 Segment_Run_Enabled = 0;
    float yaw_err = 0;

    if(Now_Dot_Flag == 0)
    {
        Angle_Setting();
        Now_Dot_Flag++;
    }

    if(Now_Dot_Flag == 1)
    {
        Reset_StartPot();
        Segment_Run_Enabled = 0;
    }

    if(Now_Dot_Flag == 2)
    {
        P_to_P();
        yaw_err = Expect_Angle + IMUData.sum_yaw_mahony;

        if (Run_SET[1] == 2)
        {
            if(Segment_Run_Enabled == 0)
            {
                if(ABS(yaw_err) < 0.5)
                {
                    Segment_Run_Enabled = 1;
                }
                else
                {
                    hCtrl.Pitch.ExpectSpeed_Act = 0;
                }
            }

            if(Segment_Run_Enabled)
            {
                Speed_Set();
            }
        }
        else
        {
            Speed_Set();
        }

    }
    if(Now_Dot_Flag == 3 && Now_Dot < All_Dot)
    {
        Now_Dot_Flag = 0;
        State_get_count = 0;
        Start_Latitude = 0;
        Start_Longitude = 0;
        Segment_Run_Enabled = 0;
        Now_Dot++;
    }
}