/*
 * Element.c
 *
 *  Created on: 2025Äê5ÔÂ13ÈÕ
 *      Author: Zzio
 */
#include "zf_common_headfile.h"
#include "Element.h"
#include "isr.h"
#include "control.h"

float Expect_Angle = 0;
float Row_LegOffset[2] = {0};
float Single_Angle = 10;
uint8 Now_Dot = 0;
uint8 All_Dot = 8;
uint8 Now_Dot_Flag = 0;
uint8 Last_GPS_State = 0;
uint8 State_get_count = 0;
int Single_Bridge = 0;
int BridgeUP_Flag = 0,BridgeDOWN_Flag = 0;

uint8 Run_GPS = 0;
uint8 Run_INS = 1;

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

extern PID_HandleTypeDef PID_RowOmega;
extern PID_HandleTypeDef PID_RowAngle;
float Stand_Height = 0.055;
float L_Height_Init = 0.028;//0.052;
float R_Height_Init = 0.028;//0.052;
float Roll_LStep = 0.00018;
float Accumulation = 0;
void Single_BridgeStateCTRL(void)
{

}

int JumpTimeCount = 0;
int JumpOff = 0;
uint16 Extend_time = 80;
uint16 Shrink_time = 200;
uint16 Cushion_time = 240;
uint16 Over_time = 320;
float Extend_Height = 0.130;
float Shrink_Height = 0.075;
float Cushion_Height = 0.110;
float Cushion_Speed  = 0.00025;
void JumpCTRL(void)
{
    JumpTimeCount+=2;
    if (Extend_time >= JumpTimeCount)
    {
        L_Height = Extend_Height;
        R_Height = Extend_Height;
    }
    else if (Shrink_time >= JumpTimeCount && JumpTimeCount > Extend_time)
    {
        L_Height = Shrink_Height;
        R_Height = Shrink_Height;
        RXc[0] = LXc[0] = 0.03;
        JumpOff = 1;
    }
    else if (JumpTimeCount > Shrink_time && JumpTimeCount <= Cushion_time)
    {
        L_Height = Cushion_Height;
        R_Height = Cushion_Height;
        RXc[0] = LXc[0] = 0.03;
    }
    else if (JumpTimeCount > Cushion_time && JumpTimeCount <= Over_time)
    {

        L_Height -= (Cushion_Height - Height_Init)/(Over_time - Cushion_time);
        R_Height -= (Cushion_Height - Height_Init)/(Over_time - Cushion_time);
        RXc[0] = LXc[0] = 0.02;
    }
    else if(JumpTimeCount >= Over_time)
    {
        JumpTimeCount = 0;
        Jump_Flag = 0;
        JumpOff = 0;
        L_Height = Height_Init;
        R_Height = Height_Init;
    }
}

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
        Start_X = INSData.Position_x;
        Start_Y = INSData.Position_y;
        Now_Dot_Flag++;
    }
}

void Angle_Set(void)
{
//    static int8 Last_Dot = 0;
//    if(Set_Sign[7] == 2 && Last_Dot != Now_Dot)
//    {
//      Last_Dot = Now_Dot;
//      Expect_Angle = IMUData.sum_yaw_mahony + 2*360;
//    }
    if(Set_Sign[7] == 2)// && ABS(Expect_Angle - IMUData.sum_yaw_mahony) < 0.5)
    {
      Expect_Angle = 720 *Now_Dot  + (float)Set_Angle[Now_Dot] * (Set_Sign[Now_Dot] == 0 ? (1):(-1));
      Now_Dot_Flag++;
    }
    else if(Set_Sign[7] != 2)
    {
      Expect_Angle = (float)Set_Angle[Now_Dot] * (Set_Sign[Now_Dot] == 0 ? (1):(-1));
      Now_Dot_Flag++;
    }
}

//void Speed_Set(void)
//{
//    if(Distance > Set_Length[Now_Dot])
//    {
//      hCtrl.Pitch.ExpectSpeed_Act = 0;
//      Now_Dot_Flag++;
//    }
//    else
//    {
//      hCtrl.Pitch.ExpectSpeed_Act = hCtrl.Pitch.ExpectSpeed_Exp;
//    }
//}

void Speed_Set(void)
{
    if(Run_GPS)
    {
        if(Distance > Set_Length[Now_Dot])
        {
            hCtrl.Pitch.ExpectSpeed_Act = 0;
            Now_Dot_Flag++;
        }
        else
        {
            hCtrl.Pitch.ExpectSpeed_Act = hCtrl.Pitch.ExpectSpeed_Exp;
        }
    }
    
    if(Run_INS)
    {
        if(Distance > Set_Length[Now_Dot])
        {
            hCtrl.Pitch.ExpectSpeed_Act = 0;
            Now_Dot_Flag++;
        }
        else
        {
            hCtrl.Pitch.ExpectSpeed_Act = hCtrl.Pitch.ExpectSpeed_Exp;
        }
    }
}

void Run_Start(void)
{   
    if(Run_INS)
    {
        float dx = (float)(Start_X - INSData.Position_x);
        float dy = (float)(Start_Y - INSData.Position_y);
        Distance = sqrtf(dx * dx + dy * dy);
    }

    if(Now_Dot_Flag == 0)
    {
      Angle_Set();
    }
    if(Now_Dot_Flag == 1)
    {
      Reset_StartPot();
    }
    if(Now_Dot_Flag == 2 && ABS(Expect_Angle + IMUData.sum_yaw_mahony) < 0.1)
    {
      Speed_Set();
    }
    if(Now_Dot_Flag == 3 && Now_Dot < All_Dot)
    {
      Now_Dot_Flag = 0;
      State_get_count = 0;
      Start_Latitude = 0;
      Start_Longitude = 0;
      Now_Dot++;
    }
}