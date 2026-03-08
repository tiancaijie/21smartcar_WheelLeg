#include "Compute.h"
/* ========= 全局变量定义 ========== */
CoCtrlHandle_TypeDef CoCtrl = {0};
extern PID_HandleTypeDef PID_Patrol_Line;

/* ========= 内部函数声明 ========== */
static double GetDistance_A2B(coordinate A, coordinate B);
static double GetAngle_A2B(coordinate A, coordinate B);
static double GetAngle_A2B_Line(coordinate A, coordinate B);
static double GetDistance_A2B_Line(coordinate A, coordinate B);

/* ========= 函数定义 ========== */

/******************************************
**Function: 分科目计算角度
**Description: ComAngle
**Input:giSubject
**Output:NULL
**Return:Null
**Others:
******************************************/
void ComAngle(uint32 ExSubject)
{
    static double Angle_INS = 0 ; //惯导计算得出的方位角
    static double Angle_INS_Next = 0 ;
    static double Angle_INS_Last = 0;

    static double Angle_GPS = 0 ; //GPS计算得出的方位角
    static double Angle_GPS_Last = 0;

//    static double SumYaw_Angle_Last;
//
//    static int Count_time = 0;

    Angle_GPS_Last = Angle_GPS;
    Angle_INS_Last = Angle_INS;

    switch (ExSubject)
    {
        case 1:
        {
            if (CoCtrl.Sub1.Sub1_RunMode == GPS)    //科目一GPS模式
            {
                switch (hCtrl.Sub1.Sub1_finish)
                {
                    case 0 :
                    {
                        CoCtrl.Sub1.latitude_Offset = gnss.latitude - CoCtrl.Sub1.Po_Sub1[0].GPS.latitude;
                        CoCtrl.Sub1.longitude_Offset = gnss.longitude - CoCtrl.Sub1.Po_Sub1[0].GPS.longitude;

                        CoCtrl.Sub1.N_Y_Angle_Offset = get_two_points_azimuth(CoCtrl.Sub1.Po_Sub1[0].GPS.latitude, CoCtrl.Sub1.Po_Sub1[0].GPS.longitude, CoCtrl.Sub1.Po_Sub1[1].GPS.latitude, CoCtrl.Sub1.Po_Sub1[1].GPS.longitude);

                        for (int i = 1; i < 4 ; i++)
                        {
                            CoCtrl.Sub1.Po_Sub1[i].GPS.latitude += CoCtrl.Sub1.latitude_Offset;
                            CoCtrl.Sub1.Po_Sub1[i].GPS.longitude += CoCtrl.Sub1.longitude_Offset;
                        }
                        hCtrl.Sub1.Sub1_finish = 1;
                        CoCtrl.Sub1.Po_Goal = 1;
                        break;
                    }
                    //计算补偿

                    case 1:
                    {
                        CoCtrl.Sub1.Distance_Car2Goal = get_two_points_distance(gnss.latitude, gnss.longitude, CoCtrl.Sub1.Po_Sub1[CoCtrl.Sub1.Po_Goal].GPS.latitude, CoCtrl.Sub1.Po_Sub1[CoCtrl.Sub1.Po_Goal].GPS.longitude);
                        if (CoCtrl.Sub1.Distance_Car2Goal < 6)
                        {
                            hCtrl.Sub1.Sub1_finish = 2;
                        }
                        else
                        {
                            Angle_GPS = get_two_points_azimuth(gnss.latitude, gnss.longitude, CoCtrl.Sub1.Po_Sub1[CoCtrl.Sub1.Po_Goal].GPS.latitude, CoCtrl.Sub1.Po_Sub1[CoCtrl.Sub1.Po_Goal].GPS.longitude);
                            Angle_GPS -= CoCtrl.Sub1.N_Y_Angle_Offset;
                        }
                        break;
                    }
                    //直线前行

                    case 2:
                    {
                        CoCtrl.Sub1.Distance_Car2Goal = get_two_points_distance(gnss.latitude, gnss.longitude, CoCtrl.Sub1.Po_Sub1[CoCtrl.Sub1.Po_Goal].GPS.latitude, CoCtrl.Sub1.Po_Sub1[CoCtrl.Sub1.Po_Goal].GPS.longitude);
                        if (CoCtrl.Sub1.Distance_Car2Goal < 3)
                        {
                            hCtrl.Sub1.Sub1_finish = 3;
                        }
                        else
                        {
                            Angle_GPS = get_two_points_azimuth(gnss.latitude, gnss.longitude, CoCtrl.Sub1.Po_Sub1[CoCtrl.Sub1.Po_Goal].GPS.latitude, CoCtrl.Sub1.Po_Sub1[CoCtrl.Sub1.Po_Goal].GPS.longitude);
                            Angle_GPS -= CoCtrl.Sub1.N_Y_Angle_Offset;
                        }
                        break;
                    }
                    //减速，掉头预备

                    case 3:
                    {
                        if(CoCtrl.Sub1.Sub1_BackSide == Left)
                        {
                            Angle_GPS = -IMUData.yaw_mahony - 90;
                        }
                        else if(CoCtrl.Sub1.Sub1_BackSide == Right)
                        {
                            Angle_GPS = -IMUData.yaw_mahony + 90;
                        }

                        if (fabs(IMUData.yaw_mahony) <= 180 && fabs(IMUData.yaw_mahony) >= 170)
                        {
                            CoCtrl.Sub1.Po_Goal++;
                            hCtrl.Sub1.Sub1_finish = 4;
                        }
                        break;
                    }
                    //掉头

                    case 4:
                    {
                        CoCtrl.Sub1.Distance_Car2Goal = get_two_points_distance(gnss.latitude, gnss.longitude, CoCtrl.Sub1.Po_Sub1[CoCtrl.Sub1.Po_Goal].GPS.latitude, CoCtrl.Sub1.Po_Sub1[CoCtrl.Sub1.Po_Goal].GPS.longitude);
                        if (CoCtrl.Sub1.Distance_Car2Goal < 3)
                        {
                            hCtrl.Sub1.Sub1_finish = 5;
                        }
                        else
                        {
                            Angle_GPS = get_two_points_azimuth(gnss.latitude, gnss.longitude, CoCtrl.Sub1.Po_Sub1[CoCtrl.Sub1.Po_Goal].GPS.latitude, CoCtrl.Sub1.Po_Sub1[CoCtrl.Sub1.Po_Goal].GPS.longitude);
                            Angle_GPS -= CoCtrl.Sub1.N_Y_Angle_Offset;
                        }
                        break;
                    }
                    //直线返回

                    case 5:
                    {
                        Angle_GPS = Angle_GPS_Last;
                        break;
                    }
                    //停车

                    default:
                    {
                        Angle_GPS = Angle_GPS_Last;
                        break;
                    }
                  }
                  Angle_INS = Angle_GPS;
            }
            else if (CoCtrl.Sub1.Sub1_RunMode == INS)
            {
                switch (hCtrl.Sub1.Sub1_finish)
                {
                    case 0:
                    {
                        hCtrl.Sub1.Sub1_finish = 1;
                        break;
                    }
                    //初始化

                    case 1:
                    {
                        Angle_INS = GetAngle_A2B(CoCtrl.Car, CoCtrl.Sub1.Po_Sub1[0]);
                        if(CoCtrl.Car.INS.y > CoCtrl.Sub1.Po_Sub1[0].INS.y - CoCtrl.Sub1.Forward_Slow_Distance)
                        {
                            hCtrl.Sub1.Sub1_finish = 2;
                        }
                        break;
                    }
                    //直线前行

                    case 2:
                    {
                        Angle_INS = GetAngle_A2B(CoCtrl.Car, CoCtrl.Sub1.Po_Sub1[0]);
                        if(CoCtrl.Car.INS.y > CoCtrl.Sub1.Po_Sub1[0].INS.y)
                        {
                            hCtrl.Sub1.Sub1_finish = 3;
                        }
                        break;
                    }
                    //减速，掉头预备

                    case 3:
                    {
                        if(CoCtrl.Sub1.Sub1_BackSide == Left)
                        {
                            Angle_INS = -IMUData.yaw_mahony - 90;
                        }
                        else if(CoCtrl.Sub1.Sub1_BackSide == Right)
                        {
                            Angle_INS = -IMUData.yaw_mahony + 90;
                        }

                        if (fabs(IMUData.yaw_mahony) <= 180 && fabs(IMUData.yaw_mahony) >= 160)
                        {
                            hCtrl.Sub1.Sub1_finish = 4;
                        }
                        break;
                    }
                    //掉头

                    case 4:
                    {
                        Angle_INS = 180;
                        if(CoCtrl.Car.INS.y < CoCtrl.Sub1.Po_Sub1[0].INS.y + CoCtrl.Sub1.Back_Slow_Distance)
                        {
                            hCtrl.Sub1.Sub1_finish = 5;
                        }
                        break;
                    }
                    //直走，加速预备

                    case 5:
                    {
                        Angle_INS = 180;
                        if(CoCtrl.Car.INS.y < 0)
                        {
                            hCtrl.Sub1.Sub1_finish = 6;
                        }
                        break;
                    }
                    //直线返回

                    case 6:
                    {
                        Angle_INS = Angle_INS_Last;
                        break;
                    }
                    //停车

                    default:
                    {
                        Angle_INS = Angle_INS_Last;
                        break;
                    }
                }
            }
            break;
        }
        //科目一

        case 2 :
        {
            switch (hCtrl.Sub2.Sub2_finish)
            {
                case 0:
                {
                    Angle_INS = GetAngle_A2B(CoCtrl.Car, CoCtrl.Sub2.Po_Sub2[0]);;
                    if(CoCtrl.Car.INS.y > CoCtrl.Sub2.Po_Sub2[0].INS.y - 200)
                    {
                        hCtrl.Sub2.Sub2_finish = 1;
                    }
                    break;
                }//靠近

                case 1:
                {
                    if(CoCtrl.Sub2.Line_Goal < (CoCtrl.Sub2.Sub2_EndLine + 1)/2.0)
                    {
                        Angle_INS = GetAngle_A2B_Line(CoCtrl.Sub2.Po_Sub2[2*CoCtrl.Sub2.Line_Goal], CoCtrl.Sub2.Po_Sub2[2*CoCtrl.Sub2.Line_Goal+1]);
                        CoCtrl.Sub2.Distance_Car2Goal = GetDistance_A2B_Line(CoCtrl.Sub2.Po_Sub2[2*CoCtrl.Sub2.Line_Goal], CoCtrl.Sub2.Po_Sub2[2*CoCtrl.Sub2.Line_Goal+1]);
                        if(CoCtrl.Sub2.Distance_Car2Goal < CoCtrl.Sub2.Change_Distance)
                        {
                            CoCtrl.Sub2.Line_Goal++;
                        }
                    }
                    else
                    {
                        hCtrl.Sub2.Sub2_finish = 2;
                    }
                    break;
                }
                //绕桩

                case 2:
                {
                    Angle_INS = 0;
                    if (fabs(IMUData.yaw_mahony) <= 10)
                    {
                        hCtrl.Sub2.Sub2_finish = 3;
                    }
                    break;
                }//转向，加速预备

                case 3:
                {
                    Angle_INS = 0;
                    if(CoCtrl.Car.INS.y > CoCtrl.Sub2.Po_Sub2[196].INS.y - 200)
                    {
                        hCtrl.Sub2.Sub2_finish = 4;
                    }
                    break;
                }//加速直线


                case 4:
                {
                    Angle_INS = 0;
                    if(CoCtrl.Car.INS.y > CoCtrl.Sub2.Po_Sub2[196].INS.y)
                    {
                        hCtrl.Sub2.Sub2_finish = 5;
                    }
                    break;
                }//减速,掉头预备

                case 5:
                {
                    Angle_INS_Next = GetAngle_A2B(CoCtrl.Car, CoCtrl.Sub2.Po_Sub2[2*CoCtrl.Sub2.Line_Goal]);
                    if (fabs(IMUData.yaw_mahony + Angle_INS_Next) <= 10)
                    {
                        hCtrl.Sub2.Sub2_finish = 6;
                    }
                    if(CoCtrl.Sub2.Sub2_BackSide == Left)
                    {
                        Angle_INS = -IMUData.yaw_mahony - 90;
                    }
                    else if(CoCtrl.Sub2.Sub2_BackSide == Right)
                    {
                        Angle_INS = -IMUData.yaw_mahony + 90;
                    }
                    break;
                }//减速掉头

                case 6:
                {
                    Angle_INS = GetAngle_A2B(CoCtrl.Car, CoCtrl.Sub2.Po_Sub2[2*CoCtrl.Sub2.Line_Goal]);;
                    if(CoCtrl.Car.INS.y < CoCtrl.Sub2.Po_Sub2[2*CoCtrl.Sub2.Line_Goal].INS.y + 200)
                    {
                        hCtrl.Sub2.Sub2_finish = 7;
                    }
                    break;
                }//加速直线回

                case 7:
                {
                    if(CoCtrl.Sub2.Line_Goal < CoCtrl.Sub2.Sub2_EndLine + 1)
                    {
                        Angle_INS = GetAngle_A2B_Line(CoCtrl.Sub2.Po_Sub2[2*CoCtrl.Sub2.Line_Goal], CoCtrl.Sub2.Po_Sub2[2*CoCtrl.Sub2.Line_Goal+1]);
                        CoCtrl.Sub2.Distance_Car2Goal = GetDistance_A2B_Line(CoCtrl.Sub2.Po_Sub2[2*CoCtrl.Sub2.Line_Goal], CoCtrl.Sub2.Po_Sub2[2*CoCtrl.Sub2.Line_Goal+1]);
                        if(CoCtrl.Sub2.Distance_Car2Goal < CoCtrl.Sub2.Change_Distance)
                        {
                            CoCtrl.Sub2.Line_Goal++;
                        }
                    }
                    else
                    {
                        hCtrl.Sub2.Sub2_finish = 8;
                    }
                    break;
                }//绕桩

                case 8:
                {
                    Angle_INS = 180;
                    if (fabs(IMUData.yaw_mahony) <= 180 && fabs(IMUData.yaw_mahony) >= 170)
                    {
                        hCtrl.Sub2.Sub2_finish = 9;
                    }
                    break;
                }//转向,加速预备

                case 9:
                {
                    Angle_INS = 180;
                    if(CoCtrl.Car.INS.y < 0)
                    {
                        hCtrl.Sub2.Sub2_finish = 10;
                    }
                    break;
                }//直线回

                case 10:
                {
                    Angle_INS = Angle_INS_Last;
                    break;
                }//停车

                default:
                {
                    Angle_INS = Angle_INS_Last;
                    break;
                }

            }
            break;
        }//科目二

        case 3:
        {
            switch (hCtrl.Sub3.Sub3_finish)
            {
                case 0:
                {
                    Angle_INS = GetAngle_A2B(CoCtrl.Car, CoCtrl.Sub3.Po_Sub3[0]);
                    if(CoCtrl.Car.INS.y > CoCtrl.Sub3.Po_Sub3[0].INS.y - 300)
                    {
                        hCtrl.Sub3.Sub3_finish = 1;
                    }
                    break;
                }//靠近元素

                case 1:
                {
                    if(CoCtrl.Sub3.Line_Goal == CoCtrl.Sub3.Back_EL)
                    {
                        hCtrl.Sub3.Sub3_finish = 6;
                    }
                    else if(CoCtrl.Sub3.Line_Goal == CoCtrl.Sub3.Slope_EL)
                    {
                        hCtrl.Sub3.Sub3_finish = 2;
                    }
                    else if(CoCtrl.Sub3.Line_Goal == CoCtrl.Sub3.Grass_EL)
                    {
                        hCtrl.Sub3.Sub3_finish = 3;
                    }
                    else if(CoCtrl.Sub3.Line_Goal == CoCtrl.Sub3.Jolt_EL)
                    {
                        hCtrl.Sub3.Sub3_finish = 4;
                    }
                    else if(CoCtrl.Sub3.Line_Goal == CoCtrl.Sub3.Bridge_EL)
                    {
                        hCtrl.Sub3.Sub3_finish = 5;
                    }
                    else if(CoCtrl.Sub3.Line_Goal < CoCtrl.Sub3.Sub3_EndLine + 1)
                    {
                        Angle_INS = GetAngle_A2B_Line(CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal], CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal+1]);
                        CoCtrl.Sub3.Distance_Car2Goal = GetDistance_A2B_Line(CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal], CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal+1]);
                        if(CoCtrl.Sub3.Distance_Car2Goal < CoCtrl.Sub3.Change_Distance)
                        {
                            CoCtrl.Sub3.Line_Goal++;
                        }
                    }

                    break;
                }//过元素前

                case 2:
                {
                    Angle_INS = GetAngle_A2B_Line(CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal], CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal+1]);
                    CoCtrl.Sub3.Distance_Car2Goal = GetDistance_A2B_Line(CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal], CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal+1]);
                    if(CoCtrl.Sub3.Distance_Car2Goal < CoCtrl.Sub3.Change_Distance)
                    {
                      CoCtrl.Sub3.Line_Goal++;
                      hCtrl.Sub3.Sub3_finish = 1;
                    }
                    break;
                }//过坡道元素中

                case 3:
                {
                    Angle_INS = GetAngle_A2B_Line(CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal], CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal+1]);
                    CoCtrl.Sub3.Distance_Car2Goal = GetDistance_A2B_Line(CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal], CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal+1]);
                    if(CoCtrl.Sub3.Distance_Car2Goal < CoCtrl.Sub3.Change_Distance)
                    {
                      CoCtrl.Sub3.Line_Goal++;
                      hCtrl.Sub3.Sub3_finish = 1;
                    }
                    break;
                }//过草地中

                case 4:
                {
                    Angle_INS = GetAngle_A2B_Line(CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal], CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal+1]);
                    CoCtrl.Sub3.Distance_Car2Goal = GetDistance_A2B_Line(CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal], CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal+1]);
                    if(CoCtrl.Sub3.Distance_Car2Goal < CoCtrl.Sub3.Change_Distance)
                    {
                      CoCtrl.Sub3.Line_Goal++;
                      hCtrl.Sub3.Sub3_finish = 1;
                    }
                    break;
                }//过颠簸路段中

                case 5:
                {
                    Angle_INS = GetAngle_A2B_Line(CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal], CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal+1]);
                    CoCtrl.Sub3.Distance_Car2Goal = GetDistance_A2B_Line(CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal], CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal+1]);
                    if(CoCtrl.Sub3.Distance_Car2Goal < CoCtrl.Sub3.Change_Distance)
                    {
                      CoCtrl.Sub3.Line_Goal++;
                      hCtrl.Sub3.Sub3_finish = 1;
                    }
                    break;
                }//过桥洞中

                case 6:
                {
                    Angle_INS = 0;
                    if (fabs(IMUData.yaw_mahony) <= 10)
                    {
                        hCtrl.Sub3.Sub3_finish = 7;
                    }
                    break;
                }//转向，加速预备

                case 7:
                {
                    Angle_INS = 0;
                    if(CoCtrl.Car.INS.y > CoCtrl.Sub3.Po_Sub3[16].INS.y - 200)
                    {
                        hCtrl.Sub3.Sub3_finish = 8;
                    }
                    break;
                }//直线加速

                case 8:
                {
                    Angle_INS = 0;
                    if(CoCtrl.Car.INS.y > CoCtrl.Sub3.Po_Sub3[16].INS.y)
                    {
                        hCtrl.Sub3.Sub3_finish = 9;
                    }
                    break;
                }//减速，掉头预备

                case 9:
                {
                    Angle_INS_Next = GetAngle_A2B(CoCtrl.Car, CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal]);
                    if (fabs(IMUData.yaw_mahony + Angle_INS_Next) <= 10)
                    {
                        hCtrl.Sub3.Sub3_finish = 10;
                    }

                    if(CoCtrl.Sub3.Sub3_BackSide == Left)
                    {
                        Angle_INS = -IMUData.yaw_mahony - 90;
                    }
                    else if(CoCtrl.Sub3.Sub3_BackSide == Right)
                    {
                        Angle_INS = -IMUData.yaw_mahony + 90;
                    }
                    break;
                }//掉头

                case 10:
                {
                    Angle_INS = GetAngle_A2B(CoCtrl.Car, CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal]);
                    if(CoCtrl.Car.INS.y < CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal].INS.y + 200)
                    {
                        hCtrl.Sub3.Sub3_finish = 11;
                    }
                    break;
                }//靠近元素

                case 11:
                {
                    if(CoCtrl.Sub3.Line_Goal == CoCtrl.Sub3.Slope_EL)
                    {
                        hCtrl.Sub3.Sub3_finish = 12;
                    }
                    else if(CoCtrl.Sub3.Line_Goal == CoCtrl.Sub3.Grass_EL)
                    {
                        hCtrl.Sub3.Sub3_finish = 13;
                    }
                    else if(CoCtrl.Sub3.Line_Goal == CoCtrl.Sub3.Jolt_EL)
                    {
                        hCtrl.Sub3.Sub3_finish = 14;
                    }
                    else if(CoCtrl.Sub3.Line_Goal == CoCtrl.Sub3.Bridge_EL)
                    {
                        hCtrl.Sub3.Sub3_finish = 15;
                    }
                    else if(CoCtrl.Sub3.Line_Goal < CoCtrl.Sub3.Sub3_EndLine + 1)
                    {
                        Angle_INS = GetAngle_A2B_Line(CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal], CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal+1]);
                        CoCtrl.Sub3.Distance_Car2Goal = GetDistance_A2B_Line(CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal], CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal+1]);
                        if(CoCtrl.Sub3.Distance_Car2Goal < CoCtrl.Sub3.Change_Distance)
                        {
                            CoCtrl.Sub3.Line_Goal++;
                        }
                    }
                    else if (CoCtrl.Sub3.Line_Goal > CoCtrl.Sub3.Sub3_EndLine)
                    {
                        hCtrl.Sub3.Sub3_finish = 16;
                    }
                    break;
                }//过元素前

                case 12:
                {
                    Angle_INS = GetAngle_A2B_Line(CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal], CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal+1]);
                    CoCtrl.Sub3.Distance_Car2Goal = GetDistance_A2B_Line(CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal], CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal+1]);
                    if(CoCtrl.Sub3.Distance_Car2Goal < CoCtrl.Sub3.Change_Distance)
                    {
                      CoCtrl.Sub3.Line_Goal++;
                      hCtrl.Sub3.Sub3_finish = 11;
                    }
                    break;
                }//过坡道中

                case 13:
                {
                    Angle_INS = GetAngle_A2B_Line(CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal], CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal+1]);
                    CoCtrl.Sub3.Distance_Car2Goal = GetDistance_A2B_Line(CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal], CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal+1]);
                    if(CoCtrl.Sub3.Distance_Car2Goal < CoCtrl.Sub3.Change_Distance)
                    {
                      CoCtrl.Sub3.Line_Goal++;
                      hCtrl.Sub3.Sub3_finish = 11;
                    }
                    break;
                }//过草地中

                case 14:
                {
                    Angle_INS = GetAngle_A2B_Line(CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal], CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal+1]);
                    CoCtrl.Sub3.Distance_Car2Goal = GetDistance_A2B_Line(CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal], CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal+1]);
                    if(CoCtrl.Sub3.Distance_Car2Goal < CoCtrl.Sub3.Change_Distance)
                    {
                      CoCtrl.Sub3.Line_Goal++;
                      hCtrl.Sub3.Sub3_finish = 11;
                    }
                    break;
                }//过颠簸路段中

                case 15:
                {
                    Angle_INS = GetAngle_A2B_Line(CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal], CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal+1]);
                    CoCtrl.Sub3.Distance_Car2Goal = GetDistance_A2B_Line(CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal], CoCtrl.Sub3.Po_Sub3[2*CoCtrl.Sub3.Line_Goal+1]);
                    if(CoCtrl.Sub3.Distance_Car2Goal < CoCtrl.Sub3.Change_Distance)
                    {
                      CoCtrl.Sub3.Line_Goal++;
                      hCtrl.Sub3.Sub3_finish = 11;
                    }
                    break;
                }//过桥洞中

                case 16:
                {
                    Angle_INS = 180;
                    if (fabs(IMUData.yaw_mahony) <= 180 && fabs(IMUData.yaw_mahony) >= 170)
                    {
                        hCtrl.Sub3.Sub3_finish = 17;
                    }

                    break;
                }//转向，加速预备

                case 17:
                {
                    Angle_INS = 180;
                    if(CoCtrl.Car.INS.y < 0)
                    {
                        hCtrl.Sub3.Sub3_finish = 18;
                    }
                    break;
                }//加速回

                case 18:
                {
                    Angle_INS = Angle_INS_Last;
                    break;
                }//停车

                default:
                {
                    Angle_INS = Angle_INS_Last;
                    break;
                }
            }
            break;
        }//科目三
    }

    hCtrl.AngleDiffer = IMUData.yaw_mahony + Angle_INS;
    while(hCtrl.AngleDiffer > 180)
    {
        hCtrl.AngleDiffer -= 360;
    }
    while(hCtrl.AngleDiffer < -180)
    {
        hCtrl.AngleDiffer += 360;
    }

    hCtrl.Com_finish = 1;
}

/******************************************
**Function: 分情况设置期望速度
**Description: ComSpeed
**Input:giSubject
**Output:NULL
**Return:Null
**Others:
******************************************/
void ComSpeed(uint32 ExSubject)
{
    switch(ExSubject)
    {
        case 1:
        {
            switch(hCtrl.Sub1.Sub1_finish)
            {
                case 0:
                {
                    hCtrl.SpeedTarget = 0;
                    break;
                }
                //初始化

                case 1:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub1.ExSp_S1_H;
                    break;
                }
                //直线

                case 2:
                {
                    while (hCtrl.SpeedTarget > hCtrl.Sub1.ExSp_S1_L)
                    {
                        hCtrl.SpeedTarget -= CoCtrl.Sub1.Speed_Change_K;
                        if (hCtrl.SpeedTarget < hCtrl.Sub1.ExSp_S1_L)
                        {
                            hCtrl.SpeedTarget = hCtrl.Sub1.ExSp_S1_L;
                        }
                    }
                    break;
                }//减速，掉头预备

                case 3:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub1.ExSp_S1_L;
                    break;
                }
                //减速掉头

                case 4:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub1.ExSp_S1_L;
                    break;
                }//直走，加速预备

                case 5:
                {
                    while (hCtrl.SpeedTarget < hCtrl.Sub1.ExSp_S1_H)
                    {
                        hCtrl.SpeedTarget += CoCtrl.Sub1.Speed_Change_K;
                        if (hCtrl.SpeedTarget > hCtrl.Sub1.ExSp_S1_H)
                        {
                            hCtrl.SpeedTarget = hCtrl.Sub1.ExSp_S1_H;
                        }
                    }
                    break;
                }
                //回

                case 6:
                {
                    hCtrl.SpeedTarget = 0;
                    break;
                }
                //停车

                default:
                {
                    hCtrl.SpeedTarget = 0;
                }
            }
            break;
        }
        //科目一

        case 2:
        {
            switch(hCtrl.Sub2.Sub2_finish)
            {
                case 0:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub2.ExSp_S2_H;
                    break;
                }//靠近

                case 1:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub2.ExSp_S2_M;
                    break;
                }//绕桩

                case 2:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub2.ExSp_S2_M;
                    break;
                }//转向，加速预备

                case 3:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub2.ExSp_S2_H;
                    break;
                }//加速直线

                case 4:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub2.ExSp_S2_L;
                    break;
                }
                //减速，掉头预备

                case 5:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub2.ExSp_S2_L;
                    break;
                }//掉头

                case 6:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub2.ExSp_S2_H;
                    break;
                }//加速直线回

                case 7:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub2.ExSp_S2_M;
                    break;
                }//绕桩

                case 8:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub2.ExSp_S2_M;
                    break;
                }//转向，加速预备

                case 9:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub1.ExSp_S1_H;
                    break;
                }//直线回

                case 10:
                {
                    hCtrl.SpeedTarget = 0;
                    break;
                }//停车

                default:
                {
                    hCtrl.SpeedTarget = 0;
                }

            }
            break;
        }//科目二


        case 3:
        {
            switch(hCtrl.Sub3.Sub3_finish)
            {
                case 0:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub3.ExSp_S3_H;
                    break;
                }//靠近元素

                case 1:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub3.ExSp_S3_M;
                    break;
                }//过元素前

                case 2:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub3.ExSp_S3_Slope;
                    break;
                }//过坡道元素中

                case 3:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub3.ExSp_S3_Grass;
                    break;
                }//过草地元素中
                case 4:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub3.ExSp_S3_Jolt;
                    break;
                }//过颠簸路段中
                case 5:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub3.ExSp_S3_Bridge;
                    break;
                }//过桥洞中

                case 6:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub3.ExSp_S3_M;
                    break;
                }//转向，加速预备

                case 7:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub3.ExSp_S3_H;
                    break;
                }//加速

                case 8:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub3.ExSp_S3_Back;
                    break;
                }//减速，预备掉头

                case 9:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub3.ExSp_S3_Back;
                    break;
                }//掉头

                case 10:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub3.ExSp_S3_H;
                    break;
                }//靠近元素

                case 11:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub3.ExSp_S3_M;
                    break;
                }//过元素前

                case 12:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub3.ExSp_S3_Slope;
                    break;
                }//过坡道元素中

                case 13:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub3.ExSp_S3_Grass;
                    break;
                }//过草地元素中

                case 14:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub3.ExSp_S3_Jolt;
                    break;
                }//过颠簸路段中

                case 15:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub3.ExSp_S3_Bridge;
                    break;
                }//过桥洞中

                case 16:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub3.ExSp_S3_M;
                    break;
                }//转向，加速预备

                case 17:
                {
                    hCtrl.SpeedTarget = hCtrl.Sub1.ExSp_S1_H;
                    break;
                }//加速回

                case 18:
                {
                    hCtrl.SpeedTarget = 0;
                    break;
                }//停车

                default:
                {
                    hCtrl.SpeedTarget = 0;
                }
            }
            break;
        }//科目三
        default:
        {
            hCtrl.SpeedTarget = 0;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 计算两点的距离
// 参数说明
// 参数说明
// 返回参数 两点的距离
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
static double GetDistance_A2B(coordinate A, coordinate B)
{
    double Distance = 0;

    Distance = pow(B.INS.x - A.INS.x, 2) + pow(B.INS.y - A.INS.y, 2);
    Distance = sqrt(Distance);

    return Distance;
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介 计算两点的角度
// 参数说明
// 参数说明
// 返回参数 方位角
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
static double GetAngle_A2B(coordinate A, coordinate B)
{
    double Angle = 0;
    Angle = (180/PI)*atan2(B.INS.x - A.INS.x, B.INS.y - A.INS.y);
    return Angle;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 计算从A点到B点的目标方位角（巡AB直线）
// 参数说明
// 参数说明
// 返回参数 方位角
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
static double GetAngle_A2B_Line(coordinate A, coordinate B)
{
    static double A2B_Angle;
    static double A2Car_Angle;
    static double Car_A_B_Angle;
    static double Angle_Err;
    static double Ex_Angle;

    static double A2Car_Distance;
    static double Car_ABLine_Tangential;

    double Vy = 100;
    double Vx = 0;

    A2B_Angle = GetAngle_A2B(A, B);
    A2Car_Angle = GetAngle_A2B(A, CoCtrl.Car);
    Car_A_B_Angle = A2B_Angle - A2Car_Angle;

    A2Car_Distance = GetDistance_A2B(A, CoCtrl.Car);
    Car_ABLine_Tangential = A2Car_Distance*sin((PI/180)*Car_A_B_Angle);

    Vx = -PID_calc(&PID_Patrol_Line,  0.0, (float)Car_ABLine_Tangential);

    Angle_Err = (180/PI)*atan2(Vx, Vy);

    Ex_Angle = A2B_Angle + Angle_Err;

    return Ex_Angle;
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介 计算车到B点的距离（巡AB直线）
// 参数说明
// 参数说明
// 返回参数 距离
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
static double GetDistance_A2B_Line(coordinate A, coordinate B)
{
    static double A2B_Angle;
    static double A2Car_Angle;
    static double Car_A_B_Angle;

    static double A2B_Distance;
    static double A2Car_Distance;
    static double Car_ABLine_Normal;

    A2B_Distance = GetDistance_A2B(A, B);
    A2Car_Distance = GetDistance_A2B(A, CoCtrl.Car);

    A2B_Angle = GetAngle_A2B(A, B);
    A2Car_Angle = GetAngle_A2B(A, CoCtrl.Car);
    Car_A_B_Angle = A2B_Angle - A2Car_Angle;

    Car_ABLine_Normal = A2B_Distance - A2Car_Distance*cos((PI/180)*Car_A_B_Angle);

    return Car_ABLine_Normal;
}