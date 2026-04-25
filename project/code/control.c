#include "zf_common_headfile.h"
#include "MahonyAHRS.h"
#include "small_driver_uart_control.h"
#include "isr.h"

CtrlHandle_TypeDef hCtrl = {
                            .Pitch.ExpectOmega = 0.0,
                            .Pitch.BalancePoint = 0.0,
                            .Pitch.ExpectSpeed_Act = 0,
                            .Pitch.ExpectSpeed_Exp = 0
                            };
PID_HandleTypeDef PID_PitchOmega;
PID_InitTypeDef   PID_PitchOmega_init=
{
    .name = "Pitch_Omega",
    .index = 1,
    .mode = PID_MODE_POSITION,
    .kp = 875,//805.0,
    .factor_kp = 1.0,
    .ki = 28,//42.0,
    .factor_ki = 1.0,
    .kd = 1200,//435.0,
    .factor_kd = 1.0,
//    .factor_iOutMax = 1.0,
//    .factor_outMax = 1.0,
    .iOutMax = 2000,
    .outMax  = 7000,
};

PID_HandleTypeDef PID_PitchAngle;
PID_InitTypeDef   PID_PitchAngle_init=
{
    .name = "Pitch_Angle",
    .index = 2,
    .mode = PID_MODE_POSITION,
    .kp = 9.5,//10.5,
    .factor_kp = 1,
    .ki = 0,//1.5,
    .factor_ki = 1,
    .kd = 28.5,//8.5,
    .factor_kd = 1,
//    .factor_iOutMax = 1.0e-3,
//    .factor_outMax  = 1.0e-3,
    .iOutMax = 1.5,
    .outMax  = 5,
};

PID_HandleTypeDef PID_PitchLeg;
PID_InitTypeDef   PID_PitchLeg_init=
{
    .name = "Pitch_Leg",
    .index = 4,
    .mode = PID_MODE_POSITION,
    .kp = 0,
    .factor_kp = 1,
    .ki = 0,
    .factor_ki = 1,
    .kd = 0,
    .factor_kd = 1,
    .iOutMax = 10000,
    .outMax  = 10000,
};

PID_HandleTypeDef PID_YawAngle;
PID_InitTypeDef   PID_YawAngle_init=
{
    .name = "Yaw_Omega",
    .index = 7,
    .mode = PID_MODE_POSITION,
    .kp = 7.0,
    .factor_kp = 1.0,
    .ki = 0.0,
    .factor_ki = 1.0,
    .kd = 15.0,
    .factor_kd = 1.0,
    .iOutMax = 0,
    .outMax  = 6,
};


PID_HandleTypeDef PID_YawOmega;
PID_InitTypeDef   PID_YawOmega_init=
{
    .name = "Yaw_Omega",
    .index = 7,
    .mode = PID_MODE_POSITION,
    .kp = 975.0,
    .factor_kp = 1.0,
    .ki = 0.0,
    .factor_ki = 1.0,
    .kd = 225.0,
    .factor_kd = 1.0,
    .iOutMax = 1000,
    .outMax  = 7000,
};

PID_HandleTypeDef PID_RollAngle;
PID_InitTypeDef   PID_RollAngle_init=
{
    .name = "Roll_Angle",
    .index = 7,
    .mode = PID_MODE_ADD,
    .kp = 100.0,
    .factor_kp = 1.0,
    .ki = 0.000,
    .factor_ki = 1.0,
    .kd = 0.95,
    .factor_kd = 1.0,
    .iOutMax = 0.01,
    .outMax  = 2000,
};


PID_HandleTypeDef PID_SwerveAngle;
PID_InitTypeDef   PID_SwerveAngle_init=
{
    .name = "Roll_Angle",
    .index = 7,
    .mode = PID_MODE_POSITION,
    .kp = 0.085,
    .factor_kp = 1.0,
    .ki = 0,
    .factor_ki = 1.0,
    .kd = 0.195,
    .factor_kd = 1.0,
    .iOutMax = 0.055,
    .outMax  = 0.055,
};

PID_HandleTypeDef PID_Patrol_Line;

void PID_InitAll(void)
{
    // 初始化所有PID参数
    PID_init(&PID_PitchLeg, &PID_PitchLeg_init);
    PID_init(&PID_PitchOmega, &PID_PitchOmega_init);
    PID_init(&PID_PitchAngle, &PID_PitchAngle_init);
    PID_init(&PID_RollAngle, &PID_RollAngle_init);
    PID_init(&PID_SwerveAngle, &PID_SwerveAngle_init);
    PID_init(&PID_YawOmega, &PID_YawOmega_init);
    PID_init(&PID_YawAngle, &PID_YawAngle_init);
}

float filter_b_coef = 1;
double filter_b[N] = {0.005542717210281,0.011085434420561,0.005542717210281};
double filter_a[N] = {1,-1.778631777824585,0.800802646665708};
void filter(float input_new, float *input, float *output,double *filter_b, double *filter_a,uint8 M)//IIR低通滤波
{
    int i;
    float sum_input = 0;
    float sum_output = 0;

    for (i = M - 1; i > 0; i--)
    {
        input[i] = input[i - 1];
        output[i] = output[i - 1];
    }
    input[0] = input_new;

    for (i = 0; i < M; i++)
    {
       sum_input += input[i] * filter_b[i];
       if (i > 0)
       {
           sum_output += output[i] * filter_a[i];
       }
    }
    output[0] = filter_b_coef * sum_input - sum_output;
}


SpeedExp_DEFINE SpeedExp;
Flag_ON_DEFINE Flag_ON;

float gfAngle;
float gfAngleSpeed = 0;
float gfAngleSpeed_I = 0;
float gfAngle_Acc[2] = {0};

float gx_last,gy_last,gz_last,ax_last,ay_last,az_last; // IMU数据上一次值
void Angle_cal(void)
{
//    icm20602_get_acc(); // 备用IMU获取加速度
//    icm20602_get_gyro(); // 备用IMU获取角速度
//    icm20602_data_trans(); // 备用IMU数据转换
    imu963ra_get_acc();
    imu963ra_get_gyro();
    imu963_data_trans();
    /* IMU数据融合 */

    // 对IMU原始数据进行一阶低通滤波，减少噪声
    ax_last = ax_last*0.9 + IMUData.ax * 0.1;
    ay_last = ay_last*0.9 + IMUData.ay * 0.1;
    az_last = az_last*0.9 + IMUData.az * 0.1;

    gx_last = gx_last*0.85 + IMUData.gx * 0.15;
    gy_last = gy_last*0.85 + IMUData.gy * 0.15;
    gz_last = gz_last*0.85 + IMUData.gz * 0.15;

    IMUData.yaw_mahony_Last = IMUData.yaw_trans;

    MahonyAHRSupdateIMU(&IMUData.q[0],IMUData.gx,IMUData.gy,IMUData.gz,ax_last,ay_last,az_last);        // Mahony算法姿态解算，更新IMUData.q
    get_angle(&IMUData.q[0],&IMUData.yaw_mahony,&IMUData.pitch_mahony,&IMUData.roll_mahony);            // 四元数转欧拉角，得到yaw、pitch、roll

    // yaw角度归一化到[-PI, PI]
    if((IMUData.yaw_mahony - hCtrl.Yaw.BalancePoint) <= PI && (IMUData.yaw_mahony - hCtrl.Yaw.BalancePoint) >= -PI)
    {
        IMUData.yaw_trans = (IMUData.yaw_mahony - hCtrl.Yaw.BalancePoint);
    }
    else if((IMUData.yaw_mahony - hCtrl.Yaw.BalancePoint) < -PI)
    {
        IMUData.yaw_trans = (IMUData.yaw_mahony - hCtrl.Yaw.BalancePoint) + 2 * PI;
    }
    else if((IMUData.yaw_mahony - hCtrl.Yaw.BalancePoint) > PI)
    {
        IMUData.yaw_trans = (IMUData.yaw_mahony - hCtrl.Yaw.BalancePoint) - 2 * PI;
    }

    // 记录yaw角累计圈数，实现360度以上的角度跟踪
    IMUData.yaw_mahony_turns = (IMUData.yaw_mahony_Last - IMUData.yaw_trans) >  PI ? IMUData.yaw_mahony_turns + 1 : IMUData.yaw_mahony_turns;
    IMUData.yaw_mahony_turns = (IMUData.yaw_mahony_Last - IMUData.yaw_trans) < -PI ? IMUData.yaw_mahony_turns - 1 : IMUData.yaw_mahony_turns;

    // 计算总yaw角度（单位：度），可用于全局航向
    IMUData.sum_yaw_mahony = (float)IMUData.yaw_mahony_turns*2*PI + IMUData.yaw_trans;
    IMUData.sum_yaw_mahony = Rad2Deg(IMUData.sum_yaw_mahony);

    // roll角度零点校正
    IMUData.roll_mahony = IMUData.roll_mahony - hCtrl.Roll.BalancePoint;
}

float L1 = 0.037;  //cm
float L2 = 0.061;
float L3 = 0.090;
#define Angle_limit 0.48//解算角度限幅
// 五连杆机构逆解，输入末端期望位置，输出各关节角度
void FiveLink_Opposite(VMC_Data_Type* leg, float LXc, float Yc)
{
    float AC,CE;
    float Phi,Phi2,Beta,Beta2;
    leg->Yc = Yc;
    leg->Xc = LXc;

    leg->OC = sqrtf(leg->Yc*leg->Yc + leg->Xc*leg->Xc);
    AC = sqrtf(leg->Yc*leg->Yc + (leg->Xc - L1/2)*(leg->Xc - L1/2));
    Phi = acosf(((L1/2)*(L1/2) + AC*AC - leg->OC*leg->OC)/(L1*AC));
    Beta = acosf((L2*L2 + AC*AC - L3*L3)/(2*AC*L2));
    leg->Theta1 = PI - Phi - Beta;
    leg->Theta1 = leg->Theta1 > Angle_limit*PI ? Angle_limit*PI : (leg->Theta1 < -Angle_limit*PI ? -Angle_limit*PI : leg->Theta1);

    CE = sqrtf(leg->Yc*leg->Yc + (leg->Xc + L1/2)*(leg->Xc + L1/2));
    Phi2 = acosf((L1*L1  + CE*CE - AC*AC)/(2*L1*CE));
    Beta2 = acosf((CE*CE + L2*L2 - L3*L3)/(2*L2*CE));
    leg->Theta2 = PI - Phi2 -Beta2;
    leg->Theta2 = leg->Theta2 > Angle_limit*PI ? Angle_limit*PI : (leg->Theta2 < -Angle_limit*PI ? -Angle_limit*PI : leg->Theta2);
}



// 输出PWM信号到四个舵机，控制腿部动作
void PWM_Output(VMC_Data_Type* leg1,VMC_Data_Type* leg2)
{
    if (Jump_Flag == 1)
    {
        leg1->PWM_Out1 =  (int)(LeftF_CENTER  + Extend_Height);  //(1000/PI*leg1->Theta1*6 + LeftF_CENTER);
        leg1->PWM_Out2 =  (int)(LeftB_CENTER  - Extend_Height);  //(1000/PI*leg1->Theta2*6 + LeftB_CENTER);
        leg2->PWM_Out1 =  (int)(RightF_CENTER - Extend_Height); //(1000/PI*leg2->Theta1*6 + RightF_CENTER);
        leg2->PWM_Out2 =  (int)(RightB_CENTER + Extend_Height); //(1000/PI*leg2->Theta2*6 + RightB_CENTER);
    }
        else if (Jump_Flag == 2)
    {
        leg1->PWM_Out1 =  (int)(LeftF_CENTER);  //(1000/PI*leg1->Theta1*6 + LeftF_CENTER);
        leg1->PWM_Out2 =  (int)(LeftB_CENTER);  //(1000/PI*leg1->Theta2*6 + LeftB_CENTER);
        leg2->PWM_Out1 =  (int)(RightF_CENTER); //(1000/PI*leg2->Theta1*6 + RightF_CENTER);
        leg2->PWM_Out2 =  (int)(RightB_CENTER); //(1000/PI*leg2->Theta2*6 + RightB_CENTER);
    }
        else if (Jump_Flag == 3)
    {
        leg1->PWM_Out1 =  (int)(LeftF_CENTER  + Cushion_Height);  //(1000/PI*leg1->Theta1*6 + LeftF_CENTER);
        leg1->PWM_Out2 =  (int)(LeftB_CENTER  - Cushion_Height);  //(1000/PI*leg1->Theta2*6 + LeftB_CENTER);
        leg2->PWM_Out1 =  (int)(RightF_CENTER - Cushion_Height); //(1000/PI*leg2->Theta1*6 + RightF_CENTER);
        leg2->PWM_Out2 =  (int)(RightB_CENTER + Cushion_Height); //(1000/PI*leg2->Theta2*6 + RightB_CENTER);
    }
        else if (Jump_Flag == 4)
    {
        leg1->PWM_Out1 =  (int)(LeftF_CENTER  + Cushion_Speed);  //(1000/PI*leg1->Theta1*6 + LeftF_CENTER);
        leg1->PWM_Out2 =  (int)(LeftB_CENTER  - Cushion_Speed);  //(1000/PI*leg1->Theta2*6 + LeftB_CENTER);
        leg2->PWM_Out1 =  (int)(RightF_CENTER - Cushion_Speed); //(1000/PI*leg2->Theta1*6 + RightF_CENTER);
        leg2->PWM_Out2 =  (int)(RightB_CENTER + Cushion_Speed); //(1000/PI*leg2->Theta2*6 + RightB_CENTER);
    }
        else if (Jump_Flag == 0)
    {
        if (hCtrl.Roll.Output > 0)              //左倾
        {
            leg1->PWM_Out1 =  (int)(LeftF_CENTER  - hCtrl.Pitch.LegOutput + fabsf(hCtrl.Roll.Output));  //(1000/PI*leg1->Theta1*6 + LeftF_CENTER);
            leg1->PWM_Out2 =  (int)(LeftB_CENTER  - hCtrl.Pitch.LegOutput - fabsf(hCtrl.Roll.Output));  //(1000/PI*leg1->Theta2*6 + LeftB_CENTER);
            leg2->PWM_Out1 =  (int)(RightF_CENTER + hCtrl.Pitch.LegOutput); //(1000/PI*leg2->Theta1*6 + RightF_CENTER);
            leg2->PWM_Out2 =  (int)(RightB_CENTER + hCtrl.Pitch.LegOutput); //(1000/PI*leg2->Theta2*6 + RightB_CENTER);
        }
        else                                    //右倾
        {
            leg1->PWM_Out1 =  (int)(LeftF_CENTER + 150 - hCtrl.Pitch.LegOutput);  //(1000/PI*leg1->Theta1*6 + LeftF_CENTER);
            leg1->PWM_Out2 =  (int)(LeftB_CENTER - 150 - hCtrl.Pitch.LegOutput);  //(1000/PI*leg1->Theta2*6 + LeftB_CENTER);
            leg2->PWM_Out1 =  (int)(RightF_CENTER - 150 + hCtrl.Pitch.LegOutput - fabsf(hCtrl.Roll.Output)); //(1000/PI*leg2->Theta1*6 + RightF_CENTER);
            leg2->PWM_Out2 =  (int)(RightB_CENTER + 150 + hCtrl.Pitch.LegOutput + fabsf(hCtrl.Roll.Output)); //(1000/PI*leg2->Theta2*6 + RightB_CENTER);
        }        
        // else                                 //右倾（备用方案）
        // {
        //     leg1->PWM_Out1 =  (int)(LeftF_CENTER +250 - hCtrl.Pitch.LegOutput);  //(1000/PI*leg1->Theta1*6 + LeftF_CENTER);
        //     leg1->PWM_Out2 =  (int)(LeftB_CENTER -250 - hCtrl.Pitch.LegOutput);  //(1000/PI*leg1->Theta2*6 + LeftB_CENTER);
        //     leg2->PWM_Out1 =  (int)(RightF_CENTER -250 + hCtrl.Pitch.LegOutput - fabsf(hCtrl.Roll.Output)); //(1000/PI*leg2->Theta1*6 + RightF_CENTER);
        //     leg2->PWM_Out2 =  (int)(RightB_CENTER +250 + hCtrl.Pitch.LegOutput + fabsf(hCtrl.Roll.Output)); //(1000/PI*leg2->Theta2*6 + RightB_CENTER);
        // }
    }

    leg1->PWM_Out1 = Amplitude_Limit(leg1->PWM_Out1, 2220, 6400);
    leg1->PWM_Out2 = Amplitude_Limit(leg1->PWM_Out2, 1900, 6000);
    leg2->PWM_Out1 = Amplitude_Limit(leg2->PWM_Out1, 1900, 6000);
    leg2->PWM_Out2 = Amplitude_Limit(leg2->PWM_Out2, 3000, 7100);

    pwm_set_duty(Left_FLeg_PWM1, leg1->PWM_Out1);  //左
    pwm_set_duty(Left_BLeg_PWM2, leg1->PWM_Out2);
    pwm_set_duty(Right_FLeg_PWM1, leg2->PWM_Out1); //右
    pwm_set_duty(Right_BLeg_PWM2, leg2->PWM_Out2);
}

float Hill__Height = 0.055;//坡道站立
// 坡道站立高度控制
void Hill_Stand(void)
{
    if (L_Height_Init < Hill__Height)
    {
        L_Height_Init+=Cushion_Speed;
        R_Height_Init+=Cushion_Speed;
    }
    else
    {
        L_Height_Init = Hill__Height;
        R_Height_Init = Hill__Height;
    }
}

extern float LXc[2];
extern float RXc[2];
float Kp_Leg  = 0.0000525;//0.0000525;
float Kd_Leg  = 0.0000075;//0.0000075;
int LLeg_Err[2] = {0};
int RLeg_Err[2] = {0};
float Wiggle_Limit_Init = 0.022;//0.024
float Wiggle_Step_Init = 0.00032;//0.00032
int Slow_Flag = 0;
int Single_Jump = 0;
float LegOutput = 0;
// 俯仰腿部控制，调节腿部输出以实现速度闭环
void Pitch_LegCtrl(void)
{
    Remote_Conctol();

    LegOutput = PID_calc(&PID_PitchLeg, -hCtrl.Pitch.ExpectSpeed_Act, (-motor_value.receive_left_speed_data + motor_value.receive_right_speed_data)/2);
    //hCtrl.Pitch.LegOutput = (hCtrl.Pitch.LegOutput * 29 + LegOutput * 1) / 30;
    if (fabsf(LegOutput) > 1000)
    {
       if(fabsf(hCtrl.Pitch.ExpectSpeed_Act) > 550)
       {
          hCtrl.Pitch.LegOutput += LegOutput / 200;
       }
       else
       {
          hCtrl.Pitch.LegOutput += LegOutput / 100;
       }
    }
    else
    {
       hCtrl.Pitch.LegOutput = (hCtrl.Pitch.LegOutput * 19 + LegOutput * 1) / 20;
    }
    hCtrl.Pitch.LegOutput = Amplitude_Limit(hCtrl.Pitch.LegOutput, -1200, 1200);
}

int PitchOut_Last = 0;
#define Pitch_StepLimit 350//直立环输出阶跃限幅
// 俯仰角速度环控制
void PitchOmegaCtrl(void)
{
    PitchOut_Last = (int)hCtrl.Pitch.Output;
    hCtrl.Pitch.Output = -PID_calc(&PID_PitchOmega,hCtrl.Pitch.ExpectOmega,gx_last);
    hCtrl.Pitch.Output = Step_Limit(hCtrl.Pitch.Output, PitchOut_Last, Pitch_StepLimit);
}

/**
 * @brief Pitch角度控制
 * @note  无
 */
// 俯仰角度环控制
void PitchAngleCtrl(void)
{
    hCtrl.Pitch.ExpectOmega = PID_calc(&PID_PitchAngle,Expect_Angle_Pitch,(IMUData.pitch_mahony - hCtrl.Pitch.BalancePoint));
}

// 航向角速度环控制
void YawOmegaCtrl(void)
{
    hCtrl.Yaw.Output = PID_calc(&PID_YawOmega,hCtrl.Yaw.ExpectOmega_Exp,-IMUData.gz);
}

// 航向角度环控制
void YawAngleCtrl(void)
{
    Remote_Conctol();
    uart_receiver_data[0] = 0;
    hCtrl.Yaw.ExpectOmega_Exp = PID_calc(&PID_YawAngle,Expect_Angle,-IMUData.sum_yaw_mahony);
}

// 横滚角度环控制
void RollAngleCtrl(void)
{
    hCtrl.Roll.Output = PID_calc(&PID_RollAngle,0,IMUData.roll_mahony);
}

float Swerve_Offset[2] = {0};
float IMU_ax_output_a[N] = {0};//Yaw角速度滤波
float IMU_ax_input_a[N] = {0};
float Swerve_P = 4.5;
#define Swerve_Step  0.0004//压弯阶跃限幅
// 压弯补偿（预留）
void Swerve_RollOffset(void)
{

}

float KP1 = 0.610,KP2 = 0,KD = 0.265,GKD = 0, KP1_5 = 0;
#define Yaw_OUtProP 0.55
float Ctrl_ProP = 1.00;

#define T 0.6
float Final_deviation[3] = {0};
float deviation_output[2] = {0};
float deviation_input[2] = {0};
double Deviation_filter_b[2] = {1,-T};
double Deviation_filter_a[2] = {1,-T};
#define ERR_Limit 10
float Angle_Offset[2] = {0};
#define Angle_Offset_Limit 0.25
#define Yaw_StepLimit 180
// 方向控制（预留）
void Direction_Ctrl(void)
{

}

int Run_Delay = 0;
float Final_Speed = 0;
// 电机速度输出，合成左右轮PWM并限幅
void Speed_Output(void)
{
    int32 liSpeedPwm_L,liSpeedPwm_R;
    if (Jump_Flag == 0)
    {
        liSpeedPwm_L =(int)(hCtrl.Pitch.Output - hCtrl.Yaw.Output + Speed_Compensation);
        liSpeedPwm_R =(int)(hCtrl.Pitch.Output + hCtrl.Yaw.Output + Speed_Compensation);
    }
    else
    {
        liSpeedPwm_L =(int)((hCtrl.Pitch.Output- hCtrl.Yaw.Output) / 3);
        liSpeedPwm_R =(int)((hCtrl.Pitch.Output+ hCtrl.Yaw.Output) / 3);
    }
//     liSpeedPwm_L =(int)((hCtrl.Pitch.Output - hCtrl.Yaw.Output)/(JumpOff + 1));
//     liSpeedPwm_R =(int)((hCtrl.Pitch.Output + hCtrl.Yaw.Output)/(JumpOff + 1));
    liSpeedPwm_L = Amplitude_Limit(liSpeedPwm_L, -9000, 9000);
    liSpeedPwm_R = Amplitude_Limit(liSpeedPwm_R, -9000, 9000);
    if(Switch_On)
    {
        FOC_SendControl(Set_Speed , liSpeedPwm_L , -liSpeedPwm_R);
    }

    else
    {
        Run_Delay = 0;
        FOC_SendControl(Set_Speed ,-0,0);
    }
}

// 电机超速保护，防止失控
void SAFE_PROTECT(void)
{
    if(ABS(motor_value.receive_left_speed_data) > 10000 || ABS(motor_value.receive_right_speed_data) > 10000)
    {
        PID_cleardata(&PID_PitchOmega);
        PID_cleardata(&PID_YawOmega);
    }
}

// 处理遥控器输入，更新期望角度和速度
void Remote_Conctol (void)
{
    if(uart_receiver.channel[3] == 1792)                                             // 4通道保护（是否打点）
    {
        Expect_Angle += (uart_receiver.channel[0] - 992) * 0.01 ;                    // 1通道方向
        hCtrl.Pitch.ExpectSpeed_Act = (uart_receiver.channel[1] - 992) * 1 ;         // 2通道油门
    }
}