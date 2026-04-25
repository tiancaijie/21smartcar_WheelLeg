#include "zf_common_headfile.h"
#include "zf_driver_uart.h"
#include "isr.h"
#include "MahonyAHRS.h"
int giWheelSpeed_Old = 0,giLeftWheelSpeed_old = 0,giRightWheelSpeed_old = 0;
int giWheelSpeed = 0,giRightWheelSpeed = 0,giLeftWheelSpeed = 0, SpeedMax, SpeedMid, SpeedMin;
int SP_RightFiltering[2] = {0};             //历史�??
int SP_LeftFiltering[2] = {0};              //历史�??
int Change[2];
int SpeedHill, SpeedCircle;
typedef union//������ת�����͹�����
{
  float fdata;
  unsigned long ldata;
}FloatLongType;

void Float_to_Byte(float f,unsigned char byte[])//������ת4�ֽ�
{
    FloatLongType fl;
    fl.fdata=f;
    byte[0]=(unsigned char)fl.ldata;
    byte[1]=(unsigned char)(fl.ldata>>8);
    byte[2]=(unsigned char)(fl.ldata>>16);
    byte[3]=(unsigned char)(fl.ldata>>24);
}


void GetSpeed (void)
{
       giWheelSpeed_Old = giWheelSpeed;
       giRightWheelSpeed = 3 * encoder_get_count(TC_CH07_ENCODER);
       giLeftWheelSpeed  = 3 * encoder_get_count(TC_CH20_ENCODER);

       giLeftWheelSpeed = (int)(0.7 *giLeftWheelSpeed+ 0.2 * SP_LeftFiltering[0] + 0.1 * SP_LeftFiltering[1]);
       giLeftWheelSpeed_old = giLeftWheelSpeed;
       SP_LeftFiltering[1] = SP_LeftFiltering[0];
       SP_LeftFiltering[0] = giLeftWheelSpeed;

       giRightWheelSpeed = (int)(0.7 *giRightWheelSpeed+ 0.2 * SP_RightFiltering[0] + 0.1 * SP_RightFiltering[1]);
       giRightWheelSpeed_old = giRightWheelSpeed;
       SP_RightFiltering[1] = SP_RightFiltering[0];
       SP_RightFiltering[0] = giRightWheelSpeed;

       giWheelSpeed = (giLeftWheelSpeed + giRightWheelSpeed) >> 1;
       encoder_clear_count(TC_CH07_ENCODER);
       encoder_clear_count(TC_CH20_ENCODER);
}

/*************************************************
Function: ClearArrayChar
Description: 清空数据
Calls: �??
Called By: main.c
Table Accessed: �??
Table Updated: �??
Input: 需要清零的数组首地址
       需要清零的数组尾地址
Output: �??
Return: �??
Others:�??
*************************************************/
void ClearArrayChar(unsigned char *PTemp, unsigned char *PEnd)
{
    while (PTemp <= PEnd)
    {
        *(PTemp++) = 0;
    }
}
void ClearArraySignedChar(signed char *PTemp,signed char *PEnd)
{
    while (PTemp <= PEnd)
    {
        *(PTemp++) = 0;
    }
}
void ClearArrayInt(int *PTemp, int *PEnd)
{
    while (PTemp <= PEnd)
    {
        *(PTemp++) = 0;
    }
}
void ClearArrayShortInt(short int *PTemp,short int *PEnd)
{
    while (PTemp <= PEnd)
    {
        *(PTemp++) = 0;
    }
}
void ClearArrayUnsignedShortInt(uint16 *PTemp,uint16 *PEnd)
{
    while (PTemp <= PEnd)
    {
        *(PTemp++) = 0;
    }
}
void ClearArrayfloat(float *PTemp, float *PEnd)
{
    while (PTemp <= PEnd)
    {
        *(PTemp++) = 0;
    }
}

/**
** Function: m_sqrt
** Description: 正数开�??
** Others:�??
**/
unsigned int m_sqrt(unsigned int x)
{
   unsigned char ans = 0,p = 0x80;
   while(p!=0)
  {
     ans+=p;
     if(ans*ans>x)
     {
       ans-=p;
     }
     p=(unsigned char)(p/2);
   }
   return(ans);
}
/************************************************
 * Function: Square
 * Description: 求数�??平方
 * Calls: None
 * Input: arg
 * Return: arg * arg;
 ************************************************/
int Square_int(int arg)
{
    return arg * arg;
}
float Square_float(float arg)
{
    return arg * arg;
}
/**
** Function: Linear_Interpolation
** Description: 线性插�??
** Others:*Ptemp：数组的起�?�指针，End_Row：一共进行�?�少�??
**/
void Linear_Interpolation(int *Ptemp,int End_Row)
{
    int i = 0,j = 0,k = 0;
    for(i = 0;i < End_Row;i++)
    {
        if(*(Ptemp + i) != 0)
        {
            for(j=i+1;j < End_Row ;j++)
            {
                if(*(Ptemp + j) == 0)
                {
                    for(k=j+1;k < End_Row;k++)
                    {
                        if(*(Ptemp + k)!=0)
                        {
                            *(Ptemp + j) = (int)(*(Ptemp + j - 1) + (*(Ptemp + k) - *(Ptemp + j - 1))/((k-j+1) * 1.0));
                            break;
                        }
                    }
                }
            }
            break;
        }
    }
}
/**
** Function: ParameterSent
** Description: ��λ��������
** Others:��
**/
void ParameterSent(void)
{
    int itmp;
    unsigned char utmp1, utmp2;

    uart_write_byte(UART_2, 0xff);
    uart_write_byte(UART_2, 0x00);

    itmp = (signed int)(giLeftWheelSpeed);//5
    utmp1 = (itmp) >> 8;
    utmp2 = (itmp) & 0x00ff;
    uart_write_byte(UART_2, utmp2);
    uart_write_byte(UART_2, utmp1);

    itmp = (signed int)(giRightWheelSpeed);//5
    utmp1 = (itmp) >> 8;
    utmp2 = (itmp) & 0x00ff;
    uart_write_byte(UART_2, utmp2);
    uart_write_byte(UART_2, utmp1);

    itmp = (signed int)(dbg[0]);//期望速度L
    utmp1 = (itmp) >> 8;
    utmp2 = (itmp) & 0x00ff;
    uart_write_byte(UART_2, utmp2);
    uart_write_byte(UART_2, utmp1);

    itmp = (signed int)(dbg[1]);//期望速度R
    utmp1 = (itmp) >> 8;
    utmp2 = (itmp) & 0x00ff;
    uart_write_byte(UART_2, utmp2);
    uart_write_byte(UART_2, utmp1);
//
    itmp = (signed int)(dbg[2]);//PWM.L
    utmp1 = (itmp) >> 8;
    utmp2 = (itmp) & 0x00ff;
    uart_write_byte(UART_2, utmp2);
    uart_write_byte(UART_2, utmp1);

    itmp = (signed int)(dbg[3]);//PWM.R
    utmp1 = (itmp) >> 8;
    utmp2 = (itmp) & 0x00ff;
    uart_write_byte(UART_2, utmp2);
    uart_write_byte(UART_2, utmp1);

//    itmp = (signed int)(Center_deviation[0]/*giRight_Eye[0]*/);//3
//    utmp1 = (itmp) >> 8;
//    utmp2 = (itmp) & 0x00ff;
//    uart_write_byte(UART_2, utmp2);
//    uart_write_byte(UART_2, utmp1);
//
//    itmp = (signed int)(YawPWM[0].P);//4 dbg[17]
//    utmp1 = (itmp) >> 8;
//    utmp2 = (itmp) & 0x00ff;
//    uart_write_byte(UART_2, utmp2);
//    uart_write_byte(UART_2, utmp1);
//
//    itmp = (signed int)(YawPWM[0].D);//4 dbg[18]
//    utmp1 = (itmp) >> 8;
//    utmp2 = (itmp) & 0x00ff;
//    uart_write_byte(UART_2, utmp2);
//    uart_write_byte(UART_2, utmp1);

    uart_write_byte(UART_2, 0xAA);
    uart_write_byte(UART_2, 0x55);
}


uint8 Vofa_Record = 0;
int Vofa_Get[4] = {0};
int uartget = 0;

void VOFA_Receive_callback (uart_index_enum uart_n)
{
    uint8 dat;//记录当下的项
    //把分块的广播报文记录入数组中
    while(uart_query_byte(uart_n, &dat) && Vofa_Record < 2 )
    {
        Vofa_Get[Vofa_Record] = dat;
        Vofa_Record++;
    }
    if(Vofa_Record == 2)
    {
        for(uint16 num =  48 ; num < 60 ; num++)
        {
            if(Vofa_Get[0] == num)
            {
                uartget = num - 48;
            }
        }
        Vofa_Record = 0;
    }
}

extern float L_Height;
extern float R_Height;
extern uint8 CircleExitSureRow;
extern uint8 MaxLeftRow;
extern uint8 ExitStartRow;
extern uint16 MaxLeftCol;
extern uint8 CircleFinishRow;
extern float Final_deviation[3];
extern uint8 LinkOn;
extern uint8 LinkEndRow, LinkStartRow;
extern uint16 LinkEndCol,LinkStartCol;
extern int State_Count;
extern uint8 dl1b_init_flag;
extern uint8 dl1b_finsh_flag;
extern uint16 dl1b_distance_mm;
extern int ISR_Time;
extern PID_HandleTypeDef PID_PitchOmega;
extern PID_HandleTypeDef PID_PitchAngle;
extern PID_HandleTypeDef PID_RollAngle;
extern float gz_last;
extern int Jump_Row;
extern float Swerve_P;
void VOFA_ParameterSent(uart_index_enum uart_n)
{
    float2u8_union SendValue[20];
    uint8 tmpu8[4];
    uint8 i = 0;
    uint8 end[4] = {0x00, 0x00, 0x80, 0x7f};

    SendValue[0].floatValue = (float)motor_value.receive_left_speed_data;
    SendValue[1].floatValue = (float)motor_value.receive_right_speed_data;
    SendValue[2].floatValue = (float)Jump_Flag;
    SendValue[3].floatValue = (float)gx_last;
    SendValue[4].floatValue = (float)Now_Dot;
    SendValue[5].floatValue = (float)IMUData.roll_mahony;
    SendValue[6].floatValue = (float)hCtrl.Roll.Output;
    SendValue[7].floatValue = (float)hCtrl.Pitch.LegOutput;
    SendValue[8].floatValue = (float)LegOutput;
    SendValue[9].floatValue = (float)hCtrl.Pitch.LegOutput;
    SendValue[10].floatValue = (float)INSData.Position_y;
    SendValue[11].floatValue = (float)INSData.Position_x;

    for (i = 0; i < 12; i++)
    {
    tmpu8[0] = SendValue[i].uint8Value[0];
	tmpu8[1] = SendValue[i].uint8Value[1];
	tmpu8[2] = SendValue[i].uint8Value[2];
	tmpu8[3] = SendValue[i].uint8Value[3];

        uart_write_buffer(uart_n, tmpu8, 4);
    }

    uart_write_buffer(uart_n, end, 4);
}

void itoa(int32_t num, char* buffer) {
    uint8_t isNegative = 0;
    uint32_t n;
    char* ptr = buffer;
    char* end;

    // 处理负数
    if (num < 0) {
        isNegative = 1;
        n = (uint32_t)(-num);
    } else {
        n = (uint32_t)num;
    }

    // 生成反向字�?�串
    do {
        *ptr++ = (n % 10) + '0';
        n /= 10;
    } while (n > 0);

    // 添加负号
    if (isNegative) {
        *ptr++ = '-';
    }
    *ptr = '\0'; // 终�?�字符串

    // 反转字�?�串
    end = ptr - 1;
    ptr = buffer;
    while (ptr < end) {
        char tmp = *ptr;
        *ptr++ = *end;
        *end-- = tmp;
    }
}

void FOC_SendControl(FOCcontrol_Type FOC_Send,int16 Left_SpeedPWM,int16 Right_SpeedPWM)
{
    uint8_t data[7] = {0x00};
    data[0] = 0xA5;
    switch(FOC_Send)
    {
        case Set_Speed:
        {
            data[1] = 0x01;
            data[2] = Left_SpeedPWM >> 8;
            data[3] = Left_SpeedPWM & 0xFF;
            data[4] = Right_SpeedPWM >> 8;
            data[5] = Right_SpeedPWM & 0xFF;;
            break;
        }
        case Get_Speed:
        {
            data[1] = 0x02;
            data[2] = 0x00;
            data[3] = 0x00;
            data[4] = 0x00;
            data[5] = 0x00;
            break;
        }
        case Zero_Init:
        {
            data[1] = 0x03;
            data[2] = 0x00;
            data[3] = 0x00;
            data[4] = 0x00;
            data[5] = 0x00;
            break;
        }
    }
    data[6] = 0;                                            // 和校验清
    for(int i = 0; i < 6; i ++)
    {
        data[6] += data[i];         // 计算校验�??
    }
    uart_write_buffer(UART_1, data, 7);
}



