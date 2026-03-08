/*************************************************
Copyright (C), 2016-2017, TYUT JBD TEAM C.
File name: I2C.c
Author: TEAM  A B C
Version:0.0               Date: 2016.11.12
Description: oled模拟I2c
Others:无
Function List: 1.Delay_us
               2.I2c_Start
               3.I2c_Stop
               4.IIC_Wait_Ask
               5.I2c_Write_OneByte
               6.I2c_Read_OneByte
               7.OLED_Cmd
               8.OLED_Dat
History:
<author>  <time>      <version > <desc>
JBD       2016.10.21  0.0      初始
信标&双车   2020.6.1    儿童节版      兼容英飞凌
**************************************************/

#include "OLED.h"
#include "IIC.h"
/*****************延时函数************************/
void Delay_us(void)
{
    /* 200MHz 系统时钟下 模拟IIC速度为 400Khz */

        uint8  i = 0;
        for(i = 0; i < 30; i++) //修改这里可以调整IIC速率
        {
            __asm("NOP"); /* delay */
        }
}
/*************************************************
Function: I2c_Start
Description:IIC开始信号
Details：IIC开始信号为SCL高电平，SDA出现下降沿
********************************************** ***/
void I2c_Start(void)
{
    OLED_SDA_OUT;   //sda线输出
    OLED_IIC_SDA_H;
    OLED_IIC_SCL_H;
    Delay_us();
    Delay_us();
    Delay_us();
    Delay_us();
    Delay_us();
    OLED_IIC_SDA_L; //START:when CLK is high,DATA change form high to low
    Delay_us();
    Delay_us();
    Delay_us();
    OLED_IIC_SCL_L; //钳住I2C总线，准备发送或接收数据
}
/*************************************************
Function: I2c_Stop
Description:IIC停止信号
Details：IIC停止信号为SCL高电平，SDA出现上升沿
*************************************************/
void I2c_Stop(void)
{
    OLED_SDA_OUT; //sda线输出
    OLED_IIC_SCL_L;
    OLED_IIC_SDA_L; //STOP:when CLK is high DATA change form low to high
    Delay_us();
    Delay_us();
    Delay_us();
    OLED_IIC_SCL_H;
    Delay_us();
    Delay_us();
    Delay_us();
    OLED_IIC_SDA_H; //发送I2C总线结束信号
    Delay_us();
}
/*************************************************
Function: IIC_Wait_Ask
Description:IIC应答信号
Details：应答信号为SDA由高电平变为低电平
*************************************************/
unsigned char I2C_Wait_Ask(void)
{
//    uint8  ucErrTime=0;
    OLED_SDA_IN; //SDA设置为输入  （从机给一个低电平做为应答）
    OLED_IIC_SDA_H;Delay_us();
    OLED_IIC_SCL_H;Delay_us();
    while(OLED_IIC_SDA_READ)
    {
//      ucErrTime++;
//      if(ucErrTime>10)
//      {
            I2c_Stop();
            return 1;
//      }
    }
    OLED_IIC_SCL_L; //时钟输出0
    return 0;
}
/*************************************************
Function: I2c_Write_OneByte
Description:IIC写一个字节
Details：应用于CH455读取键值,在进行数据传送时，时钟信号(SCL)信号为高电平期间，
        数据线(SDA)上数据必须保持稳定，只有SCL为低电平时，SDA上的状态才允许发生变化
*************************************************/
void I2c_Write_OneByte(unsigned char data_t)
{
    uint8  t;
    OLED_SDA_OUT;
    OLED_IIC_SCL_L; //拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {
//        OLED_IIC_SDA_READ = data_t&0x80;
        if(data_t&0x80)
        {
            OLED_IIC_SDA_H;
        }
        else
        {
            OLED_IIC_SDA_L;
        }
        Delay_us();
                                                                                                                                                         OLED_IIC_SCL_H;;
        Delay_us();
        data_t<<=1;
        Delay_us();
        Delay_us();
        OLED_IIC_SCL_L;
        Delay_us();
    }
    Delay_us();
}
/*************************************************
Function: I2c_Read_OneByte
Description:IIC读一个字节
Details：应用于CH455读取键值
*************************************************/
unsigned char I2c_Read_OneByte(uint8 ack)
{
    uint8  i,receive=0;
    OLED_SDA_IN; //SDA设置为输入模式 等待接收从机返回数据
    for(i=0;i<8;i++ )
    {
        OLED_IIC_SCL_L;
        Delay_us();
        Delay_us();
        Delay_us();
        OLED_IIC_SCL_H;
        receive<<=1;
        if(OLED_IIC_SDA_READ)receive++; //从机发送的电平
        Delay_us();
    }
    if(ack)
        IIC_Ack(); //发送ACK
    else
        IIC_NAck(); //发送nACK
    return receive;
}
/*************************************************************************
*  函数名称：void IIC_Ack(void)
*  功能说明：模拟IIC产生ACK应答
*  参数说明：无
*  函数返回：无
*  修改时间：2020年3月10日
*  应用举例：内部调用 主机接收完一个字节数据后，主机产生的ACK通知从机一个字节数据已正确接收
*************************************************************************/
void IIC_Ack(void)
{
    IIC_SCL_L;
    SDA_OUT;
    IIC_SDA_L;
    iic_delay();
    iic_delay();
    iic_delay();
    IIC_SCL_H;
    iic_delay();
    iic_delay();
    iic_delay();
    IIC_SCL_L;
}


/*************************************************************************
*  函数名称：void IIC_NAck(void)
*  功能说明：模拟IIC不产生ACK应答
*  参数说明：无
*  函数返回：无
*  修改时间：2020年3月10日
*  应用举例：内部调用 主机接收完最后一个字节数据后，主机产生的NACK通知从机发送结束，释放SDA,以便主机产生停止信号
*************************************************************************/
void IIC_NAck(void)
{
    IIC_SCL_L;
    SDA_OUT;
    IIC_SDA_H;
    iic_delay();
    iic_delay();
    iic_delay();
    IIC_SCL_H;
    iic_delay();
    iic_delay();
    iic_delay();
    IIC_SCL_L;
}

/*************************************************************************
*  函数名称：IIC延时
*  功能说明：ADC初始化函数
*  参数说明：us      ：  延时时间
*  函数返回：无
*  修改时间：2020年3月10日
*  应用举例：iic_delay(1);  //简单的延时
*  内部调用 修改这里可以调整IIC速率
*************************************************************************/
void iic_delay (void)
{
    /* 200MHz 系统时钟下 模拟IIC速度为 400Khz */

    uint8  i = 0;
    for(i = 0; i < 30; i++) //修改这里可以调整IIC速率
    {
        __asm("NOP"); /* delay */
    }
}
