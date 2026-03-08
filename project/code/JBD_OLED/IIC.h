/*************************************************
Copyright (C), 2016-2017, TYUT JBD TEAM C.
File name: I2C.h
Author: TEAM  A B C
Version:0.0               Date: 2016.11.12
Description:  I2C.h
Others:      无
Function List:
History:
<author>  <time>      <version > <desc>
JBD       2016.10.21  0.0        初始
AmaZzzing 2016.11.12  1.0        初步完成构架
双车&信标   2020.6.1    儿童节版      兼容英飞凌
**************************************************/
#ifndef __I2C_H
#define __I2C_H

#include "zf_driver_gpio.h"
#include  "OLED.h"
/* 注意 IIC总线规定，IIC空闲时 SCL和SDA都为高电平 最好外部上拉（一定不能下拉） */
/* 模拟 IIC需要注意，IIC地址左移一位 例如MPU6050 模拟就是地址 0xD0 */
/* 想换用别的IO 直接修改宏定义 OLED_IIC_SCL_PIN 、 OLED_IIC_SDA_PIN 即可 */
#define OLED_IIC_SCL_PIN   P07_1   /*!< P07_1  作为 SCL */
#define OLED_IIC_SDA_PIN   P07_3   /*!< P07_3  作为 SDA */

#define OLED_SDA_OUT        gpio_set_dir(OLED_IIC_SDA_PIN, GPO,GPO_PUSH_PULL);
#define OLED_SDA_IN         gpio_set_dir(OLED_IIC_SDA_PIN, GPI,GPO_PUSH_PULL);

#define OLED_IIC_SCL_INIT   gpio_init(OLED_IIC_SCL_PIN,GPO, GPO_PUSH_PULL, 1);
#define OLED_IIC_SDA_INIT   gpio_init(OLED_IIC_SDA_PIN,GPO, GPO_PUSH_PULL, 1);

#define OLED_IIC_SCL_H      gpio_set_level(OLED_IIC_SCL_PIN, 1);
#define OLED_IIC_SCL_L      gpio_set_level(OLED_IIC_SCL_PIN, 0);

#define OLED_IIC_SDA_H      gpio_set_level(OLED_IIC_SDA_PIN, 1);
#define OLED_IIC_SDA_L      gpio_set_level(OLED_IIC_SDA_PIN, 0);

#define OLED_IIC_SDA_READ   gpio_get_level(OLED_IIC_SDA_PIN)

//#define Buzzer_On    gpio_set_level(Buzzer, 1);
//#define Buzzer_Off     gpio_set_level(Buzzer, 0);

/* 注意 IIC总线规定，IIC空闲时 SCL和SDA都为高电平 最好外部上拉（一定不能下拉） */
/* 模拟 IIC需要注意，IIC地址左移一位 例如MPU6050 模拟就是地址 0xD0 */
/* 想换用别的IO 直接修改宏定义 SOFT_IIC_SCL_PIN 、 SOFT_IIC_SDA_PIN 即可 */
#define SOFT_IIC_SCL_PIN   P03_1   /*!< P13_1  作为 SCL */
#define SOFT_IIC_SDA_PIN   P03_2   /*!< P13_2  作为 SDA */

#define SDA_OUT        gpio_set_dir(SOFT_IIC_SDA_PIN, GPO,GPO_PUSH_PULL);
#define SDA_IN         gpio_set_dir(SOFT_IIC_SDA_PIN, GPI,GPO_PUSH_PULL;

#define IIC_SCL_INIT   gpio_init(SOFT_IIC_SCL_PIN, GPO_PUSH_PULL, 1);
#define IIC_SDA_INIT   gpio_init(SOFT_IIC_SDA_PIN, GPO_PUSH_PULL, 1);

#define IIC_SCL_H      gpio_set_level(SOFT_IIC_SCL_PIN, 1);
#define IIC_SCL_L      gpio_set_level(SOFT_IIC_SCL_PIN, 0);

#define IIC_SDA_H      gpio_set_level(SOFT_IIC_SDA_PIN, 1);
#define IIC_SDA_L      gpio_set_level(SOFT_IIC_SDA_PIN, 0);

#define IIC_SDA_READ   gpio_get_level(SOFT_IIC_SDA_PIN)
///************************************宏定义************************************/
//
//#define I2C_SDA_OUT     PIN_InitConfig(P02_1, PIN_MODE_OUTPUT, 1);
//#define I2C_SDA_IN      PIN_InitConfig(P02_1, PIN_MODE_INPUT, 1);
//#define I2C_SDA_GET     PIN_Read(P02_1)
//
//#define I2C_SDA_HIGH    PIN_Write(P02_1, 1);
//#define I2C_SDA_LOW     PIN_Write(P02_1, 0);
//
//#define I2C_SCL_HIGH    PIN_Write(P02_0, 1);
//#define I2C_SCL_LOW     PIN_Write(P02_0, 0);
//
//#define OLED_RST_HIGH    PIN_Write(P02_2, 1);
//#define OLED_RST_LOW     PIN_Write(P02_2, 0);

/*********************************全局变量声明*********************************/

/***********************************函数声明***********************************/

extern void I2c_Start(void);
extern void I2c_Stop(void);
extern void I2c_Write_OneByte(unsigned char data_t);
extern unsigned char I2c_Read_OneByte(uint8 ack);
extern void Delay_us(void)  ;
extern void IIC_Ack(void);
extern void IIC_NAck(void);
extern void iic_delay (void);
extern void I2c_Start(void);
extern unsigned char I2C_Wait_Ask(void);
extern void I2c_Stop(void);
#endif
