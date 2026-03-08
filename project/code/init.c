/*
 * init.c
 *
 *  Created on: 2023年12月11日
 *      Author: 86139
 */
#include "zf_common_headfile.h"
#include "MahonyAHRS.h"

void gpio_init_all()
{

}

void Init_All(void)
{
    /* ----串口初始化---- */
    uart_init(UART_1, 460800, UART1_TX_P04_1, UART1_RX_P04_0);    //电调串口
    uart_rx_interrupt (UART_1, 1);
    uart_init(UART_4, 115200, UART4_TX_P14_1, UART4_RX_P14_0);    //串口
    /* ----IO口初始化---- */
    gpio_init(Buzzer_Pin, GPO, 0, GPO_PUSH_PULL);                 //蜂鸣器
    gpio_init(Switch_Pin, GPI, 0, GPI_PULL_DOWN);                 //使能开关
    /* ----舵机初始化---- */
    pwm_init(Left_FLeg_PWM1, 300, LeftF_Init);                    //左
    pwm_init(Left_BLeg_PWM2, 300, LeftB_Init);
    pwm_init(Right_FLeg_PWM1, 300, RightF_Init);                  //右
    pwm_init(Right_BLeg_PWM2, 300, RightB_Init);
    /* ----电机初始化---- */
    adc_init(Battery_Pin,ADC_12BIT);                              //电压
    FOC_SendControl(Set_Speed ,0,-0);
    small_driver_get_speed();
    /* ----陀螺仪初始化---- */
    imu963ra_init();
    icm963_offset();
    IMUData.q[0] = 1.0f;
    IMUData.q[1] = 0.0f;
    IMUData.q[2] = 0.0f;
    IMUData.q[3] = 0.0f;
    /* ----定位模块初始化---- */
    gnss_init(TAU1201);
    /* ----键显初始化---- */
    flash_init();
    OLED_Init();
    Oled_Input();
}
