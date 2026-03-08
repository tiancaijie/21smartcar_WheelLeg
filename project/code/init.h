/*
 * init.h
 *
 *  Created on: 2023年9月17日
 *      Author: 86139
 */

#ifndef CODE_INIT_H_
#define CODE_INIT_H_
#include "zf_common_headfile.h"

//-----电源ADC采样-----
#define Battery_Power_Pin              ADC0_CH07_P06_7
//-----使能开关-----
#define Switch_Pin                     P00_2
#define Switch_On                     (gpio_get_level(Switch_Pin) == 1)
#define Switch_Off                    (gpio_get_level(Switch_Pin) == 0)

//-----蜂鸣器-----
#define Buzzer_Pin                     P19_1
#define Buzzer_On                     (gpio_set_level(Buzzer_Pin, 1))
#define Buzzer_Off                    (gpio_set_level(Buzzer_Pin, 0))

#define LeftWheel_En                   P15_1//P15_3  // P23_1        //驱动使能
#define RightWheel_En                  P11_11  // P23_1        //驱动使能
#define Left_GO_On                    (gpio_set_level(LeftWheel_En, 1))
#define Left_GO_Off                   (gpio_set_level(LeftWheel_En, 0))
#define Right_GO_On                   (gpio_set_level(RightWheel_En, 1))
#define Right_GO_Off                  (gpio_set_level(RightWheel_En, 0))

//#define LeftWheel_Brake                P15_1  // P23_1        //驱动使能
//#define RightWheel_Brake               P11_9  // P23_1        //驱动使能

//-----Leg-----
#define LeftF_CENTER                   770*6 - 100 * 9//823
#define LeftB_CENTER                   770*6//823
#define RightF_CENTER                  710*6//823
#define RightB_CENTER                  750*6//823

#define LeftF_minimum                   LeftF_CENTER - 485//(LeftF_CENTER-500)*6//823
#define LeftB_minimum                   LeftB_CENTER + 485//(LeftB_CENTER+500)*6//823
#define RightF_minimum                  RightF_CENTER + 485//(RightF_CENTER+500)*6//823
#define RightB_minimum                  RightB_CENTER - 485//(RightB_CENTER-500)*6//823

#define LeftF_maximum                   LeftF_CENTER + 485//(LeftF_CENTER+500)*6   //90
#define LeftB_maximum                   LeftB_CENTER - 485//(LeftB_CENTER-500)*6
#define RightF_maximum                  RightF_CENTER - 485//(RightF_CENTER-500)*6
#define RightB_maximum                  RightB_CENTER + 485//(RightB_CENTER+500)*6

#define LeftF_Init                      770*6 - 100 * 9//(LeftF_CENTER+500)*6   //90
#define LeftB_Init                      770*6//(LeftB_CENTER-500)*6
#define RightF_Init                     710*6//(RightF_CENTER-500)*6
#define RightB_Init                     750*6//(RightB_CENTER+500)*6


#define Left_FLeg_PWM1                 TCPWM_CH38_P12_2//ATOM0_CH1_P21_3
#define Left_BLeg_PWM2                 TCPWM_CH39_P12_3
#define Right_FLeg_PWM1                TCPWM_CH37_P12_1
#define Right_BLeg_PWM2                TCPWM_CH36_P12_0

#define Left_Wheel_PWM                 TCPWM_CH30_P10_2//
#define Right_Wheel_PWM                TCPWM_CH31_P10_3

#define Left_Wheel_Dir                 P15_4
#define Right_Wheel_Dir                P11_10
#define Left_Forwards                 (gpio_set_level(Left_Wheel_Dir, 1))
#define Right_Forwards                (gpio_set_level(Right_Wheel_Dir, 1))
#define Left_Reverse                  (gpio_set_level(Left_Wheel_Dir, 0))
#define Right_Reverse                 (gpio_set_level(Right_Wheel_Dir, 0))


#define LeftWheel_TIM                  TIM5_ENCODER            //测速
#define RightWheel_TIM                 TIM6_ENCODER
#define Left_Speed_Get                 TIM5_ENCODER_CH1_P10_3
#define Left_Direct_Get                TIM5_ENCODER_CH2_P10_1
#define Right_Speed_Get                TIM6_ENCODER_CH1_P20_3
#define Right_Direct_Get               TIM6_ENCODER_CH2_P20_0

//-----电池电量-----
#define Battery_ADC                    ADC_1
#define Battery_Pin                    ADC0_CH16_P07_0

//-----UART-----                   UART_0
#define UART_PORT_0_BaudRate           115200
#define UART_PORT_0_TXD                UART0_TX_P14_0
#define UART_PORT_0_RXD                UART0_RX_P14_1

#define UART_PORT_2_BaudRate           115200
#define UART_PORT_2_TXD                UART2_TX_P14_2
#define UART_PORT_2_RXD                UART2_RX_P14_3

extern int dbg[20];
extern void Init_All(void);
#endif /* CODE_INIT_H_ */
