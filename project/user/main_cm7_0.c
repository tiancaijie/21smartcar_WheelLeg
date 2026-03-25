/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main_cm7_0
* 公司名称          成都逐飞科技有限公司.
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
// 打开新的工程或者工程移动了位置务必执行以下操作  测试222
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************
//#pragma location = 0x28001000                                                   // 将下面这个数组定义到指定的RAM地址，便于其他核心直接访问(开源库默认在 0x28001000 地址保留了8kb的空间用于数据交互)
//__no_init float Distance;
//#pragma location = 0x28001004
//__no_init float Azimuth;
//#pragma location = 0x28001008
//__no_init float CarDirection;
//#pragma location = 0x2800100C
//__no_init float Start_Direaction;
int step = 0;
int time = 0;
int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    interrupt_global_disable();         // 关闭全局中断
    debug_init();                       // 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等
    PID_InitAll();
    Init_All();
    pit_ms_init(PIT_CH0, 2);
    
    interrupt_set_priority(CPUIntIdx2_IRQn, 1); 
    interrupt_global_enable (0);
    
    
//    uint8 step = 0;
//    uint8 time = 0;
    // 此处编写用户代码 例如外设初始化代码等     测试git
    
    while(true)
    {
        // 此处编写需要循环执行的代码
        Oled_Display();
//        Reset_StartPot();
        VOFA_ParameterSent(UART_4);
        

        if(hCtrl.Yaw.BalancePoint == 0 && Switch_On)
        {
            hCtrl.Yaw.BalancePoint = IMUData.yaw_mahony;
        }
        
        Run_Start();
        
          
//        if (Distance < 30 && step == 0)
//        {
//            hCtrl.Pitch.ExpectSpeed_Exp = hCtrl.Pitch.ExpectSpeed_Exp;
//            hCtrl.Yaw.ExpectAngle_Act = 0;
//        }
//        else if (Distance > 30 && step == 0)
//        {
//            time++;
//            if (time > 3)
//            {
//                step = 1;
//                hCtrl.Pitch.ExpectSpeed_Exp = -hCtrl.Pitch.ExpectSpeed_Exp;
//                hCtrl.Yaw.ExpectAngle_Act = 1.0;
//                time = 0;
//            }
//        }
//        else if (Distance < 20 && step == 1)
//        {
//            time++;
//            if (time > 3)
//            {
//                step = 2;
//                hCtrl.Pitch.ExpectSpeed_Exp = -hCtrl.Pitch.ExpectSpeed_Exp;
//                hCtrl.Yaw.ExpectAngle_Act = -1.0;
//                time = 0;
//            }
//        }
//        else if (Distance < 10 && step == 2)
//        {
//            time++;
//              if (time > 3)
//              {
//                  step = 3;
//                  hCtrl.Pitch.ExpectSpeed_Exp = -hCtrl.Pitch.ExpectSpeed_Exp;
//                  hCtrl.Yaw.ExpectAngle_Act = 0;
//                  time = 0;
//              }
//        }
//        else if (Distance < 5 && step == 3)
//        {
//            time++;
//            if (time > 1)
//            {
//                step = 4;
//                hCtrl.Pitch.ExpectSpeed_Exp = 0;
//            }
//        }    
        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************
