#include "Map.h"

INSData_TypeDef INSData;


Navigation_Handle_TypeDef Navigation = {
                                        .current.MapCount   = 1,
                                        .current.k          = 0.25,
                                        .SpdExp_Far  = 400,
                                        .SpdExp_Near = 200,

                                        .Map.Point_Num      = 1,

                                        .MarkPot.Point_Order = 1,
                                        .MarkPot.Mode        = No_MarkMode,

                                       };

void Hand_PotRecord(void)
{
//    unsigned char SectorNum = 0;    //��ʼ������
//    int Temp = 0;
//    int Oled_Choose = 0;
//    CH455_Write();
//    OLED_CLS();
//    
//    OLED_ShowStr(0,0,"dot_select",2);
//    
//    Oled_Choose = KeyboardInput(88, 6);
//    OLED_CLS();
//    
//    SectorNum = Oled_Choose;//������
//    
//    switch (Oled_Choose)
//    {
//        case 1:                                            // ��ͼ
//        {
//            OLED_CLS();
//            break;
//        }
//        case 2:
//        {
//            Temp = (SectorNum - 1) * 8;
//            
//            for (int i = 0; i < 4; i++)
//            {
//                flash_read_page_to_buffer(0, Temp + i , 1);
//                flash_write_page_from_buffer (0, Standby_Buffer, 1);
//                flash_read_page(0, Standby_Buffer, &Run_SET[i], 1);
//
//                OLED_ShowStr(0, 2 * i, run[i], 2);
//                OLED_Numbers(First_X, 2 * i, Run_SET[i]);
//            }
//            Run_SET[0] = KeyboardInput_with_Flash(0, Temp + 0, Second_X, 0);
//            Run_SET[1] = KeyboardInput_with_Flash(0, Temp + 1, Second_X, 2);
//            Run_SET[2] = KeyboardInput_with_Flash(0, Temp + 2, Second_X, 4);
//            Run_SET[3] = KeyboardInput_with_Flash(0, Temp + 3, Second_X, 6);
//            OLED_CLS();
//            break;
//        }
//        case 3 :        //control
//        {
//            break;
//        }
//        case 4 :        //Balance
//        {
//            Temp = (SectorNum - 1) * 8;
//            
//            for (int i = 0; i < 4; i++)
//            {
//                flash_read_page_to_buffer(0, Temp + i , 1);
//                flash_write_page_from_buffer (0, Standby_Buffer, 1);
//                flash_read_page(0, Standby_Buffer, &Balance_PID[i], 1);
//
//                OLED_ShowStr(0, 2 * i, Balance[i], 2);
//                OLED_Numbers(First_X, 2 * i, Balance_PID[i]);
//            }
//            Balance_PID[0] = KeyboardInput_with_Flash(0, Temp + 0, Second_X, 0);
//            Balance_PID[1] = KeyboardInput_with_Flash(0, Temp + 1, Second_X, 2);
//            Balance_PID[2] = KeyboardInput_with_Flash(0, Temp + 2, Second_X, 4);
//            Balance_PID[3] = KeyboardInput_with_Flash(0, Temp + 3, Second_X, 6);
//            OLED_CLS();
//
//            for (int i = 4; i < 8; i++)
//            {
//                flash_read_page_to_buffer(0, Temp + i , 1);
//                flash_write_page_from_buffer (0, Standby_Buffer, 1);
//                flash_read_page(0, Standby_Buffer, &Balance_PID[i], 1);
//
//                OLED_ShowStr(0, 2 * (i - 4), Balance[i], 2);
//                OLED_Numbers(First_X, 2 * (i - 4), Balance_PID[i]);
//            }
//            Balance_PID[4] = KeyboardInput_with_Flash(0, Temp + 4, Second_X, 0);
//            Balance_PID[5] = KeyboardInput_with_Flash(0, Temp + 5, Second_X, 2);
//            Balance_PID[6] = KeyboardInput_with_Flash(0, Temp + 6, Second_X, 4);
//            Balance_PID[7] = KeyboardInput_with_Flash(0, Temp + 7, Second_X, 6); 
//            OLED_CLS();
//            
//            break;
//        }
//        case 5 :        //Element
//        {
//            Temp = (SectorNum - 1) * 8;
//            
//            for (int i = 0; i < 4; i++)
//            {
//                flash_read_page_to_buffer(0, Temp + i , 1);
//                flash_write_page_from_buffer(0, Standby_Buffer, 1);
//                flash_read_page(0, Standby_Buffer, &Control_SET[i], 1);
//
//                OLED_ShowStr(0, 2 * i, Control[i], 2);
//                OLED_Numbers(First_X, 2 * i, Control_SET[i]);
//            }
//            Control_SET[0] = KeyboardInput_with_Flash(0, Temp + 0, Second_X, 0);
//            Control_SET[1] = KeyboardInput_with_Flash(0, Temp + 1, Second_X, 2);
//            Control_SET[2] = KeyboardInput_with_Flash(0, Temp + 2, Second_X, 4);
//            Control_SET[3] = KeyboardInput_with_Flash(0, Temp + 3, Second_X, 6);
//            OLED_CLS();
//            
//            for (int i = 4; i < 8; i++)
//            {
//                flash_read_page_to_buffer(0, Temp + i , 1);
//                flash_write_page_from_buffer(0, Standby_Buffer, 1);
//                flash_read_page(0, Standby_Buffer, &Control_SET[i], 1);
//
//                OLED_ShowStr(0, 2 * (i - 4), Control[i], 2);
//                OLED_Numbers(First_X, 2 * (i - 4), Control_SET[i]);
//            }
//            Control_SET[4] = KeyboardInput_with_Flash(0, Temp + 4, Second_X, 0);
//            Control_SET[5] = KeyboardInput_with_Flash(0, Temp + 5, Second_X, 2);
//            Control_SET[6] = KeyboardInput_with_Flash(0, Temp + 6, Second_X, 4);
//            Control_SET[7] = KeyboardInput_with_Flash(0, Temp + 7, Second_X, 6); 
//            OLED_CLS();
//            
//            break;
//        }
//        case 6 :        //imu
//        {
//            break;
//        }
//        case 7 :        //Jump
//        {
//            break;
//        }
//    }
}

/*==================== 函数定义 ====================*/
/*
 * Function:    INS_Calcu
 * Description: �?导信�?计算
 * input:       �?
 * output:      �?
 */
void INS_Calcu(void)
{
//    double a,b;0.927500  0.923599 0.923050 0.927211
    static char ms20 = 0;

    ms20++;   
    // 编码器�?�值转�?位米
    INSData.Position += ((double)motor_value.receive_left_speed_data - (double)motor_value.receive_right_speed_data) * 0.000001 / 0.5239245 * 3;

    //INSData.Position2 = (INSData.Position + 19 * INSData.Position_last) / 20 ;
    if(ms20 == 19)
    {

        INSData.v_y = INSData.Position * (cos(IMUData.sum_yaw_mahony*PI/180));// 前方是y�?
        INSData.v_x = INSData.Position * (sin(IMUData.sum_yaw_mahony*PI/180));// 右边是x�?
        INSData.Position_x += INSData.v_x;
        INSData.Position_y += INSData.v_y;
        // �?导�?�数归零
//        INSData.Position_last = INSData.Position ;
        INSData.Position = 0;
        ms20 = 0;
    }

}
