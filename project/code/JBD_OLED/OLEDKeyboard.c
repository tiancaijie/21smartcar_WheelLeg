#include"zf_common_headfile.h"
#include "OLEDKeyboard.h"
#include "OLED.h"
#include "IIC.h"
#include "isr.h"
#include "Element.h"
extern CtrlHandle_TypeDef hCtrl;
extern PID_HandleTypeDef PID_YawOmega;
extern PID_HandleTypeDef PID_YawAngle;
extern PID_HandleTypeDef PID_PitchOmega;
extern PID_HandleTypeDef PID_PitchAngle;
extern PID_HandleTypeDef PID_PitchLeg;
extern PID_HandleTypeDef PID_RowAngle;
extern PID_HandleTypeDef PID_YawOffset;
extern float Ctrl_ProP;
extern uint16 Ctrl_Distance2;
extern uint16 Ctrl_Distance;
extern float Stand_Height;
extern float Swerve_P;
extern int Jump_Row;
extern float Roll_LStep;
extern float KP1,KP2,KD,GKD;
extern float Kp_Leg;
extern float Kd_Leg;
uint8 *Test[8] = {"1", "2", "3", "4", "5", "6", "7", "8"};
uint8 *run[4] = {"Speed_Exp"};
uint8 *InitTest[4] = {"PWML1","PWML2","PWMR1","PWMR2"};
uint8 *Balance[8] = {"POmgP","POmgI","POmgD","PAngP","PAngI","PAngD","YAngP","YAngI"};
uint8 *Control[8] = {"gz_sign","gz_set","YOmgP","YOmgI","YOmgD","Leg_P","Leg_I","Leg_D"};
uint8 *control[12] = {"Contrh","Contrh2","SpdCrl","SpdExp","KP1","KP2","KD","YawP","YawD","SwrvP","SaveP","SwrStp"};
uint8 *Jump[8] = {"TExtd","TShrk","TChon","TOver","HExtd","HShrk","HChon","JmpRw"};
uint8 *Element[10] = {"Spdbri","YwOfP","YwOfd","StdHt","CuSpd","RAngP","RAngI","RAngD","Count","Hill"};
uint8 UpperChoose = 0;
int SavePicture = 0;
uint8 PWMTest[4] = {0};
uint32 Balance_PID[8];
uint32 Run_SET[8];
uint32 Control_SET[8];
uint32 Set_Sign[8];
double Set_X[20] = {0};
double Set_Y[20] = {0};
uint32 SubSet_OKb;//璺戣溅妯″紡锛氱?戜竴銆佺?戜簩銆佺?戜笁
float GPS_DisSET[8];
float GPS_AngSET[8];
double CoSub_OKb[100];
uint16 Getpoint_LastFlag = 192;
unsigned char Oled_Model_Choose = 0;
/******************************************************
** Function: Oled_Input
** Description: 閿?鏄捐緭鍏ュ弬鏁?
** Others: 
*******************************************************/
void Oled_Input(void)             
{
    unsigned char SectorNum = 0;    //閿熸枻鎷峰?嬮敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
    int Temp = 0;
    int input = 0;
    uint8 notation;
    
    Oled_Model_Choose = 0;
    CH455_Write();
    OLED_CLS();
    
    OLED_ShowStr(0,0,"1 Sub",2);
    OLED_ShowStr(0,2,"2 RUN",2);
    OLED_ShowStr(0,4,"3 EMP",2);
    OLED_ShowStr(0,6,"4 Bal",2);
    OLED_ShowStr(85,0,"5 Ctrl",2);
    OLED_ShowStr(85,2,"6 XY",2);
    Oled_Model_Choose = KeyboardInput(88, 6);
    OLED_CLS();
    
    SectorNum = Oled_Model_Choose;//閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷?
    
    switch (Oled_Model_Choose)
    {
        case 1:                                            // 閿熸枻鎷峰浘
        {
            flash_read_page(0, 0, &SubSet_OKb, 1);
            //"Subject",
            OLED_ShowStr(0, 0, "Subject", 2);    OLED_Numbers(50, 0, SubSet_OKb);
            input = KeyboardInput(88, 6);
            OLED_CLS();

            while(input > 4)
            {
                OLED_ShowStr(0, 0, "input true number", 2);//"input true number"
                input = KeyboardInput(88, 6);
                OLED_CLS();
            }
            if (input != 0)
            {
                SubSet_OKb = input;

                flash_erase_page(0, 0);
                //瀛?
                flash_write_page(0, 0, &SubSet_OKb, 1);
            }
            OLED_CLS();
            break;
        }
        case 2:
        {
            Temp = (SectorNum - 1) * 8;
            
            for (int i = 0; i < 4; i++)
            {
                flash_read_page_to_buffer(0, Temp + i , 1);
                flash_write_page_from_buffer (0, Standby_Buffer, 1);
                flash_read_page(0, Standby_Buffer, &Run_SET[i], 1);

                OLED_ShowStr(0, 2 * i, run[i], 2);
                OLED_Numbers(First_X, 2 * i, Run_SET[i]);
            }
            Run_SET[0] = KeyboardInput_with_Flash(0, Temp + 0, Second_X, 0);
            Run_SET[1] = KeyboardInput_with_Flash(0, Temp + 1, Second_X, 2);
            Run_SET[2] = KeyboardInput_with_Flash(0, Temp + 2, Second_X, 4);
            Run_SET[3] = KeyboardInput_with_Flash(0, Temp + 3, Second_X, 6);
            OLED_CLS();
            break;
        }
        case 3 :        //control
        {
            Temp = (SectorNum - 1) * 8;
            
            for (int i = 0; i < 4; i++)
            {
                flash_read_page_to_buffer(0, Temp + i , 1);
                flash_write_page_from_buffer(0, Standby_Buffer, 1);
                flash_read_page(0, Standby_Buffer, &Set_Sign[i], 1);

                OLED_ShowStr(0, 2 * i, Test[i], 2);
                OLED_Numbers(First_X, 2 * i, Set_Sign[i]);
            }
            Set_Sign[0] = KeyboardInput_with_Flash(0, Temp + 0, Second_X, 0);
            Set_Sign[1] = KeyboardInput_with_Flash(0, Temp + 1, Second_X, 2);
            Set_Sign[2] = KeyboardInput_with_Flash(0, Temp + 2, Second_X, 4);
            Set_Sign[3] = KeyboardInput_with_Flash(0, Temp + 3, Second_X, 6);
            OLED_CLS();
            
            for (int i = 4; i < 8; i++)
            {
                flash_read_page_to_buffer(0, Temp + i , 1);
                flash_write_page_from_buffer(0, Standby_Buffer, 1);
                flash_read_page(0, Standby_Buffer, &Set_Sign[i], 1);

                OLED_ShowStr(0, 2 * (i - 4), Test[i], 2);
                OLED_Numbers(First_X, 2 * (i - 4), Set_Sign[i]);
            }
            Set_Sign[4] = KeyboardInput_with_Flash(0, Temp + 4, Second_X, 0);
            Set_Sign[5] = KeyboardInput_with_Flash(0, Temp + 5, Second_X, 2);
            Set_Sign[6] = KeyboardInput_with_Flash(0, Temp + 6, Second_X, 4);
            Set_Sign[7] = KeyboardInput_with_Flash(0, Temp + 7, Second_X, 6); 
            OLED_CLS();
            break;
          
            break;
        }
        case 4 :        //Balance
        {
            Temp = (SectorNum - 1) * 8;
            
            for (int i = 0; i < 4; i++)
            {
                flash_read_page_to_buffer(0, Temp + i , 1);
                flash_write_page_from_buffer (0, Standby_Buffer, 1);
                flash_read_page(0, Standby_Buffer, &Balance_PID[i], 1);

                OLED_ShowStr(0, 2 * i, Balance[i], 2);
                OLED_Numbers(First_X, 2 * i, Balance_PID[i]);
            }
            Balance_PID[0] = KeyboardInput_with_Flash(0, Temp + 0, Second_X, 0);
            Balance_PID[1] = KeyboardInput_with_Flash(0, Temp + 1, Second_X, 2);
            Balance_PID[2] = KeyboardInput_with_Flash(0, Temp + 2, Second_X, 4);
            Balance_PID[3] = KeyboardInput_with_Flash(0, Temp + 3, Second_X, 6);
            OLED_CLS();

            for (int i = 4; i < 8; i++)
            {
                flash_read_page_to_buffer(0, Temp + i , 1);
                flash_write_page_from_buffer (0, Standby_Buffer, 1);
                flash_read_page(0, Standby_Buffer, &Balance_PID[i], 1);

                OLED_ShowStr(0, 2 * (i - 4), Balance[i], 2);
                OLED_Numbers(First_X, 2 * (i - 4), Balance_PID[i]);
            }
            Balance_PID[4] = KeyboardInput_with_Flash(0, Temp + 4, Second_X, 0);
            Balance_PID[5] = KeyboardInput_with_Flash(0, Temp + 5, Second_X, 2);
            Balance_PID[6] = KeyboardInput_with_Flash(0, Temp + 6, Second_X, 4);
            Balance_PID[7] = KeyboardInput_with_Flash(0, Temp + 7, Second_X, 6); 
            OLED_CLS();
            
            break;
        }
        case 5 :        //Element
        {
            Temp = (SectorNum - 1) * 8;
            
            for (int i = 0; i < 4; i++)
            {
                flash_read_page_to_buffer(0, Temp + i , 1);
                flash_write_page_from_buffer(0, Standby_Buffer, 1);
                flash_read_page(0, Standby_Buffer, &Control_SET[i], 1);

                OLED_ShowStr(0, 2 * i, Control[i], 2);
                OLED_Numbers(First_X, 2 * i, Control_SET[i]);
            }
            Control_SET[0] = KeyboardInput_with_Flash(0, Temp + 0, Second_X, 0);
            Control_SET[1] = KeyboardInput_with_Flash(0, Temp + 1, Second_X, 2);
            Control_SET[2] = KeyboardInput_with_Flash(0, Temp + 2, Second_X, 4);
            Control_SET[3] = KeyboardInput_with_Flash(0, Temp + 3, Second_X, 6);
            OLED_CLS();
            
            for (int i = 4; i < 8; i++)
            {
                flash_read_page_to_buffer(0, Temp + i , 1);
                flash_write_page_from_buffer(0, Standby_Buffer, 1);
                flash_read_page(0, Standby_Buffer, &Control_SET[i], 1);

                OLED_ShowStr(0, 2 * (i - 4), Control[i], 2);
                OLED_Numbers(First_X, 2 * (i - 4), Control_SET[i]);
            }
            Control_SET[4] = KeyboardInput_with_Flash(0, Temp + 4, Second_X, 0);
            Control_SET[5] = KeyboardInput_with_Flash(0, Temp + 5, Second_X, 2);
            Control_SET[6] = KeyboardInput_with_Flash(0, Temp + 6, Second_X, 4);
            Control_SET[7] = KeyboardInput_with_Flash(0, Temp + 7, Second_X, 6); 
            OLED_CLS();
            
            break;
        }
        case 6 :
        {
            uint32 x_pack = 0;
            uint32 y_pack = 0;
            int32 x_raw = 0;
            int32 y_raw = 0;
            Temp = (SectorNum - 1) * 8 * 2;

            for (int i = 0; i < 8; i++)
            {
                flash_read_page(0, Temp + 2 * i, &x_pack, 1);
                flash_read_page(0, Temp + 2 * i + 1, &y_pack, 1);

                x_raw = (int32)x_pack;
                y_raw = (int32)y_pack;

                OLED_CLS();
                OLED_ShowStr(0, 0, "Point", 2);
                OLED_Numbers(50, 0, i + 1);
                OLED_ShowStr(0, 2, "X", 2);
                OLED_Numbers(First_X, 2, x_raw);
                OLED_ShowStr(0, 4, "Y", 2);
                OLED_Numbers(First_X, 4, y_raw);

                x_raw = KeyboardInput_with_Flash(0, Temp + 2 * i, Second_X, 2);
                y_raw = KeyboardInput_with_Flash(0, Temp + 2 * i + 1, Second_X, 4);

                Set_X[i] = (double)x_raw * 0.001;
                Set_Y[i] = (double)y_raw * 0.001;

                OLED_CLS();
            }

            OLED_CLS();
            break;
        }
        case 8 :
        {
            Temp = (SectorNum - 1) * 8;
            
            flash_read_page(0, Temp, (uint32*)CoSub_OKb, 100);
            
            OLED_ShowStr(0, 0, "PoNum", 2);             OLED_Numbers(50, 0, (int32)CoSub_OKb[0]);
            OLED_ShowStr(0, 4, "Distance", 2);          OLED_Numbers(50, 4, (int32)CoSub_OKb[1]);
            
            input = KeyboardInput(88, 6);
            
            while(input > 50)
            {
                input = KeyboardInput(88, 0);
            }
            if(input != 0)
            {
                CoSub_OKb[0] = (double)input;
            }
            input = KeyboardInput(88, 6);
            if(input != 0)
            {
                CoSub_OKb[1] = (double)input;
            }
            OLED_CLS();
            
            OLED_ShowStr(0, 0, "GPWay", 2);             OLED_Numbers(40, 0, (int32)CoSub_OKb[2]);
            OLED_ShowStr(0, 1, "1.Hand", 2);
            OLED_ShowStr(0, 2, "2.RT", 2);

            OLED_ShowStr(0, 4, "Side", 2);              OLED_Numbers(40, 4, (int32)CoSub_OKb[3]);
            OLED_ShowStr(0, 5, "1.Left", 2);
            OLED_ShowStr(0, 6, "2.Right", 2);
            
            do
            {
                input = KeyboardInput(88,0);
            }while (input > 2);
            if (input != 0)
            {
                CoSub_OKb[2] = (double)input;
            }
            
            do
            {
                input = KeyboardInput(88,6);
            }while (input > 2);

            if (input != 0)
            {
                CoSub_OKb[3] = (double)input;
            }
            OLED_CLS();
            
            if(CoSub_OKb[2] == 1)//鎵嬭緭
            {
                for(uint16 i = 0; i < 2*CoSub_OKb[0] - 2; i++)
                {
                    OLED_Numbers(0, 0, i);                             OLED_ShowStr(50, 0, "A", 2);
                    OLED_ShowStr(0, 2, "X", 2);                        OLED_Numbers(30, 2, (int)CoSub_OKb[4*i+4]);
                    OLED_ShowStr(0, 4, "Y", 2);                        OLED_Numbers(30, 4, (int)CoSub_OKb[4*i+5]);

                    do
                    {
                        input = KeyboardInput(88,0);
                    }while(input > 2);
                    if (input != 0)
                    {
                        notation = (uint8)input;
                    }

                    input = KeyboardInput(88,6);
                    if(input != 0)
                    {
                        if (input == 99999)
                        {
                            CoSub_OKb[4*i+4] = 0 ;
                        }
                        else if(notation == 1)
                        {
                            CoSub_OKb[4*i+4] = (double)input;
                        }
                        else if(notation == 2)
                        {
                            CoSub_OKb[4*i+4] = (double)-input;
                        }
                     }

                    input = KeyboardInput(88, 0);
                    if(input != 0)
                    {
                        if (input == 99999)
                        {
                            CoSub_OKb[4*i+5] = 0 ;
                        }
                        else
                        {
                            CoSub_OKb[4*i+5] = (double)input;
                        }

                     }

                    OLED_CLS();

                    OLED_Numbers(0, 0, i);                              OLED_ShowStr(50, 0, "B", 2);
                    OLED_ShowStr(0, 2, "X", 2);                         OLED_Numbers(30, 2, (int)CoSub_OKb[4*i+6]);
                    OLED_ShowStr(0, 4, "Y", 2);                         OLED_Numbers(30, 4, (int)CoSub_OKb[4*i+7]);

                    do{
                        input = KeyboardInput(90, 1);
                    }while(input > 2);

                    if (input != 0)
                    {
                        notation = (uint8)input;
                    }

                    input = KeyboardInput(90, 2);
                    if(input != 0)
                    {
                        if (input == 99999)
                        {
                            CoSub_OKb[4*i+6] = 0 ;
                        }
                        else if(notation == 1)
                        {
                            CoSub_OKb[4*i+6] = (double)input;
                        }
                        else if(notation == 2)
                        {
                            CoSub_OKb[4*i+6] = (double)-input;
                        }
                     }

                    input = KeyboardInput(90, 4);
                    if(input != 0)
                    {
                        if (input == 99999)
                        {
                            CoSub_OKb[4*i+7] = 0 ;
                        }
                        else
                        {
                            CoSub_OKb[4*i+7] = (double)input;
                        }
                    }
                    OLED_CLS();
                }

                OLED_ShowStr(0, 2, "Back.y", 2);
                OLED_Numbers(50, 2, (int32)CoSub_OKb[99]);

                input = KeyboardInput(90, 2);
                if(input != 0)
                {
                    if (input == 99999)
                    {
                        CoSub_OKb[99] = 0 ;
                    }
                    else
                    {
                        CoSub_OKb[99] = (double)input;
                    }

                 }
            }
            else if(CoSub_OKb[2] == 2)//RT
            {
                hCtrl.OLEDState = ING;
//                KeyboardInput_RT(90, 0, 2, 1.0, 2*(2*CoSub_OKb[0] - 2), Hyper_Select);
            }

            OLED_CLS();

            flash_erase_page(0, Temp);
            flash_write_page(0, Temp, (uint32*)CoSub_OKb, 100);
            break;
        }
        
    }
    Set_flash_read();
    Run_flash_read();
    Balance_flash_read();
    //PID_YawAngle.kp = 0.0 ;
    Control_flash_read();
    Hyperparameter_Init();
}

void Get_Point(void)
{

    unsigned char SectorNum = 0;    //閿熸枻鎷峰?嬮敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
    int Temp = 0;
    int input = 0;
    uint32 notation;
    
    Oled_Model_Choose = 0;
    CH455_Write();
    OLED_CLS();
    
    OLED_ShowStr(0,0,"1 Run",2);
    OLED_ShowStr(0,2,"2 Pot",2);
    OLED_ShowStr(0,4,"3 ",2);
    
    Oled_Model_Choose = KeyboardInput(88, 6);
    OLED_CLS();
    
    SectorNum = Oled_Model_Choose;//閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷?
    
    switch (Oled_Model_Choose)
    {
        case 1 :
        {
            flash_read_page_to_buffer(0, 95, 1);
            flash_write_page_from_buffer(0, Standby_Buffer, 1);
            flash_read_page(0, Standby_Buffer, &notation, 1);
            
            OLED_ShowStr(0,0,"mode",2);
            OLED_ShowStr(0,2,"GPS  0",2);
            OLED_ShowStr(0,4,"INS  1",2);
            OLED_Numbers(50,0,notation);
            input = KeyboardInput(88, 6);
            
            if(input == 0)
            {
                Run_GPS = 1;
                Run_INS = 0;
                notation = 0;
                flash_write_page(0, 95, &notation, 1);
            }
            else
            {
                Run_GPS = 0;
                Run_INS = 1;
                notation = 1;
                flash_write_page(0, 95, &notation, 1);
            }
            OLED_CLS();
            break;
        }
        case 2 :
        {
            OLED_ShowStr(0,0,"pot number",2);
//          OLED_Show_float(First_X,2,Navigation.current.X, 4, 1);
//          OLED_Show_float(First_X,4,Navigation.current.Y, 4, 1);
//          OLED_Show_float(First_X,6,Navigation.current.Azimuth, 4, 1);
            
            input = KeyboardInput(88, 6);
            while(input > 8)
            {
                OLED_ShowStr(0, 0, "max 8 points", 2);
                input = KeyboardInput(88, 6);
                OLED_CLS();
            }
                   
            OLED_CLS();

            int i = 1;          
            Getpoint_LastFlag = uart_receiver.channel[2]; 

            while(i <= input)
            {
                
                OLED_Numbers(First_X,0,Navigation.MarkPot.Point_Order);
                OLED_ShowStr(0,2,"wait enter",2);

                while(1)
                {
                    if( (Getpoint_LastFlag == 192 && uart_receiver.channel[2] == 1792) || 
                        (Getpoint_LastFlag == 1792 && uart_receiver.channel[2] == 192) )
                    {
                        Getpoint_LastFlag = uart_receiver.channel[2];
                        break;
                    }
                    for(int i = 0; i < 10 ; i++); // 寮€鍏虫秷鎶?
                }

                Set_X[i] = INSData.Position_x;
                Set_Y[i] = INSData.Position_y;
                
                i++; // 涓嬩竴涓?鐐?
                Navigation.MarkPot.Point_Order++;
                OLED_CLS();
            }

            Temp = 80;
            for(int i = 0; i < 8; i++)
            {
                int32 x_raw = 0;
                int32 y_raw = 0;
                uint32 x_pack = 0;
                uint32 y_pack = 0;

                if(i + 1 <= input)
                {
                    x_raw = (int32)(Set_X[i + 1] * 1000.0);
                    y_raw = (int32)(Set_Y[i + 1] * 1000.0);
                }

                x_pack = (uint32)x_raw;
                y_pack = (uint32)y_raw;
                Set_X[i] = (double)x_raw * 0.001;
                Set_Y[i] = (double)y_raw * 0.001;

                flash_write_page(0, Temp++, &x_pack, 1);
                flash_write_page(0, Temp++, &y_pack, 1);
            }

            OLED_CLS();
           // Balance_flash_read();
            break;
        }
        case 3:
        {
              //Balance_flash_read();
            break;
        }
    }
    //Balance_flash_read();
}
            

/******************************************************
** Function: Oled_Display
** Description: OLED鏄剧ず鍥惧儚鎴栬€呭彉閲?
** Others: 
*******************************************************/
void Oled_Display(void)                 //閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷风ず
{
    if(Oled_Model_Choose == 5)
    {
        OLED_Show_float(0,0,IMUData.ax,4,1); 
        OLED_Show_float(0,1,IMUData.ay,4,1);
        OLED_Show_float(0,2,IMUData.az,4,1);
        OLED_Show_float(0,3,IMUData.gx,2,1); 
        OLED_Show_float(0,4,IMUData.gy,2,1); OLED_Show_float(60,4,motor_value.receive_left_speed_data,4,1);
        OLED_Show_float(0,5,IMUData.gz,2,1); OLED_Show_float(60,5,motor_value.receive_right_speed_data,4,1);
    }
    else
    {
        OLED_Show_float(0,0,IMUData.ax,4,1); 
        OLED_Show_float(0,1,IMUData.ay,4,1);
        OLED_Show_float(0,2,IMUData.az,4,1); OLED_Show_float(60,2,hCtrl.Pitch.ExpectSpeed_Exp,4,1);
        OLED_Show_float(0,3,IMUData.gx,2,1); OLED_Show_float(60,3,Now_Dot,4,1);
        OLED_Show_float(0,4,IMUData.gy,2,1); OLED_Show_float(60,4,Distance,4,1);
        OLED_Show_float(0,5,IMUData.gz,2,1); OLED_Show_float(60,5,Now_Dot_Flag,4,1);
    }
}

/******************************************************
** Function: casex_Read
** Description: 璇诲彇瀵瑰簲case涓?鐨勫彉閲?
** Others: 
*******************************************************/
void Run_flash_read(void)
{
    int case2_Temp = 8;
    int arr_Temp = 0;
    flash_read_page(0, case2_Temp++, &Run_SET[arr_Temp++], 1);
    flash_read_page(0, case2_Temp++, &Run_SET[arr_Temp++], 1);
    flash_read_page(0, case2_Temp++, &Run_SET[arr_Temp++], 1);
    flash_read_page(0, case2_Temp++, &Run_SET[arr_Temp++], 1);
    flash_read_page(0, case2_Temp++, &Run_SET[arr_Temp++], 1);
    flash_read_page(0, case2_Temp++, &Run_SET[arr_Temp++], 1);
    
    arr_Temp = 0;
    hCtrl.Pitch.ExpectSpeed_Exp = Run_SET[arr_Temp++];
}

void Balance_flash_read(void)
{
    int case4_Temp = 24;
    int arr_Temp   = 0;
    flash_read_page(0, case4_Temp++, &Balance_PID[arr_Temp++], 1);
    flash_read_page(0, case4_Temp++, &Balance_PID[arr_Temp++], 1);
    flash_read_page(0, case4_Temp++, &Balance_PID[arr_Temp++], 1);
    flash_read_page(0, case4_Temp++, &Balance_PID[arr_Temp++], 1);
    flash_read_page(0, case4_Temp++, &Balance_PID[arr_Temp++], 1);
    flash_read_page(0, case4_Temp++, &Balance_PID[arr_Temp++], 1);
    flash_read_page(0, case4_Temp++, &Balance_PID[arr_Temp++], 1);
    flash_read_page(0, case4_Temp  , &Balance_PID[arr_Temp]  , 1);

    arr_Temp = 0;
    
    PID_PitchOmega.kp = Balance_PID[arr_Temp++] * 1;
    PID_PitchOmega.ki = Balance_PID[arr_Temp++] * 1;
    PID_PitchOmega.kd = Balance_PID[arr_Temp++] * 1;
    PID_PitchAngle.kp = Balance_PID[arr_Temp++] * 0.1;
    PID_PitchAngle.ki = Balance_PID[arr_Temp++] * 0.1;
    PID_PitchAngle.kd = Balance_PID[arr_Temp++] * 0.1;
    PID_YawAngle.kp   = Balance_PID[arr_Temp++] * 0.01;
    PID_YawAngle.ki   = Balance_PID[arr_Temp]   * 0.01;
}

void Control_flash_read(void)
{
    int case5_Temp = 32;
    int arr_Temp = 0;
    flash_read_page(0, case5_Temp++, &Control_SET[arr_Temp++], 1);
    flash_read_page(0, case5_Temp++, &Control_SET[arr_Temp++], 1);
    flash_read_page(0, case5_Temp++, &Control_SET[arr_Temp++], 1);
    flash_read_page(0, case5_Temp++, &Control_SET[arr_Temp++], 1);
    flash_read_page(0, case5_Temp++, &Control_SET[arr_Temp++], 1);
    flash_read_page(0, case5_Temp++, &Control_SET[arr_Temp++], 1);
   
    arr_Temp = 0;

    hCtrl.Pitch.BalanceSign  = Control_SET[arr_Temp++] * 1;
    hCtrl.Pitch.BalancePoint = Control_SET[arr_Temp++] * 0.0001 * (hCtrl.Pitch.BalanceSign == 0 ? (1):(-1));
    PID_YawOmega.kp = Control_SET[arr_Temp++] * 1;
    PID_YawOmega.ki = Control_SET[arr_Temp++] * 1;
    PID_YawOmega.kd = Control_SET[arr_Temp++] * 1;
    PID_PitchLeg.kp = Control_SET[arr_Temp++] * 0.01;
    PID_PitchLeg.ki = Control_SET[arr_Temp++] * 0.01;
    PID_PitchLeg.kd = Control_SET[arr_Temp]   * 0.01;
}

void Set_flash_read(void)
{
        int case_Temp = 80;
        uint32 x_pack = 0;
        uint32 y_pack = 0;
        for(int i = 0; i < 8; i++)
        {
            flash_read_page(0, case_Temp++, &x_pack, 1);
            flash_read_page(0, case_Temp++, &y_pack, 1);
            Set_X[i] = (double)((int32)x_pack) * 0.001;
            Set_Y[i] = (double)((int32)y_pack) * 0.001;
        }

    case_Temp = 16;
    for(int i = 0; i < 8; i++)
    {
      flash_read_page(0, case_Temp++, &Set_Sign[i], 1);
    }
}
      
void GPS_flash_read(void)
{

}

void Hyperparameter_Init(void)
{
    //璺戣溅妯″紡
    flash_read_page(0, 0, &SubSet_OKb, 1);
    hCtrl.Subject_Set = SubSet_OKb;
    switch(hCtrl.Subject_Set)
    {
        case 1:
        {
            hCtrl.Sub1.Sub1_finish = 0;
            CoCtrl.Sub1.Po_Goal = 1;
        }break;

        case 2:
        {
            hCtrl.Sub2.Sub2_finish = 0;
            CoCtrl.Sub2.Line_Goal = 0;
        }break;

        case 3:
        {
            hCtrl.Sub3.Sub3_finish = 0;
            CoCtrl.Sub3.Line_Goal = 0;
        }break;
        case 4:
        {
            hCtrl.Sub4.Sub4_finish = 0;
        }break;
    }
}