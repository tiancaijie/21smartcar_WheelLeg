/*************************************************
oled.fun.c
**************************************************/
#include "OLED.h"
#include "zf_common_headfile.h"

uint8 Oled_Mode = 0;
//extern byte Run_Track;
float gfRoadOffset = 0;
float gfObstacleOffsetL = 0;
float gfObstacleOffsetR = 0;
float gfLoopOffset = 0;
short int Big_Entry = 0;
short int Big_Exit = 0;
short int Mid_Entry = 0;
short int Mid_Exit = 0;
short int Small_Entry = 0;
short int Small_Exit = 0;
short int giCurve_Redress = 0;
uint32 Clear_Flash = 0;
/*************************************************************************
*  函数名称：unsigned long KeyboardInput_with_Flash(unsigned char x,unsigned char y)
*  功能说明：键显输入且不存储
  * @param    x   ：  OLED显示坐标x
  * @param    y   ：  OLED显示坐标y
  *
*  函数返回：返回键显输入的数
*  修改时间：2020年6月6日
*************************************************************************/
int KeyboardInput(unsigned char x,unsigned char y) //（without Flash）
{
    int KeyCodeOld=0;
    int KeyCode=0;
    int KeyValue = 0;
    int KeyNumber=0;
    int RerturnKeyNumber = 0;
    int Time_One_Key_Confirmation=0;
    int One_Key_Confirmation=0;
 //   char num2str[8];
    while(KeyValue != KEY_ENTER && One_Key_Confirmation==0)
    {
        /*******************************按键识别*******************************/
        KeyCodeOld = KeyCode;
        KeyCode = CH455_Read();
        if(KeyCodeOld - KeyCode==0x40)
        {
            switch(KeyCode)
            {
                case 0x17 :     KeyValue = 1;   break;
                case 0x0f :     KeyValue = 2;   break;
                case 0x07 :     KeyValue = 3;   break;
                case 0x16 :     KeyValue = 4;   break;
                case 0x0e :     KeyValue = 5;   break;
                case 0x06 :     KeyValue = 6;   break;
                case 0x15 :     KeyValue = 7;   break;
                case 0x0d :     KeyValue = 8;   break;
                case 0x05 :     KeyValue = 9;   break;
                case 0x0c :     KeyValue = 0;   break;
                case 0x14 :     KeyValue = 10;  break;   //退格键
                case 0x04 :     KeyValue = 11;  break;   //确认键
                default:                        break;
            }
        }
        else
        {
            KeyValue = KEY_BLANK;
        }
        if(KeyCode==0x44)
        {
          Time_One_Key_Confirmation++;
          if(Time_One_Key_Confirmation > KEY_ENTER_TIME)
          {
            One_Key_Confirmation++;
          }
        }
        /****************************输入数字可退格****************************/

        if(KeyValue != KEY_BACK && KeyValue != KEY_ENTER && KeyValue != KEY_BLANK)   //输入数值
        {
            if(KeyNumber / 1000 != 0)
            {
                KeyNumber = KeyNumber % 1000;
            }
            KeyNumber = KeyNumber * 10 + KeyValue;
        }
        else if(KeyValue == KEY_BACK)    //退格
        {
            KeyNumber = KeyNumber / 10;
        }
        OLED_Numbers(x,y,KeyNumber);
    }
    RerturnKeyNumber = KeyNumber;
    return RerturnKeyNumber;
}

/*************************************************************************
*  函数名称：unsigned long KeyboardInput_with_Flash(unsigned char sector, unsigned short page, unsigned char x,unsigned char y)
*  功能说明：键显输入并存储到flash
  * @param    sector   ：  扇区   范围  0-11
  * @param    page   ：  页   范围  0-1023
  * @param    x   ：  OLED显示坐标x
  * @param    y   ：  OLED显示坐标y
*  函数返回：返回键显输入的数
*  修改时间：2020年6月6日
*************************************************************************/
unsigned long KeyboardInput_with_Flash(unsigned char sector, unsigned short page, unsigned char x,unsigned char y) // (with Flash)
{
    uint32 KeyCodeOld=0;              //键盘扫描
    uint32 KeyCode=0;
    uint32 KeyValue = 0;              //当次键入的值
    uint32 KeyZero=0;                 //清零标志
    uint32 KeyNumber=0;               //当前键入的值
 //   int KeyNumber_Flash = 0;       //Flash中的值
    unsigned long RerturnKeyNumber = 0;      //最终输出的值
    int One_Key_Confirmation=0;
    int Y_coordinate =0;   //参数所在Y值
    int Time_One_Key_Confirmation=0;
    char num2str[8];
    while(KeyValue != KEY_ENTER && One_Key_Confirmation==0)
    {
        /*******************************按键识别*******************************/
        KeyCodeOld = KeyCode;
        KeyCode = CH455_Read();
        if(KeyCodeOld - KeyCode == 0x40)
        {
            switch(KeyCode)
            {
                case 0x17 :     KeyValue = 1;   break;
                case 0x0f :     KeyValue = 2;   break;
                case 0x07 :     KeyValue = 3;   break;
                case 0x16 :     KeyValue = 4;   break;
                case 0x0e :     KeyValue = 5;   break;
                case 0x06 :     KeyValue = 6;   break;
                case 0x15 :     KeyValue = 7;   break;
                case 0x0d :     KeyValue = 8;   break;
                case 0x05 :     KeyValue = 9;   break;
                case 0x0c :     KeyValue = 0;   break;
                case 0x14 :     KeyValue = KEY_BACK;  break;   //退格键
                case 0x04 :     KeyValue = KEY_ENTER;  break;   //确认键
                default:                        break;
            }
        }
        else
        {
            KeyValue = KEY_BLANK;
        }

        if(KeyCode==0x44)//一般不使用
        {
          Time_One_Key_Confirmation++;
          if(Time_One_Key_Confirmation > KEY_ENTER_TIME)
          {
            One_Key_Confirmation++;
            flash_write_page(sector, page, &Clear_Flash, 1);
            //KeyValue = KEY_BLANK;
          }
        }
        /****************************输入数字可退格****************************/

        if(KeyValue != KEY_BACK && KeyValue != KEY_ENTER && KeyValue != KEY_BLANK)   //输入数值
        {
            if(KeyNumber / 1000 != 0)
            {
                KeyNumber = KeyNumber % 1000;
            }
            KeyNumber = KeyNumber * 10 + KeyValue;
        }
        else if(KeyValue == KEY_BACK)    //退格
        {
            KeyNumber = 0;
        }
        sprintf(num2str, "%5d", KeyNumber);
        OLED_ShowStr(x, y,(unsigned char*)num2str, 2);
        /***************************确认是否用Flash****************************/
        if(KeyNumber == 0)      //输入数为0 显示Flash中数
        {
            if(KeyValue == 0)
            {
                KeyZero = 1 ;   //清零标志给1
            }
            else if( KeyValue==10) //输入退格
            {
                KeyZero = 0 ;      //清零标志置0
            }
            else if(KeyValue == KEY_ENTER)
            {
                if(KeyZero==0)     //清零标志为0，读Flash中的值
                {
                    flash_read(0, page, &KeyNumber, 1);//以uint32类型读取备份专用扇区的指定页
                    flash_write_page(sector, page, &KeyNumber, 1);//以uint32类型将指定扇区的指定页写为KeyNumber
                }
                else              //清零标志为1,写入0
                {
                    flash_write_page(sector, page, &Clear_Flash, 1);//以uint32类型将指定扇区的指定页写为0
                }
            }
        }
        else                    //显示实际数
        {
            if(KeyValue == KEY_ENTER)//确认 存储flash
            {
                flash_write_page(sector, page, &KeyNumber, 1);//以uint32类型将指定扇区的指定页写为KeyNumber
            }
        }
    }

    RerturnKeyNumber = KeyNumber;

    //暂时无用内容↓
    Y_coordinate=Y_coordinate+2;
    if(Y_coordinate>6)
    {
          Y_coordinate=0;
    }
    if(Y_coordinate>4)
    {
          One_Key_Confirmation=0;
    }
    KeyNumber = 0;
    KeyValue = KEY_BLANK;
    //暂时无用内容↑

    return RerturnKeyNumber;
}


/*************************************************************************
*  函数名称：void Flash_Data_Recover(unsigned char desSector, unsigned char backupSector)
*  功能说明：按复位键后将数据从备份扇区恢复
  * @param    desSector   ：  目的扇区   范围  0-11
  * @param    backupSector   ：  扇区   范围  0-11, 默认11
*  函数返回：TRUE:恢复成功   FLASE:扇区不匹配
*  修改时间：2020年6月6日
*************************************************************************/
boolean Flash_Data_Recover(unsigned char desSector, unsigned char backupSector)
{
    uint32 sectorFlag = 0;
    flash_read_page(0, backupSector, &sectorFlag, 1);//备份扇区的0页存当前备份的扇区
    if(sectorFlag == desSector)//备份扇区已被使用
    {
        uint32 finishFlag = 0;
        flash_read_page(0, backupSector, &finishFlag, 1);//备份扇区的0页存当前备份的扇区
        if(finishFlag != sectorFlag)//flash数据没有输入结束
        {
            int page = 0;
            uint32 backupSectorData = 0;
            uint32 desSectorData = 0;
            for(page = 1023; page > 1024 - MAX_PAGE_NUM; --page)
            {
                flash_read(backupSector, page, &backupSectorData, 1);
                flash_read(desSector, page, &desSectorData, 1);
                if(desSectorData == 0)//desSectorData)
                {
                    flash_write_page(desSector, page, &backupSectorData, 1);
                }
            }
        }
        return TRUE;
    }
    return FALSE;
}
/*************************************************
Function: 
Description:键盘输入    (最多5位)
Details：factor 系数，输入的数在返回时将会乘的系数，1.0为不变
*************************************************/
//int32 KeyboardInput_RT(uint16 x ,uint16 y , TextSize_TypeDef TextSize, float factor, uint8 Point_Num, int32 Hyper) //（without Flash）
//{
//    int KeyValue = KEY_BLANK;   //输入的新数字
//    int KeyValue_old;
//    int NewVlaue = 0;           //输入区显示出来的数字
//
//    static uint8 Show_Point = 0;
//
//    /*先在要输入信息的位置显示一行0*/
//    OLED_Show_Numbers(x, y, 0, TextSize);
//
//    while(KeyValue != KEY_ENTER)
//    {
//        KeyValue_old = KeyValue;
//        KeyValue = CH455_GetOneKey_RT(Show_Point, Point_Num, Hyper);
//
//        /*当前不对长按进行特殊处理*/
//        if(KeyValue > 0x0F && KeyValue != KEY_BLANK)
//        {
//            KeyValue = (KeyValue - 0x0F) >> 4;
//        }
//
//        /* 直接跳过的情况取 0 */
//        if(KeyValue_old == KEY_BLANK && (KeyValue == KEY_ENTER || KeyValue == KEY_ENTER_Long))
//        {
//            NewVlaue = 0;
//        }
//
//        /****************************输入数字可退格****************************/
//        if(KeyValue != KEY_BACK && KeyValue != KEY_ENTER && KeyValue != KEY_BLANK)   //输入数值
//        {
//            if(NewVlaue / 10000 != 0)       //只保留四位
//            {
//                NewVlaue = NewVlaue % 10000;
//            }
//
//            NewVlaue = NewVlaue * 10 + KeyValue;      //新输入的数字放到个位
//        }
//        else if(KeyValue == KEY_BACK)    //退格
//        {
//            NewVlaue = NewVlaue / 10;     //最后一位去掉
//        }
//
//        OLED_Show_Numbers(x, y, NewVlaue, TextSize);   //显示当前值
//
//
//
//
//        if (NewVlaue == 4 && Show_Point > 0)
//        {
//            OLED_CLS();
//            Show_Point--;
//        }
//        else if(NewVlaue == 6 )
//        {
//            if ((Hyper == 6 || Hyper == 7) && Show_Point < Point_Num)
//            {
//                OLED_CLS();
//                Show_Point++;
//            }
//            else if (Hyper == 8 && Show_Point < Point_Num - 1 )
//            {
//                OLED_CLS();
//                Show_Point++;
//            }
//        }
//        else if(NewVlaue == 5)
//        {
//            if(Hyper == 6)//科目二
//            {
//                if(Show_Point == Point_Num/2)
//                {
//                   CoSub2_OKb[396] = CoCtrl.Car.INS.y;
//                }
//                else if (Show_Point < Point_Num/2)
//                {
//                     CoSub2_OKb[2*Show_Point + 4] = CoCtrl.Car.INS.x;
//                     CoSub2_OKb[2*Show_Point + 5] = CoCtrl.Car.INS.y;
//                }
//                else if (Show_Point > Point_Num/2)
//                {
//                     CoSub2_OKb[2*(Show_Point - 1) + 4] = CoCtrl.Car.INS.x;
//                     CoSub2_OKb[2*(Show_Point - 1) + 5] = CoCtrl.Car.INS.y;
//                }
//            }
//            else if(Hyper == 7)//科目三
//            {
//                if (Show_Point < Point_Num)
//                {
//                     CoSub3_OKb[2*Show_Point + 3] = CoCtrl.Car.INS.x;
//                     CoSub3_OKb[2*Show_Point + 4] = CoCtrl.Car.INS.y;
//                }
//                else if(Show_Point == Point_Num)
//                {
//                    CoSub3_OKb[35] = CoCtrl.Car.INS.y;
//                }
//            }
//            else if(Hyper == 8)//科目四
//            {
//                if (Show_Point < Point_Num - 1)
//                {
//                     CoSub4_OKb[2*Show_Point + 14] = CoCtrl.Car.INS.x;
//                     CoSub4_OKb[2*Show_Point + 15] = CoCtrl.Car.INS.y;
//                }
//            }
//         }
//         NewVlaue = 0;
//    }
//
//    NewVlaue *= factor;
//
//    return NewVlaue;
//}