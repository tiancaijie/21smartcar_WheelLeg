#ifndef __OLED_H__
#define __OLED_H__

//#include "Common.h"
//
//#include "OLEDfun.h"
//#include "OLEDKeyboard.h"
//#include "ssd1306.h"
//#include "IIC.h"
//#include "CH455.h"
//#include "zf_eeprom.h"
////#include "DFlash.h"
#include "zf_common_headfile.h"

#define First_X                 50  //上次参数的X坐标
#define Second_X                85  //本次输入参数的X坐标
#define MAX_PAGE_NUM 50
#define BACKUP_SECTOR 11    //用于临时备份的扇区
#define OLED_CLS(void)       OLED_Fill(0x00)

#define KEY_ENTER 11    //确认键
#define KEY_BACK  10    //退格键
#define KEY_BLANK 12    //空值  用于跳出循环
#define KEY_ENTER_TIME  200    //确认计时

#define flash_read(SectorNum, page, buf, len)       flash_read_page(SectorNum, page, buf, len)

#endif
