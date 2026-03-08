#ifndef __Map_H
#define __Map_H


//#include "JBD_GPS_tau1201.h"
#include "zf_common_headfile.h"

#define USE_JBD 0  //是否编译传统代码

#define MP_sector 7         //点位存在7扇区
#define MP_Start_Page 1023  //点位起始页




typedef enum
{
    Near = 0,
    Far,
}Dis_State;

typedef enum
{
    MarkerPointType_NULL            = 0,    // 空
    MarkerPointType_Beginning       = 1,    // 起点
    MarkerPointType_Normal             ,    // 普通标记点
    MarkerPointType_Roundabout_right   ,    // 环岛右
    MarkerPointType_Roundabout_left    ,    // 环岛左
    MarkerPointType_MarkerBucket_in    ,    // 入锥桶
    MarkerPointType_MarkerBucket_move  ,    // 开环换
    MarkerPointType_MarkerBucket_out   ,    // 出锥桶
    MarkerPointType_TowingPoint        ,    // 牵引点
    MarkerPointType_Line               ,    // 线
    MarkerPointType_Ramp               ,    // 坡道

    MarkerPointType_U_TurnLine         ,    // 掉头线
    MarkerPointType_U_TrunPoint        ,    // 掉头点
    MarkerPointType_End                ,    // 终点

    MarkerPointType_LineWaitForPointB  ,

}MarkerPointType_enum;

typedef struct __attribute__((__packed__))
{
    // 8 byte
    double                  Longitude;          // 经度
    // 8 byte
    double                  Latitude;           // 纬度
    // 1 byte
    uint8_t                 type;               // 标记点类型
    // 1 byte * 3
    uint8_t                 notused[3];         // 占位
}MarkerPoint_TypeDef;

typedef struct __attribute__((__packed__))
{
    // 4 byte
    float                   X;          // x坐标
    // 4 byte
    float                   Y;           // y坐标
    // 1 byte
    uint8_t                 type;               // 标记点类型
    // 1 byte * 3
    uint8_t                 notused[3];         // 占位
}MarkerPoint_RectTypeDef;//直角系下的标点


typedef union
{
    MarkerPoint_TypeDef     MarkerPoint;
    uint32_t                inWord[5];
}MarkerPoint2uint16_union;

typedef enum
{
    MapState_NULL           = 0,        // 地图为空
    MapState_WaitForMark       ,        // 地图等待打点
    MapState_Marking           ,        // 地图正在打点
    MapState_Ready             ,        // 地图准备就绪
    MapState_WaitForClean      ,        // 地图等待清理（删除之前的标记点）
    MapState_WaitForChange     ,        // 地图等待改点
}MapState_enum;

typedef struct
{
    int8_t Point_Index[2];
}MapElement_Line_TypeDef;

typedef struct
{
    int8_t Point_Index;
}MapElement_TargetPoint_TypeDef;

typedef struct
{
    uint8_t State;
    // From结构体记录此元素的父元素是什么
    // 目前可以是线.点.NULL 如果是线,则Index记录线的编号,如果是点,则记录点的编号,如果是NULL,则编号是-1
    struct
    {
        uint8_t ElementType;
        int8_t  Index;
    }From;

    // Current结构体记录此元素是什么
    // 目前可以是点.直线.圆环.S路
    // 点或圆则记录对应的点的编号,直线记录直线编号,S路则记录S路的编号或是S路首点的编号
    struct
    {
        uint8_t ElementType;
        int8_t  Index;
    }Current;

    // Next结构体记录此元素之后是什么
    // 目前可以是线.点,NULL
    // 如果是线则记录线的编号,如果是点则纪录点的编号,如果是NULL则编号是-1
    struct
    {
        uint8_t ElementType;
        int8_t  Index;
    }Next;
}MapElement_TypeDef;

typedef struct
{
    MapState_enum       MapState;       // 地图当前状态
    uint8_t             MarkerNum;      // 标记点数量    1开始数
    MarkerPoint_TypeDef *point;         // 标记点指针

    int8_t              LinePointCount; // 直线点计数
    int8_t              LineCount;      // 直线计数
    int8_t              ElementCount;   // 赛道元素记录

    MapElement_Line_TypeDef Lines[24];   // 记录地图里打的直线
    MapElement_TypeDef      Elements[50];   // 记录地图里的元素
}MapHandle_TypeDef;


typedef enum
{
    No_MarkMode = 0,
    RC_MarkMode,
    Hand_MarkMode,
    Input_MarkMode,
}MarkPot_Mode;

/* ----导航---- */
typedef struct
{
    struct
    {
        float            X;    //x
        float            Y;    //Y
        float      Azimuth;    //方位角
        float       Sumway;    //总路程
        uint8     MapCount;    //打点计数的序号//修改打点序号的定义，MapCount为0时，存点为(0,0),但此点不作为打的,目标点，而作为辅助点，记录车体初始信息

        float  Azimuth_A2B;
        float   Dis_Car2AB;
        float           vx;
        float            V;
        float Azimuth_V2AB;
        float            k;

        uint8 PointA_Index;
        uint8 PointB_Index;

    }current;

    float    MP_Dis;       //与下个点的距离
    float MP_Dis_La;       //与上个点的距离

    float SpdExp_Act;
    float SpdExp_Near;
    float SpdExp_Far;

    float ExpectAngle;

    Dis_State Dis2MP_Sta;



    struct
    {
        MarkerPoint_RectTypeDef MP_Data[40]; //实际有有效打点下标从1开始到Point_Num,Point_Num + 1位存的是回发车点的位置,0存的是(0,0)作为起始点，辅助计算
        uint8 Point_Num;      //实际打点的个数(不算回库) //实际有有效打点下标从1开始到Point_Num,Point_Num + 1位存的是回发车点的位置,0存的是(0,0)
    }Map;

    struct
    {
        MarkPot_Mode        Mode;
        Flag_State       Is_Here;
        Flag_State     MP_finish;
        uint8        Point_Order;          //从1开始
    }MarkPot;

}Navigation_Handle_TypeDef;

typedef struct
{
        float           SetPosition_x[128];
        float           SetPosition_y[128];

        int             SumPointNum;
        int             NowPointNum;

        float           SetBackPoint_x[32];
        float           SetBackPoint_y[32];

        int             SumBackPointNum;
        int             NowBackPointNum;

        double           Set_v_x;
        double           Set_v_y;

        double           Position_x;
        double           Position_y;
        double           Position;
        double           TargetAngle;
        double           Angle;

        double           v_x;
        double           v_y;

        float            Distance;
}INSData_TypeDef;

extern Navigation_Handle_TypeDef Navigation;
extern INSData_TypeDef INSData;

void  OdometerCal_XY(void);
float Angle_StayInLine_Calc(int8_t PointA_Index, int8_t PointB_Index);
void  Odometer_Guide(void);
void Odometer_Guide_GoStraight(void);
void Hand_PotRecord(void);
void INS_Calcu(void);

#endif /* __Map_h */
