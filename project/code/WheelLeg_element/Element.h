/*
 * Element.h
 *
 *  Created on: 2025Äê5ÔÂ13ÈÕ
 *      Author: Zzio
 */

#ifndef CODE_WHEELLEG_ELEMENT_ELEMENT_H_
#define CODE_WHEELLEG_ELEMENT_ELEMENT_H_
#include "zf_common_headfile.h"
#include "pid.h"

#define Leg_SLimit 0.023
#define Leg_LLimit 0.08
extern float Cushion_Speed;
extern uint16 Extend_time;
extern uint16 Shrink_time;
extern uint16 Cushion_time;
extern uint16 Over_time;
extern float Extend_Height;
extern float Shrink_Height;
extern float Cushion_Height;
extern float Stand_Height;
extern int BridgeUP_Flag,BridgeDOWN_Flag;
extern int Judge_Count;
extern int Bridge_Count;
extern int Single_Bridge;
extern int Single_Out;
extern int Single_OutCount;
extern uint8 Bridge_Type;
extern uint8 Now_Dot;
extern uint8 Now_Dot_Flag;
extern uint32 All_Dot;
extern uint32 Jump_Element[8];
extern uint32 Element_Flag[8];
extern int Roll_Allance;
extern int JumpOff;
extern int Jump_Finish_Flag;
extern int Jump_Finish_Flag1;
extern float Row_LegOffset[2];
extern float Final_deviation[3];
extern float Jump_Dis[2];
extern float Row_Mid;
extern float Single_Angle;
extern float Expect_Angle;
extern float Expect_Angle_Pitch;
extern uint8 Bridge1_Start,Bridge1_End,Bridge2_Start,Bridge2_End,Bridge_Type;
extern uint8 Run_GPS ;
extern uint8 Run_INS ;
extern float Speed_Compensation;
void Angle_Set(void);
void Speed_Set(void);
void Single_BridgeJudgement(void);
void Find_SingleBridge(void);
void Single_BridgeCount(void);
void Single_BridgeStateCTRL(void);
void Cut_OffJudgement(void);
void FindEdgeCut(void);
void JumpCTRL(void);
void Cut_offState(void);
void Recover(void);
void Run_Start(void);
void Reset_StartPot(void);
void Element_Play(void);
#endif /* CODE_WHEELLEG_ELEMENT_ELEMENT_H_ */
