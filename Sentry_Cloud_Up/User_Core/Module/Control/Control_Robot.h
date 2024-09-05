#ifndef _CONTROL_ROBOT_H_
#define _CONTROL_ROBOT_H_

#include "DR16.h"
#include "Control_Chassis.h"
#include "Control_Gimbal.h"
#include "Control_Shoot.h"
#include "Control_Vision.h"

//机器人工作状态
enum RobotState_e
{
	Robot_Initializing, //从失能进入到正常状态的过渡阶段，开机时候初始化的阶段，机器人设备离线检测
	Robot_Disability,	//失能状态，应对紧急情况和开机启动
	Robot_Activating	//正常运行状态
};

typedef struct{
	
		uint64_t RX : 11;  // 三个 11 bits 的数据
    uint64_t RY : 11;
    uint64_t DW : 11;
    uint64_t SL : 2;   // 两个 2 bits 的数据
    uint64_t SR : 2;
    uint64_t red_outpost_hp : 11;
    uint64_t blue_outpost_hp : 16;
	
}DR16_Rev;

typedef union{
	
	uint8_t Data[8];
	DR16_Rev pack;

}DR16_RX;

typedef struct
{
	uint64_t shooter_barrel_heat_limit :9;
	uint64_t shooter_17mm_1_barrel_heat :9;
	uint64_t shooter_17mm_2_barrel_heat :9;
	uint64_t game_progress :3;
	uint64_t team_color :1;//发送自己队伍的颜色 蓝->1
	uint64_t stage_remain_time:9;
	uint64_t my_output :1;//我方前哨站存在=1，被击毁=0
	uint64_t enemy_output :1;//前哨站存在=1，被击毁=0
	uint64_t sentry_hp :9;
	uint64_t bullet_remain :10;
	uint64_t is_attacked :1;
}robot_state;

typedef union
{
	uint8_t data[8];
	robot_state pack;
}Robot_State;


class Robot_classdef
{
public:
    RobotState_e State;
    DR16Status_Typedef Ctrl_Source;
    CHAS_CtrlMode_e Chassis_Mode;
    CHAS_CtrlMode_e Chassis_PreMode;
    Gimbal_CtrlMode_e Gimbal_Mode;
    Attack_CtrlMode_e Attack_Mode;
    Fric_CtrlMode_e Fric_Mode;
    VisionMode_e Vision_Mode;
		DR16_RX Rev_DR16;
		Robot_State robot_state;
    uint8_t ID;          /*<! 机器人ID */
    uint8_t PC_State;    /*<! PC状态 */
    uint8_t SendData[8]; /*<! 板间通信存储数据 */
    uint8_t Write_Msg[2][8];
    uint8_t Reflash_UI;  /*<! UI刷新 */
    uint8_t ResetFlag = false;   /*<! 重启标志位 */
    uint8_t Chas_Warning;   /*<! 底盘状态警告 */
    uint16_t FPS;
    WorldTime_t TIME;
		uint16_t enemy_output;
		uint16_t my_output;
		uint8_t TeamColor;
		uint8_t God_State[5];
		uint8_t CanShootNum[2];
		uint16_t shooter_id1_17mm_heat;
		uint16_t shooter_barrel_heat_limit;
		
		uint8_t HurtArmor_NoZero;
		uint8_t ShootRecNum,lastShootRecNum,mul_num;
		uint16_t TrueShootRecNum;
		uint8_t CoolBuff;
		uint8_t Enemy_Sentry_God_State;
    uint8_t Send2C_Msg[8];/*<! 发送给板的数据 */
    Robot_classdef();
    void Enable();
    void Disable();
    void Reset();
    void Control();
    void CtrlSource_Switch(DR16Status_Typedef mode);
    void Set_ChassisMode(CHAS_CtrlMode_e mode);
    void Set_GimbalMode(Gimbal_CtrlMode_e mode);
    void Set_AttackMode(Attack_CtrlMode_e mode);
    void Set_VisionMode(VisionMode_e mode);
    void Send2C_Board();
    void Gimbal_Ctrl();

    void SendMsgtoCHAS();
    void RefereeMsg_Write(uint16_t id, uint8_t can_rx_data[]);

    uint8_t Get_ID();
		uint8_t Get_TeamColor();
    uint16_t Get_FPS();

    RobotState_e Get_State();
    DR16Status_Typedef Get_CtrlSource();
    CHAS_CtrlMode_e Get_ChassisMode();
    CHAS_CtrlMode_e Get_ChassisPreMode();
    Gimbal_CtrlMode_e Get_GimbalMode();
    Attack_CtrlMode_e Get_AttackMode();
    Fric_CtrlMode_e Get_FricMode();
    VisionMode_e Get_VisionMode();
    
};


#endif
