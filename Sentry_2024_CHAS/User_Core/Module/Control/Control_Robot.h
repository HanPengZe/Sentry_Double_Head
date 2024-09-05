#ifndef _CONTROL_ROBOT_H_
#define _CONTROL_ROBOT_H_


#include "Control_Chassis.h"
#include "Control_Gimbal.h"
#include "Devices_Monitor.h"

//�����˹���״̬
enum RobotState_e
{
	Robot_Initializing, //��ʧ�ܽ��뵽����״̬�Ĺ��ɽ׶Σ�����ʱ���ʼ���Ľ׶Σ��������豸���߼��
	Robot_Disability,	//ʧ��״̬��Ӧ�Խ�������Ϳ�������
	Robot_Activating	//��������״̬
};

typedef struct
{
		uint64_t RX : 11;  // ���� 11 bits ������
    uint64_t RY : 11;
    uint64_t DW : 11;
    uint64_t SL : 2;   // ���� 2 bits ������
    uint64_t SR : 2;
    uint64_t my_output : 11;
    uint64_t red_outpost_hp : 11;
    uint64_t blue_outpost_hp : 16;
	
}DR_data;

typedef union
{
	uint8_t data[8];
	DR_data pack;
}DR16_data;



/* ���ͨ���������� */
enum Commu_e
{
    Vision_State = 0,
    Fric_State = 1,
    Uphill_Mode = 2,
    Robot_Reset = 3,
    Cap_Ctrl = 4,
    Attack_Ctrl = 5,
    Mag_Ctrl = 6,
    UI_Reflash = 7
};

/* �Ӿ�ģʽ */
enum VisionMode_e
{
	Vision_Disable = 0,		// �Ӿ�ʧ��	
	Vision_Default = 1,		// Ĭ������(�Ӿ����治��Ԥ��)
	Vision_BIGWindmill = 2, // ��糵
	// Vision_Sentry = 3,	// �ڱ�
	// Vision_BASE = 4,		// ����
	// Vision_Top = 5,		// ����
	// Vision_Record = 6	// ¼��
	Vision_Forcast = 6		// �Ӿ�Ԥ��

};
typedef struct
{
	uint64_t shooter_barrel_heat_limit :9;
	uint64_t shooter_17mm_1_barrel_heat :9;
	uint64_t shooter_17mm_2_barrel_heat :9;
	uint64_t game_progress :3;
	uint64_t team_color :1;//�����Լ��������ɫ ��->1
	uint64_t stage_remain_time:9;
	uint64_t my_output :1;//�ҷ�ǰ��վ����=1��������=0
	uint64_t enemy_output :1;//ǰ��վ����=1��������=0
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
    // Attack_CtrlMode_e Attack_Mode;
    // Fric_CtrlMode_e Fric_Mode;
    VisionMode_e Vision_Mode;
    CHAS_CtrlMode_e Get_ChassisPreMode();
    uint8_t Vision_switch;
    uint8_t Vision_Depth;
    uint8_t Write_Msg[8];
    uint8_t RefereeData[8];
		uint8_t DR16_Date[8];
    uint8_t Shoot_Freq;
    uint8_t Onebullet_heat = 10; /*<! �����ӵ�������(С����) */
		DR16_data dr_data;
		Robot_State robot_state;
    uint16_t FPS;
    WorldTime_t TIME;
		
    Robot_classdef();
    void Enable();
    void Disable();
    void Reset();
    void Control();
    void CtrlSource_Switch(DR16Status_Typedef mode);
    void Set_ChassisMode(CHAS_CtrlMode_e mode);
    void Set_GimbalMode(Gimbal_CtrlMode_e mode);
    // void Set_AttackMode(Attack_CtrlMode_e mode);

    void Gimbal_Ctrl();
		void DR16_Send(uint8_t *data);
    void RefereeMsg_Send(uint8_t *data);
     void WriteMsgFromGimbal(uint8_t id,uint8_t can_rx_data[]);
//    void WriteMsgFromGimbal(CanRxMsg_t can_rx_data);

    void ShootMsg_Process(float bullet_speed);

    uint8_t Get_ID();
    uint8_t Get_VisionDepth();
    uint8_t ShootFreq_Calc();    /**<! ��Ƶ���� */
    uint16_t Get_GunHeat();      /**<! ǹ�ܵ�ǰ���� */
    uint16_t Get_CoolingLimit(); /**<! ǹ����ȴ���� */
    uint16_t Get_CoolingRate();  /**<! ǹ����ȴ���� */
    RobotState_e Get_State();
    DR16Status_Typedef Get_CtrlSource();
    CHAS_CtrlMode_e Get_ChassisMode();
    Gimbal_CtrlMode_e Get_GimbalMode();
    // Attack_CtrlMode_e Get_AttackMode();
    // Fric_CtrlMode_e Get_FricMode();
    VisionMode_e Get_VisionMode();
    
};

extern uint32_t Get_RefereeTime();

#endif
