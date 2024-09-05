/* Includes ------------------------------------------------------------------*/
#include "Control_Robot.h"
#include "System_DataPool.h"
#include "DJI_AIMU.h"
uint32_t Get_RefereeTime()
{
	return xTaskGetTickCount()*1000;
}
Robot_classdef::Robot_classdef()
{
    //--- ��ʼ��
	// Referee.Init(&huart3, Get_RefereeTime);	
  
}

/**
 * @brief  	  �������ܿ���
 * @param[in]	None
 * @retval 	  None
 */
//int m=0;

void Robot_classdef::Control()
{
//	m++;
    static uint8_t reset_cnt = 0;

//    if(DevicesMonitor.Get_State(COMMU_0X340_MONITOR) == Off_line)
//    {
//        Chassis.Target_Vx = Chassis.Target_Vy = Chassis.Target_Vw = 0;
//        Chassis_Mode = CHAS_DisableMode;
//        Vision_Mode = Vision_Disable;

//        SupCap.Main_Switch(SupCap_OFF, Charging_OFF, Power_NotSupply);

//        for(uint8_t i = 0 ; i < 8 ; i++)
//        {
//            Write_Msg[i] = false;
//        }
//    }

//    SupCap.Control();   //--- ������� �ڱ�����
    
    Chassis.Control();  //--- ���̿���

//    ShootFreq_Calc();   //--- ���㷢����Ƶ


    // ToDo...
    // if(Write_Msg[Robot_Reset] == true && DevicesMonitor.Get_State(COMMU_0X340_MONITOR) != Off_line)
    // {
    //     reset_cnt++;
    //     if(reset_cnt>=1000) //--- �յ������źų���2s����
    //     {
    //         Reset();
    //     }
    // }
    // else
    // {
    //     reset_cnt = 0;
    // }

    //--- ����ϵͳ��Ϣ��������̨����
	DR16_Send(DR16_Date);
//	if(m==300)
//	{
	RefereeMsg_Send(RefereeData);
//		m=0;
//	}
    DevicesMonitor.Get_FPS(&Infantry.TIME, &Infantry.FPS);
}

/**
 * @brief  	  ������ʹ��
 * @param[in]	None
 * @retval 	  None
 */
void Robot_classdef::Enable()
{
    if (State == Robot_Disability) //--- ����״̬����
    {
        State = Robot_Activating;
    }
    if (State != Robot_Initializing)
    {
        State = Robot_Activating;
    }
}

/**
 * @brief  	  ������ʧ��,״̬��λ
 * @param[in]	None
 * @retval 	  None
 */
void Robot_classdef::Disable()
{
    if (Get_State() != Robot_Initializing || Get_State() != Robot_Disability)
    {
        State = Robot_Disability;
    }

    // CTRL_DR16.ExptData_Reset();
    Set_ChassisMode(CHAS_DisableMode);
    Set_GimbalMode(Gimbal_DisableMode);
    // Set_AttackMode(Attack_Disable);
}

/**
 * @brief  	    ����������(������)
 * @param[in]	None
 * @retval 	    None
 */
void Robot_classdef::Reset()
{   
    //--- оƬ��λ
    __set_FAULTMASK(1);     //--- �ر������ж�
    HAL_NVIC_SystemReset(); //--- ��λ
}

/**
 * @brief  	  �����˿���Դת��
 * @param[in]	mode
 * @retval 	  None
 */
void Robot_classdef::CtrlSource_Switch(DR16Status_Typedef mode)
{
    //--- ����ģʽ����
    if (Get_CtrlSource() != mode)
    {
        // M6020 Yaw Reset
        // Gimbal.Motor[Yaw].Reset();
    }
    Ctrl_Source = mode;
}

/**
 * @brief  	  ���õ�������ģʽ
 * @param[in]	mode
 * @retval 	  None
 */
void Robot_classdef::Set_ChassisMode(CHAS_CtrlMode_e mode)
{
    if ((Get_ChassisMode() == CHAS_SpinMode || Get_ChassisMode() == CHAS_LockMode) && mode != CHAS_SpinMode)
    {
        // Reset M6020 Yaw
        // Gimbal.Motor[Yaw].Reset();
    }

    Chassis.Mode=Chassis_Mode = mode;
}

/**
 * @brief  	  ������̨����ģʽ
 * @param[in]	mode
 * @retval 	  None
 */
void Robot_classdef::Set_GimbalMode(Gimbal_CtrlMode_e mode)
{
    Gimbal_Mode=Gimbal.Mode = mode;
}



/**
 * @brief      ����̨���巢�Ͳ���ϵͳ����
 * @param[in]  can_rx_data
 * @retval     None
 */
// float bulletspeed;
uint16_t Last_GameRobotState_remain_HP;
void Robot_classdef::RefereeMsg_Send(uint8_t *data)
{   
    //--- 0x341
//    uint8_t *temp;
//    float bulletspeed = Referee.ShootData.bullet_speed;
//    temp = (uint8_t*)(&bulletspeed);

//    data[0] = temp[0]; //--- ǹ������
//    data[1] = temp[1];
//    data[2] = temp[2];
//    data[3] = temp[3];

//    data[4] = 0;    //--- ������ID&�����Դ�ӿ�״̬
//    data[4] |= Get_ID();
//    data[4] |= Referee.GameRobotState.mains_power_shooter_output == ON?1<<7:0;

//    data[5] = Referee.GameRobotState.shooter_id1_17mm_speed_limit;  //--- �ӵ����ٶ�����
//    data[6] = Shoot_Freq;   //--- �ӵ���Ƶ
//		//����
//    data[7] = Chassis.Get_PowerLimit(); //--- ���̹�������
	//ǹ����������
//	data[0] = Referee.GameRobotState.shooter_barrel_heat_limit;//�Ͱ�λ
//	data[1] = Referee.GameRobotState.shooter_barrel_heat_limit>>8;//�߰�λ
//	
//	//ǹ��1,2��������
//	data[2] = Referee.PowerHeatData.shooter_id1_17mm_heat;
//	data[3] = Referee.PowerHeatData.shooter_id1_17mm_heat>>8;
  if(Referee.GameRobotState.current_HP!=Last_GameRobotState_remain_HP)
	{
		robot_state.pack.is_attacked = 1;		
	}
	else
	{
		robot_state.pack.is_attacked = 0;		
	}
	
		Last_GameRobotState_remain_HP = Referee.GameRobotState.current_HP;

	if(Referee.GameRobotState.robot_id>=100)//1-7��<----->101-107��
	{
			//�Լ�����1  -----  blue_outpost_HP
		robot_state.pack.team_color = 1;//�ж��Լ�����ɫ
		if(Referee.GameRobotHP.blue_outpost_HP == 0)//�ж��Լ�ǰ��վ�Ƿ����
		{
		robot_state.pack.my_output  = 0;
		}
		else
		{
		robot_state.pack.my_output  = 1;
		}
		if(Referee.GameRobotHP.red_outpost_HP == 0)//�ж϶Է�ǰ��վ�Ƿ����
		{
		robot_state.pack.enemy_output  = 0;
		}
		else
		{
		robot_state.pack.enemy_output  = 1;
		}
		
	}
	else
	{
			//�Լ��췢0  -----   red_outpost_HP
		robot_state.pack.team_color   = 0 ;//�ж��Լ�����ɫ
		if(Referee.GameRobotHP.red_outpost_HP == 0)//�ж��Լ�ǰ��վ�Ƿ����
		{
		robot_state.pack.my_output  = 0;
		}
		else
		{
		robot_state.pack.my_output  = 1;
		}
		if(Referee.GameRobotHP.blue_outpost_HP == 0)//�ж϶Է�ǰ��վ�Ƿ����
		{
		robot_state.pack.enemy_output  = 0;
		}
		else
		{
		robot_state.pack.enemy_output  = 1;
		}
	}
		robot_state.pack.shooter_barrel_heat_limit = Referee.GameRobotState.shooter_barrel_heat_limit;//ǹ����������
		robot_state.pack.shooter_17mm_1_barrel_heat = Referee.PowerHeatData.shooter_id1_17mm_heat;		//ǹ��1��ǰ����
		robot_state.pack.shooter_17mm_2_barrel_heat = Referee.PowerHeatData.shooter_id2_17mm_heat;		//ǹ��2��ǰ����
		robot_state.pack.game_progress = Referee.GameState.game_progress;															//��������
		robot_state.pack.stage_remain_time = Referee.GameState.stage_remain_time;											//��ǰ�׶�ʣ��ʱ��
		robot_state.pack.sentry_hp = Referee.GameRobotState.current_HP;																//����ʣ��Ѫ��
		robot_state.pack.bullet_remain = Referee.BulletRemaining.bullet_remaining_num_17mm;						//ʣ�൯��
		memcpy(data,&robot_state.data,sizeof(robot_state.data));
    CANx_SendData(&hcan1,0x341,data,8);

}
/**
 * @brief      ����̨���巢��DR16����
 * @param[in]  can_rx_data
 * @retval     None
 */
// float bulletspeed;


void Robot_classdef::DR16_Send(uint8_t *data)
{//0000 0000

//	int8_t LX,LY,RX,RY,DW;//��λ���ݵ�����ȡ����
//	LX = uint8_t((int16_t)(DR16.Get_LX_Norm()+660;)%10);
//	LY = uint8_t((int16_t)(DR16.Get_LY_Norm())%10);
//	RX = uint8_t((int16_t)(DR16.Get_RX_Norm())%10);
//	RY = uint8_t((int16_t)(DR16.Get_RY_Norm())%10);
//	DW = uint8_t((int16_t)(DR16.Get_DW_Norm())%10);

//	data [0] = (uint8_t) ((int16_t)DR16.Get_LX_Norm()/10);
//	data [1] = (uint8_t) ((int16_t)DR16.Get_LY_Norm()/10);
//	data [2] = (uint8_t) ((int16_t)DR16.Get_RX_Norm()/10);
//	data [3] = (uint8_t) ((int16_t)DR16.Get_RY_Norm()/10);
//	data [4] = (uint8_t) ((int16_t)DR16.Get_DW_Norm()/10);
//	data [5] = (uint8_t) ( DR16.Get_S1_L()<<4| DR16.Get_S2_R());//[7:6]-s1,[5:4]-s2,[3:0]-LX
//data [6] =0;
//data [7] =0;
uint16_t LX,LY,RX,RY,DW;
	RX = DR16.Get_RX_Norm()+660;
	RY = DR16.Get_RY_Norm()+660;
	DW = DR16.Get_DW_Norm()+660;
	dr_data.pack.RX = RX;
	dr_data.pack.RY = RY;
	dr_data.pack.DW = DW;
	dr_data.pack.SL = DR16.Get_S1_L();
	dr_data.pack.SR = DR16.Get_S2_R();
	dr_data.pack.blue_outpost_hp = Referee.GameRobotHP.blue_outpost_HP;			//��ɫ��ǰ��վѪ��
	dr_data.pack.red_outpost_hp = Referee.GameRobotHP.red_outpost_HP;  			//��ɫ��ǰ��վѪ��
	memcpy(data, &dr_data.data, sizeof(dr_data.data));
	
	
	CANx_SendData(&hcan1,0x338,data,8);
}


 /**
  * @brief      д�����̨������յ�����
  * @param[in]  can_rx_data
  * @retval     None
  */
 uint8_t test_vision[2];
 void Robot_classdef::WriteMsgFromGimbal(uint8_t id,uint8_t can_rx_data[])
 {
     //--- 0x86
     //--- ����Ŀ���ٶ�
	 switch(id)
	 {
		 case 0x86:
     Chassis.Target_Vx = ((int16_t)((can_rx_data[0]<<8)|can_rx_data[1]));
     Chassis.Target_Vy = ((int16_t)((can_rx_data[2]<<8)|can_rx_data[3]));
		 break;
			 
	 }
 }






/**
 * @brief  	    �������ݴ��� ����UI��ʾ
 * @param[in]	bullet_speed
 * @retval 	    None
 */
void Robot_classdef::ShootMsg_Process(float bullet_speed)
{
    
}

/**
 * @brief  	    ����ǹ�����ݼ�����Ƶ
 * @param[in]	None
 * @retval 	    ShootFreq
 */
uint8_t Robot_classdef::ShootFreq_Calc()
{
    if((Get_CoolingLimit() - Get_GunHeat())/Onebullet_heat > 1/* CanShoot_Limit */) //--- ���㵱ǰǹ��ʣ������
    {
        if((Get_CoolingLimit() - Get_GunHeat())/Onebullet_heat == 2) //--- �պ�������ֵ
        {
            Shoot_Freq = Get_CoolingRate()/Onebullet_heat;
        }
        else if(Get_CoolingLimit()-Get_GunHeat() <= Onebullet_heat*4) //--- ʣ����ĳ����ֵ֮�� 4���ӵ�
        {
            Shoot_Freq = ((Get_CoolingLimit()-Get_GunHeat()) / (Onebullet_heat*2)) * (20 - Get_CoolingRate()/Onebullet_heat) + Get_CoolingRate()/Onebullet_heat;
        }
        else
        {
            Shoot_Freq = 20; //---δ�ﵽ�����������䣬��Ƶ���
        }
    }
    else //--- ������պ÷�����ֵ���ӵ�
    {
        Shoot_Freq = 0;
    }

    return Shoot_Freq;
}

/**
 * @brief  	    ��ȡ�����������Ϣ
 * @param[in]	mode
 * @retval 	    None
 */
uint8_t Robot_classdef::Get_ID()
{
    return Referee.GameRobotState.robot_id;
}
RobotState_e Robot_classdef::Get_State()
{
    return State;
}
DR16Status_Typedef Robot_classdef::Get_CtrlSource()
{
    return Ctrl_Source;
}
CHAS_CtrlMode_e Robot_classdef::Get_ChassisMode()
{
    return Chassis_Mode;
}
Gimbal_CtrlMode_e Robot_classdef::Get_GimbalMode()
{
    return Gimbal_Mode;
}
// Attack_CtrlMode_e Robot_classdef::Get_AttackMode()
// {
//     return Attack_Mode;
// }
// Fric_CtrlMode_e Robot_classdef::Get_FricMode()
// {
//     return Fric_Mode;
// }
// VisionMode_e Robot_classdef::Get_VisionMode()
// {
//     return Vision_Mode;
// }
uint8_t Robot_classdef::Get_VisionDepth()
{
    return Vision_Depth;
}
CHAS_CtrlMode_e Robot_classdef::Get_ChassisPreMode()
{
    return Chassis_PreMode;
}

uint16_t Robot_classdef::Get_CoolingLimit()
{
    return Referee.GameRobotState.shooter_barrel_heat_limit;
}

uint16_t Robot_classdef::Get_CoolingRate()
{
    return Referee.GameRobotState.shooter_barrel_cooling_value;
}

uint16_t Robot_classdef::Get_GunHeat()
{
    return Referee.PowerHeatData.shooter_id1_17mm_heat;
}




