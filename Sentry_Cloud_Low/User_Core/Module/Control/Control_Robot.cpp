/* Includes ------------------------------------------------------------------*/
#include "Control_Robot.h"
#include "System_DataPool.h"
#include "stm32f4xx_hal.h"
#include "kalman_hpz.h"
#include "usbd_cdc_if.h"

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
void Robot_classdef::Control()
{
		Chassis.Radar_Control();
    Gimbal.Control();
    Shoot.Control();

//		Shoot.Send_Motor[0].Out = 0;
		
		//YAW
//		Gimbal.Send_Motor[0].Out = 0;
//		Gimbal.Send_Motor[1].Out = 0;
//		Gimbal.Send_Motor[2].Out = 0;//2��3����ò���
//		Gimbal.Send_Motor[3].Out = 0;//���Ը�ֵ0
//		MotorMsgSend(&hcan1, Gimbal.Send_Motor);//Gimbal[0]���͸���̨Yaw�ᣬGimbal[1]�ǲ���
		MotorMsgSend(&hcan2, Shoot.Send_Motor);/*<! ��FL-FR-Reload-Pit*/
//		MotorMsgSend(&hcan2, Shoot.Card_Motor);
//		CANx_SendData_Current(&hcan1,0x1FE,Gimbal.Send_Motor[0].Out,8);
    //--- ������������������
    SendMsgtoCHAS();
		Send2C_Board();
	
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

    CTRL_DR16.ExptData_Reset();
    Set_ChassisMode(CHAS_DisableMode);
    Set_GimbalMode(Gimbal_DisableMode);
    Set_AttackMode(Attack_Disable);
    Set_VisionMode(Vision_Forcast);

    Gimbal.Windmill_Offset[0] = 0.0f;
    Gimbal.Windmill_Offset[1] = 0.0f;
}

/**
 * @brief  	    ����������(������)
 * @param[in]	None
 * @retval 	    None
 */
void Robot_classdef::Reset()
{   
    //--- оƬ��λ
    __set_FAULTMASK(1);    //�ر������ж�
    HAL_NVIC_SystemReset();//��λ
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
        Gimbal.Motor[Yaw].Reset();
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
        Gimbal.Motor[Yaw].Reset();
    }
		
		if (Get_ChassisMode() == CHAS_SentryPCMode && mode != CHAS_SentryPCMode)
    {
        // Reset M6020 Yaw
        Gimbal.Motor[Yaw].Reset();
    }

    Chassis_Mode = mode;
}

/**
 * @brief  	  ������̨����ģʽ
 * @param[in]	mode
 * @retval 	  None
 */
void Robot_classdef::Set_GimbalMode(Gimbal_CtrlMode_e mode)
{
    if(mode == Gimbal_PCMode && DevicesMonitor.Get_State(VISION_MONITOR) == Off_line)
    {
        Gimbal_Mode = Gimbal_NormalMode;
        PC_State = Off_line;
        return;
    }
    PC_State = On_line;
    Gimbal_Mode = mode;
}

/**
 * @brief  	    ���÷�������ģʽ
 * @param[in]	mode
 * @retval 	    None
 */
void Robot_classdef::Set_AttackMode(Attack_CtrlMode_e mode)
{
    if(Get_AttackMode() == Attack_Auto && mode != Attack_Auto)
    {
        //--- ���Զ����ģʽ��ȡ��ʱ�������Զ����������
        Shoot.Set_ReloadNum(0);
    }
		else if(Attack_Mode != mode)
		{
			Vision.aim_flag = 0;
		}

    Attack_Mode = mode;

    switch(Get_AttackMode())
    {
    case Attack_30:
		Fric_Mode = Fric_30m_s;
		break;
	case Attack_18:
		Fric_Mode = Fric_18m_s;
		break;
    case Attack_22:
        Fric_Mode = Fric_22m_s;
        break;
	case Attack_15:
		Fric_Mode = Fric_15m_s;
		break;

	case Attack_Auto:
		switch (Shoot.Get_SpeedLimit())
		{
		case 15:
			Fric_Mode = Fric_15m_s; // --- 15m/s
			break;
		case 18:
			Fric_Mode = Fric_18m_s; // --- 18m/s
			break;
        case 22:
        case 25:
            Fric_Mode = Fric_22m_s; // --- 22m/s
            break;
		case 30:
        case 75:
			Fric_Mode = Fric_30m_s; // --- 30m/s
			break;

		default:
			Fric_Mode = Fric_15m_s; // --- 15m/s
			break;
		}
		break;

	case Attack_Unload:
		Fric_Mode = Fric_Unload;
		break;

	case Attack_Disable:
		Fric_Mode = Fric_Disable;
		Shoot.Rage_Mode = false;
		break;
	
	case Attack_Zero:
		Fric_Mode = Fric_Zero;
		Shoot.Rage_Mode = false;
	break;

	default:
		Fric_Mode = Fric_Disable;
		Shoot.Rage_Mode = false;
		break;
	
	
    }
}

/**
 * @brief  	    �����Ӿ�����ģʽ
 * @param[in]	mode
 * @retval 	    None
 */
void Robot_classdef::Set_VisionMode(VisionMode_e mode)
{
    PC_State = (DevicesMonitor.Get_State(VISION_MONITOR) == Off_line?Off_line:On_line);

    Vision_Mode = mode;

    switch(mode)
    {
    case Vision_Default:
    case Vision_Forcast:
    case Vision_Top:
        if(Infantry.Get_ChassisMode() == CHAS_LockMode && Infantry.Get_CtrlSource() == CTRL_PC)
        {
            //--- PCģʽ�˳����ģʽʱ�л�����
            Infantry.Set_ChassisMode(CHAS_FollowMode);
        }
        break;
    case Vision_BIGWindmill:
        //--- ������̲���
        Infantry.Set_ChassisMode(CHAS_LockMode);
        break;
    default:
        break;
    }
}
/**
 * @brief  	    ���ͨ�� �������� 
 * @param[in]  C 2 C
 * @note        0x346
 * @retval 	    None
 */
int u;
void Robot_classdef::Send2C_Board()
{
//    uint8_t *temp;
//		float Out = Gimbal.Motor[0].Out;
//		temp = (uint8_t *)(&Out);
//    Send2A_Msg[0] = temp[0];
//    Send2A_Msg[1] = temp[1];
//    Send2A_Msg[2] = temp[2];
//    Send2A_Msg[3] = temp[3];
//    Send2A_Msg[4] =	0;
//    Send2A_Msg[5] =	0;
//    Send2A_Msg[6] =	0;
//    Send2A_Msg[7] =	0;
float Out = Gimbal.Motor[0].Out; // ��ȡҪ���͵ĸ�����ֵ
    memcpy(&Send2C_Msg[0], &Out, sizeof(float)); // ��������������������Ϣ��������
    // ���ʣ��Ŀռ�
    for (int i = 4; i < 8; ++i) 
		{
        Send2C_Msg[i] = 0;
    }    
		u=CANx_SendData(&hcan1, 0x666, Send2C_Msg, 8);
}

/**
 * @brief  	    ���ͨ�� ��������
 * @param[in]	mode
 * @note        0x340
 * @retval 	    None
 */

void Robot_classdef::SendMsgtoCHAS()
{
    static uint8_t test_cnt = 0;
    test_cnt++;

    //--- 0x86
	

    //--- �ϲ����ؼ�����ĵ����ٶ�
    SendData[0] = Chassis.Get_TargetVx() >> 8;
    SendData[1] = Chassis.Get_TargetVx();

    SendData[2] = Chassis.Get_TargetVy() >> 8;
    SendData[3] = Chassis.Get_TargetVy();

    SendData[4] = 0;
    SendData[5] = 0;

    SendData[6] = 0;
		SendData[7] = 0;

//    //--- ����ģʽ(bit 0-3)
//    SendData[6] |= Infantry.Get_ChassisMode()&0x0F;
//    //--- �Ӿ�ģʽ(bit 4-6)
//    SendData[6] |= (Infantry.Get_VisionMode()&0x07)<<4;

//    if(test_cnt == 1)
//    {
//        //--- Data[6] bit7 ��ʶ�� 0:UI���豸��־λ
//        SendData[6] |= 0;
//    
//        //--- ģ����״̬
//        SendData[7] |= (DevicesMonitor.Get_State(VISION_MONITOR) == Off_line?1<<0:0);
//        SendData[7] |= ((DevicesMonitor.Get_State(FRIC_L_MONITOR)||DevicesMonitor.Get_State(FRIC_R_MONITOR)||DevicesMonitor.Get_State(RELOAD_MONITOR)) == Off_line?1<<1:0);

//        //--- ��������ģʽ
//        SendData[7] |= Chassis.Uphill_Mode == true?1<<2:0;
//    
//        //--- ������������
//        SendData[7] |= ResetFlag==true?1<<3:0;
//    
//        //--- ���ݿ���
//        SendData[7] |= Chassis.Cap_switch==ON?1<<4:0;
//        //--- Ħ����״̬
//        SendData[7] |= Infantry.Get_AttackMode()==Attack_Disable?0:1<<5;
//        //--- ���ֿ���
//        SendData[7] |= Shoot.Mag_Switch==OFF?0:1<<6;
//        //--- UIˢ��
//        SendData[7] |= Reflash_UI<<7;
//    }
//    else if((test_cnt%=2)==0)
//    {
//        //--- Data[6] bit7 ��ʶ�� 1:�Ӿ����
//        SendData[6] |= 1<<7;

//        SendData[7] = (uint8_t)(Vision.Get_DepthOffset()/100);
//    }

    CANx_SendData(&hcan1, 0x86, SendData, 8);

}

/**
 * @brief      д��ӵ������ؽ��յĲ���ϵͳ����
 * @param[in]  id, can_rx_data
 * @retval     None
 */
		uint8_t count1;

void Robot_classdef::RefereeMsg_Write(uint16_t id, uint8_t can_rx_data[])
{
    switch(id)
    {
    case 0x341:
			memcpy(robot_state.data, can_rx_data, sizeof(robot_state.data));
			shooter_barrel_heat_limit = robot_state.pack.shooter_barrel_heat_limit;								//ǹ����������
			shooter_id1_17mm_heat = robot_state.pack.shooter_17mm_1_barrel_heat;									//ǹ��1��ǰ����
			Vision.Radar_Send_Msg.Pack.team_color=TeamColor = robot_state.pack.team_color;																							//�ж��Լ�����ɫ
			Vision.Radar_Send_Msg.Pack.bullet_remain = robot_state.pack.bullet_remain;            //ʣ�൯��
			Vision.Radar_Send_Msg.Pack.game_progress = robot_state.pack.game_progress;            //��������
			Vision.Radar_Send_Msg.Pack.game_progress_remain = robot_state.pack.stage_remain_time; //��ǰ�׶�ʣ��ʱ��
			Vision.Radar_Send_Msg.Pack.sentry_hp = robot_state.pack.sentry_hp;                    //����ʣ��Ѫ��
			break;
				
			case 0x338:
			memcpy(Rev_DR16.Data, can_rx_data, sizeof(Rev_DR16.Data));
			DR16.RX_Norm = float(Rev_DR16.pack.RX-660);
		  DR16.RY_Norm = float(Rev_DR16.pack.RY-660);
			DR16.DW_Norm = float(Rev_DR16.pack.DW-660);
			DR16.DataPack.S1_L  = (SW_Status_Typedef)Rev_DR16.pack.SL    ;
		  DR16.DataPack.S2_R  = (SW_Status_Typedef)Rev_DR16.pack.SR    ;			
			Vision.Radar_Send_Msg.Pack.red_outpost_hp = Rev_DR16.pack.red_outpost_hp;							//��ɫ��ǰ��վѪ��
			Vision.Radar_Send_Msg.Pack.blue_outpost_hp = Rev_DR16.pack.blue_outpost_hp;						//��ɫ��ǰ��վѪ��
			break;

    default:
        break;
    }
}


/**
 * @brief  	    ��ȡ�����������Ϣ
 * @param[in]	None
 * @retval 	    robot data
 */
uint8_t Robot_classdef::Get_TeamColor()
{
    return TeamColor;
		//�Լ������Է��졪���Է��췢0
		//�Լ��죬�Է��������Է�����1
//	}
}
uint8_t Robot_classdef::Get_ID()
{
    // return Referee.GameRobotState.robot_id;
		return ID;
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
CHAS_CtrlMode_e Robot_classdef::Get_ChassisPreMode()
{
    return Chassis_PreMode;
}
Gimbal_CtrlMode_e Robot_classdef::Get_GimbalMode()
{
    return Gimbal_Mode;
}
Attack_CtrlMode_e Robot_classdef::Get_AttackMode()
{
    return Attack_Mode;
}
Fric_CtrlMode_e Robot_classdef::Get_FricMode()
{
    return Fric_Mode;
}
VisionMode_e Robot_classdef::Get_VisionMode()
{
    return Vision_Mode;
}
uint16_t Robot_classdef::Get_FPS()
{
    return FPS;
}

