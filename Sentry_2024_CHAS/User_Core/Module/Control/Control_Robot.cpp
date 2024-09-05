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
    //--- 初始化
	// Referee.Init(&huart3, Get_RefereeTime);	
  
}

/**
 * @brief  	  机器人总控制
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

//    SupCap.Control();   //--- 超电控制 哨兵不用
    
    Chassis.Control();  //--- 底盘控制

//    ShootFreq_Calc();   //--- 计算发射射频


    // ToDo...
    // if(Write_Msg[Robot_Reset] == true && DevicesMonitor.Get_State(COMMU_0X340_MONITOR) != Off_line)
    // {
    //     reset_cnt++;
    //     if(reset_cnt>=1000) //--- 收到重启信号持续2s重启
    //     {
    //         Reset();
    //     }
    // }
    // else
    // {
    //     reset_cnt = 0;
    // }

    //--- 裁判系统信息发送至云台主控
	DR16_Send(DR16_Date);
//	if(m==300)
//	{
	RefereeMsg_Send(RefereeData);
//		m=0;
//	}
    DevicesMonitor.Get_FPS(&Infantry.TIME, &Infantry.FPS);
}

/**
 * @brief  	  机器人使能
 * @param[in]	None
 * @retval 	  None
 */
void Robot_classdef::Enable()
{
    if (State == Robot_Disability) //--- 发生状态跳变
    {
        State = Robot_Activating;
    }
    if (State != Robot_Initializing)
    {
        State = Robot_Activating;
    }
}

/**
 * @brief  	  机器人失能,状态复位
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
 * @brief  	    机器人重启(软重启)
 * @param[in]	None
 * @retval 	    None
 */
void Robot_classdef::Reset()
{   
    //--- 芯片复位
    __set_FAULTMASK(1);     //--- 关闭所有中断
    HAL_NVIC_SystemReset(); //--- 复位
}

/**
 * @brief  	  机器人控制源转换
 * @param[in]	mode
 * @retval 	  None
 */
void Robot_classdef::CtrlSource_Switch(DR16Status_Typedef mode)
{
    //--- 发生模式跳变
    if (Get_CtrlSource() != mode)
    {
        // M6020 Yaw Reset
        // Gimbal.Motor[Yaw].Reset();
    }
    Ctrl_Source = mode;
}

/**
 * @brief  	  设置底盘运作模式
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
 * @brief  	  设置云台运作模式
 * @param[in]	mode
 * @retval 	  None
 */
void Robot_classdef::Set_GimbalMode(Gimbal_CtrlMode_e mode)
{
    Gimbal_Mode=Gimbal.Mode = mode;
}



/**
 * @brief      向云台主板发送裁判系统数据
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

//    data[0] = temp[0]; //--- 枪管射速
//    data[1] = temp[1];
//    data[2] = temp[2];
//    data[3] = temp[3];

//    data[4] = 0;    //--- 机器人ID&发射电源接口状态
//    data[4] |= Get_ID();
//    data[4] |= Referee.GameRobotState.mains_power_shooter_output == ON?1<<7:0;

//    data[5] = Referee.GameRobotState.shooter_id1_17mm_speed_limit;  //--- 子弹初速度限制
//    data[6] = Shoot_Freq;   //--- 子弹射频
//		//换成
//    data[7] = Chassis.Get_PowerLimit(); //--- 底盘功率限制
	//枪管热量限制
//	data[0] = Referee.GameRobotState.shooter_barrel_heat_limit;//低八位
//	data[1] = Referee.GameRobotState.shooter_barrel_heat_limit>>8;//高八位
//	
//	//枪管1,2现在热量
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

	if(Referee.GameRobotState.robot_id>=100)//1-7红<----->101-107蓝
	{
			//自己蓝发1  -----  blue_outpost_HP
		robot_state.pack.team_color = 1;//判断自己的颜色
		if(Referee.GameRobotHP.blue_outpost_HP == 0)//判断自己前哨站是否存在
		{
		robot_state.pack.my_output  = 0;
		}
		else
		{
		robot_state.pack.my_output  = 1;
		}
		if(Referee.GameRobotHP.red_outpost_HP == 0)//判断对方前哨站是否存在
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
			//自己红发0  -----   red_outpost_HP
		robot_state.pack.team_color   = 0 ;//判断自己的颜色
		if(Referee.GameRobotHP.red_outpost_HP == 0)//判断自己前哨站是否存在
		{
		robot_state.pack.my_output  = 0;
		}
		else
		{
		robot_state.pack.my_output  = 1;
		}
		if(Referee.GameRobotHP.blue_outpost_HP == 0)//判断对方前哨站是否存在
		{
		robot_state.pack.enemy_output  = 0;
		}
		else
		{
		robot_state.pack.enemy_output  = 1;
		}
	}
		robot_state.pack.shooter_barrel_heat_limit = Referee.GameRobotState.shooter_barrel_heat_limit;//枪口热量上线
		robot_state.pack.shooter_17mm_1_barrel_heat = Referee.PowerHeatData.shooter_id1_17mm_heat;		//枪口1当前热量
		robot_state.pack.shooter_17mm_2_barrel_heat = Referee.PowerHeatData.shooter_id2_17mm_heat;		//枪口2当前热量
		robot_state.pack.game_progress = Referee.GameState.game_progress;															//比赛进程
		robot_state.pack.stage_remain_time = Referee.GameState.stage_remain_time;											//当前阶段剩余时间
		robot_state.pack.sentry_hp = Referee.GameRobotState.current_HP;																//自身剩余血量
		robot_state.pack.bullet_remain = Referee.BulletRemaining.bullet_remaining_num_17mm;						//剩余弹量
		memcpy(data,&robot_state.data,sizeof(robot_state.data));
    CANx_SendData(&hcan1,0x341,data,8);

}
/**
 * @brief      向云台主板发送DR16数据
 * @param[in]  can_rx_data
 * @retval     None
 */
// float bulletspeed;


void Robot_classdef::DR16_Send(uint8_t *data)
{//0000 0000

//	int8_t LX,LY,RX,RY,DW;//个位数据单独提取出来
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
	dr_data.pack.blue_outpost_hp = Referee.GameRobotHP.blue_outpost_HP;			//红色方前哨站血量
	dr_data.pack.red_outpost_hp = Referee.GameRobotHP.red_outpost_HP;  			//蓝色方前哨站血量
	memcpy(data, &dr_data.data, sizeof(dr_data.data));
	
	
	CANx_SendData(&hcan1,0x338,data,8);
}


 /**
  * @brief      写入从云台主板接收的数据
  * @param[in]  can_rx_data
  * @retval     None
  */
 uint8_t test_vision[2];
 void Robot_classdef::WriteMsgFromGimbal(uint8_t id,uint8_t can_rx_data[])
 {
     //--- 0x86
     //--- 底盘目标速度
	 switch(id)
	 {
		 case 0x86:
     Chassis.Target_Vx = ((int16_t)((can_rx_data[0]<<8)|can_rx_data[1]));
     Chassis.Target_Vy = ((int16_t)((can_rx_data[2]<<8)|can_rx_data[3]));
		 break;
			 
	 }
 }






/**
 * @brief  	    发射数据处理 用于UI显示
 * @param[in]	bullet_speed
 * @retval 	    None
 */
void Robot_classdef::ShootMsg_Process(float bullet_speed)
{
    
}

/**
 * @brief  	    根据枪管数据计算射频
 * @param[in]	None
 * @retval 	    ShootFreq
 */
uint8_t Robot_classdef::ShootFreq_Calc()
{
    if((Get_CoolingLimit() - Get_GunHeat())/Onebullet_heat > 1/* CanShoot_Limit */) //--- 计算当前枪管剩余热量
    {
        if((Get_CoolingLimit() - Get_GunHeat())/Onebullet_heat == 2) //--- 刚好满足阈值
        {
            Shoot_Freq = Get_CoolingRate()/Onebullet_heat;
        }
        else if(Get_CoolingLimit()-Get_GunHeat() <= Onebullet_heat*4) //--- 剩余在某个阈值之内 4颗子弹
        {
            Shoot_Freq = ((Get_CoolingLimit()-Get_GunHeat()) / (Onebullet_heat*2)) * (20 - Get_CoolingRate()/Onebullet_heat) + Get_CoolingRate()/Onebullet_heat;
        }
        else
        {
            Shoot_Freq = 20; //---未达到热量警告区间，满频输出
        }
    }
    else //--- 不满足刚好发射阈值数子弹
    {
        Shoot_Freq = 0;
    }

    return Shoot_Freq;
}

/**
 * @brief  	    获取机器人相关信息
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




