/**
 ------------------------------------------------------------------------------
 * @file    Control_Gimbal.cpp
 * @author  Shake
 * @brief   云台控制
 * @version V1.0
 * @date    2021-10
 * @copyright Copyright (c) 2021
 ------------------------------------------------------------------------------
 */


/* Includes ------------------------------------------------------------------*/
#include "Control_Gimbal.h"
#include "Control_Robot.h"
#include "System_DataPool.h"
#include "Robot_Config.h"
#include "arm_math.h"
#include "kalman_hpz.h"
#include "usbd_cdc_if.h"
#include "SolveTrajectory.h"


//--- 调参模式
#define Gimbal_Debug  0

float YawVision_Ki = 5.5f;

extKalman_t Kalman_PitSpeed;
extKalman_t Kalman_PitOut;


/**
 * @brief      初始化
 * @param[in]  None
 * @retval     None
 */
Gimbal_classdef::Gimbal_classdef()
{
    //--- kp,ki,kd,ki_max,out_max,dt

#if ROBOT_ID == INFANTRY_2022_SWERVE_1
    /* Yaw 电机角度 */
    YawPID[Mech][PID_Outer].SetPIDParam(10.0f, 0.0f, 0.0f, 10000, 15000, 0.002f);
    YawPID[Mech][PID_Outer].I_SeparThresh = 9000;
    YawPID[Mech][PID_Inner].SetPIDParam(10.0f, 50.0f, 0.0f, 10000, 1000, 0.002f);
    YawPID[Mech][PID_Inner].I_SeparThresh = 9000;

    /* Yaw A板IMU */
    YawPID[A_IMU][PID_Outer].SetPIDParam(-15.0f, 0.0f, 0.0f, 10000, 15000, 0.002f);
    YawPID[A_IMU][PID_Outer].I_SeparThresh = 1000;
    YawPID[A_IMU][PID_Inner].SetPIDParam(16.0f, 100.0f, 0.0f, 10000, GM6020_MAX_OUT, 0.002f);
    YawPID[A_IMU][PID_Inner].I_SeparThresh = 5000;

    /* Yaw C板IMU */
    YawPID[C_IMU][PID_Outer].SetPIDParam(1.1f, 0.0f, 0.0f, 2000, 15000, 0.002f);// V 1.0
    YawPID[C_IMU][PID_Outer].I_SeparThresh = 500;
    YawPID[C_IMU][PID_Inner].SetPIDParam(370.0f, 600.0f, 0.0f, 12000, GM6020_MAX_OUT, 0.002f);//380 580  12000
    YawPID[C_IMU][PID_Inner].I_SeparThresh = 300;// 300

    /* Yaw 自瞄C板IMU */
    YawPID[V_CIMU][PID_Outer].SetPIDParam(1.1f, 0.0f, 0.0f, 100, 2000, 0.002f);  //1.1  1.0   //fu 0.8
    YawPID[V_CIMU][PID_Outer].I_SeparThresh = 70;
    YawPID[V_CIMU][PID_Inner].SetPIDParam(370.0f, 600.0f, 0.0f, 12000, GM6020_MAX_OUT, 0.002f);//380 580  12000
    YawPID[V_CIMU][PID_Inner].I_SeparThresh = 300;// 300

    /* Pit 电机角度 */
    PitPID[Mech][PID_Outer].SetPIDParam(0.75f, 0.0f, 0.0f, 10000, 15000, 0.002f);//
    PitPID[Mech][PID_Outer].I_SeparThresh = 1000;
    PitPID[Mech][PID_Inner].SetPIDParam(300.0f, 500.0f, 0.0f, 10000, GM6020_MAX_OUT, 0.002f);//350 500
    PitPID[Mech][PID_Inner].I_SeparThresh = 400;

    /* Pit 自瞄电机角度 */
    PitPID[V_Mech][PID_Outer].SetPIDParam(0.66f, 0.0f, 0.0f, 10000, 15000, 0.002f);  //fu 0.6
    PitPID[V_Mech][PID_Outer].I_SeparThresh = 1000;
    PitPID[V_Mech][PID_Inner].SetPIDParam(280.0f, 400.0f, 0.0f, 10000, GM6020_MAX_OUT, 0.002f);//300 500
    PitPID[V_Mech][PID_Inner].I_SeparThresh = 400;

    /* Pit C板IMU */
    PitPID[C_IMU][PID_Outer].SetPIDParam(1.3f, 0.0f, 0.0f, 2000, 15000, 0.002f);//0.75  
    PitPID[C_IMU][PID_Outer].I_SeparThresh = 500;
    PitPID[C_IMU][PID_Inner].SetPIDParam(300.0f, 500.0f, 0.0f, 10000, GM6020_MAX_OUT, 0.002f);//500 600
    PitPID[C_IMU][PID_Inner].I_SeparThresh = 100;
#elif ROBOT_ID == INFANTRY_2022_SWERVE_2
    /* Yaw 电机角度 */
    YawPID[Mech][PID_Outer].SetPIDParam(10.0f, 0.0f, 0.0f, 10000, 15000, 0.002f);
    YawPID[Mech][PID_Outer].I_SeparThresh = 9000;
    YawPID[Mech][PID_Inner].SetPIDParam(10.0f, 50.0f, 0.0f, 10000, 1000, 0.002f);
    YawPID[Mech][PID_Inner].I_SeparThresh = 9000;

    /* Yaw C板IMU */
    // YawPID[C_IMU][PID_Outer].SetPIDParam(1.2f, 0.0f, 0.0f, 2000, 15000, 0.002f);//
    // YawPID[C_IMU][PID_Outer].I_SeparThresh = 500;
    // YawPID[C_IMU][PID_Inner].SetPIDParam(310.0f, 600.0f, 0.0f, 13000, GM6020_MAX_OUT, 0.002f);//
    // YawPID[C_IMU][PID_Inner].I_SeparThresh = 300;// 300

    YawPID[C_IMU][PID_Outer].SetPIDParam(0.65f, 0.0f, 0.0f, 100, 2500, 0.002f);//0.78
    YawPID[C_IMU][PID_Outer].I_SeparThresh = 80;
    YawPID[C_IMU][PID_Inner].SetPIDParam(500.0f, 600.0f, 0.0f, 10000, GM6020_MAX_OUT, 0.002f);//
    YawPID[C_IMU][PID_Inner].I_SeparThresh = 300;// 300

    /* Yaw 自瞄C板IMU */
    YawPID[V_CIMU][PID_Outer].SetPIDParam(0.886f, 0.0f, 0.0f, 100, 2000, 0.002f); 
    YawPID[V_CIMU][PID_Outer].I_SeparThresh = 70;
    YawPID[V_CIMU][PID_Inner].SetPIDParam(1000.0f, 888.0f, 0.0f, 13000, GM6020_MAX_OUT, 0.002f);//380 580  12000
    YawPID[V_CIMU][PID_Inner].I_SeparThresh = 300;// 300

    /* Pit 电机角度 */
    PitPID[Mech][PID_Outer].SetPIDParam(0.68f, 0.0f, 0.0f, 10000, 15000, 0.002f);//
    PitPID[Mech][PID_Outer].I_SeparThresh = 1000;
    PitPID[Mech][PID_Inner].SetPIDParam(180.0f, 500.0f, 0.0f, 4000, GM6020_MAX_OUT, 0.002f);//350 500
    PitPID[Mech][PID_Inner].I_SeparThresh = 500;

    /* Pit 自瞄电机角度 */
    PitPID[V_Mech][PID_Outer].SetPIDParam(0.66f, 0.0f, 0.0f, 10000, 15000, 0.002f);  //fu 0.6
    PitPID[V_Mech][PID_Outer].I_SeparThresh = 1000;
    PitPID[V_Mech][PID_Inner].SetPIDParam(280.0f, 400.0f, 0.0f, 10000, GM6020_MAX_OUT, 0.002f);//300 500
    PitPID[V_Mech][PID_Inner].I_SeparThresh = 400;

    /* Pit C板IMU */
    PitPID[C_IMU][PID_Outer].SetPIDParam(1.1f, 0.0f, 0.0f, 2000, 15000, 0.002f);//0.75
    PitPID[C_IMU][PID_Outer].I_SeparThresh = 500;
    PitPID[C_IMU][PID_Inner].SetPIDParam(220.0f, 500.0f, 0.0f, 5000, GM6020_MAX_OUT, 0.002f);//500 600
    PitPID[C_IMU][PID_Inner].I_SeparThresh = 100;


    /* Pit 自瞄 C板IMU */
    PitPID[V_CIMU][PID_Outer].SetPIDParam(1.1f, 0.0f, 0.0f, 2000, 15000, 0.002f);
    PitPID[V_CIMU][PID_Outer].I_SeparThresh = 500;
    PitPID[V_CIMU][PID_Inner].SetPIDParam(220.0f, 500.0f, 0.0f, 8000, GM6020_MAX_OUT, 0.002f);//500 600
    PitPID[V_CIMU][PID_Inner].I_SeparThresh = 200;
#elif ROBOT_ID == SRNTRY_2023_OMNI
    /* Yaw 电机角度 */
    YawPID[Mech][PID_Outer].SetPIDParam(10.0f, 0.0f, 0.0f, 10000, 15000, 0.002f);
    YawPID[Mech][PID_Outer].I_SeparThresh = 9000;
    YawPID[Mech][PID_Inner].SetPIDParam(10.0f, 50.0f, 0.0f, 10000, 1000, 0.002f);
    YawPID[Mech][PID_Inner].I_SeparThresh = 9000;

    /* Yaw C板IMU */
    // YawPID[C_IMU][PID_Outer].SetPIDParam(1.2f, 0.0f, 0.0f, 2000, 15000, 0.002f);//
    // YawPID[C_IMU][PID_Outer].I_SeparThresh = 500;
    // YawPID[C_IMU][PID_Inner].SetPIDParam(310.0f, 600.0f, 0.0f, 13000, GM6020_MAX_OUT, 0.002f);//
    // YawPID[C_IMU][PID_Inner].I_SeparThresh = 300;// 300
		
		
#if Use_Currentloop==0
/*------------------------------------------------------------------------------------------------------------------------------*/

    YawPID[C_IMU][PID_Outer].SetPIDParam(0.55f, 0.0f, 0.0f, 100, 15000, 0.002f);//0.78
    YawPID[C_IMU][PID_Outer].I_SeparThresh = 80;
//		YawPID[C_IMU][PID_Outer].SetPidChange(0.019f,0.40f,0.030f,0.0f,0.0f,0.0f);
    YawPID[C_IMU][PID_Inner].SetPIDParam(500.0f, 570.0f, 0.0f, 10000, GM6020_MAX_OUT, 0.002f);//
//		YawPID[C_IMU][PID_Inner].SetPidChange(25.0f,25.0f,25.0f,10.0f,10.0f,10.0f);
    YawPID[C_IMU][PID_Inner].I_SeparThresh = 300;// 300
/*------------------------------------------------------------------------------------------------------------------------------*/
#elif Use_Currentloop==1
    YawPID[C_IMU][PID_Outer].SetPIDParam(0.55f, 0.0f, 0.0f, 100, 15000, 0.002f);//0.78
    YawPID[C_IMU][PID_Outer].I_SeparThresh = 80;
		YawPID[C_IMU][PID_Outer].SetPidChange(0.019f,0.40f,0.030f,0.0f,0.0f,0.0f);
    YawPID[C_IMU][PID_Inner].SetPIDParam(1150.0f, 570.0f, 0.0f, 10000, 7000, 0.002f);//
		YawPID[C_IMU][PID_Inner].I_SeparThresh = 300;// 300
		YawPID[C_IMU][PID_current].I_SeparThresh=200;
//		YawPID[C_IMU][PID_Inner].SetPidChange(25.0f,25.0f,25.0f,10.0f,10.0f,10.0f);
    YawPID[C_IMU][PID_current].SetPIDParam(100.0f, 50.0f, 0.0f, 10000, GM6020_MAX_OUT, 0.002f);//
		



#endif




    /* Yaw 自瞄C板IMU */
    YawPID[V_CIMU][PID_Outer].SetPIDParam(0.586f, 0.0f, 0.0f, 100, 2000, 0.002f); 
    YawPID[V_CIMU][PID_Outer].I_SeparThresh = 70;
//		YawPID[V_CIMU][PID_Outer].SetPidChange(0.054f,0.6400f,0.0050f,0.0f,0.0f,0.0f);

    YawPID[V_CIMU][PID_Inner].SetPIDParam(500.0f, 900.0f, 0.0f, 13000, GM6020_MAX_OUT, 0.002f);//380 580  12000
    YawPID[V_CIMU][PID_Inner].I_SeparThresh = 1000;// 300

    /* Pit 电机角度 */
    PitPID[Mech][PID_Outer].SetPIDParam(0.68f, 0.0f, 0.0f, 10000, 15000, 0.002f);//
    PitPID[Mech][PID_Outer].I_SeparThresh = 1000;
    PitPID[Mech][PID_Inner].SetPIDParam(180.0f, 500.0f, 0.0f, 4000, GM6020_MAX_OUT, 0.002f);//350 500
    PitPID[Mech][PID_Inner].I_SeparThresh = 500;

    /* Pit 自瞄电机角度 */
    PitPID[V_Mech][PID_Outer].SetPIDParam(0.66f, 0.0f, 0.0f, 10000, 15000, 0.002f);  //fu 0.6
    PitPID[V_Mech][PID_Outer].I_SeparThresh = 1000;
    PitPID[V_Mech][PID_Inner].SetPIDParam(280.0f, 400.0f, 0.0f, 10000, GM6020_MAX_OUT, 0.002f);//300 500
    PitPID[V_Mech][PID_Inner].I_SeparThresh = 400;

    /* Pit C板IMU */
    PitPID[C_IMU][PID_Outer].SetPIDParam(0.69f, 0.0f, 0.0f, 2000, 15000, 0.002f);//0.75
    PitPID[C_IMU][PID_Outer].I_SeparThresh = 500;
    PitPID[C_IMU][PID_Inner].SetPIDParam(400.0f, 150.0f, 0.0f, 5000, GM6020_MAX_OUT, 0.002f);//500 600
    PitPID[C_IMU][PID_Inner].I_SeparThresh = 100;


    /* Pit 自瞄 C板IMU */
    PitPID[V_CIMU][PID_Outer].SetPIDParam(0.59f, 0.0f, 0.0f, 2000, 15000, 0.002f);
    PitPID[V_CIMU][PID_Outer].I_SeparThresh = 500;
    PitPID[V_CIMU][PID_Inner].SetPIDParam(400.0f, 150.0f, 0.0f, 8000, GM6020_MAX_OUT, 0.002f);//500 600
    PitPID[V_CIMU][PID_Inner].I_SeparThresh = 200;
#endif

    /*--- Angle Init -------------------------------------------------------------------------*/
    Target.Mech_Yaw = Motor[Yaw].get_totalencoder();
    Target.Mech_Pit = Motor[Pit].get_totalencoder();
    // Target.AIMU_Yaw = 8888 * ENCODER_ANGLE_RATIO;//
    Target.CIMU_Yaw = Get_IMUTotalAngle(Yaw) * ENCODER_ANGLE_RATIO;
    Target.CIMU_Pit = Get_IMUAngle(Rol) * ENCODER_ANGLE_RATIO;

    //--- Yaw轴遥控控制输出系数
    YawRCCtrl_Coe = 0.0009f;

    //--- 设置云台初始模式
    Set_YawCtrlMode(C_IMU);
#if ROBOT_ID == INFANTRY_2022_SWERVE_1
    Set_PitCtrlMode(Mech);
#elif ROBOT_ID == INFANTRY_2022_SWERVE_2
    Set_PitCtrlMode(C_IMU);
    KalmanCreate(&Kalman_PitOut, 1, 80);
#elif ROBOT_ID == SRNTRY_2023_OMNI
    Set_PitCtrlMode(C_IMU);//设置哨兵的陀螺仪为C_IMU全局
    KalmanCreate(&Kalman_PitOut, 1, 80);
#endif
    KalmanCreate(&Kalman_PitSpeed, 1, 80);
}


/**
 * @brief      云台总控函数
 * @param[in]  None
 * @retval     None
 */
uint8_t zero_init = false;
void Gimbal_classdef::Control()
{
    //--- IMU数据更新
#if USE_IMU_EKF
    IMU_Update(QEKF_INS.angle, QEKF_INS.Gyro);
    init_cnt = 500;
#else
    IMU_Update(INS_angle, INS_gyro);
    init_cnt = 1300;
#endif

    //--- 等待IMU初始化完毕
    if(init_mode)
    {
        if(Get_IMUAngle(Yaw) != 0 || Get_IMUAngle(Pit) != 0 || Get_IMUAngle(Rol) != 0)
        {
            init_cnt++;
            if(--init_cnt != 0) //--- 等待IMU稳定 800*2ms数据稳定时间+500*2ms误差采样时间
            {
                Set_InitAngle();
                init_mode = false;
            }
        }
        return;
    }

#if ROBOT_ID == INFANTRY_2022_SWERVE_1 
    //--- 数据处理
    //--- 零点在Pit限位区间的处理
    if(zero_init == false)
    {
        if(Motor[Pit].encoder_offset > 7910)
        {
            Motor[Pit].encoder_offset = 8191;
        }
        else
        {
            Motor[Pit].encoder_offset = 0;
        }
        zero_init = true;
    }
#else
    Motor[Pit].encoder_offset = 0;
#endif

    if(Infantry.Get_GimbalMode() == Gimbal_DisableMode || Infantry.Get_State() != Robot_Activating || DevicesMonitor.Get_State(DR16_MONITOR) == Off_line)
    {
        //set init angle...
        Set_InitAngle();
        Motor_Yaw[Yaw_Up].Out=0;
        Motor[Yaw].Out = 0;
				Shoot.Send_Motor[SendID_Pit].Out = Motor[Pit].Out = 0;
			
        return;
    }
//		if(Infantry.Get_GimbalMode() ==Gimbal_LockMode )
//		{
//			CTRL_DR16.Expt.Target_Yaw=Get_IMUTotalAngle(Yaw) * ENCODER_ANGLE_RATIO;
//			CTRL_DR16.Expt.Target_Pit=Get_IMUAngle(Rol) * ENCODER_ANGLE_RATIO;
//		}

    //--- 目标角度更新
    TargetAngle_Update(CTRL_DR16.Get_ExptYaw(), CTRL_DR16.Get_ExptPit());

    //--- PID计算
    Motor_PIDCalc();

    //--- 发送电流值
#if ROBOT_ID == INFANTRY_2022_SWERVE_1 || INFANTRY_2022_SWERVE_2 || INFANTRY_2022_SWERVE_3
//    MotorMsgSend(&hcan1, Motor);
#endif

}


/**
 * @brief      获取计算的目标角度
 * @param[in]  type
 * @retval     The point of Target Angle
 */
float *Gimbal_classdef::Get_TargetAngle(Gimbal_type_e type)
{
    static float *return_angle[2];

    switch(type)
    {
    case Yaw:   //--- Yaw ------------------------
        switch(YawMode)
        {
        case Mech: return_angle[Yaw] = &Target.Mech_Yaw;
            break;
        case A_IMU: return_angle[Yaw] = &Target.AIMU_Yaw;
            break;
        case C_IMU: return_angle[Yaw] = &Target.CIMU_Yaw;
            break;
        }
        break;

    case Pit:   //--- Pit ------------------------
        switch(PitMode)
        {
        case Mech: return_angle[Pit] = &Target.Mech_Pit;
            break;
        case A_IMU: return_angle[Pit] = &Target.AIMU_Pit;
            break;
        case C_IMU: return_angle[Pit] = &Target.CIMU_Pit;
            break;
        }
        break;
    }
    return return_angle[type];
}

/**
 * @brief      获取计算的测量角度
 * @param[in]  type
 * @retval     Current Angle 
 * @note       返回的是计算时的量纲
 */
float Gimbal_classdef::Get_CurrentAngle(Gimbal_type_e type)
{
    static float return_angle[2];

    switch(type)
    {
    case Yaw:   //--- Yaw ------------------------
        switch(YawMode)
        {
        case Mech: return_angle[Yaw] = Motor[Yaw].get_totalencoder();
            break;
        case C_IMU: return_angle[Yaw] = Get_IMUTotalAngle(Yaw)*ENCODER_ANGLE_RATIO;
            break;
        }
        break;

    case Pit:   //--- Pit ------------------------
        switch(PitMode)
        {
        case Mech: return_angle[Pit] = Motor[Pit].get_totalencoder();
            break;
        case C_IMU: return_angle[Pit] = Get_IMUAngle(Rol)*ENCODER_ANGLE_RATIO;
            break;
        }
        break;
    }

    return return_angle[type];
}

/**
 * @brief      更新目标角度
 * @param[in]  Yawparam Pitparam
 * @note       这里的数值会换成和电机相同的量纲
 * @retval     None
 */
float YawDebug_target;
float PitDebug_target;
float YawDebug_tar_cs=0.03;
float YawDebug_tar_cs_turn=1.5;
float PitDebug_target_cs;
//int Enemy_VanishTime;
static bool Last_Enemy;
static float Last_Yaw;
static float Last_Pit;
//KFP Vision_Yaw;
uint8_t mark=0;//上一次是否有敌人标志位
uint8_t hit=0;
int time=0;//检测敌人停留时间
float time1=0;//转头反应时间
float  i;    //sin函数的
float32_t pSinVal;
float Temp_Yaw=0;
uint8_t yaw_max;
float32_t pCosVal;
float32_t ySinVal;
float32_t yCosVal;
uint8_t yaw_mark=0;
float Pitch_Err;
float Yaw_Err;
float Last_Pitch_Err;
float Last_Yaw_Err;
KFP Vision_Yaw = {0.02,0,0,0,0.001,0.543};
float depth;
float aim_err;
void Gimbal_classdef::TargetAngle_Update(float Yawparam, float Pitparam)
{		
	Pitch_Err=((RV_slove_expert.pitch)*180/PI-(Gimbal.Get_IMUAngle(Rol)-180));
	Yaw_Err=((RV_slove_expert.yaw)*180/PI-(Gimbal.Get_IMUAngle(Yaw)-180));
	if(Yaw_Err-Last_Yaw_Err>=180)
	{
		Yaw_Err-=360;
	}
	else if(Yaw_Err-Last_Yaw_Err<=-180)
	{
		Yaw_Err+=360;
	}
	else
	{}
	if(Pitch_Err-Last_Pitch_Err>=180)
	{
		Pitch_Err-=360;
	}
	else if(Pitch_Err-Last_Pitch_Err<=-180)
	{
		Pitch_Err+=360;
	}
	else
	{}
//		aim_err = abs(Yaw_Err+1.313);
//	if(aim_err<1.0)
//	{
//		RV_slove_expert.auto_aim=1;
//	}
//	else
//	{
//		RV_slove_expert.auto_aim=0;
//	}
		if(RV_slove_revice.Robot_Info.pack.x!=0&&RV_slove_revice.Robot_Info.pack.y!=0)
		{
			RV_slove_expert.enemy=1;
			
		}
		else
		{
			RV_slove_expert.enemy=0;
		}
		if(RV_slove_revice.Robot_Info.pack.id==6||RV_slove_revice.Robot_Info.pack.id==2)//不锁哨兵和工程
		{
			RV_slove_expert.enemy=0;		
		}
		if(Infantry.robot_state.pack.game_progress != 4)
		{
				RV_slove_expert.auto_aim=0;
		}
	
	
	if(RV_slove_expert.aim_x==0)//先后顺序（先过零后判断误差）不可改变
	{
		Yaw_Err=Pitch_Err=0;
	}
	Vision.Tar_Yaw_Vision = (Gimbal.Get_IMUTotalAngle(Yaw)+Yaw_Err )*ENCODER_ANGLE_RATIO;
	Vision.Tar_Pit_Vision = (Gimbal.Get_IMUAngle(Rol)+Pitch_Err)*ENCODER_ANGLE_RATIO;//在总角度的基础上加上视觉给的偏差值

    // --- 视觉有目标时不处理手动值——记得把视觉自瞄在线加上
    if((Infantry.Get_GimbalMode() == Gimbal_PCMode || Infantry.Get_GimbalMode() == Gimbal_PatrolMode))//&& DevicesMonitor.Get_State(VISION_MONITOR) != Off_line)
    {		
        if(Vision.Get_Enemy() != false)//有敌人
        {
						mark=1;
						hit=0;//锁定敌人，清空被打标志位
            Yawparam = Pitparam = 0;//DR16无影响，目标值全靠视觉，将0赋给通道值
						Target.CIMU_Yaw =Vision.Tar_Yaw_Vision;
						Target.CIMU_Pit =Vision.Tar_Pit_Vision;
						Enemy_VanishTime = 0;
						Last_Enemy = Vision.Get_Enemy();
						Last_Yaw = Target.CIMU_Yaw;
						Last_Pit = Target.CIMU_Pit;
//						kalmanFilter(&Vision_Yaw,Get_TargetAngle(Yaw));
//						Vision.KalmanFilter_Ctrl(VISION_ON, Get_TargetAngle(Yaw), Get_TargetAngle(Pit));

        }
				else//没有敌人
				{
							Yawparam=0;
							Pitparam=0;
#if Open_Race ==1

							i+=0.57;
							if(i>=180)i=-180;
							arm_sin_cos_f32(i, &pSinVal, &pCosVal);           
							Target.CIMU_Pit = GIMBAL_PIT_CENTRE + pSinVal*300;



//						if(hit==1)//被击打过还没清除，这边时间累加等待头转过去
//						{
//							time1+=0.3;
//							if(yaw_mark==0)
//							{
//							Temp_Yaw=Target.CIMU_Yaw;
//							yaw_mark=1;
//							}
//							arm_sin_cos_f32(time1, &ySinVal, &yCosVal);           
//							Target.CIMU_Yaw = Temp_Yaw + ySinVal*850;
//							
//							
//							if(time1>=1000)
//							{
//								hit=time1=yaw_mark=0;
//							}
//							
//						}
//					
					
					
//						if(Infantry.HurtArmor_NoZero==0&&hit!=1)//未被攻击且上次没有还未看到敌人置零标志位
//						{
					
//							Yawparam=0;
//							Pitparam=0;
						if(mark!=0)//上一次检测到敌人
						{
							time++;
							if(time>=500)
							{
								mark=time=0;								
							}
						}
						else if(mark==0)
							{
						
								arm_sin_cos_f32(i, &pSinVal, &pCosVal);           
								Yawparam = +0.0005*250;
								Enemy_VanishTime++;
								Pitparam = 0;					
							}
//						}
//						else//未看见敌人且被攻击
//						{
//							if(hit!=1)
//							{
//							hit=1;//被打标记位
//							float Err = (Armor_ID[Infantry.HurtArmor_NoZero-1]-Motor[Yaw].getencoder());//*360/8191;//
//							if(Err<-4096)
//								{
//								Err+=8191;
//								}
//							else if(Err>4096)
//								{
//								Err-=8191;
//								}
//							else{}
//							switch(Infantry.HurtArmor_NoZero)
//								{
//								case 1:
//								case 2:
//								case 3:
//								case 4:
//												Target.CIMU_Yaw+=Err;//执行到这里记录此刻目标值
//								break;								
//								}
//							}							
//						}
						
#endif

					}
				
				}
		
		
		
		
		
		
		
		
		
		



    /*--- 实时更新其他控制模式的初始角度值,防止模式切换时炸机 ---------*/
    switch(YawMode)
    {
    case Mech: Target.Mech_Yaw += Yawparam * 30.0f;
               Target.CIMU_Yaw = Get_IMUTotalAngle(Yaw) * ENCODER_ANGLE_RATIO;
        break;
    case A_IMU: Target.AIMU_Yaw += Yawparam * ENCODER_ANGLE_RATIO;//
                Target.Mech_Yaw = Motor[Yaw].get_totalencoder();
                Target.CIMU_Yaw = Get_IMUTotalAngle(Yaw) * ENCODER_ANGLE_RATIO;
        break;
    case C_IMU: Target.CIMU_Yaw += -(Yawparam * ENCODER_ANGLE_RATIO);
                Target.Mech_Yaw = Motor[Yaw].get_totalencoder();
        break;
    }

    /*--- Pit 目标角度更新 -----------------------------------------*/
    switch(PitMode)
    {
    case Mech: Target.Mech_Pit += Pitparam;
               Target.CIMU_Pit = Get_IMUAngle(Rol) * ENCODER_ANGLE_RATIO;
        break;
    case A_IMU: Target.AIMU_Pit += Pitparam;
                Target.Mech_Pit = Motor[Pit].get_totalencoder();
                Target.CIMU_Pit = Get_IMUAngle(Rol) * ENCODER_ANGLE_RATIO;
        break;
    case C_IMU: Target.CIMU_Pit += Pitparam*1.5f;
                Target.Mech_Pit = Motor[Pit].get_totalencoder();
        break;
    }

    /*--- Pit 角度限制 ---------------------------------------------*/
    Pit_AngleLimit();
		if(Vision.Get_Enemy() == false)//哨兵死后Yaw轴目标值还在累加，比赛结束后上电云台开始疯转，条件是在没有敌人和没被击打的情况下才限幅
		{																			 //因为里面对目标值的突变进行了限幅，避免影响hit给的目标值和视觉给的目标值
		Yaw_AngleLimit();
		}
		Last_Yaw_Err = Yaw_Err;
		Last_Pitch_Err = Pitch_Err;

	}


/**
 * @brief      云台电机位置式PID计算
 * @param[in]  None
 * @note       这里的数值会换算成和电机相同的量纲
 * @retval     None
 */
float pid_yawnow_sxxs=1.6;
float pid_pitnow_sxxs=0.7;
float debug_yaw_kp = 0.8f;
float debug_pit_kp = 0.6f;
float temp_wind_offset_yaw = 0.0f;
float temp_wind_offset_pit = 0.0f;

void Gimbal_classdef::Motor_PIDCalc()
{

    
		//当前值读取
    switch(YawMode)
    {
    case Mech:
        YawPID[Mech][PID_Outer].Current = Motor[Yaw].get_totalencoder();
        YawPID[Mech][PID_Inner].Current = Motor[Yaw].getSpeed();
        break;

    case A_IMU:
        // YawPID[A_IMU][PID_Outer].Current = 8888 * ENCODER_ANGLE_RATIO;
        // YawPID[A_IMU][PID_Inner].Current = 9999;
        break;

    case C_IMU:
    case V_CIMU:
        YawPID[C_IMU][PID_Outer].Current = YawPID[V_CIMU][PID_Outer].Current = Get_IMUTotalAngle(Yaw) * ENCODER_ANGLE_RATIO;
        // YawPID[C_IMU][PID_Inner].Current = YawPID[V_CIMU][PID_Inner].Current = CIMU.OffsetGyro[Yaw]/*  Get_IMUGyro(Yaw) *//* temp_gz */;

        YawPID[C_IMU][PID_Inner].Current = YawPID[V_CIMU][PID_Inner].Current = Get_IMUGyro(Yaw);//*pid_yawnow_sxxs/*  Get_IMUGyro(Yaw) *//* temp_gz */;
				YawPID[YawMode][PID_current].Current = Motor[Yaw].givenCurrent;
        break;
    }

    switch(PitMode)
    {
    case Mech:
    case V_Mech:
        PitPID[Mech][PID_Outer].Current = PitPID[V_Mech][PID_Outer].Current = Motor[Pit].get_totalencoder();
        // PitPID[Mech][PID_Inner].Current = PitPID[V_Mech][PID_Inner].Current = Get_IMUGyro(Rol)/* Motor[Pit].getSpeed() */;
        // PitPID[Mech][PID_Inner].Current = PitPID[V_Mech][PID_Inner].Current = KalmanFilter(&Kalman_PitSpeed, /* Get_IMUGyro(Rol) */INS_OffsetGyro[1]*(180/PI));

        PitPID[Mech][PID_Inner].Current = PitPID[V_Mech][PID_Inner].Current = Get_IMUGyro(Rol);
        break;

    case A_IMU: 
        break;

    case C_IMU:
    case V_CIMU:
        PitPID[C_IMU][PID_Outer].Current = PitPID[V_CIMU][PID_Outer].Current = Get_IMUAngle(Rol) * ENCODER_ANGLE_RATIO;
        // PitPID[C_IMU][PID_Inner].Current = PitPID[V_CIMU][PID_Inner].Current = /* Motor[Pit].getSpeed() */KalmanFilter(&Kalman_PitSpeed, /* Get_IMUGyro(Rol) */INS_OffsetGyro[1]*(180/PI));

        PitPID[C_IMU][PID_Inner].Current = PitPID[V_CIMU][PID_Inner].Current = Get_IMUGyro(Rol)*pid_pitnow_sxxs;
        break;
    }
    /*--- Yaw PID Calc --------------------------------------------------------------------------*/
    // YawPID[YawMode][PID_Outer].Target = *Get_TargetAngle(Yaw);
    // YawPID[YawMode][PID_Inner].Target = YawPID[YawMode][PID_Outer].Cal();
    // Motor[Yaw].Out = YawPID[YawMode][PID_Inner].Cal();

    /*--- Pit PID Calc --------------------------------------------------------------------------*/
    if(Infantry.Get_GimbalMode() == Gimbal_PCMode || Infantry.Get_GimbalMode() == Gimbal_PatrolMode)//[5][0]/[5][1]  V_CIMU视觉自瞄pid
    {
        /*--- Yaw PID Calc --------------------------------------------------------------------------*/
        YawPID[YawMode+3][PID_Outer].Target = *Get_TargetAngle(Yaw) + temp_wind_offset_yaw;
        YawPID[YawMode+3][PID_Inner].Target = YawPID[YawMode+3][PID_Outer].Cal();
				
//				if(abs(YawPID[YawMode+3][PID_Outer].Error)<180*ENCODER_ANGLE_RATIO)
//				{
//					YawPID[YawMode+3][PID_Inner].Current *= (abs(YawPID[YawMode+3][PID_Outer].Error)/(180*ENCODER_ANGLE_RATIO)*0.7+1.3);
//				}
//				else{YawPID[YawMode+3][PID_Inner].Current *= 2;}
        Motor[Yaw].Out = YawPID[YawMode+3][PID_Inner].Cal();

        /*--- Pit PID Calc --------------------------------------------------------------------------*/
        PitPID[PitMode+3][PID_Outer].Target = *Get_TargetAngle(Pit) + temp_wind_offset_pit;
        PitPID[PitMode+3][PID_Inner].Target = PitPID[PitMode+3][PID_Outer].Cal();
        Motor[Pit].Out = PitPID[PitMode+3][PID_Inner].Cal() /* + (3.5873f*PitPID[PitMode+3][PID_Outer].Current) */;

    }
    else																																														//[2][0]/[2][1]  C_IMU正常Pid
    {
        /*--- Yaw PID Calc --------------------------------------------------------------------------*/
			#if Use_Currentloop==0

        YawPID[YawMode][PID_Outer].Target = *Get_TargetAngle(Yaw);
        YawPID[YawMode][PID_Inner].Target = YawPID[YawMode][PID_Outer].Cal();
        Motor[Yaw].Out = YawPID[YawMode][PID_Inner].Cal();
			#elif Use_Currentloop==1
			  YawPID[YawMode][PID_Outer].Target = *Get_TargetAngle(Yaw);
        YawPID[YawMode][PID_Inner].Target = YawPID[YawMode][PID_Outer].Cal();
        YawPID[YawMode][PID_current].Target= YawPID[YawMode][PID_Inner].Cal();
				Motor[Yaw].Out = YawPID[YawMode][PID_current].Cal();

			
			#endif


        /*--- Pit PID Calc --------------------------------------------------------------------------*/
        PitPID[PitMode][PID_Outer].Target = *Get_TargetAngle(Pit);
        PitPID[PitMode][PID_Inner].Target = PitPID[PitMode][PID_Outer].Cal();
        Motor[Pit].Out = PitPID[PitMode][PID_Inner].Cal() /* + (3.5873f*PitPID[PitMode][PID_Outer].Current) */;

        // YawPID[YawMode+3][PID_Outer].Reset();
        // YawPID[YawMode+3][PID_Inner].Reset();
        // PitPID[PitMode+3][PID_Outer].Reset();
        // PitPID[PitMode+3][PID_Inner].Reset();
    }
		
//		if(Chassis.Mode == CHAS_SpinMode)
//		{
//			
//			if(Chassis.Get_TargetVw()>9000)
//			{
//				Send_Motor[SendID_Yaw].Out = Motor[Yaw].Out - 10000;
//			}
//			if(Chassis.Get_TargetVw()<-9000)
//			{
//				Send_Motor[SendID_Yaw].Out = Motor[Yaw].Out + 10000;
//			}
//			else
//			{
//				Send_Motor[SendID_Yaw].Out = Motor[Yaw].Out -	Chassis.Get_TargetVw()/9000*10000;
//		
//			}
//		}
//		else
//		{
//			Send_Motor[SendID_Yaw].Out = Motor[Yaw].Out;
//		}
//		Shoot.Send_Motor[SendID_Pit].Out = Motor[Pit].Out;//Pitch电机和shoot的电机的电调的接收id一样所以打包放在shoot里面统一发送
		Motor_Yaw[Yaw_Up].Out = Motor[Yaw].Out;
		Shoot.Send_Motor[SendID_Pit].Out = Motor[Pit].Out;
//		Send_Motor[SendID_Yaw].Out = Motor[Yaw].Out;
//		Shoot.Send_Motor[SendID_Pit].Out = -Motor[Pit].Out;//Pitch电机和shoot的电机的电调的接收id一样所以打包放在shoot里面统一发送
}


/**
 * @brief      设置Yaw轴电机控制模式
 * @param[in]  mode
 * @retval     None
 */
void Gimbal_classdef::Set_YawCtrlMode(GimbalMotor_Ctrl_e mode)
{
    switch (mode)
    {
    case Mech:
        YawMode = Mech;
        // Target.AIMU_Yaw = 8888 * ENCODER_ANGLE_RATIO;
        Target.CIMU_Yaw = Get_IMUTotalAngle(Yaw);
        break;

    case A_IMU:
        YawMode = A_IMU;
        Target.Mech_Yaw = Motor[Yaw].get_totalencoder();
        break;

    case C_IMU:
        YawMode = C_IMU;
        Target.Mech_Yaw = Motor[Yaw].get_totalencoder();
        break;
    }
}


/**
 * @brief      设置Pit轴电机控制模式
 * @param[in]  mode
 * @retval     None
 */
void Gimbal_classdef::Set_PitCtrlMode(GimbalMotor_Ctrl_e mode)
{
    switch (mode)
    {
    case Mech:
        PitMode = Mech;
        // Target.AIMU_Yaw = AIMU..
        Target.CIMU_Pit = Get_IMUAngle(Rol);
        break;

    case A_IMU:
        PitMode = A_IMU;
        break;

    case C_IMU:
        PitMode = C_IMU;
        Target.Mech_Pit = Motor[Pit].get_totalencoder();
        break;
    }
}



/**
 * @brief      设置初始目标角度
 * @param[in]  yaw_init
 * @param[in]  pit_init
 * @retval     None
 */
void Gimbal_classdef::Set_InitAngle()
{
    switch(YawMode)
    {
    case Mech: Target.Mech_Yaw = Motor[Yaw].get_totalencoder();
        break;
    case A_IMU: Target.AIMU_Yaw = 8888;//
        break;
    case C_IMU: Target.CIMU_Yaw = Get_IMUTotalAngle(Yaw)*ENCODER_ANGLE_RATIO;
        break;
    }

    switch(PitMode)
    {
    case Mech: Target.Mech_Pit = (init_mode == true?GIMBAL_PIT_CENTRE:Motor[Pit].get_totalencoder());
        break;
    case A_IMU: Target.AIMU_Pit = 7777;
        break;
    //case C_IMU: Target.CIMU_Pit = (init_mode == true?Get_IMUTotalAngle(Pit)-(Motor[Pit].get_totalencoder()-GIMBAL_PIT_CENTRE):Get_IMUTotalAngle(Pit) * ENCODER_ANGLE_RATIO);
		case C_IMU: Target.CIMU_Pit = (init_mode == true?Get_IMUAngle(Rol)-(Motor[Pit].get_totalencoder()-GIMBAL_PIT_CENTRE):Get_IMUAngle(Rol) * ENCODER_ANGLE_RATIO);

        break;
    }
}
void Gimbal_classdef::Yaw_AngleLimit()
{
	
	if(abs(Target.CIMU_Yaw-Get_IMUTotalAngle(Yaw) * ENCODER_ANGLE_RATIO)>2096)
	{
		Target.CIMU_Yaw=Get_IMUTotalAngle(Yaw) * ENCODER_ANGLE_RATIO;
	}
	
	
	
}

/**
 * @brief      Pit电机角度限幅
 * @param[in]  None
 * @retval     None
 */
void Gimbal_classdef::Pit_AngleLimit()
{
//    static float Limit_Min;
//    static float Limit_Max;

//    switch(PitMode)
//    {
//    case Mech:
//        Limit_Min = GIMBAL_PIT_MIN;
//        Limit_Max = GIMBAL_PIT_MAX;
//        break;

//    case C_IMU:
//        //--- 陀螺仪模式以码盘值作为参考 映射到IMU角度进行角度限制
//        Limit_Min = Get_IMUAngle(Pit)*ENCODER_ANGLE_RATIO - fabs(Get_PitDeviateL());
//        Limit_Max = Get_IMUAngle(Pit)*ENCODER_ANGLE_RATIO + fabs(Get_PitDeviateH());
//        break;
//    }

//    if(*Get_TargetAngle(Pit) > Limit_Max)
//    {
//        *Get_TargetAngle(Pit) = Limit_Max;
//    }
//    else if(*Get_TargetAngle(Pit) < Limit_Min)
//    {
//        *Get_TargetAngle(Pit) = Limit_Min;
//    }
			if(*Get_TargetAngle(Pit)>GIMBAL_PIT_MAX)		
			{
				*Get_TargetAngle(Pit)=GIMBAL_PIT_MAX;
			}
			else if(*Get_TargetAngle(Pit)<GIMBAL_PIT_MIN)
			{
				*Get_TargetAngle(Pit)=GIMBAL_PIT_MIN;
			}
			else
			{}
}


/**
 * @brief      获取当前位置与Yaw轴中心点的偏离角度
 * @param[in]  cur_encoder
 * @retval     Off angle(弧度制)
 */
float Gimbal_classdef::Get_YawDeviateC(float cur_encoder)
{
    return (cur_encoder * (PI/180));
}
/**
 * @brief      获取当前位置与Yaw轴反向中心点的偏离角度
 * @param[in]  cur_encoder
 * @retval     Off angle(弧度制)
 */
float Gimbal_classdef::Get_YawDeviateReC(float cur_encoder)
{
    return ((cur_encoder - GIMBAL_YAW_CENTRE - 4096) / ENCODER_ANGLE_RATIO * (PI/180));
}

/**
 * @brief      获取Pit电机偏离限幅值的角度
 * @param[in]  None
 * @retval     deviate encoder
 */
float Gimbal_classdef::Get_PitDeviateH()
{
    return ((Motor[Pit].get_totalencoder()>GIMBAL_PIT_MAX)? 0 : Motor[Pit].get_totalencoder()-GIMBAL_PIT_MAX);
}
float Gimbal_classdef::Get_PitDeviateL()
{
    return ((Motor[Pit].get_totalencoder()<GIMBAL_PIT_MIN)? 0 : Motor[Pit].get_totalencoder()-GIMBAL_PIT_MIN);
}

/**
 * @brief      限制遥控模式下的云台与底盘的分离角度
 * @param[in]  Switch
 * @param[in]  yawparam
 * @retval     None
 */
float xxxx = 0.0015; //0.05
float aaaa = 0.03f; //5
float bbbb = 180.0f;
void Gimbal_classdef::CHAS_SeparateLimit(bool Switch, float *yawparam)
{
    static float Separate_diff = 0;

    if(Switch == OFF || Infantry.Get_CtrlSource() != CTRL_RC || (Infantry.Get_ChassisMode() != CHAS_FollowMode && Infantry.Get_ChassisMode() != CHAS_ReFollowMode))
    {
        return;
    }

    Separate_diff = Gimbal.Motor[Yaw].getAngle();

    if(Separate_diff > 300)
    {
        Separate_diff -= 360.0f;
    }
    else if(Separate_diff < -300)
    {
        Separate_diff += 360.0f;
    }

    if(fabs(Separate_diff) > 65.0f && *yawparam != 0)
    {
        YawRCCtrl_Coe -= (aaaa / bbbb);

        if(fabs(YawRCCtrl_Coe) < aaaa / (bbbb-10.0f))
        {
            YawRCCtrl_Coe = 0;
        }
    }
    else
    {
        YawRCCtrl_Coe = xxxx;
    }

    // if(fabs(Motor[Yaw].getencoder() - GIMBAL_YAW_CENTER) > 1000)
    // {
    //     Target.AIMU_Yaw -= temp_param;
    // }
		
}


/**
 * @brief      获取IMU角度(角度制)
 * @param[in]  type
 * @retval     Angle
 */
void Gimbal_classdef::IMU_Update(float *angle, float *gyro)
{
#if USE_IMU_EKF

    for(uint8_t i = 0 ; i < 3 ; i++)
    {
        //--- 角度
        CIMU.Angle[i] = angle[i] + 180; //--- (0 ~ 360°)

        //--- 过零处理,累计角度
        if(CIMU.Angle[i] - CIMU.Pre_Angle[i] < -180)
        {
            CIMU.Cnt[i]++;
        }
        else if(CIMU.Angle[i] - CIMU.Pre_Angle[i] > 180)
        {
            CIMU.Cnt[i]--;
        }

        CIMU.TotalAngle[i] = CIMU.Angle[i] + CIMU.Cnt[i] * 360.0f;
        CIMU.Pre_Angle[i] = CIMU.Angle[i];
    }
    //--- 角速度
    CIMU.Gyro[0] = gyro[2]*(180/PI);
    CIMU.Gyro[1] = gyro[0]*(180/PI);
    CIMU.Gyro[2] = LPF_Gyro_y*(180/PI);//gyro[1]*(180/PI);


#else
    for(uint8_t i = 0 ; i < 3 ; i++)
    {
        //--- 角度
        CIMU.Angle[i] = angle[i]*(180/PI) + 180; //--- 弧度转化为度(0 ~ 360°)
        //--- 角速度
        CIMU.Gyro[i] = gyro[2-i]*(180/PI);
        //--- 解算后的角速度 Yaw
        CIMU.OffsetGyro[i] = INS_OffsetGyro[2-i]*(180/PI);

        //--- 过零处理,累计角度
        if(CIMU.Angle[i] - CIMU.Pre_Angle[i] < -180)
        {
            CIMU.Cnt[i]++;
        }
        else if(CIMU.Angle[i] - CIMU.Pre_Angle[i] > 180)
        {
            CIMU.Cnt[i]--;
        }

        CIMU.TotalAngle[i] = CIMU.Angle[i] + CIMU.Cnt[i] * 360.0f;
        CIMU.Pre_Angle[i] = CIMU.Angle[i];
    }
#endif

}

/**
 * @brief      获取IMU角度(角度制)
 * @param[in]  type
 * @retval     Angle
 */
float Gimbal_classdef::Get_IMUAngle(Gimbal_type_e type)
{
    return CIMU.Angle[type];
}

/**
 * @brief      获取IMU累计角度(角度制)
 * @param[in]  type
 * @retval     TotalAngle
 */
float Gimbal_classdef::Get_IMUTotalAngle(Gimbal_type_e type)
{
    return CIMU.TotalAngle[type];
}

/**
 * @brief      获取IMU角速度
 * @param[in]  type
 * @retval     Gyro
 */
float Gimbal_classdef::Get_IMUGyro(Gimbal_type_e type)
{
    return CIMU.Gyro[type];
}




