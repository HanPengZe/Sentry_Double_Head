#include "Task_init.h"
#include "System_DataPool.h"
#include "SolveTrajectory.h"
#include "Devices_Monitor.h"

/**
 * @brief      裁判系统数据更新
 * @param[in]  None
 * @retval     None
 */
void Recv_Referee(void *arg)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2);
		SolveTrajectory_Init();
    for(;;)
	{

//		Add_time();
		RV_SolveTrajectory();
		RV_slove_expert.auto_aim=ssss_solve();
		if((Gimbal.Motor[0].getencoder()>1700&&Gimbal.Motor[0].getencoder()<3600)||(Gimbal.Motor[0].getencoder()>5800&&Gimbal.Motor[0].getencoder()<7700))
		{
			RV_slove_expert.auto_aim=0;//下头龙门架机械角度限制下头开火
		}
		if(RV_slove_revice.Robot_Info.pack.vx!=0&&RV_slove_revice.Robot_Info.pack.vy!=0)
		{
			RV_slove_expert.enemy=1;		
		}
		
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
		
	}
}
