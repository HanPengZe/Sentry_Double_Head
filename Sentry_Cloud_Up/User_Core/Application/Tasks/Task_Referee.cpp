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
//		RV_SolveTrajectory();
//		RV_slove_expert.auto_aim=ssss_solve();
//		if(RV_slove_revice.Robot_Info.pack.vx!=0&&RV_slove_revice.Robot_Info.pack.vy!=0)
//		{
//			RV_slove_expert.enemy=1;
//			
//		}
		
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
		
	}
}
