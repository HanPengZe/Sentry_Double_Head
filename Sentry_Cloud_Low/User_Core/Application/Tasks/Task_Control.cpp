#include "Task_init.h"
#include "System_DataPool.h"
//#include "Control_Robot.h"


/**
 * @brief      总控任务
 * @param[in]  None
 * @retval     None
 */
void Task_Control(void *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2);  // --- 2MS 
		SolveTrajectory_Init();

    for(;;)
    {
		RV_SolveTrajectory();
		RV_slove_expert.auto_aim=ssss_solve();
		if((Gimbal.Motor[0].getencoder()>1600&&Gimbal.Motor[0].getencoder()<3700)||(Gimbal.Motor[0].getencoder()>5700&&Gimbal.Motor[0].getencoder()<7800))
		{
			RV_slove_expert.auto_aim=0;//下头龙门架机械角度限制下头开火
		}
        Infantry.Control();

        if(Infantry.Get_FPS() == 500)
        {
            LED.BLN_Ctrl();
        }

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
