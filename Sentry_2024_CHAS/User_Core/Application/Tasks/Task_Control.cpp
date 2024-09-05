#include "Task_Control.h"
#include "Task_Init.h"
#include "System_DataPool.h"
#include "Dev_LED.h"

/**
 * @brief      �ܿ�����
 * @param[in]  None
 * @retval     None
 */
void Task_Control(void *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2);  // --- 2MS 

    static uint8_t led_cnt = 0;

    for(;;)
    {
        led_cnt++;
#if Up_Control == 0//ע��һ�£����˶�ʱ���Ҷ���������ô�������¿����ˣ��Ұ�Expt.Target_Vx��Target_Low_Vx��Target_Vx�ֿ���
        CTRL_DR16.LeverMode_Update();
				CTRL_DR16.CTRLSource_Update();
#else 
        CTRL_DR16.LeverMode_Update();
#endif
        Infantry.Control();

        if((led_cnt%=50) == 0)
        {   //--- ��������������ˮ��
            Waterfall_LED();
        }

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

