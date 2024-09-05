#include "Task_CAN.h"
#include "Task_Init.h"
#include "BSP_CAN.h"
#include "System_DataPool.h"
#include "Power_Meter.h"


/**
 * @brief      CAN1 接收任务
 * @param[in]  None
 * @retval     None
 */
void Task_CAN1_Recv(void *argument)
{
	static CanRxMsg_t CAN1_Rx;

    for(;;)
    {
        // --- 等待队列不为空
        xQueueReceive(Can1Recv_QueueHandle, &CAN1_Rx, portMAX_DELAY);

        switch(CAN1_Rx.Header.StdId)
        {
        case 0x20A:
            Gimbal.Motor[Yaw_Low].update(CAN1_Rx.Data); // Yaw轴电机，和拨盘都是CAN1接收所以封装在一起发送
            break;
        case 0x209:
            Gimbal.Motor[Yaw_Up].update(CAN1_Rx.Data); // Yaw轴电机，和拨盘都是CAN1接收所以封装在一起发送
            break;
				case 0x86:
            Infantry.WriteMsgFromGimbal(0x86,CAN1_Rx.Data);
            break;




        }

    }
}


/**
 * @brief      CAN2 接收任务
 * @param[in]  None
 * @retval     None
 */
void Task_CAN2_Recv(void *argument)
{
	static CanRxMsg_t CAN2_Rx;

    for(;;)
    {

        // --- 等待队列不为空
        xQueueReceive(Can2Recv_QueueHandle, &CAN2_Rx, portMAX_DELAY);

        switch(CAN2_Rx.Header.StdId)
        {
			  case 0x201:
            Chassis.DRV_Motor[0].update(CAN2_Rx.Data);
            DevicesMonitor.Update(Frame_CHAS_DRV0);
            break;

        case 0x202:
            Chassis.DRV_Motor[1].update(CAN2_Rx.Data);
            DevicesMonitor.Update(Frame_CHAS_DRV1);
            break;
        
        case 0x203:
            Chassis.DRV_Motor[2].update(CAN2_Rx.Data);
            DevicesMonitor.Update(Frame_CHAS_DRV2);
            break;

        case 0x204:
            Chassis.DRV_Motor[3].update(CAN2_Rx.Data);
            DevicesMonitor.Update(Frame_CHAS_DRV3);
            break;
        case 0x205:
            Chassis.RUD_Motor[0].update(CAN2_Rx.Data);
            DevicesMonitor.Update(Frame_CHAS_RUD0);
            break;

        case 0x206:
            Chassis.RUD_Motor[1].update(CAN2_Rx.Data);
            DevicesMonitor.Update(Frame_CHAS_RUD1);
            break;
        
        case 0x207:
            Chassis.RUD_Motor[2].update(CAN2_Rx.Data);
            DevicesMonitor.Update(Frame_CHAS_RUD2);
            break;

        case 0x208:
            Chassis.RUD_Motor[3].update(CAN2_Rx.Data);
            DevicesMonitor.Update(Frame_CHAS_RUD3);
            break;

        }
    }
}
