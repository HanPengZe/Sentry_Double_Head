#include "Task_init.h"
#include "System_DataPool.h"
#include "BSP_CAN.h"

/**
 * @brief      CAN1 ��������
 * @param[in]  None
 * @retval     None
 */
void Task_CAN1_Recv(void *argument)
{
	static CanRxMsg_t CAN1_Rx;

    for(;;)
    {
        // --- �ȴ����в�Ϊ��
        xQueueReceive(Can1Recv_QueueHandle, &CAN1_Rx, portMAX_DELAY);

        switch(CAN1_Rx.Header.StdId)
        {
        case 0x209:
					  Gimbal.Motor[Yaw].update(CAN1_Rx.Data);//Yaw�������Ͳ��̶���CAN1�������Է�װ��һ����
            DevicesMonitor.Update(Frame_GIMBAL_YAW);
            break;
				case 0x666:
						Infantry.RefereeMsg_Write(0x666, CAN1_Rx.Data);
						break;
				case 0x338:
						Infantry.RefereeMsg_Write(0x338, CAN1_Rx.Data);
						break;
        case 0x341:
            Infantry.RefereeMsg_Write(0x341, CAN1_Rx.Data);
            DevicesMonitor.Update(Frame_COMMU_0X341);
            break;

        }

    }
}


/**
 * @brief      CAN2 ��������
 * @param[in]  None
 * @retval     None
 */
void Task_CAN2_Recv(void *argument)
{
	static CanRxMsg_t CAN2_Rx;

    for(;;)
    {
        // --- �ȴ����в�Ϊ��
        xQueueReceive(Can2Recv_QueueHandle, &CAN2_Rx, portMAX_DELAY);

        switch(CAN2_Rx.Header.StdId)
        {
        case 0x205:
            Gimbal.Motor[Pit].update(CAN2_Rx.Data);
            DevicesMonitor.Update(Frame_GIMBAL_PIT);
            break;
        case 0x206:
            Shoot.Fric_Motor[Fric_L].update(CAN2_Rx.Data);
            DevicesMonitor.Update(Frame_FRIC_L);
            break;

        case 0x207:
            Shoot.Fric_Motor[Fric_R].update(CAN2_Rx.Data);
            DevicesMonitor.Update(Frame_FRIC_R);
            break;
        
        case 0x208:
            Shoot.Reload_Motor.update(CAN2_Rx.Data);
            DevicesMonitor.Update(Frame_RELOAD);
            break;
        }
    }
}

/**
 * @brief      CAN1 ��������
 * @param[in]  None
 * @retval     None
 */
void Task_CAN1_Transmit(void *argument)
{ 
    static CanTxMsg_t CAN1_Tx;

    for(;;)
    {
        // �ȴ����в�Ϊ��
		xQueueReceive(Can1Send_QueueHandle, &CAN1_Tx, portMAX_DELAY);
        
    }
}

