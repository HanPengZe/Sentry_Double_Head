#ifndef __SOLVETRAJECTORY_H__
#define __SOLVETRAJECTORY_H__
#ifndef PI
#define PI 3.1415926535f
#endif
#define GRAVITY 9.7833f
typedef unsigned char uint8_t;
enum ARMOR_ID
{
    ARMOR_OUTPOST = 0,
    ARMOR_HERO = 1,
    ARMOR_ENGINEER = 2,
    ARMOR_INFANTRY3 = 3,
    ARMOR_INFANTRY4 = 4,
    ARMOR_INFANTRY5 = 5,
    ARMOR_GUARD = 6,
    ARMOR_BASE = 7
};

enum ARMOR_NUM
{
    ARMOR_NUM_BALANCE = 2,
    ARMOR_NUM_OUTPOST = 3,
    ARMOR_NUM_NORMAL = 4
};

enum BULLET_TYPE
{
    BULLET_17 = 0,
    BULLET_42 = 1
};

typedef	struct
		{
			//Ŀ�����
			float xw;             //ROS����ϵ�µ�x
			float yw;             //ROS����ϵ�µ�y
			float zw;             //ROS����ϵ�µ�z
			float tar_yaw;        //Ŀ��yaw
			float vxw;            //ROS����ϵ�µ�vx
			float vyw;            //ROS����ϵ�µ�vy
			float vzw;            //ROS����ϵ�µ�vz
			float v_yaw;          //Ŀ��yaw�ٶ�
			float r1;             //Ŀ�����ĵ�ǰ��װ�װ�ľ���
			float r2;             //Ŀ�����ĵ�����װ�װ�ľ���
			float dz;             //��һ��װ�װ������ڱ�����װ�װ�ĸ߶Ȳ�
		}ST;

		
		typedef	union
	{
		ST pack;
		uint8_t data[44];
	}sss_t;
//���ò���
struct SolveTrajectoryParams
{
    float k;             //����ϵ��

    //�������
    enum BULLET_TYPE bullet_type;  //������������� 0-���� 1-Ӣ��
    float current_v;      //��ǰ����
    float current_pitch;  //��ǰpitch
    float current_yaw;    //��ǰyaw

		sss_t sss;
    int bias_time;        //ƫ��ʱ��
    float s_bias;         //ǹ��ǰ�Ƶľ���
    float z_bias;         //yaw������ǹ��ˮƽ��Ĵ�ֱ����
    enum ARMOR_ID armor_id;     //װ�װ�����  0-outpost 6-guard 7-base
                                //1-Ӣ�� 2-���� 3-4-5-���� 
    enum ARMOR_NUM armor_num;   //װ�װ�����  2-balance 3-outpost 4-normal
};

//���ڴ洢Ŀ��װ�װ����Ϣ
struct tar_pos
{
    float x;           //װ�װ�����������ϵ�µ�x
    float y;           //װ�װ�����������ϵ�µ�y
    float z;           //װ�װ�����������ϵ�µ�z
    float yaw;         //װ�װ�����ϵ�������������ϵ��yaw��
};

typedef struct
{
		float pitch, yaw,aim_x, aim_y, aim_z,auto_aim,enemy;
} RV_Slove_Expert_t;

uint8_t Add_time();
extern uint8_t Add_time();
//�������������ģ��
extern float monoDirectionalAirResistanceModel(float s, float v, float angle);
//��ȫ��������ģ��
extern float completeAirResistanceModel(float s, float v, float angle);
//pitch��������
extern float pitchTrajectoryCompensation(float s, float y, float v);
//�������ž��ߵó�������װ�װ� �Զ����㵯��
extern void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z);
void SolveTrajectory_Init(void);
uint8_t ssss_solve(void);

void RV_SolveTrajectory(void);
extern uint8_t count;
extern RV_Slove_Expert_t RV_slove_expert;
extern struct SolveTrajectoryParams st;
#endif /*__SOLVETRAJECTORY_H__*/
