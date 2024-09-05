/*
@brief: 弹道解算 适配陈君的rm_vision
@author: CodeAlan  华南师大Vanguard战队
*/
// 近点只考虑水平方向的空气阻力


#include <math.h>
#include <stdio.h>
#include "string.h"
#include "SolveTrajectory.h"
#include "usbd_cdc_if.h"
#include "System_DataPool.h"

struct SolveTrajectoryParams st;
struct tar_pos tar_position[4]; //最多只有四块装甲板
float t = 0.5f; // 飞行时间
RV_Slove_Expert_t RV_slove_expert;
float dis,x;float yaw_e;
//uint8_t Add_time()
//{
//	
//	st.bias_time+=2;
//	
//}

/*
@brief 单方向空气阻力弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
float monoDirectionalAirResistanceModel(float s, float v, float angle)
{
    float z;
	if((st.k * v * cos(angle)) == 0)
	{
		t = 0;
        return 0;
	}
	
    //t为给定v与angle时的飞行时间
    t = (float)((exp(st.k * s) - 1) / (st.k * v * cos(angle)));
    if(t < 0)
    {
        //由于严重超出最大射程，计算过程中浮点数溢出，导致t变成负数
        //重置t，防止下次调用会出现nan
        t = 0;
        return 0;
    }
    //z为给定v与angle时的高度
    z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2.0f);
    return z;
}


/*
@brief pitch轴解算
@param s:m 距离
@param z:m 高度
@param v:m/s
@return angle_pitch:rad
*/
float pitchTrajectoryCompensation(float s, float z, float v)
{
    float z_temp, z_actual, dz;
    float angle_pitch;
    int i = 0;
    z_temp = z;
    // iteration
    for (i = 0; i < 20; i++)
    {
        angle_pitch = atan2(z_temp, s); // rad
        z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
        if(z_actual == 0)
        {
            angle_pitch = 0;
            break;
        }
        dz = 0.3f*(z - z_actual);
        z_temp = z_temp + dz;
        if (fabsf(dz) < 0.00001f)
        {
            break;
        }
    }
    return angle_pitch;
}

/*
@brief 根据最优决策得出被击打装甲板 自动解算弹道
@param pitch:rad  传出pitch
@param yaw:rad    传出yaw
@param aim_x:传出aim_x  打击目标的x
@param aim_y:传出aim_y  打击目标的y
@param aim_z:传出aim_z  打击目标的z
*/
uint8_t count=0;
float limit;
void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z)
{
	
//    float timeDelay = st.bias_time/1000.0f + t;
////		count++;

//    st.sss.pack.tar_yaw = st.sss.pack.v_yaw * (timeDelay+2/1000*count);

    // 线性预测
    float timeDelay = st.bias_time/1000.0f + t;
		
    st.sss.pack.tar_yaw += st.sss.pack.v_yaw * timeDelay;

    //计算四块装甲板的位置
    //装甲板id顺序，以四块装甲板为例，逆时针编号
    //      2
    //   3     1
    //      0
	int use_1 = 1;
	int i = 0;
    int idx = 0; // 选择的装甲板
    //armor_num = ARMOR_NUM_BALANCE 为平衡步兵
    if (st.armor_num == ARMOR_NUM_BALANCE) 
	{
		limit = 0.2;
        for (i = 0; i<2; i++) 
		{
            float tmp_yaw = st.sss.pack.tar_yaw + i * PI;
            float r = st.sss.pack.r1;
            tar_position[i].x = st.sss.pack.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.sss.pack.yw - r*sin(tmp_yaw);
            tar_position[i].z = st.sss.pack.zw;
            tar_position[i].yaw = tmp_yaw;
        }

        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);

        //因为是平衡步兵 只需判断两块装甲板即可
        float temp_yaw_diff = fabsf(*yaw - tar_position[1].yaw);
        if (temp_yaw_diff < yaw_diff_min)
        {
            yaw_diff_min = temp_yaw_diff;
            idx = 1;
        }
    } 
	else if (st.armor_num == ARMOR_NUM_OUTPOST) 
	{  //前哨站
		limit = 0.07;
        for (i = 0; i<3; i++) 
		{
            float tmp_yaw = st.sss.pack.tar_yaw + i * 2.0 * PI/3.0;  // 2/3PI
            float r =  (st.sss.pack.r1 + st.sss.pack.r2)/2;   //理论上r1=r2 这里取个平均值
            tar_position[i].x = st.sss.pack.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.sss.pack.yw - r*sin(tmp_yaw);
            tar_position[i].z = st.sss.pack.zw;
            tar_position[i].yaw = tmp_yaw;
        }

        //TODO 选择最优装甲板 选板逻辑你们自己写，这个一般给英雄用


    } 
	else 
	{
		limit = 0.1;
        for (i = 0; i<4; i++) 
		{
            float tmp_yaw = st.sss.pack.tar_yaw + i * PI/2.0f;
            float r = use_1 ? st.sss.pack.r1 : st.sss.pack.r2;
            tar_position[i].x = st.sss.pack.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.sss.pack.yw - r*sin(tmp_yaw);
            tar_position[i].z = use_1 ? st.sss.pack.zw : st.sss.pack.zw + st.sss.pack.dz;
            tar_position[i].yaw = tmp_yaw;
            use_1 = !use_1;
        }

            //2种常见决策方案：
            //1.计算枪管到目标装甲板yaw最小的那个装甲板
            //2.计算距离最近的装甲板
		    //3 车中心yaw 追踪+-angle之内

            //计算距离最近的装甲板
			float dis_diff_min = sqrt(tar_position[0].x * tar_position[0].x + tar_position[0].y * tar_position[0].y);
			int idx = 0;
			for (i = 1; i<4; i++)
			{
				float temp_dis_diff = sqrt(tar_position[i].x * tar_position[0].x + tar_position[i].y * tar_position[0].y);
				if (temp_dis_diff < dis_diff_min)
				{
					dis_diff_min = temp_dis_diff;
					idx = i;
				}
			}


            //计算枪管到目标装甲板yaw最小的那个装甲板
//        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);
//        for (i = 1; i<4; i++) 
//		{
//            float temp_yaw_diff = fabsf(*yaw - tar_position[i].yaw);
//            if (temp_yaw_diff < yaw_diff_min)
//            {
//                yaw_diff_min = temp_yaw_diff;
//                idx = i;
//            }
//        }

    }

    *aim_z = tar_position[idx].z + st.sss.pack.vzw* timeDelay;
    *aim_x = tar_position[idx].x + st.sss.pack.vxw * timeDelay;
    *aim_y = tar_position[idx].y + st.sss.pack.vyw * timeDelay;
    //这里符号给错了
    float temp_pitch =-pitchTrajectoryCompensation(sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)) - st.s_bias,
            *aim_z + st.z_bias, st.current_v);
    if(temp_pitch)
        *pitch = temp_pitch;
    if(*aim_x || *aim_y)
        *yaw = (float)(atan2(*aim_y, *aim_x));
}

uint8_t ssss_solve()
{
  //1： 相对角度差做开火
	//--一定频率开火
	//--
		uint8_t out;
	
		dis = sqrt(RV_slove_revice.Robot_Info.pack.x*RV_slove_revice.Robot_Info.pack.x+RV_slove_revice.Robot_Info.pack.y*RV_slove_revice.Robot_Info.pack.y);
	  yaw_e = fabs((Gimbal.Get_IMUAngle(Yaw))/180*PI - RV_slove_expert.yaw); 
	  x = dis*tan(yaw_e);
	  
		//
	  //test = tar_position[idx].yaw;
	
		if(x<limit&&x>-limit&&RV_slove_expert.aim_x!=0)			
		{
			out = 1;
		}
		else out = 0;
		return out;

	
	
	
	
	

}

// 从坐标轴正向看向原点，逆时针方向为正

void SolveTrajectory_Init(void)
{
    //定义参数
    st.k = 0.038;
    st.bullet_type =  BULLET_17;
    st.current_v = 30;
	
    st.current_pitch = 0;
    st.current_yaw = 0;
    st.sss.pack.xw = 3.0;
    // st.yw = 0.0159;
    st.sss.pack.yw = 0;
    // st.zw = -0.2898;
    st.sss.pack.zw = 1.5;

    st.sss.pack.vxw = 0;
    st.sss.pack.vyw = 0;
    st.sss.pack.vzw = 0;
    st.sss.pack.v_yaw = 0;
    st.sss.pack.tar_yaw = 0.09131;
    st.sss.pack.r1 = 0.5;
    st.sss.pack.r2 = 0.5;
    st.sss.pack.dz = 0.1;
		
    st.bias_time = 100;
    st.s_bias = 0.30;
    st.z_bias = 0.015;
    st.armor_id = ARMOR_INFANTRY3;
    st.armor_num = ARMOR_NUM_NORMAL;
}

void RV_SolveTrajectory(void)
{	
	memcpy(st.sss.data,&RV_slove_revice.Robot_Info.data[2],44);
	
	autoSolveTrajectory(&RV_slove_expert.pitch, 
						&RV_slove_expert.yaw, 
						&RV_slove_expert.aim_x, 
						&RV_slove_expert.aim_y,
						&RV_slove_expert.aim_z);
}

