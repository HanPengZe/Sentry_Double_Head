#ifndef _ROBOT_CONFIG_H_
#define _ROBOT_CONFIG_H_

/* Macro Definitions ---------------------------------------------------------*/
/* 以此纪念入队至今所接触过的RM步兵机器人 */

#define ROBOT_ID INFANTRY_2022_SWERVE_2 // - 当前机器人 ID

#define INFANTRY_2019 1          // - 2019步兵
#define INFANTRY_2020 2          // - 2020步兵
#define INFANTRY_2021 3          // - 2021步兵
#define INFANTRY_2021_ZJ 4       // - 2021焯钧步兵
#define INFANTRY_2022_SWERVE_1 5   // - 2022第1台舵轮步兵
#define INFANTRY_2022_SWERVE_2 6 // - 2022第2台舵步
#define INFANTRY_2022_SWERVE_3 7 // - 2022第3台舵步

#define SRNTRY_2023_OMNI 8 // - 2023全向哨兵

#define Device_BoardType Device_BoardType_A

#define Device_BoardType_Wolf 0 // 彬哥板
#define Device_BoardType_old 1  // RM 旧板
#define Device_BoardType_A 2    // RM A板
#define Device_BoardType_C 3    // RM C板

#define USE_DeviceMonitor 1 //启用离线、错误报警
#define USE_Buzzer 1        //启用蜂鸣器

#define USE_GimbalCtrl 1  // 使能云台控制
#define USE_ShootSystem 1 // 启用射击系统
#define USE_GimbalIMU 1   // 启用云台陀螺仪
#define USE_RM_Referee 1  // 启动裁判系统

#endif
