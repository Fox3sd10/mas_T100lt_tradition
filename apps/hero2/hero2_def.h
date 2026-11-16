#ifndef _HERO2_DEF_H_
#define _HERO2_DEF_H_

#include <stdint.h>
#include "module_boardcomm.h"
#include "solution_def.h"


/* 板间通讯结构体 */
#pragma pack(push, 1)

typedef struct
{
    int8_t  vx;           /* X速度, -10 ～ +10 */
    int8_t  vy;           /* Y速度, -10 ～ +10 */
    int8_t  wz;           /* 旋转速度, -10 ～ +10 */
    int16_t offset_angle; /* yaw偏移角度 */
    uint8_t chassis_mode; /* 底盘控制指令 (chassis_mode_e) */
    int8_t  reserved[2];  /* 填充至8字节 */
} GimbalToChassis_cmd_t;
_Static_assert(sizeof(GimbalToChassis_cmd_t) == 8, "GimbalToChassis_cmd_t must be 8 bytes");

typedef struct
{
    uint8_t  robot_color;      /* 机器人颜色 (0=红,1=蓝) */
    uint8_t  game_progress;    /* 比赛阶段 */
    uint16_t current_hp;       /* 当前血量 */
    uint16_t bullet_allow;     /* 可发射17mm弹丸剩余量 */
    uint16_t shooter_heat_pct; /* 枪管热量% = heat/limit*100 */
} ChassisToGimbal_referee_t;
_Static_assert(sizeof(ChassisToGimbal_referee_t) == 8, "ChassisToGimbal_referee_t must be 8 bytes");

#pragma pack(pop)

#endif
