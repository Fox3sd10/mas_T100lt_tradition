#ifndef _HERO2_DEF_H_
#define _HERO2_DEF_H_

#include <stdint.h>

/* 底盘遥控命令 */
typedef struct
{
    float body_target_dx; /* 前后速度命令 m/s, 上推为正, ±2.0 */
    float turn_cmd;       /* 转向设定角命令 (rad), CH4, 满推 ±0.15 (符号实车修正) */
} Chassis_Ctrl_Cmd_t;

/* 板间通讯: 云台 → 底盘 (BOARDCOMM, 4 字节) */
typedef struct
{
    int16_t body_target_dx; /* 前后速度, mm/s, 上推为正, ±2000 */
    int16_t turn_cmd;       /* 转向设定角, mrad, 满推 ±150 */
} GimbalToChassis_cmd_t;
_Static_assert(sizeof(GimbalToChassis_cmd_t) == 4, "GimbalToChassis_cmd_t must be 4 bytes");

#endif
