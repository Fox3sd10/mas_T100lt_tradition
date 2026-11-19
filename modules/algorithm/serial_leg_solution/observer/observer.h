/*
 * @Author: 550wsleep 1329258004@qq.com
 * @Date: 2026-07-26 12:10:31
 * @LastEditors: 550wsleep 1329258004@qq.com
 * @LastEditTime: 2026-07-27 17:55:22
 * @FilePath: \mas_embedded_threadx\modules\algorithm\serial_leg_solution\observer\observer.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef _OBSERVER_H_
#define _OBSERVER_H_

#include "motor_damiao.h"
#include "motor_dji.h"
#include "solution_def.h"

#define DT                  0.002f

#define FN_THRESH           5.0f
// 卡尔曼滤波矩阵维度（状态2维：x/x_dot；量测2维：v/a；控制0维）
#define KF_STATE_DIM        2
#define KF_CTRL_DIM         0
#define KF_MEASURE_DIM      2

typedef struct {
    uint8_t StateDim;        // 状态向量维度
    uint8_t CtrlDim;         // 控制向量维度
    uint8_t MeasureDim;      // 量测向量维度

    float* P_data;           // 状态协方差矩阵 (StateDim x StateDim)
    float* F_data;           // 状态转移矩阵 (StateDim x StateDim)
    float* H_data;           // 量测矩阵 (MeasureDim x StateDim)
    float* Q_data;           // 过程噪声协方差矩阵 (StateDim x StateDim)
    float* R_data;           // 量测噪声协方差矩阵 (MeasureDim x MeasureDim)

    float FilteredValue[KF_STATE_DIM];    // 滤波后状态值 [x, x_dot]
    float MeasuredVector[KF_MEASURE_DIM]; // 量测向量 [v_measure, a_measure]

    float K[KF_STATE_DIM * KF_MEASURE_DIM];
    float P_temp[KF_STATE_DIM * KF_STATE_DIM];
    float x_temp[KF_STATE_DIM];
} Kalman_t;


void Kalman_Init(Kalman_t* kf, uint8_t state_dim, uint8_t ctrl_dim, uint8_t measure_dim);
void Kalman_Update(Kalman_t* kf);
 
void observer_init(DM_Motor_t *motor_8009[], DJI_Motor_t *motor_3508[]);
void observer_update(DM_Motor_t *motor_8009[], DJI_Motor_t *motor_3508[]);
const observer *observer_get(void);

#endif