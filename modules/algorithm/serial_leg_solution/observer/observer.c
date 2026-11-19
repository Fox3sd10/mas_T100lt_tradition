/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2026-07-25 19:41:18
 * @LastEditors: 550wsleep 1329258004@qq.com
 * @LastEditTime: 2026-09-17 17:39:34
 * @FilePath: \mas_embedded_threadx\modules\algorithm\serial_leg_solution\observer\observer.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "observer.h"
#include "module_ins.h"
#include "module_bmi088.h"
#include "solution_def.h"
#include "user_lib.h"
#include "motor_damiao.h"
#include "motor_dji.h"
#include "arm_math.h"  

static observer obs;  
const Ins_t           *ins = NULL;
const Bmi088_device_t *bmi = NULL;

Kalman_t KF;

void observer_init(DM_Motor_t *motor_8009[], DJI_Motor_t *motor_3508[])
{
    Kalman_Init(&KF,KF_STATE_DIM, KF_CTRL_DIM, KF_MEASURE_DIM);
    ins = Module_INS_get();
    bmi = Module_BMI088_get_device();
    obs.phi_1_L = motor_8009[3]->base.measure.single_round_angle+phi_1_offset*DEGREE_2_RAD;
    obs.dphi_1_L = motor_8009[3]->base.measure.speed_rad;
    obs.phi_4_L = motor_8009[0]->base.measure.single_round_angle+phi_4_offset*DEGREE_2_RAD;
    obs.dphi_4_L = motor_8009[0]->base.measure.speed_rad;
    obs.phi_1_R = - motor_8009[2]->base.measure.single_round_angle+phi_1_offset*DEGREE_2_RAD;
    obs.dphi_1_R = - motor_8009[2]->base.measure.speed_rad;
    obs.phi_4_R = - motor_8009[1]->base.measure.single_round_angle+phi_4_offset*DEGREE_2_RAD;
    obs.dphi_4_R = - motor_8009[1]->base.measure.speed_rad;
    /* 左轮反馈取负 (两轮镜像安装, 前进时左轮 3508 转速为负) 存疑待修改 */
    /* 3508 反馈为转子量, 轮子位移/速度需 ÷gear_ratio */
    obs.x_L     = -motor_3508[0]->base.measure.total_angle / motor_3508[0]->base.info.gear_ratio * WHEEL_R;
    obs.dx_L    = -motor_3508[0]->base.measure.speed_rad / motor_3508[0]->base.info.gear_ratio * WHEEL_R;
    obs.x_R     = motor_3508[1]->base.measure.total_angle / motor_3508[1]->base.info.gear_ratio * WHEEL_R;
    obs.dx_R    = motor_3508[1]->base.measure.speed_rad / motor_3508[1]->base.info.gear_ratio * WHEEL_R;


    // C板安装: gyro[0]=左, gyro[1]=后, gyro[2]=上
    // INS euler: euler_rad[0]=后, euler_rad[1]=左, euler_rad[2]=上
    // pitch=绕左转, roll=绕前转=-绕后转
    obs.pitch   = -ins->euler_rad[1];  /* 机体俯仰角 (绕左轴, 与 INS 约定相反) */
    obs.dpitch  = -bmi->gyro[0];       /* 机体俯仰角速度 */
    obs.roll    = -ins->euler_rad[0];
    obs.droll   = -bmi->gyro[1];
    obs.yaw     = ins->euler_rad[2];
    obs.initialized = 1;
}

void observer_update(DM_Motor_t *motor_8009[], DJI_Motor_t *motor_3508[])
{
    obs.phi_1_L = motor_8009[3]->base.measure.total_angle+phi_1_offset*DEGREE_2_RAD;
    obs.dphi_1_L = motor_8009[3]->base.measure.speed_rad;
    obs.phi_4_L = motor_8009[0]->base.measure.total_angle+phi_4_offset*DEGREE_2_RAD;
    obs.dphi_4_L = motor_8009[0]->base.measure.speed_rad;
    obs.phi_1_R = - motor_8009[2]->base.measure.total_angle+phi_1_offset*DEGREE_2_RAD;
    obs.dphi_1_R = - motor_8009[2]->base.measure.speed_rad;
    obs.phi_4_R = - motor_8009[1]->base.measure.total_angle+phi_4_offset*DEGREE_2_RAD;
    obs.dphi_4_R = - motor_8009[1]->base.measure.speed_rad;
    /* 左轮反馈取负 (两轮镜像安装, 前进时左轮 3508 转速为负) 存疑待修改 */
    /* 3508 反馈为转子量, 轮子位移/速度需 ÷gear_ratio */
    obs.x_L     = -motor_3508[0]->base.measure.total_angle / motor_3508[0]->base.info.gear_ratio * WHEEL_R;
    obs.dx_L    = -motor_3508[0]->base.measure.speed_rad / motor_3508[0]->base.info.gear_ratio * WHEEL_R;
    obs.x_R     = motor_3508[1]->base.measure.total_angle / motor_3508[1]->base.info.gear_ratio * WHEEL_R;
    obs.dx_R    = motor_3508[1]->base.measure.speed_rad / motor_3508[1]->base.info.gear_ratio * WHEEL_R;

    KF.MeasuredVector[0]=(obs.dx_L+obs.dx_R)/2;
    KF.MeasuredVector[1]=ins->MotionAccel_b[1];
    Kalman_Update(&KF);
    obs.dx=KF.FilteredValue[1];
    obs.x+=obs.dx*0.002;

    obs.pitch   = ins->euler_rad[1];  /* 机体俯仰角 (绕左轴, 与 INS 约定相反) */
    obs.dpitch  = bmi->gyro[0];       /* 机体俯仰角速度 */
    obs.roll    = ins->euler_rad[0];
    obs.droll   = bmi->gyro[1];
    obs.yaw     = ins->euler_rad[2];
    obs.dyaw    = bmi->gyro[2];
}

const observer *observer_get(void)
{
    if (obs.initialized)
        return &obs;
    return NULL;
}



// 状态协方差矩阵 P [x, x_dot]
static float KF_P[KF_STATE_DIM * KF_STATE_DIM] = {
    1.0f,  0.0f,
    0.0f,  1.0f
};

// 状态转移矩阵 F (x(k) = F*x(k-1))
static float KF_F[KF_STATE_DIM * KF_STATE_DIM] = {
    1.0f,  DT,    // x(k) = x(k-1) + x_dot(k-1)*dt
    0.0f,  1.0f   // x_dot(k) = x_dot(k-1)
};

// 量测矩阵 H (z(k) = H*x(k))
static float KF_H[KF_MEASURE_DIM * KF_STATE_DIM] = {
    0.0f,  1.0f,  // v_measure = x_dot (速度对应状态量x_dot)
    1.0f,  0.0f   // a_measure 暂不关联（可根据传感器调整）
};

// 过程噪声协方差矩阵 Q（状态噪声，越小越信任模型）
static float KF_Q[KF_STATE_DIM * KF_STATE_DIM] = {
    0.01f, 0.0f,
    0.0f,  0.1f
};

// 量测噪声协方差矩阵 R（量测噪声，越大越不信任传感器）
static float KF_R[KF_MEASURE_DIM * KF_MEASURE_DIM] = {
    10.0f,   0.0f,
    0.0f,  500.0f
};




/**
 * @brief  卡尔曼滤波初始化
 * @param  kf: 卡尔曼滤波结构体指针
 * @param  state_dim: 状态维度
 * @param  ctrl_dim: 控制维度
 * @param  measure_dim: 量测维度
 * @retval 无
 */
void Kalman_Init(Kalman_t* kf, uint8_t state_dim, uint8_t ctrl_dim, uint8_t measure_dim)
{
    // 初始化维度
    kf->StateDim = state_dim;
    kf->CtrlDim = ctrl_dim;
    kf->MeasureDim = measure_dim;

    // 绑定矩阵指针
    kf->P_data = KF_P;
    kf->F_data = KF_F;
    kf->H_data = KF_H;
    kf->Q_data = KF_Q;
    kf->R_data = KF_R;

    // 初始化滤波后状态值为0
    for(uint8_t i=0; i<state_dim; i++) {
        kf->FilteredValue[i] = 0.0f;
    }

    // 初始化量测向量为0
    for(uint8_t i=0; i<measure_dim; i++) {
        kf->MeasuredVector[i] = 0.0f;
    }
}

/**
 * @brief  卡尔曼滤波更新（预测+校正）
 * @param  kf: 卡尔曼滤波结构体指针
 * @retval 无
 * @note   简化版卡尔曼实现，适配2维状态/2维量测
 */
void Kalman_Update(Kalman_t* kf)
{
    float x_pred[2] = {0};
    x_pred[0] = kf->F_data[0]*kf->FilteredValue[0] + kf->F_data[1]*kf->FilteredValue[1];
    x_pred[1] = kf->F_data[2]*kf->FilteredValue[0] + kf->F_data[3]*kf->FilteredValue[1];
    float P_pred[4] = {0};
    // F*P
    P_pred[0] = kf->F_data[0]*kf->P_data[0] + kf->F_data[1]*kf->P_data[2];
    P_pred[1] = kf->F_data[0]*kf->P_data[1] + kf->F_data[1]*kf->P_data[3];
    P_pred[2] = kf->F_data[2]*kf->P_data[0] + kf->F_data[3]*kf->P_data[2];
    P_pred[3] = kf->F_data[2]*kf->P_data[1] + kf->F_data[3]*kf->P_data[3];
    // (F*P)*F^T + Q
    float P_temp[4] = {
        P_pred[0]*kf->F_data[0] + P_pred[1]*kf->F_data[2],
        P_pred[0]*kf->F_data[1] + P_pred[1]*kf->F_data[3],
        P_pred[2]*kf->F_data[0] + P_pred[3]*kf->F_data[2],
        P_pred[2]*kf->F_data[1] + P_pred[3]*kf->F_data[3]
    };
    for(uint8_t i=0; i<4; i++) {
        P_pred[i] = P_temp[i] + kf->Q_data[i];
    }

    float H_P_Ht[4] = {0};
    // H*P_pred
    H_P_Ht[0] = kf->H_data[0]*P_pred[0] + kf->H_data[1]*P_pred[2];
    H_P_Ht[1] = kf->H_data[0]*P_pred[1] + kf->H_data[1]*P_pred[3];
    H_P_Ht[2] = kf->H_data[2]*P_pred[0] + kf->H_data[3]*P_pred[2];
    H_P_Ht[3] = kf->H_data[2]*P_pred[1] + kf->H_data[3]*P_pred[3];
    // (H*P_pred)*H^T + R
    float S[4] = {
        H_P_Ht[0]*kf->H_data[0] + H_P_Ht[1]*kf->H_data[2] + kf->R_data[0],
        H_P_Ht[0]*kf->H_data[1] + H_P_Ht[1]*kf->H_data[3] + kf->R_data[1],
        H_P_Ht[2]*kf->H_data[0] + H_P_Ht[3]*kf->H_data[2] + kf->R_data[2],
        H_P_Ht[2]*kf->H_data[1] + H_P_Ht[3]*kf->H_data[3] + kf->R_data[3]
    };

    float S_det = S[0]*S[3] - S[1]*S[2];  // 行列式
    float S_inv[4] = {
        S[3]/S_det,  -S[1]/S_det,
        -S[2]/S_det,  S[0]/S_det
    };
    // P_pred*H^T
    float P_Ht[4] = {
        P_pred[0]*kf->H_data[0] + P_pred[1]*kf->H_data[2],
        P_pred[0]*kf->H_data[1] + P_pred[1]*kf->H_data[3],
        P_pred[2]*kf->H_data[0] + P_pred[3]*kf->H_data[2],
        P_pred[2]*kf->H_data[1] + P_pred[3]*kf->H_data[3]
    };
    // 卡尔曼增益 K = P_Ht * S_inv
    float K[4] = {0};
    K[0] = P_Ht[0]*S_inv[0] + P_Ht[1]*S_inv[2];
    K[1] = P_Ht[0]*S_inv[1] + P_Ht[1]*S_inv[3];
    K[2] = P_Ht[2]*S_inv[0] + P_Ht[3]*S_inv[2];
    K[3] = P_Ht[2]*S_inv[1] + P_Ht[3]*S_inv[3];

    float z_Hx[2] = {0};
    z_Hx[0] = kf->MeasuredVector[0] - (kf->H_data[0]*x_pred[0] + kf->H_data[1]*x_pred[1]);
    z_Hx[1] = kf->MeasuredVector[1] - (kf->H_data[2]*x_pred[0] + kf->H_data[3]*x_pred[1]);

    kf->FilteredValue[0] = x_pred[0] + K[0]*z_Hx[0] + K[1]*z_Hx[1];
    kf->FilteredValue[1] = x_pred[1] + K[2]*z_Hx[0] + K[3]*z_Hx[1];

    float I_KH[4] = {
        1 - K[0]*kf->H_data[0] - K[1]*kf->H_data[2],
        0 - K[0]*kf->H_data[1] - K[1]*kf->H_data[3],
        0 - K[2]*kf->H_data[0] - K[3]*kf->H_data[2],
        1 - K[2]*kf->H_data[1] - K[3]*kf->H_data[3]
    };
    for(uint8_t i=0; i<4; i++) {
        kf->P_data[i] = I_KH[i/2*2]*P_pred[i%2] + I_KH[i/2*2+1]*P_pred[2 + i%2];
    }
}

