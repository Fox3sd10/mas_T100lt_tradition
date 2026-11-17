#include "length_control.h"
#include "solution_def.h"
#include "kinematics.h"
#include "pid.h"
#include "arm_math.h"
#include "car_state.h"
#include"leg_swing.h"
LegSwing leg_swing_ctrl;


PIDInstance left_swing_pid;
PIDInstance right_swing_pid;


 void leg_swing_init(void)
{
    PID_Init_Config_s pd_config = {
        .Kp       = 15.0f,  // N/m
        .Ki       = 0.001f,
        .Kd       = 0.03f,
        .MaxOut   = 40.0f,   // N
        .DeadBand = 0.0f,
        .Improve  = PID_Derivative_On_Measurement,
    };

    PIDInit(&left_swing_pid,  &pd_config);
    PIDInit(&right_swing_pid, &pd_config);
}
void leg_swing_control(ObsState *obs_state)
{
    const fk *fk_left = kinematics_get_left();
    const fk *fk_right = kinematics_get_right();

    if(!(obs_state->GetUpState&0x40)){//起立完成标志位为0，说明还在起立过程中
        if(obs_state->GetUpState&0x80){//倒地自启标志位为1，说明是倒地自启
            leg_swing_ctrl.left_leg_swing_set=1.5f;
            leg_swing_ctrl.right_leg_swing_set=1.5f;

        }else{//非倒地自启，说明是正常起立
            if((obs_state->GetUpState&0x07)==0x04 ||((obs_state->GetUpState>>3)&0x07)==0x04){
                if((obs_state->GetUpState&0x07)==0x04){
                    leg_swing_ctrl.left_leg_swing_set=-2.0;
                }else{
                    leg_swing_ctrl.left_leg_swing_set=0;
                }
                if(((obs_state->GetUpState>>3)&0x07)==0x04){
                    leg_swing_ctrl.right_leg_swing_set=-2.0;
                }else{
                    leg_swing_ctrl.right_leg_swing_set=0;  
                }
            }else{
                if((obs_state->GetUpState&0x07)==0x02 &&((obs_state->GetUpState>>3)&0x07)==0x02){
                    leg_swing_ctrl.left_leg_swing_set=2.0;
                    leg_swing_ctrl.right_leg_swing_set=0;
                }else{
                    if((obs_state->GetUpState&0x07)==0x01){
                        leg_swing_ctrl.left_leg_swing_set=-2.0;
                    }else if((obs_state->GetUpState&0x07)==0x02){
                        leg_swing_ctrl.left_leg_swing_set=2.0;
                    }
                    if(((obs_state->GetUpState>>3)&0x07)==0x01){
                        leg_swing_ctrl.right_leg_swing_set=-2.0;
                    }else if(((obs_state->GetUpState>>3)&0x07)==0x02){
                        leg_swing_ctrl.right_leg_swing_set=2.0;
                    }
                }
            }
            if((obs_state->GetUpState&0x07)==0x00){
                leg_swing_ctrl.left_leg_swing_set=0.0;}
            if(((obs_state->GetUpState>>3)&0x07)==0x00){
                leg_swing_ctrl.right_leg_swing_set=0.0;}

        }

        leg_swing_ctrl.left_leg_swing_out=PIDCalculate(&left_swing_pid,  fk_left->dtheta,  leg_swing_ctrl.left_leg_swing_set);
        leg_swing_ctrl.right_leg_swing_out=PIDCalculate(&right_swing_pid, fk_right->dtheta, leg_swing_ctrl.right_leg_swing_set); 

        if((obs_state->GetUpState&0x07)==0x01){
            leg_swing_ctrl.left_leg_swing_out-=16*9.8 /2*fk_left->L_0*arm_sin_f32(fk_left->theta);
        }
        if(((obs_state->GetUpState>>3)&0x07)==0x01){
            leg_swing_ctrl.right_leg_swing_out-=16*9.8 /2*fk_right->L_0*arm_sin_f32(fk_right->theta);
        }
        
    }else{
        leg_swing_ctrl.left_leg_swing_out=0.0f;
        leg_swing_ctrl.right_leg_swing_out=0.0f;
    }

}

const  LegSwing *leg_swing_get()
{
    return &leg_swing_ctrl;
}
