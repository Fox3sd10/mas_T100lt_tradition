#include"motion_control.h"
#include"solution_def.h"
#include"pid.h"
#include"observer.h"
#include"LQR_leg.h"
static PIDInstance angle2speed_pid;
static PIDInstance speed2torque_pid;
float march_displacement=0.0f;

void motion_control_init(Chassis_Ctrl_Cmd_t *cmd)
{
    cmd->body_target_dx = 0.0f;
    cmd->turn_cmd       = 0.0f;
    cmd->body_target_dy_out = 0.0f;
    cmd->turn_cmd_out = 0.0f;

    PID_Init_Config_s angle2speed_config = {
        .Kp       = 8.00f,
        .Ki       = 0.0f,
        .Kd       = 1.8f,
        .MaxOut   = 3.0f,   
        .DeadBand = 0.0f,
        .Improve  = PID_Derivative_On_Measurement,
    };
    PIDInit(&angle2speed_pid, &angle2speed_config);

    PID_Init_Config_s speed2torque_config = {
        .Kp       = 1.0f,
        .Ki       = 0.0f,
        .Kd       = 0.0f,
        .MaxOut   = 3.0f,   
        .DeadBand = 0.0f,
        .Improve  = PID_Derivative_On_Measurement,
    };
    PIDInit(&speed2torque_pid, &speed2torque_config);
}
void motion_control_update(Chassis_Ctrl_Cmd_t *cmd,ObsState *obs_state)
{
    if (cmd == NULL) return;
   // const observer *obs = observer_get();
    float speed=PIDCalculate(&angle2speed_pid, cmd->turn_cmd, 0);

    //cmd->turn_cmd_out=PIDCalculate(&speed2torque_pid, obs->dyaw, speed);

    if ((obs_state->GetUpState&0x40)){
            cmd->turn_cmd_out=speed;
    }else{
         cmd->turn_cmd_out=0.0;
    }
    march_displacement+=cmd->body_target_dx*0.002f;
    LQR_calc(-march_displacement, 0.0f, obs_state);

    
}
