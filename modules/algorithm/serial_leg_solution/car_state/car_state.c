#include "car_state.h"
#include "kinematics.h"
#include <math.h>
#include "observer.h"
#include"solution_def.h"
static car_state_e car_state = STATE_STANDUP;
static uint16_t    stable_cnt = 0;

#define CAR_DONE_THETA  0.30f /* 两腿 |θ|<17° 判站立完成 */
#define CAR_DONE_CYCLES 1     /* 无防抖: 瞬时满足即切 */

void car_state_init(void)
{
    car_state = STATE_STANDUP;
    stable_cnt = 0;
}

car_state_e car_state_get(void)
{
    return car_state;
}

void car_state_update(void)
{
    const fk *l = kinematics_get_left();
    const fk *r = kinematics_get_right();
    if (l == NULL || r == NULL)
        return;

    if (car_state == STATE_STANDUP)
    {
        if (fabsf(l->theta) < CAR_DONE_THETA && fabsf(r->theta) < CAR_DONE_THETA)
        {
            if (++stable_cnt >= CAR_DONE_CYCLES)
            {
                car_state = STATE_BALANCE;
                stable_cnt = 0; /* 切换后清零, 防溢出兜底 */
            }
        }
        else
        {
            stable_cnt = 0; /* 任一腿超窗, 重新攒 */
        }
    }
    /* BALANCE: latch 不回退 (掉线用 car_state_reset); 后续 JUMP/STEP 在此扩展转移 */
}

void car_state_reset(void)
{
    car_state = STATE_STANDUP;
    stable_cnt = 0;
}

void ObsStateInit(ObsState *obs_state)
{
    //规定每个模块的最高位0为未完成/失能，1为完成/使能
    //AllSwitchState:默认全开 第1位为起立使能位，第2位为跳跃使能位，第3位为上楼使能位，剩余5位预留
    //GetUpState: 第8位为倒地自启标志位0为不用，1为用 ，第7位为起立完成标志位
        // 剩余6位，123分别为左腿的状态123，456分别为右腿的状态123
        //同时规定，12只能存在一个1，34只能存在一个1
        //第8位为倒地自启标志位为1时
            //先两个腿转到同一个角度，在同时转动使车体翻过来
    obs_state->GetUpState = 0;
    obs_state->JumpState = 0;
    obs_state->UpStairState = 0;
    obs_state->AllSwitchState = 0XFF;
}
void GetUpUpdate(ObsState *obs_state)
{
    //规定状态1时，角度在-20°~80°之间，状态2时，角度在-20°~-60°之间，状态3时，角度为剩余的在空中的角度
    const fk *l = kinematics_get_left();
    const fk *r = kinematics_get_right();
    const observer *obs = observer_get();
    if((obs_state->AllSwitchState & 0x01)&&(!(obs_state->GetUpState&0x40))) //起立使能位为1
    {
        if(obs->pitch>-1.0&&obs->pitch<1.0) //俯仰角在-60°~60°之间
        {
            obs_state->GetUpState&=0xC0;//清除从倒地自启带来的前6位的值

            if(l->theta<1.39&&l->theta>-0.34){
                obs_state->GetUpState&=0x78;   //清左腿的状态顺带清倒地自救标志位
                obs_state->GetUpState |= 0x01; //左腿状态1
            }
            else if(l->theta<-0.34&&l->theta>-1.0){
                obs_state->GetUpState&=0x78;//清左腿的状态
                obs_state->GetUpState |= 0x02; //左腿状态2
            }
            else{
                obs_state->GetUpState&=0x78;//清左腿的状态
                obs_state->GetUpState |= 0x04; //左腿状态3
            }

            if(r->theta<1.39&&r->theta>-0.34){
                obs_state->GetUpState&=0x47;   //清右腿的状态
                obs_state->GetUpState |= 0x08; //右腿状态1
            }
            else if(r->theta<-0.34&&r->theta>-1.0){
                obs_state->GetUpState&=0x47;//清右腿的状态
                obs_state->GetUpState |= 0x10; //右腿状态2
            }
            else{
                obs_state->GetUpState&=0x47;//清右腿的状态
                obs_state->GetUpState |= 0x20; //右腿状态3
            }

             if(l->theta<0.3&&l->theta>-0.3&&r->theta<0.3&&r->theta>-0.3){
                obs_state->GetUpState&=0x00; //清所有的标志位
                obs_state->GetUpState|=0x40; //起立完成标志位为1
             }

        }else{
            obs_state->GetUpState|=0x80; //倒地自救标志位为1
            if(l->theta-r->theta>0.2){
                obs_state->GetUpState|=0x07;
            }
            if(r->theta-l->theta>0.2){
                obs_state->GetUpState|=0x38;
            }
        }

        
    }
}

// void JumpUpdate(ObsState *obs_state,uint8_t jump_enable)
// {
//     if(jump_enable==0){
//         return ;
//     }else{
//         obs_state->JumpState|=0x80; //起跳标志位为1
//     }

//     //obs_state.JumpState的第1位为状态1：收腿  第二位为状态2：起跳  第三位为状态3：空中收腿 第8位为起跳标志位，第7为位为跳跃完成标志位
//     const fk *l = kinematics_get_left();
//     const fk *r = kinematics_get_right();
//     const observer *obs = observer_get();
//     if((obs_state->AllSwitchState & 0x02)&&(obs_state->JumpState&0x80)){
//         obs_state->JumpState|=0x01; //收腿标志位为1

//     }
// }