#ifndef _CHASSIS_FUNC_H_
#define _CHASSIS_FUNC_H_

#include "solution_def.h"
#include"hero2_def.h"
void chassis_init(Chassis_Ctrl_Cmd_t *chassis_cmd);

void chassis_func(Chassis_Ctrl_Cmd_t *chassis_cmd,GimbalToChassis_cmd_t *cmd);

#endif // _CHASSIS_FUNC_H_