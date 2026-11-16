#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "solution_def.h"

void motion_control_init(Chassis_Ctrl_Cmd_t *cmd);
void motion_control_update(Chassis_Ctrl_Cmd_t *cmd);

#endif // 