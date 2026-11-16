#ifndef LENGTH_SWING_H
#define LENGTH_SWING_H

#include "solution_def.h"

 void leg_swing_init(void);
void leg_swing_control(ObsState *obs_state);
const  LegSwing *leg_swing_get();
#endif // 