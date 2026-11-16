#ifndef LENGTH_CONTROL_H
#define LENGTH_CONTROL_H

#include "solution_def.h"

void length_init(void);
void length_control(ObsState *obs_state);
const Length *Length_get(void);

#endif // 