
#pragma once

#include "../platform.h"

#define VELOCITY_SPEEDDOWN_MAX -2000
#define VELOCITY_SPEEDUP_MAX   +2000

extern uint8_t altitudeCtrlTimer;

extern int32_t BaroAlt;
extern int32_t  EstAlt; 

extern int32_t  accSumZ[3]; 
extern float    accSumZTimer;
extern uint16_t accSumZCount; 

extern int32_t BaroPID;

extern float velGlobal[3], distanceGlobal[3];

//void altitude_baro_update(void);
void altitude_ctrl(void);

