

#pragma once

#include "../platform.h"
#include "../common/fifo.h"

extern uint16_t opticalflowCtrlTimer, opticalflowMonitorTimer;	
extern int32_t opticalflowPID[2];

extern float optGyro[2];

void opflow_init(void);
void opticalflow_ctrl(void);

