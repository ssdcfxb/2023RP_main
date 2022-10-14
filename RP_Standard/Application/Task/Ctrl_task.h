#ifndef __CTRL_TASK
#define __CTRL_TASK

#include "cmsis_os.h"
#include "main.h"
#include "can.h"
#include "device.h"
#include "driver.h"

#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"

void Start_Ctrl_task(void const * argument);

#endif
