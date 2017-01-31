/*
 * mb_PID.c
 *
 *  Created on: 5 sty 2017
 *      Author: Mateusz
 */

#include <stdint.h>
#include <stdbool.h>
#include "mb_Motor.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "mb_Motor.h"
#include "mb_PID.h"

void mb_PID_init(void)
{
//Set current PID variables
/////////////////////////////////////////////////////////////////////////////////////
	motor1Struct.currentPid.kp = 70000;
	motor1Struct.currentPid.ti = 110000;
	motor1Struct.currentPid.td = 0;
	motor1Struct.currentPid.antiwindup = 0;

	motor2Struct.currentPid.kp = 50000;
	motor2Struct.currentPid.ti = 100000;
	motor2Struct.currentPid.td = 0;
	motor2Struct.currentPid.antiwindup = 0;

//Set velocity PID variables
/////////////////////////////////////////////////////////////////////////////////////
	motor1Struct.velocityPid.kp = 600000;
	motor1Struct.velocityPid.ti = 50000;
	motor1Struct.velocityPid.td = 0;
	motor1Struct.velocityPid.antiwindup = 0;

	motor2Struct.velocityPid.kp = 0;
	motor2Struct.velocityPid.ti = 4000;
	motor2Struct.velocityPid.td = 0;
	motor2Struct.velocityPid.antiwindup = 0;
}
