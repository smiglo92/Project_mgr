/*
 * mb_Motor.h
 *
 *  Created on: 16 maj 2016
 *      Author: Mateusz
 */

#ifndef MB_MOTOR_H_
#define MB_MOTOR_H_

typedef enum {
    MOTOR1 = 0,
    MOTOR2 = 1,
}MbMotor;

typedef struct {
	int32_t kp;
	int32_t ti;
	int32_t td;
}MbMotorPid;

typedef struct {
	volatile int32_t current;
	int32_t currentTarget;
	int32_t currentTarget2;
	int32_t currentError;
	MbMotorPid currentPid;
	int32_t position;
	int32_t velocity;
	int32_t velocityTarget;
	int32_t velocityError;
	MbMotorPid velocityPid;
	int32_t pwmInput;
}MbMotorStruct;


void mb_Motor_Init(MbMotor motor);
void mb_Motor_Enable(MbMotor motor);
void mb_Motor_Disable(MbMotor motor);
void mb_Motor_Set_Pulse_Width(MbMotor motor, int16_t width);

#endif /* MB_MOTOR_H_ */