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

typedef enum {
	LEFT = 0,
	RIGHT = 1,
}MbMotorDirection;

typedef struct {
	int32_t kp;
	int32_t ti;
	int32_t td;
	uint8_t antiwindup;
}MbMotorPid;

typedef struct {
	volatile uint8_t isEnable;
	volatile uint8_t stopAfterIndex;
	volatile uint8_t method;
	uint16_t velocity;
}MbMotorSynchronization;

typedef struct {
	volatile int32_t current;
	volatile int32_t currentTarget;
	volatile int32_t currentTargetVelocityPid;
	volatile int32_t currentTargetVelocityPidPrevious;
	volatile int32_t currentTargetVelocityPidUnlimited;
	volatile int32_t currentError;
	MbMotorPid currentPid;
	volatile int32_t position;
	volatile int32_t velocity;
	volatile int32_t velocityTarget;
	volatile int32_t velocityError;
	MbMotorPid velocityPid;
	volatile int32_t pwmInput;
	volatile int32_t pwmInputPrevious;
	volatile int32_t pwmInputUnlimited;
	volatile MbMotorDirection direction;
	MbMotorSynchronization synchronization;
}MbMotorStruct;


/* Init peripherals for motor support
 * arg:
 * 		- motor: MOTOR1 or MOTOR2*/
void mb_Motor_Init(MbMotor motor);


/* Enable motor
 * arg:
 * 		- motor: MOTOR1 or MOTOR2*/
void mb_Motor_Enable(MbMotor motor);


/* Disable motor
 * arg:
 * 		- motor: MOTOR1 or MOTOR2*/
void mb_Motor_Disable(MbMotor motor);


/* Set motor PWM pulse width (if negative set PWM at second output)
 * arg:
 * 		- motor: MOTOR1 or MOTOR2
 * 		- width: -4000 <= width <= 4000*/
void mb_Motor_Set_Pulse_Width(MbMotor motor, int16_t width);


/* Enable motor synchronization
 * arg:
 * 		- motor: MOTOR1 or MOTOR2*/
void mb_Motor_Synchro_ON(MbMotor motor);


/* Returns a state of home switch
 * arg:
 * 		- motor: MOTOR1 or MOTOR2
 * return:
 * 		- 0 if low state
 * 		- 1 if high state*/
uint8_t mb_Motor_Synchro_HomeSwitch(MbMotor motor);


/* Set motor synchronization velocity
 * arg:
 * 		- motor: MOTOR1 or MOTOR2*/
void mb_Motor_Synchro_setVelocity(MbMotor motor, uint16_t velocity);

#endif /* MB_MOTOR_H_ */
