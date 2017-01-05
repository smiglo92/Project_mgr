/*
 * interrupts.h
 *
 *  Created on: 3 sty 2017
 *      Author: Mateusz
 */

#ifndef INTERRUPTS_H_
#define INTERRUPTS_H_

#include <math.h>
#include "arm_math.h"

extern MbMotorStruct motor1Struct, motor2Struct;
extern uint32_t adc0_value[2];
extern uint32_t adc1_value[2];
extern volatile uint8_t isTerminalSend, isPid1Switch, isPid2Switch;
extern volatile uint32_t inputVoltage1, inputVoltage2, temperature1, temperature2;
extern volatile uint8_t isMotor1Synchronization, isMotor2Synchronization;
extern volatile uint8_t motor1VelocityPidIterator, motor2VelocityPidIterator;
volatile int32_t currentMotor1[8];
volatile int32_t currentMotorAvg;
volatile uint8_t currentMotorIter;


extern arm_pid_instance_q31 motor1CurrentPid, motor2CurrentPid;
extern arm_pid_instance_q31 motor1VelocityPid, motor2VelocityPid;

void ADC0_Handler(void);
void ADC1_Handler(void);
void PortAIntHandler(void);
void PortBIntHandler(void);
void PortCIntHandler(void);
void PortDIntHandler(void);
void PortFIntHandler(void);
void PWM0LoadIntHandler(void);
void PWM1LoadIntHandler(void);
void Timer0IntHandler(void);

#endif /* INTERRUPTS_H_ */
