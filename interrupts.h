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
uint32_t adc0_value[1];
uint32_t adc1_value[2];
volatile uint8_t isTerminalSend;
volatile uint8_t isPid1Switch;
volatile uint8_t isPid2Switch;
volatile uint32_t inputVoltage1;
volatile uint32_t inputVoltage2;
volatile uint32_t temperature1;
volatile uint32_t temperature2;
volatile uint8_t motor1VelocityPidIterator;
volatile uint8_t motor2VelocityPidIterator;
volatile int32_t currentMotor1[8];
volatile int32_t currentMotorAvg;
volatile uint8_t currentMotorIter;

volatile uint8_t isMeasureZeroCurrent;
volatile uint32_t zeroCurrentAdcTab[32];
volatile uint32_t zeroCurrentAdc;
volatile uint8_t zeroCurrentAdcIter;

extern arm_pid_instance_q31 motor1CurrentPid, motor2CurrentPid;
extern arm_pid_instance_q31 motor1VelocityPid, motor2VelocityPid;

//extern volatile uint8_t synchronizationIsEnable[2];				//pierwszy el. tablicy to duzy silnik
//extern volatile uint8_t synchronizationStopAfterIndex[2];		//a drugi to maly silnik
//extern volatile uint8_t synchronizationMethod[2];

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
