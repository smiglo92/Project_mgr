/*
 * interrupts.h
 *
 *  Created on: 3 sty 2017
 *      Author: Mateusz
 */

#ifndef INTERRUPTS_H_
#define INTERRUPTS_H_

#include "arm_math.h"

extern volatile uint8_t isMeasureZeroCurrent;
extern volatile uint32_t zeroCurrentAdcTab[32];
extern volatile uint32_t zeroCurrentAdc;
extern volatile uint8_t zeroCurrentAdcIter;

extern volatile uint8_t isTerminalSend;
extern volatile uint8_t isPid1Switch;
extern volatile uint8_t isPid2Switch;
extern volatile uint32_t inputVoltage1;
extern volatile uint32_t inputVoltage2;
extern volatile uint32_t temperature1;
extern volatile uint32_t temperature2;

extern MbMotorStruct motor1Struct, motor2Struct;

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
