/*
 * interrupts.h
 *
 *  Created on: 3 sty 2017
 *      Author: Mateusz
 */

#ifndef INTERRUPTS_H_
#define INTERRUPTS_H_

extern MbMotorStruct motor1Struct, motor2Struct;
extern uint32_t adc0_value[1];
extern uint32_t adc1_value[1];
extern volatile uint8_t isTerminalSend, isPid1Switch, isPid2Switch;
extern volatile uint32_t synchro1, synchro2;

void ADC0_Handler(void);
void ADC1_Handler(void);
void PortAIntHandler(void);
void PortBIntHandler(void);
void PortCIntHandler(void);
void PortDIntHandler(void);
void PortFIntHandler(void);
void PWM0LoadIntHandler(void);
void Timer0IntHandler(void);

#endif /* INTERRUPTS_H_ */
