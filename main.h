/*
 * main.h
 *
 *  Created on: 5 sty 2017
 *      Author: Mateusz
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/qei.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "utils/uartstdio.h"
#include "mb_LED.h"
#include "mb_Motor.h"
#include "mb_PID.h"
#include "interrupts.h"
#include "utils/uartstdio.c"
#include <string.h>
#include <math.h>
#include "arm_math.h"


extern void ADC0_Handler(void);
extern void ADC1_Handler(void);
extern void PortAIntHandler(void);
extern void PortBIntHandler(void);
extern void PortCIntHandler(void);
extern void PortDIntHandler(void);
extern void PortFIntHandler(void);
extern void PWM0LoadIntHandler(void);
extern void PWM1LoadIntHandler(void);
extern void Timer0IntHandler(void);

extern uint32_t adc0_value[1];
extern uint32_t adc1_value[2];

extern volatile uint8_t isTerminalSend;
extern volatile uint8_t isPid1Switch;
extern volatile uint8_t isPid2Switch;
extern volatile uint32_t inputVoltage1;
extern volatile uint32_t inputVoltage2;
extern volatile uint32_t temperature1;
extern volatile uint32_t temperature2;
extern volatile uint8_t motor1VelocityPidIterator;
extern volatile uint8_t motor2VelocityPidIterator;

extern volatile uint8_t isMeasureZeroCurrent;
extern volatile uint32_t zeroCurrentAdcTab[32];
extern volatile uint32_t zeroCurrentAdc;
extern volatile uint8_t zeroCurrentAdcIter;

#endif /* MAIN_H_ */
