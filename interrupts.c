/*
 * interrupts.c
 *
 *  Created on: 3 sty 2017
 *      Author: Mateusz
 */
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

#include <math.h>
#include "arm_math.h"

#include "mb_Motor.h"
#include "mb_LED.h"

#include "interrupts.h"

void PWM0LoadIntHandler(void)
{
	PWMGenIntClear(PWM0_BASE, PWM_GEN_2, PWM_INT_CNT_LOAD);

	PWMSyncTimeBase(PWM1_BASE, PWM_GEN_0_BIT);
	mb_LED_On(LED1);
	isPid2Switch = 1;

}

void PWM1LoadIntHandler(void)
{
//	volatile uint8_t i;

	PWMGenIntClear(PWM1_BASE, PWM_GEN_0, PWM_INT_CNT_LOAD);

//    ADCSequenceDataGet(ADC0_BASE, 0, adc0_value);

//    motor1Struct.current = (int32_t)adc0_value[0];
//    currentMotor1[currentMotorIter] = ((int32_t)(adc0_value[0])-2045)*100000/14978;
//    currentMotorIter++;
//    if(currentMotorIter == 8)
//    	currentMotorIter = 0;
//    currentMotorAvg = 0;
//    for(i = 0; i < 8; i++)
//    	currentMotorAvg += currentMotor1[i];
//
//    motor1Struct.current = currentMotorAvg >> 3;
//
//    inputVoltage1 = adc0_value[1]*182380/12467;
//    inputVoltage2 = inputVoltage1 % 1000;
//    inputVoltage1 = inputVoltage1 / 1000;

	mb_LED_On(LED1);
	isPid1Switch = 1;

}

void Timer0IntHandler(void)
{
	// Clear the timer interrupt
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	isTerminalSend = 1;
}


void ADC0_Handler(void) {

	volatile uint8_t i;

    while (!ADCIntStatus(ADC0_BASE, 0, false));

    ADCIntClear(ADC0_BASE, 0);
    ADCSequenceDataGet(ADC0_BASE, 0, adc0_value);

    if (isMeasureZeroCurrent)
    {
    	zeroCurrentAdcTab[zeroCurrentAdcIter] = adc0_value[0];
    	zeroCurrentAdcIter++;
    }
    else
//    	motor1Struct.current = ((int32_t)(adc0_value[0])-(int32_t)zeroCurrentAdc)*100000/14978;
    currentMotor1[currentMotorIter] = ((int32_t)(adc0_value[0])-2045)*100000/14978;
    currentMotorIter++;
    if(currentMotorIter == 8)
    	currentMotorIter = 0;
    currentMotorAvg = 0;
    for(i = 0; i < 8; i++)
    	currentMotorAvg += currentMotor1[i];

    motor1Struct.current = currentMotorAvg >> 3;

//    isPid1Switch = 1;

}

void ADC1_Handler(void) {

    while (!ADCIntStatus(ADC1_BASE, 0, false));

    ADCIntClear(ADC1_BASE, 0);
    ADCSequenceDataGet(ADC1_BASE, 0, adc1_value);


    motor2Struct.current = (int32_t)(adc1_value[0])*1078605/4037670 - 4;
    if(motor2Struct.current < 0)
    	motor2Struct.current = 0;
    motor2Struct.current *= QEIDirectionGet(QEI1_BASE);

    inputVoltage1 = adc1_value[1]*182380/12467;
    inputVoltage2 = inputVoltage1 % 1000;
    inputVoltage1 = inputVoltage1 / 1000;

//    temperature1 = 14750 - 24750 * adc1_value[1] / 4095;
//    temperature2 = temperature1 % 100;
//    temperature1 = temperature1 / 100;

//    isPid2Switch = 1;

}

void PortAIntHandler(void)
{
	uint32_t status = 0;

	status = GPIOIntStatus(GPIO_PORTA_BASE, true);
	GPIOIntClear(GPIO_PORTA_BASE, status);

	if(status & GPIO_INT_PIN_2 == GPIO_INT_PIN_2)
	{

	}

	if(status & GPIO_INT_PIN_3 == GPIO_INT_PIN_3)
	{

	}

	if(status & GPIO_INT_PIN_4 == GPIO_INT_PIN_4)
	{

	}

	if(status & GPIO_INT_PIN_5 == GPIO_INT_PIN_5)
	{

	}
}

void PortBIntHandler(void)
{
	uint32_t status = 0;

	status = GPIOIntStatus(GPIO_PORTB_BASE, true);
	GPIOIntClear(GPIO_PORTB_BASE, status);

	if(status & GPIO_INT_PIN_0 == GPIO_INT_PIN_0)
	{

	}

	if(status & GPIO_INT_PIN_3 == GPIO_INT_PIN_3)
	{

	}
}

void PortCIntHandler(void)
{
	uint32_t status = 0;

	status = GPIOIntStatus(GPIO_PORTC_BASE, true);
	GPIOIntClear(GPIO_PORTC_BASE, status);

	if(status & GPIO_INT_PIN_4 == GPIO_INT_PIN_4)
	{

	}
	if(status & GPIO_INT_PIN_7 == GPIO_INT_PIN_7)
	{

	}
}

void PortDIntHandler(void)
{
	uint32_t status = 0;

	status = GPIOIntStatus(GPIO_PORTD_BASE, true);
	GPIOIntClear(GPIO_PORTD_BASE, status);

	if(status & GPIO_INT_PIN_3 == GPIO_INT_PIN_3)
	{

	}
}

void PortFIntHandler(void)
{
	uint32_t status = 0;

	status = GPIOIntStatus(GPIO_PORTF_BASE, true);
	GPIOIntClear(GPIO_PORTF_BASE, status);

	if(status & GPIO_INT_PIN_1 == GPIO_INT_PIN_1)
	{

	}

	if(status & GPIO_INT_PIN_3 == GPIO_INT_PIN_3)
	{

	}
	if((status & GPIO_INT_PIN_4) == GPIO_INT_PIN_4)
	{

	}
}
