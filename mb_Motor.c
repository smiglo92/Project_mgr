/*
 * mb_Motor.c
 *
 *  Created on: 16 maj 2016
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

static const uint32_t mbMotorPeriphPWM[] = {SYSCTL_PERIPH_PWM1, SYSCTL_PERIPH_PWM0};
static const uint32_t mbMotorGPIOPWMPin1[] = {GPIO_PD0_M1PWM0, GPIO_PE4_M0PWM4};
static const uint32_t mbMotorGPIOPWMPin2[] = {GPIO_PD1_M1PWM1, GPIO_PE5_M0PWM5};
static const uint32_t mbMotorGPIOPWMPort[] = {GPIO_PORTD_BASE, GPIO_PORTE_BASE};
static const uint32_t mbMotorGPIOEnablePort[] = {GPIO_PORTD_BASE, GPIO_PORTB_BASE};
static const uint32_t mbMotorGPIOPWMPin[] = {(GPIO_PIN_0 | GPIO_PIN_1), (GPIO_PIN_4 | GPIO_PIN_5)};
static const uint32_t mbMotorGPIOEnablePin[] = {GPIO_PIN_2, GPIO_PIN_1};
static const uint32_t mbMotorPWMBase[] = {PWM1_BASE, PWM0_BASE};
static const uint32_t mbMotorPWMGen[] = {PWM_GEN_0, PWM_GEN_2};
static const uint32_t mbMotorPWMOut1[] = {PWM_OUT_0, PWM_OUT_4};
static const uint32_t mbMotorPWMOut2[] = {PWM_OUT_1, PWM_OUT_5};
static const uint32_t mbMotorPWMOut[] = {(PWM_OUT_0_BIT | PWM_OUT_1_BIT),(PWM_OUT_4_BIT | PWM_OUT_5_BIT)};


void mb_Motor_Init(MbMotor motor)
{
	//ustawia zegar taktujacy PWM na rowny zegarowi glownemu
	SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

	//wlacza zegary portow GPIO
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	if(motor == MOTOR1)
	{
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	}

	if(motor == MOTOR1)
	{
		//Unlock PD7
		HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
		HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
		HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;
	}

	//wlacza zegar PWM
	SysCtlPeripheralEnable(mbMotorPeriphPWM[motor]);
	SysCtlDelay(3);

	//konfiguruje piny generujace sygnal PWM
	GPIOPinConfigure(mbMotorGPIOPWMPin1[motor]);
	GPIOPinConfigure(mbMotorGPIOPWMPin2[motor]);
	GPIOPinTypePWM(mbMotorGPIOPWMPort[motor], mbMotorGPIOPWMPin[motor]);


	PWMGenConfigure(mbMotorPWMBase[motor], mbMotorPWMGen[motor], PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

	PWMGenPeriodSet(mbMotorPWMBase[motor], mbMotorPWMGen[motor], 4000);			//czestotliwosc PWM to 50kHz - 80MHz / 4000 = 20kHz
	PWMPulseWidthSet(mbMotorPWMBase[motor], mbMotorPWMOut1[motor], 1);
	PWMPulseWidthSet(mbMotorPWMBase[motor], mbMotorPWMOut2[motor], 1);

	PWMGenEnable(mbMotorPWMBase[motor], mbMotorPWMGen[motor]);

	PWMGenIntTrigEnable(mbMotorPWMBase[motor], mbMotorPWMGen[motor], PWM_TR_CNT_LOAD);

	GPIOPinTypeGPIOOutput(mbMotorGPIOEnablePort[motor], mbMotorGPIOEnablePin[motor]);
}

void mb_Motor_Enable(MbMotor motor)
{
	PWMOutputState(mbMotorPWMBase[motor], mbMotorPWMOut[motor], true);

	GPIOPinWrite(mbMotorGPIOEnablePort[motor], mbMotorGPIOEnablePin[motor], mbMotorGPIOEnablePin[motor]);

}

void mb_Motor_Disable(MbMotor motor)
{
	PWMOutputState(mbMotorPWMBase[motor], mbMotorPWMOut[motor], false);

	GPIOPinWrite(mbMotorGPIOEnablePort[motor], mbMotorGPIOEnablePin[motor], 0);

}

void mb_Motor_Set_Pulse_Width(MbMotor motor, int16_t width)
{
	if(width < 0)
	{
		PWMPulseWidthSet(mbMotorPWMBase[motor], mbMotorPWMOut1[motor], -width);
		PWMPulseWidthSet(mbMotorPWMBase[motor], mbMotorPWMOut2[motor], 1);
	}
	else if(width > 0)
	{
		PWMPulseWidthSet(mbMotorPWMBase[motor], mbMotorPWMOut1[motor], 1);
		PWMPulseWidthSet(mbMotorPWMBase[motor], mbMotorPWMOut2[motor], width);
	}
	else
	{
		PWMPulseWidthSet(mbMotorPWMBase[motor], mbMotorPWMOut1[motor], 1);
		PWMPulseWidthSet(mbMotorPWMBase[motor], mbMotorPWMOut2[motor], 1);
	}
}
