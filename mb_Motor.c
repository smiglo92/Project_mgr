/*
 * mb_Motor.c
 *
 *  Created on: 16 maj 2016
 *      Author: Mateusz
 */

/* MOTOR1:
 * D0(M1PWM0), D1(M1PWM1) - PWMA, PWMB
 * D2 - - - - - - - - - - - ENABLE
 * E3(AIN0) - - - - - - - - CURRENT SENSOR
 * F3, C4 - - - - - - - - - ENDSTOPS(LEFT, RIGHT)
 * B3 - - - - - - - - - - - HOME SWITCH
 * D6(PhA0), D7(PhB0), F4 - ENCODER(A, B, I)
 * B0, F1 - - - - - - - - - WARNING SIGNALS(ACTIVE LOW)*/

/* MOTOR2:
 * E4(M0PWM4), E5(M0PWM5) - PWMA, PWMB
 * B1 - - - - - - - - - - - ENABLE
 * E2(AIN1) - - - - - - - - CURRENT SENSOR
 * A2, A4 - - - - - - - - - ENDSTOPS(LEFT, RIGHT)
 * A3 - - - - - - - - - - - HOME SWITCH
 * C5(PhA1), C6(PhB1), C7 - ENCODER(A, B, I)*/

/* COMMON:
 * A5, D3 - - - - - - - - - SAFETY INPUTS
 * E1(AIN2) - - - - - - - - INPUT VOLTAGE MEASURE*/

#include <stdint.h>
#include <stdbool.h>
#include "mb_Motor.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"
#include "interrupts.h"

static const uint32_t mbMotorPeriphPWM[] = {SYSCTL_PERIPH_PWM1, SYSCTL_PERIPH_PWM0};
static const uint32_t mbMotorGPIOPWMPin1[] = {GPIO_PD0_M1PWM0, GPIO_PE4_M0PWM4};
static const uint32_t mbMotorGPIOPWMPin2[] = {GPIO_PD1_M1PWM1, GPIO_PE5_M0PWM5};
static const uint32_t mbMotorGPIOPWMPort[] = {GPIO_PORTD_BASE, GPIO_PORTE_BASE};
static const uint32_t mbMotorGPIOEnablePort[] = {GPIO_PORTD_BASE, GPIO_PORTB_BASE};
static const uint32_t mbMotorGPIOPWMPin[] = {(GPIO_PIN_0 | GPIO_PIN_1),
		(GPIO_PIN_4 | GPIO_PIN_5)};
static const uint32_t mbMotorGPIOEnablePin[] = {GPIO_PIN_2, GPIO_PIN_1};
static const uint32_t mbMotorPWMBase[] = {PWM1_BASE, PWM0_BASE};
static const uint32_t mbMotorPWMGen[] = {PWM_GEN_0, PWM_GEN_2};
static const uint32_t mbMotorPWMOut1[] = {PWM_OUT_0, PWM_OUT_4};
static const uint32_t mbMotorPWMOut2[] = {PWM_OUT_1, PWM_OUT_5};
static const uint32_t mbMotorPWMOut[] = {(PWM_OUT_0_BIT | PWM_OUT_1_BIT),
		(PWM_OUT_4_BIT | PWM_OUT_5_BIT)};
static MbMotorStruct* const mbMotorStructureTab[] = {&motor1Struct, &motor2Struct};

/////////////////////////////////////////////////////////////////////////////////////////
void mb_Motor_Init(MbMotor motor)
{
	//PWM clock is equal main clock
	SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

	//enable GPIO clocks
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

	//enable PWM clock
	SysCtlPeripheralEnable(mbMotorPeriphPWM[motor]);
	SysCtlDelay(3);

	//PWM pins configure
	GPIOPinConfigure(mbMotorGPIOPWMPin1[motor]);
	GPIOPinConfigure(mbMotorGPIOPWMPin2[motor]);
	GPIOPinTypePWM(mbMotorGPIOPWMPort[motor], mbMotorGPIOPWMPin[motor]);

	//PWM generator configure
	PWMGenConfigure(mbMotorPWMBase[motor], mbMotorPWMGen[motor],
			PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
	/*PWM frequency - 80MHz / 4000 = 20kHz*/
	PWMGenPeriodSet(mbMotorPWMBase[motor], mbMotorPWMGen[motor], 4000);
	PWMPulseWidthSet(mbMotorPWMBase[motor], mbMotorPWMOut1[motor], 1);
	PWMPulseWidthSet(mbMotorPWMBase[motor], mbMotorPWMOut2[motor], 1);
	PWMGenEnable(mbMotorPWMBase[motor], mbMotorPWMGen[motor]);

	//MOTOR2 PWM generate trigger for ADC
	if (motor == MOTOR2)
		PWMGenIntTrigEnable(mbMotorPWMBase[motor], mbMotorPWMGen[motor],
				PWM_TR_CNT_LOAD);

	//ENABLE pin as output
	GPIOPinTypeGPIOOutput(mbMotorGPIOEnablePort[motor], mbMotorGPIOEnablePin[motor]);
}


/////////////////////////////////////////////////////////////////////////////////////////
void mb_Motor_Enable(MbMotor motor)
{
	//PWM output enable
	PWMOutputState(mbMotorPWMBase[motor], mbMotorPWMOut[motor], true);

	//ENABLE pin in high state
	GPIOPinWrite(mbMotorGPIOEnablePort[motor], mbMotorGPIOEnablePin[motor],
			mbMotorGPIOEnablePin[motor]);
}


/////////////////////////////////////////////////////////////////////////////////////////
void mb_Motor_Disable(MbMotor motor)
{
	//PWM output disable
	PWMOutputState(mbMotorPWMBase[motor], mbMotorPWMOut[motor], false);

	//ENABLE pin in low state
	GPIOPinWrite(mbMotorGPIOEnablePort[motor], mbMotorGPIOEnablePin[motor], 0);
}


/////////////////////////////////////////////////////////////////////////////////////////
void mb_Motor_Set_Pulse_Width(MbMotor motor, int16_t width)
{
	//if width is negative, set PWM signal at first output
	if(width < 0)
	{
		PWMPulseWidthSet(mbMotorPWMBase[motor], mbMotorPWMOut1[motor], -width);
		PWMPulseWidthSet(mbMotorPWMBase[motor], mbMotorPWMOut2[motor], 1);
	}

	//if width is positive, set PWM signal at second output
	else if(width > 0)
	{
		PWMPulseWidthSet(mbMotorPWMBase[motor], mbMotorPWMOut1[motor], 1);
		PWMPulseWidthSet(mbMotorPWMBase[motor], mbMotorPWMOut2[motor], width);
	}

	//if width is equal zero, set low state signal at both outputs
	else
	{
		PWMPulseWidthSet(mbMotorPWMBase[motor], mbMotorPWMOut1[motor], 1);
		PWMPulseWidthSet(mbMotorPWMBase[motor], mbMotorPWMOut2[motor], 1);
	}
}


/////////////////////////////////////////////////////////////////////////////////////////
void initExternalInterrupts(void)
{
	IntRegister(INT_GPIOA_TM4C123, PortAIntHandler);
	IntEnable(INT_GPIOA_TM4C123);
	IntRegister(INT_GPIOB_TM4C123, PortBIntHandler);
	IntEnable(INT_GPIOB_TM4C123);
	IntRegister(INT_GPIOC_TM4C123, PortCIntHandler);
	IntEnable(INT_GPIOC_TM4C123);
	IntRegister(INT_GPIOD_TM4C123, PortDIntHandler);
	IntEnable(INT_GPIOD_TM4C123);
	IntRegister(INT_GPIOF_TM4C123, PortFIntHandler);
	IntEnable(INT_GPIOF_TM4C123);

	//endstops (A2, A4, C4, F3)
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2);
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
	GPIOIntDisable(GPIO_PORTA_BASE, GPIO_INT_PIN_2);
	GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_2);
	GPIOIntRegister(GPIO_PORTA_BASE, PortAIntHandler);
	GPIOIntEnable(GPIO_PORTA_BASE, GPIO_INT_PIN_2);

	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_4);
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
	GPIOIntDisable(GPIO_PORTA_BASE, GPIO_INT_PIN_4);
	GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_4);
	GPIOIntEnable(GPIO_PORTA_BASE, GPIO_INT_PIN_4);

	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_3);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_FALLING_EDGE);
	GPIOIntDisable(GPIO_PORTF_BASE, GPIO_INT_PIN_3);
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_3);
	GPIOIntRegister(GPIO_PORTF_BASE, PortFIntHandler);
	GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_3);

	GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4);
	GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
	GPIOIntDisable(GPIO_PORTC_BASE, GPIO_INT_PIN_4);
	GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_4);
	GPIOIntRegister(GPIO_PORTC_BASE, PortCIntHandler);
	GPIOIntEnable(GPIO_PORTC_BASE, GPIO_INT_PIN_4);


	//synchronization (A3, B3)
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_3);
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_BOTH_EDGES);
	GPIOIntDisable(GPIO_PORTA_BASE, GPIO_INT_PIN_3);
	GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_3);
	GPIOIntEnable(GPIO_PORTA_BASE, GPIO_INT_PIN_3);

	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_3);
	GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_BOTH_EDGES);
	GPIOIntDisable(GPIO_PORTB_BASE, GPIO_INT_PIN_3);
	GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_3);
	GPIOIntRegister(GPIO_PORTB_BASE, PortBIntHandler);
	GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_3);


	//safety inputs (A5, D3)
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_5);
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
	GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_BOTH_EDGES);
	GPIOIntDisable(GPIO_PORTA_BASE, GPIO_INT_PIN_5);
	GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_5);
	GPIOIntEnable(GPIO_PORTA_BASE, GPIO_INT_PIN_5);

	GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_3);
	GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
	GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_BOTH_EDGES);
	GPIOIntDisable(GPIO_PORTD_BASE, GPIO_INT_PIN_3);
	GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_3);
	GPIOIntRegister(GPIO_PORTD_BASE, PortDIntHandler);
	GPIOIntEnable(GPIO_PORTD_BASE, GPIO_INT_PIN_3);

	//MOTOR1 Overtemperature and fault warning
	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0);
	GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_BOTH_EDGES);
	GPIOIntDisable(GPIO_PORTB_BASE, GPIO_INT_PIN_0);
	GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_0);
	GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_0);

	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_1);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_BOTH_EDGES);
	GPIOIntDisable(GPIO_PORTF_BASE, GPIO_INT_PIN_1);
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_1);
	GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_1);

	//Encoder index pulses (F4, C7)
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
	GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_RISING_EDGE);
	GPIOIntDisable(GPIO_PORTF_BASE, GPIO_INT_PIN_4);
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_4);
	GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_4);

	GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_7);
	GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
	GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_RISING_EDGE);
	GPIOIntDisable(GPIO_PORTC_BASE, GPIO_INT_PIN_7);
	GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_7);
	GPIOIntEnable(GPIO_PORTC_BASE, GPIO_INT_PIN_7);

}


/////////////////////////////////////////////////////////////////////////////////////////
void mb_Motor_Synchro_ON(MbMotor motor)
{
	mbMotorStructureTab[motor]->synchronization.isEnable = 1;

	if(mbMotorStructureTab[motor]->synchronization.method & 0b100 == 0)
		if((mb_Motor_Synchro_HomeSwitch(motor)) &&
				(mbMotorStructureTab[motor]->synchronization.method & 0b10 == 0b10))
		{
			/*Starts in right direction*/
			mbMotorStructureTab[motor]->velocityTarget =
					mbMotorStructureTab[motor]->synchronization.velocity;
			mbMotorStructureTab[motor]->direction = RIGHT;
		}
		else
		{
			/*Starts in left direction*/
			mbMotorStructureTab[motor]->velocityTarget =
					(-1) * mbMotorStructureTab[motor]->synchronization.velocity;
			mbMotorStructureTab[motor]->direction = LEFT;
		}

	if(mbMotorStructureTab[motor]->synchronization.method & 0b100 == 0b100)
		if((mb_Motor_Synchro_HomeSwitch(motor)) &&
				(mbMotorStructureTab[motor]->synchronization.method & 0b10 == 0b0))
		{
			/*Starts in left direction*/
			mbMotorStructureTab[motor]->velocityTarget =
					(-1) * mbMotorStructureTab[motor]->synchronization.velocity;
			mbMotorStructureTab[motor]->direction = LEFT;
		}
		else
		{
			/*Starts in right direction*/
			mbMotorStructureTab[motor]->velocityTarget =
					mbMotorStructureTab[motor]->synchronization.velocity;
			mbMotorStructureTab[motor]->direction = RIGHT;
		}
}


/////////////////////////////////////////////////////////////////////////////////////////
uint8_t mb_Motor_Synchro_HomeSwitch(MbMotor motor)
{
	uint8_t state;
	if(motor == MOTOR1)
		state = (uint8_t)GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_3);
	else
		state = (uint8_t)GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3);
	if(state)
		state = 1;
	return state;
}


/////////////////////////////////////////////////////////////////////////////////////////
void mb_Motor_Synchro_setVelocity(MbMotor motor, uint16_t velocity)
{
	if(velocity < 50)
		mbMotorStructureTab[motor]->synchronization.velocity = 50;
	else if(velocity > 2000)
		mbMotorStructureTab[motor]->synchronization.velocity = 2000;
	else
		mbMotorStructureTab[motor]->synchronization.velocity = velocity;
}


