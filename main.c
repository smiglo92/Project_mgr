
#include "main.h"

MbMotorStruct motor1Struct, motor2Struct;

//TODO - SYNCHRONIZACJA

arm_pid_instance_q31 motor1CurrentPid, motor2CurrentPid;

arm_pid_instance_q31 motor1VelocityPid, motor2VelocityPid;

int main()
{
	motor1VelocityPidIterator = 0;
	motor2VelocityPidIterator = 10;

	//Set the system clock to 80Mhz
	/////////////////////////////////////////////////////////////////////////////////////
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	//Enable motors
	/////////////////////////////////////////////////////////////////////////////////////
	IntMasterEnable();

	IntRegister(INT_PWM0_2_TM4C123, PWM0LoadIntHandler);
	IntEnable(INT_PWM0_2_TM4C123);

	IntRegister(INT_PWM1_0_TM4C123, PWM1LoadIntHandler);
	IntEnable(INT_PWM1_0_TM4C123);

	mb_Motor_Init(MOTOR1);
	mb_Motor_Init(MOTOR2);

	PWMGenIntTrigDisable(PWM0_BASE, PWM_GEN_2, PWM_INT_CNT_LOAD);
	PWMGenIntClear(PWM0_BASE, PWM_GEN_2, PWM_INT_CNT_LOAD);
	PWMGenIntRegister(PWM0_BASE, PWM_GEN_2, PWM0LoadIntHandler);
	PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_2, PWM_INT_CNT_LOAD);
	PWMIntEnable(PWM0_BASE, PWM_INT_GEN_2);

	PWMGenIntTrigDisable(PWM1_BASE, PWM_GEN_0, PWM_INT_CNT_LOAD);
	PWMGenIntClear(PWM1_BASE, PWM_GEN_0, PWM_INT_CNT_LOAD);
	PWMGenIntRegister(PWM1_BASE, PWM_GEN_0, PWM1LoadIntHandler);
	PWMGenIntTrigEnable(PWM1_BASE, PWM_GEN_0, PWM_INT_CNT_LOAD);
	PWMIntEnable(PWM1_BASE, PWM_INT_GEN_0);

	mb_LED_Init(LED1);
	mb_LED_Init(LED2);

	mb_Motor_Enable(MOTOR1);
	mb_Motor_Enable(MOTOR2);

	mb_Motor_Set_Pulse_Width(MOTOR1, 500);
	mb_Motor_Set_Pulse_Width(MOTOR2, 500);

	mb_LED_On(LED1);
	mb_LED_On(LED2);

	mb_PID_init();

	initExternalInterrupts();

	//Enable ADC to current measure and input voltage
	/////////////////////////////////////////////////////////////////////////////////////
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlDelay(3);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    SysCtlDelay(3);

	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

	ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_ALWAYS, 1);
	ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_PWM_MOD0 | ADC_TRIGGER_PWM2, 0);

	ADCHardwareOversampleConfigure(ADC0_BASE, 32);
	ADCHardwareOversampleConfigure(ADC1_BASE, 0);

	ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0|ADC_CTL_IE|ADC_CTL_END);

	ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC1_BASE, 0, 1, ADC_CTL_CH2|ADC_CTL_IE|ADC_CTL_END);

	ADCSequenceEnable(ADC0_BASE, 0);
	ADCSequenceEnable(ADC1_BASE, 0);

//	IntEnable(INT_ADC0SS1_TM4C123);
//	IntEnable(INT_ADC1SS1_TM4C123);

//	IntRegister(INT_ADC0SS1_TM4C123, ADC0_Handler);
//	IntEnable(INT_ADC0SS1_TM4C123);
//	IntRegister(INT_ADC1SS1_TM4C123, ADC1_Handler);
//	IntEnable(INT_ADC1SS1_TM4C123);

	ADCIntDisable(ADC0_BASE, 0);
	ADCIntClear(ADC0_BASE, 0);
	ADCIntRegister(ADC0_BASE, 0, ADC0_Handler);
	ADCIntEnable(ADC0_BASE, 0);

	ADCIntDisable(ADC1_BASE, 0);
	ADCIntClear(ADC1_BASE, 0);
	ADCIntRegister(ADC1_BASE, 0, ADC1_Handler);
	ADCIntEnable(ADC1_BASE, 0);

	//Enable UART
	/////////////////////////////////////////////////////////////////////////////////////
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	UARTStdioConfig(0, 115200, 16000000);

	//Enable timer for console messages sending
	/////////////////////////////////////////////////////////////////////////////////////
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_A_PERIODIC_UP);

//	TimerLoadSet(TIMER0_BASE, TIMER_A, 1000);
	TimerLoadSet(TIMER0_BASE, TIMER_A, (SysCtlClockGet()/1000) -1);

	TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0IntHandler);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	IntEnable(INT_TIMER0A);

	TimerEnable(TIMER0_BASE, TIMER_A);

	//Enable QEI Peripherals
	/////////////////////////////////////////////////////////////////////////////////////
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0));

	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI1));

	GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
//	GPIOPinTypeQEI(GPIO_PORTF_BASE, GPIO_PIN_4);
	GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6);

	GPIOPinConfigure(GPIO_PD6_PHA0);
	GPIOPinConfigure(GPIO_PD7_PHB0);
//	GPIOPinConfigure(GPIO_PF4_IDX0);
	GPIOPinConfigure(GPIO_PC5_PHA1);
	GPIOPinConfigure(GPIO_PC6_PHB1);

	QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B|QEI_CONFIG_NO_RESET|QEI_CONFIG_QUADRATURE|QEI_CONFIG_NO_SWAP), 2000000);
	QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B|QEI_CONFIG_NO_RESET|QEI_CONFIG_QUADRATURE|QEI_CONFIG_NO_SWAP), 2000000);

	QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, 80000-1);
	QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_1, 80000-1);

	QEIVelocityEnable(QEI0_BASE);
	QEIVelocityEnable(QEI1_BASE);

	QEIEnable(QEI0_BASE);
	QEIEnable(QEI1_BASE);

	//Write variables
	/////////////////////////////////////////////////////////////////////////////////////
	motor1CurrentPid.Kp = motor1Struct.currentPid.kp;
	motor1CurrentPid.Ki = motor1Struct.currentPid.ti;
	motor1CurrentPid.Kd = motor1Struct.currentPid.td;

	arm_pid_init_q31(&motor1CurrentPid, 1);

	motor1CurrentPid.state[2] = 100;
	motor1Struct.currentTarget = 80;
	motor1Struct.pwmInput = 100;
	motor1Struct.currentError = 0;

	motor2CurrentPid.Kp = motor2Struct.currentPid.kp;
	motor2CurrentPid.Ki = motor2Struct.currentPid.ti;
	motor2CurrentPid.Kd = motor2Struct.currentPid.td;

	arm_pid_init_q31(&motor2CurrentPid, 1);

	motor2CurrentPid.state[2] = 200;
	motor2Struct.currentTarget = 20;
	motor2Struct.pwmInput = 200;
	motor2Struct.currentError = 0;

	motor1VelocityPid.Kp = motor1Struct.velocityPid.kp;
	motor1VelocityPid.Ki = motor1Struct.velocityPid.ti;
	motor1VelocityPid.Kd = motor1Struct.velocityPid.td;

	arm_pid_init_q31(&motor1VelocityPid, 1);

	motor1VelocityPid.state[2] = 0;
	motor1Struct.velocityTarget = 500;
	motor1Struct.velocityError = 0;

	motor2VelocityPid.Kp = motor2Struct.velocityPid.kp;
	motor2VelocityPid.Ki = motor2Struct.velocityPid.ti;
	motor2VelocityPid.Kd = motor2Struct.velocityPid.td;

	arm_pid_init_q31(&motor2VelocityPid, 1);

	motor2VelocityPid.state[2] = 0;
	motor2Struct.velocityTarget = 3000;
	motor2Struct.velocityError = 0;

	while (1)
	{


		if(isPid2Switch)
		{
//			mb_LED_Switch(LED1);

			if (motor2VelocityPidIterator == 0 && isMotor2Synchronization)
			{
				motor2Struct.velocity = (QEIVelocityGet(QEI1_BASE) * 30) * (QEIDirectionGet(QEI1_BASE));
				motor2Struct.velocityError = (motor2Struct.velocityTarget - motor2Struct.velocity) << 10;

				motor2Struct.currentTarget2 = arm_pid_q31(&motor2VelocityPid, motor2Struct.velocityError);

				if(motor2Struct.currentTarget2 > 150)
				{
					motor2Struct.currentTarget2 = 150;
					motor2VelocityPid.state[2] = 150;
				}
				if(motor2Struct.currentTarget2 < -150)
				{
					motor2Struct.currentTarget2 = -150;
					motor2VelocityPid.state[2] = -150;
				}
			}

//			Zrobiæ sprawdzanie zadanego pr¹du który bêdzie przychodzi³ z komputera

			if (isMotor2Synchronization)
				motor2Struct.currentError = (motor2Struct.currentTarget2 - motor2Struct.current) << 10;
			else
				motor2Struct.currentError = (motor2Struct.currentTarget - motor2Struct.current) << 10;

			motor2Struct.pwmInput = arm_pid_q31(&motor2CurrentPid, motor2Struct.currentError);

			if(motor2Struct.pwmInput > 2000)
			{
				motor2Struct.pwmInput = 2000;
				motor2CurrentPid.state[2] = 2000;
			}
			if(motor2Struct.pwmInput < -2000)
			{
				motor2Struct.pwmInput = -2000;
				motor2CurrentPid.state[2] = -2000;
			}

			mb_Motor_Set_Pulse_Width(MOTOR2, motor2Struct.pwmInput);

			motor2Struct.position = QEIPositionGet(QEI1_BASE);

			motor2VelocityPidIterator++;
			if(motor2VelocityPidIterator == 19) motor2VelocityPidIterator = 0;

			isPid2Switch = 0;
			mb_LED_Off(LED1);
		}

		if(isPid1Switch)
		{
//			mb_LED_Switch(LED1);

			if (motor1VelocityPidIterator == 0 && isMotor1Synchronization)
			{
				motor1Struct.velocity = (QEIVelocityGet(QEI0_BASE) * 30) * (QEIDirectionGet(QEI0_BASE));
				motor1Struct.velocityError = (motor1Struct.velocityTarget - motor1Struct.velocity) << 10;

				motor1Struct.currentTarget2 = arm_pid_q31(&motor1VelocityPid, motor1Struct.velocityError);

				if(motor1Struct.currentTarget2 > 200)
				{
					motor1Struct.currentTarget2 = 200;
					motor1VelocityPid.state[2] = 200;
				}
				if(motor1Struct.currentTarget2 < -200)
				{
					motor1Struct.currentTarget2 = -200;
					motor1VelocityPid.state[2] = -200;
				}
			}

			if (isMotor1Synchronization)
				motor1Struct.currentError = (motor1Struct.currentTarget2 - motor1Struct.current) << 10;
			else
				motor1Struct.currentError = (motor1Struct.currentTarget - motor1Struct.current) << 10;

			motor1Struct.pwmInput = arm_pid_q31(&motor1CurrentPid, motor1Struct.currentError);

			if(motor1Struct.pwmInput > 1000)
			{
				motor1Struct.pwmInput = 1000;
				motor1CurrentPid.state[2] = 1000;
			}
			if(motor1Struct.pwmInput < -1000)
			{
				motor1Struct.pwmInput = -1000;
				motor1CurrentPid.state[2] = -1000;
			}

			mb_Motor_Set_Pulse_Width(MOTOR1, motor1Struct.pwmInput);

			motor1Struct.position = QEIPositionGet(QEI0_BASE);

			motor1VelocityPidIterator++;
			if(motor1VelocityPidIterator == 19) motor1VelocityPidIterator = 0;

			isPid1Switch = 0;
			mb_LED_Off(LED1);
//			isTerminalSend = 1;
		}

//        UARTprintf("AIN0 = %4d\t", motor1Current);
//        UARTprintf("AIN1 = %4d\t", motor2Current);
//        UARTprintf("AIN2 = %4d\t", inputVoltage);
//        UARTprintf("temperature = %d,%d\r", temperature1, temperature2);

//		motor1Struct->velocity = QEIVelocityGet(QEI0_BASE) * 3000 / 500;
//		motor1Struct->velocityErrorOldOld = motor1Struct->velocityErrorOld;
//		motor1Struct->velocityErrorOld = motor1Struct->velocityError;
//		motor1Struct->velocityError = motor1Struct->velocityTarget - motor1Struct->velocity;
//
//		motor1Struct->pwmInputOld = motor1Struct->pwmInput;
//		motor1Struct->pwmInputDelta = motor1Struct->velocityError - motor1Struct->velocityErrorOld +
//				loopTime * (motor1Struct->velocityError + motor1Struct->velocityErrorOld) /
//				(2 * motor1Struct->velocityPid->ti) + motor1Struct->velocityPid->td *
//				(motor1Struct->velocityError - 2 * motor1Struct->velocityErrorOld +
//				motor1Struct->velocityErrorOldOld) / loopTime;
//		motor1Struct->pwmInputDelta = motor1Struct->velocityPid->kp*motor1Struct->pwmInputDelta;
//
//		if (motor1Struct->pwmInputDelta >= 30)
//			motor1Struct->pwmInputDelta = 30;
//
//		if (motor1Struct->pwmInputDelta <= -30)
//			motor1Struct->pwmInputDelta = -30;
//
//		motor1Struct->pwmInput = motor1Struct->pwmInputDelta + motor1Struct->pwmInputOld;
//
//		if (motor1Struct->pwmInput <= 100)
//		{
//			motor1Struct->pwmInput = 100;
//		}
//		if (motor1Struct->pwmInput >= 800)
//		{
//			motor1Struct->pwmInput = 800;
//		}
//
//		mb_Motor_Set_Pulse_Width(MOTOR1, motor1Struct->pwmInput);
//
//		motor2Struct->velocity = QEIVelocityGet(QEI1_BASE);
//		motor2Struct->velocity = motor2Struct->velocity * 3000 / 32;
//		motor2Struct->velocityError = motor2Struct->velocityTarget - motor2Struct->velocity;
//
////		UARTprintf("%d\n", motor2Current);
//
//		if (loopIterator == 100)
//		{
//			UARTprintf("Motor_1 velocity: %u\n", motor1Struct->velocity);
//			UARTprintf("Motor_2 velocity: %u\n", motor2Struct->velocity);
//			UARTprintf("\n\n\n");
//			loopIterator = 0;
//		}

//		UARTprintf("Motor_1 position: %u\n", motor1Position);
//		UARTprintf("Motor_2 position: %u\n", motor2Position);
//
		if(isTerminalSend)
		{
//		cur1 = current1;

//			isMotor1Synchronization = 1;
//			isMotor2Synchronization = 1;

		/*UARTprintf("pwm: %i\t%i\n", motor1Struct.pwmInput, motor2Struct.pwmInput);
		UARTprintf("error: %i\t%i\n", motor1Struct.currentError, motor2Struct.currentError);
		UARTprintf("current: %i\t%i\n", motor1Struct.current, motor2Struct.current);
		UARTprintf("temperature = %d,%d\r", temperature1, temperature2);*/

			UARTprintf("%i\n", motor1Struct.current);

//		if(cur1 >= 0)
//			UARTprintf("wp: %i mA\n", cur1);
//		if(cur1 < 0)
//			UARTprintf("Motor_1 current: -%i mA\n", -cur1);
//		UARTprintf("Motor_2 current: %d mA\n", current2);
//		if(inputVoltage2 < 10)
//			UARTprintf("Input voltage: %u,00%u\n", inputVoltage1, inputVoltage2);
//		else if(inputVoltage2 < 100 && inputVoltage2 > 9)
//			UARTprintf("Input voltage: %u,0%u\n", inputVoltage1, inputVoltage2);
//		else if(inputVoltage2 > 99)
//			UARTprintf("Input voltage: %u,%u\n", inputVoltage1, inputVoltage2);
//		if(temperature2 < 10)
//			UARTprintf("Microcontroller temperature: %u,0%u\n", temperature1, temperature2);
//		else
//			UARTprintf("Microcontroller temperature: %u,%u\n", temperature1, temperature2);
		isTerminalSend = 0;
		}
//		ADCProcessorTrigger(ADC0_BASE, 1);
//		delayMS(200);
//		ADCProcessorTrigger(ADC1_BASE, 0);
//		delayMS(200);
	}

}
