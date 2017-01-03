/*
 * mb_LED.c
 *
 *  Created on: 16 maj 2016
 *      Author: Mateusz
 */

#include <stdint.h>
#include <stdbool.h>
#include "mb_LED.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

static volatile const uint32_t mbLedPeriph [] = {SYSCTL_PERIPH_GPIOE, SYSCTL_PERIPH_GPIOF};
static volatile const uint32_t mbLedBase [] = {GPIO_PORTE_BASE, GPIO_PORTF_BASE};
static volatile const uint32_t mbLedPin [] = {GPIO_PIN_0, GPIO_PIN_0};

void mb_LED_Init(MbDiodaLed LED)
{
	SysCtlPeripheralEnable(mbLedPeriph[LED]);
	SysCtlDelay(3);
	if(LED == LED2)
	{
		//Unlock PF0
		HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
		HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
		HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
	}
	GPIOPinTypeGPIOOutput(mbLedBase[LED], mbLedPin[LED]);
}

void mb_LED_On(MbDiodaLed LED)
{
	GPIOPinWrite(mbLedBase[LED], mbLedPin[LED], mbLedPin[LED]);
}

void mb_LED_Off(MbDiodaLed LED)
{
	GPIOPinWrite(mbLedBase[LED], mbLedPin[LED], 0);
}

void mb_LED_Switch(MbDiodaLed LED)
{
	if(GPIOPinRead(mbLedBase[LED], mbLedPin[LED]))
		mb_LED_Off(LED);
	else
		mb_LED_On(LED);
}
