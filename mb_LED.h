/*
 * mb_LED.h
 *
 *  Created on: 16 maj 2016
 *      Author: Mateusz
 */

#ifndef MB_LED_H_
#define MB_LED_H_

typedef enum {
    LED1 = 0,
    LED2 = 1
}MbDiodaLed;


/* Initialize LED GPIO
 * arg:
 * 		- LED: LED1 or LED2*/
void mb_LED_Init(MbDiodaLed LED);


/* Switch on LED
 * arg:
 * 		- LED: LED1 or LED2*/
void mb_LED_On(MbDiodaLed LED);


/* Switch off LED
 * arg:
 * 		- LED: LED1 or LED2*/
void mb_LED_Off(MbDiodaLed LED);


/* Switch LED state
 * arg:
 * 		- LED: LED1 or LED2*/
void mb_LED_Switch(MbDiodaLed LED);

#endif /* MB_LED_H_ */
