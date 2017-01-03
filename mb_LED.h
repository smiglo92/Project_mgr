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

void mb_LED_Init(MbDiodaLed LED);
void mb_LED_On(MbDiodaLed LED);
void mb_LED_Off(MbDiodaLed LED);
void mb_LED_Switch(MbDiodaLed LED);

#endif /* MB_LED_H_ */
