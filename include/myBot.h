/*
 * myBot.h
 *
 *  Created on: Jul 28, 2016
 *      Author: leo
 */

#ifndef MYBOT_H_
#define MYBOT_H_

#include <stdint.h>

//Initializations
void myBot_initializePins(void);
void myBot_initializeBluetooth(void);
void myBot_initializeMotors(uint16_t prescaler, uint16_t arr, uint16_t ccr1, uint16_t ccr2);

//Motor control
void myBot_motors(uint8_t PWM);

//Bluetooth controls
void myBot_bluetoothSend(unsigned char send);
unsigned char myBot_bluetoothReceive(void);

//ADC
void myBot_initilizeADC(uint8_t resolution);

//Basic timer
void myBot_TIM2(uint16_t PSC, uint16_t ARR);
void myBot_acknowledge_TIM2(void);


#endif /* MYBOT_H_ */
