/*
 * myBot.c
 *
 *  Created on: Jul 28, 2016
 *      Author: leo
 */

#include "myBot.h"
#define STM32F051
#include "stm32f0xx.h"

/*-------------------------------------------------------------------------
							Initializations
-------------------------------------------------------------------------*/

//Initializes the GPIO pins
void myBot_initializePins(void){
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	GPIOB->MODER |= GPIO_MODER_MODER0_0;
	GPIOB->MODER |= GPIO_MODER_MODER1_0;
	GPIOB->MODER |= GPIO_MODER_MODER2_0;
	GPIOB->MODER |= GPIO_MODER_MODER3_0;
	GPIOB->MODER |= GPIO_MODER_MODER4_0;
	GPIOB->MODER |= GPIO_MODER_MODER5_0;
	GPIOB->MODER |= GPIO_MODER_MODER6_0;
	GPIOB->MODER |= GPIO_MODER_MODER7_0;
}

//Sets up UART communication with HC-06 Bluetooth module
void myBot_initializeBluetooth(void) {
	// clock to USART1
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	// clock to GPIOA
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	//IMPORTANT - set the bits low of the pin. They are probably in output mode
	GPIOB->MODER &= ~(0b11 << (2*9));
	GPIOB->MODER &= ~(0b11 << (2*10));
	// PA9 and PA10 to AF
	GPIOA->MODER |= GPIO_MODER_MODER9_1;
	GPIOA->MODER |= GPIO_MODER_MODER10_1;

	// remap to correct AF
	GPIOA->AFR[1] |= (1 << (1*4)); // remap pin 9 to AF1
	GPIOA->AFR[1] |= (1 << (2*4)); // remap pin 10 to AF1
	// BRR = fclk / baud = fclk / 115200
	SystemCoreClockUpdate();
	USART1->BRR = SystemCoreClock/9600;
	// enable with UE in CR1
	USART1->CR1 |= USART_CR1_UE;
	USART1->CR1 |= USART_CR1_RE;
	USART1->CR1 |= USART_CR1_TE;
}

//Initializes PWM to the motors. Begin with an appropriate frequency
void myBot_initializeMotors(uint16_t prescaler, uint16_t arr, uint16_t ccr1, uint16_t ccr2){
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; 			// enable clock to the timer
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; 				//enable clock for LEDs

	//IMPORTANT - set the bits low of the pin. They are probably in output mode
	GPIOB->MODER &= ~(0b11 << (2*4));
	GPIOB->MODER &= ~(0b11 << (2*5));
	GPIOB->MODER |= GPIO_MODER_MODER4_1;  			//set B4 to alternate function
	GPIOB->MODER |= GPIO_MODER_MODER5_1;

	GPIOB->AFR[0] |= 0x01 << (4*4); 				//set AFR to AF1 for B4
	GPIOB->AFR[0] |= 0x01 << (4*5);

	// ARR determines the frequency
	// CCRx	 determines duty cycle
	/* (1) Set prescaler to 47, so APBCLK/48 i.e 1MHz*/
	/* (2) Set ARR = 8,   as timer clock is 1MHz the period is 8 us */
	/* (3) Set CCRx = 4, the signal will be high during 4 us */
	/* (4) Select PWM mode 1 on OC1 (OC1M = 110), enable preload register on OC1 (OC1PE = 1) */
	/* (5) Select active high polarity on OC1 (CC1P = 0, reset value), enable the output on OC1 (CC1E = 1)*/
	/* (6) Enable counter (CEN = 1) select edge aligned mode (CMS = 00, reset value) select direction as upcounter (DIR = 0, reset value) */
	/* (7) Force update generation (UG = 1) */
	TIM3->PSC = prescaler; 								/* (1) */
	TIM3->ARR = arr;   								/* (2) */
	TIM3->CCR1 = ccr1;   								/* (3) */
	TIM3->CCR2 = ccr2;
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE; /* (4) */
	TIM3->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE; /* (4) */
	TIM3->CCER |= TIM_CCER_CC1E;					 /* (5) */
	TIM3->CCER |= TIM_CCER_CC2E;
	TIM3->CR1 |= TIM_CR1_CEN; 						/* (6) 	*/
	TIM3->CR2 |= TIM_CR1_CEN;
	TIM3->EGR |= TIM_EGR_UG; 						/* (7) */
}
/*-------------------------------------------------------------------------
							Motor control
-------------------------------------------------------------------------*/

void myBot_motors(uint8_t PWM){
	//Using PB6 for right motor
	//Using PB7 for left motor
	//Forward corresponds to high

	if (PWM < 20){
	//	GPIOB->ODR |= 0b10000000;		//Original
		GPIOB->ODR &= ~(0b10000000);
		TIM3->CCR1 = PWM;
	}else if (PWM < 40){
	//	GPIOB->ODR &= ~(0b10000000);	//Original
		GPIOB->ODR |= 0b10000000;
		TIM3->CCR1 = (PWM - 20);
	}else if (PWM < 60){
	//	GPIOB->ODR |= 0b1000000;		//Original
		GPIOB->ODR &= ~(0b1000000);
		TIM3->CCR2 = (PWM - 40);
	}else{
	//	GPIOB->ODR &= ~(0b1000000);		//Original
		GPIOB->ODR |= 0b1000000;
		TIM3->CCR2 = (PWM - 60);
	}
}
/*-------------------------------------------------------------------------
								Actions
-------------------------------------------------------------------------*/
//This function will allow the robot to do a quick scan of the black and white track
//void myBot_whiteScan (void){
//	//Turns in a clockwise direction
//	myBot_motors(19);
//	myBot_motors(79);
//
//
//}




/*-------------------------------------------------------------------------
							Bluetooth communication
-------------------------------------------------------------------------*/

void myBot_bluetoothSend(unsigned char send){
	USART1->TDR = send;
}

unsigned char myBot_bluetoothReceive(void){
	while ( (USART1->ISR & USART_ISR_RXNE) == 0); // while receive IS empty, hang
	return USART1->RDR;
}

/*-------------------------------------------------------------------------
									ADC
-------------------------------------------------------------------------*/

void myBot_initilizeADC(uint8_t resolution) {
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN; 			//enable clock for ADC
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 			//enable clock for port
		GPIOA->MODER |= GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5 | GPIO_MODER_MODER6; 			//Select the pins to be scanned through
		ADC1->CHSELR |= ADC_CHSELR_CHSEL3 | ADC_CHSELR_CHSEL4 | ADC_CHSELR_CHSEL5 | ADC_CHSELR_CHSEL6; 			// select channel 6
		if (resolution == 12){
			ADC1->CFGR1 &= ~0b11000;			// resolution to 12 bit
		}
		else if (resolution == 10){
			ADC1->CFGR1 &= ~0b11000;
			ADC1->CFGR1 = 0b1000;				// resolution to 10 bit
		}
		else if (resolution == 8){
			ADC1->CFGR1 &= ~0b11000;
			ADC1->CFGR1 |= 0b10000; 			// resolution to 8 bit
		}
		else {
			ADC1->CFGR1 |= 0b11000;				// resolution to 6 bit
		}
		ADC1->CFGR1 |=ADC_CFGR1_WAIT |!ADC_CFGR1_SCANDIR; //WAIT mode enabled, and upwards scanning direction
	//	ADC1->CFGR1 |= ADC_CFGR1_CONT;				// Setting continuous mode
		ADC1->CR |= ADC_CR_ADEN; 					// set ADEN=1 in the ADC_CR register
		while((ADC1->ISR & ADC_ISR_ADRDY) == 0);	//wait until ADRDY==1 in ADC_ISR
}

//ADC1->CR |= ADC_CR_ADSTART;
//while((ADC1->ISR & ADC_ISR_EOC) == 0){
//}
//black = ADC1->DR;
//while((ADC1->ISR & ADC_ISR_EOC) == 0){
//}
//black = ADC1->DR;
//while((ADC1->ISR & ADC_ISR_EOC) == 0){
//}
//black = ADC1->DR;

/*-------------------------------------------------------------------------
								BASIC TIMER
-------------------------------------------------------------------------*/

void myBot_TIM2(uint16_t PSC, uint16_t ARR){
	RCC->APB1ENR |= RCC_APB1Periph_TIM2;
    TIM2->PSC = PSC;
    TIM2->ARR = ARR;
	//Buffereing the timer
	TIM2->CR1 |= TIM_CR1_ARPE;
	//enable interrupt request
    TIM2->DIER |= TIM_DIER_UIE;
    //enable TIM6 IRQ in NVIC
    NVIC_EnableIRQ(TIM2_IRQn);
    //start timer
    TIM2->CR1 |= TIM_CR1_CEN;
}

void myBot_acknowledge_TIM2(void){
    TIM2->SR = 0x0;
}

/*-------------------------------------------------------------------------

-------------------------------------------------------------------------*/
