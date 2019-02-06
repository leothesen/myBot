#define STM32F051

#include <stdint.h>
#include "stm32f0xx.h"
#include "myBot.h"
#include "lcd_stm32f0.h"

void main(void);
void doAction (uint8_t colour);

uint32_t redReading;
uint32_t greenReading;
uint32_t blueReading;
uint32_t colourReading;
uint8_t currentColour;		/// 1-red. 2-green. 3-blue. 4-white. 5-blue

uint8_t redCommand; //0 - Do nothing, 1 - Go faster, 2 - Go slower, 3 - Do a spin
uint8_t greenCommand;
uint8_t blueCommand;

uint8_t leftSpeed = 19;
uint8_t rightSpeed = 59;

uint8_t spin = 0;

uint8_t timerOccured;
void main(void){

	myBot_initializePins();
	myBot_initializeBluetooth();
	myBot_initializeMotors(480,19,0,0);
	lcd_init();

	//new----------------------------------------------
	myBot_initilizeADC(8);
	myBot_TIM2(4800,500);

	//State of the line sensor. 1 - Black, 0 - White
	uint8_t leftState;
	uint8_t rightState;
	uint8_t centerState;

	//Flags for the line sensors. 0 - on white. 1 - on black (the line)
	uint32_t leftSensor;
	uint32_t centerSensor;
	uint32_t rightSensor;
	uint32_t black;
	uint32_t temp;
	uint8_t sampleBlack = 1;

	//Variables used for colour sensor

	uint8_t toggleColour = 0;





	uint32_t redArray[100];
	uint32_t greenArray[100];
	uint32_t blueArray[100];

	uint8_t redCount = 0;
	uint8_t greenCount = 0;
	uint8_t blueCount = 0;

	uint32_t redSum = 0;
	uint32_t greenSum = 0;
	uint32_t blueSum = 0;

	//------------------------------------------------



	unsigned char received = '(';
	uint8_t receivedInt;

	uint8_t progState = 0; //0 - waiting for instructions. 1 - currently running.
	uint8_t redFlag = 0; //flag will go high when 'r' has been received.
	uint8_t greenFlag = 0;
	uint8_t blueFlag = 0;

	uint8_t setCommand;


	uint8_t state = 0; //State variable
	//0 - No state (app started or exited state)
	//1 - remote control
	//2 - programming

	for(;;) {
		if (progState == 1){
			received = USART1->RDR;
			//received = myBot_bluetoothReceive();
		}else{
			received = myBot_bluetoothReceive();
		}
		receivedInt = received;

		if (received == 'x'){
			state = 0;
			progState = 0;
		}else if (received == 'c'){
			state = 1;
		}else if (received == 'p'){
			state = 2;
		}else if (received == 's'){
			progState = 1;
		}else if (received == 't'){
			progState = 0;
			sampleBlack = 1;
		}else if (received == 'n'){
			setCommand = 0;
		}else if (received == 'i'){
			setCommand = 1;
		}else if (received == 'd'){
			setCommand = 2;
		}else if (received == 'o'){
			setCommand = 3;
		}

		switch (state){
		case 0:
			GPIOB->ODR &= ~0b11111111;
			myBot_motors(0);
			myBot_motors(40);
			break;

		case 1:
//			This will handle the remote control state that the app could put the micro in.
			//just for debugging - take out.
//			GPIOB->ODR &= ~0b11;
//			GPIOB->ODR |= 0b1;

			myBot_motors(receivedInt);
			break;

		case 2:
//			This will handle the programming state that the app could put the micro in.
			//Debugging
//			GPIOB->ODR &= ~0b11;
//			GPIOB->ODR |= 0b10;

			switch(progState){

			case 0:
				//stop the robot if it is moving.
				//GPIOB->ODR &= ~0b11111111;
				myBot_motors(0);
				myBot_motors(40);

				//will only execute the loop after the flag has been set, therefore receiving the character sent after the colour
				if (redFlag == 1){
					redCommand = setCommand;
					redFlag = 0;
				}else if (greenFlag == 1){
					greenCommand = setCommand;
					greenFlag = 0;
				}else if (blueFlag == 1){
					blueCommand = setCommand;
					blueFlag = 0;
				}
				//looking at the received character for indication which colour's command will come next.
				switch (received){
				case 'r':
					redFlag = 1;
					greenFlag = 0;
					blueFlag = 0;
					break;
				case 'g':
					redFlag = 0;
					greenFlag = 1;
					blueFlag = 0;
					break;
				case 'b':
					redFlag = 0;
					greenFlag = 0;
					blueFlag = 1;
					break;
				}

				break;

			case 1:
				//now put all the code in here that will follow lines and shit.

				//Taking a sample of the middle QRD1114 to see what black looks like

				if (sampleBlack == 1){
					ADC1->CR |= ADC_CR_ADSTART;
					while((ADC1->ISR & ADC_ISR_EOC) == 0){
					}
					black = ADC1->DR;
					while((ADC1->ISR & ADC_ISR_EOC) == 0){
					}
					black = ADC1->DR;
					while((ADC1->ISR & ADC_ISR_EOC) == 0){
					}
					black = ADC1->DR;
					while((ADC1->ISR & ADC_ISR_EOC) == 0){
					}
					temp = ADC1->DR;





					sampleBlack = 0;
				}

				//new----------------------------------------------
				//toggling the colours of the RGB LED
						if (timerOccured == 1){
							timerOccured = 0;
							toggleColour++;
							if (toggleColour == 3){
								toggleColour = 4;
							}else if (toggleColour > 4){
								toggleColour = 1;
							}
						}

						GPIOB->ODR &= ~0b111;
						GPIOB->ODR |= toggleColour;

						//Taking ADC readings from the four channels

						ADC1->CR |= ADC_CR_ADSTART;
						while((ADC1->ISR & ADC_ISR_EOC) == 0){
						}
						leftSensor = ADC1->DR;
						while((ADC1->ISR & ADC_ISR_EOC) == 0){
						}
						rightSensor = ADC1->DR;
						while((ADC1->ISR & ADC_ISR_EOC) == 0){
						}
						centerSensor = ADC1->DR;
						while((ADC1->ISR & ADC_ISR_EOC) == 0){
						}
						colourReading = ADC1->DR;

						//Assigning the ADC reading of the colour to the colour the RGB was showing

						switch (toggleColour){
						case 1:
							redReading = colourReading;
							redArray[redCount] = redReading;
							if (redCount < 99){
								redCount++;
							}else{
								redCount = 0;
							}

							break;
						case 2:
							greenReading = colourReading;
							greenArray[greenCount] = greenReading;
							if (greenCount < 99){
								greenCount++;
							}else{
								greenCount = 0;
							}
							break;
						case 4:
							blueReading = colourReading;
							blueArray[blueCount] = blueReading;
							if (blueCount < 99){
								blueCount++;
							}else{
								blueCount = 0;
							}
						}

						//Adding the current colour to an array of the 10 most current readings
						//The average of this array is calculated to avoid misreading the colour

						redSum = 0;
						greenSum = 0;
						blueSum = 0;
						for (uint8_t i = 0;i < 99; i++){
							redSum = redSum + redArray[i];
							greenSum = greenSum + greenArray[i];
							blueSum = blueSum + blueArray[i];
						}
						redReading = redSum/100;
						greenReading = greenSum/100;
						blueReading = blueSum/100;

						//Assigning the value of the current colour
						if ((redReading > 75) & (greenReading > 75) & (blueReading > 75)){
							currentColour = 4;
						}else if((redReading < 65) & (greenReading < 65) & (blueReading < 65)){
							currentColour = 5;
						}else if ((redReading > greenReading) && (redReading > blueReading)){
							currentColour = 1;
						}
						else if ((greenReading > redReading) && (greenReading > blueReading)){
							currentColour = 2;
						}
						else if ((blueReading > redReading) && (blueReading > redReading)){
							currentColour = 3;
						}

						//showing the current colour on the LCD

//						lcd_command(LCD_CURSOR_HOME);
//						lcd_display_uint8(currentColour);
//						lcd_display_uint8(redReading);

						//Displaying which sensor is on white or black
						if(spin == 1){
							if (leftSensor < (black/2)){
								leftState = 0;
							}else{
								leftState = 1;
							}
							if (rightSensor < (black/2)){
								rightState = 0;
							}else{
								rightState = 1;
							}

							if (rightState == 0){
								myBot_motors(39);
								myBot_motors(59);
							}else{
								myBot_motors(leftSpeed);
								myBot_motors(rightSpeed);
								spin = 0;
							}
						}else{
							if (leftSensor < (black/2)){
								leftState = 0;
								myBot_motors(leftSpeed);
							}else{
								leftState = 1;
								myBot_motors(0);
							}
							if (rightSensor < (black/2)){
								rightState = 0;
								myBot_motors(rightSpeed);
							}else{
								rightState = 1;
								myBot_motors(40);
							}
							if (centerSensor < (black/2)){
								centerState = 0;
							}else{
								centerState = 1;
							}
						}
				//------------------------------------------------

				//Now I need to add code that will respond to the robot hitting a colour.
						//fist make sure that both line sensors are on white.
						if ((rightState == 0) & (leftState == 0)){
							doAction(currentColour);
						}


				break;
			}


			break;
		}
	}
}

void doAction (uint8_t colour){
	switch (colour){
	case 1:
		chooseAction(redCommand);
		break;
	case 2:
		chooseAction(greenCommand);
		break;
	case 3:
		chooseAction(blueCommand);
		break;
	}
}

void chooseAction (uint8_t actionColour){
	switch (actionColour){
	case 1:
		leftSpeed = 19;
		rightSpeed = 59;
		break;
	case 2:
		leftSpeed = 17;
		rightSpeed = 57;
		break;
	case 3:
		spin = 1;
		break;
	default:
		break;
	}
}



void TIM2_IRQHandler(void){
	myBot_acknowledge_TIM2();

	//Minimizing code in the interrupt by setting a flag high, which is checked in the for loop.

	timerOccured = 1;

}


