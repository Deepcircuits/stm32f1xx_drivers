/*
 * 001_ledtoggle.c
 *
 *  Created on: 11 Jan 2026
 *      Author: NAVDEEP
 */
#include"stm32f1xx.h"

void delay(void){
		for(int i=0;i<500000;i++);
	}

void delay1(void){
		for(int i=0;i<100000;i++);
	}

int main(void){

GPIO_Handle_t led;
GPIO_Handle_t input;

led.pGPIOx=GPIOC;
led.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_13;
led.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUTPUT;
led.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
led.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
led.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_10MHZ;

input.pGPIOx=GPIOA;
input.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_0;
input.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_INPUT;
input.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PD;

GPIO_Init(&led);
GPIO_Init(&input);

 while(1){
	if (GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_0)==SET){
	GPIO_ToggleOutputPin(GPIOC,GPIO_PIN_13);
	delay1();
	}
	else{
		GPIO_WriteToOutputPin(GPIOC,GPIO_PIN_13,SET);

	}

}
}



