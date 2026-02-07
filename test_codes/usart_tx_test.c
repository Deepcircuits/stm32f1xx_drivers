/*
 * usart_tx_test.c
 *
 *  Created on: 6 Feb 2026
 *      Author: NAVDEEP
 */
/*
 * usart_tx_test.c
 *
 *  Created on: 6 Feb 2026
 *      Author: NAVDEEP
 */
#include "stm32f1xx.h"
#include <string.h>

USART_Handle_t tx;

char msg[]= "HELLO WORLD !\n";

void delay(){

	for(uint32_t i=0;i<500000;i++);
}

void USARTGPIO_Init(){

	GPIO_Handle_t pin;
	pin.pGPIOx=GPIOA;
	pin.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	pin.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_9;
	pin.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	pin.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	pin.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_50MHZ;

	GPIO_Init(&pin);
}

void USART1_Init(){


	tx.pUSARTx=USART1;
	tx.USART_Config.USART_Baud=USART_STD_BAUD_115200;
	tx.USART_Config.USART_HWFlowControl=USART_HW_FLOW_CTRL_NONE;
	tx.USART_Config.USART_Mode=USART_MODE_ONLY_TX;
	tx.USART_Config.USART_NoOfStopBits=USART_STOPBITS_1;
	tx.USART_Config.USART_ParityControl=USART_PARITY_DISABLE;
	tx.USART_Config.USART_WordLength=USART_WORDLEN_8BITS;

	USART_Init(&tx);
}

int main(){

	USARTGPIO_Init();
	USART1_Init();

	USART_PeripheralControl(USART1,ENABLE);

	while(1){
	for (volatile uint32_t i = 0; i < 100000; i++){

	USART_SendData(&tx,(uint8_t*)msg,strlen(msg));
	delay();
	}

	}
	return 0;

}


