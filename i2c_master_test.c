/*
 * i2c_master_test.c
 *
 *  Created on: 22 Jan 2026
 *      Author: NAVDEEP
 */
#include "stm32f1xx.h"
#include<string.h>

#define SLAVE_ADDR         0x28

I2C_Handle_t tx;
uint8_t data[]="HELLO WORLD!\n";


void I2C_GPIOInits(void){

	GPIO_Handle_t pin;
	memset(&pin,0,sizeof(pin));
	//SCL CONFIG
	pin.pGPIOx=GPIOB;
	pin.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	pin.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD;
	pin.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;
	pin.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_10MHZ;

	pin.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_6;
	GPIO_Init(&pin);

	pin.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_7;
	GPIO_Init(&pin);

}

void I2C_Inits(void){

	tx.pI2Cx=I2C1;
	tx.I2C_Config.I2C_ACKControl=I2C_ACK_EN;
	tx.I2C_Config.I2C_DeviceAddress=0x22;
	tx.I2C_Config.I2C_FMDutyCycle=I2C_FM_DUTY_2;
	tx.I2C_Config.I2C_SCLSpeed=I2C_SCLSPEED_STDMODE;

	I2C_Init(&tx);


}

int main(){

/*	// Enable GPIOB clock
 *	RCC->APB2ENR |= (1 << 3);   // IOPBEN
 *
 *	// PB6 & PB7 = AF Open-Drain, 10 MHz
 *	GPIOB->CRL &= ~((0xF << 24) | (0xF << 28));
 *	GPIOB->CRL |=  ((0xB << 24) | (0xB << 28));

 */
	I2C_PeriphClockControl(I2C1, ENABLE);

	I2C_GPIOInits();
	I2C_Inits();
	I2C_PeripheralControl(I2C1,ENABLE);

	I2C_MasterSendData(&tx,data,strlen((char*)data),SLAVE_ADDR,I2C_DISABLE_SR);

	while(1);
}
