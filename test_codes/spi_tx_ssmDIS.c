/*
 * spi_tx_ssmDIS.c
 *
 *  Created on: 19 Jan 2026
 *      Author: NAVDEEP
 */

// NSS/PA4, SCK/PA5, MISO/PA6, MOSI/PA7

#include "stm32f1xx.h"

void SPI_GPIOInit(void){


	GPIO_Handle_t pin;
	memset(&pin,0,sizeof(pin));
	//SCLK CONFIG
	pin.pGPIOx=GPIOA;
	pin.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	pin.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	pin.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	pin.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_10MHZ;
	pin.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_5;
	GPIO_Init(&pin);
	//MOSI CONFIG
	pin.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_7;
	GPIO_Init(&pin);
	//MISO CONFIG
	pin.pGPIOx=GPIOA;
	pin.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_INPUT;
	pin.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	pin.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_10MHZ;
	pin.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_6;
	GPIO_Init(&pin);
	//NSS CONFIG
	pin.pGPIOx=GPIOA;
	pin.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUTPUT;
	pin.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	pin.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_10MHZ;
	pin.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_4;
	GPIO_Init(&pin);

	GPIO_WriteToOutputPin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
}

void SPI_Inits(void){

	SPI_Handle_t tx;
	tx.pSPIx=SPI1;
	tx.SPI_PinConfig.BusConfig=SPI_BUSCONFIG_FD;
	tx.SPI_PinConfig.CPHA=SPI_CPHA_LOW;
	tx.SPI_PinConfig.CPOL=SPI_CPOL_LOW;
	tx.SPI_PinConfig.DFF=SPI_DFF_8BIT;
	tx.SPI_PinConfig.DeviceMode=SPI_DEVICE_MODE_MASTER;
	tx.SPI_PinConfig.SSM=SPI_SSM_DIS;
	tx.SPI_PinConfig.Speed=SPI_CLKSPEED_DIV32;

	SPI_Init(&tx);

}

int main(){
	char data[]="HELLO WORLD !\n";
	SPI_GPIOInit();
	SPI_Inits();

	SPI_SSOEConfig(SPI1,ENABLE);

	SPI_Enable(SPI1,ENABLE);
	//SPI_SSIConfig(SPI1, ENABLE);


		//GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // NSS LOW

		//first send length information
				//uint8_t dataLen = strlen(data);
				//SPI_SendData(SPI1,&dataLen,1);

		SPI_SendData(SPI1,(uint8_t*)data,strlen(data));
		while(SPI_Get_FlagStatus(SPI1,SPI_BUSY_FLAG));

		//GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // NSS HIGH

		SPI_Enable(SPI1,DISABLE);


		while(1);


	}

