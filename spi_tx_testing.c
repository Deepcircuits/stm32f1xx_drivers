/*
 * spi_tx_testing.c
 *
 *  Created on: 19 Jan 2026
 *      Author: NAVDEEP
 */

// NSS/PA4, SCK/PA5, MISO/PA6, MOSI/PA7
#include"stm32f1xx.h"

void SPI1_GPIOInits(void){

	GPIO_Handle_t SPIPins;
	memset(&SPIPins, 0, sizeof(SPIPins));
	/*********** SPI1 SCK ***********/
	SPIPins.pGPIOx=GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_10MHZ;
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_5;
	GPIO_Init(&SPIPins);

	/*********** SPI1 MOSI ***********/
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&SPIPins);

	/*********** SPI1 MISO ***********/
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&SPIPins);

	/*********** SPI1 NSS (Software controlled) ***********/
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_4;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_10MHZ;
	GPIO_Init(&SPIPins);

	/* Pull NSS HIGH initially (deselect slave) */
	GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

}


void SPI1_Inits(void)
{
	SPI_Handle_t test;
	test.pSPIx=SPI1;
	test.SPI_PinConfig.DeviceMode=SPI_DEVICE_MODE_MASTER;
	test.SPI_PinConfig.BusConfig=SPI_BUSCONFIG_FD;
	test.SPI_PinConfig.DFF=SPI_DFF_8BIT;
	test.SPI_PinConfig.Speed=SPI_CLKSPEED_DIV2;
	test.SPI_PinConfig.CPOL=SPI_CPOL_LOW;
	test.SPI_PinConfig.CPHA=SPI_CPHA_LOW;
	test.SPI_PinConfig.SSM=SPI_SSM_EN;
	SPI_Init(&test);
}

int main(){
	char data[]="Hello world";

    SPI1_GPIOInits();
	SPI1_Inits();

	// SSI Config makes NSS signal internally high and avoids MODF error
	SPI_SSIConfig(SPI1,ENABLE);

	SPI_Enable(SPI1,ENABLE);
	SPI_SendData(SPI1,(uint8_t*)data,strlen(data));
	SPI_Enable(SPI1,DISABLE);


	while(1);

	return 0;

}
