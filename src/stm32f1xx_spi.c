/*
 * stm32f1xx_spi.c
 *
 *  Created on: 17 Jan 2026
 *      Author: NAVDEEP
 */
#include "stm32f1xx.h"

void SPI_Init(SPI_Handle_t *pSPIHandle){

	//peripheral clock enable

		SPI_PeriphClockControl(pSPIHandle->pSPIx, ENABLE);

		//first lets configure the SPI_CR1 register

		uint32_t tempreg = 0;

		//1. configure the device mode
		tempreg |= pSPIHandle->SPI_PinConfig.DeviceMode << SPI_CR1_MSTR ;

		//2. Configure the bus config
		if(pSPIHandle->SPI_PinConfig.BusConfig == SPI_BUSCONFIG_FD)
		{
			//bidi mode should be cleared
			tempreg &= ~( 1 << SPI_CR1_BIDIMODE);

		}else if (pSPIHandle->SPI_PinConfig.BusConfig == SPI_BUSCONFIG_HD)
		{
			//bidi mode should be set
			tempreg |= ( 1 << SPI_CR1_BIDIMODE);
		}else if (pSPIHandle->SPI_PinConfig.BusConfig == SPI_BUSCONFIG_S_RX)
		{
			//BIDI mode should be cleared
			tempreg &= ~( 1 << SPI_CR1_BIDIMODE);
			//RXONLY bit must be set
			tempreg |= ( 1 << SPI_CR1_RXONLY);
		}

		// 3. Configure the spi serial clock speed (baud rate)
		tempreg |= pSPIHandle->SPI_PinConfig.Speed << SPI_CR1_BR;

		//4.  Configure the DFF
		tempreg |= pSPIHandle->SPI_PinConfig.DFF << SPI_CR1_DFF;

		//5. configure the CPOL
		tempreg |= pSPIHandle->SPI_PinConfig.CPOL << SPI_CR1_CPOL;

		//6 . configure the CPHA
		tempreg |= pSPIHandle->SPI_PinConfig.CPHA << SPI_CR1_CPHA;

		tempreg |= pSPIHandle->SPI_PinConfig.SSM << SPI_CR1_SSM;

		pSPIHandle->pSPIx->CR1 = tempreg;

}
void SPI_DeInit(SPI_RegDef_t *pSPIx){

		if(pSPIx==SPI1){

			SPI1_REG_RST();

		}else if(pSPIx==SPI2){

			SPI2_REG_RST();

		}else if(pSPIx==SPI3){

			SPI3_REG_RST();
	}
}
void SPI_PeriphClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi){

		if(EnorDi == ENABLE){

			if(pSPIx==SPI1){

				SPI1_CLKEN();

			}else if(pSPIx==SPI2){

				SPI2_CLKEN();

			}else if(pSPIx==SPI3){

				SPI3_CLKEN();
			}
		}
		else
		{
			if(pSPIx==SPI1){

						SPI1_CLKDIS();

					}else if(pSPIx==SPI2){

						SPI2_CLKDIS();

					}else if(pSPIx==SPI3){

						SPI3_CLKDIS();

					}

		}


	}

uint8_t SPI_Get_FlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName){

	if(pSPIx->SR & FlagName){

		return FLAG_SET;
	}

		return FLAG_RESET;
}

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len){
//Refer flowchart
		while(Len>0){

			// Wait unitl TXE is SET
			while(SPI_Get_FlagStatus(pSPIx,SPI_TXE_FLAG)==FLAG_RESET);

			//Check DFF bit in CR1
			if(pSPIx->CR1 & (1<<SPI_CR1_DFF)){

			//16 Bit data is loaded
		     pSPIx->DR= *((uint16_t*)pTxBuffer);

		     Len--;
			 Len--;
			 (uint16_t*)pTxBuffer++;
			}

			else{

			//8 Bit data is loaded
			 pSPIx->DR= *(pTxBuffer);

			 Len--;
			 pTxBuffer += 2;
			}
		}

}
void SPI_RecieveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len){

	while(Len>0){

				// Wait unitl TXE is SET
				while(SPI_Get_FlagStatus(pSPIx,SPI_RXNE_FLAG)==(uint8_t)FLAG_RESET);

				//Check DFF bit in CR1
				if(pSPIx->CR1 & (1<<SPI_CR1_DFF)){

				//16 Bit data is loaded from DR to Rx buff
			     *((uint16_t*)pRxBuffer) =  pSPIx->DR;

			     Len--;
				 Len--;
				 (uint16_t*)pRxBuffer++;
				}

				else{

				//8 Bit data is loaded
				  *(pRxBuffer)=pSPIx->DR;

				 Len--;
				 pRxBuffer += 2;
				}
			}

}
void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi){

}
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority){

}
void SPI_IRQHandling(SPI_Handle_t *pHandle){

}

void SPI_Enable(SPI_RegDef_t *pSPIx,uint8_t EnorDi){

	if(EnorDi==ENABLE){
		pSPIx->CR1|=(1<<SPI_CR1_SPE);
	}
	else{
		pSPIx->CR1&=~(1<<SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi){

	if(EnorDi==ENABLE){
			pSPIx->CR1|=(1<<SPI_CR1_SSI );
		}
		else{
			pSPIx->CR1&=~(1<<SPI_CR1_SSI);
		}
	}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi){

	if(EnorDi==ENABLE){
			pSPIx->CR2|=(1<<SPI_CR2_SSOE );
		}
		else{
			pSPIx->CR2&=~(1<<SPI_CR2_SSOE);
		}
	}

