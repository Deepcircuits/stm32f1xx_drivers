/*
 * stm32f1xx_spi.h
 *
 *  Created on: 17 Jan 2026
 *      Author: NAVDEEP
 */

#ifndef INC_STM32F1XX_SPI_H_
#define INC_STM32F1XX_SPI_H_


#include"stm32f1xx.h"

typedef struct {
	uint8_t DeviceMode;  // M or S
	uint8_t BusConfig;   // FD/HD/S
	uint8_t DFF;         // 8/16b Format
	uint8_t CPHA;
	uint8_t CPOL;
	uint8_t SSM;
	uint8_t Speed;

}SPI_PinConfig_t;

typedef struct {
	SPI_RegDef_t *pSPIx;
	SPI_PinConfig_t SPI_PinConfig;

}SPI_Handle_t;

/*************APIS SUPPORTED BY THE DRIVER***************/

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
void SPI_PeriphClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len);
void SPI_RecieveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len);
void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);
void SPI_Enable(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
uint8_t SPI_Get_FlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi);


#define SPI_DEVICE_MODE_MASTER     1
#define SPI_DEVICE_MODE_SLAVE      0

#define SPI_BUSCONFIG_FD           1   // FULL DUPLEX
#define SPI_BUSCONFIG_HD           2   // HALF DUPLEX
#define SPI_BUSCONFIG_S_RX         3   // SIMPLEX RX ONLY

#define SPI_CLKSPEED_DIV2          0       // SPI CLKSPEED DIVIDED BY 2
#define SPI_CLKSPEED_DIV4          1
#define SPI_CLKSPEED_DIV8          2
#define SPI_CLKSPEED_DIV16         3
#define SPI_CLKSPEED_DIV32         4
#define SPI_CLKSPEED_DIV64         5
#define SPI_CLKSPEED_DIV128        6
#define SPI_CLKSPEED_DIV256        7

#define SPI_DFF_8BIT               0
#define SPI_DFF_16BIT              1

#define SPI_CPOL_LOW               0
#define SPI_CPOL_HIGH              1

#define SPI_CPHA_LOW               0
#define SPI_CPHA_HIGH              1

#define SPI_SSM_DIS                0
#define SPI_SSM_EN                 1


#endif /* INC_STM32F1XX_SPI_H_ */
