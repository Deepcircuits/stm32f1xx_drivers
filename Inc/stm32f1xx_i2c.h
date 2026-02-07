/*
 * stm32f1xx_i2c.h
 *
 *  Created on: 20 Jan 2026
 *      Author: NAVDEEP
 */

#ifndef INC_STM32F1XX_I2C_H_
#define INC_STM32F1XX_I2C_H_

#include "stm32f1xx.h"

typedef struct{

	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

typedef struct
{
	I2C_RegDef_t 	*pI2Cx;
	I2C_Config_t 	I2C_Config;
	uint8_t 		*pTxBuffer; /* !< To store the app. Tx buffer address > */
	uint8_t 		*pRxBuffer;	/* !< To store the app. Rx buffer address > */
	uint32_t 		TxLen;		/* !< To store Tx len > */
	uint32_t 		RxLen;		/* !< To store Rx len > */
	uint8_t 		TxRxState;	/* !< To store Communication state > */
	uint8_t 		DevAddr;	/* !< To store slave/device address > */
    uint32_t        RxSize;		/* !< To store Rx size  > */
    uint8_t         Sr;			/* !< To store repeated start value  > */
}I2C_Handle_t;

/*
 * I2C application states
 */
#define I2C_READY 					0
#define I2C_BUSY_IN_RX 				1
#define I2C_BUSY_IN_TX 				2
#define I2C_SCLSPEED_STDMODE        100000
#define I2C_SCLSPEED_FASTMODE_4K    400000
#define I2C_SCLSPEED_FASTMODE_2K    200000

#define I2C_ACK_EN                  1
#define I2C_ACK_DIS                 0

#define I2C_FM_DUTY_2               0
#define I2C_FM_DUTY_16_9            1

// I2C FLAG DEFNS from SR1

#define I2C_FLAG_TXE        ( 1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE       ( 1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB			( 1 << I2C_SR1_SB)
#define I2C_FLAG_OVR  		( 1 << I2C_SR1_OVR)
#define I2C_FLAG_AF   		( 1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO 		( 1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR 		( 1 << I2C_SR1_BERR)
#define I2C_FLAG_STOPF 		( 1 << I2C_SR1_STOPF)
#define I2C_FLAG_ADD10 		( 1 << I2C_SR1_ADD10)
#define I2C_FLAG_BTF  		( 1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR 		( 1 << I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT 	( 1 << I2C_SR1_TIMEOUT)

#define I2C_DISABLE_SR  	RESET
#define I2C_ENABLE_SR   	SET


/*************APIS SUPPORTED BY THE DRIVER***************/

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
void I2C_MasterRecieveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);
void I2C_PeriphClockControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);
void I2C_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void I2C_IRQHandling(I2C_Handle_t *pHandle);
uint8_t I2C_Get_FlagStatus(I2C_RegDef_t *pI2Cx,uint32_t FlagName);
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);
#endif /* INC_STM32F1XX_I2C_H_ */
