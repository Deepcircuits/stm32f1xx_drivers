/*
 * stm32f1xx_i2c.c
 *
 *  Created on: 20 Jan 2026
 *      Author: NAVDEEP  */

#include "stm32f1xx.h"



// local prototypes
static void I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle );
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);


static void I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx){

			pI2Cx->CR1 |= (1<< I2C_CR1_START);

			}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
    }
    else
    {
        pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
    }
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
    SlaveAddr = SlaveAddr << 1;      // left shift for R/W bit
    SlaveAddr &= ~(1);               // clear bit0 → write

    pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr){

	 SlaveAddr = SlaveAddr << 1;      // left shift for R/W bit
	    SlaveAddr |= (1);               // set bit0 → read

	    pI2Cx->DR = SlaveAddr;
}



static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle )
{
	uint32_t dummy_read;
	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
	{
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
		    if(pI2CHandle->RxSize == 1)
		    {
		        I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
		    }

		    dummy_read = pI2CHandle->pI2Cx->SR1;
		    dummy_read = pI2CHandle->pI2Cx->SR2;
		    (void)dummy_read;
		}

		else
		{
			//clear the ADDR flag ( read SR1 , read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;

		}

	}
	else
	{
		//device is in slave mode
		//clear the ADDR flag ( read SR1 , read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}


}


 void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP);
}



uint8_t I2C_Get_FlagStatus(I2C_RegDef_t *pI2Cx,uint32_t FlagName){

if(pI2Cx->SR1 & FlagName){

	return FLAG_SET;
}

	return FLAG_RESET;
}


uint32_t  RCC_GetPLLOutputClock();

uint32_t RCC_GetPCLK1Value();


void I2C_Init(I2C_Handle_t *pI2CHandle){

	uint32_t tempreg = 0 ;

		//enable the clock for the i2cx peripheral
		I2C_PeriphClockControl(pI2CHandle->pI2Cx,ENABLE);

		//ack control bit
		pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
		pI2CHandle->pI2Cx->CR1 |= (pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);


		//configure the FREQ field of CR2
		tempreg = 0;
		tempreg |= RCC_GetPCLK1Value() /1000000U ;
		pI2CHandle->pI2Cx->CR2 =  (tempreg & 0x3F);

	   //program the device own address
		tempreg = 0;
		tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
		tempreg |= ( 1 << 14);
		pI2CHandle->pI2Cx->OAR1 = tempreg;

		//CCR calculations
		uint16_t ccr_value = 0;
		tempreg = 0;
		if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCLSPEED_STDMODE)
		{
			//mode is standard mode
			ccr_value = (RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
			tempreg |= (ccr_value & 0xFFF);
		}else
		{
			//mode is fast mode
			tempreg |= ( 1 << 15);
			tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
			if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
			{
				ccr_value = (RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
			}else
			{
				ccr_value = (RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
			}
			tempreg |= (ccr_value & 0xFFF);
		}
		pI2CHandle->pI2Cx->CCR = tempreg;

		//TRISE Configuration
		if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCLSPEED_STDMODE)
		{
			//mode is standard mode

			tempreg = (RCC_GetPCLK1Value() /1000000U) + 1 ;

		}else
		{
			//mode is fast mode
			tempreg = ( (RCC_GetPCLK1Value() * 300) / 1000000000U ) + 1;

		}

		pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);


		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_PE);


	}

void I2C_DeInit(I2C_RegDef_t *pI2Cx){

	if(pI2Cx==I2C1){
		I2C1_REG_RST();
	}

	else if(pI2Cx==I2C2){
		I2C2_REG_RST();

	}

}
void I2C_PeriphClockControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi){

	if(EnorDi==ENABLE){

		if(pI2Cx==I2C1){
			I2C1_CLKEN();
		}

		else if(pI2Cx==I2C2){
			I2C2_CLKEN();
		}
	}

	else if(EnorDi==DISABLE){

		if(pI2Cx==I2C1){
					I2C1_CLKDIS();
				}

				else if(pI2Cx==I2C2){
					I2C2_CLKDIS();
				}

	}

}


void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        pI2Cx->CR1 |= (1 << I2C_CR1_PE);
    }
    else
    {
        pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
    }
}


void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,
                        uint8_t *pTxbuffer,
                        uint32_t Len,
                        uint8_t SlaveAddr,
                        uint8_t Sr)
{
	/* Clear STOPF if it is set (slave-mode residue) */
	if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF))
	{
	    volatile uint32_t dummy;
	    dummy = pI2CHandle->pI2Cx->SR1;   // read SR1
	    pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_PE); // write CR1
	    (void)dummy;
	}

    /* 0. Disable ACK for master TX */
    I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

    /* 1. Generate START condition */
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    /* 2. Wait for SB flag */
    while(!I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

    /* 3. Send slave address + write bit */
    I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

    /* 4. Wait for ADDR flag OR AF error */
    while(1)
    {
        if(I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR))
        {
            break;
        }

        if(I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_AF))
        {
            /* Clear AF */
            pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

            /* Generate STOP to release bus */
        //    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

            return; // address not acknowledged
        }
    }

    /* 5. Clear ADDR flag */
    I2C_ClearADDRFlag(pI2CHandle);

    /* 6. Send data */
    while(Len > 0)
    {
        while(!I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
        pI2CHandle->pI2Cx->DR = *pTxbuffer;
        pTxbuffer++;
        Len--;
    }

    /* 7. Wait for TXE and BTF */
    while(!I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
    while(!I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

    /* 8. Generate STOP if no repeated START */
    if(Sr == I2C_DISABLE_SR)
    {
        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
    }
}

void I2C_MasterRecieveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr){


		//1. Generate the START condition
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//2. confirm that start generation is completed by checking the SB flag in the SR1
		//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
    while(!I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

		//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits )
    I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);
		//4. wait until address phase is completed by checking the ADDR flag in the SR1
    while(!I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)){

    }

		//procedure to read only 1 byte from slave
		if(Len == 1)
		{
			//Disable Acking
			I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DIS);
			//clear the ADDR flag
			 I2C_ClearADDRFlag(pI2CHandle);
			//wait until  RXNE becomes 1
			  while(!I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)){

			  }
			  //generate STOP condition
			  if(Sr == I2C_DISABLE_SR)
			  			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			//read data in to buffer
			 *pRxbuffer = pI2CHandle->pI2Cx->DR;
		}


	    //procedure to read data from slave when Len > 1
		if(Len > 1)
		{
			//clear the ADDR flag
			 I2C_ClearADDRFlag(pI2CHandle);
			//read the data until Len becomes zero
			for ( uint32_t i = Len ; i > 0 ; i--)
			{
				//wait until RXNE becomes 1
				while(!I2C_Get_FlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));
				if(i == 2) //if last 2 bytes are remaining
				{
					//Disable Acking
					I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DIS);
					//generate STOP condition
					 if(Sr == I2C_DISABLE_SR)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}

				//read the data from data register in to buffer
				 *pRxbuffer = pI2CHandle->pI2Cx->DR;
				//increment the buffer address
				 pRxbuffer++;
			}

		}
		//re-enable ACKing
		if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_EN){
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_EN);
	}
}


void I2C_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}

}



void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}

void I2C_IRQHandling(I2C_Handle_t *pHandle){

}

void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data)
{
	pI2C->DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
    return (uint8_t) pI2C->DR;
}

