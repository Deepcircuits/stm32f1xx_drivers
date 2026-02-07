/*
 * stm32f1xx_usart.c
 *
 *  Created on: 29 Jan 2026
 *      Author: NAVDEEP
 */
#include "stm32f1xx.h"

void USART_PeriphClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi){

	if(EnOrDi==ENABLE){
		if(pUSARTx==USART1){
			USART1_CLKEN();
		}
			else if(pUSARTx==USART2){
				USART2_CLKEN();
		}
			else if(pUSARTx==USART3){
							USART3_CLKEN();
		}

			else if(pUSARTx==UART4){
										UART4_CLKEN();
		}

			else if(pUSARTx==UART5){
													UART5_CLKEN();
		}

	}

	else if(EnOrDi==DISABLE){
		if(pUSARTx==USART1){
					USART1_CLKDIS();
				}
					else if(pUSARTx==USART2){
						USART2_CLKDIS();
				}
					else if(pUSARTx==USART3){
									USART3_CLKDIS();
				}

					else if(pUSARTx==UART4){
												UART4_CLKDIS();
				}

					else if(pUSARTx==UART5){
															UART5_CLKDIS();
				}
	}
}

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle){


	//Temporary variable
	uint32_t tempreg=0;

/******************************** Configuration of CR1******************************************/

	//Implement the code to enable the Clock for given USART peripheral
	USART_PeriphClockControl(pUSARTHandle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//Implement the code to enable the Receiver bit field
		tempreg|= (1 << USART_CR1_RE);
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//Implement the code to enable the Transmitter bit field
		tempreg |= ( 1 << USART_CR1_TE );

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//Implement the code to enable the both Transmitter and Receiver bit fields
		tempreg |= ( ( 1 << USART_CR1_RE) | ( 1 << USART_CR1_TE) );
	}

    //Implement the code to configure the Word length configuration item
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M ;


    //Configuration of parity control bit fields
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//Implement the code to enable the parity control
		tempreg |= ( 1 << USART_CR1_PCE);

		//Implement the code to enable EVEN parity
		//Not required because by default EVEN parity will be selected once you enable the parity control

	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		//Implement the code to enable the parity control
	    tempreg |= ( 1 << USART_CR1_PCE);

	    //Implement the code to enable ODD parity
	    tempreg |= ( 1 << USART_CR1_PS);

	}

   //Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

/******************************** Configuration of CR2******************************************/

	tempreg=0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

/******************************** Configuration of CR3******************************************/

	tempreg=0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//Implement the code to enable CTS flow control
		tempreg |= ( 1 << USART_CR3_CTSE);


	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//Implement the code to enable RTS flow control
		tempreg |= USART_CR3_RTSE;

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control
		tempreg |= ( 1 << USART_CR3_CTSE);
		tempreg |= ( 1 << USART_CR3_RTSE);
	}


	pUSARTHandle->pUSARTx->CR3 = tempreg;

/******************************** Configuration of BRR(Baud rate register)******************************************/

	//Implement the code to configure the baud rate
	//We will cover this in the lecture. No action required here
	USART_SetBaudRate(pUSARTHandle->pUSARTx,pUSARTHandle->USART_Config.USART_Baud);

}
void USART_DeInit(USART_RegDef_t *pUSARTx){

	if(pUSARTx==USART1){

		USART1_REG_RST();
	}

	else if(pUSARTx==USART2){

			USART2_REG_RST();
		}

	else if(pUSARTx==USART3){

			USART3_REG_RST();
		}

	else if(pUSARTx==UART4){

			UART4_REG_RST();
		}

	else if(pUSARTx==UART5){

			UART5_REG_RST();
		}

}

/*
 * Data Send and Receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len){


	uint16_t *pdata;

   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

		//Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer , so 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));

}
void  USART_ReceiveData(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len){

	   //Loop over until "Len" number of bytes are transferred
		for(uint32_t i = 0 ; i < Len; i++)
		{
			//Implement the code to wait until RXNE flag is set in the SR
			while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_RXNE));

			//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
			if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
			{
				//We are going to receive 9bit data in a frame

				//Now, check are we using USART_ParityControl control or not
				if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					//No parity is used , so all 9bits will be of user data

					//read only first 9 bits so mask the DR with 0x01FF
					*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

					//Now increment the pRxBuffer two times
					pRxBuffer++;
					pRxBuffer++;
				}
				else
				{
					//Parity is used, so 8bits will be of user data and 1 bit is parity
					 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
					 pRxBuffer++;
				}
			}
			else
			{
				//We are going to receive 8bit data in a frame

				//Now, check are we using USART_ParityControl control or not
				if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					//No parity is used , so all 8bits will be of user data

					//read 8 bits from DR
					 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
				}

				else
				{
					//Parity is used, so , 7 bits will be of user data and 1 bit is parity

					//read only 7 bits , hence mask the DR with 0X7F
					 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

				}

				//Now , increment the pRxBuffer
				pRxBuffer++;
			}
		}

}


uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t FlagName){

	if(pUSARTx->SR & FlagName){

		return FLAG_SET;
	}

		return FLAG_RESET;
}


void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t FlagName){

	pUSARTx->SR &= ~( FlagName);

}
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi){

	if(EnOrDi==ENABLE){

				pUSARTx->CR1 |= (1 << USART_CR1_UE);

			}
				else {


					pUSARTx->CR1 &= ~(1 << USART_CR1_UE);

			}

}

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
    uint32_t PCLKx;
    uint32_t usartdiv;
    uint32_t M_part, F_part;
    uint32_t tempreg = 0;

    // 1. Get APB clock
    if (pUSARTx == USART1)
    {
        PCLKx = RCC_GetPCLK2Value();  // APB2
    }
    else
    {
        PCLKx = RCC_GetPCLK1Value();  // APB1
    }

    // 2. USARTDIV calculation (oversampling by 16 only)
    usartdiv = (25 * PCLKx) / (4 * BaudRate);

    // 3. Mantissa part
    M_part = usartdiv / 100;
    tempreg |= (M_part << 4);

    // 4. Fraction part
    F_part = usartdiv - (M_part * 100);

    // Fraction calculation for oversampling by 16
    F_part = ((F_part * 16) + 50) / 100;
    F_part &= 0x0F;

    tempreg |= F_part;

    // 5. Write to BRR reg
    pUSARTx->BRR = tempreg;
}




