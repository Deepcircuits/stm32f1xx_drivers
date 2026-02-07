/*
 * stm32f1xx_gpio.c
 *
 *  Created on: 13 Jan 2026
 *      Author: NAVDEEP
 */

#include"stm32f1xx.h"


/* APIs supported by this driver */


/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             -  This function can be used to initialize the GPIO.
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  In order to init a GPIO Pin,The user program should create a pointer named pGPIOHandle of datatype GPIO_Handle_t

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t temp = 0;
    uint32_t pin  = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    uint32_t pos;
    volatile uint32_t *configReg;

    /* 1. Enable GPIO peripheral clock */
    GPIO_PeriphClockControl(pGPIOHandle->pGPIOx, ENABLE);

    /* 2. Select CRL or CRH */
    if(pin < 8)
    {
        configReg = &pGPIOHandle->pGPIOx->CRL;
        pos = pin * 4;
    }
    else
    {
        configReg = &pGPIOHandle->pGPIOx->CRH;
        pos = (pin - 8) * 4;
    }

    /* 3. Clear the 4 configuration bits */
    *configReg &= ~(0xF << pos);

    /* 4. GPIO MODE CONFIGURATION */

    /* INPUT or INTERRUPT modes are GPIO inputs */
    if(
        pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INPUT  ||
        pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT  ||
        pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT  ||
        pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT
      )
    {
        temp = 0x0;   /* MODE = 00 */

        if(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl == GPIO_PIN_PU)
        {
            temp |= (0x2 << 2);                /* CNF = 10 */
            pGPIOHandle->pGPIOx->ODR |= (1 << pin);
        }
        else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl == GPIO_PIN_PD)
        {
            temp |= (0x2 << 2);                /* CNF = 10 */
            pGPIOHandle->pGPIOx->ODR &= ~(1 << pin);
        }
        else
        {
            temp |= (0x1 << 2);                /* CNF = 01 (floating) */
        }
    }
    else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ANALOG)
    {
        temp = 0x0;                            /* MODE = 00, CNF = 00 */
    }
    else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUTPUT)
    {
        temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed;

        if(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType == GPIO_OP_TYPE_PP)
            temp |= (0x0 << 2);                /* GP push-pull */
        else
            temp |= (0x1 << 2);                /* GP open-drain */
    }
    else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
    {
        temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed;

        if(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType == GPIO_OP_TYPE_PP)
            temp |= (0x2 << 2);                /* AF push-pull */
        else
            temp |= (0x3 << 2);                /* AF open-drain */
    }

    /* 5. Write configuration to CRL/CRH */
    *configReg |= (temp << pos);

    /* 6. EXTI CONFIGURATION — ONLY FOR INTERRUPT MODES */
    if(
        pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT  ||
        pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT  ||
        pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT
      )
    {
        if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
        {
            EXTI->FTSR |= (1 << pin);
            EXTI->RTSR &= ~(1 << pin);
        }
        else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
        {
            EXTI->RTSR |= (1 << pin);
            EXTI->FTSR &= ~(1 << pin);
        }
        else
        {
            EXTI->RTSR |= (1 << pin);
            EXTI->FTSR |= (1 << pin);
        }

        /* Unmask EXTI line */
        EXTI->IMR |= (1 << pin);

        /* Enable AFIO clock */
        RCC->APB2ENR |= (1 << 0);

        uint8_t exti_index = pin / 4;
        uint8_t exti_pos   = (pin % 4) * 4;
        uint8_t portcode   = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

        /* Map EXTI line to GPIO port */
        AFIO->EXTICR[exti_index] &= ~(0xF << exti_pos);
        AFIO->EXTICR[exti_index] |=  (portcode << exti_pos);
    }
}


/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

	if(pGPIOx==GPIOA){

		GPIOA_REG_RST();

	}else if(pGPIOx==GPIOB){

		GPIOB_REG_RST();

	}else if(pGPIOx==GPIOC){

		GPIOC_REG_RST();

	}else if(pGPIOx==GPIOD){

		GPIOD_REG_RST();

	}else if(pGPIOx==GPIOE){

		GPIOE_REG_RST();

	}else if(pGPIOx==GPIOF){

		GPIOF_REG_RST();

	}else if(pGPIOx==GPIOG){

		GPIOG_REG_RST();
	}

}

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriphClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi){

	if(EnorDi == ENABLE){

		if(pGPIOx==GPIOA){

			GPIOA_CLKEN();

		}else if(pGPIOx==GPIOB){

			GPIOB_CLKEN();

		}else if(pGPIOx==GPIOC){

			GPIOC_CLKEN();

		}else if(pGPIOx==GPIOD){

			GPIOD_CLKEN();

		}else if(pGPIOx==GPIOE){

			GPIOE_CLKEN();

		}else if(pGPIOx==GPIOF){

			GPIOF_CLKEN();

		}else if(pGPIOx==GPIOG){

			GPIOG_CLKEN();
		}
	}
	else
	{
		if(pGPIOx==GPIOA){

					GPIOA_CLKDIS();

				}else if(pGPIOx==GPIOB){

					GPIOB_CLKDIS();

				}else if(pGPIOx==GPIOC){

					GPIOC_CLKDIS();

				}else if(pGPIOx==GPIOD){

					GPIOD_CLKDIS();

				}else if(pGPIOx==GPIOE){

					GPIOE_CLKDIS();

				}else if(pGPIOx==GPIOF){

					GPIOF_CLKDIS();

				}else if(pGPIOx==GPIOG){

					GPIOG_CLKDIS();
				}
	}

}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -   0 or 1
 *
 * @Note              -

 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){

	 uint8_t value;
	 value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x01);
	 return value;
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value){

	if(Value == GPIO_PIN_SET){


		pGPIOx->ODR |=(1<<PinNumber);

		}

	else{

		pGPIOx->ODR &= ~(1<<PinNumber);

		}
	}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value){

		pGPIOx->ODR=Value;
	}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Togging is just flipping of states and not blinking.

 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){

	pGPIOx->ODR = pGPIOx->ODR ^(1<<PinNumber);
}


/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi){

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
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 &= ~( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 &= ~( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER2 &= ~( 1 << (IRQNumber % 64) );
		}
	}

}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber%4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}


	// This function can be used to configure priorities of the Interrupt


/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_IRQHandling(uint8_t PinNumber){

	//clear the exti pr register corresponding to the pin number

		if(EXTI->PR & ( 1 << PinNumber))        // Testing of bits
		{
			//clear
			EXTI->PR |= ( 1 << PinNumber);      // Setting of bits
		}


}










