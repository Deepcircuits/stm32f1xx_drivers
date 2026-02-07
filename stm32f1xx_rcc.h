/*
 * stm32f1xx_rcc.h
 *
 *  Created on: 2 Feb 2026
 *      Author: NAVDEEP
 */

#ifndef INC_STM32F1XX_RCC_H_
#define INC_STM32F1XX_RCC_H_

#include "stm32f1xx.h"

//This returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//This returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);


uint32_t  RCC_GetPLLOutputClock(void);

#endif /* INC_STM32F1XX_RCC_H_ */
