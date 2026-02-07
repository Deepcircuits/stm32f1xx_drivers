/*
 * stm32f1xx_gpio.h
 *
 *  Created on: Jan 12, 2026
 *      Author: NAVDEEP
 */
// THIS IS THE HEADER FILE FOR CONFIGURING GPIO REGISTERS-> "USER LEVEL"

#ifndef INC_STM32F1XX_GPIO_H_
#define INC_STM32F1XX_GPIO_H_

#include "stm32f1xx.h"

// this includes a configuration structure for a GPIO pin

typedef struct{
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode;
    uint8_t GPIO_PinSpeed;
    uint8_t GPIO_PinPuPdControl;
    uint8_t GPIO_PinOPType;

} GPIO_PinConfig_t;

// this includes the Handle structure for a GPIO pin

typedef struct{
	GPIO_RegDef_t *pGPIOx;     // This holds the base address of the GPIO Port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig; // This holds the GPIO Pin configuration settings.

}GPIO_Handle_t;

/*
 * GPIO pin numbers
 */
#define GPIO_PIN_0     0
#define GPIO_PIN_1     1
#define GPIO_PIN_2     2
#define GPIO_PIN_3     3
#define GPIO_PIN_4     4
#define GPIO_PIN_5     5
#define GPIO_PIN_6     6
#define GPIO_PIN_7     7
#define GPIO_PIN_8     8
#define GPIO_PIN_9     9
#define GPIO_PIN_10    10
#define GPIO_PIN_11    11
#define GPIO_PIN_12    12
#define GPIO_PIN_13    13
#define GPIO_PIN_14    14
#define GPIO_PIN_15    15

/*
 * GPIO pin modes
 */
#define GPIO_MODE_INPUT        0
#define GPIO_MODE_OUTPUT       1
#define GPIO_MODE_ALTFN        2
#define GPIO_MODE_ANALOG       3

/* Interrupt modes (handled via EXTI, not CRL/CRH) */
#define GPIO_MODE_IT_FT        4   // falling edge
#define GPIO_MODE_IT_RT        5   // rising edge
#define GPIO_MODE_IT_RFT       6   // rising + falling

/*
 * GPIO output types
 */
#define GPIO_OP_TYPE_PP        0   // Push-pull
#define GPIO_OP_TYPE_OD        1   // Open-drain

/*
 * GPIO output speeds (STM32F1)
 * These map directly to MODE bits
 */
#define GPIO_SPEED_10MHZ       1   // MODE = 01
#define GPIO_SPEED_2MHZ        2   // MODE = 10
#define GPIO_SPEED_50MHZ       3   // MODE = 11

/*
 * GPIO pull-up / pull-down configuration
 */
#define GPIO_NO_PUPD           0
#define GPIO_PIN_PU            1
#define GPIO_PIN_PD            2


/* APIs supported by this driver */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
void GPIO_PeriphClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi);
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F1XX_GPIO_H_ */
