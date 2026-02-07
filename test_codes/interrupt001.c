#include "stm32f1xx.h"
#include <string.h>

void delay(void){
		for(int i=0;i<200000;i++);
	}

int main(void)
{

    GPIO_Handle_t led;
    GPIO_Handle_t input;

    memset(&led, 0, sizeof(led));
    memset(&input, 0, sizeof(input));

    /* PC13 → LED (active-LOW) */
    led.pGPIOx = GPIOC;
    led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
    led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
    led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_10MHZ;

    /* PA0 → Input with interrupt on BOTH edges */
    input.pGPIOx = GPIOA;
    input.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
    input.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
    input.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;

    GPIO_Init(&led);
    GPIO_Init(&input);

    /* Start with LED OFF */
    GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_13,SET);

    /* Enable NVIC for EXTI0 */
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

    while(1);
    // {
   //  /* CPU waits here, interrupt does the work */
  //  }
}

void EXTI0_IRQHandler(void){           // EXTI0_IRQHandler is obtained from startup file. This line overrides the inbuilt IRQ handler to user defined IRQ Handler{
    GPIO_IRQHandling(GPIO_PIN_0);   // clear EXTI pending bit


   if(GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_0) == SET)
    {
        GPIO_ToggleOutputPin(GPIOC,GPIO_PIN_13);
       // delay();
    }
/*else
    {
        GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   // LED OFF
    }*/
}
