#include "stm32f1xx.h"

/*
 * AHB prescaler lookup table
 * Index = HPRE[3:0] - 8
 */
static uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};

/*
 * APB prescaler lookup table
 * Index = PPRE[2:0] - 4
 */
static uint8_t APB_PreScaler[4] = {2,4,8,16};


/*********************************************************************
 * @fn      		  - RCC_GetPLLOutputClock
 * @brief             - Returns PLL clock frequency
 *********************************************************************/
uint32_t RCC_GetPLLOutputClock(void)
{
    uint32_t pllclk, pllmul, pllsrc;

    pllsrc = (RCC->CFGR >> 16) & 0x1;      // PLLSRC
    pllmul = (RCC->CFGR >> 18) & 0xF;      // PLLMUL

    if(pllsrc == 0)
    {
        // HSI / 2 selected as PLL input
        pllclk = 4000000;
    }
    else
    {
        // HSE selected as PLL input
        pllclk = 8000000;
    }

    pllmul += 2;   // As per RM0008 (0000 = x2)

    return pllclk * pllmul;
}


/*********************************************************************
 * @fn      		  - RCC_GetPCLK1Value
 * @brief             - Returns APB1 peripheral clock
 *********************************************************************/
uint32_t RCC_GetPCLK1Value(void)
{
    uint32_t systemclk, pclk1;
    uint8_t clksrc, temp;
    uint16_t ahbp = 1, apb1p = 1;

    clksrc = (RCC->CFGR >> 2) & 0x3;

    if(clksrc == 0)
    {
        systemclk = 8000000;     // HSI
    }
    else if(clksrc == 1)
    {
        systemclk = 8000000;     // HSE (assumed 8 MHz)
    }
    else
    {
        systemclk = RCC_GetPLLOutputClock();
    }

    // AHB prescaler
    temp = (RCC->CFGR >> 4) & 0xF;
    if(temp >= 8)
    {
        ahbp = AHB_PreScaler[temp - 8];
    }

    // APB1 prescaler
    temp = (RCC->CFGR >> 10) & 0x7;
    if(temp >= 4)
    {
        apb1p = APB_PreScaler[temp - 4];
    }

    pclk1 = systemclk / ahbp / apb1p;
    return pclk1;
}


/*********************************************************************
 * @fn      		  - RCC_GetPCLK2Value
 * @brief             - Returns APB2 peripheral clock
 *********************************************************************/
uint32_t RCC_GetPCLK2Value(void)
{
    uint32_t systemclk, pclk2;
    uint8_t clksrc, temp;
    uint16_t ahbp = 1, apb2p = 1;

    clksrc = (RCC->CFGR >> 2) & 0x3;

    if(clksrc == 0)
    {
        systemclk = 8000000;     // HSI
    }
    else if(clksrc == 1)
    {
        systemclk = 8000000;     // HSE
    }
    else
    {
        systemclk = RCC_GetPLLOutputClock();
    }

    // AHB prescaler
    temp = (RCC->CFGR >> 4) & 0xF;
    if(temp >= 8)
    {
        ahbp = AHB_PreScaler[temp - 8];
    }

    // APB2 prescaler
    temp = (RCC->CFGR >> 13) & 0x7;
    if(temp >= 4)
    {
        apb2p = APB_PreScaler[temp - 4];
    }

    pclk2 = systemclk / ahbp / apb2p;
    return pclk2;
}
