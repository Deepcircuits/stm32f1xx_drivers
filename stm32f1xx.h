/*
 * stm32f1xx.h
 *
 *  Created on: Jan 11, 2026
 *      Author: NAVDEEP
 */

// HEADER FILE CONTAINING ALL THE CONFIGURABLES OF THE MCU

#ifndef INC_STM32F1XX_H_
#define INC_STM32F1XX_H_
#define __v volatile

#include<stdint.h>    // because uint32_t is used

/* BASE ADDRESSES OF IMPORTANT REGISTERS*/

#define FLASH_BASEADDR               0x08000000U
#define SRAM1_BASEADDR               0x20000000U
#define SRAM                         SRAM1_BASEADDR
#define ROM                          0x1FFFF000U             /* SYSTEM MEMORY ADDRESS*/
#define RCC_BASEADDR                 0x40021000U
#define AFIO_BASEADDR                0x40010000U

#define PERIPHERAL_BASEADDR          0x40000000U             /* The address from where the peripheral's memory locations start*/
#define APB1PERIPH_BASEADDR          PERIPHERAL_BASEADDR      /*addresses of peripherals lying on the apb1 bus starts from here*/
#define APB2PERIPH_BASEADDR          0x40010000U
#define AHBPERIPH_BASEADDR           0x40020000U              /*there is only 1 AHB Bus in cortex M3 */

/* BASE ADDRESSES OF PERIPHERALS ON APB2 BUS */

#define GPIOA_BASEADDR               0x40010800U              /* GPIOA Base address = apb2 base addr + GPIOA offset address*/
#define GPIOB_BASEADDR               0x40010C00U
#define GPIOC_BASEADDR               0x40011000U
#define GPIOD_BASEADDR               0x40011400U               /* GPIO A-G (only set of gpios in f1x mcu) lie in the apb2 bus line*/
#define GPIOE_BASEADDR               0x40011800U
#define GPIOF_BASEADDR               0x40011C00U
#define GPIOG_BASEADDR               0x40012000U
#define EXTI_BASEADDR                0x40010400U
#define SPI1_BASEADDR                0x40013000U
#define USART1_BASEADDR              0x40013800U

/* BASE ADDRESSES OF PERIPHERALS ON APB1 BUS */

#define SPI2_BASEADDR                0x40003800U
#define SPI3_BASEADDR                0x40003C00U
#define USART2_BASEADDR              0x40004400U
#define USART3_BASEADDR              0x40004800U
#define UART4_BASEADDR               0x40004C00U
#define UART5_BASEADDR               0x40005000U
#define I2C1_BASEADDR                0x40005400U
#define I2C2_BASEADDR                0x40005800U

/* peripheral registers of GPIOS */

typedef struct {
	__v uint32_t CRL;                    //  typedef structs are used to define every peripheral of the GPIOs instead of using macros for every peripheral
	__v uint32_t CRH;                    //  [ sample code to access any GPIO :
	__v uint32_t IDR;                    //  GPIO_RegDef_t* GPIOA = (GPIO_RegDef_t*)GPIOA_BASEADDR;
	__v uint32_t ODR;                    //  GPIOA->CRL = 25;
	__v uint32_t BSRR;                   //  GPIOA->CRH = 69; ]
	__v uint32_t BRR;
	__v uint32_t LCKR;

} GPIO_RegDef_t;

#define GPIOA               ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB               ((GPIO_RegDef_t*)GPIOB_BASEADDR)         // Because of this definitions, instead of writing : GPIO_RegDef_t *GPIOA = (GPIO_RegDef_t*)GPIOA_BASEADDR;
#define GPIOC               ((GPIO_RegDef_t*)GPIOC_BASEADDR)         // everytime in main() , it can be substituted by GPIOA
#define GPIOD               ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE               ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF               ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG               ((GPIO_RegDef_t*)GPIOG_BASEADDR)



// STRUCTURE FOR RCC ENGINE
typedef struct {
	__v uint32_t CR;
	__v uint32_t CFGR;
	__v uint32_t CIR;
	__v uint32_t APB2RSTR;
	__v uint32_t APB1RSTR;          // suppose u want to set RCC'S APB2ENR'S 4th bit:
	__v uint32_t AHBENR;            // RCC_RegDef_t* RCC = (RCC_RegDef_t*)RCC_BASEADDR
	__v uint32_t APB2ENR;           // RCC->APB2ENR|=(1<<4);
	__v uint32_t APB1ENR;
	__v uint32_t BDCR;
	__v uint32_t CSR;
	//__v uint32_t AHBRSTR;          // uncomment this for F105/7 MCUs
	//__v uint32_t CFGR2;

}RCC_RegDef_t;

#define RCC                   ((RCC_RegDef_t*)RCC_BASEADDR)             // RCC->APB2ENR|=(1<<4);

// ENABLING CLOCK FOR GPIO PERIPHERALS

#define GPIOA_CLKEN()           (RCC->APB2ENR|=(1<<2))            // Enables clock for GPIO port A
#define GPIOB_CLKEN()           (RCC->APB2ENR|=(1<<3))            // Enables clock for GPIO port B
#define GPIOC_CLKEN()           (RCC->APB2ENR|=(1<<4))            // Enables clock for GPIO port C
#define GPIOD_CLKEN()           (RCC->APB2ENR|=(1<<5))            // Enables clock for GPIO port D
#define GPIOE_CLKEN()           (RCC->APB2ENR|=(1<<6))            // Enables clock for GPIO port E
#define GPIOF_CLKEN()           (RCC->APB2ENR|=(1<<7))            // Enables clock for GPIO port F
#define GPIOG_CLKEN()           (RCC->APB2ENR|=(1<<8))            // Enables clock for GPIO port G

// ENABLING CLOCK FOR SPI  PERIPHERALS

#define SPI1_CLKEN()            (RCC->APB2ENR|=(1<<12))            // Enables clock for SPI1 peripheral
#define SPI2_CLKEN()            (RCC->APB1ENR|=(1<<14))            // Enables clock for SPI2 peripheral
#define SPI3_CLKEN()            (RCC->APB1ENR|=(1<<15))            // Enables clock for SPI3 peripheral

// ENABLING CLOCK FOR USART  PERIPHERALS

#define USART1_CLKEN()          (RCC->APB2ENR|=(1<<14))
#define USART2_CLKEN()          (RCC->APB1ENR|=(1<<17))
#define USART3_CLKEN()          (RCC->APB1ENR|=(1<<18))

// ENABLING CLOCK FOR UART  PERIPHERALS

#define UART4_CLKEN()           (RCC->APB1ENR|=(1<<19))
#define UART5_CLKEN()           (RCC->APB1ENR|=(1<<20))

// ENABLING CLOCK FOR I2C  PERIPHERALS

#define I2C1_CLKEN()            (RCC->APB1ENR|=(1<<21))
#define I2C2_CLKEN()            (RCC->APB1ENR|=(1<<22))

//CLK DISABLING MACROS FOR GPIOS, SPI,USART,UART AND I2C PERIPHERALS

#define GPIOA_CLKDIS()          (RCC->APB2ENR&= ~(1<<2))            // Disables clock for GPIO port A
#define GPIOB_CLKDIS()          (RCC->APB2ENR&= ~(1<<3))            // Disables clock for GPIO port B
#define GPIOC_CLKDIS()          (RCC->APB2ENR&= ~(1<<4))            // Disables clock for GPIO port C
#define GPIOD_CLKDIS()          (RCC->APB2ENR&= ~(1<<5))            // Disables clock for GPIO port D
#define GPIOE_CLKDIS()          (RCC->APB2ENR&= ~(1<<6))            // Disables clock for GPIO port E
#define GPIOF_CLKDIS()          (RCC->APB2ENR&= ~(1<<7))            // Disables clock for GPIO port F
#define GPIOG_CLKDIS()          (RCC->APB2ENR&= ~(1<<8))            // Disables clock for GPIO port G

#define SPI1_CLKDIS()            (RCC->APB2ENR&= ~(1<<12))          // Disables clock for SPI Peripherals
#define SPI2_CLKDIS()            (RCC->APB1ENR&= ~(1<<14))
#define SPI3_CLKDIS()            (RCC->APB1ENR&= ~(1<<15))

#define USART1_CLKDIS()          (RCC->APB2ENR&= ~(1<<14))
#define USART2_CLKDIS()          (RCC->APB1ENR&= ~(1<<17))
#define USART3_CLKDIS()          (RCC->APB1ENR&= ~(1<<18))

#define UART4_CLKDIS()           (RCC->APB1ENR&= ~(1<<19))
#define UART5_CLKDIS()           (RCC->APB1ENR&= ~(1<<20))

#define I2C1_CLKDIS()            (RCC->APB1ENR&= ~(1<<21))
#define I2C2_CLKDIS()            (RCC->APB1ENR&= ~(1<<22))

//MACROS TO RESET REGISTERS

#define GPIOA_REG_RST()          do{(RCC->APB2RSTR|= (1<<2)); (RCC->APB2RSTR&= ~(1<<2)); } while(0)               // Do while loop is used to set the reset bit to 1,
#define GPIOB_REG_RST()          do{(RCC->APB2RSTR|= (1<<3)); (RCC->APB2RSTR&= ~(1<<3)); } while(0)               //  and then set it back to zero so that the gpio doesn't
#define GPIOC_REG_RST()          do{(RCC->APB2RSTR|= (1<<4)); (RCC->APB2RSTR&= ~(1<<4)); } while(0)               //  turn off forever
#define GPIOD_REG_RST()          do{(RCC->APB2RSTR|= (1<<5)); (RCC->APB2RSTR&= ~(1<<5)); } while(0)
#define GPIOE_REG_RST()          do{(RCC->APB2RSTR|= (1<<6)); (RCC->APB2RSTR&= ~(1<<6)); } while(0)
#define GPIOF_REG_RST()          do{(RCC->APB2RSTR|= (1<<7)); (RCC->APB2RSTR&= ~(1<<7)); } while(0)
#define GPIOG_REG_RST()          do{(RCC->APB2RSTR|= (1<<8)); (RCC->APB2RSTR&= ~(1<<8)); } while(0)

#define SPI1_REG_RST()           do{(RCC->APB2RSTR|= (1<<12)); (RCC->APB2RSTR&= ~(1<<12)); } while(0)
#define SPI2_REG_RST()           do{(RCC->APB1RSTR|= (1<<14)); (RCC->APB1RSTR&= ~(1<<14)); } while(0)
#define SPI3_REG_RST()           do{(RCC->APB1RSTR|= (1<<15)); (RCC->APB1RSTR&= ~(1<<15)); } while(0)

#define I2C1_REG_RST()           do{(RCC->APB1RSTR|= (1<<21)); (RCC->APB1RSTR&= ~(1<<21)); } while(0)
#define I2C2_REG_RST()           do{(RCC->APB1RSTR|= (1<<22)); (RCC->APB1RSTR&= ~(1<<22)); } while(0)

#define USART1_REG_RST()         do{(RCC->APB2RSTR|= (1<<14)); (RCC->APB2RSTR&= ~(1<<14)); } while(0)
#define USART2_REG_RST()         do{(RCC->APB1RSTR|= (1<<17)); (RCC->APB1RSTR&= ~(1<<17)); } while(0)
#define USART3_REG_RST()         do{(RCC->APB1RSTR|= (1<<18)); (RCC->APB1RSTR&= ~(1<<18)); } while(0)
#define UART4_REG_RST()         do{(RCC->APB1RSTR|= (1<<19)); (RCC->APB1RSTR&= ~(1<<19)); } while(0)
#define UART5_REG_RST()         do{(RCC->APB1RSTR|= (1<<20)); (RCC->APB1RSTR&= ~(1<<20)); } while(0)

//peripheral registers of AFIO

typedef struct{
		__v uint32_t EVCR;
		__v uint32_t MAPR;
		__v uint32_t EXTICR[4];
		__v uint32_t MAPR2;

	} AFIO_RegDef_t;

#define AFIO                      ((AFIO_RegDef_t*)AFIO_BASEADDR)

// EXTI ENGINE
typedef struct {
	__v uint32_t IMR;
	__v uint32_t EMR;
	__v uint32_t RTSR;
	__v uint32_t FTSR;
	__v uint32_t SWIER;
	__v uint32_t PR;

} EXTI_RegDef_t;

#define EXTI                     ((EXTI_RegDef_t*)EXTI_BASEADDR)

#define GPIO_BASEADDR_TO_CODE(x)  \
        ((x == GPIOA) ? 0 : \
         (x == GPIOB) ? 1 : \
         (x == GPIOC) ? 2 : \
         (x == GPIOD) ? 3 : \
         (x == GPIOE) ? 4 : 0)


// NVIC REGISTERS

#define NVIC_ISER0                    ((__v uint32_t*)0xE000E100)        //ISER->INTERRUPT SET ENABLE REGISTER
#define NVIC_ISER1                    ((__v uint32_t*)0xE000E104)
#define NVIC_ISER2                    ((__v uint32_t*)0xE000E108)
#define NVIC_ISER3                    ((__v uint32_t*)0xE000E10C)

#define NVIC_ICER0                    ((__v uint32_t*)0XE000E180)        //INTERRUPT CLEAR ENABLE REGISTERS
#define NVIC_ICER1                    ((__v uint32_t*)0XE000E184)
#define NVIC_ICER2                    ((__v uint32_t*)0XE000E188)
#define NVIC_ICER3                    ((__v uint32_t*)0XE000E18C)

#define NVIC_ISPR0                    ((__v uint32_t*)0XE000E200)        //INTERRUPT SET PENDING REGISTERS
#define NVIC_ISPR1                    ((__v uint32_t*)0XE000E204)
#define NVIC_ISPR2                    ((__v uint32_t*)0XE000E208)
#define NVIC_ISPR3                    ((__v uint32_t*)0XE000E20C)

#define NVIC_ICPR0                    ((__v uint32_t*)0XE000E28O)        //INTERRUPT CLEAR PENDING REGISTERS
#define NVIC_ICPR1                    ((__v uint32_t*)0XE000E284)
#define NVIC_ICPR2                    ((__v uint32_t*)0XE000E288)
#define NVIC_ICPR3                    ((__v uint32_t*)0XE000E28C)

#define NVIC_PR_BASE_ADDR             ((__v uint32_t*)0xE000E400)
#define NVIC_IPR0                           NVIC_PR_BASE_ADDR
#define NVIC_IPR1                     ((__v uint32_t*)0XE000E404)
#define NVIC_IPR2                     ((__v uint32_t*)0XE000E408)
#define NVIC_IPR3                     ((__v uint32_t*)0XE000E40C)

#define NO_PR_BITS_IMPLEMENTED                     4


#define IRQ_NO_WWDG               0
#define IRQ_NO_PVD                1
#define IRQ_NO_TAMPER             2
#define IRQ_NO_RTC                3
#define IRQ_NO_FLASH              4
#define IRQ_NO_RCC                5

/* EXTI */
#define IRQ_NO_EXTI0              6
#define IRQ_NO_EXTI1              7
#define IRQ_NO_EXTI2              8
#define IRQ_NO_EXTI3              9
#define IRQ_NO_EXTI4              10
#define IRQ_NO_EXTI9_5            23
#define IRQ_NO_EXTI15_10          40

/* DMA */
#define IRQ_NO_DMA1_CH1           11
#define IRQ_NO_DMA1_CH2           12
#define IRQ_NO_DMA1_CH3           13
#define IRQ_NO_DMA1_CH4           14
#define IRQ_NO_DMA1_CH5           15
#define IRQ_NO_DMA1_CH6           16
#define IRQ_NO_DMA1_CH7           17
#define IRQ_NO_DMA2_CH1           56
#define IRQ_NO_DMA2_CH2           57
#define IRQ_NO_DMA2_CH3           58
#define IRQ_NO_DMA2_CH4_5         59

/* ADC */
#define IRQ_NO_ADC1_2             18

/* CAN */
#define IRQ_NO_CAN_TX             19
#define IRQ_NO_CAN_RX0            20
#define IRQ_NO_CAN_RX1            21
#define IRQ_NO_CAN_SCE            22

/* TIMERS */
#define IRQ_NO_TIM1_BRK           24
#define IRQ_NO_TIM1_UP            25
#define IRQ_NO_TIM1_TRG_COM       26
#define IRQ_NO_TIM1_CC            27
#define IRQ_NO_TIM2               28
#define IRQ_NO_TIM3               29
#define IRQ_NO_TIM4               30
#define IRQ_NO_TIM5               50
#define IRQ_NO_TIM6               54
#define IRQ_NO_TIM7               55
#define IRQ_NO_TIM8_BRK           43
#define IRQ_NO_TIM8_UP            44
#define IRQ_NO_TIM8_TRG_COM       45
#define IRQ_NO_TIM8_CC            46

/* I2C */
#define IRQ_NO_I2C1_EV            31
#define IRQ_NO_I2C1_ER            32
#define IRQ_NO_I2C2_EV            33
#define IRQ_NO_I2C2_ER            34

/* SPI */
#define IRQ_NO_SPI1               35
#define IRQ_NO_SPI2               36
#define IRQ_NO_SPI3               51

/* USART / UART */
#define IRQ_NO_USART1             37
#define IRQ_NO_USART2             38
#define IRQ_NO_USART3             39
#define IRQ_NO_UART4              52
#define IRQ_NO_UART5              53

/* USB */
#define IRQ_NO_USB_HP_CAN_TX      19
#define IRQ_NO_USB_LP_CAN_RX0     20
#define IRQ_NO_USB_WAKEUP         42

/* RTC Alarm */
#define IRQ_NO_RTC_ALARM          41


// Some essential misc. macros

#define ENABLE                    1
#define DISABLE                   0
#define SET                       ENABLE
#define RESET                     DISABLE
#define GPIO_PIN_SET              SET
#define GPIO_PIN_RESET            RESET
#define FLAG_SET                  SET
#define FLAG_RESET                RESET

//*********************************************** ESSENTIAL MACROS FOR SPI DRIVER*******************************************************//

typedef struct{
	__v uint32_t CR1;
	__v uint32_t CR2;
	__v uint32_t SR; // status reg
	__v uint32_t DR; // data reg
	__v uint32_t CRCPR;
	__v uint32_t RXCRCR;
	__v uint32_t TXCRCR;
	__v uint32_t I2SCFGR;
	__v uint32_t I2SPR;

	}SPI_RegDef_t;

#define SPI1                 ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2                 ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3                 ((SPI_RegDef_t*)SPI3_BASEADDR)

#define SPI_CR1_CPHA         0
#define SPI_CR1_CPOL         1
#define SPI_CR1_MSTR         2
#define SPI_CR1_BR           3
#define SPI_CR1_SPE          6
#define SPI_CR1_LSBFIRST     7
#define SPI_CR1_SSI          8
#define SPI_CR1_SSM          9
#define SPI_CR1_RXONLY       10
#define SPI_CR1_DFF          11
#define SPI_CR1_CRCNEXT      12
#define SPI_CR1_CRCEN        13
#define SPI_CR1_BIDIOE       14
#define SPI_CR1_BIDIMODE     15

#define SPI_CR2_RXDMAEN       0
#define SPI_CR2_TXDMAEN       1
#define SPI_CR2_SSOE          2
#define SPI_CR2_ERRIE         5
#define SPI_CR2_RXNEIE        6
#define SPI_CR2_TXEIE         7

#define SPI_SR_RXNE           0
#define SPI_SR_TXE            1
#define SPI_SR_CHSIDE         2
#define SPI_SR_UDR            3
#define SPI_SR_CRCERR         4
#define SPI_SR_MODF           5
#define SPI_SR_OVR            6
#define SPI_SR_BSY            7

#define SPI_TXE_FLAG          (1<<SPI_SR_TXE)
#define SPI_RXNE_FLAG         (1<<SPI_SR_RXNE)
#define SPI_BUSY_FLAG         (1<<SPI_SR_BSY)

/*************I2C DRIVER REQUIREMENTS*********************/

typedef struct{
	__v uint32_t CR1;
	__v uint32_t CR2;
	__v uint32_t OAR1;
	__v uint32_t OAR2;
	__v uint32_t DR;
	__v uint32_t SR1;
	__v uint32_t SR2;
	__v uint32_t CCR;
	__v uint32_t TRISE;

}I2C_RegDef_t;

#define I2C1                ((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2                ((I2C_RegDef_t*) I2C2_BASEADDR)

#define I2C_CR1_PE              0
#define I2C_CR1_SMBUS           1
#define I2C_CR1_SMBTYPE			3
#define I2C_CR1_ENGC			6
#define I2C_CR1_NOSTRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK			   10
#define I2C_CR1_POS			   11
#define I2C_CR1_PEC			   12
#define I2C_CR1_ALERT		   13
#define I2C_CR1_SWRST		   15

#define I2C_CR2_FREQ            0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN 	   10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15

/*********************************** USART DRIVER REQUIREMENTS *********************************************/

typedef struct {
	__v uint32_t SR;
	__v uint32_t DR;
	__v uint32_t BRR;
	__v uint32_t CR1;
	__v uint32_t CR2;
	__v uint32_t CR3;
	__v uint32_t GTPR;

}USART_RegDef_t;

#define USART1        ((USART_RegDef_t*) USART1_BASEADDR )             // (USART_RegDef_t*) USART= (USART_RegDef_t*) USART_BASEADDR;
#define USART2		  ((USART_RegDef_t*) USART2_BASEADDR )             // USART->SR=1;
#define USART3        ((USART_RegDef_t*) USART3_BASEADDR )
#define UART4		  ((USART_RegDef_t*) UART4_BASEADDR  )
#define UART5         ((USART_RegDef_t*) UART5_BASEADDR  )

/******************************************************************************************
                     *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
//#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CLKEN                     11
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9















#include<string.h>
#include"stm32f1xx_gpio.h"
#include "stm32f1xx_spi.h"
#include "stm32f1xx_i2c.h"
#include "stm32f1xx_usart.h"
#include "stm32f1xx_rcc.h"
#endif /* INC_STM32F1XX_H_ */

