/*
 * stm32f411xx.h
 *
 *  Created on: Aug 17, 2024
 *      Author: lmthu
 */

#ifndef STM32F411XX_H_
#define STM32F411XX_H_

#include<stddef.h>
#include<stdint.h>
#include<stdbool.h>

#define __weak __attribute__((weak))

#define ENABLE 					1
#define DISABLE 				0
#define SET						1
#define RESET					0
#define FLAG_RESET         		1
#define FLAG_SET 				0

#define NO_PR_BITS_IMPLEMENTED 	4


#define APB1_BASEADDR 			0x40000000
#define APB2_BASEADDR 			0x40010000
#define AHB1_BASEADDR 			0x40020000
#define AHB2_BASEADDR 			0x50000000


#define NVIC_ISER0          	((volatile uint32_t*) 0xE000E100)
#define NVIC_ISER1          	((volatile uint32_t*) 0xE000E104)
#define NVIC_ISER2          	((volatile uint32_t*) 0xE000E108)
#define NVIC_ISER3          	((volatile uint32_t*) 0xE000E10C)

#define NVIC_ICER0 				((volatile uint32_t*)0XE000E180)
#define NVIC_ICER1				((volatile uint32_t*)0XE000E184)
#define NVIC_ICER2  			((volatile uint32_t*)0XE000E188)
#define NVIC_ICER3				((volatile uint32_t*)0XE000E18C)

#define NVIC_IPR0 		    	((volatile uint32_t*) 0xE000E400)

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */
#define GPIOA_BASEADDR 			(AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR 			(AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 			(AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 			(AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 			(AHB1_BASEADDR + 0x1000)
#define GPIOH_BASEADDR 			(AHB1_BASEADDR + 0x1C00)
#define RCC_BASEADDR 			(AHB1_BASEADDR + 0x3800)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */
#define SPI2_BASEADDR			(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1_BASEADDR + 0x3800)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */
#define EXTI_BASEADDR 			(APB2_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR        	(APB2_BASEADDR + 0x3800)
#define SPI1_BASEADDR			(APB2_BASEADDR + 0x3000)


/*
 * peripheral definitions ( Peripheral base addresses type casted to xxx_RegDef)
 */
#define GPIOA					((GPIO_TypeDef*) GPIOA_BASEADDR)
#define GPIOB					((GPIO_TypeDef*) GPIOB_BASEADDR)
#define GPIOC					((GPIO_TypeDef*) GPIOC_BASEADDR)
#define GPIOD					((GPIO_TypeDef*) GPIOD_BASEADDR)
#define GPIOE					((GPIO_TypeDef*) GPIOE_BASEADDR)
#define GPIOH					((GPIO_TypeDef*) GPIOH_BASEADDR)

#define EXTI					((EXTI_TypeDef*) EXTI_BASEADDR)
#define RCC						((RCC_TypeDef*)	 RCC_BASEADDR)
#define SYSCFG					((SYSCFG_TypeDef*)SYSCFG_BASEADDR)

#define SPI1					((SPI_TypeDef*)SPI1_BASEADDR)
#define SPI2					((SPI_TypeDef*)SPI2_BASEADDR)
#define SPI3					((SPI_TypeDef*)SPI3_BASEADDR)
/**********************************peripheral register definition structures **********************************/
/*
 * Note : Registers of a peripheral are specific to MCU
 * e.g : Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different(more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your Device RM
 */
typedef struct {
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEED;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
} GPIO_TypeDef;

/*
 * peripheral register definition structure for RCC
 */
typedef struct {
	volatile uint32_t RCC_CR;
	volatile uint32_t RCC_PLLCFGR;
	volatile uint32_t RCC_CFGR;
	volatile uint32_t RCC_CIR;
	volatile uint32_t RCC_AHB1RSTR;
	volatile uint32_t RCC_AHB2RSTR;
	volatile uint32_t Reserved0[2];
	volatile uint32_t RCC_APB1RSTR;
	volatile uint32_t RCC_APB2RSTR;
	volatile uint32_t Reserved1[2];
	volatile uint32_t RCC_AHB1ENR;
	volatile uint32_t RCC_AHB2ENR;
	volatile uint32_t Reserved2[2];
	volatile uint32_t RCC_APB1ENR;
	volatile uint32_t RCC_APB2ENR;
	volatile uint32_t Reserved3[2];
	volatile uint32_t RCC_AHB1LPENR;
	volatile uint32_t RCC_AHB2LPENR;
	volatile uint32_t Reserved4[2];
	volatile uint32_t RCC_APB1LPENR;
	volatile uint32_t RCC_APB2LPENR;
	volatile uint32_t Reserved5[2];
	volatile uint32_t RCC_BDCR;
	volatile uint32_t RCC_CSR;
	volatile uint32_t Reserved6[2];
	volatile uint32_t RCC_SSCGR;
	volatile uint32_t RCC_PLLI2SCFGR;
	volatile uint32_t Reserved7;
	volatile uint32_t RCC_DCKCFGR;
} RCC_TypeDef;

/*
 * peripheral register definition structure for EXTI
 */
typedef struct {
	volatile uint32_t EXTI_IMR;
	volatile uint32_t EXTI_EMR;
	volatile uint32_t EXTI_RTSR;
	volatile uint32_t EXTI_FTSR;
	volatile uint32_t EXTI_SWIER;
	volatile uint32_t EXTI_PR;
} EXTI_TypeDef;

/*
 * peripheral register definition structure for SPI
 */
typedef struct
{
	volatile uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
	volatile uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
	volatile uint32_t SR;         /*!< TODO,     										Address offset: 0x08 */
	volatile uint32_t DR;         /*!< TODO,     										Address offset: 0x0C */
	volatile uint32_t CRCPR;      /*!< TODO,     										Address offset: 0x10 */
	volatile uint32_t RXCRCR;     /*!< TODO,     										Address offset: 0x14 */
	volatile uint32_t TXCRCR;     /*!< TODO,     										Address offset: 0x18 */
	volatile uint32_t I2SCFGR;    /*!< TODO,     										Address offset: 0x1C */
	volatile uint32_t I2SPR;      /*!< TODO,     										Address offset: 0x20 */
} SPI_TypeDef;

/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];    /*!< TODO , 									  Address offset: 0x08-0x14 */
	volatile uint32_t RESERVED1[2];  /*!< TODO          							  Reserved, 0x18-0x1C    	*/
	volatile uint32_t CMPCR;        /*!< TODO         								  Address offset: 0x20      */
	volatile uint32_t RESERVED2[2];  /*!<                                             Reserved, 0x24-0x28 	    */
	volatile uint32_t CFGR;         /*!< TODO                                         Address offset: 0x2C   	*/
} SYSCFG_TypeDef;

/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1 << 7))

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()	(RCC->RCC_APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()	(RCC->RCC_APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()	(RCC->RCC_APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()	(RCC->RCC_APB2ENR |= (1 << 13))

#define SYSCFG_PCLK_EN() (RCC->RCC_APB2ENR |= (1 << 14))

#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA) ? 0:\
										(x == GPIOB) ? 1:\
										(x == GPIOC) ? 2:\
										(x == GPIOD) ? 3:\
								        (x == GPIOE) ? 4:\
								        (x == GPIOH) ? 5: 0)

/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()               do{ (RCC->RCC_AHB1RSTR |= (1 << 0)); (RCC->RCC_AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->RCC_AHB1RSTR |= (1 << 1)); (RCC->RCC_AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->RCC_AHB1RSTR |= (1 << 2)); (RCC->RCC_AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->RCC_AHB1RSTR |= (1 << 3)); (RCC->RCC_AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->RCC_AHB1RSTR |= (1 << 4)); (RCC->RCC_AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->RCC_AHB1RSTR |= (1 << 7)); (RCC->RCC_AHB1RSTR &= ~(1 << 7)); }while(0)

/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: You may complete this list for other peripherals
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4
#define IRQ_NO_I2C1_EV     	31
#define IRQ_NO_I2C1_ER     	32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71

/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI15    15

/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

#include "stm32f411xx_gpio_driver.h"
#include "stm32f411xx_spi_driver.h"

#endif /* STM32F411XX_H_ */

