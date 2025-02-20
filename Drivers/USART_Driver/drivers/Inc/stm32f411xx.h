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

#define ENABLE 			1
#define DISABLE 		0
#define SET				ENABLE
#define RESET			DISABLE
#define NO_PR_BITS_IMPLEMENTED 4


#define APB1_BASEADDR 		0x40000000
#define APB2_BASEADDR 		0x40010000
#define AHB1_BASEADDR 		0x40020000
#define AHB2_BASEADDR 		0x50000000


#define NVIC_ISER0          	((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1          	((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2          	((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3          	((volatile uint32_t*)0xE000E10C)

#define NVIC_IPR0 		    	((volatile uint32_t*)0xE000E400)


#define GPIOA_BASEADDR 			(AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR 			(AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 			(AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 			(AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 			(AHB1_BASEADDR + 0x1000)
#define GPIOH_BASEADDR 			(AHB1_BASEADDR + 0x1C00)

#define RCC_BASEADDR 			(AHB1_BASEADDR + 0x3800)
#define EXTI_BASEADDR 			(APB2_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR        	(APB2_BASEADDR + 0x3800)


#define GPIOA					((GPIO_TypeDef*)GPIOA_BASEADDR)
#define GPIOB					((GPIO_TypeDef*)GPIOB_BASEADDR)
#define GPIOC					((GPIO_TypeDef*)GPIOC_BASEADDR)
#define GPIOD					((GPIO_TypeDef*)GPIOD_BASEADDR)
#define GPIOE					((GPIO_TypeDef*)GPIOE_BASEADDR)
#define GPIOH					((GPIO_TypeDef*)GPIOH_BASEADDR)

#define EXTI					((EXTI_TypeDef*)EXTI_BASEADDR)
#define RCC						((RCC_TypeDef*)RCC_BASEADDR)
#define SYSCFG					((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

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

typedef struct {
	volatile uint32_t EXTI_IMR;
	volatile uint32_t EXTI_EMR;
	volatile uint32_t EXTI_RTSR;
	volatile uint32_t EXTI_FTSR;
	volatile uint32_t EXTI_SWIER;
	volatile uint32_t EXTI_PR;
} EXTI_TypeDef;

typedef struct
{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];    /*!< TODO , 									  Address offset: 0x08-0x14 */
	volatile uint32_t RESERVED1[2];  /*!< TODO          							  Reserved, 0x18-0x1C    	*/
	volatile uint32_t CMPCR;        /*!< TODO         								  Address offset: 0x20      */
	volatile uint32_t RESERVED2[2];  /*!<                                             Reserved, 0x24-0x28 	    */
	volatile uint32_t CFGR;         /*!< TODO                                         Address offset: 0x2C   	*/
} SYSCFG_RegDef_t;

#define GPIOA_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()			(RCC->RCC_AHB1ENR |= (1 << 7))

#define SYSCFG_PCLK_EN() (RCC->RCC_APB2ENR |= (1 << 14))

#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA) ? 0:\
										(x == GPIOB) ? 1:\
										(x == GPIOC) ? 2:\
										(x == GPIOD) ? 3:\
								        (x == GPIOE) ? 4:\
								        (x == GPIOH) ? 5: 0)

#define GPIOA_REG_RESET()               do{ (RCC->RCC_AHB1RSTR |= (1 << 0)); (RCC->RCC_AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->RCC_AHB1RSTR |= (1 << 1)); (RCC->RCC_AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->RCC_AHB1RSTR |= (1 << 2)); (RCC->RCC_AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->RCC_AHB1RSTR |= (1 << 3)); (RCC->RCC_AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->RCC_AHB1RSTR |= (1 << 4)); (RCC->RCC_AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->RCC_AHB1RSTR |= (1 << 7)); (RCC->RCC_AHB1RSTR &= ~(1 << 7)); }while(0)


#include "stm32f411xx_gpio_driver.h"

#endif /* STM32F411XX_H_ */

