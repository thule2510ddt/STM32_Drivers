/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: Aug 25, 2024
 *      Author: lmthu
 */

#ifndef STM32F411XX_GPIO_DRIVER_H_
#define STM32F411XX_GPIO_DRIVER_H_

#include "stm32f411xx.h"

typedef enum {
	GPIO_PIN_RESET = 0,
	GPIO_PIN_SET
} GPIO_PinState;



#define GPIO_PIN_0       0
#define GPIO_PIN_1       1
#define GPIO_PIN_2       2
#define GPIO_PIN_3       3
#define GPIO_PIN_4       4
#define GPIO_PIN_5       5
#define GPIO_PIN_6       6
#define GPIO_PIN_7       7
#define GPIO_PIN_8       8
#define GPIO_PIN_9       9
#define GPIO_PIN_10      10
#define GPIO_PIN_11      11
#define GPIO_PIN_12      12
#define GPIO_PIN_13      13
#define GPIO_PIN_14      14
#define GPIO_PIN_15      15



#define GPIO_INPUT_MODE 			((uint16_t)0x0)
#define GPIO_OUTPUT_MODE 			((uint16_t)0x1)
#define GPIO_ALTFN_MODE				((uint16_t)0x2)
#define GPIO_ANALOG_MODE			((uint16_t)0x3)
#define GPIO_INTERRUPT_RT			((uint16_t)0x4)
#define GPIO_INTERRUPT_FT			((uint16_t)0x5)
#define GPIO_INTERRUPT_RFT			((uint16_t)0x6)

#define GPIO_PIN_NO_PUPD			((uint16_t)0x0)
#define GPIO_PIN_PU					((uint16_t)0x1)
#define GPIO_PIN_PD					((uint16_t)0x2)


#define GPIO_SPEED_LOW				((uint16_t)0x0)
#define GPIO_SPEED_MEDIUM			((uint16_t)0x1)
#define GPIO_SPEED_FAST				((uint16_t)0x2)
#define GPOI_SPEED_HIGH				((uint16_t)0x3)

#define GPIO_OP_TYPE_PP				0
#define GPIO_OP_TYPE_OD				1

typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

typedef struct {
	GPIO_TypeDef 		*GPIOx;
	GPIO_PinConfig_t	GPIO_PinConfig;
} GPIO_Handle_t;



void			HAL_GPIO_PeriClockControl(GPIO_TypeDef *GPIOx, uint8_t EnorDi);

void 			HAL_GPIO_Init(GPIO_Handle_t *GPIOHandle);
void 			HAL_GPIO_DeInit(GPIO_TypeDef *GPIOx);

GPIO_PinState 	HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void 			HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void 			HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

void 			HAL_GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void 			HAL_GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void 			HAL_GPIO_IRQHandling(uint8_t IRQNumber);
void 			printMessage(void);


#endif /* STM32F411XX_GPIO_DRIVER_H_ */
