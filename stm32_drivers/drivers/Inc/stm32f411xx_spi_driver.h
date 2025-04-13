/*
 * stm32f411xx_spi_driver.h
 *
 *  Created on: Feb 19, 2025
 *      Author: lmthu
 */

#ifndef INC_STM32F411XX_SPI_DRIVER_H_
#define INC_STM32F411XX_SPI_DRIVER_H_

#include "stm32f411xx.h"

/*
 * Configuration structure for SPIx peripheral
 *
 * */
typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 *
 *
 * */
typedef struct {
	SPI_TypeDef *SPIx;
	SPI_Config_t SPI_Config;
	uint8_t 	 *TXBuffer;
	uint8_t 	 *RXBuffer;
	uint32_t     TxLength;
	uint32_t 	 RxLength;
	uint8_t 	 TxState;
	uint8_t 		 RxState;
} SPI_Handle_t;

/*
 * SPI application states
 * */
#define SPI_READY			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX 		2

/*
 * Possible SPI Application events
 *
 * */
#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_ERR	3
#define SPI_EVENT_CRC_ERR	4

/*
 * SPI Device Mode
 * */
#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE	0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD                1
#define SPI_BUS_CONFIG_HD                2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY    3


/*
 * SPI SCL Speed
 * */
#define SPI_SCLK_SPEED_DIV2     0
#define SPI_SCLK_SPEED_DIV4     1
#define SPI_SCLK_SPEED_DIV8     2
#define SPI_SCLK_SPEED_DIV16    3
#define SPI_SCLK_SPEED_DIV32    4
#define SPI_SCLK_SPEED_DIV64    5
#define SPI_SCLK_SPEED_DIV128   6
#define SPI_SCLK_SPEED_DIV256   7

/*
 * SPI DFF
 * */
#define SPI_DFF_8BITS 			0
#define SPI_DFF_16BITS 			1

/*
 * SPI CPOL
 * */
#define SPI_CPOL_HIGH 			1
#define SPI_CPOL_LOW 			0

/*
 * SPI CPHA
 * */
#define SPI_CPHA_HIGH 			1
#define SPI_CPHA_LOW 			0

/*
 * SPI SSM
 *
 * */
#define SPI_SSM_EN				1
#define SPI_SSM_DI				0

/*
 * SPI related status flags definitions
 *
 * */
#define SPI_TXE_FLAG			(1 << SPI_SR_TXE)
#define SPI_RXXE_FLAG			(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG			(1 << SPI_SR_BSY)

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/

/*
 * Peripheral Clock setup
 *
 * */
void SPI_PeriClockControl(SPI_TypeDef *SPIx, uint8_t EnOrDi);

/*
 * Init and De-init
 *
 * */
void SPI_Init(SPI_Handle_t *SPIHandle);
void SPI_DeInit(SPI_TypeDef *SPIx);


/*
 * Data Send and Receive
 *
 * */
void SPI_SendData(SPI_TypeDef *SPIx, uint8_t *TxBuffer, uint32_t Length);
void SPI_ReceiveData(SPI_TypeDef *SPIx, uint8_t *RxBuffer, uint32_t Length);

uint8_t SPI_SendDataIT(SPI_Handle_t *SPIx, uint8_t *TxBuffer, uint32_t Length);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *SPIx, uint8_t *RxBuffer, uint32_t Length);



/*
 * IRQ Configuration and ISR Handling
 *
 * */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *Handle);

/*
 * Other Peripheral Control APIs
 *
 * */
void SPI_PeripheralControl(SPI_TypeDef *SPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_TypeDef *SPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_TypeDef *SPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_TypeDef *SPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_TypeDef *SPIx);
void SPO_CloseTransmission(SPI_Config_t *SPIHandle);
void SPI_CloseReception(SPI_Handle_t *SPI_Handle);


/*
 * Application callback
 *
 *
 * */
void SPI_ApplicationEventCallback(SPI_Handle_t *SPIHandle, uint8_t AppEv);


#endif /* INC_STM32F411XX_SPI_DRIVER_H_ */
