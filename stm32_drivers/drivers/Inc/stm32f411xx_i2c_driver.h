/*
 * stm32f411xx_i2c_driver.h
 *
 *  Created on: Apr 14, 2025
 *      Author: lmthu
 */

#ifndef INC_STM32F411XX_I2C_DRIVER_H_
#define INC_STM32F411XX_I2C_DRIVER_H_

#include "stm32f411xx.h"

/*
 * Configuration structure for I2Cx peripheral
 * */
typedef struct {
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_AckControl;
	uint8_t  I2C_FMDutyCycle;
} I2C_Config_t;

/*
 * Handle structure for I2Cx peripheral
 * */
typedef struct {
	I2C_TypeDef *I2Cx;
	I2C_Config_t I2C_Config;
	uint8_t 	*TxBuffer;
	uint8_t 	*RxBuffer;
	uint32_t	TxLength;
	uint32_t	RxLength;
	uint8_t 	TxRxState;
	uint8_t		DevAddr;
	uint32_t	RxSize;
	uint8_t		Sr;
} I2C_Handle_t;

/*
 * I2C Application states
 *
 * */
#define I2C_READY				0
#define I2C_BUSY_IN_RX			1
#define I2C_BUSY_IN_TX			2

/*
 * I2C SCL Speed
 *
 * */
#define I2C_SCL_SPEED_SM		100000
#define I2C_SCL_SPEED_SM2K		200000
#define I2C_SCL_SPEED_SM4K		400000

/*
 * I2C ACK Control
 *
 * */
#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0

/*
 * I2C FM Duty Cycle
 *
 * */
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1

/*
 * I2C related status flags definitions
 *
 * */
#define	I2C_FLAG_TXE		( 1 << I2C_SR1_TXE )
#define I2C_FLAG_RXNE   	( 1 << I2C_SR1_RXNE )
#define I2C_FLAG_SB			( 1 << I2C_SR1_SB )
#define I2C_FLAG_OVR  		( 1 << I2C_SR1_OVR )
#define I2C_FLAG_AF   		( 1 << I2C_SR1_AF )
#define I2C_FLAG_ARLO 		( 1 << I2C_SR1_ARLO )
#define I2C_FLAG_BERR 		( 1 << I2C_SR1_BERR )
#define I2C_FLAG_STOPF 		( 1 << I2C_SR1_STOPF )
#define I2C_FLAG_ADD10 		( 1 << I2C_SR1_ADD10 )
#define I2C_FLAG_BTF  		( 1 << I2C_SR1_BTF )
#define I2C_FLAG_ADDR 		( 1 << I2C_SR1_ADDR )
#define I2C_FLAG_TIMEOUT 	( 1 << I2C_SR1_TIMEOUT )

#define I2C_DISABLE_SR		RESET
#define I2C_ENABLE_SR		SET

/*
 *	I2C Application events macros
 *
 * */
#define I2C_EV_TX_CMPLT			0
#define I2C_EV_RX_CMPLT			1
#define I2C_EV_STOP				2
#define I2C_ERROR_BERR			3

#define I2C_ERROR_ARLO			4
#define I2C_ERROR_AF			5
#define I2C_ERROR_OVR			6
#define I2C_ERROR_TIMEOUT		7

#define I2C_EV_DATA_REQ			8
#define I2C_EV_DATA_RCV			9

/*
 * APIs supported by this driver
 * For more information about the APIs check the function definitions
 *
 * */

/* Peripheral Clock setup */
void I2C_PeriClockControl(I2C_TypeDef *I2Cx, uint8_t EnOrDi);

/*
 * Data Send and Receive
 *
 * */
void I2C_Init(I2C_Handle_t *I2CHandle);
void I2C_DeInit(I2C_TypeDef *I2Cx);

/*
 * Data Send and Receive
 *
 * */
void I2C_MasterSendData(I2C_Handle_t *I2CHandle, uint8_t *RxBuffer, uint8_t Length, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *I2CHandle, uint8_t *RxBuffer, uint8_t Length, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *I2CHandle, uint8_t *TxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *I2CHandle, uint8_t *RxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t Sr);

void I2C_CloseReceiveData(I2C_Handle_t *I2CHandle);
void I2C_CloseSendData(I2C_Handle_t *I2CHandle);

void I2C_SlaveSendData(I2C_TypeDef *I2C, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_TypeDef *I2Cx);

/*
 * IRQ Configuration and ISR handling
 *
 * */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *I2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *I2CHandle);

/*
 * Other Peripheral Control APIs
 *
 * */
void I2C_PeripheralControl(I2C_TypeDef *I2Cx, uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_TypeDef *I2Cx, uint32_t FlagName);
void I2C_ManageAcking(I2C_TypeDef *I2Cx, uint8_t EnOrDi);
void I2C_GenerateStopCondition(I2C_TypeDef *I2Cx);

void I2C_SlaveEnableDisableCallbackEvents(I2C_TypeDef *I2Cx, uint8_t EnOrDi);

/*
 * Application callback
 *
 * */
void I2C_ApplicationEventCallback(I2C_Handle_t *I2CHandle, uint8_t AppEv);

#endif












