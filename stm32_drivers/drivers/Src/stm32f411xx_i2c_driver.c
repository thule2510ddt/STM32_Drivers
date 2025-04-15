/*
 * stm32f411xx_i2c_driver.c
 *
 *  Created on: Apr 14, 2025
 *      Author: lmthu
 */
#include "stm32f411xx_i2c_driver.h"

static void I2C_GenerateStartCondition(I2C_TypeDef *I2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_TypeDef *I2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_TypeDef *I2Cx, uint8_t SlaveAddr);

static void I2C_GenerateStartCondition(I2C_TypeDef *I2Cx){
	I2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_TypeDef *I2Cx, uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); /* SlaveAddr is Slave address + R/NW bit = 0 */
	I2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_TypeDef *I2Cx, uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1; /* SlaveAddr is Slave address + R/NW bit = 0 */
	I2Cx->DR = SlaveAddr;
}

static void I2C_CLearADDRFlag(I2C_Handle_t *I2CHandle){
	uint32_t dummy_read;
	/* Check for device mode */
	if(I2CHandle->I2Cx->SR2 & (1 << I2C_SR2_MSL)){
		if(I2CHandle->TxRxState == I2C_BUSY_IN_RX){
			if(I2CHandle->RxSize == 1){
				/* First disable the ACK */
				I2C_ManageAcking(I2CHandle->I2Cx, DISABLE);

				/* Clear the ADDR flag (read SR1, read SR2) */
				dummy_read = I2CHandle->I2Cx->SR1;
				dummy_read = I2CHandle->I2Cx->SR2;
				(void)dummy_read;
			}
		}
		else{
			/* Clear the ADDR flag (read SR1, read SR2) */
			dummy_read = I2CHandle->I2Cx->SR1;
			dummy_read = I2CHandle->I2Cx->SR2;
			(void)dummy_read;
		}
	}
	else{
		/*
		 * Devcie is in slave mode
		 * Clear the ADDR flag (read SR1, read SR2)
		 *
		 * */
		dummy_read = I2CHandle->I2Cx->SR1;
		dummy_read = I2CHandle->I2Cx->SR2;
		(void)dummy_read;
	}
}

void I2C_GenerateStopCondition(I2C_TypeDef *I2Cx){
	I2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

void I2C_SlaveEnableDisableCallbackEvents(I2C_TypeDef *I2Cx, uint8_t EnOrDi){
	if(EnOrDi){
		I2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		I2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		I2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	else{
		I2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		I2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		I2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
	}
}

void I2C_PeripheralControl(I2C_TypeDef *I2Cx, uint8_t EnOrDi){
	if(EnOrDi){
		I2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else{
		I2Cx->CR1 &= ~(1 << 0);
	}
}

void I2C_PeriClockControl(I2C_TypeDef *I2Cx, uint8_t EnOrDi){
	if(EnOrDi){
		if(I2Cx == I2C1){
			I2C1_PCLK_EN();
		}
		else if(I2Cx == I2C2){
			I2C2_PCLK_EN();
		}
		else if(I2Cx == I2C3){
			I2C3_PCLK_EN();
		}
	}
	else{
		/* TODO */
	}
}

void I2C_Init(I2C_Handle_t *I2CHandle){
	uint32_t tempreg = 0;

	I2C_PeriClockControl(I2CHandle->I2Cx, ENABLE);

	tempreg |= I2CHandle->I2C_Config.I2C_AckControl << 10;
	I2CHandle->I2Cx->CR1 = tempreg;

	tempreg = 0;
	tempreg |= I2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	I2CHandle->I2Cx->OAR1 = tempreg;

	uint16_t ccr_value = 0;
	tempreg = 0;
	if(I2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		ccr_value = (RCC_Get)
	}
}



























