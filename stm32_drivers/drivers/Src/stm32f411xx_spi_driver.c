/*
 * stm32f411xx_spi_driver.c
 *
 *  Created on: Feb 19, 2025
 *      Author: lmthu
 */


#include "stm32f411xx_spi_driver.h"

static void  spi_txe_interrupt_handle(SPI_Handle_t *SPIHandle);
static void  spi_rxne_interrupt_handle(SPI_Handle_t *SPIHandle);
static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *SPIHandle);

void SPI_PeriClockControl(SPI_TypeDef *SPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		if(SPIx == SPI1){
			SPI1_PCLK_EN();
		}
		else if(SPIx == SPI2){
			SPI2_PCLK_EN();
		}
		else if(SPIx == SPI3){
			SPI3_PCLK_EN();
		}
	}
	else{
		// To do
	}
}

void SPI_Init(SPI_Handle_t *SPIHandle){
	SPI_PeriClockControl(SPIHandle->SPIx, ENABLE);
	uint32_t tempreg = 0;

	/* 1. Configure Device Mode */
	tempreg |= SPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

	/* 2. Configure the Bus Configuration */
	if(SPIHandle->SPI_Config.SPI_DeviceMode == SPI_BUS_CONFIG_FD){
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(SPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(SPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	/* 3. Configure the SPI Serial Clock Speed (Baud rate) */
	tempreg |= SPIHandle->SPI_Config.SPI_SclSpeed << SPI_CR1_BR;

	/* 4. Configure the DFF */
	tempreg |= SPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;

	/* 3. Configure the CPOL */
	tempreg |= SPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	/* 3. Configure the CPHA */
	tempreg |= SPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;
	tempreg |= SPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

	SPIHandle->SPIx->CR1 = tempreg;
}

void SPI_Denit(SPI_TypeDef *SPIx){
	// To do
}

uint8_t SPI_GetFlagStatus(SPI_TypeDef *SPIx, uint32_t FlagName){
	if(SPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void SPI_SendData(SPI_TypeDef *SPIx, uint8_t *TxBuffer, uint32_t Length){
	while(Length > 0){
		/* 1. Wait until TXE is set */
		while(SPI_GetFlagStatus(SPIx, SPI_TXE_FLAG) == FLAG_RESET);

		/* 2. Check the DFF bit in CR1 */
		if(SPIx->CR1 & (1 << SPI_CR1_DFF)){
			/* 16 Bit DFF */
			/* 1. Load the data in to the DR */
			SPIx->DR = *((uint16_t*)TxBuffer);
			Length--;
			Length--;
			(uint16_t*)TxBuffer++;
		}
		else{
			/* 8 Bit DFF */
			SPIx->DR = *TxBuffer;
			Length--;
			TxBuffer++;
		}
	}
}

void SPI_ReceiveData(SPI_TypeDef *SPIx, uint8_t *RxBuffer, uint32_t Length){
	while(Length > 0){
		/* 1. Wait until RXNE is set */
		while(SPI_GetFlagStatus(SPIx, SPI_RXXE_FLAG) == (uint8_t)FLAG_RESET);

		/*2. Check the DFF bit in CR1*/
		if((SPIx->CR1 & (1 << SPI_CR1_DFF))){
			/* 16 bit DFF */
			/* 1. Load the data from DR to RX Buffer Address */
//			*((uint16_t*)RxBuffer) == SPIx->DR;
			uint16_t* rxBuf16  = (uint16_t*)RxBuffer;
			*rxBuf16  = SPIx->DR;
			Length--;
			Length--;
			(uint16_t*)RxBuffer++;
		}
		else{
			/* 8 Bit DFF */
			*(RxBuffer) = SPIx->DR;
			Length--;
			RxBuffer++;
		}
	}
}

void SPI_SSIConfig(SPI_TypeDef *SPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		SPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else{
		SPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_TypeDef *SPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		SPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else{
		SPIx->CR2 &= ~ (1 << SPI_CR2_SSOE);
	}
}

void SPI_IRQInterrupt(uint8_t IRQNumber, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		if(IRQNumber <= 31){
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64){
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 94){
			*NVIC_ISER3 |= (1 << (IRQNumber % 64));
		}
	}
	else{
		if(IRQNumber <= 31){
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64){
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 94){
			*NVIC_ICER3 |= (1 << (IRQNumber % 64));
		}
	}
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	/* 1. First lets find out the ipr register */
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section =IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR0 + iprx) |= (IRQPriority << shift_amount);
}

uint8_t SPI_SendDataIT(SPI_Handle_t *SPIHandle, uint8_t *TxBuffer, uint32_t Length){
	uint8_t state = SPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX){
		/* 1. Save the TX Buffer Address and Length information in some global variables */
		SPIHandle->TXBuffer = TxBuffer;
		SPIHandle->TxLength = Length;

		/* 2. Mark the SPI state as busy in transmission so that
		 * no other code can take over same SPI peripheral until transmission is over
		 *  */
		SPIHandle->TxState = SPI_BUSY_IN_TX;

		/* 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR */
		SPIHandle->SPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *SPIHandle, uint8_t *TxBuffer, uint32_t Length){
	uint8_t state = SPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX){
		/* 1. Save the RX Buffer Address and Length information in some global variables */
		SPIHandle->RXBuffer = TxBuffer;
		SPIHandle->RxLength = Length;

		/* 2. Mark the SPI state as busy in reception so that
		 * no other code can take over same SPI peripheral until reception is over
		 *  */
		SPIHandle->RxState = SPI_BUSY_IN_RX;

		/* 3. Enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR */
		SPIHandle->SPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
}


void SPI_IRQHandling(SPI_Handle_t *Handle){
	uint8_t temp1;
	uint8_t temp2;

	/* 1. Lets check for TXE */
	temp1 = Handle->SPIx->SR & (1 << SPI_SR_TXE);
	temp2 = Handle->SPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 & temp2){
		/* 2. Handle TXE */
		spi_txe_interrupt_handle(Handle);
	}

	/* 3. Lets check for RXNE */
	temp1 = Handle->SPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = Handle->SPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 & temp2){
		/* 2. Handle TXE */
		spi_rxne_interrupt_handle(Handle);
	}

	/* 3. Lets check for RXNE */
	temp1 = Handle->SPIx->SR & (1 << SPI_SR_OVR);
	temp2 = Handle->SPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 & temp2){
		/* 2. Handle TXE */
		spi_ovr_err_interrupt_handle(Handle);
	}
}

void SPI_CloseTransmission(SPI_Handle_t *SPIHandle){
	SPIHandle->SPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	SPIHandle->TXBuffer = NULL;
	SPIHandle->TxLength = 0;
	SPIHandle->RxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *SPIHandle){
	SPIHandle->SPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	SPIHandle->RXBuffer = NULL;
	SPIHandle->RxLength = 0;
	SPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_TypeDef *SPIx){
	uint8_t temp;
	temp = SPIx->DR;
	temp = SPIx->SR;
	(void)temp;
}

static void spi_txe_interrupt_handle(SPI_Handle_t *SPIHandle){
	/* 1. Check the DFF bit in CR1 */
	if(SPIHandle->SPIx->CR1 & (1 << SPI_CR1_DFF)){
		/* 16 Bit DFF
		 * Load the data in to the DR
		 * */
		SPIHandle->SPIx->DR = *((uint16_t*)SPIHandle->TXBuffer);
		SPIHandle->TxLength--;
		SPIHandle->TxLength--;
		(uint16_t*)SPIHandle->TXBuffer++;
	}
	else{
		/* 16 Bit DFF
		 * Load the data in to the DR
		 * */
		SPIHandle->SPIx->DR = *((uint16_t*)SPIHandle->TXBuffer);
		SPIHandle->TxLength--;
		(uint16_t*)SPIHandle->TXBuffer++;
	}
	if(!SPIHandle->TxLength){
		/*
		 * TX Length is zero, so close the SPI transmission and inform the application that
		 * TX is over
		 * This prevents interrupts from setting up of TXE flag
		 * */
		SPI_CloseTransmission(SPIHandle);
		SPI_ApplicationEventCallback(SPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *SPIHandle){
	/* 1. Check the DFF bit in CR1 */
	if(SPIHandle->SPIx->CR1 & (1 << SPI_CR1_DFF)){
		/* 16 Bit DFF
		 * Load the data in to the DR
		 * */
		SPIHandle->SPIx->DR = *((uint16_t*)SPIHandle->RXBuffer);
		SPIHandle->TxLength -= 2;
		SPIHandle->RXBuffer++;
		SPIHandle->RXBuffer++;
	}
	else{
		/* 16 Bit DFF
		 * Load the data in to the DR
		 * */
		SPIHandle->SPIx->DR = *((uint16_t*)SPIHandle->RXBuffer);
		SPIHandle->RxLength--;
		(uint16_t*)SPIHandle->RXBuffer++;
	}
	if(!SPIHandle->RxLength){
		/*
		 * TX Length is zero, so close the SPI transmission and inform the application that
		 * TX is over
		 * This prevents interrupts from setting up of TXE flag
		 * */
		SPI_CloseTransmission(SPIHandle);
		SPI_ApplicationEventCallback(SPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *SPIHandle){
	uint8_t temp;
	/* 1. Clear the ovr flag */
	if(SPIHandle->TxState != SPI_BUSY_IN_TX){
		temp = SPIHandle->SPIx->DR;
		temp = SPIHandle->SPIx->SR;
	}
	(void)temp;

	/* 2. Inform the application */
	SPI_ApplicationEventCallback(SPIHandle, SPI_EVENT_OVR_ERR);
}



__weak void SPI_ApplicationEventCallback(SPI_Handle_t *SPIHandle, uint8_t AppEv){
	/*
	 * This is a weak implementation, the user application may override this function.
	 * */
}





























