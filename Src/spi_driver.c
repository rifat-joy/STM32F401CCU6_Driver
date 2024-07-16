/*
 * spi_driver.c
 *
 *  Created on: Jan 24, 2023
 *      Author: rifat
 */

#include "spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*< @fn			->	SPI_ClkControl >
 * @brief		->	This function enables or disables peripheral clock for the given GPIO port
 * @param[in]	->  Base addresses of the GPIO peripheral
 * @param[in]	->	Enable and Disable macros
 * @return		-> 	none
 * @note		-> 	none
 */
void SPI_ClkControl(SPI_RegDef_t *pSPIx, uint8_t EnD){
	if (EnD == ENABLE){
		if (pSPIx == SPI1){
			SPI1_CLKEN();
		}else if (pSPIx == SPI2){
			SPI2I_I2S2_CLKEN();
		}else if (pSPIx == SPI3){
			SPI3I_I2S3_CLKEN();
		}else if (pSPIx == SPI4){
			SPI4_CLKEN();
		}
	} else {
		// GPIO Clock Disable macros
	}
}

/*<@fn			->	SPI_Init >
 * @brief		->	This function enables or disables peripheral clock for the given GPIO port
 * @param[in]	->  Base addresses of the GPIO peripheral
 * @param[in]	->	Enable and Disable macros
 * @return		-> 	none
 * @note		-> 	none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle){
	//masking SPI_CR1 register values to bitmask S
	uint32_t bitmask = 0;

	// SPIx peripheral clock enable
	SPI_ClkControl(pSPIHandle->pSPIx, ENABLE);

	//1. Configure the device mode master or slave
	bitmask |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	//2. Configuring the bus configuration
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_FullDuplex){
		//biirectional mode bit should be cleared
		bitmask &= ~(1 << SPI_CR1_BIDIMODE);
	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_HalfDuplex){
		//biirectional mode bit should be set
		bitmask |= (1 << SPI_CR1_BIDIMODE);
	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_Simplex_Rx){
		//biirectional mode bit should be cleared
		bitmask &= ~(1 << SPI_CR1_BIDIMODE);
		//RxOnly bit should be set
		bitmask |= (1 << SPI_CR1_Rx_ONLY);
	}
	//3. configure the SPI serial clock speed (Baud rate) to fPCLK/4
	bitmask |= (pSPIHandle->SPIConfig.SPI_ClockSpeed<< SPI_CR1_BR);
	//4. Configure DFF
	bitmask |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);
	//5. Configure CPOL
	bitmask |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);
	//6. Configure CPHA
	bitmask |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	bitmask |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);
	/*< "Assigning the masked register to SPIx control register" >*/
	pSPIHandle->pSPIx->CR1 = bitmask;
}

/*<@fn			->	SPI_DeInit >
 * @brief		->	This function enables or disables peripheral clock for the given GPIO port
 * @param[in]	->  Base addresses of the GPIO peripheral
 * @param[in]	->	Enable and Disable macros
 * @return		-> 	none
 * @note		-> 	none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if (pSPIx == SPI1){
		SPI1_REG_RST();
	}
	else if (pSPIx == SPI2){
		SPI2I_I2S2_REG_RST();
	}
	else if (pSPIx == SPI3){
		SPI3I_I2S3_REG_RST();
	}
	else if (pSPIx == SPI4){
		SPI4_REG_RST();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){
	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*<@fn			->	SPI_TransmitData >
 * @brief		->
 * @param[in]	->  Base addresses of the SPIx peripheral
 * @param[in]	->	Data buffer that is about to transmit
 * @param[in]	->	Length of Data buffer
 * @return		-> 	none
 * @note		-> 	This is blocking API
 */
void SPI_Transmit(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t TxLen){
	while(TxLen > 0){
		//1. wait untill TXE is set
//		while(!(pSPIx->SR & (1 << SPI_SR_TXE)));
		while(SPI_GetFlagStatus(pSPIx, SPI_FLAG_TXE) == FLAG_RESET);
		//2. Check the DFF bit in CR1
		if (pSPIx->CR1 & (1<< SPI_CR1_DFF)){
			// set 16 bit data frame format
			//1. loading the data in to the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			TxLen--; TxLen--;
			(uint16_t*)pTxBuffer++;
		}else {
			// set 8 bit data frame format
			pSPIx->DR = *pTxBuffer;
			TxLen--;
			pTxBuffer++;
		}
	}
}

/*<@fn			->	SPI_ReceiveData >
 * @brief		->
 * @param[in]	->
 * @param[in]	->
 * @return		-> 	none
 * @note		-> 	This is blocking API
 */
void SPI_Receive(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t RxLen){
	while(RxLen > 0){
		//1. wait untill RXNE is set
//		while(!(pSPIx->SR & (1 << SPI_SR_TXE)));
		while(SPI_GetFlagStatus(pSPIx, SPI_FLAG_RXNE) == FLAG_RESET);
		//2. Check the DFF bit in CR1
		if (pSPIx->CR1 & (1<< SPI_CR1_DFF)){
			// set 16 bit data frame format
			//1. loading the data from DR to pRxBuffer
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			RxLen--; RxLen--;
			(uint16_t*)pRxBuffer++;
		}else {
			// set 8 bit data frame format
			*pRxBuffer = pSPIx->DR;
			RxLen--;
			pRxBuffer++;
		}
	}
}

/*<@fn			->	SPI_IRQInterruptConfig >
 * @brief		->
 * @param[in]	->
 * @param[in]	->	Enable and Disable macros
 * @return		-> 	none
 * @note		-> 	none
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnD){
	if(EnD == ENABLE){
		if (IRQNumber <= 31){
			//Program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if (IRQNumber <= 31 && IRQNumber < 64){
			//Program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}else if (IRQNumber <= 64 && IRQNumber < 96){
			//Program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}else{
		if (IRQNumber <= 31){
			//Program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if (IRQNumber <= 31 && IRQNumber < 64){
			//Program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber%32));
		}else if (IRQNumber <= 64 && IRQNumber < 96){
			//Program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber%64));
		}
	}
}

/*< @fn			->	SPI_IRQPriorityConfig >
 * @brief		->
 * @param[in]	->
 * @param[in]	->	Enable and Disable macros
 * @return		-> 	none
 * @note		-> 	none
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLIMENTED);
	*( NVIC_PR_BASEADDR + iprx ) |= ( IRQPriority << shift_amount );
}

/*< @fn			->	 >
 * @brief		->	This API will not send anything but saves the pointer length information sets interrupy and returns
 */
uint8_t SPI_IntTransmit(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t TxLen){
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_Tx){
//	if(pSPIHandle->TxState != SPI_BUSY_IN_Tx){
		//1. Save the Tx buffer and length varieable in some golbal varieable
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->Txlen = TxLen;
		//2. Mark the SPI state as busy in transmission so that no ther code cant take over same API untill transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_Tx;
		//3. Enable the TXEIE control bit to get interrupt when TXE is set in SR
		pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_TXEIE);
		//4. Data transmission will handle by ISR
	}
	return pSPIHandle->TxState;
}

uint8_t SPI_IntReceive(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t RxLen){
	if(pSPIHandle->RxState != SPI_BUSY_IN_Rx){
		//1. Save the Tx buffer address and length information in some golbal varieable
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->Rxlen = RxLen;
		//2. Mark the SPI state as busy in transmission so that no ther code cant take over same API untill transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_Rx;
		//3. Enable the TXEIE control bit to get interrupt when TXE is set in SR
		pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_RXNEIE);
		//4. Data Recieve will handle by ISR
	}
	return pSPIHandle->RxState;
}


/*< @fn			->	SPI_IRQHandling >
 * @brief		->	This API transfers each byte in every interrupt
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){
//First we have to check why the interrupt triggered
	uint8_t temp1, temp2;
	//checking for TXE
	temp1 = pSPIHandle->pSPIx->SR & (1<<SPI_SR_TXE); // if TXE is set temp1=1
	temp2 = pSPIHandle->pSPIx->CR2 & (1<<SPI_CR2_TXEIE);
	if(temp1 && temp2){
		//Handling TXE implementing a helper function not exposing to user
		spi_txe_interrupt_handle(pSPIHandle);
	}

	//checking for RXNE
	temp1 = pSPIHandle->pSPIx->SR & (1<<SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1<<SPI_CR2_RXNEIE);
	if(temp1 && temp2){
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	//checking for OVR
	temp1 = pSPIHandle->pSPIx->SR & (1<<SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1<<SPI_CR2_ERRIE);

	if(temp1 && temp2){
		spi_ovr_interrupt_handle(pSPIHandle);
	}
}

/*< @fn			->	SPI_PeripheralControl >
 * @brief		->	This function enables or disables SPIx
 * @param[in]	->  Base addresses of the GPIO peripheral
 * @param[in]	->	Enable and Disable macros
 * @return		-> 	none
 * @note		-> 	none
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnD){
	if(EnD == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*< @fn			->	SPI_SSIConfig >
 * @brief		->	This function enables or disables SSI
 * @param[in]	->  Base addresses of the GPIO peripheral
 * @param[in]	->	Enable and Disable macros
 * @return		-> 	none
 * @note		-> 	none
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnD){
	if(EnD == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/*< @fn			->	SPI_SSOEonfig >
 * @brief		->	This function enables or disables SSOE bit
 * @param[in]	->  Base addresses of the GPIO peripheral
 * @param[in]	->	Enable and Disable macros
 * @return		-> 	none
 * @note		-> 	none
 */
void SPI_SSOEonfig(SPI_RegDef_t *pSPIx, uint8_t EnD){
	if(EnD == ENABLE){
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/* <<Implimenting helper functions>> */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){
	//2. Check the DFF bit in CR1
	if (pSPIHandle->pSPIx->CR1 & (1<< SPI_CR1_DFF)){
		// set 16 bit data frame format
		//1. loading the data in to the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->Txlen--; pSPIHandle->Txlen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else {
		// set 8 bit data frame format
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->Txlen--;
		pSPIHandle->pTxBuffer++;
	}
	if(! pSPIHandle->Txlen){
// Txlen is zero so close the communication and inform application that TX is over
		// 1. Deactivate the TXEIE bit cause we dont need any more interrupt
		SPI_CloseTransmission(pSPIHandle);
//		2. inform application through callback. Application has to implement the callback
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){
	//2. Check the DFF bit in CR1
		if (pSPIHandle->pSPIx->CR1 & (1<< SPI_CR1_DFF)){
			// set 16 bit data frame format
			//1. loading the data in to the DR
			*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
			pSPIHandle->Rxlen -= 2;
			pSPIHandle->pRxBuffer--;
			pSPIHandle->pRxBuffer--;
		}else {
			// set 8 bit data frame format
			*(pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
			pSPIHandle->Rxlen--;
			pSPIHandle->pRxBuffer--;
		}
		if(! pSPIHandle->Rxlen){
	// Txlen is zero so close the communication and inform application that TX is over
			// 1. Deactivate the TXEIE bit cause we dont need any more interrupt
			SPI_CloseReception(pSPIHandle);
	//		2. inform application through callback. Application has to implement the callback
			SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
		}
}

static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle){
	// Clear the OVR flag
	uint8_t temp;
	if(pSPIHandle->TxState != SPI_BUSY_IN_Tx){
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	// infrom the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERROR);
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &=~ (1<<SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->Txlen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &=~ (1<<SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->Rxlen = 0;
	pSPIHandle->RxState = SPI_READY;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t Event){
	// This is a weak function implementation application may change the function

}





