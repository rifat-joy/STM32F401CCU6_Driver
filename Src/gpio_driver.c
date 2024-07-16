/*
 * 	GPIO API Implementation
 *
 *  Created on: Jan 2, 2023
 *      Author: Rifat
 */

#include "gpio_driver.h"


/*< Function Documentation >
 * @fn			->	GPIO_ClkControl
 *
 * @brief		->	This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]	->  Base addresses of the GPIO peripheral
 * @param[in]	->	Enable and Disable macros
 *
 * @return		-> 	none
 *
 * @note		-> 	none
 *
 */
void GPIO_ClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnD){ /* < pointer to the base address, Enable or disable> */
	if (EnD == ENABLE){
		if (pGPIOx == GPIOA){
			GPIOA_CLKEN();
		}else if (pGPIOx == GPIOB){
			GPIOB_CLKEN();
		}else if (pGPIOx == GPIOC){
			GPIOC_CLKEN();
		}else if (pGPIOx == GPIOD){
			GPIOD_CLKEN();
		}else if (pGPIOx == GPIOE){
			GPIOE_CLKEN();
		}else if (pGPIOx == GPIOH){
			GPIOH_CLKEN();
		}
	} else {
		// GPIO Clock Disable macros
	}
}

/*< Function Documentation >
 * @fn			->	GPIO_Init
 *
 * @brief		->	This function configures and initialize given GPIO port and Interrupt
 *
 * @param[in]	->  GPIO port as pointer to the GPIO handle
 *
 * @return		-> 	none
 *
 * @note		-> 	none
 *
 */
void GPIO_Init(GPIO_handle_t *pGPIOHandle){
	uint32_t BitMask = 0;

	//Enabling peripheral clock
	GPIO_ClkControl(pGPIOHandle->pGPIOx, ENABLE);

	/* configure peripheral mode */
	if(pGPIOHandle->GPIO_pinConfig.pinMode <= Analog_mode){
		BitMask = (pGPIOHandle->GPIO_pinConfig.pinMode << (2*pGPIOHandle->GPIO_pinConfig.pinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_pinConfig.pinNumber); // Clearing
		pGPIOHandle->pGPIOx->MODER |= BitMask; // setting
	} else {
		/* Configuring [Interrupt] */
		if(pGPIOHandle->GPIO_pinConfig.pinMode == ext_Int_falling_edge){
			//1. Configure FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_pinConfig.pinNumber);
			// Clear the corresponding RTSR bits
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_pinConfig.pinNumber);
		}else if (pGPIOHandle->GPIO_pinConfig.pinMode == ext_Int_rising_edge){
			//1. Configure RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_pinConfig.pinNumber);
			// Clear the corresponding FTSR bits
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_pinConfig.pinNumber);
		}else if (pGPIOHandle->GPIO_pinConfig.pinMode == ext_Int_rising_falling_edge){
			//1. Configure both FTSR & RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_pinConfig.pinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_pinConfig.pinNumber);
		}
			// 2. Configure the GPIO port selection in SYSCFG_EXTICR->External interrupt control register
			uint8_t reg_pos = pGPIOHandle->GPIO_pinConfig.pinNumber / 4;
			uint8_t bit_pos = pGPIOHandle->GPIO_pinConfig.pinNumber % 4;
			uint8_t portcode = GPIO_BASEADDR_CODE(pGPIOHandle->pGPIOx);
			SYSCFG_CLKEN();
			SYSCFG->EXTICR[reg_pos] = (portcode << (bit_pos * 4));
			// 3. Enable the EXTI interrupt delivery using IMR->Interrupt mask register
			EXTI->IMR |= (1 << pGPIOHandle->GPIO_pinConfig.pinNumber);
	}
	BitMask = 0;

	/* Speed */
	BitMask = (pGPIOHandle->GPIO_pinConfig.pinSpeed << (2*pGPIOHandle->GPIO_pinConfig.pinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_pinConfig.pinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= BitMask;
	BitMask = 0;

	/* PUPD configuration */
	BitMask = (pGPIOHandle->GPIO_pinConfig.pinPUPD << (2*pGPIOHandle->GPIO_pinConfig.pinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_pinConfig.pinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= BitMask;
	BitMask = 0;

	/* Output type */
	BitMask = (pGPIOHandle->GPIO_pinConfig.pinOutputType << pGPIOHandle->GPIO_pinConfig.pinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_pinConfig.pinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= BitMask;
	BitMask = 0;

	/* Alternate Functionality */
	if(pGPIOHandle->GPIO_pinConfig.pinMode == Alternate_function_mode){
		uint8_t afr_s, bit_s; /* to distinguish between alternate function registers and its bits */
		afr_s = pGPIOHandle->GPIO_pinConfig.pinNumber / 8; // This will decide which register we need to use
		bit_s = pGPIOHandle->GPIO_pinConfig.pinNumber % 8; // This will decide which bits we neeed to set
		pGPIOHandle->pGPIOx->AFR[afr_s] &= ~(0xF << (4 * bit_s));
		pGPIOHandle->pGPIOx->AFR[afr_s] |= (pGPIOHandle->GPIO_pinConfig.pinAltFunction << (4 * bit_s));
	}
}

/*< Function Documentation >
 * @fn			->	GPIO_DeInit
 *
 * @brief		->	This function disables peripheral clock for the given GPIO port
 *
 * @param[in]	->  Base addresses of the GPIO peripheral
 *
 * @return		-> 	none
 *
 * @note		-> 	none
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if (pGPIOx == GPIOA){
		GPIOA_REG_RST();
	}
	else if (pGPIOx == GPIOB){
		GPIOB_REG_RST();
	}
	else if (pGPIOx == GPIOC){
		GPIOC_REG_RST();
	}
	else if (pGPIOx == GPIOD){
		GPIOD_REG_RST();
	}
	else if (pGPIOx == GPIOE){
		GPIOE_REG_RST();
	}
	else if (pGPIOx == GPIOH){
		GPIOH_REG_RST();
	}
}

/*< Function Documentation >
 * @fn			->	GPIO_ReadPin
 *
 * @brief		->	Reads the current status for given GPIO pin
 *
 * @param[in]	->  Base addresses of the GPIO peripheral
 * @param[in]	->	Pin number of GPIO
 *
 * @return		-> 	Given GPIO status in uint8_t format
 *
 * @note		-> 	none
 *
 */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber){
	uint8_t value;
	value = ((pGPIOx->IDR >> pinNumber) & 0x00000001);
	return value;
}

/*< Function Documentation >
 * @fn			->	GPIO_ReadPort
 *
 * @brief		->	Reads the current status for given GPIO port
 *
 * @param[in]	->  Base addresses of the GPIO peripheral
 * @param[in]	->	Pin number of GPIO
 *
 * @return		-> 	Given GPIO status in uint16_t format
 *
 * @note		-> 	none
 *
 */
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/*< Function Documentation >
 * @fn			->	GPIO_WritePin
 *
 * @brief		->	Writes user defined values to certain pin of certain port
 *
 * @param[in]	->  Base addresses of the GPIO peripheral
 * @param[in]	->	Pin number of GPIO
 *
 * @return		-> 	none
 *
 * @note		-> 	none
 *
 */
void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value){
	if (value == SET){
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= (1<<pinNumber);
	}else{
		//0
		pGPIOx->ODR &= ~(1<<pinNumber);
	}
}

/*< Function Documentation >
 * @fn			->	GPIO_WritePort
 *
 * @brief		->	Writes user defined values to certain port
 *
 * @param[in]	->  Base addresses of the GPIO peripheral
 * @param[in]	->	Pin number of GPIO
 *
 * @return		-> 	none
 *
 * @note		-> 	none
 *
 */
void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint16_t value){
	pGPIOx->ODR |= value;
}

/*< Function Documentation >
 * @fn			->	GPIO_TogglePin
 *
 * @brief		->	Toggles a user defined pin of certain port
 *
 * @param[in]	->  Base addresses of the GPIO peripheral
 * @param[in]	->	Pin number of GPIO
 *
 * @return		-> 	none
 *
 * @note		-> 	none
 *
 */
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber){
	pGPIOx->ODR ^= (1<<pinNumber);
}

/*< Function Documentation >
 * @fn			->	GPIO_IRQConfig
 *
 * @brief		->
 *
 * @param[in]	->
 * @param[in]	->
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnD){
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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	//1. Find out the IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLIMENTED);
	*( NVIC_PR_BASEADDR + iprx ) |= ( IRQPriority << shift_amount );
}

void GPIO_IRQHandling(uint8_t pinNumber){
	// Configure the EXTI PR register corresponding to the pin number
	if(EXTI->PR & (1<<pinNumber)){
		//Clear pending register bit
		EXTI->PR |= (1<<pinNumber);
	}
}




