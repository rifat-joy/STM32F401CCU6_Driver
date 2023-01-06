/*
 * 	GPIO API Implementation
 *
 *  Created on: Jan 2, 2023
 *      Author: rifat
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
 * @brief		->	This function configures and initialize given GPIO port
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

	/* configure mode */
	if(pGPIOHandle->GPIO_pinConfig.pinMode <= Analog_mode){
		BitMask = (pGPIOHandle->GPIO_pinConfig.pinMode << (2*pGPIOHandle->GPIO_pinConfig.pinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_pinConfig.pinNumber);
		pGPIOHandle->pGPIOx->MODER |= BitMask;
	} else {
		// Will do later [Interrupt]
	}
	BitMask = 0;

	/* Speed */
	BitMask = (pGPIOHandle->GPIO_pinConfig.pinSpeed << (2*pGPIOHandle->GPIO_pinConfig.pinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_pinConfig.pinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= BitMask;
	BitMask = 0;

	/* PUPD configuration */
	BitMask = (pGPIOHandle->GPIO_pinConfig.pinPUPD << (2*pGPIOHandle->GPIO_pinConfig.pinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x1 << pGPIOHandle->GPIO_pinConfig.pinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= BitMask;
	BitMask = 0;

	/* Output type */
	BitMask = (pGPIOHandle->GPIO_pinConfig.pinOutputType << pGPIOHandle->GPIO_pinConfig.pinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x3 << pGPIOHandle->GPIO_pinConfig.pinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= BitMask;
	BitMask = 0;

	/* Alternate Functionality */
	if(pGPIOHandle->GPIO_pinConfig.pinMode == Alternate_function_mode){
		uint8_t afr_s, bit_s; /* to distinguish between alternate function registers and its bits */
		afr_s = pGPIOHandle->GPIO_pinConfig.pinNumber / 8;
		bit_s = pGPIOHandle->GPIO_pinConfig.pinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[afr_s] |= (0xF << (4 * bit_s));
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

/* Interrupt configuration and Handling */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnD){

}

void GPIO_IRQHandling(uint8_t pinNumber){

}

