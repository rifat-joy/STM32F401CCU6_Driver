/*
 * gpio_driver.h
 *
 *  Created on: Jan 2, 2023
 *      Author: Rifat
 */

#ifndef INC_GPIO_DRIVER_H_
#define INC_GPIO_DRIVER_H_

#include "_hal_f401xx.h"

typedef struct {
	uint8_t pinNumber;						/* Takes values from @GPIO_pins 		*/
	uint8_t pinMode;						/* Takes values from @GPIO_modes 		*/
	uint8_t pinSpeed;						/* Takes values from @GPIO_speed 		*/
	uint8_t pinPUPD;						/* Takes values from @PUPD_type 		*/
	uint8_t pinOutputType;					/* Takes values from @GPIO_Output_type 	*/
	uint8_t pinAltFunction;					/* Takes values from  		*/
}GPIO_pinConfig_t;

typedef struct {
	GPIO_RegDef_t *pGPIOx;
	GPIO_pinConfig_t GPIO_pinConfig;
}GPIO_handle_t;


/*
 * @GPIO_pins:
 */
#define PIN_0						0
#define PIN_1						1
#define PIN_2						2
#define PIN_3						3
#define PIN_4						4
#define PIN_5						5
#define PIN_6						6
#define PIN_7						7
#define PIN_8						8
#define PIN_9						9
#define PIN_10						10
#define PIN_11						11
#define PIN_12						12
#define PIN_13						13
#define PIN_14						14
#define PIN_15						15

/*
 * @GPIO_modes:
 */
#define Input_mode					0
#define Output_mode					1
#define Alternate_function_mode		2
#define Analog_mode					3
#define ext_Int_falling_edge			4
#define ext_Int_rising_edge				5
#define ext_Int_rising_falling_edge		6

/*
 * @GPIO_Output_type:
 */
#define Output_push_pull			0
#define Output_open_drain			1

/*
 * @PUPD_type:
 */
#define No_pull_up_pull_down		0
#define	Pull_up						1
#define Pull_down					2

/*
 * @GPIO_speed:
 */
#define Low_speed					0
#define Medium_speed				1
#define High_speed					2
#define Very_high_speed				3

/* GPIOx Peripheral clock setup */
void GPIO_ClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnD); 			/* < pointer to the base address, Enable or disable> */

/* Initialization and Deinitialization */
void GPIO_Init(GPIO_handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/* Data read and write */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

/* Interrupt configuration and Handling */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnD);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t pinNumber);



#endif /* INC_GPIO_DRIVER_H_ */
