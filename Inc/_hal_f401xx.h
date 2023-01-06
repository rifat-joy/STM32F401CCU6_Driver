/*
 * _hal_f401xx.h
 *
 *  Created on: Jan 1, 2023
 *      Author: rifat
 */

#ifndef INC__HAL_F401XX_H_
#define INC__HAL_F401XX_H_

#include <stdint.h>
//#include <stm32f4xx.h>

#define SET 	1
#define RESET 	0
#define ENABLE  SET
#define DISABLE RESET

/*
 * Base addresses of flash and SRAM memories (Manual Table 5)
 */

#define FLASH_BASEADDR		0x08000000U			//(uint32_t)0x08000000 -> same thing
#define SRAM1_BASEADDR		0x20000000U
#define ROM__BASEADDR		0x1FFF0000U
#define SRAM 				SRAM1_BASEADDR

/*
 * AHBx & APBx bus peripheral base addresses (Manual Table 1)
 */

#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U

/*
 * Base addresses of peripherals hanging on AHB1 bus
 */
#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800)
#define CRC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3000)
#define DMA1_BASEADDR			(AHB1PERIPH_BASEADDR + 0x6000)
#define DMA2_BASEADDR			(AHB1PERIPH_BASEADDR + 0x6400)

/*
 * Base addresses of peripherals hanging on AHB2 bus
 */
#define USB_OTG_FS_BASEADDR		(AHB2PERIPH_BASEADDR + 0x0000)

/*
 * Base addresses of peripherals hanging on APB1 bus
 */
#define PWR_BASEADDR             (APB1PERIPH_BASEADDR + 0x7000)
#define I2C3_BASEADDR            (APB1PERIPH_BASEADDR + 0x5C00)
#define I2C2_BASEADDR            (APB1PERIPH_BASEADDR + 0x5800)
#define I2C1_BASEADDR            (APB1PERIPH_BASEADDR + 0x5400)
#define USART2_BASEADDR          (APB1PERIPH_BASEADDR + 0x4400)
#define I2S3ext_BASEADDR         (APB1PERIPH_BASEADDR + 0x4000)
#define SPI3_I2S3_BASEADDR       (APB1PERIPH_BASEADDR + 0x3C00)
#define SPI2_I2S2_BASEADDR       (APB1PERIPH_BASEADDR + 0x3800)
#define I2S2ext_BASEADDR         (APB1PERIPH_BASEADDR + 0x3400)
#define IWDG_BASEADDR            (APB1PERIPH_BASEADDR + 0x3000)
#define WWDG_BASEADDR            (APB1PERIPH_BASEADDR + 0x2C00)
#define RTC_BKP_BASEADDR         (APB1PERIPH_BASEADDR + 0x2800)
#define TIM5_BASEADDR            (APB1PERIPH_BASEADDR + 0x0C00)
#define TIM4_BASEADDR            (APB1PERIPH_BASEADDR + 0x8000)
#define TIM3_BASEADDR            (APB1PERIPH_BASEADDR + 0x4000)
#define TIM2_BASEADDR            (APB1PERIPH_BASEADDR + 0x0000)


/*
 * Base addresses of peripherals hanging on APB2 bus
 */
#define TIM11_BASEADDR           (APB2PERIPH_BASEADDR + 0x4800)
#define TIM10_BASEADDR           (APB2PERIPH_BASEADDR + 0x4400)
#define TIM9_BASEADDR            (APB2PERIPH_BASEADDR + 0x4000)
#define EXTI_BASEADDR            (APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR          (APB2PERIPH_BASEADDR + 0x3800)
#define SPI4_BASEADDR            (APB2PERIPH_BASEADDR + 0x3400)
#define SPI1_BASEADDR            (APB2PERIPH_BASEADDR + 0x3000)
#define SDIO_BASEADDR            (APB2PERIPH_BASEADDR + 0x2C00)
#define ADC1_BASEADDR            (APB2PERIPH_BASEADDR + 0x2000)
#define USART6_BASEADDR          (APB2PERIPH_BASEADDR + 0x1400)
#define USART1_BASEADDR          (APB2PERIPH_BASEADDR + 0x1000)
#define TIM1_BASEADDR            (APB2PERIPH_BASEADDR + 0x0000)


/******************peripherals register definition structures***************/
typedef struct{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
}GPIO_RegDef_t;

typedef struct{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	uint32_t 		  RESERVED0[2];
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t 		  RESERVED1[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	uint32_t 		  RESERVED2[2];
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t 		  RESERVED3[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	uint32_t 		  RESERVED4[2];
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t 		  RESERVED5[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t 		  RESERVED6[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	uint32_t 		  RESERVED7;
	volatile uint32_t DCKCFGR;
}RCC_RegDef_t;

/******Peripheral base address is typecasted to GPIO_RegDef_t*******/

#define GPIOA	((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB	((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC	((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD	((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE	((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH	((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC		((RCC_RegDef_t*)RCC_BASEADDR)

/******Clock enable macros for GPIOx*******/
#define GPIOA_CLKEN()	(RCC->AHB1ENR |= (1<<0))
#define GPIOB_CLKEN()	(RCC->AHB1ENR |= (1<<1))
#define GPIOC_CLKEN()	(RCC->AHB1ENR |= (1<<2))
#define GPIOD_CLKEN()	(RCC->AHB1ENR |= (1<<3))
#define GPIOE_CLKEN()	(RCC->AHB1ENR |= (1<<4))
#define GPIOH_CLKEN()	(RCC->AHB1ENR |= (1<<7))

/****** GPIO register reset *******/
#define GPIOA_REG_RST()		do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RST()		do{(RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RST()		do{(RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RST()		do{(RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RST()		do{(RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOH_REG_RST()		do{(RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7));}while(0)

/******Clock enable macros for I2Cx*******/
#define I2C1_CLKEN()	(RCC->APB1ENR |= (1<<21))
#define I2C2_CLKEN()	(RCC->APB1ENR |= (1<<22))
#define I2C3_CLKEN()	(RCC->APB1ENR |= (1<<23))

/******Clock enable macros for SPIx*******/
#define SPI1_CLKEN()		(RCC->APB2ENR |= (1<<12))
#define SPI2I_I2S2_CLKEN()	(RCC->APB1ENR |= (1<<14))
#define SPI3I_I2S3_CLKEN()	(RCC->APB1ENR |= (1<<15))
#define SPI4_CLKEN()		(RCC->APB2ENR |= (1<<13))

/******Clock enable macros for USARTx*******/
#define USART1_CLKEN()	(RCC->APB2ENR |= (1<<4))
#define USART2_CLKEN()	(RCC->APB1ENR |= (1<<17))
#define USART6_CLKEN()	(RCC->APB2ENR |= (1<<5))


#include "gpio_driver.h"

#endif /* INC__HAL_F401XX_H_ */
