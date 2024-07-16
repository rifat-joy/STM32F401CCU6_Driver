/*
 * _hal_f401xx.h
 *
 *  Created on: Jan 1, 2023
 *      Author: Rifat
 */

#ifndef INC__HAL_F401XX_H_
#define INC__HAL_F401XX_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
//#include <stm32f4xx.h>

#define __weak __attribute__ ((weak))

#define SET 		1
#define RESET 		0
#define ENABLE  	SET
#define DISABLE 	RESET
#define FLAG_SET	SET
#define FLAG_RESET	RESET

/*>>>>>>>>>>>>>>>> START: Processor specific base addresses <<<<<<<<<<<<<<<<<<<*/

/*
  * ARM Cortex M4 processor NVIC ISERx register base addresses
  */
#define NVIC_ISER0				((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1				((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2				((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3				((volatile uint32_t*)0xE000E10C)

/*
  * ARM Cortex M4 processor NVIC ICERx register base addresses
  */
#define NVIC_ICER0				((volatile uint32_t*)0XE000E180)
#define NVIC_ICER1				((volatile uint32_t*)0XE000E184)
#define NVIC_ICER2				((volatile uint32_t*)0XE000E188)
#define NVIC_ICER3				((volatile uint32_t*)0XE000E18C)

/*
  * ARM Cortex M4 processor NVIC priority register base addresses
  */
#define NVIC_PR_BASEADDR		((volatile uint32_t*)0xE000E400)
#define NO_PR_BITS_IMPLIMENTED	4

/*>>>>>>>>>>>>>>>> END: Processor specific base addresses <<<<<<<<<<<<<<<<<<<*/

/*
 * Base addresses of flash and SRAM memories (Manual Table 5)
 */

#define FLASH_BASEADDR			0x08000000U			//(uint32_t)0x08000000 -> same thing
#define SRAM1_BASEADDR			0x20000000U
#define ROM__BASEADDR			0x1FFF0000U
#define SRAM 					SRAM1_BASEADDR

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

/*
 * IRQ (Interrupt Request) number of EXTIx STM32F401CCUx MCU
 */
#define IRQ_NO_EXTI0 					6
#define IRQ_NO_EXTI1 					7
#define IRQ_NO_EXTI2					8
#define IRQ_NO_EXTI3 					9
#define IRQ_NO_EXTI4					10
#define IRQ_NO_EXTI9_5					23
//#define IRQ_NO_EXTI16_PVD				1
#define IRQ_NO_EXTI15_10				40
//#define IRQ_NO_EXTI17_RTC_Alarm			41
//#define IRQ_NO_EXTI18_OTG_FS_WKUP		42

/*
 * IRQ (Interrupt Request) number of SPIx STM32F401CCUx MCU
 */
#define IRQ_NO_SPI1 					35
#define IRQ_NO_SPI2 					36
#define IRQ_NO_SPI3 					51
#define IRQ_NO_SPI4 					84


/*
 * IRQ (Interrupt Request) priority of STM32F401CCUx MCU
 */
#define IRQ_PRI0						0
#define IRQ_PRI1						1
#define IRQ_PRI2						2
#define IRQ_PRI3						3
#define IRQ_PRI4						4
#define IRQ_PRI5						5
#define IRQ_PRI6						6
#define IRQ_PRI7						7
#define IRQ_PRI8						8
#define IRQ_PRI9						9
#define IRQ_PRI10						10
#define IRQ_PRI11						11
#define IRQ_PRI12						12
#define IRQ_PRI13						13
#define IRQ_PRI14						14
#define IRQ_PRI15						15


/******************peripherals register definition structures***************/
typedef struct{				/* GPIO Register definition*/
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

typedef struct{				/* RCC-> Register clock control Register definition*/
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

typedef struct{				/* External Interrupt Register definition*/
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTI_RegDef_t;

typedef struct{
	volatile uint32_t MEMRMP;						/* Address offset : 0x00 */
	volatile uint32_t PMC;							/* Address offset : 0x04 */
	volatile uint32_t EXTICR[4];					/* Address offset : 0x08 - 0x14 */
	volatile uint32_t CMPCR;						/* Address offset : 0x20 */
}SYSCFG_RegDef_t;

typedef struct{				/* SPI Register definition*/
	volatile uint32_t CR1;						/* SPI control register 1 offset 			: 0x00 */
	volatile uint32_t CR2;						/* SPI control register 2 offset 			: 0x04 */
	volatile uint32_t SR;						/* SPI status register 						: 0x08 */
	volatile uint32_t DR;						/* SPI data register offset 				: 0x0C */
	volatile uint32_t CRCPR;					/* SPI CRC polynomial register offset 		: 0x10 */
	volatile uint32_t RXCRCR;					/* SPI RX CRC register offset 				: 0x14 */
	volatile uint32_t TXCRCR;					/* SPI TX CRC register offset 				: 0x18 */
	volatile uint32_t I2SCFGR; 					/* SPI_I2S configuration register offset 	: 0x20 */
	volatile uint32_t I2SPR;					/* SPI_I2S prescaler register offset 		: 0x20 */
}SPI_RegDef_t;

/******Peripheral base address is typecasted to GPIO_RegDef_t*******/

#define GPIOA	((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB	((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC	((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD	((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE	((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH	((GPIO_RegDef_t*)GPIOH_BASEADDR)

/******Peripheral base address is typecasted to RCC_RegDef_t*******/
#define RCC		((RCC_RegDef_t*)RCC_BASEADDR)

/******Peripheral base address is typecasted to EXTI_RegDef_t*******/
#define EXTI	((EXTI_RegDef_t*)EXTI_BASEADDR)

/******Peripheral base address is typecasted to SYSCFG_RegDef_t*******/
#define SYSCFG	((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/******Peripheral base address is typecasted to SPI_RegDef_t*******/
#define SPI1	((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2	((SPI_RegDef_t*)SPI2_I2S2_BASEADDR)
#define SPI3	((SPI_RegDef_t*)SPI3_I2S3_BASEADDR)
#define SPI4	((SPI_RegDef_t*)SPI4_BASEADDR)

/******Clock enable macros for GPIOx*******/
#define GPIOA_CLKEN()	(RCC->AHB1ENR |= (1<<0))
#define GPIOB_CLKEN()	(RCC->AHB1ENR |= (1<<1))
#define GPIOC_CLKEN()	(RCC->AHB1ENR |= (1<<2))
#define GPIOD_CLKEN()	(RCC->AHB1ENR |= (1<<3))
#define GPIOE_CLKEN()	(RCC->AHB1ENR |= (1<<4))
#define GPIOH_CLKEN()	(RCC->AHB1ENR |= (1<<7))

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

/******Clock enable macros for SYSCFGx*******/
#define SYSCFG_CLKEN()	(RCC->APB2ENR |= (1<<14))


/****** GPIO register reset *******/
#define GPIOA_REG_RST()		do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RST()		do{(RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RST()		do{(RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RST()		do{(RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RST()		do{(RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOH_REG_RST()		do{(RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7));}while(0)

/****** SPI register reset *******/
#define SPI1_REG_RST()				do{(RCC->APB2RSTR |= (1<<12)); (RCC->APB2RSTR &= ~(1<<12));}while(0)
#define SPI2I_I2S2_REG_RST()		do{(RCC->APB1RSTR |= (1<<14)); (RCC->APB1RSTR &= ~(1<<14));}while(0)
#define SPI3I_I2S3_REG_RST()		do{(RCC->APB1RSTR |= (1<<15)); (RCC->APB1RSTR &= ~(1<<15));}while(0)
#define SPI4_REG_RST()				do{(RCC->APB2RSTR |= (1<<13)); (RCC->APB2RSTR &= ~(1<<13));}while(0)

/****** GPIO register reset [Conditional/ternary operation] *******/
#define GPIO_BASEADDR_CODE(x)	((x == GPIOA)?0:\
								(x == GPIOB)?1:\
								(x == GPIOC)?2:\
								(x == GPIOD)?3:\
								(x == GPIOE)?4:\
								(x == GPIOH)?7:0)

/*******************************************************************************************************************
 * Bit position definitions for SPI peripheral
 ******************************************************************************************************************/
/*
 * < SPI_CR1 BITS >
 */
#define SPI_CR1_CPHA		0				/*< Clock phase bit >*/
#define	SPI_CR1_CPOL		1				/*< Clock polarity bit >*/
#define SPI_CR1_MSTR		2				/*< Master selection bit >*/
#define	SPI_CR1_BR			3				/*< Baud rate control bit 0 >*/
//#define	SPI_CR1_BR_1		4				/*< Baud rate control bit 1 >*/
//#define	SPI_CR1_BR_2		5				/*< Baud rate control bit 2 >*/
#define SPI_CR1_SPE			6				/*< SPI enable bit >*/
#define SPI_CR1_LSB_FIRST	7				/*< Frame format bit >*/
#define SPI_CR1_SSI			8				/*< Internal slave select bit >*/
#define SPI_CR1_SSM			9				/*< Internal slave select bit >*/
#define SPI_CR1_Rx_ONLY		10				/*< Receive only bit >*/
#define SPI_CR1_DFF			11				/*< Data frame format bit >*/
#define SPI_CR1_CRC_NEXT	12				/*< CRC transfer next bit >*/
#define SPI_CR1_CRCEN		13				/*< Hardware CRC calculation enable bit >*/
#define SPI_CR1_BIDIOE		14				/*< Output enable in bidirectional mode bit >*/
#define SPI_CR1_BIDIMODE	15				/*< Bidirectional data mode enable bit >*/

/*
 * < SPI_CR2 BITS >
 */
#define SPI_CR2_RXDMAEN		0				/*< Rx buffer DMA enable bit >*/
#define SPI_CR2_TXDMAEN		1				/*< Tx buffer DMA enable bit >*/
#define SPI_CR2_SSOE		2				/*< SS output enable bit >*/
#define SPI_CR2_FRF			4				/*< Frame format bit >*/
#define SPI_CR2_ERRIE		5				/*< Error interrupt enable bit >*/
#define SPI_CR2_RXNEIE		6				/*< RX buffer not empty interrupt enable bit >*/
#define SPI_CR2_TXEIE		7				/*< Tx buffer empty interrupt enable bit >*/

/*
 * < SPI_SR BITS >
 */
#define SPI_SR_RXNE			0				/*< Receive buffer not empty bit >*/
#define SPI_SR_TXE			1				/*< Transmit buffer empty bit >*/
#define SPI_SR_CHSIDE		2				/*< Channel side bit >*/
#define SPI_SR_UDR			3				/*< Underrun flag bit >*/
#define SPI_SR_CRCERR		4				/*< CRC error flag bit >*/
#define SPI_SR_MODF			5				/*< Mode fault bit >*/
#define SPI_SR_OVR			6				/*< Overrun flag bit >*/
#define SPI_SR_BSY			7				/*< Busy flag bit >*/
#define SPI_SR_FRE			8				/*< Frame format error bit >*/


#include "gpio_driver.h"
#include "spi_driver.h"

#endif /* INC__HAL_F401XX_H_ */
