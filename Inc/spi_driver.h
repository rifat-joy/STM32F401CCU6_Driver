/*
 * spi_driver.h
 *
 *  Created on: Jan 24, 2023
 *      Author: rifat
 */

#ifndef INC_SPI_DRIVER_H_
#define INC_SPI_DRIVER_H_

#include "_hal_f401xx.h"

/*
 * Configuration structure for SPIx peripheral
 */
typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_ClockSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */
typedef struct{
	SPI_RegDef_t 	*pSPIx;			/*<This holds the base address of SPI1,2,3,4 base addresses>*/
	SPI_Config_t 	SPIConfig;		/*<This holds user configuration>*/
	uint8_t			*pTxBuffer;		/*<Stores Tx buffer address>*/
	uint8_t 		*pRxBuffer;		/*<Stores Rx buffer address>*/
	uint32_t		Txlen;			/*<Tx data length>*/
	uint32_t		Rxlen;			/*<Rx data length>*/
	uint8_t 		TxState;		/*<Transmission state>*/
	uint8_t 		RxState;		/*<Received state>*/
}SPI_Handle_t;

/*
 * @SPI_DeviceMode
 */
#define SPI_MASTER	1
#define SPI_SLAVE	0

/*
 * @SPI_BusConfig
 */
#define SPI_FullDuplex	1
#define SPI_HalfDuplex	2
#define SPI_Simplex_Rx	3

/*
 * @SPI_ClockSpeed
 */
#define SPI_Speed_Div2		0
#define SPI_Speed_Div4		1
#define SPI_Speed_Div8		2
#define SPI_Speed_Div16		3
#define SPI_Speed_Div32		4
#define SPI_Speed_Div64		5
#define SPI_Speed_Div128	6
#define SPI_Speed_Div256	7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8B		0
#define SPI_DFF_16B		1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW	0
#define SPI_CPOL_HIGH	1

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_LOW	0
#define SPI_CPHA_HIGH	1

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN		1
#define SPI_SSM_DI		0

/*
 * @SPI_FLAG_BITS
 */
#define SPI_FLAG_RXNE		(1<<SPI_SR_RXNE)
#define SPI_FLAG_TXE		(1<<SPI_SR_TXE)
#define SPI_FLAG_CHSIDE		(1<<SPI_SR_CHSIDE)
#define SPI_FLAG_UDR		(1<<SPI_SR_UDR)
#define SPI_FLAG_CRCERR		(1<<SPI_SR_CRCERR)
#define SPI_FLAG_MODF		(1<<SPI_SR_MODF)
#define SPI_FLAG_OVR		(1<<SPI_SR_OVR)
#define SPI_FLAG_BSY		(1<<SPI_SR_BSY)
#define SPI_FLAG_FRE		(1<<SPI_SR_FRE)

/*
 * @SPI_APPLICATION_STATE
 */
#define SPI_READY			0
#define SPI_BUSY_IN_Rx		1
#define SPI_BUSY_IN_Tx		2

/*
 * @SPI_APPLICATION_EVENTS
 */
#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_ERROR	3


/* SPIx Peripheral clock setup */
void SPI_ClkControl(SPI_RegDef_t *pSPIx, uint8_t EnD); 			/* < pointer to the base address, Enable or disable> */

/* Initialization and Deinitialization */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/* To check the flag status set by the hardware */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

/* Data send and Receive non interrupt mode*/
void SPI_Transmit(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t TxLen);
void SPI_Receive(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t RxLen);

/* Data send and Receive in interrupt mode*/
uint8_t SPI_IntTransmit(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t TxLen);
uint8_t SPI_IntReceive(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t RxLen);


/* SPIx IRQ configuration and Handling */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnD);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/* Other peripheral controll API's */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnD);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnD);
void SPI_SSOEonfig(SPI_RegDef_t *pSPIx, uint8_t EnD);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/* Application callback functios */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t Event);

#endif /* INC_SPI_DRIVER_H_ */
