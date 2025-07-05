/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: Jun 29, 2025
 *      Author: uma_r
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stm32f446xx.h"

typedef struct {

	uint8_t SPI_Mode; //MASTER /SLAVE    //@SPI_Mode
	uint8_t SPI_BusConfig;
	uint8_t SPI_SCLKSpeed ;     //Prescalar   default is 2
	uint8_t SPI_DFF;  //8 BIT /16 BIT
	uint8_t SPI_CPOL;  //IDLE STATE 1 OR 0
	uint8_t SPI_CPHA;  // 1 OR 0
	uint8_t SPI_SSM;    // Slave select management 1 software slave select 0 hardware


}SPI_Config_struct;



typedef struct{

	SPI_struct *pSPIx; //////Gives peripheral base address of SPI MODE/ PERIPHERAL - SPI1,2,3,4
	SPI_Config_struct SPI_Config; ///SPI user configurable data items

	uint8_t * SPI_TXBuffer;
	uint8_t * SPI_RXBuffer;
	uint32_t  SPI_TXLen;
	uint32_t  SPI_RXLen;
	uint8_t  SPI_TXState;
	uint8_t  SPI_RXState;


}SPI_handle_t;

//MACROS
//@SPI_Mode

#define SPI_MODE_MASTER 1
#define SPI_MODE_SLAVE  0

//@BUS CONFIG

#define SPI_BUS_CONFIG_FD  0
#define SPI_BUS_CONFIG_HD  1
#define SPI_BUS_CONFIG_SIM_TXONLY  2
#define SPI_BUS_CONFIG_SIM_RXONLY  3

//@SCLK SPEED
#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7

//@DFF
#define SPI_DFF_8BITS 0
#define SPI_DFF_16BITS 1

//@CPOL

#define SPI_CPOL_HIGH  	1
#define SPI_CPOL_LOW	0


//@CPHA

#define SPI_CPHA_HIGH  	1
#define SPI_CPHA_LOW	0

//SSM

#define SPI_SSM_DI	0
#define SPI_SSM_EN 	1


#define SPI_STATE_BUSY_IN_TX	1
#define SPI_STATE_BUSY_IN_RX	2
#define SPI_STATE_READY			0

/****************************************************************************************
 ********************* SPI PERIPHERAL REGISTERS BIT DEFINITION MACROS *******************
 ****************************************************************************************/


#define SPI_CR1_CPHA 0
#define SPI_CR1_CPOL 1
#define SPI_CR1_MSTR 2
#define SPI_CR1_BR 3
#define SPI_CR1_SPE 6
#define SPI_CR1_LSB_FIRST 7
#define SPI_CR1_SSI 8
#define SPI_CR1_SSM 9
#define SPI_CR1_RX_ONLY 10
#define SPI_CR1_DFF 11
#define SPI_CR1_CRC_NEXT 12
#define SPI_CR1_CRC_EN 13
#define SPI_CR1_BIDIOE 14
#define SPI_CR1_BIDIMODE 15


#define SPI_CR2_RXDMAEN 0
#define SPI_CR2_TXDMAEN 1
#define SPI_CR2_SSOE 2
#define SPI_CR2_FRF 4
#define SPI_CR2_ERRIR 5
#define SPI_CR2_RXNEIE 6
#define SPI_CR2_TXEIE 7

#define SPI_SR_RXNE 0
#define SPI_SR_TXE 1
#define SPI_SR_CHSIDE 2
#define SPI_SR_UDR 3
#define SPI_SR_CRC_ERR 4
#define SPI_SR_MODF 5
#define SPI_SR_OVR 6
#define SPI_SR_BSY 7
#define SPI_SR_FRE 8



#define SPI_TXE_FLAG    (1<<SPI_SR_TXE)
#define SPI_RXNE_FLAG    (1<<SPI_SR_RXNE)
#define SPI_CHSIDE_FLAG    (1<<SPI_SR_CHSIDE)
#define SPI_UDR_FLAG    (1<<SPI_SR_UDR)
#define SPI_CRC_ERR_FLAG    (1<<SPI_SR_CRC_ERR)
#define SPI_MODF_FLAG    (1<<SPI_SR_MODF)
#define SPI_OVR_FLAG    (1<<SPI_SR_OVR)
#define SPI_BSY_FLAG    (1<<SPI_SR_BSY)
#define SPI_FRE_FLAG    (1<<SPI_SR_FRE)

//APPLICATION VENT MACROS

#define SPI_EVNT_TX_CMPLT 1
#define SPI_EVNT_RX_CMPLT 2
#define SPI_EVNT_OVR_ERROR 3




uint8_t Get_Flag_Status(SPI_struct *pSPIx, uint32_t FlagName);

/*
 * API PROTOTYPES FOR SPI PERIPHERAL
 */


// SPI PERIPHERAL CLOCK ENABLE
void SPI_PeriClockControl(SPI_struct *pSPIx, uint8_t EnOrDis );

// SPI Init and Deinit

void SPI_Init(SPI_handle_t *pSPI_Handle);

void SPI_Deinit(SPI_struct *pSPIx);

//SPI PERIPHERAL ENABLE DISABLE
void SPI_PeriEnable(SPI_struct * pSPIx,uint8_t EnOrDis);

//SSI ENABLE
void SPI_SSI_Conifg(SPI_struct *pSPIx,uint8_t EnOrDis);


//SPI TRANSMIT AND RECEIVE BY POLLING MODE / BLOCKING MODE

void SPI_Transmitdata(SPI_struct *pSPIx, uint8_t *pTXBuffer, uint32_t len);

void SPI_Receivedata(SPI_struct *pSPIx, uint8_t *pRXBuffer, uint32_t len);

//SPI TRANSMIT AND RECEIVE BY INTERRUPT MODE
uint8_t SPI_TransmitdataIT(SPI_handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t len);

uint8_t SPI_ReceivedataIT(SPI_handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t len);

// SPI TRANSMIT AND RECEIVE BY INTERRUPT /NON BLOCKING

void SPI_IRQInterruptConfig(uint8_t IRQNum, uint8_t EnOrDis) ;
void SPI_IRQPriorityConfig(uint8_t IRQNum, uint8_t IRQPriority) ;
void SPI_IRQHandler(SPI_handle_t* pSPIhandle);

void SPI_ClearOVRFlag(SPI_struct *pSPIx);
void SPI_CloseTransmission(SPI_handle_t *pSPIHandle);
void SPI_CloseReception(SPI_handle_t *pSPIHandle);
void SPI_ApplicationCallBack(SPI_handle_t* pSPIHandle,uint8_t APPEV);





#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
