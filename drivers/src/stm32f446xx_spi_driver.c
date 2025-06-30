


//HEADERS

#include "stm32f446xx.h"
#include "stm32f446xx_spi_driver.h"


//STRUCTS DEFINED IN .h
/*typedef struct {

	uint8_t SPI_Mode;   //MASTER /SLAVE
	uint8_t SPI_SCLKSpeed ;     //Prescalar   default is 2
	uint8_t SPI_DFF;  //8 BIT /16 BIT
	uint8_t SPI_CPOL;  //IDLE STATE 1 OR 0
	uint8_t SPI_CPHA;  // 1 OR 0
	uint8_t SPI_SSM;    // Slave select management 1 software slave select 0 hardware
	uint8_t SPI_BusConfig;

}SPI_Config_struct;



typedef struct{

	SPI_struct *pSPIx; //////Gives peripheral base address of SPI MODE/ PERIPHERAL - SPI1,2,3,4
	SPI_Config_struct SPI_Config; ///SPI user configurable data items


}SPI_handle_t;*/

uint8_t Get_Flag_Status(SPI_struct *pSPIx, uint32_t FlagName)
{
	if (pSPIx->SR & FlagName==0)
		return FLAG_RESET;
	else
		return FLAG_SET;
}




/*
 * API PROTOTYPES FOR SPI PERIPHERAL
 */

// SPI PERIPHERAL CLOCK ENABLE
void SPI_PeriClockControl(SPI_struct *pSPIx, uint8_t EnOrDis )
{
	if(EnOrDis==ENABLE)
	{
		if (pSPIx==SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx==SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx==SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if(pSPIx==SPI4)
		{
			SPI4_PCLK_EN();
		}
	}

	else
	{
		if (pSPIx==SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx==SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx==SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if(pSPIx==SPI4)
		{
			SPI4_PCLK_DI();
		}

	}
}

// SPI Init and Deinit

void SPI_Init(SPI_handle_t *pSPI_Handle)
{

	SPI_PeriClockControl(pSPI_Handle->pSPIx, ENABLE);

	//1. SPI_Mode
	pSPI_Handle->pSPIx->CR1 &=~(1<<SPI_CR1_MSTR);
	pSPI_Handle->pSPIx->CR1 |=(pSPI_Handle->SPI_Config.SPI_Mode<<2);

	//BUS CONFIG
	if(pSPI_Handle->SPI_Config.SPI_BusConfig==SPI_BUS_CONFIG_FD)
	{
		pSPI_Handle->pSPIx->CR1 &=~(1<<SPI_CR1_BIDIMODE);
	}
	else if(pSPI_Handle->SPI_Config.SPI_BusConfig==SPI_BUS_CONFIG_HD)
	{
		pSPI_Handle->pSPIx->CR1 |=(1<<SPI_CR1_BIDIMODE);
	}
	else if(pSPI_Handle->SPI_Config.SPI_BusConfig==SPI_BUS_CONFIG_SIM_RXONLY)
	{
		pSPI_Handle->pSPIx->CR1 &=~(1<<SPI_CR1_BIDIMODE);
		pSPI_Handle->pSPIx->CR1 |=(1<<SPI_CR1_RX_ONLY);
	}
	//3. SPI_SCLK_SPEED
	pSPI_Handle->pSPIx->CR1 &=~(7<<SPI_CR1_BR);
	pSPI_Handle->pSPIx->CR1 |=(pSPI_Handle->SPI_Config.SPI_SCLKSpeed<<3);

	//4. DFF CONFIG

	pSPI_Handle->pSPIx->CR1 &= ~(1<<SPI_CR1_DFF);
	pSPI_Handle->pSPIx->CR1 |= (pSPI_Handle->SPI_Config.SPI_DFF<<11);

	//5. CPOL CONFIG

		pSPI_Handle->pSPIx->CR1 &= ~(1<<SPI_CR1_CPOL);
		pSPI_Handle->pSPIx->CR1 |= (pSPI_Handle->SPI_Config.SPI_CPOL<<1);

	//6. CPHA CONFIG

		pSPI_Handle->pSPIx->CR1 &= ~(1<<SPI_CR1_CPHA);
		pSPI_Handle->pSPIx->CR1 |= (pSPI_Handle->SPI_Config.SPI_CPHA<<0);


	//7. SSM CONFIG

		pSPI_Handle->pSPIx->CR1 &= ~(1<<SPI_CR1_SSM);
		pSPI_Handle->pSPIx->CR1 |= (pSPI_Handle->SPI_Config.SPI_SSM<<9);


}

void SPI_Deinit(SPI_struct *pSPIx)
{
	if (pSPIx==SPI1)
	{
		RCC->RCC_APB2_RSTR |= 1<<12;
		RCC->RCC_APB2_RSTR &= ~(1<<12);

	}
	else if (pSPIx==SPI4)
	{
		RCC->RCC_APB2_RSTR |= 1<<13;
		RCC->RCC_APB2_RSTR &= ~(1<<13);

	}
	else if (pSPIx==SPI2)
	{
		RCC->RCC_APB1_RSTR |= 1<<14;
		RCC->RCC_APB1_RSTR &= ~(1<<14);

	}
	else if (pSPIx==SPI3)
	{
		RCC->RCC_APB1_RSTR |= 1<<15;
		RCC->RCC_APB1_RSTR &= ~(1<<15);

	}


}

//SPI TRANSMIT AND RECEIVE BY POLLING MODE / BLOCKING MODE

void SPI_Transmitdata(SPI_struct *pSPIx, uint8_t *pTXBuffer, uint32_t len)
{
	while (len>0)
	{
		while (Get_Flag_Status(pSPIx, SPI_TXE_FLAG)==FLAG_RESET);/// POLLING (CPU WAITS CONTINUOUSLY CHECKING FOR THE FLAG TO CHANGE STATE)


	// if get_flag_status() returns FLAG SET TX BUFFER EMPTY
		// Check DF BIT IN CR1 REGISTER

		if ((pSPIx->CR1 & (1<<SPI_CR1_DFF))==0)    /// 8 BIT FRAME FORMAT
		{
			pSPIx->DR= *(pTXBuffer);
			len--;
			pTXBuffer+=1;
		}
		else
		{
			pSPIx->DR= *((uint16_t *)pTXBuffer);
			len--;
			len--;
			pTXBuffer+=2;
		}

	}



}
//SPI PERIPHERAL ENABLE DISABLE
void SPI_PeriEnable(SPI_struct * pSPIx,uint8_t EnOrDis){

	if(EnOrDis==ENABLE)
	{
	pSPIx->CR1 |= (1<<SPI_CR1_SPE);
	}
	else
	{
	pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);
	}
}

//ssi
void SPI_SSI_Conifg(SPI_struct *pSPIx,uint8_t EnOrDis){
	if (EnOrDis==ENABLE)
	{
		pSPIx->CR1|=(1<<SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1&=~(1<<SPI_CR1_SSI);
	}
}



void SPI_Receivedata(SPI_struct *pSPIx, uint8_t *pRXBuffer, uint32_t len)
{
	while (len>0)
	{
		while (Get_Flag_Status(pSPIx, SPI_RXNE_FLAG)==FLAG_RESET);/// POLLING (CPU WAITS CONTINUOUSLY CHECKING FOR THE FLAG TO CHANGE STATE)


	// if get_flag_status() returns FLAG SET TX BUFFER EMPTY
		// Check DF BIT IN CR1 REGISTER

		if ((pSPIx->CR1 & (1<<SPI_CR1_DFF))==0)    /// 8 BIT FRAME FORMAT
		{
			*(pRXBuffer)=pSPIx->DR ;
			len--;
			pRXBuffer+=1;
		}
		else
		{
			*((uint16_t*)(pRXBuffer))=pSPIx->DR ;
			len--;
			len--;
			pRXBuffer+=2;
		}

	}

}

// SPI TRANSMIT AND RECEIVE BY INTERRUPT /NON BLOCKING

void SPI_IRQInterruptConfig()
{

}
void SPI_IRQPriorityConfig()
{

}
void SPI_IRQHandler()
{

}

