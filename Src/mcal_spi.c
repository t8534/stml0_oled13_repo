

//#if 0


//#include "stm32l0xx_hal_spi.h"
#include "mcal_spi.h"


/* SPI handler declaration */
SPI_HandleTypeDef SpiHandle;

/* Buffer used for transmission */
//#define MCAL_SPI_TX_BUFF_LEN  20
//uint8_t aTxBuffer[MCAL_SPI_TX_BUFF_LEN];

// Arduino default:
// - Mode 0 (the default) - clock is normally low (CPOL = 0) (SPI_POLARITY_HIGH - stm), and the data is sampled
//   on the transition from low to high (leading edge) (CPHA = 0) (SPI_PHASE_1EDGE - stm)
// - Data order: The default is most-significant bit first,
// - Speed: The default setting for SPI is to use the system clock speed divided by four,
//   that is, one SPI clock pulse every 250 ns, assuming a 16 MHz CPU clock. 
//   You can change the clock divider by using setClockDivider like this:
//
//   SPI.setClockDivider (SPI_CLOCK_DIV128);
//   
//   The fastest rate is "divide by 2" or one SPI clock pulse every 125 ns, assuming
//   a 16 MHz CPU clock. This would therefore take 8 * 125 ns or 1 탎 to transmit one byte.
// 
//   However empirical testing shows that it is necessary to have two clock pulses between
//   bytes, so the maximum rate at which bytes can be clocked out is 1.125 탎 each
//   (with a clock divider of 2).
//   To summarize, each byte can be sent at a maximum rate of one per 1.125 탎
//   (with a 16 MHz clock) giving a theoretical maximum transfer rate of 1/1.125 탎,
//   or 888,888 bytes per second (excluding overhead like setting SS low and so on).
void MCAL_SPI_Init()
{
  HAL_StatusTypeDef res = HAL_ERROR;
	
	// Data sheed of display shows it should be mode 3 - Clock IDLE = High and Data 
	// acquired on the rising edge.
  /*##-1- Configure the SPI peripheral #######################################*/
  /* Set the SPI parameters */
  SpiHandle.Instance               = SPIx;  //SPI2;  // todo: which SPI ?
  
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;  //todo most slow, check others 
  //SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	
	SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
	
	// Arduino is working on CPOL=0 CPHA=0 (Mode 0)
  SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE; 
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW; 
	
  //SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;   // arek: orig
  //SpiHandle.Init.CLKPolarity       = SPI_POLARITY_HIGH; // arek: orig
  //SpiHandle.Init.CLKPhase          = SPI_PHASE_2EDGE;
  //SpiHandle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
	
	SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;      //todo
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;  //todo
  SpiHandle.Init.Mode = SPI_MODE_MASTER;

	res = HAL_SPI_Init(&SpiHandle);
  if(HAL_OK != res)
  {
		//FaultHandler(res);  //todo
  }
	
}


void MCAL_SPI_Tx(uint8_t *pData, uint16_t size)
{
	HAL_StatusTypeDef res = HAL_ERROR; 
	
	res = HAL_SPI_Transmit(&SpiHandle, pData, size, 1000);  // timeout 1s
  //HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
  switch (res)
  {
    case HAL_OK:  
      // Communication is completed
    
      // Turn LED2 on: Transfer in transmission process is correct 
      //BSP_LED_On(LED2);
      break;  
    
    case HAL_TIMEOUT:
      //FaultHandler(res);  //todo
      break;  
    
    case HAL_ERROR:
      //FaultHandler(res);  //todo
      break;
		
    default:
      break;
  }
	
}


void MCAL_SPI_DeInit()
{
  HAL_StatusTypeDef res = HAL_ERROR;

	// HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *hspi);
	res = HAL_SPI_DeInit(&SpiHandle);
  if(HAL_OK != res)
  {
		//FaultHandler(res); //todo
  }
	
}

//#endif
