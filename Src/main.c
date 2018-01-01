/**
  ******************************************************************************
  * @file    SPI/SPI_FullDuplex_ComPolling/Src/main.c 
  * @author  MCD Application Team
  * @version V1.7.0
  * @date    31-May-2016
  * @brief   This sample code shows how to use STM32L0xx SPI HAL API to transmit 
  *          and receive a data buffer with a communication process based on
  *          Polling transfer. 
  *          The communication is done using 2 Boards.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* arek:
 * Choose push-pull without pull up and without pull down.
 * Slowest speed.
 *
 *
 *
 * https://electronics.stackexchange.com/questions/156930/stm32-understanding-gpio-settings
 *
 * Display:                                           Nucleo L053R8
 *
 * OLED_RST    9 ------------------------------------------------- PC0
 * OLED_DC     8 ------------------------------------------------- PC1
 * OLED_CS    10 ------------------------------------------------- PC2
 * SPI_MOSI   11  -  connect to the DIN pin of OLED - D11 - MOSI - PA7
 * SPI_SCK    13  -  connect to the CLK pin of OLED - D13 - SCK  - PA5
 *
 */

// Log
//
// 1. 
// Longer and shorter version of GPIO is working
// Macro definition for GPIO On/Off done.
//
// 2.[171203]
// Not working
// todo: 
// Check MOSI SCLK pins and spi config.
// Check is CS should be not SPI CS - as D10 arduino (PB6)
// This is set to PB14 and PB15, but default could be PA7 (D11) MOSI and PA5 (D13) SCK
// following arduino settings. Check is PA7 and PA5 are SPI2, if not change correctly.
// Finally check the spi clock divider.
//
// 3.[171231_1047] 
// Atmel version is working but with SPI CPOL=0 and CPHA=0, what is different
// than in OLED datasheet (CPOL=1, CPHA=1)
//
// The test section run alone is working:
//		BSP_LED_Toggle(LED2);
//		HAL_Delay(250);
// 
// But after add basic sh1106 related, LED2 is not flashing.
// So this is stopped somewhere. Check it with debugger.
//
//  uint8_t oled_buf[WIDTH * HEIGHT / 8];
//	
//  // display an image of bitmap matrix
//  SH1106_begin();
//	
//  SH1106_clear(oled_buf);
//  SH1106_bitmap(0, 0, Waveshare12864, 128, 64, oled_buf);
//  SH1106_display(oled_buf);
//	HAL_Delay(2000);
//  SH1106_clear(oled_buf);
//
//

// TODO:
//
// 2.
// Check SPI configration
// Arduino default:
// https://arduino.stackexchange.com/questions/16348/how-do-you-use-spi-on-an-arduino
//
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
//
//
// 3.
// Display driver
//
// 4.
// Disable not necessary MCAL modules - stm32l0xx_hal_conf.h
//
// 5.
// Use non blcoking SPI
//


// Display OLED start up.
//
// [171203]
// Not working
// todo: 
// Check MOSI SCLK pins and spi config.
// Check is CS should be not SPI CS - as D10 arduino (PB6)
// This is set to PB14 and PB15, but default could be PA7 (D11) MOSI and PA5 (D13) SCK
// following arduino settings. Check is PA7 and PA5 are SPI2, if not change correctly.
// Finally check the spi clock divider.


/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "sh1106.h"

//test
#include "mcal_gpio.h"



/** @addtogroup STM32L0xx_HAL_Examples
  * @{
  */

/** @addtogroup SPI_FullDuplex_ComPolling
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Uncomment this line to use the board as master, if not it is used as slave */
//#define MASTER_BOARD

/* Private variables ---------------------------------------------------------*/


///////////////////////////////////////////////////////////////////////////////
// SPI - begin
///////////////////////////////////////////////////////////////////////////////
#if 0

/* SPI handler declaration */
SPI_HandleTypeDef SpiHandle;

/* Buffer used for transmission */
uint8_t aTxBuffer[] = "****SPI - Two Boards communication based on Polling **** SPI Message ******** SPI Message ******** SPI Message ****";

/* Buffer used for reception */
uint8_t aRxBuffer[BUFFERSIZE];
#endif
///////////////////////////////////////////////////////////////////////////////
// SPI - end
///////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////
// GPIO - begin
///////////////////////////////////////////////////////////////////////////////
#if 0

#define OLED_RST_PIN        GPIO_PIN_0
#define OLED_DC_PIN         GPIO_PIN_1
#define OLED_CS_PIN         GPIO_PIN_2
#define OLED_GPIO_PORT      GPIOC

#define OLED_RST_SET        HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_RST_PIN, GPIO_PIN_SET)
#define OLED_RST_RESET      HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_RST_PIN, GPIO_PIN_RESET)
#define OLED_DC_SET         HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_DC_PIN, GPIO_PIN_SET)
#define OLED_DC_RESET       HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_DC_PIN, GPIO_PIN_RESET)
#define OLED_CS_SET         HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_CS_PIN, GPIO_PIN_SET)
#define OLED_CS_RESET       HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_CS_PIN, GPIO_PIN_RESET)



//static GPIO_InitTypeDef  GPIO_InitStruct;
/*
static GPIO_InitTypeDef  GPIO_Init_OLED_RST;
static GPIO_InitTypeDef  GPIO_Init_OLED_DC;
static GPIO_InitTypeDef  GPIO_Init_OLED_CS;
*/
// test shorter version:
static GPIO_InitTypeDef  GPIO_InitStruct;

#endif

///////////////////////////////////////////////////////////////////////////////
// GPIO - end
///////////////////////////////////////////////////////////////////////////////




/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);  //todo remove
static void Timeout_Error_Handler(void);  //todo remove
//static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	
  /* STM32L0xx HAL library initialization:
       - Configure the Flash prefetch, Flash preread and Buffer caches
       - Systick timer is configured by default as source of time base, but user 
             can eventually implement his proper time base source (a general purpose 
             timer for example or other time source), keeping in mind that Time base 
             duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
             handled in milliseconds basis.
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure LED2 */
  BSP_LED_Init(LED2);

  /* Configure the system clock to 2 Mhz */
  SystemClock_Config();

	
  /////////////////////////////////////////////////////////////////////////////
	// GPIO begin
  /////////////////////////////////////////////////////////////////////////////

  /*
  // -1- Enable GPIOA Clock (to be able to program the configuration registers) 
  //__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

  // Test longer version  
  
  GPIO_Init_OLED_RST.Pin = (OLED_RST_PIN);
  GPIO_Init_OLED_RST.Mode = GPIO_MODE_OUTPUT_PP; // Output push-pull
  GPIO_Init_OLED_RST.Pull = GPIO_NOPULL;         //No pull up/down because this is not led.
  GPIO_Init_OLED_RST.Speed = GPIO_SPEED_FREQ_HIGH  ;

  GPIO_Init_OLED_DC.Pin = (OLED_DC_PIN);
  GPIO_Init_OLED_DC.Mode = GPIO_MODE_OUTPUT_PP; // Output push-pull
  GPIO_Init_OLED_DC.Pull = GPIO_NOPULL;         //No pull up/down because this is not led.
  GPIO_Init_OLED_DC.Speed = GPIO_SPEED_FREQ_HIGH  ;

  GPIO_Init_OLED_CS.Pin = (OLED_CS_PIN);
  GPIO_Init_OLED_CS.Mode = GPIO_MODE_OUTPUT_PP; // Output push-pull
  GPIO_Init_OLED_CS.Pull = GPIO_NOPULL;         //No pull up/down because this is not led.
  GPIO_Init_OLED_CS.Speed = GPIO_SPEED_FREQ_HIGH  ;
	
  //HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
  HAL_GPIO_Init(OLED_GPIO_PORT, &GPIO_Init_OLED_RST); 
  HAL_GPIO_Init(OLED_GPIO_PORT, &GPIO_Init_OLED_DC); 
  HAL_GPIO_Init(OLED_GPIO_PORT, &GPIO_Init_OLED_CS); 
  */


  //todo test shorter GPIO version
	/*
  GPIO_InitStruct.Pin = (OLED_RST_PIN);
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Output push-pull
  GPIO_InitStruct.Pull = GPIO_NOPULL;         //No pull up/down because this is not led.
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH  ;
  HAL_GPIO_Init(OLED_GPIO_PORT, &GPIO_InitStruct); 
  GPIO_InitStruct.Pin = (OLED_DC_PIN);
  HAL_GPIO_Init(OLED_GPIO_PORT, &GPIO_InitStruct); 
  GPIO_InitStruct.Pin = (OLED_CS_PIN);
  HAL_GPIO_Init(OLED_GPIO_PORT, &GPIO_InitStruct); 
  */

  //HAL_Delay(100);

  // #define LED2_PIN        GPIO_PIN_5
  // #define LED2_GPIO_PORT  GPIOA
  //GPIO_TypeDef* LED_PORT[LEDn] = {LED2_GPIO_PORT};
  //const uint16_t LED_PIN[LEDn] = {LED2_PIN};
  //HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET); 
  //void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
  /*
  HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_RST_PIN, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_DC_PIN, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_CS_PIN, GPIO_PIN_RESET); 

  HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_RST_PIN, GPIO_PIN_SET); 
  HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_DC_PIN, GPIO_PIN_SET); 
  HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_CS_PIN, GPIO_PIN_SET); 
	*/

  /////////////////////////////////////////////////////////////////////////////
	// GPIO end
  /////////////////////////////////////////////////////////////////////////////


 
  /////////////////////////////////////////////////////////////////////////////
	// SPI tests - begin
  /////////////////////////////////////////////////////////////////////////////	

/*
  //##-1- Configure the SPI peripheral #######################################
  // Set the SPI parameters 
  SpiHandle.Instance               = SPIx;
  
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
  
#ifdef MASTER_BOARD
  SpiHandle.Init.Mode = SPI_MODE_MASTER;
#else
  SpiHandle.Init.Mode = SPI_MODE_SLAVE;
#endif // MASTER_BOARD 

  if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
  {
    // Initialization Error 
    Error_Handler();
  }
  
#ifdef MASTER_BOARD
  // Configure Tamper push button 
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
  BSP_LED_On(LED2);
  // Wait for Tamper Button press before starting the Communication 
  while(BSP_PB_GetState(BUTTON_KEY) != 0)
  {
  } 
  BSP_LED_Off(LED2);
#endif // MASTER_BOARD  

  // ##-2- Start the Full Duplex Communication process ########################
  // While the SPI in TransmitReceive process, user can transmit data through 
  //   "aTxBuffer" buffer & receive data through "aRxBuffer" 
  /// Timeout is set to 5S 
  
  switch(HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, BUFFERSIZE, 5000))
  {
  case HAL_OK:  
    // Communication is completed ___________________________________________
    // Compare the sent and received buffers 
    if(Buffercmp((uint8_t*)aTxBuffer, (uint8_t*)aRxBuffer, BUFFERSIZE))
    {
      // Transfer error in transmission process 
      Error_Handler();     
    }
    
    // Turn LED2 on: Transfer in transmission process is correct 
    BSP_LED_On(LED2);
    break;  
    
  case HAL_TIMEOUT:
    // A Timeout Occur ______________________________________________________
    // Call Timeout Handler
    Timeout_Error_Handler();  
    break;  
    
    // An Error Occur ______________________________________________________ 
  case HAL_ERROR:
    // Call Timeout Handler
    Error_Handler();  
    break;
  default:
    break;
  }
*/
	
	
  /////////////////////////////////////////////////////////////////////////////
	// SPI tests - end
  /////////////////////////////////////////////////////////////////////////////	


  /////////////////////////////////////////////////////////////////////////////
	// OLED tests - begin
  /////////////////////////////////////////////////////////////////////////////	

  // Current test
	
	//todo move to better place
  uint8_t oled_buf[WIDTH * HEIGHT / 8];
	
  // display an image of bitmap matrix
  SH1106_begin();
	
	/*
  SH1106_clear(oled_buf);
  SH1106_bitmap(0, 0, Waveshare12864, 128, 64, oled_buf);
  SH1106_display(oled_buf);
	HAL_Delay(2000);
  SH1106_clear(oled_buf);
  */

/*
  // display images of bitmap matrix
  SH1106_bitmap(0, 2, Signal816, 16, 8, oled_buf); 
  SH1106_bitmap(24, 2,Bluetooth88, 8, 8, oled_buf); 
  SH1106_bitmap(40, 2, Msg816, 16, 8, oled_buf); 
  SH1106_bitmap(64, 2, GPRS88, 8, 8, oled_buf); 
  SH1106_bitmap(90, 2, Alarm88, 8, 8, oled_buf); 
  SH1106_bitmap(112, 2, Bat816, 16, 8, oled_buf); 

  SH1106_string(0, 52, "MUSIC", 12, 0, oled_buf); 
  SH1106_string(52, 52, "MENU", 12, 0, oled_buf); 
  SH1106_string(98, 52, "PHONE", 12, 0, oled_buf);
  */



  /*
  //SH1106_clear(oled_buf);
  SH1106_char3216(0, 16, '1', oled_buf);
  SH1106_char3216(16, 16, '2', oled_buf);
  SH1106_char3216(32, 16, ':', oled_buf);
  SH1106_char3216(48, 16, '3', oled_buf);
  SH1106_char3216(64, 16, '4', oled_buf);
  SH1106_char3216(80, 16, ':', oled_buf);
  SH1106_char3216(96, 16, '5', oled_buf);
  SH1106_char3216(112, 16, '6', oled_buf);

  SH1106_display(oled_buf); 
  */

  /////////////////////////////////////////////////////////////////////////////
	// OLED tests - end
  /////////////////////////////////////////////////////////////////////////////	

  // OLED pinstest
	//MCAL_GPIO_Init(); //todo: what about deinit ?	

	
	
  // Infinite loop 
  while (1)
  {
		BSP_LED_Toggle(LED2);
		HAL_Delay(250);
		
    //BSP_LED_Toggle(LED2);
		//OLED_RST_SET;
		//OLED_DC_RESET;
		//OLED_CS_SET;		
    //HAL_Delay(500);
		//OLED_RST_RESET;
		//OLED_DC_SET;
		//OLED_CS_RESET;
		//HAL_Delay(500);

		
    // OLED pins test	
    /*
		digitalWrite(OLED_CS, HIGH); 
		digitalWrite(OLED_RST, HIGH);
		digitalWrite(OLED_DC, HIGH);
    HAL_Delay(1000);  
    digitalWrite(OLED_CS, LOW); 
		digitalWrite(OLED_RST, LOW);
		digitalWrite(OLED_DC, LOW);
		HAL_Delay(1000);
		*/
		
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Infinite loop */
  while(1)
  {
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 2000000
  *            HCLK(Hz)                       = 2000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            Flash Latency(WS)              = 0
  *            Main regulator output voltage  = Scale3 mode
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  
  /* Enable MSI Oscillator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.MSICalibrationValue=0x00;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  
  /* Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
  * @brief  SPI error callbacks
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
 void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  /* Infinite loop */
  while(1)
  {
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Timeout_Error_Handler(void)
{
  /* Infinite loop */
  while(1)
  {
  }
}

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0  : pBuffer1 identical to pBuffer2
  *         >0 : pBuffer1 differs from pBuffer2
  */
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
