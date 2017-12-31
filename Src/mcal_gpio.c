
#include "mcal_gpio.h"


static GPIO_InitTypeDef  GPIO_InitStruct;


void MCAL_GPIO_Init()
{

  /* -1- Enable GPIOA Clock (to be able to program the configuration registers) */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct.Pin = (OLED_RST_PIN);
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Output push-pull
  GPIO_InitStruct.Pull = GPIO_NOPULL;         //No pull up/down because this is not led.
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH  ;
  HAL_GPIO_Init(OLED_GPIO_PORT, &GPIO_InitStruct); 
  GPIO_InitStruct.Pin = (OLED_DC_PIN);
  HAL_GPIO_Init(OLED_GPIO_PORT, &GPIO_InitStruct); 
  GPIO_InitStruct.Pin = (OLED_CS_PIN);
  HAL_GPIO_Init(OLED_GPIO_PORT, &GPIO_InitStruct); 
	
}

