#ifndef __MCAL_GPIO_H
#define __MCAL_GPIO_H

#include "stm32l0xx_hal.h"
//#include <stdint.h>  // uint8_t, etc.

//


#define OLED_RST_PIN        GPIO_PIN_0  // PORT C, pin 0
#define OLED_DC_PIN         GPIO_PIN_1  // PORT C, pin 1
#define OLED_CS_PIN         GPIO_PIN_2  // PORT C, pin 2
#define OLED_GPIO_PORT      GPIOC

//todo: replaced for test
/*
#define OLED_RST_SET        HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_RST_PIN, GPIO_PIN_SET)
#define OLED_RST_RESET      HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_RST_PIN, GPIO_PIN_RESET)
#define OLED_DC_SET         HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_DC_PIN, GPIO_PIN_SET)
#define OLED_DC_RESET       HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_DC_PIN, GPIO_PIN_RESET)
#define OLED_CS_SET         HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_CS_PIN, GPIO_PIN_SET)
#define OLED_CS_RESET       HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_CS_PIN, GPIO_PIN_RESET)
*/


#define OLED_RST_SET        __nop()
#define OLED_RST_RESET      __nop()
#define OLED_DC_SET         __nop()
#define OLED_DC_RESET       __nop()
#define OLED_CS_SET         __nop()
#define OLED_CS_RESET       __nop()




void MCAL_GPIO_Init(void);
void MCAL_GPIO_Set(uint8_t pin, uint8_t state);
void MCAL_GPIO_DeInit(void);


#endif

