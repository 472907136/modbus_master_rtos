#ifndef __RS485_H
#define __RS485_H
#include "stm32f4xx_hal.h"
#include "main.h"

#define 	RS485_TX_EN()	HAL_GPIO_WritePin(RS485EN_GPIO_Port, RS485EN_Pin, GPIO_PIN_SET)
#define 	RS485_RX_EN()	HAL_GPIO_WritePin(RS485EN_GPIO_Port, RS485EN_Pin, GPIO_PIN_RESET)



extern unsigned char uartRevChar;


int uart_receive_update(void);
#endif

