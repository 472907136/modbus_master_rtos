#include "rs485.h"
#include "usart.h"

unsigned char uartRevChar = 0;

int uart_receive_update(void)
{
	return HAL_UART_Receive_IT(&huart2, &uartRevChar, sizeof(uartRevChar));
}

uint16_t uartTransmit(uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	uint16_t hal_state = 0;
	hal_state = (uint16_t)HAL_UART_Transmit(&huart2, pData, Size, Timeout);
	return hal_state;
}

uint16_t uartread(uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	uint16_t hal_state = 0;
	hal_state = (uint16_t)HAL_UART_Receive(&huart2, pData, Size, Timeout);
	return hal_state;
}

