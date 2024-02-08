/*
 * io_interface.c
 *
 *  Created on: Feb 1, 2024
 *      Author: OMEN
 */
#include "io_interface.h"
#include <stdio.h>
#include <stm32f4xx_hal.h>

extern UART_HandleTypeDef huart2;

int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 100);
	return ch;
}

int __io_getchar(void) {
	uint8_t ch;
    HAL_UART_Receive(&huart2, &ch, 1, 100);
    return ch;
}

void UART_Transmit(char *buffer) {
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void UART_Receive(char *buffer, uint16_t size) {
    char receivedChar;
    uint16_t index = 0;

    do {
        HAL_UART_Receive(&huart2, (uint8_t*)&receivedChar, 1, HAL_MAX_DELAY);
        buffer[index++] = receivedChar;
    } while (receivedChar != '\n' && index < size - 1);

    buffer[index] = '\0';
}

