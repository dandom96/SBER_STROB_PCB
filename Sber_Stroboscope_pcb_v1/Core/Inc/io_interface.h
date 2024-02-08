/*
 * print.h
 *
 *  Created on: Feb 1, 2024
 *      Author: OMEN
 */

#ifndef INC_IO_INTERFACE_H_
#define INC_IO_INTERFACE_H_

#include <stm32f4xx_hal.h>

int __io_putchar(int ch);
int __io_getchar(void);
void UART_Receive(char *buffer, uint16_t size);
void UART_Transmit(char *buffer);
#endif /* INC_IO_INTERFACE_H_ */
