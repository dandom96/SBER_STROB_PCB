/*
 * strob.h
 *
 *  Created on: Feb 7, 2024
 *      Author: OMEN
 */

#ifndef INC_STROB_H_
#define INC_STROB_H_

#include <stm32f4xx_hal.h>

#define GREEN_LED_TEST_ON HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
#define GREEN_LED_TEST_OFF HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

#define STATUS_LED_YELLOW_ON HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
#define STATUS_LED_YELLOW_OFF HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

#define OUT_WHITE_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
#define OUT_WHITE_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

void out24V_on();
void out24V_off();
void time_init();
void clear_rxData(uint8_t* pRxData);
void set_freq(double *pPeriod, double *pPulse);
void set_duty_cycle(double *pPulse, double *pPeriod);
void status(double *pPulse, double *pPeriod, uint8_t *pVOut);


#endif /* INC_STROB_H_ */
