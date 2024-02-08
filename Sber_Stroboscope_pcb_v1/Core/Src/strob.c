/*
 * strob.c
 *
 *  Created on: Feb 7, 2024
 *      Author: OMEN
 */

#include "strob.h"
#include "io_interface.h"
#include <stdlib.h>

extern TIM_HandleTypeDef htim2;

void out24V_on(){
	STATUS_LED_YELLOW_ON
	OUT_WHITE_ON
}

void out24V_off(){
	STATUS_LED_YELLOW_OFF
	OUT_WHITE_OFF
}

void time_init(){
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

void set_freq(double *pPeriod, double *pPulse){
	uint16_t freq = 0;
    uint8_t rxData[20]={0};
    double divider=0;
    double periodStart = *pPeriod;

    while(1){
		printf("Enter integer frequency value from 1 to 1000Hz\n");
		UART_Receive(rxData, sizeof(rxData));
		freq=atoi(rxData);

		if(freq>=1 && freq<=1000){
			break;
		}
		else{
			printf("You entered wrong frequency\n");
		}
    }

	*pPeriod=100000 / freq;
	divider = periodStart / *pPeriod;
	*pPulse=*pPulse/divider;
}

void set_duty_cycle(double *pPulse, double *pPeriod){
    uint8_t rxData[20]={0};
    double pulseInput;

    while(1)
    {
		printf("Enter integer duty cycle in percent from 0 to 100%\n");
		UART_Receive(rxData, sizeof(rxData));
		pulseInput=atoi(rxData);

		if(pulseInput>=1 && pulseInput<=100){
			break;
		}
		else{
			printf("You entered wrong duty cycle\n");
		}
    }

	*pPulse=*pPeriod/100 * pulseInput;
}

void clear_rxData(uint8_t* pRxData){
	for (int i=0; i<20; i++){
		pRxData[i]=0;
	}
}

void status(double *pPulse, double *pPeriod, uint8_t *pVOut){
	uint32_t currentFreq;
	uint32_t currentDutyCycle;

	currentFreq = 100000 /(uint32_t)*pPeriod;
	currentDutyCycle=100 * (uint32_t)*pPulse / (uint32_t)*pPeriod;

	printf("Current frequency is %d Hz\n", currentFreq);
	printf("Current duty cycle is %d %%\n", currentDutyCycle);

	if(*pVOut){
		printf("Out 24V is on\n");
	}
	else{
		printf("Out 24V is off\n");
	}
}

