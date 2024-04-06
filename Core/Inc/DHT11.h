/*
 * DHT11.h
 *
 *  Created on: Apr 30, 2020
 *      Author: Controllerstech
 */

#ifndef INC_DHT11_H_
#define INC_DHT11_H_

#include "stm32f4xx_hal.h"


void DHT_Delay (uint16_t time);

void DHT11_Start (void);

uint8_t DHT11_Check_Response (void);

uint8_t DHT11_Read (void);

uint8_t DHT11_Get_Data (int *Temperature, int *Humidity);

#endif /* INC_DHT11_H_ */
