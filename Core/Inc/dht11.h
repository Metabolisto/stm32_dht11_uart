#ifndef DHT11_H
#define DHT11_H

#include "stm32f1xx_hal.h"

uint8_t DHT11_GetData(uint8_t *temperature, uint8_t *humidity);

#endif
