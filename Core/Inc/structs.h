#ifndef STRUCTS_H
#define STRUCTS_H

#include <stm32f4xx_hal.h>

typedef struct{
	double pressure;
	int32_t temp;
} bmp180;

typedef struct{
	int16_t ax;
	int16_t ay;
	int16_t az;
	uint16_t scale;
} adxl345;

typedef struct{
	bmp180 bmp180;
	adxl345 adxl345;
} readings;

#endif // STRUCTS_H
