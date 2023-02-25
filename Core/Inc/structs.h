#ifndef STRUCTS_H
#define STRUCTS_H

#include <stm32f4xx_hal.h>

typedef struct{
	uint32_t pressure;
	int32_t temp;
} bmp180;

typedef struct{
	int16_t ax;
	int16_t ay;
	int16_t az;
	uint16_t scale;
} adxl345;

typedef struct{
	int16_t gx;
	int16_t gy;
	int16_t gz;
} l3g4200d;

typedef struct{
	bmp180 bmp180;
	adxl345 adxl345;
	l3g4200d l3g4200d;
} readings;

#endif // STRUCTS_H
