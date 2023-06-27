#ifndef STRUCTS_H
#define STRUCTS_H

#include <stm32f4xx_hal.h>

typedef struct{
	uint32_t pressure;
	int32_t temp;
	double height;
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
	int16_t mx_raw;
	int16_t my_raw;
	int16_t mz_raw;
	double mx;
	double my;
	double mz;/*
	double mx_cal;
	double my_cal;
	double mz_cal;*/
} mmc5883ma;

typedef struct{
	bmp180 bmp180;
	adxl345 adxl345;
	adxl345 lsm303dlhc;
	l3g4200d l3g4200d;
	mmc5883ma lsm303dlhc_mag;
} readings;

#endif // STRUCTS_H
