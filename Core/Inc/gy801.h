#ifndef INC_GY_H_
#define INC_GY_H_
#include "stm32f4xx_hal.h"
#include "debug.h"
#include "bmp180.h"
#include "adxl345.h"
#include "l3g4200d.h"
#include "lsm303dlhc.h"
#include "structs.h"

void GY801_init(I2C_HandleTypeDef *i2c,readings *data);

void GY801_update_data();

#endif /* INC_GY_H_ */
