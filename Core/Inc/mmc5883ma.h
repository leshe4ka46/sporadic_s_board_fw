#ifndef INC_MAG_H_
#define INC_MAG_H_
#include "stm32f4xx_hal.h"
#include "debug.h"
#include "structs.h"

#define MAG_ADDR 0x30
#define MAG_REG_PRODUCT_ID 0x2F

#define MAG_REG_STATUS 0x07
#define MAG_REG_CONFIG 0x08

uint8_t mag_read_data(uint8_t reg);
void mag_write_data(uint8_t reg, uint8_t cmd);
void mag_init(I2C_HandleTypeDef *i2c,readings *data);
void mag_get_data();


#endif /* INC_MAG_H_ */
