#ifndef INC_GYRO_H_
#define INC_GYRO_H_
#include "stm32f4xx_hal.h"
#include "debug.h"
#include "structs.h"

#define GYRO_ADDR 0x69
#define GYRO_REG_DEVID 0x0F
#define GYRO_REG_CTRL_REG1 0x20
#define GYRO_REG_CTRL_REG2 0x21
#define GYRO_REG_CTRL_REG3 0x22
#define GYRO_REG_CTRL_REG4 0x23

uint8_t GYRO_ReadReg(uint8_t reg);
uint8_t GYRO_init(I2C_HandleTypeDef *i2c, readings *data);
void GYRO_power(uint8_t pwr);
void GYRO_get_data();

#endif /* INC_GYRO_H_ */
