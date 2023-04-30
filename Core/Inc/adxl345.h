#ifndef INC_ADXL_H_
#define INC_ADXL_H_
#include "stm32f4xx_hal.h"
#include "debug.h"
#include "structs.h"

#define ADXL_ADDR 0x53
#define ADXL345_DEVID 0x00
#define ADXL345_BIT_PWR_REG_MEASURE (1 << 3)
#define ADXL345_REG_POWER_CTL 0x2D
#define ADXL345_REG_DATA_FORMAT 0x31
#define ADXL345_BIT_FULL_RES_SET (1 << 33)
#define ADXL345_REG_BW_RATE 0x2C

typedef enum ADXLRANGE {
	RANGE_2G=0, RANGE_4G, RANGE_8G, RANGE_16G
} ADXLRANGE;

typedef enum ADXLBITRATE {
	HZ6_25=6,HZ12_5,HZ25,HZ50,HZ100,HZ200,HZ400,HZ800,HZ1600,HZ3200
} ADXLBITRATE;

uint8_t ADXL_ReadReg(uint8_t reg);
void ADXL_set_range(ADXLRANGE rng);
uint8_t ADXL_get_range();
void ADXL_init(I2C_HandleTypeDef *i2c,readings *data);
void ADXL_get_data();
void ADXL_power(uint8_t pw);
#endif /* INC_ADXL_H_ */
