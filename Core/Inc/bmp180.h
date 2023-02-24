#ifndef INC_BMP_H_
#define INC_BMP_H_
#include "stm32f4xx_hal.h"
#include "debug.h"
#include "structs.h"


#define BMP180_ADDR 0x77
#define BMP180_CONTROL_REG 0xF4
#define BMP180_MSB 0xF6
#define BMP180_LSB 0xF7
#define BMP180_XLSB_REG 0xf8
#define BMP180_CMD_TEMP 0x2e
#define BMP180_CHIP_ID  0x55
#define BMP180_GET_ID_REG 0xD0



#define BMP180_AC1_MSB 0xAA
#define BMP180_AC1_LSB 0xAB
#define BMP180_AC2_MSB 0xAC
#define BMP180_AC2_LSB 0xAD
#define BMP180_AC3_MSB 0xAE
#define BMP180_AC3_LSB 0xAF
#define BMP180_AC4_MSB 0xB0
#define BMP180_AC4_LSB 0xB1
#define BMP180_AC5_MSB 0xB2
#define BMP180_AC5_LSB 0xB3
#define BMP180_AC6_MSB 0xB4
#define BMP180_AC6_LSB 0xB5
#define BMP180_B1_MSB  0xB6
#define BMP180_B1_LSB  0xB7
#define BMP180_B2_MSB  0xB8
#define BMP180_B2_LSB  0xB9
#define BMP180_MB_MSB  0xBA
#define BMP180_MB_LSB  0xBB
#define BMP180_MC_MSB  0xBC
#define BMP180_MC_LSB  0xBD
#define BMP180_MD_MSB  0xBE
#define BMP180_MD_LSB  0xBF


typedef enum BMP180_REGS {
    AC1_MSB = 0xAA,
    AC1_LSB,
    AC2_MSB,
    AC2_LSB,
    AC3_MSB,
    AC3_LSB,
    AC4_MSB,
    AC4_LSB,
    AC5_MSB,
    AC5_LSB,
    AC6_MSB,
    AC6_LSB,
    B1_MSB,
    B1_LSB,
    B2_MSB,
    B2_LSB,
    MB_MSB,
    MB_LSB,
    MC_MSB,
    MC_LSB,
    MD_MSB,
    MD_LSB
} BMP180_REGS;


typedef struct BMP180_SETTINGS {
	short AC1;
	short AC2;
	short AC3;
	unsigned short AC4;
	unsigned short AC5;
	unsigned short AC6;
	short B1;
	short B2;
	short MB;
	short MC;
	short MD;
} BMP180_SETTINGS;

uint8_t BMP180_ReadReg(uint8_t reg);

void BMP180_init(I2C_HandleTypeDef *i2c,readings *data);

void BMP180_get_calibration_coefficients();

void BMP180_set_oss(uint8_t oss);

void BMP180_get_data();

void BMP180_upd_data();
#endif /* INC_BMP_H_ */
