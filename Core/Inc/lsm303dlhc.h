#ifndef INC_MAG_H_
#define INC_MAG_H_
#include "stm32f4xx_hal.h"
#include "debug.h"
#include "structs.h"

#define LSM303DLHC_MAG_ADDR 0x1e
#define LSM303DLHC_ACCEL_ADDR 0x19

/* accel */
#define LSM303DLHC_REG_OUT_X_L_A 0x28
#define LSM303DLHC_REG_OUT_X_H_A 0x29
#define LSM303DLHC_REG_OUT_Y_L_A 0x2A
#define LSM303DLHC_REG_OUT_Y_H_A 0x2B
#define LSM303DLHC_REG_OUT_Z_L_A 0x2C
#define LSM303DLHC_REG_OUT_Z_H_A 0x2D

#define LSM303DLHC_REG_WHOAMI 0x0F
#define LSM303DLHC_REG_CTRL_REG1_A 0x20
#define LSM303DLHC_REG_CTRL_REG2_A 0x21
#define LSM303DLHC_REG_CTRL_REG3_A 0x22
#define LSM303DLHC_REG_CTRL_REG4_A 0x23
#define LSM303DLHC_REG_CTRL_REG5_A 0x24

/* mag */
#define LSM303DLHC_REG_CRA_REG_M 0x00
#define LSM303DLHC_REG_CRB_REG_M 0x01
#define LSM303DLHC_REG_MR_REG_M  0x02
#define LSM303DLHC_REG_OUT_X_H_M 0x03
#define LSM303DLHC_REG_OUT_X_L_M 0x04
#define LSM303DLHC_REG_OUT_Z_H_M 0x05
#define LSM303DLHC_REG_OUT_Z_L_M 0x06
#define LSM303DLHC_REG_OUT_Y_H_M 0x07
#define LSM303DLHC_REG_OUT_Y_L_M 0x08
#define LSM303DLHC_REG_SR_REG_M  0x09
#define LSM303DLHC_REG_IRA_REG_M 0x0A
#define LSM303DLHC_REG_IRB_REG_M 0x0B
#define LSM303DLHC_REG_IRC_REG_M 0x0C
#define LSM303DLHC_REG_TEMP_OUT_H_M 0x31
#define LSM303DLHC_REG_TEMP_OUT_L_M 0x32

//type 0 - mag_addr   1 - acc_addr
uint8_t mag_read_data(uint8_t type, uint8_t reg);
void mag_write_data(uint8_t type, uint8_t reg, uint8_t cmd);
uint8_t mag_init(I2C_HandleTypeDef *i2c, readings *data);
void lsm303dlhc_get_acc();
void lsm303dlhc_get_mag();

#endif /* INC_MAG_H_ */
