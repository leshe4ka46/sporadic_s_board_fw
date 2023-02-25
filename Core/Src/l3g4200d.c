#include "l3g4200d.h"
#include "math.h"
#include "stdio.h"
#include "string.h"
I2C_HandleTypeDef *_gyro_i2c;
readings *_gyro_data;
void GYRO_init(I2C_HandleTypeDef *i2c,readings *data){
	_gyro_i2c=i2c;
	_gyro_data=data;
	log_s_wnl("L3G4200D started---");
	if(GYRO_ReadReg(GYRO_REG_DEVID)==211){
		log_s("ok");
	}
	else{
		log_s("ERROR");
	}
	GYRO_WriteReg(GYRO_REG_CTRL_REG4,0b00110000);
}

void GYRO_WriteReg(uint8_t reg, uint8_t cmd) {
	uint8_t arr[2] = { reg, cmd };
	HAL_I2C_Master_Transmit(_gyro_i2c, GYRO_ADDR<<1, arr, 2, 1000);
}

uint8_t GYRO_ReadReg(uint8_t reg) {
	HAL_I2C_Master_Transmit(_gyro_i2c, GYRO_ADDR<<1, &reg, 1, 1000);
	uint8_t result;
	HAL_I2C_Master_Receive(_gyro_i2c, GYRO_ADDR<<1, &result, 1, 1000);
	return result;
}

void GYRO_power(uint8_t pwr){
	GYRO_WriteReg(GYRO_REG_CTRL_REG1,pwr?0x0F:0x00);
}

void GYRO_get_data(){
	_gyro_data->l3g4200d.gx=((GYRO_ReadReg(0x29)<<8)|GYRO_ReadReg(0x28));
	_gyro_data->l3g4200d.gy=((GYRO_ReadReg(0x2B)<<8)|GYRO_ReadReg(0x2A));
	_gyro_data->l3g4200d.gz=((GYRO_ReadReg(0x2D)<<8)|GYRO_ReadReg(0x2C));
}
