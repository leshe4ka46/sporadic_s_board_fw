#include "lsm303dlhc.h"
I2C_HandleTypeDef *_mag_i2c;
readings *_mag_data;
void mag_init(I2C_HandleTypeDef *i2c,readings *data){
	_mag_i2c=i2c;
	_mag_data=data;
	log_s_wnl("LSM303DLHC started---");
	if(mag_read_data(1,LSM303DLHC_REG_WHOAMI)==0x33){
		log_s("ok");
	}
	else{
		log_s("ERROR");
		return;
	}
	/* accel config */
	mag_write_data(1,LSM303DLHC_REG_CTRL_REG1_A,0b01110111);
	mag_write_data(1,LSM303DLHC_REG_CTRL_REG2_A,0b10010000);
	mag_write_data(1,LSM303DLHC_REG_CTRL_REG4_A,0b00111000);
	/* mag config */
	mag_write_data(0,LSM303DLHC_REG_CRA_REG_M,0b00011100);
	mag_write_data(0,LSM303DLHC_REG_CRB_REG_M,0b10000000);
	mag_write_data(0,LSM303DLHC_REG_MR_REG_M,0b00000000);
}


uint8_t mag_read_data(uint8_t type,uint8_t reg){
	//log_s_int("addr",((type)?LSM303DLHC_ACCEL_ADDR:LSM303DLHC_MAG_ADDR)<<1);
	uint8_t result;
	HAL_I2C_Mem_Read(_mag_i2c, (((type)?LSM303DLHC_ACCEL_ADDR:LSM303DLHC_MAG_ADDR)<<1)+!type, reg, I2C_MEMADD_SIZE_8BIT, &result, 1, 1000);
	return result;
}
void mag_write_data(uint8_t type,uint8_t reg, uint8_t cmd){
	HAL_I2C_Mem_Write(_mag_i2c, ((type)?LSM303DLHC_ACCEL_ADDR:LSM303DLHC_MAG_ADDR)<<1, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 1000);
}

void lsm303dlhc_get_acc(){
	//uint32_t tm=HAL_GetTick();
	_mag_data->lsm303dlhc.ax=((mag_read_data(1,LSM303DLHC_REG_OUT_X_H_A)<<8)|mag_read_data(1,LSM303DLHC_REG_OUT_X_L_A));
	_mag_data->lsm303dlhc.ay=((mag_read_data(1,LSM303DLHC_REG_OUT_Y_H_A)<<8)|mag_read_data(1,LSM303DLHC_REG_OUT_Y_L_A));
	_mag_data->lsm303dlhc.az=((mag_read_data(1,LSM303DLHC_REG_OUT_Z_H_A)<<8)|mag_read_data(1,LSM303DLHC_REG_OUT_Z_L_A));
	//log_s_int("ACCEL",HAL_GetTick()-tm);
}
void lsm303dlhc_get_mag(){
	/*uint8_t buffer[6];
		uint8_t i=0;

		buffer[0] = mag_read_data(0,LSM303DLHC_REG_OUT_X_H_M);
		buffer[1] = mag_read_data(0,LSM303DLHC_REG_OUT_X_L_M);
		buffer[2] = mag_read_data(0,LSM303DLHC_REG_OUT_Y_H_M);
		buffer[3] = mag_read_data(0,LSM303DLHC_REG_OUT_Y_L_M);
		buffer[4] = mag_read_data(0,LSM303DLHC_REG_OUT_Z_H_M);
		buffer[5] = mag_read_data(0,LSM303DLHC_REG_OUT_Z_L_M);

		for(i=0;i<3;i++)
		{
			log_s_int("v",((uint16_t)((uint16_t)buffer[2*i]<<8)+buffer[2*i+1]));
		}*/
	//uint32_t tb=HAL_GetTick();
	_mag_data->lsm303dlhc_mag.mx_raw=(uint16_t)((uint16_t)(mag_read_data(0,LSM303DLHC_REG_OUT_X_H_M)<<8)|mag_read_data(0,LSM303DLHC_REG_OUT_X_L_M));
	_mag_data->lsm303dlhc_mag.my_raw=(uint16_t)((uint16_t)(mag_read_data(0,LSM303DLHC_REG_OUT_Y_H_M)<<8)|mag_read_data(0,LSM303DLHC_REG_OUT_Y_L_M));
	_mag_data->lsm303dlhc_mag.mz_raw=(uint16_t)((uint16_t)(mag_read_data(0,LSM303DLHC_REG_OUT_Z_H_M)<<8)|mag_read_data(0,LSM303DLHC_REG_OUT_Z_L_M));
	_mag_data->lsm303dlhc_mag.mx=4.383713*(_mag_data->lsm303dlhc_mag.mx_raw+43.206892)+0.021545*(_mag_data->lsm303dlhc_mag.my_raw+-25.655564)+-0.077899*(_mag_data->lsm303dlhc_mag.mz_raw+-0.569458);
	_mag_data->lsm303dlhc_mag.my=0.021545*(_mag_data->lsm303dlhc_mag.mx_raw+43.206892)+4.431516*(_mag_data->lsm303dlhc_mag.my_raw+-25.655564)+0.225705*(_mag_data->lsm303dlhc_mag.mz_raw+-0.569458);
	_mag_data->lsm303dlhc_mag.mz=-0.077899*(_mag_data->lsm303dlhc_mag.mx_raw+43.206892)+0.066753*(_mag_data->lsm303dlhc_mag.my_raw+-25.655564)+5.335236*(_mag_data->lsm303dlhc_mag.mz_raw+-0.569458);
	//log_s_int("MAG",HAL_GetTick()-tb);
}
