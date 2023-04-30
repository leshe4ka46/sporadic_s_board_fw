#include "adxl345.h"
#include "math.h"
#include "stdio.h"
#include "string.h"
I2C_HandleTypeDef *_adxl_i2c;
readings *_adxl_data;
uint16_t scale=0;
void ADXL_init(I2C_HandleTypeDef *i2c,readings *data){
	_adxl_i2c=i2c;
	_adxl_data=data;
	log_s_wnl("ADXL345 started---");
	if(ADXL_ReadReg(ADXL345_DEVID)==229){
		log_s("ok");
	}
	else{
		log_s("ERROR");
	}
	_adxl_data->adxl345.scale=2*(1<<(7-ADXL_get_range()));
	//log_s_int("dev id=",ADXL_ReadReg(ADXL345_DEVID));
}




void ADXL_WriteReg(uint8_t reg, uint8_t cmd) {
	uint8_t arr[2] = { reg, cmd };
	HAL_I2C_Master_Transmit(_adxl_i2c, ADXL_ADDR<<1, arr, 2, 1000);
}

uint8_t ADXL_ReadReg(uint8_t reg) {
	HAL_I2C_Master_Transmit(_adxl_i2c, ADXL_ADDR<<1, &reg, 1, 1000);
	uint8_t result;
	HAL_I2C_Master_Receive(_adxl_i2c, ADXL_ADDR<<1, &result, 1, 1000);
	return result;
	/*uint8_t result;
	HAL_I2C_Mem_Read (_adxl_i2c, ADXL_ADDR<<1, reg, 1, (uint8_t *)result, 1, 100);
	return result;*/
}
void ADXL_set_range(ADXLRANGE rng){
	ADXL_WriteReg(ADXL345_REG_DATA_FORMAT,(ADXL_ReadReg(ADXL345_REG_DATA_FORMAT) & !0x3)| rng);
	_adxl_data->adxl345.scale=2*(1<<(7-rng));
}

uint8_t ADXL_get_range(){
	return ADXL_ReadReg(ADXL345_REG_DATA_FORMAT)%4;
}

void ADXL_set_rate(ADXLBITRATE rate){
	ADXL_WriteReg(ADXL345_REG_BW_RATE,(ADXL_ReadReg(ADXL345_REG_DATA_FORMAT) & !0b111)| rate);
}
double ADXL_get_rate(){
	switch(ADXL_ReadReg(ADXL345_REG_DATA_FORMAT)%16){ //HZ6_25=6,HZ12_5,HZ25,HZ50,HZ100,HZ200,HZ400,HZ800,HZ1600,HZ3200
		case HZ6_25:
			return 6.25;
		case HZ12_5:
			return 12.5;
		case HZ25:
			return 25;
		case HZ50:
			return 50;
		case HZ100:
			return 100;
		case HZ200:
			return 200;
		case HZ400:
			return 400;
		case HZ800:
			return 800;
		case HZ1600:
			return 1600;
		case HZ3200:
			return 3200;
	}
}

void ADXL_power(uint8_t pw){
	if(pw==1){
		ADXL_WriteReg(ADXL345_REG_POWER_CTL,ADXL_ReadReg(ADXL345_REG_POWER_CTL)| ADXL345_BIT_PWR_REG_MEASURE);
	}
	else{
		ADXL_WriteReg(ADXL345_REG_POWER_CTL,ADXL_ReadReg(ADXL345_REG_POWER_CTL) & !ADXL345_BIT_PWR_REG_MEASURE);
	}
}
void ADXL_full_res(uint8_t res){
	if(res==1){
		ADXL_WriteReg(ADXL345_REG_DATA_FORMAT,ADXL_ReadReg(ADXL345_REG_DATA_FORMAT)| ADXL345_BIT_FULL_RES_SET);
	}
	else{
		ADXL_WriteReg(ADXL345_REG_DATA_FORMAT,ADXL_ReadReg(ADXL345_REG_DATA_FORMAT) & !ADXL345_BIT_FULL_RES_SET);
	}
}

void ADXL_get_data(){
	uint8_t data_rec[6];
	HAL_I2C_Mem_Read (_adxl_i2c, ADXL_ADDR<<1, 0x32, 1, (uint8_t *)data_rec, 6, 100);
	_adxl_data->adxl345.ax=((data_rec[1]<<8)|data_rec[0]);
	_adxl_data->adxl345.ay=((data_rec[3]<<8)|data_rec[2]);
	_adxl_data->adxl345.az=((data_rec[5]<<8)|data_rec[4]);
}
