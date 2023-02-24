#include "gy801.h"
#include "adxl345.h"
//I2C_HandleTypeDef *_i2c;
readings *_data_gy;
void GY801_init(I2C_HandleTypeDef *i2c,readings *data){
	//_i2c=i2c;
	_data_gy=data;
	ADXL_init(i2c,data);
	ADXL_set_range(RANGE_16G);
	ADXL_full_res(1);
	ADXL_set_rate(HZ100);
	log_s_int("df",ADXL_ReadReg(ADXL345_REG_DATA_FORMAT));
	ADXL_power(1);
	BMP180_init(i2c,data);
	BMP180_set_oss(1);
	BMP180_get_calibration_coefficients();

}

void GY801_update_data(){
	log_s("Getting data");
	BMP180_get_data();
	BMP180_upd_data();
	ADXL_get_data();
	ADXL_read();
	char* gy_data[100];
	sprintf(gy_data,"bmp180:%f %ld.%ld   adxl345: %d %d %d",_data_gy->bmp180.pressure,_data_gy->bmp180.temp/10,_data_gy->bmp180.temp%10,_data_gy->adxl345.ax,_data_gy->adxl345.ay,_data_gy->adxl345.az);
	log_s(gy_data);

}
