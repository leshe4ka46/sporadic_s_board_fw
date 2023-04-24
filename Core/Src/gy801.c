#include "gy801.h"
#include <stdio.h>
readings *_data_gy;
void GY801_init(I2C_HandleTypeDef *i2c,readings *data){
	//_i2c=i2c;
	_data_gy=data;
	ADXL_init(i2c,data);
	ADXL_set_range(RANGE_2G);
	ADXL_full_res(1);
	ADXL_set_rate(HZ100);
	ADXL_power(1);
	BMP180_init(i2c,data);
	BMP180_set_oss(1);
	BMP180_get_calibration_coefficients();
	GYRO_init(i2c,data);
	GYRO_power(1);
	mag_init(i2c,data);


}

void GY801_update_data(){
	BMP180_get_data();
	BMP180_upd_data();
	ADXL_get_data();
	GYRO_get_data();
	mag_get_data();
	char* gy_data[100];
	sprintf((char*)gy_data,"%ld;%ld;%ld.%ld;%d;%d;%d;%d;%d;%d;%d;%d;%d;%f*|",HAL_GetTick(),_data_gy->bmp180.pressure,_data_gy->bmp180.temp/10,_data_gy->bmp180.temp%10,_data_gy->adxl345.ax,_data_gy->adxl345.ay,_data_gy->adxl345.az,_data_gy->l3g4200d.gx,_data_gy->l3g4200d.gy,_data_gy->l3g4200d.gz,_data_gy->mmc5883ma.mx,_data_gy->mmc5883ma.my,_data_gy->mmc5883ma.mz,atan(_data_gy->mmc5883ma.mx_cal/_data_gy->mmc5883ma.my_cal)*180/355*113);
	log_s((char*)gy_data);

}
