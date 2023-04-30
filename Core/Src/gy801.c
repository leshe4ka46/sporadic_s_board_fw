#include "gy801.h"
#include <stdio.h>
#include <math.h>
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
	/*BMP180_get_data();
	BMP180_upd_data();
	ADXL_get_data();*/
	GYRO_get_data();
	//mag_get_data();
	lsm303dlhc_get_acc();
	lsm303dlhc_get_mag();
	/*char* gy_data[100];
	//sprintf((char*)gy_data,"X:%06d Y:%06d Z:%06d %p %p %p",_data_gy->lsm303dlhc_mag.mx_raw,_data_gy->lsm303dlhc_mag.my_raw,_data_gy->lsm303dlhc_mag.mz_raw,&_data_gy->lsm303dlhc_mag.mx,&_data_gy->lsm303dlhc_mag.my,&_data_gy->lsm303dlhc_mag.mz);
	//sprintf((char*)gy_data,"X:%06d Y:%06d Z:%06d",_data_gy->lsm303dlhc_mag.mx,_data_gy->lsm303dlhc_mag.my,_data_gy->lsm303dlhc_mag.mz);
	//sprintf((char*)gy_data,"%ld;%ld;%ld.%ld;%d;%d;%d;%f;%f;%f;%f;%f;%f;%f;%f;%f|",HAL_GetTick(),_data_gy->bmp180.pressure,_data_gy->bmp180.temp/10,_data_gy->bmp180.temp%10,_data_gy->adxl345.ax,_data_gy->adxl345.ay,_data_gy->adxl345.az,(float)_data_gy->l3g4200d.gx*M_PI/180/ 131.0,(float)_data_gy->l3g4200d.gy*M_PI/180/ 131.0,(float)_data_gy->l3g4200d.gz*M_PI/180/ 131.0,(float)_data_gy->lsm303dlhc.ax*21.5625,(float)_data_gy->lsm303dlhc.ay*21.5625,(float)_data_gy->lsm303dlhc.az*21.5625,_data_gy->lsm303dlhc_mag.mx,_data_gy->lsm303dlhc_mag.my,_data_gy->lsm303dlhc_mag.mz);
	sprintf((char*)gy_data,"%ld;%f;%f;%f;%f;%f;%f;%f;%f;%f|",HAL_GetTick(),(float)_data_gy->l3g4200d.gx*M_PI/180/ 131.0,(float)_data_gy->l3g4200d.gy*M_PI/180/ 131.0,(float)_data_gy->l3g4200d.gz*M_PI/180/ 131.0,(float)_data_gy->lsm303dlhc.ax*21.5625,(float)_data_gy->lsm303dlhc.ay*21.5625,(float)_data_gy->lsm303dlhc.az*21.5625,_data_gy->lsm303dlhc_mag.mx*0.1388,_data_gy->lsm303dlhc_mag.my*0.1388,_data_gy->lsm303dlhc_mag.mz*0.1388);
	log_s((char*)gy_data);*/

}
