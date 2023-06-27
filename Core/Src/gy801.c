#include "gy801.h"
#include <stdio.h>
#include <math.h>
readings *_data_gy;

uint8_t GY801_init(I2C_HandleTypeDef *i2c, readings *data) {
	//_i2c=i2c;
	_data_gy = data;
	uint8_t ret = 0;
	ret |= ADXL_init(i2c, data);
	if (ret)
		return ret;
	ADXL_set_range(RANGE_16G);
	ADXL_full_res(1);
	ADXL_set_rate(HZ800);
	ADXL_power(1);
	ret |= BMP180_init(i2c, data);
	if (ret)
		return ret;
	BMP180_set_oss(0);
	BMP180_get_calibration_coefficients();
	ret |= GYRO_init(i2c, data);
	if (ret)
		return ret;
	GYRO_power(1);
	ret |= mag_init(i2c, data);
	if (ret)
		return ret;
	return ret;
}
int16_t axfilt[3], ayfilt[3], azfilt[3];
int16_t gxfilt[3], gyfilt[3], gzfilt[3];
int16_t temp[3];
int16_t med(uint8_t mode, int16_t new_val) {
	switch (mode) {
	case 0:
		memcpy(&temp, axfilt, sizeof(axfilt));
		break;
	case 1:
		memcpy(&temp, ayfilt, sizeof(ayfilt));
		break;
	case 2:
		memcpy(&temp, azfilt, sizeof(azfilt));
		break;
	case 3:
		memcpy(&temp, gxfilt, sizeof(gxfilt));
		break;
	case 4:
		memcpy(&temp, gyfilt, sizeof(gyfilt));
		break;
	case 5:
		memcpy(&temp, gzfilt, sizeof(gzfilt));
		break;
	}
	temp[0] = temp[1];
	temp[1] = temp[2];
	temp[2] = new_val;
	switch (mode) {
	case 0:
		memcpy(&axfilt, temp, sizeof(temp));
		break;
	case 1:
		memcpy(&ayfilt, temp, sizeof(temp));
		break;
	case 2:
		memcpy(&azfilt, temp, sizeof(temp));
		break;
	case 3:
		memcpy(&gxfilt, temp, sizeof(temp));
		break;
	case 4:
		memcpy(&gyfilt, temp, sizeof(temp));
		break;
	case 5:
		memcpy(&gzfilt, temp, sizeof(temp));
		break;
	}
	return (temp[0] < temp[1]) ?
			((temp[1] < temp[2]) ?
					temp[1] : ((temp[2] < temp[0]) ? temp[0] : temp[2])) :
			((temp[0] < temp[2]) ?
					temp[0] : ((temp[2] < temp[1]) ? temp[1] : temp[2]));
}

void GY801_update_data() {
	//BMP180_upd_data();
	ADXL_get_data();
	GYRO_get_data();
	//mag_get_data();
	lsm303dlhc_get_acc();
	lsm303dlhc_get_mag();
	_data_gy->adxl345.ax = med(0, _data_gy->adxl345.ax);
	_data_gy->adxl345.ay = med(1, _data_gy->adxl345.ay);
	_data_gy->adxl345.az = med(2, _data_gy->adxl345.az);
	_data_gy->l3g4200d.gx=med(3, _data_gy->l3g4200d.gx);
	_data_gy->l3g4200d.gy=med(4, _data_gy->l3g4200d.gy);
	_data_gy->l3g4200d.gz=med(5, _data_gy->l3g4200d.gz);

	/*
	 _data_gy->lsm303dlhc.ax=med(0,_data_gy->lsm303dlhc.ax);
	 _data_gy->lsm303dlhc.ay=med(1,_data_gy->lsm303dlhc.ay);
	 _data_gy->lsm303dlhc.az=med(2,_data_gy->lsm303dlhc.az);
	 */
	/*char* gy_data[100];
	 //sprintf((char*)gy_data,"X:%06d Y:%06d Z:%06d %p %p %p",_data_gy->lsm303dlhc_mag.mx_raw,_data_gy->lsm303dlhc_mag.my_raw,_data_gy->lsm303dlhc_mag.mz_raw,&_data_gy->lsm303dlhc_mag.mx,&_data_gy->lsm303dlhc_mag.my,&_data_gy->lsm303dlhc_mag.mz);
	 //sprintf((char*)gy_data,"X:%06d Y:%06d Z:%06d",_data_gy->lsm303dlhc_mag.mx,_data_gy->lsm303dlhc_mag.my,_data_gy->lsm303dlhc_mag.mz);
	 //sprintf((char*)gy_data,"%ld;%ld;%ld.%ld;%d;%d;%d;%f;%f;%f;%f;%f;%f;%f;%f;%f|",HAL_GetTick(),_data_gy->bmp180.pressure,_data_gy->bmp180.temp/10,_data_gy->bmp180.temp%10,_data_gy->adxl345.ax,_data_gy->adxl345.ay,_data_gy->adxl345.az,(float)_data_gy->l3g4200d.gx*M_PI/180/ 131.0,(float)_data_gy->l3g4200d.gy*M_PI/180/ 131.0,(float)_data_gy->l3g4200d.gz*M_PI/180/ 131.0,(float)_data_gy->lsm303dlhc.ax*21.5625,(float)_data_gy->lsm303dlhc.ay*21.5625,(float)_data_gy->lsm303dlhc.az*21.5625,_data_gy->lsm303dlhc_mag.mx,_data_gy->lsm303dlhc_mag.my,_data_gy->lsm303dlhc_mag.mz);
	 sprintf((char*)gy_data,"%ld;%f;%f;%f;%f;%f;%f;%f;%f;%f|",HAL_GetTick(),(float)_data_gy->l3g4200d.gx*M_PI/180/ 131.0,(float)_data_gy->l3g4200d.gy*M_PI/180/ 131.0,(float)_data_gy->l3g4200d.gz*M_PI/180/ 131.0,(float)_data_gy->lsm303dlhc.ax*21.5625,(float)_data_gy->lsm303dlhc.ay*21.5625,(float)_data_gy->lsm303dlhc.az*21.5625,_data_gy->lsm303dlhc_mag.mx*0.1388,_data_gy->lsm303dlhc_mag.my*0.1388,_data_gy->lsm303dlhc_mag.mz*0.1388);
	 log_s((char*)gy_data);*/

}
