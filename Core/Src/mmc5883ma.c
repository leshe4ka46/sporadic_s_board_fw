#include <mmc5883ma.h>
#include "math.h"
#include "stdio.h"
#include "string.h"
I2C_HandleTypeDef *_mag_i2c;
readings *_mag_data;

typedef struct{
	int16_t mx;
	int16_t my;
	int16_t mz;
} raw_mag;
raw_mag raw_data;

void mag_init(I2C_HandleTypeDef *i2c,readings *data){
	_mag_i2c=i2c;
	_mag_data=data;
	log_s_wnl("HMC5883L started---");
	if(mag_read_data(MAG_REG_PRODUCT_ID)==0xC){
		log_s("ok");
	}
	else{
		log_s("ERROR");
		return;
	}
	mag_write_data(0x8,0x8);
	mag_write_data(0x9,mag_read_data(0x9)&0b11100000);
	log_s_int("CTRL1", mag_read_data(0x9));
	log_s_int("CTRL2", mag_read_data(0xA));


}
uint32_t micros;
void mag_get_data(){
	mag_write_data(MAG_REG_CONFIG,1);// Take magnetic field measurement
	while((mag_read_data(MAG_REG_STATUS)&1)==0){
		micros=10;
		micros *= (SystemCoreClock / 1000000) / 9;
		while (micros--){};
	}
	uint8_t buffer[6];
	HAL_I2C_Mem_Read(_mag_i2c, MAG_ADDR<<1, 0x00, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&buffer, 6, HAL_MAX_DELAY);

	_mag_data->mmc5883ma.mx = ((buffer[0] << 8) | buffer[1])*0.025;
	_mag_data->mmc5883ma.my = ((buffer[2] << 8) | buffer[3])*0.025;
	_mag_data->mmc5883ma.mz = ((buffer[4] << 8) | buffer[5])*0.025;
	_mag_data->mmc5883ma.mx_cal=62.088658*(_mag_data->mmc5883ma.mx+826.572722)+1.065439*(_mag_data->mmc5883ma.my+811.516393)+2.134073*(_mag_data->mmc5883ma.mz+862.426848);
	_mag_data->mmc5883ma.my_cal=1.065439*(_mag_data->mmc5883ma.mx+826.572722)+65.162289*(_mag_data->mmc5883ma.my+811.516393)+0.185725*(_mag_data->mmc5883ma.mz+862.426848);
	_mag_data->mmc5883ma.mz_cal=2.134073*(_mag_data->mmc5883ma.mx+826.572722)+0.185725*(_mag_data->mmc5883ma.my+811.516393)+69.919138*(_mag_data->mmc5883ma.mz+862.426848);



	//magx_cal = 1.06*(magx + -7.49) + -0.01*(magy + -23.59) + 0.07*(magz + -108.24)
	//magy_cal = -0.01*(magx + -7.49) + 1.11*(magy + -23.59) + 0.09*(magz + -108.24)
	//magz_cal = 0.07*(magx + -7.49) + 0.09*(magy + -23.59) + 1.00*(magz + -108.24)

}

uint8_t mag_read_data(uint8_t reg){
	HAL_I2C_Master_Transmit(_mag_i2c, MAG_ADDR<<1, &reg, 1, 1000);
	uint8_t result;
	HAL_I2C_Master_Receive(_mag_i2c, MAG_ADDR<<1, &result, 1, 1000);
	return result;
}

void mag_write_data(uint8_t reg, uint8_t cmd) {
	uint8_t arr[2] = { reg, cmd };
	HAL_I2C_Master_Transmit(_mag_i2c, MAG_ADDR<<1, arr, 2, 1000);
}

void mag_calibrate(){

}
