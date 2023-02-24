#include "bmp180.h"
#include "math.h"
#include "stdio.h"
#include "string.h"
I2C_HandleTypeDef *_bmp_i2c;
readings *_bmp_data;

const uint8_t BMP180_EEPROM_ADDR_MSB[11] = { 0xaa, 0xac, 0xae, 0xb0, 0xb2, 0xb4, 0xb6, 0xb8, 0xba, 0xbc, 0xbe };
const uint8_t BMP180_EEPROM_ADDR_LSB[11] = { 0xab, 0xad, 0xaf, 0xb1, 0xb3, 0xb5, 0xb7, 0xb9, 0xbb, 0xbd, 0xbf };

BMP180_SETTINGS _calib_data;
BMP180_REGS _regs;

void BMP180_init(I2C_HandleTypeDef *i2c,readings *data){
	_bmp_i2c=i2c;
	_bmp_data=data;
	log_s_wnl("BMP180 started---");
	if (BMP180_ReadReg(BMP180_GET_ID_REG)==BMP180_CHIP_ID){
		log_s("ok");
	}
	else{
		log_s("ERROR");
	}
}



void BMP180_WriteReg(uint8_t reg, uint8_t cmd) {
	uint8_t arr[2] = { reg, cmd };
	HAL_I2C_Master_Transmit(_bmp_i2c, BMP180_ADDR << 1, arr, 2, 1000);
}

uint8_t BMP180_ReadReg(uint8_t reg) {
	HAL_I2C_Master_Transmit(_bmp_i2c, BMP180_ADDR << 1, &reg, 1, 1000);
	uint8_t result;
	HAL_I2C_Master_Receive(_bmp_i2c, BMP180_ADDR << 1, &result, 1, 1000);
	return result;
}

int16_t _raw_t;
uint32_t _raw_p;

uint8_t _oss=0;
void BMP180_set_oss(uint8_t oss){
	_oss=oss;
}

void _oss_delay(){
	if (_oss==0){
		HAL_Delay (5);
	} else if (_oss==1){
		HAL_Delay (8);
	} else if (_oss==2){
		HAL_Delay (14);
	} else  if (_oss==3){
		HAL_Delay (26);
	}
}

void BMP180_get_calibration_coefficients()
{
	_calib_data.AC1 = (BMP180_ReadReg(BMP180_AC1_MSB) << 8) | BMP180_ReadReg(BMP180_AC1_LSB);
	_calib_data.AC2 = (BMP180_ReadReg(BMP180_AC2_MSB) << 8) | BMP180_ReadReg(BMP180_AC2_LSB);
	_calib_data.AC3 = (BMP180_ReadReg(BMP180_AC3_MSB) << 8) | BMP180_ReadReg(BMP180_AC3_LSB);
	_calib_data.AC4 = (BMP180_ReadReg(BMP180_AC4_MSB) << 8) | BMP180_ReadReg(BMP180_AC4_LSB);
	_calib_data.AC5 = (BMP180_ReadReg(BMP180_AC5_MSB) << 8) | BMP180_ReadReg(BMP180_AC5_LSB);
	_calib_data.AC6 = (BMP180_ReadReg(BMP180_AC6_MSB) << 8) | BMP180_ReadReg(BMP180_AC6_LSB);
	_calib_data.B1 =  (BMP180_ReadReg(BMP180_B1_MSB) << 8)  | BMP180_ReadReg(BMP180_B1_LSB);
	_calib_data.B2 =  (BMP180_ReadReg(BMP180_B2_MSB) << 8)  | BMP180_ReadReg(BMP180_B1_LSB);
	_calib_data.MB =  (BMP180_ReadReg(BMP180_MB_MSB) << 8)  | BMP180_ReadReg(BMP180_MB_LSB);
	_calib_data.MC =  (BMP180_ReadReg(BMP180_MC_MSB) << 8)  | BMP180_ReadReg(BMP180_MC_LSB);
	_calib_data.MD =  (BMP180_ReadReg(BMP180_MD_MSB) << 8)  | BMP180_ReadReg(BMP180_MD_LSB);
	char* test[200];
	sprintf((char*)test,"Calib coef:%d %d %d %d %d %d %d %d %d %d %d",_calib_data.AC1,_calib_data.AC2,_calib_data.AC3,_calib_data.AC4,_calib_data.AC5,_calib_data.AC6,_calib_data.B1,_calib_data.B1,_calib_data.MB,_calib_data.MC,_calib_data.MD);
	log_s(test);
}


int32_t BMP180_UT(){
	BMP180_WriteReg(BMP180_CONTROL_REG,BMP180_CMD_TEMP); //measure
	HAL_Delay (5);  //wait
	return (BMP180_ReadReg(BMP180_MSB) << 8) | BMP180_ReadReg(BMP180_LSB); //read
}

void BMP180_get_data()
{
	BMP180_WriteReg(BMP180_CONTROL_REG,0x2E); //measure
	HAL_Delay (5);  //wait
	_raw_t = (BMP180_ReadReg(BMP180_MSB) << 8) | BMP180_ReadReg(BMP180_LSB); //read

	BMP180_WriteReg(0xF4, 0x34+(_oss<<6));
	_oss_delay();
	_raw_p= ((BMP180_ReadReg(BMP180_MSB) << 16) | (BMP180_ReadReg(BMP180_LSB) << 8) | BMP180_ReadReg(BMP180_XLSB_REG)) >> (8 - _oss);
}

double BMP180_Press()
{
	int32_t x1 = (_raw_t - _calib_data.AC6) * _calib_data.AC5 / (1 << 15);
	int32_t x2 = (_calib_data.MC * (1 << 11)) / (x1 + _calib_data.MD);
	int32_t b5 = x1 + x2;
	int32_t b6 = b5 - 4000;
	x1 = (_calib_data.B2 * (b6 * b6 / (1 << 12))) / (1 << 11);
	x2 = _calib_data.AC2 * b6 / (1 << 11);
	int32_t x3 = x1 + x2;
	int32_t b3 = (((_calib_data.AC1 * 4 + x3) << _oss) + 2) / 4;
	x1 = _calib_data.AC3 * b6 / (1 << 13);
	x2 = (_calib_data.B1 * (b6 * b6 / (1 << 12))) / (1 << 16);
	x3 = ((x1 + x2) + 2) / 4;
	uint32_t b4 = _calib_data.AC4 * (uint32_t) (x3 + 32768) / (1 << 15);
	uint32_t b7 = ((uint32_t) _raw_p - b3) * (50000 >> _oss);
	int32_t p;
	if (b7 < 0x80000000)
		p = (b7 * 2) / b4;
	else
		p = (b7 / b4) * 2;
	x1 = (p / (1 << 8)) * (p / (1 << 8));
	x1 = (x1 * 3038) / (1 << 16);
	x2 = (-7357 * p) / (1 << 16);
	p = p + (x1 + x2 + 3791) / (1 << 4);
	return p;
}

int32_t BMP180_Temp()
{
	int32_t x1 = (_raw_t - _calib_data.AC6) * _calib_data.AC5 / (1 << 15);
	int32_t x2 = (_calib_data.MC * (1 << 11)) / (x1 + _calib_data.MD);
	int32_t b5 = x1 + x2;
	return (b5 + 8) / (1 << 4);
}


void BMP180_upd_data()
{
	_bmp_data->bmp180.pressure=BMP180_Press();
	_bmp_data->bmp180.temp=BMP180_Temp();
}

/*void print_data(){

}*/
