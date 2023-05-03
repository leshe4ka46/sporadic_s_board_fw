#include "debug.h"
#include <stdio.h>
UART_HandleTypeDef *_dbg_uart;

void debug_init(UART_HandleTypeDef *uart){
	_dbg_uart=uart;
}



void log_s(const char *str){
	HAL_UART_Transmit(_dbg_uart,(uint8_t*)str,strlen(str),1000);
	HAL_UART_Transmit(_dbg_uart,(uint8_t*)"\r\n",2,1000);
}

void log_s_wnl(const char *str){
	HAL_UART_Transmit(_dbg_uart,(uint8_t*)str,strlen(str),1000);
}
char* char_log[50];
void log_s_int(const char *str,int32_t i){
	sprintf((char*)char_log,"%s %ld\r\n",str,i);
	log_s_wnl((const char*)char_log);
}
void log_p(int16_t *data_p){
	sprintf((char*)char_log,"%p\r\n",data_p);
	log_s_wnl((const char*)char_log);
}
void log_s_p_3(const char *str,int16_t *data_p0,int16_t *data_p1,int16_t *data_p2){
	sprintf((char*)char_log,"%s %p %p %p",str,data_p0,data_p1,data_p2);
	log_s((const char*)char_log);
}
void log_s_p(const char *str,int16_t *data_p0){
	sprintf((char*)char_log,"%s %p",str,data_p0);
	log_s((const char*)char_log);
}

void log_int(int32_t i){
	sprintf((char*)char_log,"%ld\r\n",i);
	log_s_wnl((const char*)char_log);
}

void log_e(const char *str){
	sprintf((char*)char_log,"ERROR --- %s\r\n",str);
	log_s_wnl((const char*)char_log);
}
