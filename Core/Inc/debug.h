#ifndef INC_BMP_H_
#define INC_BMP_H_
#include "stm32f4xx_hal.h"
#include "string.h"
void debug_init(UART_HandleTypeDef *uart);
void log_s(const char *str);
void log_s_wnl(const char *str);
void log_s_int(const char *str,int32_t i);
void log_int(int32_t i);
void log_e(const char *str);
void log_p(float *data_p);
#endif /* INC_BMP_H_ */
