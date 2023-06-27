#ifndef INC_BMP_H_
#define INC_BMP_H_
#include "stm32f4xx_hal.h"
#include "string.h"
void debug_init(UART_HandleTypeDef *uart);
void log_s(const char *str);
void log_s_wnl(const char *str);
void log_s_int(const char *str, int32_t i);
void log_s_float(const char *str, double i);
void log_int(int32_t i);
void log_e(const char *str);
void log_p(int16_t *data_p);
void log_s_p_3(const char *str, int16_t *data_p0, int16_t *data_p1,
		int16_t *data_p2);
void log_s_p(const char *str, int16_t *data_p0);

#endif /* INC_BMP_H_ */
