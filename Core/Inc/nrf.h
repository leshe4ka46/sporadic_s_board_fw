#ifndef INC_NRF_H_
#define INC_NRF_H_
#include "stm32f4xx_hal.h"
#include "debug.h"
#include "structs.h"

/*typedef enum DataRate {
	DR_250KBPS=0, DR_1MBPS, DR_2MBPS
} DataRate;

typedef enum DataPower {
	PA_MIN=0, PA_LOW, PA_HIGH, PA_MAX
} DataPower;*/

#define TX_ADDR 0x10
#define FLUSH_TX 0xE1

#define W_MASK 0x20
void NRF_init(SPI_HandleTypeDef *spi,readings *data);


#endif /* INC_NRF_H_ */
