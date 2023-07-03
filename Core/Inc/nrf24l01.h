#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_
#include "stm32f4xx_hal.h"
#include "debug.h"

__STATIC_INLINE void DelayMicro(__IO uint32_t micros) {
	micros *= (SystemCoreClock / 1000000) / 9;
	/* Wait till done */
	while (micros--)
		;
}
__STATIC_INLINE void DelayMilli(__IO uint32_t millis) {
	millis *= (SystemCoreClock / 1000) / 9;
	/* Wait till done */
	while (millis--)
		;
}

#define NRF_REG_CONFIG 0x00
#define NRF_REG_EN_AA 0x01
#define NRF_REG_EN_RXADDR 0x02
#define NRF_REG_SETUP_AW 0x03
#define NRF_REG_SETUP_RETR 0x04
#define NRF_REG_RF_CH 0x05
#define NRF_REG_RF_SETUP 0x06
#define NRF_REG_STATUS 0x07
#define NRF_REG_OBSERVE_TX 0x08
#define NRF_REG_RX_ADDR_P0 0x0A
#define NRF_REG_RX_ADDR_P1 0x0B
#define NRF_REG_TX_ADDR 0x10
#define NRF_REG_RX_PW_P0 0x11
#define NRF_REG_RX_PW_P1 0x12
#define NRF_REG_RX_PW_P2 0x13
#define NRF_REG_RX_PW_P3 0x14
#define NRF_REG_RX_PW_P4 0x15
#define NRF_REG_RX_PW_P5 0x16
#define NRF_REG_FIFO_STATUS 0x17
#define NRF_REG_DYNPD 0x1C
#define NRF_REG_FEATURE 0x1D

#define NRF_CMD_W_REGISTER 0x20
#define NRF_CMD_FLUSH_TX 0xE1
#define NRF_CMD_FLUSH_RX 0xE2
#define NRF_CMD_ACTIVATE 0x50
#define NRF_CMD_W_TX_PAYLOAD 0xA0

#define NRF_BIT_PRIM_RX 0x00
#define NRF_BIT_PWR_UP 0x01
#define NRF_BIT_CONFIG_TX_DS 0x20
#define NRF_BIT_CONFIG_MAX_RT 0x10

uint8_t nrf_read_reg(uint8_t addr);
void nrf_write_reg(uint8_t addr, uint8_t dt);
void nrf_read_buf(uint8_t addr, uint8_t *pBuf, uint8_t bytes);
void nrf_write_buf(uint8_t addr, uint8_t *pBuf, uint8_t bytes);
void nrf_flush_tx();
void nrf_flush_rx();
void nrf_send(uint8_t *pBuf);
void nrf_init(SPI_HandleTypeDef *spi);

#endif /* INC_NRF24L01_H_ */
