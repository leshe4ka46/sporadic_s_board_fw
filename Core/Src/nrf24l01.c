#include "nrf24l01.h"
SPI_HandleTypeDef *_nrf_spi;

uint8_t dt[1] = { 0 };
uint8_t regval = 0;
#define CSN_Pin GPIO_PIN_4
#define CSN_GPIO_Port GPIOA
#define CE_Pin GPIO_PIN_4
#define CE_GPIO_Port GPIOC
#define IRQ_Pin GPIO_PIN_10
#define IRQ_GPIO_Port GPIOB
#define CE(x) HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, (x)?GPIO_PIN_SET:GPIO_PIN_RESET);
#define CSN(x) HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, (x)?GPIO_PIN_SET:GPIO_PIN_RESET);
#define IRQ HAL_GPIO_ReadPin(IRQ_GPIO_Port, IRQ_Pin)

uint8_t TX_ADDRESS[5] = { 0x73, 0x70, 0x72, 0x64, 0x67 };
uint8_t transmit_len = 1;
void nrf_init(SPI_HandleTypeDef *spi) {
	_nrf_spi = spi;
	nrf_write_reg(NRF_REG_CONFIG, 0b01011010);
	nrf_write_reg(NRF_REG_EN_AA, 0b00000000);
	nrf_write_reg(NRF_REG_EN_RXADDR, 0b00000011);
	nrf_write_reg(NRF_REG_SETUP_AW, 0b00000011); // 5 byte
	nrf_write_reg(NRF_REG_SETUP_RETR, 0b00000000); //  4ms 15 t
	nrf_toggle_activate();
	nrf_write_reg(NRF_REG_RF_CH, 0x05); // frequency
	//--------------------------------76543210
	nrf_write_reg(NRF_REG_RF_SETUP, 0b00100110);
	nrf_write_reg(NRF_REG_STATUS, 0b01110000);
	nrf_write_reg(NRF_REG_RX_PW_P0, 0b00100000);
	nrf_write_reg(NRF_REG_RX_PW_P1, 0b00000000);
	nrf_write_reg(NRF_REG_RX_PW_P2, 0b00000000);
	nrf_write_reg(NRF_REG_RX_PW_P3, 0b00000000);
	nrf_write_reg(NRF_REG_RX_PW_P4, 0b00000000);
	nrf_write_reg(NRF_REG_RX_PW_P5, 0b00000000);
	nrf_write_reg(NRF_REG_FIFO_STATUS, 0b00000000);
	nrf_write_reg(NRF_REG_DYNPD, 0b00000000);
	nrf_write_reg(NRF_REG_FEATURE, 0b00000000);

	nrf_write_buf(NRF_REG_TX_ADDR, TX_ADDRESS, 5);
	nrf_write_buf(NRF_REG_RX_ADDR_P0, TX_ADDRESS, 5);
	nrf_rx_mode();
	/*char *str1[50];
	 sprintf(str1,"CONFIG: 0x%x",nrf_read_reg(0x00));
	 log_s(str1);
	 sprintf(str1,"EN_AA: 0x%x",nrf_read_reg(0x01));
	 log_s(str1);
	 sprintf(str1,"EN_RXADDR: 0x%x",nrf_read_reg(0x02));
	 log_s(str1);
	 sprintf(str1,"SETUP_AW: 0x%x",nrf_read_reg(0x03));
	 log_s(str1);
	 sprintf(str1,"CHANNEL: 0x%x",nrf_read_reg(0x05));
	 log_s(str1);
	 sprintf(str1,"STATUS: 0x%x",nrf_read_reg(0x07));
	 log_s(str1);
	 sprintf(str1,"RF_SETUP: 0x%x",nrf_read_reg(0x06));
	 log_s(str1);
	 uint8_t buf1[5];
	 NRF24_Read_Buf(0x10,buf1,5);
	 sprintf(str1,"TX_ADDR: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x" ,buf1[0],buf1[1],buf1[2],buf1[3],buf1[4]);
	 log_s(str1);*/

}
void nrf_toggle_activate(void) {
	dt[0] = NRF_CMD_ACTIVATE;
	CSN(0);
	HAL_SPI_Transmit(_nrf_spi, dt, 1, 1000);
	DelayMicro(1);
	dt[0] = 0x73;
	HAL_SPI_Transmit(_nrf_spi, dt, 1, 1000);
	CSN(1);
}
void nrf_rx_mode(void) {
	regval = 0;
	regval = nrf_read_reg(NRF_REG_CONFIG);
	regval |= (1 << NRF_BIT_PWR_UP) | (1 << NRF_BIT_PRIM_RX);
	nrf_write_reg(NRF_REG_CONFIG, regval);
	CE(1);
	DelayMicro(150);
	nrf_flush_rx();
	nrf_flush_tx();
}
void nrf_flush_tx() {
	dt[0] = NRF_CMD_FLUSH_TX;
	CSN(0);
	HAL_SPI_Transmit(_nrf_spi, &dt, 1, 1000);
	DelayMicro(1);
	CSN(1);
}
void nrf_flush_rx() {
	dt[0] = NRF_CMD_FLUSH_RX;
	CSN(0);
	HAL_SPI_Transmit(_nrf_spi, &dt, 1, 1000);
	DelayMicro(1);
	CSN(1);
}
void nrf_tx_mode() {
	nrf_write_buf(NRF_REG_TX_ADDR, TX_ADDRESS, 5);
	CE(0);
	nrf_flush_rx();
	nrf_flush_tx();
	regval = nrf_read_reg(NRF_REG_CONFIG);
	regval |= (1 << NRF_BIT_PWR_UP);
	regval &= ~(1 << NRF_BIT_PRIM_RX);
	nrf_write_reg(0x00, regval);
}
void nrf_send(uint8_t *pBuf) {
	uint8_t status = 0x00, regval = 0x00;
	nrf_tx_mode();
	nrf_write_reg(NRF_REG_STATUS, 0b00110000);
	DelayMicro(150);
	nrf_write_buf(NRF_CMD_W_TX_PAYLOAD, pBuf, 32);
	CE(1);
	DelayMicro(15);
	while((GPIO_PinState)IRQ == GPIO_PIN_SET) {}
	CE(0);
	status = nrf_read_reg(NRF_REG_STATUS);
	if (status & NRF_BIT_CONFIG_TX_DS) //tx_ds == 0x20
	{
		nrf_write_reg(NRF_REG_STATUS, 0x20);
	} else if (status & NRF_BIT_CONFIG_MAX_RT) {
		nrf_write_reg(NRF_REG_STATUS, 0x10);
		nrf_flush_tx();
	}
	nrf_rx_mode();
}
uint8_t _read_data = 0;
uint8_t nrf_read_reg(uint8_t addr) {
	_read_data = 0;
	uint8_t cmd = 0xFF;
	CSN(0);
	HAL_SPI_Transmit(_nrf_spi, &addr, 1, 1000);
	HAL_SPI_Transmit(_nrf_spi, &cmd, 1, 1000);
	HAL_SPI_Receive(_nrf_spi, &_read_data, 1, 1000);
	CSN(1);
	return _read_data;

}
void nrf_write_reg(uint8_t addr, uint8_t dt) {

	CSN(0);
	addr |= NRF_CMD_W_REGISTER;
	HAL_SPI_Transmit(_nrf_spi, &addr, 1, 1000);
	HAL_SPI_Transmit(_nrf_spi, &dt, 1, 1000);
	CSN(1);

}

void nrf_read_buf(uint8_t addr, uint8_t *pBuf, uint8_t bytes) {
	CSN(0);
	HAL_SPI_Transmit(_nrf_spi, &addr, 1, 1000);
	HAL_SPI_Receive(_nrf_spi, pBuf, bytes, 1000);
	CSN(1);
}

void nrf_write_buf(uint8_t addr, uint8_t *pBuf, uint8_t bytes) {
	addr |= NRF_CMD_W_REGISTER;
	CSN(0);
	HAL_SPI_Transmit(_nrf_spi, &addr, 1, 1000);
	HAL_SPI_Transmit(_nrf_spi, pBuf, bytes, 1000);
	CSN(1);
}
