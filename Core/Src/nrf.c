//7370726467-sprdg

#include "nrf.h"
SPI_HandleTypeDef *_nrf_spi;
readings *_nrf_data;

#define CE(x) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, x?GPIO_PIN_SET:GPIO_PIN_RESET);
#define CSN(x) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, x?GPIO_PIN_SET:GPIO_PIN_RESET);

void sleep_micros(uint32_t micros)
{
  micros *= (SystemCoreClock / 1000000) / 9;
  while (micros--){};
}
void spi_write(uint8_t buf){
	HAL_SPI_Transmit(_nrf_spi,&buf,1,1000);
}

void nrf_write(uint8_t addr, uint8_t dt)
{
  addr |= W_MASK;
  CSN(0);
  HAL_SPI_Transmit(_nrf_spi,&addr,1,1000);
  HAL_SPI_Transmit(_nrf_spi,&dt,1,1000);
  CSN(1);
}
uint8_t nrf_read(uint8_t addr)
{
  uint8_t data=0, cmd;
  CSN(0);
  HAL_SPI_TransmitReceive(_nrf_spi,&addr,&data,1,1000);
  cmd=0xFF;
  HAL_SPI_TransmitReceive(_nrf_spi,&cmd,&data,1,1000);
  CSN(1);
  return data;
}
void nrf_set_addr(char* c) {
	CSN(0);
	uint8_t trn;
	trn=W_MASK|TX_ADDR;
	HAL_SPI_Transmit(_nrf_spi,&trn,1,1000);
	for(uint8_t i = 0; i < 5; i++) {
		HAL_SPI_Transmit(_nrf_spi,*c,1,1000);
		c++;
	}
	CSN(1);
}

void NRF_init(SPI_HandleTypeDef *spi,readings *data){
	_nrf_data=data;
	_nrf_spi=spi;
	nrf_write(0x00, 0b00001010);
	nrf_write(0x01, 0b00000000);
	nrf_write(0x02, 0b00000000); // enable data pipe 0
	nrf_write(0x03, 0b00000011); // 5 byte
	nrf_write(0x04, 0b11111111); //  4ms 15 t
	nrf_write(0x05, 0x05);       // channel
	//----------------76543210
	nrf_write(0x06, 0b00000110);
	nrf_write(0x07, 0b01110000);
	nrf_write(0x11, 0b00100000);
	nrf_write(0x12, 0b00100000);
	nrf_write(0x13, 0b00100000);
	nrf_write(0x14, 0b00100000);
	nrf_write(0x15, 0b00100000);
	nrf_write(0x16, 0b00100000);
	nrf_write(0x17, 0b00000000);
	nrf_write(0x1C, 0b00000000);// dynamic payload [5:0]
	nrf_write(0x1D, 0b00000011);
	nrf_set_addr((char*)"sprdg");
	log_s_int("nrf channel:",(uint32_t)nrf_read(0x05));
}

void nrf_flush_tx(void)
{
  uint8_t dt[1] = {FLUSH_TX};
  CSN(1);
  HAL_SPI_Transmit(_nrf_spi,dt,1,1000);
  sleep_micros(1);
  CSN(0);
}

#define PRIM_RX 0x00 //RX/TX control (1: PRX, 0: PTX)
#define PWR_UP 0x01 //1: POWER UP, 0:POWER DOWN
#define RX_DR 0x40 //Data Ready RX FIFO interrupt
#define TX_DS 0x20 //Data Sent TX FIFO interrupt
#define MAX_RT 0x10 //Maximum number of TX retransmits interrupt
void nrf_send(uint8_t* buf) {
	nrf_flush_tx();
	/*uint8_t regval = nrf_read(0x00);
	regval |= (1<<PWR_UP);
	regval &= ~(1<<PRIM_RX);*/
	nrf_write(0x00,10);
	sleep_micros(150);
	NRF24_Transmit(0xA0,&buf,32);
	CE(1);
	sleep_micros(50); //minimum 10us high pulse (Page 21)
	CE(0);
}
void NRF24_Transmit(uint8_t addr,uint8_t *pBuf,uint8_t bytes)
{
	CE(0);
	CSN(1);
	HAL_SPI_Transmit(_nrf_spi,&addr,1,1000);//�������� ����� � ����
	sleep_micros(1);
	HAL_SPI_Transmit(_nrf_spi,pBuf,bytes,1000);//�������� ������ � �����
	CSN(1);
	CE(1);
}
#define NRF24L01P_CMD_W_TX_PAYLOAD                0b10100000
uint8_t nrf24l01p_write_tx_fifo(uint8_t* tx_payload)
{
    uint8_t command = NRF24L01P_CMD_W_TX_PAYLOAD;
    uint8_t status;

    CSN(1);
    HAL_SPI_TransmitReceive(_nrf_spi, &command, &status, 1, 2000);
    HAL_SPI_Transmit(_nrf_spi, tx_payload, 32, 2000);
    CSN(0);

    return status;
}




