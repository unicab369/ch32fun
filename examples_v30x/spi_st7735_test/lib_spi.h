#include "ch32fun.h"

typedef struct {
	s16 mosi_pin;
	s16 miso_pin;
	s16 sck_pin;
	s16 cs_pin;
	s16 dc_pin;
	s16 rst_pin;
} SPI_Pins_t;

static void SPI_init(SPI_Pins_t *pins) {
	funPinMode(pins->mosi_pin, GPIO_CFGLR_OUT_50Mhz_AF_PP);
	funPinMode(pins->sck_pin, GPIO_CFGLR_OUT_50Mhz_AF_PP);

	if (pins->miso_pin >= 0) {
		funPinMode(pins->miso_pin, GPIO_CNF_IN_FLOATING);
	}

	if (pins->cs_pin >= 0) {
		funPinMode(pins->cs_pin, GPIO_CFGLR_OUT_10Mhz_PP);
	}

	if (pins->dc_pin >= 0) {
		funPinMode(pins->dc_pin, GPIO_CFGLR_OUT_10Mhz_PP);
		funDigitalWrite(pins->dc_pin, 0);
	}
	
	if (pins->rst_pin >= 0) {
		funPinMode(pins->rst_pin, GPIO_CFGLR_OUT_10Mhz_PP);
		funDigitalWrite(pins->rst_pin, 0);
		Delay_Ms(100);
		funDigitalWrite(pins->rst_pin, 1);
		Delay_Ms(100);	
	}

	// Enable GPIO Port C and SPI peripheral
	RCC->APB1PCENR |= RCC_APB1Periph_SPI2;

	// reset control register
	SPI2->CTLR1 = 0;

	// Configure SPI
	SPI2->CTLR1 |= SPI_CPHA_1Edge | SPI_CPOL_Low | SPI_Mode_Master |
					SPI_BaudRatePrescaler_4 |
					SPI_NSS_Soft |
					SPI_DataSize_16b |
					3<<3; // Divisior = 16 (48/16 = 3MHz)

	SPI2->CTLR1 |= SPI_Direction_2Lines_FullDuplex;
	// SPI2->CTLR1 |= SPI_Direction_1Line_Tx;

	// Enable SPI Port
	SPI2->CTLR1 |= CTLR1_SPE_Set;
}


static void SPI_DMA_init(DMA_Channel_TypeDef* DMA_Channel, u32 *DATA_BUF) {
	// Enable DMA peripheral
	RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;

	// Enable Tx DMA
	SPI2->CTLR2 |= SPI_I2S_DMAReq_Tx;

	// Configure DMA
	DMA_Channel->PADDR = (uint32_t)&SPI2->DATAR;
	// DMA1_Channel3->MADDR = *DATA_BUF;
	DMA_Channel->CFGR = DMA_M2M_Disable |
						DMA_Priority_VeryHigh |
						DMA_MemoryDataSize_Byte |
						DMA_PeripheralDataSize_Byte |
						DMA_MemoryInc_Enable |
						DMA_PeripheralInc_Disable |
						DMA_Mode_Circular |
						DMA_DIR_PeripheralDST;
}

static void SPI_send_DMA(const uint8_t* buffer, uint16_t len) {
	FN_SPI_DC_HIGH();
	
	DMA1_Channel3->CNTR  = len;
	DMA1_Channel3->MADDR = (uint32_t)buffer;

	// Start DMA transfer
	DMA1_Channel3->CFGR |= DMA_CFGR1_EN;  

	DMA1->INTFCR = DMA1_FLAG_TC3;
	while (!(DMA1->INTFR & DMA1_FLAG_TC3));

	// Stop DMA transfer
	DMA1_Channel3->CFGR &= ~DMA_CFGR1_EN;
}

//# write read raw
static inline uint8_t SPI_read_8() {
	return SPI2->DATAR;
}
static inline uint16_t SPI_read_16() {
	return SPI2->DATAR;
}
static inline void SPI_write_8(uint8_t data) {
	SPI2->DATAR = data;
}
static inline void SPI_write_16(uint16_t data) {
	SPI2->DATAR = data;
}

static inline void SPI_wait_TX_complete() {
	while(!(SPI2->STATR & SPI_STATR_TXE)) {}
}

void SPI_end() {
	SPI2->CTLR1 &= ~(SPI_CTLR1_SPE);
}

uint8_t SPI_transfer_8(uint8_t data) {
	SPI_write_8(data);
	while (!(SPI2->STATR & SPI_STATR_TXE));
	asm volatile("nop");
	while (!(SPI2->STATR & SPI_STATR_RXNE));
	return SPI_read_8();
}

uint16_t SPI_transfer_16(uint16_t data) {
	SPI_write_16(data);
	while (!(SPI2->STATR & SPI_STATR_TXE));
	asm volatile("nop");
	while (!(SPI2->STATR & SPI_STATR_RXNE));
	return SPI_read_16();
}
