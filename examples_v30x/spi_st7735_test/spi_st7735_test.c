// Simple example to show PWM output on TIM2 Channel 4 (PA3)
// Connect an LED to PA3 to see the PWM signal output

#include "ch32fun.h"
#include <stdio.h>
#include "lib_st7735.h"

// R32_SPI0_CONTROL
// PA13:	SCK
// PA14:	MOSI

// R32_SPI1_CONTROL
// PA0: 	SCK1
// PA1: 	MOSI1

int main() {
	SystemInit();
	Delay_Ms(100);
	funGpioInitAll();

	SPI_Pins_t spi_pins = {
		.mosi_pin = PB15,
		.miso_pin = PB14,
		.sck_pin = PB13,
		.cs_pin = -1,
		.dc_pin = PA11,
		.rst_pin = PA9
	};

	printf("~SPI ST7735 TEST 2~\n");
	fun_st7335_init(160, 80, &spi_pins);
	tft_fill_rect(0, 0, 160, 80, ST_PURPLE);

	char str[25] = {0};
	int counter = 0;
	

	while(1) {
		// SPI_cmd_8(ST7735_COLMODE);
		// SPI_data_8(0x05); // 0x03: 12-bit, 0x05: 16-bit, 0x06: 18-bit, 0x07: Not used

		// fun_st7335_init(160, 80, &spi_pins);
		tft_fill_rect(0, 0, 160, 80, ST_PURPLE);
		sprintf(str, "Hello Bee %d", counter++);
		tft_print(str, 0, 0, ST_WHITE, ST_PURPLE);
		Delay_Ms(1000);
	}

	// while(1) {
	// 	funDigitalWrite(SPI_PINS->dc_pin, 1);
	// 	Delay_Ms(1000);
	// 	funDigitalWrite(SPI_PINS->dc_pin, 0);
	// 	Delay_Ms(1000);
	// 	SPI1->DATAR = ST7735_COLMODE;
	// 	Delay_Ms(1000);
	// }
}
