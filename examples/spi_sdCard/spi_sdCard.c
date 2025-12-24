#include "ch32fun.h"

#include "mod_sdCard.h"

// #include "../../examples_ch5xx/spi_st7735_test/lib_st7735.h"
// #include "../../examples_ch5xx/spi_sdcard_test/mmc/mmcbbp.h"


#define TARGETED_SPI SPI1



int main() {
	SystemInit();
	Delay_Ms(100);
	funGpioInitAll();

	// SPI_Device_t spi_device = {
	// 	.SPIx = TARGETED_SPI,
	// 	.mosi_pin = PC6,
	// 	.sck_pin = PC5,
	// 	.rst_pin = PD4, 
	// 	.dc_pin = PD2, 
	// 	.cs_pin = -1
	// };

	printf("~SPI SDCARD TEST~\n");
	// clock div = 16, mode: 1 = slave, 0 = master
	SPI_init2();

	char str[25] = {0};
	int counter = 0;

	FRESULT rc = mod_sdCard_write("testfile.txt", "hello world 1111!\n\r");

	if (rc == 0) {
		Delay_Ms(200);
		rc = mod_sdCard_loadFile("testfile.txt",0);
		printf("read result: %u\n\r", rc);
	} else {
		printf("write error: %u\n\r", rc);
	}

	while(1) {
		printf("\nIM HERE.");
		// sprintf(str, "Hello Bee %d", counter++);
		// tft_print(&default_font, str, 0, 0, ST_WHITE, ST_PURPLE);
		Delay_Ms(1000);
	}
}


