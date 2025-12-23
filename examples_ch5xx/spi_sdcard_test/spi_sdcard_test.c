// Simple example to test SPI ST7735 display
// This example uses SPI0, wire the display as follows:
// PA13:	SCK
// PA14:	MOSI
// PA10:	RST
// PA11:	DC

// The display should be filled with a purple color
// and display a text string with a counter

#include "ch32fun.h"
#include "../spi_st7735_test/lib_st7735.h"
#include "mmc/mmcbbp.h"

// R32_SPI0_CONTROL
// PA13:	SCK
// PA14:	MOSI, TX0_

// R32_SPI1_CONTROL
// PA0: 	SCK1
// PA1: 	MOSI1

#define TARGETED_SPI SPI0


FATFS fatfs; /* File system object */
UINT br;
FRESULT rc;

FRESULT mod_sdCard_write(const char filename[], const char data[]) {
	printf("\nMounting volume.\n\r");
	rc = pf_mount(&fatfs);
	print("rc1=%u\n\r", rc);
	if (rc) return rc;

	printf("Opening file \"%s\"\n\r", filename);
	rc = pf_open(filename);
	print("rc2=%u\n\r", rc);
	if (rc) return rc;

	rc = pf_write(data, strlen(data), &br);
	print("rc3=%u\n\r", rc);
	if (rc) return rc;

	rc = pf_write(0, 0, &br);
	return rc;
}

FRESULT mod_sdCard_loadFile(const char filename[], uint32_t addr) {
	BYTE buff[64];

	printf("\nlseek to %u\n\r", addr);
	rc = pf_lseek(addr);
	printf("rc=%u\n\r", rc);
	
	// printf("\nMounting volume.\n\r");
	// rc = pf_mount(&fatfs);
	// printf("rc=%u\n\r", rc);
	// if (rc) return;

	// printf("Opening file \"%s\"\n\r", filename);
	// rc = pf_open(filename);
	// printf("rc=%u\n\r", rc);
	// if (rc) return;

	uint32_t total_bytes = 0;
	uint8_t cnt = 0;
	const char spinner[] = "/-\\|";

	for (;;) {
		rc = pf_read(buff, sizeof(buff), &br); /* Read a chunk of file */
		if (rc || !br) break; /* Error or end of file */

		printf("Read %u bytes\n\r", br);
		printf("string: %s\n\r", buff);

		total_bytes += br;
		addr += br;

		if(total_bytes % (16*1024) == 0){
			cnt++;
			printf("%d kb so far...  ", total_bytes/1024);
			putchar(spinner[cnt%4]);
			putchar('\r');
		}
	}

	printf("\n\rLoaded %d kilobytes.\n\r", total_bytes/1024);
	
	return rc;
}


uint8_t SPI_transfer_8(uint8_t data) {
	return SPI_send8(TARGETED_SPI, data);
}

int main() {
	SystemInit();
	Delay_Ms(100);
	funGpioInitAll();

	SPI_Device_t spi_device = {
		.SPIx = TARGETED_SPI,
		.miso_pin = PA15,
		.mosi_pin = PA14,
		.sck_pin = PA13,
		.rst_pin = PA12,
		.dc_pin = PA11,
		.cs_pin = PA10
		// .cs_pin = PB4
	};

	printf("~SPI ST7735 TEST~\n");
	// clock div = 16, mode: 1 = slave, 0 = master
	SPI_init(&spi_device, 16, 0);
	// ST7735_init(&spi_device, 160, 80);
	// ST7735_fill_all(ST_PURPLE);

	printf("IM HERE 111.\n\r");

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


