// Listen to messages from iSLER_sensor_lowpower_node

#include "ch32fun.h"
#include <stdio.h>
#include "../iSLER_sensor_lowpower_node/lib_ch5xx_Mess.h"
#include "../iSLER_sensor_lowpower_node/i2c_devices.h"

#define LED_PIN PA8

#define I2C_SDA PB12
#define I2C_SCL PB13

#define ACCESS_ADDRESS 0x8E89BED6 // the "BED6" address for BLE advertisements

void blink(int led_pin, int n) {
	for(int i = n-1; i >= 0; i--) {
		funDigitalWrite(led_pin, 0); // Turn on LED
		Delay_Ms(33);
		funDigitalWrite(led_pin, 1); // Turn off LED
		if(i) Delay_Ms(33);
	}
}

void onHandle_pingFound(int address) {
	printf("i2C found: 0x%02X\n", address);
}


int main() {
    SystemInit();
	funGpioInitAll();

	funPinMode(LED_PIN, GPIO_CFGLR_OUT_10Mhz_PP);
	funPinMode(I2C_SCL, GPIO_CFGLR_IN_PUPD);
	funPinMode(I2C_SDA, GPIO_CFGLR_IN_PUPD);

	funPinMode(PA8, GPIO_CFGLR_OUT_10Mhz_PP);
	funPinMode(PA9, GPIO_CFGLR_OUT_10Mhz_PP);

	printf("~ iSLER sensors gateway ~\n");
	RFCoreInit(LL_TX_POWER_0_DBM);

	//# setup I2C
	u8 err = i2c_init(100);
	ssd1306_init();

	#ifdef I2C_SCAN_ENABLED
		printf("\nI2C init: %d\r\n", err);
		i2c_scan(onHandle_pingFound);
	#endif

	//# clear display
	ssd1306_draw_fill(0x00);
	menu_render_text_at(0, "iSLER gateway");

	//# update display
	ssd1306_draw_all();

	u32 time_ref = 0;
	u32 counter = 0;

	char str_output[16] = {0};
	remote_command_t cmd_buffer = {0};
	int received_counter = 0;

	while (1) {		
		// we stepped over !rx_ready so we got a frame
		remote_command_t *cmd = chMess_rx_handler();
	// 	if (!cmd) return;
	// 	cmd_buffer = *cmd;

	// 	blink(LED_PIN, 1);
	// 	printf("\n\nReceiv Command: %02X", cmd->command);
	// 	printf("\nValue1: %u", cmd->value1);
	// 	printf("\nValue2: %u", cmd->value2);
	// 	printf("\nValue3: %u", cmd->value3);
	// 	received_counter++;

	// 	if (TimeElapsed32(SysTick->CNT, time_ref) > DELAY_SEC_COUNT(1)) {
	// 		time_ref = SysTick->CNT;

	// 		//# clear display
	// 		ssd1306_draw_fill(0x00);

	// 		sprintf(str_output, "Cmd: 0x%02X", cmd_buffer.command);
	// 		menu_render_text_at(0, str_output);

	// 		sprintf(str_output, "%dF, %d%%, lux:%d", 
	// 				cmd_buffer.value1, cmd_buffer.value2, cmd_buffer.value3);
	// 		menu_render_text_at(1, str_output);

	// 		sprintf(str_output, "B:%d, %dmA", cmd_buffer.value4, cmd_buffer.value5);
	// 		menu_render_text_at(2, str_output);

	// 		sprintf(str_output, "B:%d, %dmA", cmd_buffer.value6, cmd_buffer.value7);
	// 		menu_render_text_at(3, str_output);

	// 		sprintf(str_output, "%d /%d", received_counter, counter++);
	// 		menu_render_text_at(4, str_output);

	// 		//# update display
	// 		ssd1306_draw_all();
	// 	}
	}
}