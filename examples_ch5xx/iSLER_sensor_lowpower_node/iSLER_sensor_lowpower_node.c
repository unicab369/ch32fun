// Send message to iSLER_sensors_gateway

// When the chip is sleeping, you won't be able to flash it,
// Hold the BOOT pin while powering up the chip and flash it before it enters user mode

//! WARNING: WHEN USE SHUTDOWN_MODE_ENABLED
//! PA15 NEEDS TO BE IN PULLED UP MODE OR YOU WONT BE ABLE TO REFLASH THE CHIP 
//! if you dont have access to the BOOT pin PB22
//# Ground PA15 to enter programming mode (in programming mode, PA8 led will be flashing)
//# after flashing done, unground PA15 to enter shutdown mode

#include "ch32fun.h"
#include <stdio.h>

#include "lib_ch5xx_sleep.h"
#include "lib_ch5xx_Mess.h"
#include "i2c_devices.h"
#include "../adc_basic/fun_adc_ch5xx.h"

#define SHUTDOWN_MODE_ENABLED
// #define I2C_SCAN_ENABLED
// #define TEST_MODE_ENABLED

#define SLEEPTIME_MS 10000

#define LED_PIN PA8

// #define POWER_CTRL_PIN PB23			// HIGH = switch to auxiliary power source
#define SENSOR_POWER_PIN PA4		// HIGH = Turn on power to sensors
#define SENSOR_MODE_PIN PA15		// LOW = Exit shutdown mode
// #define VOLTAGE_DIVIDER_CTRL PA4	// HIGH = enable voltage divider

#define I2C_SDA PB12
#define I2C_SCL PB13

remote_command_t sensor_cmd = {
	.command = 0xBB,
	.value1 = 0,
	.value2 = 0,
	.value3 = 0,
	.value4 = 0,
	.value5 = 0,
	.value6 = 0,
	.value7 = 0,
	.value8 = 0
};

void onHandle_pingFound(int address) {
	printf("i2C found: 0x%02X\n", address);
}

char str_output[16] = {0};

void collect_readings() {
	u16 bus_mV, shunt_mV, current_mA, power_mW;
	u16 temp, hum, lux;

	//# get INA219 reading
	ina219_read(INA219_ADDR, &bus_mV, &shunt_mV, &current_mA, &power_mW);
	sensor_cmd.value4 = bus_mV;
	sensor_cmd.value5 = current_mA;

	//# get BH1750 reading
	bh1750_read(BH1750_ADDR, &lux);
	sensor_cmd.value3 = lux;

	//# get SHT3x reading
	sht3x_read(SHT3X_ADDR, &temp, &hum);
	sensor_cmd.value1 = temp;
	sensor_cmd.value2 = hum;

	#ifdef I2C_DEBUG_ENABLED
		//# clear display
		ssd1306_draw_fill(0x00);

		sprintf(str_output, "%dF, %d%%, lux:%d", temp, hum, lux);
		menu_render_text_at(0, str_output);
		printf("\n\n%s", str_output);

		sprintf(str_output, "B:%d, %dmA, %dmW", bus_mV, current_mA, power_mW);
		menu_render_text_at(1, str_output);
		printf("\n%s", str_output);

		//# update display
		ssd1306_draw_all();
	#else
		#ifdef TEST_MODE_ENABLED
			printf("T:%dF, H:%d%%, Lux:%d, V:%d mV, I:%d mA\n",
				temp, hum, lux,
				bus_mV, current_mA
			);
		#endif
	#endif
}

#ifdef SHUTDOWN_MODE_ENABLED
	int main() {
		SystemInit();
		funGpioInitAll();

		ch5xx_allPinsPullUp();
		// funPinMode(VOLTAGE_DIVIDER_CTRL, GPIO_CFGLR_OUT_2Mhz_PP);
		// funDigitalWrite(VOLTAGE_DIVIDER_CTRL, 1); // enable voltage divider
		
		funPinMode(SENSOR_POWER_PIN, GPIO_CFGLR_OUT_2Mhz_PP);
		// funDigitalWrite(SENSOR_POWER_PIN, 0);
		// Delay_Ms(1000);
		funDigitalWrite(SENSOR_POWER_PIN, 1);
		

		//# Turn on the LED_PIN - for WeAct board PA8 is active LOW
		funPinMode(LED_PIN, GPIO_CFGLR_OUT_2Mhz_PP);
		funDigitalWrite(LED_PIN, 0);

		//# SENSOR_MODE_PIN LOW on RESET = exit shutdown mode
		while(!funDigitalRead(SENSOR_MODE_PIN)) {
			funDigitalWrite(LED_PIN, 0); Delay_Ms(100);
			funDigitalWrite(LED_PIN, 1); Delay_Ms(100);
		}

		//# get internal voltage reading
		adc_set_channel(ADC_VBAT_CHANNEL);
		adc_set_config(ADC_FREQ_DIV_10, ADC_PGA_GAIN_1_2, 0);
		int vInternal_mV = adc_to_mV(adc_get_singleReading(), ADC_PGA_GAIN_1_2);
		sensor_cmd.value6 = vInternal_mV;

		//# get ch0 reading PA5
		adc_set_channel(1);
		adc_buf_enable(1);
		adc_set_config(ADC_FREQ_DIV_10, ADC_PGA_GAIN_1_2, 0);

		// while(1) {
		// 	int solar_mV = adc_to_mV(adc_get_singleReading(), ADC_PGA_GAIN_1_2);
		// 	sensor_cmd.value7 = solar_mV;
		// 	printf("Solar: %d mV\r\n", solar_mV);
		// 	Delay_Ms(1000);
		// }

		//# switch to supercap power if internal voltage is sufficient
		// int vInternal_OK = vInternal_mV > 2500;
		// funPinMode(POWER_CTRL_PIN, GPIO_CFGLR_OUT_2Mhz_PP);
		// funDigitalWrite(POWER_CTRL_PIN, vInternal_OK);
		// sensor_cmd.value8 = vInternal_OK;
		// Delay_Ms(1);

		// int ldo_OK = solar_mV > 2500;
		// //# switch back to battery if LDO voltage is low
		// if (solar_mV < 2500) {
		// 	funDigitalWrite(POWER_CTRL_PIN, 0);
		// 	sensor_cmd.value8 = 0xDD;
		// }
		
		u8 err = i2c_init(100);
		#ifdef I2C_SCAN_ENABLED
			printf("\nI2C init: %d\r\n", err);
			i2c_scan(onHandle_pingFound);
		#endif

		#ifdef I2C_DEBUG_ENABLED
			printf("Battery: %d mV\r\n", vInternal_mV);
			printf("PA4: %d mV\r\n", ldo_mV);

			ssd1306_init();
		#endif

		prepare_sensors();

		#ifdef TEST_MODE_ENABLED
			while(1) {
				collect_readings();
				Delay_Ms(1000);
			}
		#else
			collect_readings();
		#endif

		//# Turn off sensor power
		funPinMode(SENSOR_POWER_PIN, GPIO_CFGLR_OUT_2Mhz_PP);
		funDigitalWrite(SENSOR_POWER_PIN, 0);

		//# Turn off the LED_PIN by doing pullup
		funPinMode(LED_PIN, GPIO_CFGLR_IN_PUPD);
		funPinMode(I2C_SCL, GPIO_CFGLR_IN_PUPD);
		funPinMode(I2C_SDA, GPIO_CFGLR_IN_PUPD);
		funPinMode(PA4, GPIO_CFGLR_IN_PUPD);

		//# prepare for sleep
		ch5xx_setClock(CLK_SOURCE_PLL_60MHz);
		DCDCEnable(); // Enable the internal DCDC
		LSIEnable(); // Disable LSE, enable LSI
		ch5xx_sleep_rtc_init();

		//# advertise
		RFCoreInit(LL_TX_POWER_0_DBM);
		MESS_advertise(&sensor_cmd);

		//# Disconnect SENSOR_MODE_PIN from GND to enter power_down mode.
		if (funDigitalRead(SENSOR_MODE_PIN)) {
			ch5xx_sleep_powerDown( MS_TO_RTC(SLEEPTIME_MS), (RB_PWR_RAM2K) );
		}
	}

#else
	int main() {
		SystemInit();
		funGpioInitAll();

		// WARNGING: Allow some delay time to allow reprogramming the chip uppon power up
		Delay_Ms(4000);

		// ch5xx_setClock(CLK_SOURCE_HSE_4MHz);

		ch5xx_sleep_rtc_init();
		LSIEnable(); // Disable LSE, enable LSI

		while(1) {
			funPinMode( LED_PIN, GPIO_CFGLR_OUT_2Mhz_PP );

			RFCoreInit(LL_TX_POWER_0_DBM); // RF wakes up in an odd state, we need to reinit after sleep
			DCDCEnable(); // DCDC gets disabled during sleep
			ch5xx_allPinsPullUp(); // this reduces sleep from ~70uA to 1uA

			// BLE advertisements are sent on channels 37, 38 and 39, over the 1M PHY
			for(int c = 0; c < sizeof(adv_channels); c++) {
				chMess_advertise(&sensor_cmd);
			}

			sensor_cmd.value1++;
			// Delay_Ms(1000);
			LowPower( MS_TO_RTC(SLEEPTIME_MS), (RB_PWR_RAM2K | RB_PWR_RAMX) ); // PWR_RAM can be optimized
		}
	}
#endif