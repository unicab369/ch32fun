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
// #define TEST_MODE_ENABLED
// #define I2C_SCAN_ENABLED

#define SLEEPTIME_MS 10000
#define SOLAR_SWITCH_THRESHOLD_mV 2600

#define LED_PIN PA8

#define SLEEP_MODE_PIN 			PA15		// LOW = Exit shutdown mode
#define SENSOR_POWER_PIN 		PA4			// HIGH = Turn on power to sensors
#define POWER_CTRL_PIN 			PA5			// HIGH = switch to Solar power source

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
u16 bus_mV, shunt_mV, current_mA, power_mW;

void collect_readings() {
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
			printf("T:%dF, H:%d%%, Lux:%d, V:%d mV, I:%d mA, W:%d mW\n",
				temp, hum, lux,
				bus_mV, current_mA, power_mW
			);
		#endif
	#endif
}

#ifdef SHUTDOWN_MODE_ENABLED
	int main() {
		SystemInit();
		funGpioInitAll();
		ch5xx_allPinsPullUp();

		//# Sensor Power Pin LOW = turn ON sensors (PFet)
		funPinMode(SENSOR_POWER_PIN, GPIO_CFGLR_OUT_2Mhz_PP);
		funDigitalWrite(SENSOR_POWER_PIN, 0);

		//# Power Control Pin LOW = default to battery power
		funPinMode(POWER_CTRL_PIN, GPIO_CFGLR_OUT_2Mhz_PP);
		funDigitalWrite(POWER_CTRL_PIN, 0);

		//# Turn on the LED_PIN - for WeAct board PA8 is active LOW
		funPinMode(LED_PIN, GPIO_CFGLR_OUT_2Mhz_PP);
		funDigitalWrite(LED_PIN, 0);

		//# Sleep Mode Pin HIGH = enter shutdown mode
		funPinMode(SLEEP_MODE_PIN, GPIO_CFGLR_IN_PUPD);
		funDigitalWrite(SLEEP_MODE_PIN, 1);

		//# SLEEP_MODE_PIN LOW on RESET = exit shutdown mode
		while(!funDigitalRead(SLEEP_MODE_PIN)) {
			funDigitalWrite(LED_PIN, 0); Delay_Ms(100);
			funDigitalWrite(LED_PIN, 1); Delay_Ms(100);
		}

		//# get internal voltage reading
		adc_set_channel(ADC_VBAT_CHANNEL);
		adc_set_config(ADC_FREQ_DIV_10, ADC_PGA_GAIN_1_2, 0);
		int vInternal_mV = adc_to_mV(adc_get_singleReading(), ADC_PGA_GAIN_1_2);
		sensor_cmd.value6 = vInternal_mV;

		//# setup I2C
		u8 err = i2c_init(100);
		#ifdef I2C_SCAN_ENABLED
			printf("\nI2C init: %d\r\n", err);
			i2c_scan(onHandle_pingFound);
		#endif

		//# get sensors readings
		prepare_sensors();
		// Delay_Us(100);		// wait for sensors to stabilize

		#ifdef TEST_MODE_ENABLED
			printf("\nInternal Voltage: %d mV\r\n", vInternal_mV);
			ssd1306_init();

			while(1) {
				collect_readings();
				Delay_Ms(1000);
			}
		#else
			collect_readings();
		#endif

		if (bus_mV > SOLAR_SWITCH_THRESHOLD_mV && vInternal_mV > SOLAR_SWITCH_THRESHOLD_mV) {
			// switch to solar power
			funDigitalWrite(POWER_CTRL_PIN, 1);
			// printf("*** Switched to Solar power source\r\n");
		} else {
			// switch to battery power
			funDigitalWrite(POWER_CTRL_PIN, 0);
			// printf("Battery power source\r\n");
		}

		//# Turn off sensor power
		funDigitalWrite(SENSOR_POWER_PIN, 1);

		//# Turn off the LED_PIN
		funDigitalWrite(LED_PIN, 1);

		//# Set I2C to reduce power consumption
		funPinMode(I2C_SCL, GPIO_CFGLR_OUT_2Mhz_PP);
		funPinMode(I2C_SDA, GPIO_CFGLR_OUT_2Mhz_PP);
		funDigitalWrite(I2C_SCL, 0);
		funDigitalWrite(I2C_SDA, 0);

		//# prepare for sleep
		ch5xx_setClock(CLK_SOURCE_PLL_60MHz);
		DCDCEnable(); // Enable the internal DCDC
		LSIEnable(); // Disable LSE, enable LSI
		ch5xx_sleep_rtc_init();

		//# advertise
		RFCoreInit(LL_TX_POWER_0_DBM);
		MESS_advertise(&sensor_cmd);

		//# sleep
		ch5xx_sleep_powerDown( MS_TO_RTC(SLEEPTIME_MS), (RB_PWR_RAM2K) );
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