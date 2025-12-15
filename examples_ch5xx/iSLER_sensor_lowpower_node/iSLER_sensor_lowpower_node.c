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

#define SLEEPTIME_MS 4000
#define SOLAR_SWITCH_THRESHOLD_mV 2900

#define LED_PIN					PA8
#define SLEEP_MODE_PIN 			PA15		// LOW = Exit shutdown mode

#define SW_DIVIDER 				PA5			// HIGH = use external voltage divider
#define SW_SENSORS 				PB22		// HIGH = Turn on power to sensors
#define SW_SOLAR 				PB10			// HIGH = switch to Solar power source

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
u16 bus_mV, shunt_mV, power_mW;

void collect_readings() {
	u16 temp, hum, lux;

	//# Turn off the LED_PIN
	funDigitalWrite(LED_PIN, 1);

	//# get internal voltage reading
	adc_set_channel(ADC_VBAT_CHANNEL);
	adc_set_config(ADC_FREQ_DIV_10, ADC_PGA_GAIN_1_2, 0);
	int vInternal_mV = adc_to_mV(adc_get_singleReading(), ADC_PGA_GAIN_1_2);
	vInternal_mV = (vInternal_mV + adc_to_mV(adc_get_singleReading(), ADC_PGA_GAIN_1_2))/2;
	sensor_cmd.value4 = vInternal_mV;

	//# get SHT3x reading
	sht3x_read(SHT3X_ADDR, &temp, &hum);
	sensor_cmd.value1 = temp;
	sensor_cmd.value2 = hum;

	//# get BH1750 reading
	bh1750_read(BH1750_ADDR, &lux);
	sensor_cmd.value3 = lux;

	//# ADC PA14
	adc_set_channel(4);
	adc_set_config(ADC_FREQ_DIV_10, ADC_PGA_GAIN_1_2, 0);
	int solar_mV = adc_to_mV(adc_get_singleReading(), ADC_PGA_GAIN_1_2);
	solar_mV = (solar_mV + adc_to_mV(adc_get_singleReading(), ADC_PGA_GAIN_1_2))/2;
	solar_mV = 200+(solar_mV*1000)/333;
	sensor_cmd.value5 = solar_mV;

	//# turn OFF voltage divider
	funDigitalWrite(SW_DIVIDER, 0);

	//! WARNING: turn OFF SW_DIVIDER FIRST to avoid the solar voltage to go into the ADC
	//# GND PA14 to reduce power consumption
	funPinMode(PA14, GPIO_CFGLR_OUT_2Mhz_PP);
	funDigitalWrite(PA14, 0);

	//# ADC PA13
	adc_set_channel(3);
	adc_set_config(ADC_FREQ_DIV_10, ADC_PGA_GAIN_1_2, 0);
	int current_mV = adc_to_mV(adc_get_singleReading(), ADC_PGA_GAIN_1_2);
	current_mV = (current_mV + adc_to_mV(adc_get_singleReading(), ADC_PGA_GAIN_1_2))/2;

	// current = ADC / (Gain * ShuntResistor); Gain = 50 for INA180A2; Shunt = 0.1 Ohm
	int current_uA = current_mV*1000/5 - 2000; 		// 2000 offset
	sensor_cmd.value6 = current_uA;
	//# Pullup PA13 to reduce power consumption
	// funPinMode(PA13, GPIO_CFGLR_OUT_50Mhz_PP);
	// funDigitalWrite(PA13, 1);

	//# Turn off sensor power
	funDigitalWrite(SW_SENSORS, 1);

	//# turn ON solar panel if internal voltage and solar voltage are above threshold
	int check1 = vInternal_mV > SOLAR_SWITCH_THRESHOLD_mV;
	int check2 = solar_mV > SOLAR_SWITCH_THRESHOLD_mV;
	funPinMode(SW_SOLAR, GPIO_CFGLR_OUT_10Mhz_PP);
	funDigitalWrite(SW_SOLAR, check1 && check2);

	#ifdef TEST_MODE_ENABLED
		//# clear display
		// ssd1306_draw_fill(0x00);

		sprintf(str_output, "%dF, %d%%, lux:%d", temp, hum, lux);
		// menu_render_text_at(0, str_output);
		printf("\n\n%s", str_output);

		printf("\nInternal Voltage: %d mV", vInternal_mV);
		printf("\nSolar Voltage: ~%d mV", solar_mV);

		printf("\ncurrent: %d uA", current_uA);
		printf("\nSensors readings:\n");

		//# update display
		// ssd1306_draw_all();
	#endif
}

#ifdef SHUTDOWN_MODE_ENABLED
	int main() {
		SystemInit();
		funGpioInitAll();
		ch5xx_allPinsPullUp();
		DCDCEnable(); // Enable the internal DCDC
		LSIEnable(); // Disable LSE, enable LSI

		//# Voltage Divider Pin HIGH = use external voltage divider
		funPinMode(SW_DIVIDER, GPIO_CFGLR_OUT_2Mhz_PP);
		funDigitalWrite(SW_DIVIDER, 1);

		//# Sensor Power Pin LOW = turn ON sensors (PFet)
		funPinMode(SW_SENSORS, GPIO_CFGLR_OUT_2Mhz_PP);
		funDigitalWrite(SW_SENSORS, 0);

		//# Power Control Pin LOW = default to battery power
		// funPinMode(SW_SOLAR, GPIO_CFGLR_OUT_10Mhz_PP);

		//# Sleep Mode Pin HIGH = enter shutdown mode
		funPinMode(SLEEP_MODE_PIN, GPIO_CFGLR_IN_PUPD);
		funDigitalWrite(SLEEP_MODE_PIN, 1);
		
		//# Turn on the LED_PIN - for WeAct board PA8 is active LOW
		funPinMode(LED_PIN, GPIO_CFGLR_OUT_2Mhz_PP);
		funDigitalWrite(LED_PIN, 0);

		//# setup RF
		// ch5xx_setClock(CLK_SOURCE_PLL_80MHz);
		RFCoreInit(LL_TX_POWER_3_DBM);

		//# setup I2C
		u8 err = i2c_init(100);
		#ifdef I2C_SCAN_ENABLED
			printf("\nI2C init: %d", err);
			i2c_scan(onHandle_pingFound);
		#endif

		#ifdef TEST_MODE_ENABLED
			int toggle = 1;
			ssd1306_init();

			for(;;) {
				// if (funDigitalRead(SLEEP_MODE_PIN)) break;

				funDigitalWrite(SW_DIVIDER, 1);
				funDigitalWrite(SW_SENSORS, 0);
				funDigitalWrite(LED_PIN, toggle);
				toggle = !toggle;

				//# get sensors readings  // 10ms
				Delay_Ms(10);
				sensor_cmd.value8 = prepare_sensors();
				Delay_Ms(15);
				collect_readings();

				//# advertise
				MESS_advertise(&sensor_cmd);
				Delay_Ms(1000);
			}
		#else
			Delay_Ms(1);
			sensor_cmd.value8 = prepare_sensors();
			Delay_Ms(17);
			collect_readings();
		#endif

		while(!funDigitalRead(SLEEP_MODE_PIN)) {
            funDigitalWrite(LED_PIN, 0); Delay_Ms(100);
            funDigitalWrite(LED_PIN, 1); Delay_Ms(100);
        }

		//# Set I2C to reduce power consumption
		funPinMode(I2C_SCL, GPIO_CFGLR_OUT_2Mhz_PP);
		funPinMode(I2C_SDA, GPIO_CFGLR_OUT_2Mhz_PP);
		funDigitalWrite(I2C_SCL, 0);
		funDigitalWrite(I2C_SDA, 0);

		//# advertise
		MESS_advertise(&sensor_cmd);

		//# prepare for sleep
		ch5xx_sleep_rtc_init();

		//! Enter sleep
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