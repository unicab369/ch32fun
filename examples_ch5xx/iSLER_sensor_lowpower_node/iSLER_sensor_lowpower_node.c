// Send message to iSLER_sensors_gateway

// When the chip is sleeping, you won't be able to flash it,
// Hold the BOOT pin while powering up the chip and flash it before it enters user mode

//! WARNING: WHEN USE SHUTDOWN_MODE_ENABLED
//! PA15 NEEDS TO BE IN PULLED UP MODE OR YOU WONT BE ABLE TO REFLASH THE CHIP if you dont have access to the BOOT pin PB22
//# Ground PA15 to enter programming mode (in programming mode, PA8 led will be flashing)
//# after flashing done, unground PA15 to enter shutdown mode

#include "ch32fun.h"
#include <stdio.h>

#include "ch5xx_sleep.h"
#include "ch5xx_Mess.h"
#include "i2c_sensors.h"

#define SLEEPTIME_MS 10000

#define LED_PIN PA8
#define SENSOR_POWER_PIN PB22
#define SENSOR_MODE_PIN PA15

#define I2C_SDA PB12
#define I2C_SCL PB13

#define SHUTDOWN_MODE_ENABLED

remote_command_t sensor_cmd = {
	.command = 0xBB,
	.value1 = 0,
	.value2 = 0,
	.value3 = 0,
	.value4 = 0,
	.value5 = 0
};

void onHandle_pingFound(int address) {
	printf("i2C found: 0x%02X\n", address);
}

void collect_readings() {
	u16 bus_mV, shunt_mV, current_mA, power_mW;
	u16 temp, hum, lux;

	Delay_Us(100);	//! DELAY is REQUIRED on power up
	sht3x_read(0x44, &temp, &hum);
	sensor_cmd.value1 = temp;
	sensor_cmd.value2 = hum;

	//# get BH1750 reading
	bh1750_read(0x23, &lux);
	sensor_cmd.value3 = lux;
	printf("\ntemp: %d, hum: %d, lux: %d\n", temp, hum, lux);

	ina219_read(0x40, &bus_mV, &shunt_mV, &current_mA, &power_mW);
	sensor_cmd.value4 = bus_mV;
	sensor_cmd.value5 = current_mA;
	printf("bus_mV: %d, shunt_mV: %d, current_mA: %d, power_mW: %d\n",
			bus_mV, shunt_mV, current_mA, power_mW);
}

#ifdef SHUTDOWN_MODE_ENABLED
	int main() {
		SystemInit();
		funGpioInitAll();

		funDigitalWrite(SENSOR_POWER_PIN, 1);
		funPinMode(I2C_SCL, GPIO_CFGLR_IN_PUPD);
		funPinMode(I2C_SDA, GPIO_CFGLR_IN_PUPD);
		funPinMode(SENSOR_POWER_PIN, GPIO_CFGLR_OUT_2Mhz_PP);

		u8 err = i2c_init(100);
		#ifdef DEBUG_ENABLED
			// printf("\nI2C init: %d\r\n", err);
			// i2c_scan(onHandle_pingFound);
		#endif

		//# Turn on the LED_PIN - for WeAct board PA8 is active LOW
		funPinMode(LED_PIN, GPIO_CFGLR_OUT_2Mhz_PP);
		funDigitalWrite(LED_PIN, 0);

		// //# Do these tasks to buy time for the i2C sensors to power up
		// ch5xx_setClock(CLK_SOURCE_PLL_60MHz);
		// RFCoreInit(LL_TX_POWER_0_DBM);
		// DCDCEnable(); // Enable the internal DCDC
		// LSIEnable(); // Disable LSE, enable LSI
		// ch5xx_sleep_rtc_init();

		//# get the i2C sensors readings
		Delay_Ms(500);
		prepare_sensors();

		while(1) {
			collect_readings();
			Delay_Ms(1000);
		}

		return;

		//# Turn off the LED_PIN by doing pullup
		ch5xx_allPinsPullUp();
		funPinMode(SENSOR_POWER_PIN, GPIO_CFGLR_OUT_2Mhz_PP);
		funDigitalWrite(SENSOR_POWER_PIN, 0);

		//# advertise
		MESS_advertise(&sensor_cmd);

		//# Disconnect SENSOR_MODE_PIN from GND to enter power_down mode.
		if (funDigitalRead(SENSOR_MODE_PIN)) {
			ch5xx_sleep_powerDown( MS_TO_RTC(SLEEPTIME_MS), (RB_PWR_RAM2K) );
		} 
		else {
			funPinMode(LED_PIN, GPIO_CFGLR_OUT_10Mhz_PP);

			while (1) {
				if (funDigitalRead(SENSOR_MODE_PIN)) {
					ch5xx_sleep_powerDown( MS_TO_RTC(SLEEPTIME_MS), (RB_PWR_RAM2K) );
				}
				funDigitalWrite(LED_PIN, 0); Delay_Ms(50);
				funDigitalWrite(LED_PIN, 1); Delay_Ms(50);
			}
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