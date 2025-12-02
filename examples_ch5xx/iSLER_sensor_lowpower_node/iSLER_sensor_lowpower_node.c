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

#define SLEEPTIME_MS 5000
#define LED_PIN PA8
#define SENSOR_POWER_PIN PB22

#define I2C_SDA PB12
#define I2C_SCL PB13

#define SHUTDOWN_MODE_ENABLED


void onHandle_pingFound(int address) {
	printf("i2C found: 0x%02X\n", address);
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
		// funPinMode(LED_PIN, GPIO_CFGLR_OUT_2Mhz_PP);
		// funDigitalWrite(LED_PIN, 0);

		//# Do these tasks to buy time for the i2C sensors to power up
		// ch5xx_setClock(CLK_SOURCE_PLL_60MHz);
		// RFCoreInit(LL_TX_POWER_0_DBM);
		// DCDCEnable(); // Enable the internal DCDC
		// LSIEnable(); // Disable LSE, enable LSI
		// // ch5xx_sleep_rtc_init();

		//# get the i2C sensors readings
		Delay_Ms(500);
		collect_readings();

		return;

		//# Turn off the LED_PIN by doing pullup
		ch5xx_allPinsPullUp();
		funPinMode(SENSOR_POWER_PIN, GPIO_CFGLR_OUT_2Mhz_PP);
		funDigitalWrite(SENSOR_POWER_PIN, 0);

		//# advertise
		MESS_advertise(&sensor_cmd);

		//# Disconnect PA15 from GND to enter power_down mode.
		if (funDigitalRead(PA15)) {
			ch5xx_sleep_powerDown( MS_TO_RTC(SLEEPTIME_MS), (RB_PWR_RAM2K) );
		} 
		else {
			funPinMode(LED_PIN, GPIO_CFGLR_OUT_10Mhz_PP);

			while (1) {
				if (funDigitalRead(PA15)) {
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