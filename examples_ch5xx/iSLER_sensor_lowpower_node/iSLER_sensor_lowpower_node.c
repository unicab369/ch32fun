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

#include "lib_i2c_ch5xx.h"

#define SLEEPTIME_MS 5000
#define LED_PIN PA8
#define SENSOR_POWER_PIN PB22

#define I2C_SDA PB12
#define I2C_SCL PB13

#define SHUTDOWN_MODE_ENABLED

remote_command_t sensor_cmd = {
	.command = 0xBB,
	.value1 = 0,
	.value2 = 0,
	.value3 = 0
};

u16 bh1750_read() {
	u8 address = 0x23;

	//# request reading
	u8 ret = i2c_writeData(address, (u8[]){0x13}, 1);
	if (ret != 0) { return 0; }

	//# parse reading
	u8 data[2];
	ret = i2c_readData(address, data, 2);
	if (ret != 0) { return 0; }

	u16 lux_raw = BUF_MAKE_U16(data);
	return lux_raw * 12 / 10;
}

void sht3x_read(u16 *temp, u16 *hum) {
	u8 addres = 0x44;

	//# parse reading
	u8 data[6];
	u8 ret = i2c_readData(addres, data, 6);
	if (ret != 0) { printf("\nERROR: I2C reading 0x%02X\r\n", ret); return; }

	u16 temp_raw = BUF_MAKE_U16(data);
	u16 hum_raw = (data[3] << 8) | data[4];
	*temp = (175 * temp_raw) >> 16;		// >> 16 is equivalent to / 65536
	*hum = (100 * hum_raw) >> 16;		// >> 16 is equivalent to / 65536
}

void collect_readings() {
	u8 err = i2c_init(100);
	// printf("\nI2C init: %d\r\n", err);

	//# setup BH1750
	u16 temp, hum, lux;

	// power on
	u8 ret = i2c_writeData(0x23, (u8[]){0x01}, 1);
	if (ret != 0) { return; }

	// set resolution
	ret = i2c_writeData(0x23, (u8[]){0x23}, 1);
	if (ret != 0) { return; }

	//# setup SHT3x
	// soft reset
	ret = i2c_writeData(0x44, (u8[]){0x30, 0xA2}, 2);
	// this command will alwasy be busy, don't check for error
	// Delay_Ms(1);	//! DELAY is REQUIRED on power up

	// config
	ret = i2c_writeData(0x44, (u8[]){0x21, 0x30}, 2);
	if (ret != 0) { return; }
	Delay_Us(100);	//! DELAY is REQUIRED on power up
	sht3x_read(&temp, &hum);
	sensor_cmd.value1 = temp;
	sensor_cmd.value2 = hum;

	//# get BH1750 reading
	lux = bh1750_read();
	sensor_cmd.value3 = lux;
	printf("temp: %d, hum: %d, lux: %d\n", temp, hum, lux);
}

#ifdef SHUTDOWN_MODE_ENABLED
	int main() {
		SystemInit();
		funGpioInitAll();

		funDigitalWrite(SENSOR_POWER_PIN, 1);
		funPinMode(I2C_SCL, GPIO_CFGLR_IN_PUPD);
		funPinMode(I2C_SDA, GPIO_CFGLR_IN_PUPD);
		funPinMode(SENSOR_POWER_PIN, GPIO_CFGLR_OUT_2Mhz_PP);

		//# Turn on the LED_PIN - for WeAct board PA8 is active LOW
		// funPinMode(LED_PIN, GPIO_CFGLR_OUT_2Mhz_PP);
		// funDigitalWrite(LED_PIN, 0);

		//# Do these tasks to buy time for the i2C sensors to power up
		ch5xx_setClock(CLK_SOURCE_PLL_60MHz);
		RFCoreInit(LL_TX_POWER_0_DBM);
		DCDCEnable(); // Enable the internal DCDC
		LSIEnable(); // Disable LSE, enable LSI
		ch5xx_sleep_rtc_init();

		//# get the i2C sensors readings
		collect_readings();

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

		// printf("Size of message: %d\n", sizeof(iSLER_frame_t));
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