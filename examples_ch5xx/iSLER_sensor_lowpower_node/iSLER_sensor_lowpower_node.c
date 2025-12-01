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
#include "register_debug_utilities.h"

#define DEBUG_ENABLED
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

u16 bh1750_read(u8 address) {
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

void sht3x_read(u8 address, u16 *temp, u16 *hum) {
	//# parse reading
	u8 data[6];
	u8 ret = i2c_readData(address, data, 6);
	if (ret != 0) {
		#ifdef DEBUG_ENABLED 
			printf("\nERROR: I2C reading 0x%02X\r\n", ret); return;
		#endif
	}

	u16 temp_raw = BUF_MAKE_U16(data);
	u16 hum_raw = (data[3] << 8) | data[4];
	*temp = (175 * temp_raw) >> 16;		// >> 16 is equivalent to / 65536
	*hum = (100 * hum_raw) >> 16;		// >> 16 is equivalent to / 65536
}

#define MAX_CURRENT_MA 300

u8 ina219_read(u8 address, u16 *shunt_uV, u16 *bus_mV, u16 *current_uA, u16 *power_uW) {
	u8 ret;
	u8 I2C_DATA_BUF[2];

	//# Read shunt voltage
	if (shunt_uV != NULL) {
		printf("IM HERE 1\n");
		ret = i2c_readReg_buffer(address, 0x01, I2C_DATA_BUF, 2);
		if (ret != 0) { printf("\nERROR: I2C reading 0x%02X\r\n", ret); return ret; }
		u16 shunt_raw = BUF_MAKE_U16(I2C_DATA_BUF);
		*shunt_uV = shunt_raw * 10;
	}

	//# Read bus voltage
	if (bus_mV != NULL) {
		printf("IM HERE 2\n");
		ret = i2c_readReg_buffer(address, 0x02, I2C_DATA_BUF, 2);
		if (ret != 0) { printf("\nERROR: I2C reading 0x%02X\r\n", ret); return ret; }
		u16 bus_raw = BUF_MAKE_U16(I2C_DATA_BUF);
		*bus_mV = (bus_raw >> 3) * 4;
	}

	//# Read power
	if (power_uW != NULL) {
		printf("IM HERE 3\n");
		ret = i2c_readReg_buffer(address, 0x03, I2C_DATA_BUF, 2);
		if (ret != 0) { printf("\nERROR: I2C reading 0x%02X\r\n", ret); return ret; }
		// power_LSB = 20 * current_LSB = 20 * max_current/32768
		u16 power_raw = BUF_MAKE_U16(I2C_DATA_BUF);
		*power_uW = 20 * power_raw * 1000 * MAX_CURRENT_MA / 32768;
	}


	//# Read current
	if (current_uA != NULL) {
		printf("IM HERE 4\n");
		ret = i2c_readReg_buffer(address, 0x04, I2C_DATA_BUF, 2);
		if (ret != 0) { printf("\nERROR: I2C reading 0x%02X\r\n", ret); return ret; }
		// current_LSB = max_current/32768
		// current_mA = current_raw * current_LSB
		u16 current_raw = BUF_MAKE_U16(I2C_DATA_BUF);
		*current_uA = current_raw * 1000 * MAX_CURRENT_MA / 32768;
	}
}


void collect_readings() {
	//# setup BH1750
	u16 temp, hum, lux;
	u8 ret;
	u8 i2c_address = 0x23;

	// power on
	ret = i2c_writeData(i2c_address, (u8[]){0x01}, 1);
	if (ret != 0) {
		#ifdef DEBUG_ENABLED 
			printf("\nERROR: BH1750 powerON 0x%02X\r\n", ret);
		#endif
		return;
	}

	// set resolution
	ret = i2c_writeData(i2c_address, (u8[]){0x23}, 1);
	if (ret != 0) {
		#ifdef DEBUG_ENABLED
			printf("\nERROR: BH1750 resolution 0x%02X\r\n", ret);
		#endif
		return;
	}

	//# setup SHT3x
	i2c_address = 0x44;

	// soft reset
	ret = i2c_writeData(i2c_address, (u8[]){0x30, 0xA2}, 2);
	// this command will alwasy be busy, don't check for error
	// Delay_Ms(1);	//! DELAY is REQUIRED on power up

	// config
	ret = i2c_writeData(i2c_address, (u8[]){0x21, 0x30}, 2);
	if (ret != 0) {
		#ifdef DEBUG_ENABLED
			printf("\nERROR: SHT3x config 0x%02X\r\n", ret);
		#endif
		return;
	}

	Delay_Us(100);	//! DELAY is REQUIRED on power up
	sht3x_read(0x44, &temp, &hum);
	sensor_cmd.value1 = temp;
	sensor_cmd.value2 = hum;

	//# get BH1750 reading
	lux = bh1750_read(0x23);
	sensor_cmd.value3 = lux;
	printf("temp: %d, hum: %d, lux: %d\n", temp, hum, lux);


	//# setup INA219
	i2c_address = 0x40;

	// max_cal <= 0xFFFF, solve for usuable max_current & shunt
	// max_current_mA * shunt_mOhm >= 1342,177,280/0XFFFF = 20480
	// max_current_mA * shunt_mOhm has to be greater than 20480 for a valid calibration value
	// if shunt_mOhm = 100 mOhm, max_current_mA has to be at least 200 (300 recommended bc of rounding errors)
	u16 ina219_config = (0 << 13) |	// 16V range
						(0 << 11) |	// +-40 mV
						(3 << 7)  |	// 12-bit Bus Resolution
						(3 << 3)  |	// 12-bit Shunt Resolution
						(7 << 0);	// Mode: Shunt and Bus Continous
	u8 config_bytes[3] = { 0x00, ina219_config >> 8, ina219_config & 0xFF };
	ret = i2c_writeData(i2c_address, config_bytes, 3);
	if (ret != 0) {
		#ifdef DEBUG_ENABLED
			printf("\nERROR: INA219 config 0x%02X\r\n", ret);
		#endif
	}

	u8 I2C_DATA_BUF[8];
	ret = i2c_readData(i2c_address, I2C_DATA_BUF, 2);

	u16 config_read = BUF_MAKE_U16(I2C_DATA_BUF);
	printf("\nConfig register: 0x%04X\n", config_read);
	UTIL_PRINT_BITS(config_read, 16, 16);

	u16 SHUNT_RESISTER_OHM = 140;
	u32 cal = 1342177280 / (MAX_CURRENT_MA * SHUNT_RESISTER_OHM);
	u8 cal_bytes[3] = { 0x05, cal >> 8, cal & 0xFF };
	ret = i2c_writeData(i2c_address, cal_bytes, 3);
	if (ret != 0) {
		#ifdef DEBUG_ENABLED
			printf("\nERROR: INA219 cal 0x%02X\r\n", ret);
		#endif
	}

	ret = i2c_readReg_buffer(i2c_address, 0x05, I2C_DATA_BUF, 2);
	u16 cal_read = BUF_MAKE_U16(I2C_DATA_BUF);

	printf("\nCalibration value: 0x%04X\n", cal);
	printf("Calibration register: 0x%04X\n", cal_read);

	u16 shunt_uV = 0, bus_mV = 0, current_uA = 0, power_uW = 0;

	while(1) {
		ina219_read(i2c_address, &shunt_uV, &bus_mV, &current_uA, &power_uW);
		printf("shunt_uV: %d, bus_mV: %d, current_uA: %d, power_uW: %d\n", shunt_uV, bus_mV, current_uA, power_uW);
		Delay_Ms(1000);
	}
}

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
		Delay_Ms(100);
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