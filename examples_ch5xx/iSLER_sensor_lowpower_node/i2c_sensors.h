#include "lib_i2c_ch5xx.h"
#include "register_debug_utilities.h"

#define DEBUG_ENABLED
#define R_SHUNT_mOHM 100

void bh1750_read(u8 address, u16 *lux) {
	//# request reading
	u8 ret = i2c_writeData(address, (u8[]){0x13}, 1);
	if (ret != 0) return;

	//# parse reading
	u8 data[2];
	ret = i2c_readData(address, data, 2);
	if (ret != 0) return;

	u16 lux_raw = BUF_MAKE_U16(data);
	*lux = lux_raw * 12 / 10;
}

void sht3x_read(u8 address, u16 *temp, u16 *hum) {
	//# parse reading
	u8 data[6];
	u8 ret = i2c_readData(address, data, 6);
	if (ret != 0) return;

	u16 temp_raw = BUF_MAKE_U16(data);
	u16 hum_raw = (data[3] << 8) | data[4];
	*temp = (175 * temp_raw) >> 16;		// >> 16 is equivalent to / 65536
	*hum = (100 * hum_raw) >> 16;		// >> 16 is equivalent to / 65536
}

u32 max_shunt_mV = 0;

void ina219_read(
	u8 address,
	u16 *bus_mV, u16 *shunt_mV,
	u16 *current_mA, u16 *power_mW
) {
	u8 ret;
	u8 buff[2];

	//# Read shunt voltage in uV
	ret = i2c_readReg_buffer(address, 0x01, buff, 2);
	if (ret != 0) return;

	u16 raw_shunt = (buff[0] << 8) | buff[1];
	*shunt_mV = raw_shunt/100;

	//# Read bus voltage in mV
	ret = i2c_readReg_buffer(address, 0x02, buff, 2);
	if (ret != 0) return;

	u16 raw_bus = (buff[0] << 8) | buff[1];
	*bus_mV = (raw_bus >> 3) * 4;

	// Max_Shunt_Current = Max_shunt_V / R_shunt;
	// Current_LSB = Max_Shunt_Current / 2^15 = Max_shunt_V / (R_shunt * 32768)
	u32 uCurrent_LSB = 1000000 * max_shunt_mV / (R_SHUNT_mOHM * 32768);

	//# Read power in uW
	ret = i2c_readReg_buffer(address, 0x03, buff, 2);
	if (ret != 0) return;

	u32 raw_power = (buff[0] << 8) | buff[1];
	// power = raw_value * 20 * Current_LSB
	u32 _power_mW = raw_power * 20 * uCurrent_LSB;
	*power_mW = (_power_mW + 500) / 1000; // convert to mW - rounded up

	//# Read current in mA
	ret = i2c_readReg_buffer(address, 0x04, buff, 2);
	if (ret != 0) return;

	u16 raw_current = (buff[0] << 8) | buff[1];
	// current = raw_value * current_LSB
	*current_mA = raw_current * uCurrent_LSB;
	*current_mA = (*current_mA + 500) / 1000; // convert to mA - rounded up
}


// bus_vRange = 1 (32V) or 0 (16V)
// pg_gain = 0 (gain/1 40mV), 1 (gain/2 80mV), 2 (gain/4 160mV), 3 (gain/8 320mV)
void i2c_ina219_setup(u8 address, u8 bus_vRange, u8 pg_gain) {
	// 9bits				= 0x0000	84us
	// 10bits 				= 0x0080	148us 
	// 11bits 				= 0x0100	276us 
	// 12bits 				= 0x0180	532us 
	// 12bits 2 samples 	= 0x0480	1.06ms
	// 12bits 4 samples 	= 0x0500	2.13ms
	// 12bits 8 samples 	= 0x0580	4.26ms	
	// 12bits 16 samples 	= 0x0600	8.51ms	
	// 12bits 32 samples 	= 0x0680	17.02ms	
	// 12bits 64 samples 	= 0x0700	34.05ms	 
	// 12bits 128 samples 	= 0x0780	68.10ms
	uint16_t BUS_RESOLUTION_AVERAGE = 0x0180;

	// Shunt Resolution = Bus_Resolution >> 1
	uint16_t SHUNT_RESOLUTION_AVERAGE = 0x0018;

	// 0x00 = Power Down
	// 0x01 = Shunt Voltage, triggered
	// 0x02 = Bus Voltage, triggered
	// 0x03 = Shunt and Bus Voltage, triggered
	// 0x04 = ADC Off
	// 0x05 = Shunt Voltage, continuous
	// 0x06 = Bus Voltage, continuous
	// 0x07 = Shunt and Bus Voltage, continuous - Default
	uint8_t DEVICE_MODE = 0x07;

	if (pg_gain > 3) pg_gain = 3;

	uint16_t config = (bus_vRange << 13) |
					(pg_gain << 11) |
					BUS_RESOLUTION_AVERAGE |
					SHUNT_RESOLUTION_AVERAGE |
					DEVICE_MODE;
    uint8_t config_bytes[3] = { 0x00, config >> 8, config & 0xFF };

	#ifdef DEBUG_ENABLED
		// expect 0x19 0x9F
		printf("\nconfig: 0x%02X 0x%02X\n", config_bytes[1], config_bytes[2]);
	#endif

    u8 ret;
    ret = i2c_writeData(address, config_bytes, 3);
	if (ret != 0) {
		#ifdef DEBUG_ENABLED
			printf("\nERROR: INA219 config 0x%02X\r\n", ret);
		#endif
		return;
	}

	// u8 data[2];
	// ret = i2c_readData(address, data, 2);
	// // expect 0x19 0x9F
	// printf("read config: 0x%02X 0x%02X\n", data[0], data[1]);

	
    switch (pg_gain) {
        case 0:	// 40mV
			max_shunt_mV = 40;
            break;
        case 1:	// 80mV
			max_shunt_mV = 80;
            break;
        case 2:	// 160mV
			max_shunt_mV = 160;
            break;
        case 3:	// 320mV - Default
			max_shunt_mV = 320;
            break;
    }

	// Given:
	// Max_Shunt_Current = Max_shunt_V / R_shunt;
	// Current_LSB = Max_Shunt_Current / 2^15 = Max_Shunt_Current / 32768
	// calib = .04096 / (current_LSB * R_shunt)

	// Therefore:
	// calib = .04096 / ((Max_Shunt_Current/32768) * R_shunt)
	// calib = .04096 * 32768 / (Max_shunt_V)
	// calib = .04096 * 32768 * 10^6 / (max_shunt_mV * 1000)
	// calib = 1342,177,280 / (max_shunt_mV * 1000)
	// Note: shunt_resistor cancel out, the calibration value does not depend on shunt_resistor
	u32 calibration_value = 1342177280 / (max_shunt_mV * 1000);
	uint8_t cal_bytes[3] = { 0x05, calibration_value >> 8, calibration_value & 0xFF };
	ret = i2c_writeData(address, cal_bytes, 3);

	#ifdef DEBUG_ENABLED
		// expect 0x10 0x62
		printf("cal: 0x%02X 0x%02X\n", cal_bytes[1], cal_bytes[2]);
	#endif

	if (ret != 0) {
		#ifdef DEBUG_ENABLED
			printf("\nERROR: INA219 calibration 0x%02X\r\n", ret);
		#endif
	}

	// ret = i2c_readReg_buffer(address, 0x05, data, 2);
	// // expect 0x10 0x62
	// printf("read calib: 0x%02X 0x%02X\n", data[0], data[1]);
}

void prepare_sensors() {
	//# setup BH1750
	u8 ret;
	u8 i2c_address = 0x23;

	// power on
	ret = i2c_writeData(i2c_address, (u8[]){0x01}, 1);
	if (ret != 0) {
		#ifdef DEBUG_ENABLED 
			printf("\nERROR: BH1750 powerON 0x%02X\r\n", ret);
		#endif
	} else {
		// set resolution
		ret = i2c_writeData(i2c_address, (u8[]){0x23}, 1);
		if (ret != 0) {
			#ifdef DEBUG_ENABLED
				printf("\nERROR: BH1750 resolution 0x%02X\r\n", ret);
			#endif
		}
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
	}

	//# setup INA219
	i2c_address = 0x40;
	i2c_ina219_setup(i2c_address, 0, 3);
}
