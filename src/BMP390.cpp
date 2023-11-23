#include "BMP390.h"
#include <Wire.h>

#define CHIP_ID 0x00

//Status registers
#define ERR_REG 0x02
#define STATUS 0x03
#define EVENT 0x10

//Mode Settings
#define PWR_CTRL 0x1B

//Data Settings
#define OSR 0x1C
#define ODR 0x1D
#define CONFIG 0x1F

//Commands
#define CMD 0x7E
#define SOFTRESET 0xB6

//Data Registers
#define PRESS_REG 0x04 // to 0x06
#define TEMP_REG 0x07 // to 0x09

//Compensation Values
#define CALIBRATION_REG 0x31 //to 0x45

BMP390::BMP390(bool addrLSB = 0) {
	BMP_ADDR = 0x76 + addrLSB;
}

bool BMP390::I2Cread(uint8_t addr,  uint8_t* data,  uint8_t len) {
  uint8_t pos = 0;

  Wire.beginTransmission(addr);
  Wire.requestFrom(addr, len);

  while(Wire.available() && pos<len) {
    data[pos] = Wire.read();
    pos++;
  }
  return pos == len;
}

bool BMP390::I2Cwrite(uint8_t addr,  uint8_t* data, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(data, len);
  return Wire.endTransmission() == 0;
}

bool BMP390::getCompensationData() {
	bool status = 1; 
	if(compData != nullptr) {
		delete compData;
		compData = nullptr;
	}
	uint8_t data[1];
	data[0] = CALIBRATION_REG;
	I2Cwrite(BMP_ADDR, data, 1);
	status = I2Cread(BMP_ADDR, NVM_PAR, 21);
	if(!status)
		return status;
	compData = new calibData(NVM_PAR, sizeof(NVM_PAR));
	return status;
}

uint8_t BMP390::begin() {

	bool ret = 1;
	uint8_t* dataByte;
	uint8_t data[2];
	uint8_t len=0;
	*dataByte = CHIP_ID;
	I2Cwrite(BMP_ADDR, dataByte, 1);
	ret = I2Cread(BMP_ADDR, dataByte, 1);

	if(!ret)
		return 1;

	if(*dataByte & 0x60)
		return 2;
	
	if(!getCompensationData())
	return 3;

	if(!setSettings(DEFAULT_PWR_SETTINGS, DEFAULT_ODR_SETTINGS, DEFAULT_OSR_SETTINGS, DEFAULT_CONFIG_SETTINGS)) 
		return 4;

	return 0;
}

int64_t BMP390::calcTemp(int64_t raw, calibData* parData) {
    int64_t partial_data1;
    int64_t partial_data2;
    int64_t partial_data3;
    int64_t partial_data4;
    int64_t partial_data5;
    int64_t partial_data6;
    int64_t comp_temp;

    partial_data1 = (int64_t)(raw - ((int64_t)256 * parData->PAR_T1));
    partial_data2 = (int64_t)(parData->PAR_T2 * partial_data1);
    partial_data3 = (int64_t)(partial_data1 * partial_data1);
    partial_data4 = (int64_t)partial_data3 * parData->PAR_T3;
    partial_data5 = (int64_t)((int64_t)(partial_data2 * 262144) + partial_data4);
    partial_data6 = (int64_t)(partial_data5 / 4294967296);

    /* Store t_lin in dev. structure for pressure calculation */
 	parData->linTemp = (int64_t)partial_data6;
    return ((int64_t)((partial_data6 * 25) / 16384));


}
//TODO change division to shift registers
uint64_t BMP390::calcPres(int64_t raw, calibData* parData) {
    int64_t partial_data1;
    int64_t partial_data2;
    int64_t partial_data3;
    int64_t partial_data4;
    int64_t partial_data5;
    int64_t partial_data6;
    int64_t offset;
    int64_t sensitivity;
    uint64_t comp_press;

    partial_data1 = (int64_t)(parData->linTemp * parData->linTemp);
    partial_data2 = (int64_t)(partial_data1 / 64);
    partial_data3 = (int64_t)((partial_data2 * parData->linTemp) / 256);
    partial_data4 = (int64_t)((parData->PAR_P8 * partial_data3) / 32);
    partial_data5 = (int64_t)((parData->PAR_P7 * partial_data1) * 16);
    partial_data6 = (int64_t)((parData->PAR_P6 * parData->linTemp) * 4194304);
    offset = (int64_t)((parData->PAR_P5 * 140737488355328) + partial_data4 + partial_data5 + partial_data6);
    partial_data2 = (int64_t)((parData->PAR_P4 * partial_data3) / 32);
    partial_data4 = (int64_t)((parData->PAR_P3 * partial_data1) * 4);
    partial_data5 = (int64_t)((parData->PAR_P2 - (int32_t)16384) * parData->linTemp * 2097152);
    sensitivity =
        (int64_t)(((parData->PAR_P1 - (int32_t)16384) * 70368744177664) + partial_data2 + partial_data4 +
                  partial_data5);
    partial_data1 = (int64_t)((sensitivity / 16777216) * raw);
    partial_data2 = (int64_t)(parData->PAR_P10 * parData->linTemp);
    partial_data3 = (int64_t)(partial_data2 + ((int32_t)65536 * parData->PAR_P9));
    partial_data4 = (int64_t)((partial_data3 * raw) / (int32_t)8192);

    /* dividing by 10 followed by multiplying by 10
     * To avoid overflow caused by (uncomp_data->pressure * partial_data4)
     */
    partial_data5 = (int64_t)((raw * (partial_data4 / 10)) / (int32_t)512);
    partial_data5 = (int64_t)(partial_data5 * 10);
    partial_data6 = (int64_t)(raw * raw);
    partial_data2 = (int64_t)((parData->PAR_P11 * partial_data6) / (int32_t)65536);
    partial_data3 = (int64_t)((int64_t)(partial_data2 * raw) / 128);
    partial_data4 = (int64_t)((offset / 4) + partial_data1 + partial_data5 + partial_data3);

    return ((((uint64_t)partial_data4 * 25) / (uint64_t)1099511627776));
}

void BMP390::requestNewData() {
	uint8_t data[2];
	data[0] = PWR_CTRL;
	data[1] = DEFAULT_PWR_SETTINGS;
	I2Cwrite(BMP_ADDR, data, 2);
}

uint8_t BMP390::readRaw() {
	uint8_t data[1];
	data[0] = STATUS;
	I2Cwrite(BMP_ADDR, data, 1);
	I2Cread(BMP_ADDR, data, 1);
	if((data[0] & 0b01100000) != 0b01100000) 
		return 1;

	data[0] = PRESS_REG;
	I2Cwrite(BMP_ADDR, data, 1);
	I2Cread(BMP_ADDR, rawData, 6);

	rawTemp = (((uint32_t)rawData[5] << 16) | ((uint32_t)rawData[4] << 8)) | (uint32_t)rawData[3]; 
	rawPres = (((uint32_t)rawData[2] << 16) | ((uint32_t)rawData[1] << 8)) | (uint32_t)rawData[0]; 
	return 0;
}

uint8_t BMP390::getData(float *array, uint8_t arrLen) {
	if(arrLen<2) 
		return 1;

	if(readRaw() == 1)
		return 2;
	array[0] = (float)calcTemp(rawTemp, compData)/100.0;
	array[1] = (float)calcPres(rawPres, compData)/100.0;
	
	return 0;
}

bool BMP390::setSettings(uint8_t PWR_Settings, uint8_t ODR_Settings, uint8_t OSR_Settings, uint8_t CONFIG_Settings) {
	bool status = 1;
	uint8_t data[2];

	data[0] = PWR_CTRL;
	data[1] = PWR_Settings;
	status = I2Cwrite(BMP_ADDR, data, 2);
	if(!status) return 0;

	data[0] = ODR;
	data[1] = ODR_Settings;
	status = I2Cwrite(BMP_ADDR, data, 2);
	if(!status) return 0;

	data[0] = OSR;
	data[1] = OSR_Settings;
	status = I2Cwrite(BMP_ADDR, data, 2);
	if(!status) return 0;

	data[0] = CONFIG;
	data[1] = CONFIG_Settings;
	status = I2Cwrite(BMP_ADDR, data, 2);
	return status;
}

uint8_t BMP390::getStatus() {
	uint8_t data[1];
	data[0] = STATUS;
	I2Cwrite(BMP_ADDR, data, 1);
	I2Cread(BMP_ADDR, data, 1);
	return data[0];
}

void BMP390::softReset() {
	uint8_t data[2];
	data[0] = CMD;
	data[1] = SOFTRESET;
	I2Cwrite(BMP_ADDR, data, 2);
}

void BMP390::getCalibData(float* array) {
	array[0] = compData->PAR_T1;
	array[1] = compData->PAR_T2;
	array[2] = compData->PAR_T3;

	array[3] = compData->PAR_P1;
	array[4] = compData->PAR_P2;
	array[5] = compData->PAR_P3;
	array[6] = compData->PAR_P4;
	array[7] = compData->PAR_P5;
	array[8] = compData->PAR_P6;
	array[9] = compData->PAR_P7;
	array[10] = compData->PAR_P8;
	array[11] = compData->PAR_P9;
	array[12] = compData->PAR_P10;
	array[13] = compData->PAR_P11;
}