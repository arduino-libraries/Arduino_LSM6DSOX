/*
  This file is part of the Arduino_LSM6DSOX library.
  Copyright (c) 2021 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "LSM6DSOX.h"

#include <map>

#define LSM6DSOX_ADDRESS            0x6A

#define LSM6DSOX_FIFO_CTRL1         0x07
#define LSM6DSOX_FIFO_CTRL2         0x08
#define LSM6DSOX_FIFO_CTRL3         0x09
#define LSM6DSOX_FIFO_CTRL4         0x0A

#define LSM6DSOX_WHO_AM_I_REG       0X0F
#define LSM6DSOX_CTRL1_XL           0X10
#define LSM6DSOX_CTRL2_G            0X11
#define LSM6DSOX_CTRL3_C            0X12

#define LSM6DSOX_CTRL2_G            0X11

#define LSM6DSOX_CTRL5_C            0X14
#define LSM6DSOX_CTRL6_C            0X15
#define LSM6DSOX_CTRL7_G            0X16
#define LSM6DSOX_CTRL8_XL           0X17
#define LSM6DSOX_CTRL10_C           0X19

#define LSM6DSOX_STATUS_REG         0X1E

#define LSM6DSOX_OUT_TEMP_L         0X20
#define LSM6DSOX_OUT_TEMP_H         0X21

#define LSM6DSOX_OUTX_L_G           0X22
#define LSM6DSOX_OUTX_H_G           0X23
#define LSM6DSOX_OUTY_L_G           0X24
#define LSM6DSOX_OUTY_H_G           0X25
#define LSM6DSOX_OUTZ_L_G           0X26
#define LSM6DSOX_OUTZ_H_G           0X27

#define LSM6DSOX_OUTX_L_XL          0X28
#define LSM6DSOX_OUTX_H_XL          0X29
#define LSM6DSOX_OUTY_L_XL          0X2A
#define LSM6DSOX_OUTY_H_XL          0X2B
#define LSM6DSOX_OUTZ_L_XL          0X2C
#define LSM6DSOX_OUTZ_H_XL          0X2D

#define LSM6DSOX_TIMESTAMP0         0x40
#define LSM6DSOX_TIMESTAMP1         0x41
#define LSM6DSOX_TIMESTAMP2         0x42
#define LSM6DSOX_TIMESTAMP3         0x43

#define LSM6DSOX_INTERNAL_FREQ_FINE 0x63

// Number of retries after read or write error (this number is a bit arbitrary...)
#define i2C_RETRIES                 5

// Map from sample rate to ODR configuration bits
std::map< uint16_t, uint8_t > mapSampleRateToODR = { 
  {   13, 0b0001 }, 
  {   26, 0b0010 }, 
  {   52, 0b0011 }, 
  {  104, 0b0100 }, 
  {  208, 0b0101 }, 
  {  417, 0b0110 }, 
  {  833, 0b0111 }, 
  { 1667, 0b1000 }, 
  { 3333, 0b1001 }, 
  { 6667, 0b1010 } 
};
std::map< uint8_t, uint8_t > mapAccelRangeToFSXL = { // FS1_XL FS0_XL (CTRL1_XL)
  {  2, 0b00 }, 
  {  4, 0b10 }, 
  {  8, 0b11 }, 
  { 16, 0b01 }
};
std::map< uint16_t, uint8_t > mapGyroRangeToFSG = { // FS1_G FS0_G FS_125 (CTRL2_G)
  {  125, 0b001 }, 
  {  250, 0b000 }, 
  {  500, 0b010 }, 
  { 1000, 0b100 }, 
  { 2000, 0b110 }
};

LSM6DSOXClass::LSM6DSOXClass(TwoWire& wire, uint8_t slaveAddress, bool use400kHz) :
  fifo(this),
  _wire(&wire),
  _spi(NULL),
  _slaveAddress(slaveAddress),
  _use400kHz(use400kHz)
{
  initializeSettings();
}

LSM6DSOXClass::LSM6DSOXClass(SPIClass& spi, int csPin, int irqPin) :
  fifo(this),
  _wire(NULL),
  _spi(&spi),
  _csPin(csPin),
  _irqPin(irqPin),
  _spiSettings(10E6, MSBFIRST, SPI_MODE0)
{
  initializeSettings();
}

LSM6DSOXClass::~LSM6DSOXClass()
{
}

void LSM6DSOXClass::initializeSettings(uint16_t sampleRate, uint16_t gyroRange, uint8_t accelRange)
{
  settings.sampleRate = sampleRate;
	settings.gyroRange = gyroRange;
	settings.accelRange = accelRange;
}

uint8_t LSM6DSOXClass::getODRbits() {
  uint8_t odr = 0b0100; // Default 104Hz
  if(mapSampleRateToODR.count(settings.sampleRate) > 0) {
    odr = mapSampleRateToODR[settings.sampleRate];
  }
  return odr;
}

int LSM6DSOXClass::begin()
{
  if (_spi != NULL) {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    _spi->begin();
  } else {
    _wire->begin();
    if (_use400kHz) {
      _wire->setClock( 400000UL);
    }
  }

  if (!(readRegister(LSM6DSOX_WHO_AM_I_REG) == 0x6C || readRegister(LSM6DSOX_WHO_AM_I_REG) == 0x69)) {
    end();
    return 0;
  }

  reset();

  // ODR
  uint8_t odr = getODRbits();

  // FS XL
  uint8_t fs_xl = 0b00; // Default 2g
  if(mapAccelRangeToFSXL.count(settings.accelRange) > 0) {
    fs_xl = mapAccelRangeToFSXL[settings.accelRange];
  }

  // FS G
  uint8_t fs_g = 0b100; // Default 1000 deg/s
  if(mapGyroRangeToFSG.count(settings.gyroRange) > 0) {
    fs_g = mapGyroRangeToFSG[settings.gyroRange];
  }

  // set the Accelerometer control register:
  // - odr from settings
  // - range from settings
  // - LPF2 disabled
  uint8_t ctrl1_xl = (odr << 4) | (fs_xl << 2) | 0x00;
  writeRegister(LSM6DSOX_CTRL1_XL, ctrl1_xl);

  // set the Gyroscope control register:
  // - odr from settings
  // - range from settings
  uint8_t ctrl2_xl = (odr << 4) | (fs_g << 1);
  writeRegister(LSM6DSOX_CTRL2_G, ctrl2_xl);

  // leave CTRL3_C as it is
  // leave CTRL4_C as it is
  // leave CTRL5_C as it is

  // set accelerometer power mode to high performance and LPF1 cutoff as high as possible
  writeRegister(LSM6DSOX_CTRL6_C, 0x03);

  // set gyroscope power mode to high performance and HPF cutoff to 16 mMHz
  writeRegister(LSM6DSOX_CTRL7_G, 0x00);

  // Set the ODR config register to ODR/4
  writeRegister(LSM6DSOX_CTRL8_XL, 0x00);

  // leave CTRL9_X as it is

  // Enable timestamp counter
  writeRegister(LSM6DSOX_CTRL10_C, 0x20);

  // Find actual XL and G full scale values
  fullScaleXL = accelerationFullScale();
  fullScaleG = gyroscopeFullScale();

  return 1;
}

void LSM6DSOXClass::end()
{
  // Stop FIFO
  fifo.end();

  if (_spi != NULL) {
    _spi->end();
    digitalWrite(_csPin, LOW);
    pinMode(_csPin, INPUT);
  } else {
    writeRegister(LSM6DSOX_CTRL2_G, 0x00);
    writeRegister(LSM6DSOX_CTRL1_XL, 0x00);
    _wire->end();
  }
}

int LSM6DSOXClass::reset() {
  // Stop FIFO
  fifo.end();

  // Start reset sequence
  writeRegister(LSM6DSOX_CTRL1_XL, 0x00); // Power-down XL
  writeRegister(LSM6DSOX_CTRL2_G, 0x00); // Power-down G
  int ctrl3_c_value = readRegister(LSM6DSOX_CTRL3_C);
  if( ctrl3_c_value < 0 ) {
    return -1;
  }
  ctrl3_c_value |= 0x01; // Set SW_RESET to 1
  writeRegister(LSM6DSOX_CTRL3_C, ctrl3_c_value ); // Initiate the reset
  // Wait for SW_RESET to be reset to 0. This takes ~50us
  while( ctrl3_c_value & 0x01 ) {
    if((ctrl3_c_value = readRegister(LSM6DSOX_CTRL3_C)) < 0) return -1;
  }
  return 0;
}

int LSM6DSOXClass::readAcceleration(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(LSM6DSOX_OUTX_L_XL, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  float scalefactor = fullScaleXL / 32768.0;
  x = data[0] * scalefactor;
  y = data[1] * scalefactor;
  z = data[2] * scalefactor;

  return 1;
}

int LSM6DSOXClass::accelerationAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x01) {
    return 1;
  }

  return 0;
}

float LSM6DSOXClass::accelerationSampleRate()
{
  int ctrl1_xl = readRegister(LSM6DSOX_CTRL1_XL);
  uint8_t odr_xl = (ctrl1_xl & 0xF0) >> 4;
  for(auto &it : mapSampleRateToODR) { 
    if(it.second == odr_xl) { 
      return (float)it.first;
    }
  }
  return NAN;
}

uint8_t LSM6DSOXClass::fs_xl_to_range(uint8_t fs_xl)
{
  for(auto &it : mapAccelRangeToFSXL) { 
    if(it.second == fs_xl) { 
      return it.first;
    }
  }
  return 0;
}

uint8_t LSM6DSOXClass::accelerationFullScale()
{
  int ctrl1_xl = readRegister(LSM6DSOX_CTRL1_XL);
  uint8_t fs_xl = (ctrl1_xl & 0x0C) >> 2;
  return fs_xl_to_range(fs_xl);
}

int LSM6DSOXClass::setAccelerationFullScale(uint8_t range) // 2/4/8/16
{
  // FS XL
  uint8_t fs_xl = 0b00; // Default 2g
  if(mapAccelRangeToFSXL.count(range) > 0) {
    fs_xl = mapAccelRangeToFSXL[range];
  } else {
    return -2; // Invalid range specified
  }

  // Modify FS0_XL and FS1_XL
  int ctrl1_xl = readRegister(LSM6DSOX_CTRL1_XL) & 0xF3;
  int write_result = writeRegister(LSM6DSOX_CTRL1_XL, ((uint8_t)ctrl1_xl) | (fs_xl << 2));
  if(write_result == 1) {
    fullScaleXL = accelerationFullScale();
  }
  return write_result;
}

int LSM6DSOXClass::readGyroscope(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(LSM6DSOX_OUTX_L_G, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  float scalefactor = fullScaleG / 32768.0;
  x = data[0] * scalefactor;
  y = data[1] * scalefactor;
  z = data[2] * scalefactor;

  return 1;
}

int LSM6DSOXClass::gyroscopeAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x02) {
    return 1;
  }

  return 0;
}

float LSM6DSOXClass::gyroscopeSampleRate()
{
  int ctrl2_g = readRegister(LSM6DSOX_CTRL2_G);
  uint8_t odr_g = (ctrl2_g & 0xF0) >> 4;
  for(auto &it : mapSampleRateToODR) { 
    if(it.second == odr_g) { 
      return (float)it.first;
    }
  }
  return NAN;
}

uint16_t LSM6DSOXClass::fs_g_to_range(uint8_t fs_g)
{
  for(auto &it : mapGyroRangeToFSG) { 
    if(it.second == fs_g) { 
      return it.first;
    }
  }
  return 0;
}

uint16_t LSM6DSOXClass::gyroscopeFullScale()
{
  int ctrl2_g = readRegister(LSM6DSOX_CTRL2_G);
  uint8_t fs_g = (ctrl2_g & 0x0E) >> 1;
  return fs_g_to_range(fs_g);
}

int LSM6DSOXClass::setGyroscopeFullScale(uint8_t range) // 125/250/500/1000/2000
{
  // FS G
  uint8_t fs_g = 0b100; // Default 1000 deg/s
  if(mapGyroRangeToFSG.count(range) > 0) {
    fs_g = mapGyroRangeToFSG[range];
  }

  // Modify FS_0_G, FS_1_G and FS_125
  int ctrl2_g = readRegister(LSM6DSOX_CTRL2_G) & 0xF1;
  int write_result = writeRegister(LSM6DSOX_CTRL2_G, ((uint8_t)ctrl2_g) | (fs_g << 1));
  if(write_result == 1) {
    fullScaleG = gyroscopeFullScale();
  }
  return write_result;
}

int LSM6DSOXClass::readTemperature(int& temperature_deg)
{
  float temperature_float = 0;
  readTemperatureFloat(temperature_float);

  temperature_deg = static_cast<int>(temperature_float);

  return 1;
}

int LSM6DSOXClass::readTemperatureFloat(float& temperature_deg)
{
  /* Read the raw temperature from the sensor. */
  int16_t temperature_raw = 0;

  if (readRegisters(LSM6DSOX_OUT_TEMP_L, reinterpret_cast<uint8_t*>(&temperature_raw), sizeof(temperature_raw)) != 1) {
    return 0;
  }

  temperature_deg = temperatureToCelsius(temperature_raw);

  return 1;
}

float LSM6DSOXClass::temperatureToCelsius(int16_t temperature_raw) {
  /* Convert to °C. */
  static int const TEMPERATURE_LSB_per_DEG = 256;
  static int const TEMPERATURE_OFFSET_DEG = 25;

  return (static_cast<float>(temperature_raw) / TEMPERATURE_LSB_per_DEG) + TEMPERATURE_OFFSET_DEG;
}

int LSM6DSOXClass::temperatureAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x04) {
    return 1;
  }

  return 0;
}

int LSM6DSOXClass::readTimestamp(uint32_t& timestamp) {
  uint8_t buffer[4];
  int result = readRegisters(LSM6DSOX_TIMESTAMP0, buffer, 4);
  if( result == 1 ) {
    timestamp = (((uint32_t)buffer[3]) << 24) | 
                (((uint32_t)buffer[2]) << 16) |
                (((uint32_t)buffer[1]) <<  8) |
                 ((uint32_t)buffer[0]);
  }
  return result;
}
    
int LSM6DSOXClass::readInternalFrequency(int8_t& freq_fine) {
  int result = readRegister(LSM6DSOX_INTERNAL_FREQ_FINE);
  if( result >= 0 ) {
    freq_fine = (int8_t)result;
    return 0;
  }
  return result;
}

int LSM6DSOXClass::readTimestampDouble(double& timestamp) {
  uint32_t t;
  int result = readTimestamp(t);
  if( result == 1 ) {
    // See AN5272, par 6.4
    int8_t freq_fine;
    result = readInternalFrequency(freq_fine);
    if( result >= 0 ) {
      timestamp = correctTimestamp(t, freq_fine);
    } else {
      return -1;
    }
  }
  return result;
}

int LSM6DSOXClass::resetTimestamp() {
  // See AN5272, par. 6.4
  return writeRegister(LSM6DSOX_TIMESTAMP2, 0xAA);
}

int LSM6DSOXClass::readRegister(uint8_t address)
{
  uint8_t value;
  
  if (readRegisters(address, &value, sizeof(value)) != 1) {
    return -1;
  }
  
  return value;
}

int LSM6DSOXClass::setPosSelfTestXL() {
  return setSelfTestReg(0xFC, 0x01);
}
int LSM6DSOXClass::setNegSelfTestXL() {
  return setSelfTestReg(0xFC, 0x10);
}
int LSM6DSOXClass::resetSelfTestXL() {
  return setSelfTestReg(0xFC, 0x00);
}
int LSM6DSOXClass::setPosSelfTestG() {
  return setSelfTestReg(0xF3, 0x01);
}
int LSM6DSOXClass::setNegSelfTestG() {
  return setSelfTestReg(0xF3, 0x11);
}
int LSM6DSOXClass::resetSelfTestG() {
  return setSelfTestReg(0xF3, 0x00);
}
int LSM6DSOXClass::setSelfTestReg(uint8_t mask, uint8_t config) {
  int result = -1;
  int ctrl5_c = readRegister(LSM6DSOX_CTRL5_C);
  if(ctrl5_c >= 0) {
    ctrl5_c = (ctrl5_c & mask) | config;
    result = writeRegister(LSM6DSOX_CTRL5_C, (uint8_t)ctrl5_c);
  }
  return result;
}

int LSM6DSOXClass::readRegisters(uint8_t address, uint8_t* data, size_t length)
{
  uint8_t result = 0;
  if (_spi != NULL) {
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    _spi->transfer(0x80 | address);
    _spi->transfer(data, length);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
    result = 1;
  } else {
    uint8_t retries = 0;
    while((result != 1) && (retries++ < i2C_RETRIES)) {
      _wire->beginTransmission(_slaveAddress);
      _wire->write(address);
      // Don't send stop bit to prevent other master from seizing
      // the bus here
      result = _wire->endTransmission(false);
      if(result == 0) {
        size_t l = _wire->requestFrom(_slaveAddress, length);
        if(l == length) {
          for (size_t i = 0; i < length; i++) {
            *data++ = _wire->read();
          }
          result = 1;
        } else {
          Serial.println("readRegisters:requestFrom length = "+String(l)+" @ addr="+String(address));
          result = 0;
        }
      } else {
        Serial.println("readRegisters:endTransmission error = "+String(result)+" @ addr="+String(address)+" length "+String(length));
        result = -1;
      }
    } // END while((result != 1) && (retries++ < i2C_RETRIES))
  }
  return result;
}

int LSM6DSOXClass::writeRegister(uint8_t address, uint8_t value)
{
  if (_spi != NULL) {
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    _spi->transfer(address);
    _spi->transfer(value);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
  } else {
    uint8_t retries = 0;
    uint8_t result = 2;
    while((result != 0) && (retries++ < i2C_RETRIES)) {
      _wire->beginTransmission(_slaveAddress);
      _wire->write(address);
      _wire->write(value);
      if((result = _wire->endTransmission()) == 0) {
        return 1;
      } else { 
        // result != 0 means an error:
        // 1: data too long to fit in transmit buffer
        // 2: received NACK on transmit of address
        // 3: received NACK on transmit of data
        // 4: other error
        // 5: timeout
        Serial.println("writeRegister:endTransmission error = "+String(result)+" @ addr="+String(address)+"="+String(value));
      }
    }
    return 0; // Still no sucess after multiple retries
  }
  return 1;
}

#ifdef LSM6DS_DEFAULT_SPI
LSM6DSOXClass IMU_LSM6DSOX(LSM6DS_DEFAULT_SPI, PIN_SPI_SS1, LSM6DS_INT);
#else
LSM6DSOXClass IMU_LSM6DSOX(Wire, LSM6DSOX_ADDRESS, true);
#endif