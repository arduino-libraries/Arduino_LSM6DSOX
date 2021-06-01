/*
  Extended by Tore Leikanger, 1st of June 2021, to include control over
  the sample rate and full scale of the accelerometer and gyroscope 
  sensors.
  
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

#define LSM6DSOX_ADDRESS            0x6A

#define LSM6DSOX_WHO_AM_I_REG       0X0F
#define LSM6DSOX_CTRL1_XL           0X10
#define LSM6DSOX_CTRL2_G            0X11

#define LSM6DSOX_STATUS_REG         0X1E

#define LSM6DSOX_CTRL6_C            0X15
#define LSM6DSOX_CTRL7_G            0X16
#define LSM6DSOX_CTRL8_XL           0X17

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


LSM6DSOXClass::LSM6DSOXClass(TwoWire& wire, uint8_t slaveAddress) :
  _wire(&wire),
  _spi(NULL),
  _slaveAddress(slaveAddress)
{
}

LSM6DSOXClass::LSM6DSOXClass(SPIClass& spi, int csPin, int irqPin) :
  _wire(NULL),
  _spi(&spi),
  _csPin(csPin),
  _irqPin(irqPin),
  _spiSettings(10E6, MSBFIRST, SPI_MODE0)
{
}

LSM6DSOXClass::~LSM6DSOXClass()
{
}

int LSM6DSOXClass::begin()
{
  if (_spi != NULL) {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    _spi->begin();
  } else {
    _wire->begin();
  }

  if (readRegister(LSM6DSOX_WHO_AM_I_REG) != 0x6C) {
    end();
    return 0;
  }

  //set the gyroscope control register to work at 104 Hz, 2000 dps and in bypass mode
  writeRegister(LSM6DSOX_CTRL2_G, _CTRL2_G);

  // Set the Accelerometer control register to work at 104 Hz, 4 g,and in bypass mode and enable ODR/4
  // low pass filter (check figure9 of LSM6DSOX's datasheet)
  writeRegister(LSM6DSOX_CTRL1_XL, _CTRL1_XL);

  // set gyroscope power mode to high performance and bandwidth to 16 MHz
  writeRegister(LSM6DSOX_CTRL7_G, _CTRL7_G);

  // Set the ODR config register to ODR/4
  writeRegister(LSM6DSOX_CTRL8_XL, _CTRL8_XL);

  return 1;
}

void LSM6DSOXClass::end()
{
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

int LSM6DSOXClass::readAcceleration(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(LSM6DSOX_OUTX_L_XL, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0] * 4.0 / 32768.0;
  y = data[1] * 4.0 / 32768.0;
  z = data[2] * 4.0 / 32768.0;

  return 1;
}

int LSM6DSOXClass::accelerationAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x01) {
    return 1;
  }

  return 0;
}

float LSM6DSOXClass::sampleRate(uint8_t cfg)
{
  switch ( cfg ) 
  {
    case LSM6DSOX_ODR_OFF :
      return 0.0F;
    case LSM6DSOX_ODR_12_5_HZ:
      return 12.5F;
    case LSM6DSOX_ODR_26_HZ:
      return 26.0F;
    case LSM6DSOX_ODR_52_HZ:
      return 52.0F;
    case LSM6DSOX_ODR_104_HZ:
      return 104.0F;
    case LSM6DSOX_ODR_208_HZ:
      return 208.0F;
    case LSM6DSOX_ODR_416_HZ:
      return 416.0F;
    case LSM6DSOX_ODR_833_HZ:
      return 833.0F;
    case LSM6DSOX_ODR_1K66_HZ:
      return 1660.0F;
    case LSM6DSOX_ODR_3K33_HZ:
      return 3330.0F;
    case LSM6DSOX_ODR_6K66_HZ:
      return 6660.0F;
    default:
      return -1.0F;
  }
}

float LSM6DSOXClass::accelerationSampleRate()
{
  return sampleRate(_CTRL1_XL>>4);
}

float LSM6DSOXClass::accelerationFullScale()
{
  switch (_CTRL1_XL>>2) 
  {
    case LSM6DSOX_ACCEL_FS_2G:
      return 2.0F;
    case LSM6DSOX_ACCEL_FS_4G:
      return 4.0F;
    case LSM6DSOX_ACCEL_FS_8G:
      return 8.0F;
    case LSM6DSOX_ACCEL_FS_16G:
      return 16.0F;
    default:
      return -1.0F;
  }
}

int LSM6DSOXClass::setAccelerationSampleRate(uint8_t rate)
{
  _CTRL1_XL &= 0x0F;
  _CTRL1_XL |= rate<<4;
  return writeRegister(LSM6DSOX_CTRL1_XL, _CTRL1_XL);
}

int LSM6DSOXClass::setAccelerationFullScale(uint8_t scale)
{

  _CTRL1_XL &= ~0x0C;
  _CTRL1_XL |= scale<<2;

  if ( scale == LSM6DSOX_ACCEL_FS_16G ){
    _CTRL8_XL &= ~0x02;
  }
  else {
    _CTRL8_XL |= 0x02;
  }

  if ( writeRegister(LSM6DSOX_CTRL8_XL, _CTRL8_XL) == 0 ) {
    return 0;
  };

  return writeRegister(LSM6DSOX_CTRL1_XL, _CTRL1_XL);
}

int LSM6DSOXClass::setAccelerationConfig(uint8_t rate, uint8_t scale)
{
  _CTRL1_XL &= 0x09;
  _CTRL1_XL |= rate<<4;
  _CTRL1_XL |= scale<<2;

  if ( scale == LSM6DSOX_ACCEL_FS_16G ){
    _CTRL8_XL &= ~0x02;
  }
  else {
    _CTRL8_XL |= 0x02;
  }

  if ( writeRegister(LSM6DSOX_CTRL8_XL, _CTRL8_XL) == 0 ) {
    return 0;
  }

  return writeRegister(LSM6DSOX_CTRL1_XL, _CTRL1_XL);
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

  x = data[0] * 2000.0 / 32768.0;
  y = data[1] * 2000.0 / 32768.0;
  z = data[2] * 2000.0 / 32768.0;

  return 1;
}

float LSM6DSOXClass::gyroscopeSampleRate()
{
  return sampleRate(_CTRL2_G>>4);
}

float LSM6DSOXClass::gyroscopeFullScale()
{
  switch (_CTRL2_G>>1) 
  {
    case LSM6DSOX_GYRO_FS_125DPS:
      return 125.0F;
    case LSM6DSOX_GYRO_FS_250DPS:
      return 250.0F;
    case LSM6DSOX_GYRO_FS_500DPS:
      return 500.0F;
    case LSM6DSOX_GYRO_FS_1000DPS:
      return 1000.0F;
    case LSM6DSOX_GYRO_FS_2000DPS:
      return 2000.0F;
    default:
      return -1.0F;
  }
}

int LSM6DSOXClass::setGyroscopeSampleRate(uint8_t rate)
{
  _CTRL2_G &= 0x0F;
  _CTRL2_G |= rate<<4;
  return writeRegister(LSM6DSOX_CTRL2_G, _CTRL2_G);
}

int LSM6DSOXClass::setGyroscopeFullScale(uint8_t scale)
{

  _CTRL2_G &= ~0x0E;
  _CTRL2_G |= scale<<1;

  return writeRegister(LSM6DSOX_CTRL2_G, _CTRL2_G);
}

int LSM6DSOXClass::setGyroscopeConfig(uint8_t rate, uint8_t scale)
{
  _CTRL2_G &= 0x01;
  _CTRL2_G |= rate<<4;
  _CTRL2_G |= scale<<1;

  return writeRegister(LSM6DSOX_CTRL2_G, _CTRL2_G);
}

int LSM6DSOXClass::gyroscopeAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x02) {
    return 1;
  }

  return 0;
}

int LSM6DSOXClass::readRegister(uint8_t address)
{
  uint8_t value;
  
  if (readRegisters(address, &value, sizeof(value)) != 1) {
    return -1;
  }
  
  return value;
}

int LSM6DSOXClass::readRegisters(uint8_t address, uint8_t* data, size_t length)
{
  if (_spi != NULL) {
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    _spi->transfer(0x80 | address);
    _spi->transfer(data, length);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
  } else {
    _wire->beginTransmission(_slaveAddress);
    _wire->write(address);

    if (_wire->endTransmission(false) != 0) {
      return -1;
    }

    if (_wire->requestFrom(_slaveAddress, length) != length) {
      return 0;
    }

    for (size_t i = 0; i < length; i++) {
      *data++ = _wire->read();
    }
  }
  return 1;
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
    _wire->beginTransmission(_slaveAddress);
    _wire->write(address);
    _wire->write(value);
    if (_wire->endTransmission() != 0) {
      return 0;
    }
  }
  return 1;
}

LSM6DSOXClass IMU(Wire, LSM6DSOX_ADDRESS);
