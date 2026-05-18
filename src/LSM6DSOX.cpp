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

#define LSM6DSOX_ADDRESS            0x6A

#define LSM6DSOX_WHO_AM_I_REG       0X0F
#define LSM6DSOX_CTRL1_XL           0X10
#define LSM6DSOX_CTRL2_G            0X11

#define LSM6DSOX_STATUS_REG         0X1E

#define LSM6DSOX_CTRL6_C            0X15
#define LSM6DSOX_CTRL7_G            0X16
#define LSM6DSOX_CTRL8_XL           0X17

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

#define LSM6DSOX_FUNC_CFG_ACCESS    0X01
#define LSM6DSOX_MD1_CFG            0X5E
#define LSM6DSOX_MD2_CFG            0X5F

#define LSM6DSOX_PAGE_SEL           0X02
#define LSM6DSOX_PAGE_ADDRESS       0X08
#define LSM6DSOX_PAGE_VALUE         0X09
#define LSM6DSOX_PAGE_RW            0X17

#define LSM6DSOX_EMB_FUNC_EN_A      0X04
#define LSM6DSOX_EMB_FUNC_EN_B      0X05
#define LSM6DSOX_EMB_FUNC_INT1      0X0A
#define LSM6DSOX_EMB_FUNC_INT2      0X0E
#define LSM6DSOX_EMB_FUNC_SRC       0X64
#define LSM6DSOX_STEP_COUNTER_L     0X62

#define LSM6DSOX_PEDO_DEB_STEPS_CONF 0x0184

#define LSM6DSOX_PEDO_EN_MASK       0x08
#define LSM6DSOX_PEDO_RST_STEP_MASK 0x80
#define LSM6DSOX_PEDO_INT_MASK      0x08
#define LSM6DSOX_INT_EMB_FUNC_MASK  0x02

#define LSM6DSOX_FUNC_CFG_BANK_USER  0
#define LSM6DSOX_FUNC_CFG_BANK_EMBED 1


namespace {
  constexpr uint8_t MEMORY_BANK_MASK = 0xC0;
  constexpr uint8_t MEMORY_BANK_SHIFT = 6;

  constexpr uint8_t PAGE_MSB_MASK = 0x0F;
  constexpr uint8_t PAGE_LSB_MASK = 0xFF;
  constexpr uint8_t PAGE_SHIFT = 4;
  constexpr uint8_t PAGE_ENABLE_BIT = 0x01;

  constexpr uint8_t PAGE_WRITE_BIT = 0x40;
  constexpr uint8_t PAGE_READ_BIT = 0x20;
  constexpr uint8_t PAGE_RW_MASK = 0x60;
}

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

  if (!(readRegister(LSM6DSOX_WHO_AM_I_REG) == 0x6C || readRegister(LSM6DSOX_WHO_AM_I_REG) == 0x69)) {
    end();
    return 0;
  }

  //set the gyroscope control register to work at 104 Hz, 2000 dps and in bypass mode
  writeRegister(LSM6DSOX_CTRL2_G, 0x4C);

  // Set the Accelerometer control register to work at 104 Hz, 4 g,and in bypass mode and enable ODR/4
  // low pass filter (check figure9 of LSM6DSOX's datasheet)
  writeRegister(LSM6DSOX_CTRL1_XL, 0x4A);

  // set gyroscope power mode to high performance and bandwidth to 16 MHz
  writeRegister(LSM6DSOX_CTRL7_G, 0x00);

  // Set the ODR config register to ODR/4
  writeRegister(LSM6DSOX_CTRL8_XL, 0x09);

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

  if (readRegisters(LSM6DSOX_OUTX_L_XL, (uint8_t*)data, sizeof(data)) != 1) {
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

float LSM6DSOXClass::accelerationSampleRate()
{
  return 104.0F;
}

int LSM6DSOXClass::readGyroscope(float& x, float& y, float& z)
{
  int16_t data[3];

  if (readRegisters(LSM6DSOX_OUTX_L_G, (uint8_t*)data, sizeof(data)) != 1) {
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

int LSM6DSOXClass::gyroscopeAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x02) {
    return 1;
  }

  return 0;
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

  /* Convert to °C. */
  static int const TEMPERATURE_LSB_per_DEG = 256;
  static int const TEMPERATURE_OFFSET_DEG = 25;

  temperature_deg = (static_cast<float>(temperature_raw) / TEMPERATURE_LSB_per_DEG) + TEMPERATURE_OFFSET_DEG;

  return 1;
}

int LSM6DSOXClass::temperatureAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x04) {
    return 1;
  }

  return 0;
}

float LSM6DSOXClass::gyroscopeSampleRate()
{
  return 104.0F;
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

bool LSM6DSOXClass::modifyRegisterBits(uint8_t address, uint8_t clearMask, uint8_t setMask) {
  int value = readRegister(address);
  if (value < 0) {
    return false;
  }
  return writeRegister(address, (value & ~clearMask) | setMask) == 1;
}

bool LSM6DSOXClass::setMemoryBank(uint8_t bank) {
  return modifyRegisterBits(LSM6DSOX_FUNC_CFG_ACCESS, MEMORY_BANK_MASK, bank << MEMORY_BANK_SHIFT);
}

bool LSM6DSOXClass::selectPage(uint16_t address, bool write, uint8_t* value) {
  uint8_t msb = (address >> 8) & PAGE_MSB_MASK;
  uint8_t lsb = address & PAGE_LSB_MASK;

  if (!setMemoryBank(LSM6DSOX_FUNC_CFG_BANK_EMBED)) return false;

  uint8_t rwBit = write ? PAGE_WRITE_BIT : PAGE_READ_BIT;
  if (!modifyRegisterBits(LSM6DSOX_PAGE_RW, PAGE_RW_MASK, rwBit)) return false;

  if (!writeRegister(LSM6DSOX_PAGE_SEL, (msb << PAGE_SHIFT) | PAGE_ENABLE_BIT)) return false;
  if (!writeRegister(LSM6DSOX_PAGE_ADDRESS, lsb)) return false;

  if (write) {
    if (!writeRegister(LSM6DSOX_PAGE_VALUE, *value)) return false;
  } else {
    int val = readRegister(LSM6DSOX_PAGE_VALUE);
    if (val < 0) return false;
    *value = val;
  }

  if (!writeRegister(LSM6DSOX_PAGE_SEL, PAGE_ENABLE_BIT)) return false;
  if (!modifyRegisterBits(LSM6DSOX_PAGE_RW, rwBit, 0x00)) return false; // Clear the rw bit

  return setMemoryBank(LSM6DSOX_FUNC_CFG_BANK_USER);
}

bool LSM6DSOXClass::configurePedometer(bool enable, uint8_t debounce, bool int1Enable, bool int2Enable) {
  // Select page to set debounce
  if (!selectPage(LSM6DSOX_PEDO_DEB_STEPS_CONF, true, &debounce)) return false;

  if (int1Enable) {
    if (!modifyRegisterBits(LSM6DSOX_MD1_CFG, 0, LSM6DSOX_INT_EMB_FUNC_MASK)) return false;
  }
  if (int2Enable) {
    if (!modifyRegisterBits(LSM6DSOX_MD2_CFG, 0, LSM6DSOX_INT_EMB_FUNC_MASK)) return false;
  }

  if (!setMemoryBank(LSM6DSOX_FUNC_CFG_BANK_EMBED)) return false;

  if (!modifyRegisterBits(LSM6DSOX_EMB_FUNC_EN_A, LSM6DSOX_PEDO_EN_MASK, enable ? LSM6DSOX_PEDO_EN_MASK : 0)) return false;
  if (!modifyRegisterBits(LSM6DSOX_EMB_FUNC_INT1, LSM6DSOX_PEDO_INT_MASK, int1Enable ? LSM6DSOX_PEDO_INT_MASK : 0)) return false;
  if (!modifyRegisterBits(LSM6DSOX_EMB_FUNC_INT2, LSM6DSOX_PEDO_INT_MASK, int2Enable ? LSM6DSOX_PEDO_INT_MASK : 0)) return false;

  return setMemoryBank(LSM6DSOX_FUNC_CFG_BANK_USER);
}

bool LSM6DSOXClass::resetPedometer() {
  if (!setMemoryBank(LSM6DSOX_FUNC_CFG_BANK_EMBED)) return false;
  if (!modifyRegisterBits(LSM6DSOX_EMB_FUNC_SRC, 0, LSM6DSOX_PEDO_RST_STEP_MASK)) return false;
  return setMemoryBank(LSM6DSOX_FUNC_CFG_BANK_USER);
}

int LSM6DSOXClass::pedometerSteps() {
  if (!setMemoryBank(LSM6DSOX_FUNC_CFG_BANK_EMBED)) return -1;

  uint8_t data[2];
  if (readRegisters(LSM6DSOX_STEP_COUNTER_L, data, sizeof(data)) != 1) return -1;

  if (!setMemoryBank(LSM6DSOX_FUNC_CFG_BANK_USER)) return -1;

  return data[0] | (data[1] << 8);
}

#ifdef LSM6DS_DEFAULT_SPI
LSM6DSOXClass IMU_LSM6DSOX(LSM6DS_DEFAULT_SPI, PIN_SPI_SS1, LSM6DS_INT);
#else
  #if defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_NANO_R4) || defined(ARDUINO_UNO_Q)
  LSM6DSOXClass IMU_LSM6DSOX(Wire1, LSM6DSOX_ADDRESS);
  #else
  LSM6DSOXClass IMU_LSM6DSOX(Wire, LSM6DSOX_ADDRESS);
  #endif
#endif
