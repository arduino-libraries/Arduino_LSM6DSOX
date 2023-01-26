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

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "LSM6DSOXFIFO.h"

struct SensorSettings {
public:
  uint16_t sampleRate;
	uint16_t gyroRange;
	uint8_t accelRange;
};

class LSM6DSOXClass {
  friend class LSM6DSOXFIFOClass;
  public:
    //IMU settings
    SensorSettings settings;

    //FIFO
    LSM6DSOXFIFOClass fifo;

    LSM6DSOXClass(TwoWire& wire, uint8_t slaveAddress, bool use400kHz = false);
    LSM6DSOXClass(SPIClass& spi, int csPin, int irqPin);
    ~LSM6DSOXClass();

    void initializeSettings(uint16_t sampleRate = 416, uint16_t gyroRange = 1000, uint8_t accelRange = 8);
    
    int begin();
    void end();
    int reset();

    // Accelerometer
    int readAcceleration(float& x, float& y, float& z); // Results are in g (earth gravity).
    float accelerationSampleRate(); // Sampling rate of the sensor.
    int accelerationAvailable(); // Check for available data from accelerometer
    uint8_t accelerationFullScale(); // Retrieve current accelerometer full scale setting
    int setAccelerationFullScale(uint8_t range);

    // Gyroscope
    int readGyroscope(float& x, float& y, float& z); // Results are in degrees/second.
    float gyroscopeSampleRate(); // Sampling rate of the sensor.
    int gyroscopeAvailable(); // Check for available data from gyroscope
    uint16_t gyroscopeFullScale(); // Retrieve current gyroscope full scale setting
    int setGyroscopeFullScale(uint8_t range);
  
    // Temperature
    int readTemperature(int& temperature_deg);
    int readTemperatureFloat(float& temperature_deg);
    int temperatureAvailable();

    // Timestamp
    int readTimestamp(uint32_t& timestamp);
    int readTimestampDouble(double& timestamp);
    int resetTimestamp();

    // Self-test
    int setPosSelfTestXL();
    int setNegSelfTestXL();
    int resetSelfTestXL();
    int setPosSelfTestG();
    int setNegSelfTestG();
    int resetSelfTestG();
    
private:  
    uint8_t getODRbits();

    uint8_t fs_xl_to_range(uint8_t fs_xl);
    uint16_t fs_g_to_range(uint8_t fs_g);
    float temperatureToCelsius(int16_t temperature_raw);

    int readInternalFrequency(int8_t& freq_fine);
    double correctTimestamp(uint32_t timestamp, int8_t freq_fine) { 
      return timestamp / (40000 * (1 + 0.0015 * freq_fine));  // See AN5272, par 6.4
    }

    int setSelfTestReg(uint8_t mask, uint8_t config);

    int readRegister(uint8_t address);
    int readRegisters(uint8_t address, uint8_t* data, size_t length);
    int writeRegister(uint8_t address, uint8_t value);

  private:
    TwoWire* _wire;
    SPIClass* _spi;
    uint8_t _slaveAddress;
    int _csPin;
    int _irqPin;
    bool _use400kHz;

    SPISettings _spiSettings;
};

extern LSM6DSOXClass IMU_LSM6DSOX;
#undef IMU
#define IMU IMU_LSM6DSOX