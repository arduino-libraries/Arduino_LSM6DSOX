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

class LSM6DSOXClass {
  public:
    LSM6DSOXClass(TwoWire& wire, uint8_t slaveAddress);
    LSM6DSOXClass(SPIClass& spi, int csPin, int irqPin);
    ~LSM6DSOXClass();

    int begin();
    void end();

    // Accelerometer
    int readAcceleration(float& x, float& y, float& z); // Results are in g (earth gravity).
    float accelerationSampleRate(); // Sampling rate of the sensor.
    int accelerationAvailable(); // Check for available data from accelerometer

    // Gyroscope
    int readGyroscope(float& x, float& y, float& z); // Results are in degrees/second.
    float gyroscopeSampleRate(); // Sampling rate of the sensor.
    int gyroscopeAvailable(); // Check for available data from gyroscope

    // Temperature
    int readTemperature(int& temperature_deg);
    int readTemperatureFloat(float& temperature_deg);
    int temperatureAvailable();

  private:
    int readRegister(uint8_t address);
    int readRegisters(uint8_t address, uint8_t* data, size_t length);
    int writeRegister(uint8_t address, uint8_t value);


  private:
    TwoWire* _wire;
    SPIClass* _spi;
    uint8_t _slaveAddress;
    int _csPin;
    int _irqPin;

    SPISettings _spiSettings;
};

extern LSM6DSOXClass IMU_LSM6DSOX;
#undef IMU
#define IMU IMU_LSM6DSOX