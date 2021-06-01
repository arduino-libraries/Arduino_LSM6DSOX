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

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#define LSM6DSOX_ODR_OFF          0x00
#define LSM6DSOX_ODR_12_5_HZ      0x01
#define LSM6DSOX_ODR_26_HZ        0x02
#define LSM6DSOX_ODR_52_HZ        0x03
#define LSM6DSOX_ODR_104_HZ       0x04
#define LSM6DSOX_ODR_208_HZ       0x05
#define LSM6DSOX_ODR_416_HZ       0x06
#define LSM6DSOX_ODR_833_HZ       0x07
#define LSM6DSOX_ODR_1K66_HZ      0x08
#define LSM6DSOX_ODR_3K33_HZ      0x09
#define LSM6DSOX_ODR_6K66_HZ      0x0A

#define LSM6DSOX_ACCEL_FS_2G      0x00
#define LSM6DSOX_ACCEL_FS_16G     0x01
#define LSM6DSOX_ACCEL_FS_4G      0x02
#define LSM6DSOX_ACCEL_FS_8G      0x03
#define LSM6DSOX_ACCEL_FS_2G      0x04

#define LSM6DSOX_GYRO_FS_125DPS   0x01
#define LSM6DSOX_GYRO_FS_250DPS   0x00
#define LSM6DSOX_GYRO_FS_500DPS   0x02
#define LSM6DSOX_GYRO_FS_1000DPS  0x04
#define LSM6DSOX_GYRO_FS_2000DPS  0x06


class LSM6DSOXClass {
  public:
    LSM6DSOXClass(TwoWire& wire, uint8_t slaveAddress);
    LSM6DSOXClass(SPIClass& spi, int csPin, int irqPin);
    virtual ~LSM6DSOXClass();

    int begin();
    void end();

    virtual float sampleRate(uint8_t cfg);

    // Accelerometer
    virtual int readAcceleration(float& x, float& y, float& z); // Results are in g (earth gravity).
    virtual float accelerationSampleRate(); // Sampling rate of the sensor.
    virtual float accelerationFullScale(); // Full scale of the sensor.
    virtual int setAccelerationSampleRate(uint8_t rate); // Set sampling rate of the sensor.
    virtual int setAccelerationFullScale(uint8_t scale); // Set full scale of the sensor.
    virtual int setAccelerationConfig(uint8_t rate, uint8_t scale); // Set sampling rate and full scale of the sensor
    virtual int accelerationAvailable(); // Check for available data from accelerometer

    // Gyroscope
    virtual int readGyroscope(float& x, float& y, float& z); // Results are in degrees/second.
    virtual float gyroscopeSampleRate(); // Sampling rate of the sensor.
    virtual float gyroscopeFullScale(); // Full scale of the sensor
    virtual int setGyroscopeSampleRate(uint8_t rate); // Set sampling rate of the sensor.
    virtual int setGyroscopeFullScale(uint8_t scale); // Set full scale of the sensor.
    virtual int setGyroscopeConfig(uint8_t rate, uint8_t scale); // Set sampling rate and full scale of the sensor
    virtual int gyroscopeAvailable(); // Check for available data from gyroscope


  private:

    // Set the Accelerometer control register to work at 104 Hz, 4 g,and in bypass mode and enable ODR/4
    // low pass filter (check figure9 of LSM6DSOX's datasheet)
    uint8_t _CTRL1_XL = 0x4A;
    //set the gyroscope control register to work at 104 Hz, 2000 dps and in bypass mode
    uint8_t _CTRL2_G  = 0x4C;
    // set gyroscope power mode to high performance and bandwidth to 16 MHz
    uint8_t _CTRL7_G  = 0x00;
    // Set the ODR config register to ODR/4
    uint8_t _CTRL8_XL = 0x09;

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

extern LSM6DSOXClass IMU;
