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
    /**
     * @brief Construct a new LSM6DSOXClass object using I2C.
     * 
     * @param wire The I2C wire instance.
     * @param slaveAddress The I2C slave address of the IMU.
     */
    LSM6DSOXClass(TwoWire& wire, uint8_t slaveAddress);

    /**
     * @brief Construct a new LSM6DSOXClass object using SPI.
     * 
     * @param spi The SPI instance.
     * @param csPin The Chip Select pin.
     * @param irqPin The Interrupt pin.
     */
    LSM6DSOXClass(SPIClass& spi, int csPin, int irqPin);

    /**
     * @brief Destroy the LSM6DSOXClass object.
     */
    ~LSM6DSOXClass();

    /**
     * @brief Initialize the IMU.
     * 
     * @return 1 on success, 0 on failure.
     */
    int begin();

    /**
     * @brief De-initialize the IMU.
     * 
     * @return None.
     */
    void end();

    // Accelerometer
    /**
     * @brief Query the IMU's accelerometer and return the acceleration in g's.
     * 
     * The accelerometer is configured with a scale of [-4, +4] g.
     * 
     * @param x float variable where the acceleration value in the IMU's x-axis will be stored.
     * @param y float variable where the acceleration value in the IMU's y-axis will be stored.
     * @param z float variable where the acceleration value in the IMU's z-axis will be stored.
     * @return 1 on success, 0 on failure.
     */
    int readAcceleration(float& x, float& y, float& z);

    /**
     * @brief Return the IMU's accelerometer sample rate.
     * 
     * @return The IMU's accelerometer sample rate in Hz.
     */
    float accelerationSampleRate();

    /**
     * @brief Query if new acceleration data from the IMU is available.
     * 
     * @return 0 if no new acceleration data is available, 1 if new acceleration data is available.
     */
    int accelerationAvailable();

    // Gyroscope
    /**
     * @brief Query the IMU's gyroscope and return the angular speed in dps (degrees per second).
     * 
     * The gyroscope is configured with a scale of [-2000, +2000] dps.
     * 
     * @param x float variable where the gyroscope value in the IMU's x-axis will be stored.
     * @param y float variable where the gyroscope value in the IMU's y-axis will be stored.
     * @param z float variable where the gyroscope value in the IMU's z-axis will be stored.
     * @return 1 on success, 0 on failure.
     */
    int readGyroscope(float& x, float& y, float& z);

    /**
     * @brief Return the IMU's gyroscope sample rate.
     * 
     * @return The IMU's gyroscope sample rate in Hz.
     */
    float gyroscopeSampleRate();

    /**
     * @brief Query if new gyroscope data from the IMU is available.
     * 
     * @return 0 if no new gyroscope data is available, 1 if new gyroscope data is available.
     */
    int gyroscopeAvailable();

    // Temperature
    /**
     * @brief Reads the temperature from the sensor (Celsius).
     * 
     * @param temperature_deg int variable where the temperature value will be stored.
     * @return None.
     */
    int readTemperature(int& temperature_deg);

    /**
     * @brief Reads the temperature from the sensor (Celsius).
     * 
     * @param temperature_deg float variable where the temperature value will be stored.
     * @return None.
     */
    int readTemperatureFloat(float& temperature_deg);

    /**
     * @brief Checks if temperature data is available.
     * 
     * @return 0 if no new temperature data is available, 1 if new temperature data is available.
     */
    int temperatureAvailable();

  private:
    /**
     * @brief Read a single byte from a register.
     * 
     * @param address The register address.
     * @return The value of the register, or -1 on failure.
     */
    int readRegister(uint8_t address);

    /**
     * @brief Read multiple bytes from a register.
     * 
     * @param address The starting register address.
     * @param data Pointer to a buffer to store the read data.
     * @param length The number of bytes to read.
     * @return 1 on success, 0 when the data could not be read,
                -1 when the device was not found at the specified address.
     */
    int readRegisters(uint8_t address, uint8_t* data, size_t length);

    /**
     * @brief Write a single byte to a register.
     * 
     * @param address The register address.
     * @param value The value to write to the register.
     * @return 1 on success, 0 on failure.
     */
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