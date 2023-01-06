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

#ifndef LSM6DSOXFIFO_H
#define LSM6DSOXFIFO_H

#include <Arduino.h>

struct FIFOSettings {
public:
  uint16_t watermark_level;       //9 bits (0-511)
  uint8_t timestamp_decimation;   //0/1/8/32
  uint8_t temperature_frequency;  //0/2/13/52
  uint16_t counter_threshold;     //11 bits (0-2047)
  bool counter_gyro;              //false = count XL, true = count G
  uint8_t fifo_mode;              //3-bit pattern, see datasheet
};

class LSM6DSOXClass;

class LSM6DSOXFIFOClass {
  public:
    LSM6DSOXFIFOClass(LSM6DSOXClass* imu);
    ~LSM6DSOXFIFOClass();

    void initializeSettings(
      uint16_t watermark_level = 0,
      uint8_t timestamp_decimation = 8,
      uint8_t temperature_frequency = 2,  //1.6Hz
      uint16_t counter_threshold = 9,
      bool counter_gyro = true,           //Use G count (rather than XL)
      uint8_t fifo_mode = 6);             //Continuous mode

    void begin();
    void end();

  private:
    FIFOSettings    settings;
    LSM6DSOXClass*  imu;
};

#endif
