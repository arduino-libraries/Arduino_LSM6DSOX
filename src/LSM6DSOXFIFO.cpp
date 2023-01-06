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

#include "LSM6DSOXFIFO.h"
#include "LSM6DSOX.h"

#include <map>

#define LSM6DSOX_ADDRESS            0x6A

#define LSM6DSOX_FIFO_CTRL1         0x07
#define LSM6DSOX_FIFO_CTRL2         0x08
#define LSM6DSOX_FIFO_CTRL3         0x09
#define LSM6DSOX_FIFO_CTRL4         0x0A
#define LSM6DSOX_COUNTER_BDR_REG1   0x0B
#define LSM6DSOX_COUNTER_BDR_REG2   0x0C

std::map< uint8_t, uint8_t > mapTimestampDecimation = { // DEC_TS_BATCH_[1:0]
  {  0, 0b00 }, 
  {  1, 0b01 }, 
  {  8, 0b10 }, 
  { 32, 0b11 }
};
std::map< uint8_t, uint8_t > mapTemperatureODR = { // ODR_T_BATCH_[1:0]
  {  0, 0b00 }, 
  {  2, 0b01 }, 
  { 13, 0b10 }, 
  { 52, 0b11 }
};

LSM6DSOXFIFOClass::LSM6DSOXFIFOClass(LSM6DSOXClass* imu) {
  this->imu = imu;
}

LSM6DSOXFIFOClass::~LSM6DSOXFIFOClass() {
}

void LSM6DSOXFIFOClass::initializeSettings(
  uint16_t watermark_level,
  uint8_t timestamp_decimation,
  uint8_t temperature_frequency,
  uint16_t counter_threshold,
  bool counter_gyro,
  uint8_t fifo_mode) {
    this->settings.watermark_level = watermark_level;
    this->settings.timestamp_decimation = timestamp_decimation;
    this->settings.temperature_frequency = temperature_frequency;
    this->settings.counter_threshold = counter_threshold;    
    this->settings.counter_gyro = counter_gyro;
    this->settings.fifo_mode = fifo_mode;
}

void LSM6DSOXFIFOClass::begin()
{
  uint8_t fifo_ctrl1 = this->settings.watermark_level & 0xFF; // WTM0..8
  uint8_t fifo_ctrl2 = 0b00000000; // Default STOP_ON_WTM=0 and no compression
  fifo_ctrl2 |= (this->settings.watermark_level >> 8) & 0x01; // Put WTM8 into CTRL2 bit 0

  // Set Batching Data Rate for XL and G equal to their ODR
  uint8_t odr = this->imu->getODRbits();
  uint8_t fifo_ctrl3 = (odr << 4) | odr;

  // Retrieve timestamp decimation
  // DEC_TS_BATCH_[1:0]
  uint8_t dec_ts = 0b00; // Default no timestamp
  if(mapTimestampDecimation.count(settings.timestamp_decimation) > 0) {
    dec_ts = mapTimestampDecimation[settings.timestamp_decimation];
  }
  // Retrieve temperature ODR
  // ODR_T_BATCH_[1:0]
  uint8_t odr_t = 0b00; // Default no temperature
  if(mapTemperatureODR.count(settings.temperature_frequency) > 0) {
    odr_t = mapTemperatureODR[settings.temperature_frequency];
  }
  uint8_t fifo_ctrl4 = (dec_ts << 6) | (odr_t << 4) | this->settings.fifo_mode;

  uint8_t counter_bdr_reg1 = (settings.counter_threshold >> 8) & 0x07; // CNT_BDR_TH_8..10
  if(settings.counter_gyro) {
    counter_bdr_reg1 |= 0x20; //Use G count (rather than XL)
  }
  uint8_t counter_bdr_reg2 = settings.counter_threshold  & 0xFF; // CNT_BDR_TH_0..7

  this->imu->writeRegister(LSM6DSOX_FIFO_CTRL1, fifo_ctrl1);
  this->imu->writeRegister(LSM6DSOX_FIFO_CTRL2, fifo_ctrl2);
  this->imu->writeRegister(LSM6DSOX_FIFO_CTRL3, fifo_ctrl3);
  this->imu->writeRegister(LSM6DSOX_FIFO_CTRL4, fifo_ctrl4);
  this->imu->writeRegister(LSM6DSOX_COUNTER_BDR_REG1, counter_bdr_reg1);
  this->imu->writeRegister(LSM6DSOX_COUNTER_BDR_REG2, counter_bdr_reg2);
}

void LSM6DSOXFIFOClass::end()
{
}
