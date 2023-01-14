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
#include <algorithm>

#define LSM6DSOX_ADDRESS            0x6A

#define LSM6DSOX_FIFO_CTRL1         0x07
#define LSM6DSOX_FIFO_CTRL2         0x08
#define LSM6DSOX_FIFO_CTRL3         0x09
#define LSM6DSOX_FIFO_CTRL4         0x0A
#define LSM6DSOX_COUNTER_BDR_REG1   0x0B
#define LSM6DSOX_COUNTER_BDR_REG2   0x0C

#define LSM6DSOX_STATUS1            0x3A
#define LSM6DSOX_STATUS2            0x3B

#define LSM6DSOX_FIFO_DATA_OUT_TAG  0x78


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

  initializeSettings();
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
  // Find timestamp correction factor. See AN5272, par 6.4
  int8_t freq_fine;
  imu->readInternalFrequency(freq_fine);
  timestampCorrection = 1.0 / (40000 * (1 + 0.0015 * freq_fine));

  // Find actual XL and G full scale values
  fullScaleXL = imu->accelerationFullScale();
  fullScaleG = imu->gyroscopeFullScale();

  // Now find all configuration values
  uint8_t fifo_ctrl1 = settings.watermark_level & 0xFF; // WTM0..8
  uint8_t fifo_ctrl2 = 0b00000000; // Default STOP_ON_WTM=0 and no compression
  fifo_ctrl2 |= (this->settings.watermark_level >> 8) & 0x01; // Put WTM8 into CTRL2 bit 0

  // Set Batching Data Rate for XL and G equal to their ODR
  uint8_t odr = imu->getODRbits();
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
  uint8_t fifo_ctrl4 = (dec_ts << 6) | (odr_t << 4) | settings.fifo_mode;

  uint8_t counter_bdr_reg1 = (settings.counter_threshold >> 8) & 0x07; // CNT_BDR_TH_8..10
  if(settings.counter_gyro) {
    counter_bdr_reg1 |= 0x20; //Use G count (rather than XL)
  }
  uint8_t counter_bdr_reg2 = settings.counter_threshold  & 0xFF; // CNT_BDR_TH_0..7

  imu->writeRegister(LSM6DSOX_FIFO_CTRL1, fifo_ctrl1);
  imu->writeRegister(LSM6DSOX_FIFO_CTRL2, fifo_ctrl2);
  imu->writeRegister(LSM6DSOX_FIFO_CTRL3, fifo_ctrl3);
  imu->writeRegister(LSM6DSOX_FIFO_CTRL4, fifo_ctrl4);
  imu->writeRegister(LSM6DSOX_COUNTER_BDR_REG1, counter_bdr_reg1);
  imu->writeRegister(LSM6DSOX_COUNTER_BDR_REG2, counter_bdr_reg2);

  // Buffer management
  read_idx = 0;
  write_idx = 0;
  buffer_empty = true;
}

void LSM6DSOXFIFOClass::end()
{
  // Disable XL and G batching
  imu->writeRegister(LSM6DSOX_FIFO_CTRL3, 0x00);

  // Disable timestamp and temperature batching, and set FIFO mode=0 (FIFO disabled)
  imu->writeRegister(LSM6DSOX_FIFO_CTRL4, 0x00);
}

uint16_t LSM6DSOXFIFOClass::unread_words()
{
  int16_t diff = (int16_t)write_idx - (int16_t)read_idx;
  if(diff < 0) {
    diff += BUFFER_WORDS;
  }
  return (uint16_t)diff;
}

int LSM6DSOXFIFOClass::readStatus(FIFOStatus& status)
{
  uint8_t status_registers[2];
  int result = imu->readRegisters(LSM6DSOX_STATUS1, &status_registers[0], 2);
  if(result == 1) {
    status.DIFF_FIFO = status_registers[0] | ((status_registers[1] & 0x03) << 8);
    status.FIFO_OVR_LATCHED = (status_registers[1] & 0x08) == 0x08;
    status.COUNTER_BDR_IA = (status_registers[1] & 0x10) == 0x10;
    status.FIFO_FULL_IA = (status_registers[1] & 0x20) == 0x20;
    status.FIFO_OVR_IA = (status_registers[1] & 0x40) == 0x40;
    status.FIFO_WTM_IA = (status_registers[1] & 0x80) == 0x80;
  }
  return result;
}

// Read as much data as possible in one multiple byte/word read from sensor fifo
// to our own buffer
int LSM6DSOXFIFOClass::readData(uint16_t& words_read, bool& too_full)
{
  words_read = 0;
  too_full = false;

  FIFOStatus status;
  int result = readStatus(status);
  if(result == 1) {
    // The I2C/SPI multibyte read requires contiguous memory. Therefore fifo reading 
    // operations can not run past the end of the buffer. They can also not run
    // up to the current read pointer, in order to prevent data overrun.
    uint16_t to_read = status.DIFF_FIFO;
    if(read_idx > write_idx) {
      uint16_t readable = read_idx - write_idx;
      if(to_read > readable) {
        to_read = readable;
        too_full = true;
      }
    // Check special case where read and write pointer coincide: the buffer
    // is either empty or completely full
    } else if((read_idx == write_idx) && !buffer_empty) {
        to_read = 0;
        too_full = true;
    } else { // Now read_idx < write_idx, so we can write up all the way to buffer end
      uint16_t to_end = BUFFER_WORDS - write_idx;
      if(to_read > to_end) to_read = to_end;
    }
    if(to_read == 0) return 2; // No data read, but other reason than communication problem (<= 0)

    // This read operation uses fifo register rounding, see AN5272 par. 9.8
    result = imu->readRegisters(LSM6DSOX_FIFO_DATA_OUT_TAG, buffer_pointer(write_idx), to_read*BUFFER_BYTES_PER_WORD);
    if(result != 1) return result;

    words_read = to_read;
    buffer_empty = false;
    if((write_idx += words_read) >= BUFFER_WORDS) write_idx -= BUFFER_WORDS; // Wrap around to buffer start
  }
  return result;
}

int LSM6DSOXFIFOClass::getSample(Sample& sample)
{
  int result = -1;

  //bool Gfound = false;
  //bool XLfound = false;

  uint16_t unread = unread_words();
  if(unread > 0) {
    memcpy((void*)&sample, (const void*)buffer_pointer(read_idx++), (size_t)BUFFER_BYTES_PER_WORD);
    if(read_idx >= BUFFER_WORDS) read_idx -= BUFFER_WORDS;
    buffer_empty = (read_idx == write_idx);
    result = 1;
  }

  return result;
}

