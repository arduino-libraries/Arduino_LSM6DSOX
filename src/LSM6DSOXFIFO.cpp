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

#define LSM6DSOX_FUNC_CFG_ACCESS    0x01
#define LSM6DSOX_FIFO_CTRL1         0x07
#define LSM6DSOX_FIFO_CTRL2         0x08
#define LSM6DSOX_FIFO_CTRL3         0x09
#define LSM6DSOX_FIFO_CTRL4         0x0A
#define LSM6DSOX_COUNTER_BDR_REG1   0x0B
#define LSM6DSOX_COUNTER_BDR_REG2   0x0C

#define LSM6DSOX_STATUS1            0x3A
#define LSM6DSOX_STATUS2            0x3B

#define LSM6DSOX_FIFO_DATA_OUT_TAG  0x78

//#define LSM6DSOX_EMB_FUNC_PAGE_SEL  0x02
#define LSM6DSOX_EMB_FUNC_EN_B      0x05
//#define LSM6DSOX_EMB_FUNC_PAGE_RW   0x17

// Used by decoder for start-up
#define COUNTER_UNDEFINED           0xFFFFFFFF

// I2C buffer size is limited to 32 bytes, see link below.
// https://reference.arduino.cc/reference/en/language/functions/communication/wire/
#define I2C_BUFFER_LENGTH           32
#define READ_MAX_WORDS              (I2C_BUFFER_LENGTH / BUFFER_BYTES_PER_WORD)

// Number of retries after read error (this number is a bit arbitrary...)
#define READ_RETRIES                5

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

/* For debugging purposes
std::map<uint8_t, String> mapTagToStr = {
  { 0x01, "G_NC" }, 
  { 0x02, "XL_NC" }, 
  { 0x03, "TEMP" }, 
  { 0x04, "TIME" },
  { 0x05, "CFG" }, 
  { 0x06, "XL_NC_T_2" }, 
  { 0x07, "XL_NC_T_1" }, 
  { 0x08, "XL_2C" },
  { 0x09, "XL_3C" }, 
  { 0x0A, "G_NC_T_2" }, 
  { 0x0B, "G_NC_T_1" }, 
  { 0x0C, "G_2C" }, 
  { 0x0D, "G_3C" },
  { 0x0E, "SENS_HUB_0" }, 
  { 0x0F, "SENS_HUB_1" }, 
  { 0x10, "SENS_HUB_2" }, 
  { 0x11, "SENS_HUB_3" },
  { 0x12, "STEP_CNT" },
  { 0x19, "SENS_HUB_NACK" }
}; */

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
  bool compression,
  uint8_t force_non_compressed_write,
  bool cfg_change,
  uint8_t fifo_mode) {
    settings.watermark_level = watermark_level;
    settings.timestamp_decimation = timestamp_decimation;
    settings.temperature_frequency = temperature_frequency;
    settings.counter_threshold = counter_threshold;    
    settings.counter_gyro = counter_gyro;
    settings.compression = compression;
    settings.force_non_compressed_write = force_non_compressed_write;
    settings.cfg_change = cfg_change;
    settings.fifo_mode = fifo_mode;
}

void LSM6DSOXFIFOClass::begin()
{
  // Find timestamp correction factor. See AN5272, par 6.4
  int8_t freq_fine;
  imu->readInternalFrequency(freq_fine);
  timestampCorrection = imu->correctTimestamp(1, freq_fine);

  // Reset timestamp counter. Note that its lowest 2 bits will be filled by tagcnt
  // Set it to max to signal it should be reset using tagcnt
  timestamp_counter = 4;

  // Enable or disable compression. See also below (FIFO_CTRL2)
  uint8_t emb_func_en_b = settings.compression ? 0x08 : 0x00;
  imu->writeRegister(LSM6DSOX_FUNC_CFG_ACCESS, 0x80); // Enable embedded function registers access
  imu->writeRegister(LSM6DSOX_EMB_FUNC_EN_B, emb_func_en_b); // Enable or disable compression
  imu->writeRegister(LSM6DSOX_FUNC_CFG_ACCESS, 0x00); // Disable embedded function registers access
  compressionEnabled = settings.compression;

  // Now find all configuration values
  uint8_t fifo_ctrl1 = settings.watermark_level & 0xFF; // WTM0..8
  uint8_t fifo_ctrl2 = settings.compression ? 0x40 : 0x00; // Default STOP_ON_WTM=0
  fifo_ctrl2 |= (settings.watermark_level >> 8) & 0x01; // Put WTM8 into CTRL2 bit 0
  fifo_ctrl2 |= (settings.force_non_compressed_write & 0x03) << 1; // UNCOPTR_RATE[1:0]
  fifo_ctrl2 |= settings.cfg_change ? 0x10 : 0x00;  // Enable FIFO storage of configuration changes 

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
  uint8_t counter_bdr_reg2 = settings.counter_threshold & 0xFF; // CNT_BDR_TH_0..7

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

  // Decoder
  timestamp_counter = COUNTER_UNDEFINED;
  to_release_counter = COUNTER_UNDEFINED;

  // Initialize circular sample buffer
  for(uint8_t idx = 0; idx < SAMPLE_BUFFER_SIZE; idx++) {
    initializeSample(idx);
  } 
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
  // Note that write_id and read_idx are always >= 0
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
int LSM6DSOXFIFOClass::readData(uint16_t& words_read, bool& too_full, FIFOStatus& status)
{
  words_read = 0;
  too_full = false;

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

    // Break down read operations into a maximum number of words
    while(to_read > 0) {
      uint16_t read_now = (to_read > READ_MAX_WORDS) ? READ_MAX_WORDS : to_read;

      result = imu->readRegisters(LSM6DSOX_FIFO_DATA_OUT_TAG, buffer_pointer(write_idx), read_now*BUFFER_BYTES_PER_WORD);
      if(result != 1) return result;

      to_read -= read_now;
      if((write_idx += read_now) >= BUFFER_WORDS) write_idx -= BUFFER_WORDS; // Wrap around to buffer start
      buffer_empty = false;
    } // END while(to_read > 0)
  }
  return result;
}

SampleStatus LSM6DSOXFIFOClass::getRawWord(RawWord& word)
{
  SampleStatus result = SampleStatus::BUFFER_UNDERRUN;

  uint16_t unread = unread_words();
  if(unread > 0) {
    memcpy((void*)&word, (const void*)buffer_pointer(read_idx), size_t(BUFFER_BYTES_PER_WORD));
    updateReadPointer();
    result = SampleStatus::OK;
  }

  return result;
}

SampleStatus LSM6DSOXFIFOClass::getSample(Sample& sample)
{
  // First process words in the local buffer, return as soon as a sample
  // can be released (lazy approach).
  // If the local buffer runs empty, fill it from the fifo, then
  // again decode words until a sample may be released.
  uint16_t words_read = 0;
  do {
    // Process all words available in the local buffer,
    // until a sample is produced or no more words are
    // available in the local buffer.
    while(!buffer_empty) {
      // Inspect word at read idx pointer
      SampleStatus inspectStatus = inspectWord(read_idx);
      switch(inspectStatus) {
        case SampleStatus::OK:
          // Nothing wrong, continue below
          break;

        case SampleStatus::PARITY_ERROR: // Parity error -> communication problem?
        default:
          return inspectStatus;
      }

      // Extract sample if available
      int samples_released = releaseSample(read_idx, sample);
      if(samples_released > 0) {
        // Ready: sample released, no errors
        return SampleStatus::OK;
      }
      
      SampleStatus decodeStatus = decodeWord(read_idx);
      updateReadPointer(); // Updates buffer_empty as well
      switch(decodeStatus) {
        case SampleStatus::OK:
          // Continue below, i.e. enter next !buffer_empty loop iteration
          break;

        case SampleStatus::TAG_NOT_IMPLEMENTED:
        case SampleStatus::UNKNOWN_TAG:
        default:
          return decodeStatus;
      }
    } // END while(!buffer_empty)

    // If no sample was produced, read a fresh batch of
    // words from the IMU to the local buffer. Then resume
    // processing them, again until a sample is produced.
    bool too_full = false;
    // Read block of data. Note that too_full will always be false,
    // since the buffer was emptied above.
    FIFOStatus status;
    int read_result = readData(words_read, too_full, status);
    // If an error occurred (result <= 0), report communication error
    if(read_result <= 0) {
      return SampleStatus::COMMUNICATION_ERROR;
    }
    // Buffer overrun qualifies as an error too
    if(status.FIFO_OVR_LATCHED) {
      return SampleStatus::BUFFER_OVERRUN;
    }
  } while (words_read > 0);

  // No words were read (so fifo is empty, 'underrun')
  return SampleStatus::BUFFER_UNDERRUN;
}

void LSM6DSOXFIFOClass::updateReadPointer()
{
  if(++read_idx >= BUFFER_WORDS) read_idx -= BUFFER_WORDS;
  buffer_empty = (read_idx == write_idx);
}

SampleStatus LSM6DSOXFIFOClass::inspectWord(uint16_t idx)
{
  uint8_t *word = buffer_pointer(idx);
  
  // Perform parity check
  uint8_t parity = word[FIFO_DATA_OUT_TAG] ^ (word[FIFO_DATA_OUT_TAG] >> 4);
  parity ^= (parity >> 2);
  parity ^= (parity >> 1);
  if(parity & 0x01) {
    return SampleStatus::PARITY_ERROR; // Parity error -> communication problem?
  }

  return SampleStatus::OK;
}

int LSM6DSOXFIFOClass::releaseSample(uint16_t idx, Sample& extracted_sample)
{
  // Note: this function updates previoustagcnt, timestamp_counter and initializes a new sample
  // in the circular sample buffer

  // Retrieve word from word buffer
  uint8_t *word = buffer_pointer(idx);

  // Tag counter
  uint8_t tagcnt = (word[FIFO_DATA_OUT_TAG] & 0x6) >> 1;  // T

  // timestamp_counter is undefined at fifo startup.
  if(timestamp_counter == COUNTER_UNDEFINED) {
    // If still unitialized, initialize it with tagcnt,
    // so the lower 2 bits always resemble tagcnt
    timestamp_counter = tagcnt;

    // The first sample to be released is that of
    // the current timestamp counter
    to_release_counter = timestamp_counter;
  }

  // Update counter based on tagcnt
  uint8_t prev_tagcnt = timestamp_counter & 0x03;
  if(tagcnt != prev_tagcnt) {
    if(tagcnt < prev_tagcnt) {
      timestamp_counter += 4;
    }
    timestamp_counter &= 0xFFFFFFFC;
    timestamp_counter |= tagcnt;
  }

  // If compression is enabled, there should be a delay of
  // 2 in releasing samples to account for the compression
  // algorithm modifying data at T-2 and T-1, rather than 
  // just at time T.
  uint8_t delta_cnt = compressionEnabled ? 2 : 0;
  if((to_release_counter + delta_cnt) < timestamp_counter) {
    uint8_t releasecnt = (uint8_t)(to_release_counter & 0x03);
    extracted_sample = sample[releasecnt];
    initializeSample(releasecnt);
    to_release_counter++;

    return 1; // Sample released
  }

  return 0; // No sample released, keep decoding
}

SampleStatus LSM6DSOXFIFOClass::decodeWord(uint16_t idx)
{
  // Note: this function updates fullScaleG, fullScaleXL and compressionEnabled,
  // as well as the sample circular buffer
  uint8_t *word = buffer_pointer(idx);

  // Tag counters
  uint8_t tagcnt = (word[FIFO_DATA_OUT_TAG] & 0x6) >> 1;  // T
  uint8_t tagcnt_1 = (tagcnt-1) & 0x03; // T-1
  uint8_t tagcnt_2 = (tagcnt-2) & 0x03; // T-2
  uint8_t tagcnt_3 = (tagcnt-3) & 0x03; // T-3

  // Decode tag
  uint8_t tag = word[FIFO_DATA_OUT_TAG] >> 3;
  switch(tag) {
    case 0x01: // Gyroscope NC Main Gyroscope uncompressed data
    {
      sample[tagcnt].G_X = bytesToInt16(word[FIFO_DATA_OUT_X_L], word[FIFO_DATA_OUT_X_H]);
      sample[tagcnt].G_Y = bytesToInt16(word[FIFO_DATA_OUT_Y_L], word[FIFO_DATA_OUT_Y_H]);
      sample[tagcnt].G_Z = bytesToInt16(word[FIFO_DATA_OUT_Z_L], word[FIFO_DATA_OUT_Z_H]);
      sample[tagcnt].G_fullScale = imu->fullScaleG;
      break;
    }

    case 0x02: // Accelerometer NC Main Accelerometer uncompressed data
    {
      sample[tagcnt].counter = timestamp_counter;
      sample[tagcnt].XL_X = bytesToInt16(word[FIFO_DATA_OUT_X_L], word[FIFO_DATA_OUT_X_H]);
      sample[tagcnt].XL_Y = bytesToInt16(word[FIFO_DATA_OUT_Y_L], word[FIFO_DATA_OUT_Y_H]);
      sample[tagcnt].XL_Z = bytesToInt16(word[FIFO_DATA_OUT_Z_L], word[FIFO_DATA_OUT_Z_H]);
      sample[tagcnt].XL_fullScale = imu->fullScaleXL;
      break;
    }

    case 0x03: // Temperature Auxiliary Temperature data
    {
      sample[tagcnt].temperature = 
        imu->temperatureToCelsius(bytesToInt16(word[FIFO_DATA_OUT_X_L], word[FIFO_DATA_OUT_X_H]));
      break;
    }

    case 0x04: // Timestamp Auxiliary Timestamp data
    {
      uint32_t timestamp = 
        (((uint32_t)word[FIFO_DATA_OUT_Y_H]) << 24) | 
        (((uint32_t)word[FIFO_DATA_OUT_Y_L]) << 16) |
        (((uint32_t)word[FIFO_DATA_OUT_X_H]) <<  8) |
         ((uint32_t)word[FIFO_DATA_OUT_X_L]);
      sample[tagcnt].timestamp = timestamp * timestampCorrection;
      break;
    }

    case 0x05: // CFG_Change Auxiliary Meta-information data
    {
      // Gyro full range
      uint8_t fs_g = word[FIFO_DATA_OUT_X_H] >> 5; // FS1_G FS0_G FS_125
      imu->fullScaleG = imu->fs_g_to_range(fs_g);
      sample[tagcnt].G_fullScale = imu->fullScaleG;

      // Accelerometer full range
      uint8_t fs_xl = word[FIFO_DATA_OUT_Y_L] >> 6; // FS1_XL FS0_XL
      imu->fullScaleXL = imu->fs_xl_to_range(fs_xl);
      sample[tagcnt].XL_fullScale = imu->fullScaleXL;

      // Compression
      compressionEnabled = (word[FIFO_DATA_OUT_Y_H] & 0x80) == 0x80;
      break;
    }

    case 0x06: // Accelerometer NC_T_2 Main Accelerometer uncompressed batched at two times the previous time slot
    {
      sample[tagcnt_2].counter = timestamp_counter-2;
      sample[tagcnt_2].XL_X = bytesToInt16(word[FIFO_DATA_OUT_X_L], word[FIFO_DATA_OUT_X_H]);
      sample[tagcnt_2].XL_Y = bytesToInt16(word[FIFO_DATA_OUT_Y_L], word[FIFO_DATA_OUT_Y_H]);
      sample[tagcnt_2].XL_Z = bytesToInt16(word[FIFO_DATA_OUT_Z_L], word[FIFO_DATA_OUT_Z_H]);
      sample[tagcnt_2].XL_fullScale = imu->fullScaleXL;
      break;
    }

    case 0x07: // Accelerometer NC_T_1 Main Accelerometer uncompressed data batched at the previous time slot
    {
      sample[tagcnt_1].counter = timestamp_counter-1;
      sample[tagcnt_1].XL_X = bytesToInt16(word[FIFO_DATA_OUT_X_L], word[FIFO_DATA_OUT_X_H]);
      sample[tagcnt_1].XL_Y = bytesToInt16(word[FIFO_DATA_OUT_Y_L], word[FIFO_DATA_OUT_Y_H]);
      sample[tagcnt_1].XL_Z = bytesToInt16(word[FIFO_DATA_OUT_Z_L], word[FIFO_DATA_OUT_Z_H]);
      sample[tagcnt_1].XL_fullScale = imu->fullScaleXL;
      break;
    }

    case 0x08: // Accelerometer 2xC Main Accelerometer 2x compressed data
    {
      sample[tagcnt_2].counter = timestamp_counter-2;
      sample[tagcnt_1].counter = timestamp_counter-1;
      sample[tagcnt_2].XL_X = sample[tagcnt_3].XL_X + int8ToInt16(word[FIFO_DATA_OUT_X_L]);
      sample[tagcnt_2].XL_Y = sample[tagcnt_3].XL_Y + int8ToInt16(word[FIFO_DATA_OUT_X_H]);
      sample[tagcnt_2].XL_Z = sample[tagcnt_3].XL_Z + int8ToInt16(word[FIFO_DATA_OUT_Y_L]);
      sample[tagcnt_1].XL_X = sample[tagcnt_2].XL_X + int8ToInt16(word[FIFO_DATA_OUT_Y_H]);
      sample[tagcnt_1].XL_Y = sample[tagcnt_2].XL_Y + int8ToInt16(word[FIFO_DATA_OUT_Z_L]);
      sample[tagcnt_1].XL_Z = sample[tagcnt_2].XL_Z + int8ToInt16(word[FIFO_DATA_OUT_Z_H]);
      sample[tagcnt_2].XL_fullScale = imu->fullScaleXL;
      sample[tagcnt_1].XL_fullScale = imu->fullScaleXL;
      break;
    }

    case 0x09: // Accelerometer 3xC Main Accelerometer 3x compressed data
    {
      sample[tagcnt_2].counter = timestamp_counter-2;
      sample[tagcnt_1].counter = timestamp_counter-1;
      sample[tagcnt  ].counter = timestamp_counter;
      sample[tagcnt_2].XL_X = sample[tagcnt_3].XL_X + int5ToInt16(word[1] & 0x1F);
      sample[tagcnt_2].XL_Y = sample[tagcnt_3].XL_Y + int5ToInt16(((word[1] & 0xE0) >> 5) | (word[2] & 0x03));
      sample[tagcnt_2].XL_Z = sample[tagcnt_3].XL_Z + int5ToInt16((word[2] & 0x7C) >> 2);
      sample[tagcnt_1].XL_X = sample[tagcnt_2].XL_X + int5ToInt16(word[3] & 0x1F);
      sample[tagcnt_1].XL_Y = sample[tagcnt_2].XL_Y + int5ToInt16(((word[3] & 0xE0) >> 5) | (word[4] & 0x03));
      sample[tagcnt_1].XL_Z = sample[tagcnt_2].XL_Z + int5ToInt16((word[4] & 0x7C) >> 2);
      sample[tagcnt].XL_X = sample[tagcnt_1].XL_X + int5ToInt16(word[5] & 0x1F);
      sample[tagcnt].XL_Y = sample[tagcnt_1].XL_Y + int5ToInt16(((word[5] & 0xE0) >> 5) | (word[6] & 0x03));
      sample[tagcnt].XL_Z = sample[tagcnt_1].XL_Z + int5ToInt16((word[6] & 0x7C) >> 2);
      sample[tagcnt_2].XL_fullScale = imu->fullScaleXL;
      sample[tagcnt_1].XL_fullScale = imu->fullScaleXL;
      sample[tagcnt].XL_fullScale = imu->fullScaleXL;
      break;
    }

    case 0x0A: // Gyroscope NC_T_2 Main Gyroscope uncompressed data batched at two times the previous time slot
    {
      sample[tagcnt_2].G_X = bytesToInt16(word[FIFO_DATA_OUT_X_L], word[FIFO_DATA_OUT_X_H]);
      sample[tagcnt_2].G_Y = bytesToInt16(word[FIFO_DATA_OUT_Y_L], word[FIFO_DATA_OUT_Y_H]);
      sample[tagcnt_2].G_Z = bytesToInt16(word[FIFO_DATA_OUT_Z_L], word[FIFO_DATA_OUT_Z_H]);
      sample[tagcnt_2].G_fullScale = imu->fullScaleG;
      break;    
    }

    case 0x0B: // Gyroscope NC_T_1 Main Gyroscope uncompressed data batched at the previous time slot
    {
      sample[tagcnt_1].G_X = bytesToInt16(word[FIFO_DATA_OUT_X_L], word[FIFO_DATA_OUT_X_H]);
      sample[tagcnt_1].G_Y = bytesToInt16(word[FIFO_DATA_OUT_Y_L], word[FIFO_DATA_OUT_Y_H]);
      sample[tagcnt_1].G_Z = bytesToInt16(word[FIFO_DATA_OUT_Z_L], word[FIFO_DATA_OUT_Z_H]);
      sample[tagcnt_1].G_fullScale = imu->fullScaleG;
      break;
    }

    case 0x0C: // Gyroscope 2xC Main Gyroscope 2x compressed data
    {
      sample[tagcnt_2].G_X = sample[tagcnt_3].G_X + int8ToInt16(word[FIFO_DATA_OUT_X_L]);
      sample[tagcnt_2].G_Y = sample[tagcnt_3].G_Y + int8ToInt16(word[FIFO_DATA_OUT_X_H]);
      sample[tagcnt_2].G_Z = sample[tagcnt_3].G_Z + int8ToInt16(word[FIFO_DATA_OUT_Y_L]);
      sample[tagcnt_1].G_X = sample[tagcnt_2].G_X + int8ToInt16(word[FIFO_DATA_OUT_Y_H]);
      sample[tagcnt_1].G_Y = sample[tagcnt_2].G_Y + int8ToInt16(word[FIFO_DATA_OUT_Z_L]);
      sample[tagcnt_1].G_Z = sample[tagcnt_2].G_Z + int8ToInt16(word[FIFO_DATA_OUT_Z_H]);
      sample[tagcnt_2].G_fullScale = imu->fullScaleG;
      sample[tagcnt_1].G_fullScale = imu->fullScaleG;
      break;
    }

    case 0x0D: // Gyroscope 3xC Main Gyroscope 3x compressed data
    {
      sample[tagcnt_2].G_X = sample[tagcnt_3].G_X + int5ToInt16(word[FIFO_DATA_OUT_X_L] & 0x1F);
      sample[tagcnt_2].G_Y = sample[tagcnt_3].G_Y + int5ToInt16(((word[FIFO_DATA_OUT_X_L] & 0xE0) >> 5) | ((word[FIFO_DATA_OUT_X_H] & 0x03) << 3));
      sample[tagcnt_2].G_Z = sample[tagcnt_3].G_Z + int5ToInt16((word[FIFO_DATA_OUT_X_H] & 0x7C) >> 2);
      sample[tagcnt_1].G_X = sample[tagcnt_2].G_X + int5ToInt16(word[FIFO_DATA_OUT_Y_L] & 0x1F);
      sample[tagcnt_1].G_Y = sample[tagcnt_2].G_Y + int5ToInt16(((word[FIFO_DATA_OUT_Y_L] & 0xE0) >> 5) | ((word[FIFO_DATA_OUT_Y_H] & 0x03) << 3));
      sample[tagcnt_1].G_Z = sample[tagcnt_2].G_Z + int5ToInt16((word[FIFO_DATA_OUT_Y_H] & 0x7C) >> 2);
      sample[tagcnt].G_X = sample[tagcnt_1].G_X + int5ToInt16(word[FIFO_DATA_OUT_Z_L] & 0x1F);
      sample[tagcnt].G_Y = sample[tagcnt_1].G_Y + int5ToInt16(((word[FIFO_DATA_OUT_Z_L] & 0xE0) >> 5) | ((word[FIFO_DATA_OUT_Z_H] & 0x03) << 3));
      sample[tagcnt].G_Z = sample[tagcnt_1].G_Z + int5ToInt16((word[FIFO_DATA_OUT_Z_H] & 0x7C) >> 2);
      sample[tagcnt_2].G_fullScale = imu->fullScaleG;
      sample[tagcnt_1].G_fullScale = imu->fullScaleG;
      sample[tagcnt].G_fullScale = imu->fullScaleG;
      break;
    }

    case 0x0E: // Sensor Hub Slave 0 Virtual Sensor hub data from slave 0
    case 0x0F: // Sensor Hub Slave 1 Virtual Sensor hub data from slave 1
    case 0x10: // Sensor Hub Slave 2 Virtual Sensor hub data from slave 2
    case 0x11: // Sensor Hub Slave 3 Virtual Sensor hub data from slave 3
    case 0x12: // Step Counter Virtual Step counter data
    case 0x19: // Sensor Hub Nack Virtual Sensor hub nack from slave 0/1/2/3
      return SampleStatus::TAG_NOT_IMPLEMENTED;

    default:
      return SampleStatus::UNKNOWN_TAG;
  }

  return SampleStatus::OK;
}

void LSM6DSOXFIFOClass::initializeSample(uint8_t idx)
{
  // timestamp and temperature are not always sent
  sample[idx].timestamp = NAN;
  sample[idx].temperature = NAN;

  // Set counter and full scale to 'impossible' values
  // to help identifying errors
  sample[idx].counter = COUNTER_UNDEFINED;
  sample[idx].XL_fullScale = 0;
  sample[idx].G_fullScale = 0;

  // If compression is disabled, XL and G data may be
  // set to remarkable values in order to signal errors
  if(!compressionEnabled) {
    int16_t default_value = 0x7FFF;

    sample[idx].XL_X = default_value;
    sample[idx].XL_Y = default_value;
    sample[idx].XL_Z = default_value;

    sample[idx].G_X = default_value;
    sample[idx].G_Y = default_value;
    sample[idx].G_Z = default_value;
  }
}

int16_t LSM6DSOXFIFOClass::bytesToInt16(uint8_t lo, uint8_t hi)
{ 
   return ((int16_t)hi << 8) + lo;
}

int16_t LSM6DSOXFIFOClass::int5ToInt16(uint8_t five)
{
  return (five & 0x10) ? 
    (int16_t)five | 0xFFE0 : //sign extension
    (int16_t)five & 0x000F;
} 

int16_t LSM6DSOXFIFOClass::int8ToInt16(uint8_t eight)
{
  return (eight & 0x80) ? 
    (int16_t)eight | 0xFF00 : //sign extension
    (int16_t)eight & 0x007F;
}

/* For debugging purposes
void LSM6DSOXFIFOClass::displaySamples()
{
  Serial.println("---");
  for(uint8_t idx = 0; idx < SAMPLE_BUFFER_SIZE; idx++) {
    Serial.print("["+String(idx)+"] cnt= " + String(sample[idx].counter) + " t="+String(sample[idx].timestamp)+" T="+String(sample[idx].temperature));
    Serial.print(" G=("+String(sample[idx].G_X)+", " + String(sample[idx].G_Y) + ", "+String(sample[idx].G_Z) + ") {FS="+String(sample[idx].G_fullScale)+"}");
    Serial.println(" XL=("+String(sample[idx].XL_X)+", " + String(sample[idx].XL_Y) + ", "+String(sample[idx].XL_Z) + ") {FS="+String(sample[idx].XL_fullScale)+"}");
  }
  Serial.println("---");
} */
