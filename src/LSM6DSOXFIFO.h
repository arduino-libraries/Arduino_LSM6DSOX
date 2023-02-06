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

// Buffer size. Define extra slots for compression, timestamp and config change
#define BUFFER_WORDS          (512 + 5) // Number of 'words'
#define BUFFER_BYTES_PER_WORD 7         // Tag + 3 * (2 byte word)
#define SAMPLE_BUFFER_SIZE    4         // 4 possible TAGCNT values (0-3)

struct FIFOSettings {
public:
  uint16_t watermark_level;           //9 bits (0-511)
  uint8_t timestamp_decimation;       //0/1/8/32
  uint8_t temperature_frequency;      //0/2/13/52
  uint16_t counter_threshold;         //11 bits (0-2047)
  bool counter_gyro;                  //false = count XL, true = count G
  bool compression;                   //true = enable compression
  uint8_t force_non_compressed_write; //0=never, 1=every 8 BDR, 2=16 BDR, 3=32 BDR
  bool cfg_change;                    //store CFG-change data in FIFO
  uint8_t fifo_mode;                  //3-bit pattern, see datasheet
};

struct FIFOStatus {
public:
  bool FIFO_WTM_IA;     // FIFO watermark status: 1: FIFO filling >= WTM
  bool FIFO_OVR_IA;     // FIFO overrun status. 1: FIFO is completely filled
  bool FIFO_FULL_IA;    // Smart FIFO full status. 1: FIFO will be full at the next ODR
  bool COUNTER_BDR_IA;  // Counter BDR reaches CNT_BDR_TH_[10:0] threshold
  bool FIFO_OVR_LATCHED;// Latched FIFO overrun status
  uint16_t DIFF_FIFO;   // Number of unread sensor data (TAG + 6 bytes) stored in FIFO
};

struct Sample {
  int16_t G_X;
  int16_t G_Y;
  int16_t G_Z;
  int16_t XL_X;
  int16_t XL_Y;
  int16_t XL_Z;
  double timestamp;     // May be NaN
  float temperature;    // May be NaN
  uint32_t counter;     // Lowest 2 bits provided by tag byte (TAG_CNT)
  uint8_t XL_fullScale;
  uint16_t G_fullScale;
};

struct RawWord {
  uint8_t bytes[BUFFER_BYTES_PER_WORD];
};

class LSM6DSOXClass;

class LSM6DSOXFIFOClass {
  public:
    FIFOSettings  settings;

    LSM6DSOXFIFOClass(LSM6DSOXClass* imu);
    ~LSM6DSOXFIFOClass();

    void initializeSettings(
      uint16_t watermark_level = 0,
      uint8_t timestamp_decimation = 8,
      uint8_t temperature_frequency = 2,  //1.6Hz
      uint16_t counter_threshold = 9,
      bool counter_gyro = true,           //Use G count (rather than XL)
      bool compression = false,
      uint8_t force_non_compressed_write = 2, // 0=never, 1=every 8 BDR, 2=16 BDR, 3=32 BDR
      bool cfg_change = true,
      uint8_t fifo_mode = 6);             //Continuous mode

    void begin();
    void end();

    // Fetch data from IMU
    int readStatus(FIFOStatus& status);
    int readData(uint16_t& words_read, bool& too_full, FIFOStatus& status);

    // Retrieve fetched data from local buffer
    int getRawWord(RawWord& word);
    int getSample(Sample& sample);

    uint8_t         buffer[BUFFER_WORDS * BUFFER_BYTES_PER_WORD];
    uint16_t        read_idx;
    uint16_t        write_idx;
    bool            buffer_empty;

    double          timestampCorrection;
    uint8_t         fullScaleXL;
    uint16_t        fullScaleG;
    bool            compressionEnabled;

  private:
    LSM6DSOXClass*  imu;
    

    void            updateReadPointer();
    uint16_t        unread_words();

    enum class WordStatus { 
      OK,
      TAG_NOT_IMPLEMENTED,
      UNKNOWN_TAG,
      MISSING_TAGCNT_ERROR,
      PARITY_ERROR,
      LOGIC_ERROR
    };
    WordStatus      inspectWord(uint16_t idx);
    int             releaseSample(uint16_t idx, Sample& extracted_sample);
    WordStatus      decodeWord(uint16_t idx);

    uint8_t*        buffer_pointer(uint16_t idx) { return &buffer[idx * BUFFER_BYTES_PER_WORD]; }
    void            initializeSample(uint8_t idx);
    void            extractSample(uint8_t cnt, Sample& extracted_sample);
    int16_t         bytesToInt16(uint8_t lo, uint8_t hi);
    int16_t         int5ToInt16(uint8_t five);
    int16_t         int8ToInt16(uint8_t eight);

    Sample          sample[SAMPLE_BUFFER_SIZE]; // Ring buffer, contains the words at T-3, T-2, T-1 and T
    uint8_t         previoustagcnt;
    uint8_t         nextsampletagcnt;
    uint32_t        timestamp_counter;

    // For convenience and clarity
    const uint8_t FIFO_DATA_OUT_TAG = 0;
    const uint8_t FIFO_DATA_OUT_X_L = 1;
    const uint8_t FIFO_DATA_OUT_X_H = 2;
    const uint8_t FIFO_DATA_OUT_Y_L = 3;
    const uint8_t FIFO_DATA_OUT_Y_H = 4;
    const uint8_t FIFO_DATA_OUT_Z_L = 5;
    const uint8_t FIFO_DATA_OUT_Z_H = 6;

    /* For debugging purposes
    void displaySamples();
    */
};

#endif
