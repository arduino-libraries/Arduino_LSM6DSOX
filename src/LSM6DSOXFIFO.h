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

struct FIFOSettings {
public:
  uint16_t watermark_level;       //9 bits (0-511)
  uint8_t timestamp_decimation;   //0/1/8/32
  uint8_t temperature_frequency;  //0/2/13/52
  uint16_t counter_threshold;     //11 bits (0-2047)
  bool counter_gyro;              //false = count XL, true = count G
  uint8_t fifo_mode;              //3-bit pattern, see datasheet
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

/*
struct Sample {
  int16_t G_X;
  int16_t G_Y;
  int16_t G_Z;
  int16_t XL_X;
  int16_t XL_Y;
  int16_t XL_Z;
  double timestamp;     // May be NaN
  uint16_t XL_fullScale;
  uint8_t G_fullScale;
}; */
struct Sample {
  uint8_t data[7];
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
      uint8_t fifo_mode = 6);             //Continuous mode

    void begin();
    void end();

    int readStatus(FIFOStatus& status);
    int readData(uint16_t& words_read, bool& too_full);
    int getSample(Sample& sample);

    uint8_t         buffer[BUFFER_WORDS * BUFFER_BYTES_PER_WORD];
    uint16_t        read_idx;
    uint16_t        write_idx;
    bool            buffer_empty;

    double          timestampCorrection;
    uint8_t         fullScaleXL;
    uint16_t        fullScaleG;

  private:
    LSM6DSOXClass*  imu;

    uint8_t*        buffer_pointer(uint16_t idx) { return &buffer[idx * BUFFER_BYTES_PER_WORD]; }
    uint16_t        unread_words();
};

#endif
