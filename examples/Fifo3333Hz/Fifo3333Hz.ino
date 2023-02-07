/*
  Arduino LSM6DSOX - Read FIFO

  This example demonstrates reading acceleratomer and
  gyroscope values from an LSM6DSOX FIFO.

  The circuit:
  - Arduino Nano RP2040 Connect

  created 7 February 2023
  by Rene van Ee

  This example code is in the public domain.
*/

#include <Arduino_LSM6DSOX.h>

int lasttime;
int counter;
int underrun;

#define REPORT_INTERVAL 5000

void setup() {
  Serial.begin(115200);
  while (!Serial);

  IMU.settings.sampleRate = 3333;
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }

  IMU.resetTimestamp();

  IMU.fifo.settings.compression = true;
  IMU.fifo.settings.timestamp_decimation = 32;
  IMU.fifo.settings.temperature_frequency = 2;
  IMU.fifo.begin();

  // Wait until samples are being produced
  Sample sample;
  SampleStatus sampleResult;
  while((sampleResult = IMU.fifo.getSample(sample)) == SampleStatus::BUFFER_UNDERRUN);
  if(sampleResult != SampleStatus::OK) {
    Serial.println("Error reading from IMU!");
    while (1);
  }

  counter = 0;
  underrun = 0;
  lasttime = millis();
}

void loop() {
  Sample sample;
  SampleStatus sampleResult = IMU.fifo.getSample(sample);
  bool errorOccurred = false;
  switch(sampleResult) {
    case SampleStatus::OK:
      counter++;
      break;
    case SampleStatus::BUFFER_UNDERRUN:
      underrun++;
      delay(1);
      break;
    case SampleStatus::COMMUNICATION_ERROR:
      Serial.print("Communication error");
      errorOccurred = true;
      break;
    case SampleStatus::PARITY_ERROR:
      Serial.print("Tag parity error");
      errorOccurred = true;
      break;
    case SampleStatus::TAG_NOT_IMPLEMENTED:
      Serial.print("Tag not implemented error");
      errorOccurred = true;
      break;
    case SampleStatus::UNKNOWN_TAG:
      Serial.print("Unknow tag error");
      errorOccurred = true;
      break;
    case SampleStatus::BUFFER_OVERRUN:
      Serial.print("Buffer overrun error");
      errorOccurred = true;
      break;
    default:
      break;
  }
  if(errorOccurred) {
    Serial.println(" while reading from IMU!");
    while (1);
  }
  
  // Reporting
  int currenttime = millis();
  int deltat = currenttime - lasttime;
  if(deltat >  REPORT_INTERVAL) {
    double frequency = (double)counter / (0.001*deltat);
    Serial.println("Sample frequency = " + String(frequency) + 
      " (underrun = " + String(underrun) + "/ " + String(counter) + ")");
    lasttime = currenttime;
    counter = 0;
    underrun = 0;
  }
}
