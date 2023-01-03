/*
  Arduino LSM6DSOX - Simple Temperature

  This example reads the timestamp values from the LSM6DSOX
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.
  The timestamp is reset every 20 seconds.

  The circuit:
  - Arduino Nano RP2040 Connect

  created 30 August 2021
  by Alexander Entinger

  This example code is in the public domain.
*/

#include <Arduino_LSM6DSOX.h>

void setup()
{
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin())
  {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}

void loop()
{
  double timestamp = 0.0;
  IMU.readTimestampDouble(timestamp);
  Serial.print("Timestamp: ");
  Serial.print(timestamp, 6);
  Serial.println("s");

  if(timestamp > 20.0) {
    IMU.resetTimestamp();
  }

  delay(1000);
}
