/*
  Arduino LSM6DSOX - Simple Temperature

  This example reads the temperature values from the LSM6DSOX
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

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
  if (IMU.temperatureAvailable())
  {
    int temperature_deg = 0;
    IMU.readTemperature(temperature_deg);

    Serial.print("LSM6DSOX Temperature = ");
    Serial.print(temperature_deg);
    Serial.println(" °C");
  }
}
