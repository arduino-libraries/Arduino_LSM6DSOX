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
    int temperature_int = 0;
    float temperature_float = 0;
    IMU.readTemperature(temperature_int);
    IMU.readTemperatureFloat(temperature_float);

    Serial.print("LSM6DSOX Temperature = ");
    Serial.print(temperature_int);
    Serial.print(" (");
    Serial.print(temperature_float);
    Serial.print(")");
    Serial.println(" °C");
  }
}
