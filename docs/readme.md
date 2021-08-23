# ArduinoLSM6DSOX library


The ArduinoLSM6DSOX library allows you to use the inertial measurement unit (IMU) available on the Arduino&reg; Nano RP2040 Connect board. The IMU is a [LSM6DSOX](https://www.st.com/resource/en/datasheet/lsm6dsox.pdf), it is a 3-axis accelerometer and 3-axis gyroscope. The IMU is connected to the Nano RP2040 Connect board's microcontroller through I2C. The values returned are signed floats.

To use this library:

```
#include <Arduino_LSM6DSOX.h>
```

The ArduinoLSM6DSOX library takes care of the sensor initialization and sets its values as follows:

- Accelerometer range is set at ±4 g with a resolution of 0.122 mg.
- Gyroscope range is set at ±2000 dps with a resolution of 70 mdps.
- Accelerometer and gyrospcope output data rate is fixed at 104 Hz.

