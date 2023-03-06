# ArduinoLSM6DSOX library


The ArduinoLSM6DSOX library allows you to use the inertial measurement unit (IMU) available on the Arduino&reg; Nano RP2040 Connect board. The IMU is a [LSM6DSOX](https://www.st.com/resource/en/datasheet/lsm6dsox.pdf), it is a 3-axis accelerometer and 3-axis gyroscope. The IMU is connected to the Nano RP2040 Connect board's microcontroller through I2C. The values returned are mostly signed 16 bit integers, accompanied by their full range (gyroRange or accelRange, see below).

To use this library:

```
#include <Arduino_LSM6DSOX.h>
```

The ArduinoLSM6DSOX library takes care of the sensor initialization. It has
a range of parameters that can be set:

- sampleRate: 13, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 Hz
- gyroRange: 2, 4, 8, 16 g
- accelRange: 125, 250, 250, 500, 1000, 2000 deg/s

Note that 3333 and 6667 Hz can not be used with FIFO compression (see below) and require more bandwidth than (400kHz) I2C has to offer.

Then there are various FIFO options:

- compression: true = enable compression
- timestamp_decimation;: 0, 1, 8, 32 (0 means no timestamps)
- temperature_frequency: 0, 2, 13, 52 (0 means no temperature measurements)
- force_non_compressed_write: 0=never, 1=every 8 BDR, 2=16 BDR, 3=32 BDR

The following FIFO options can be used for finetuning its behaviour and will normally be irrelevant to the user:

- watermark_level: 0-511
- counter_threshold: 0-2047
- counter_gyro: false = count XL, true = count G
- cfg_change: store CFG-change data in FIFO
- fifo_mode: 3-bit pattern, see datasheet
