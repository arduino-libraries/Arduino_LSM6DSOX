# 🤖 LSM6DSOX Arduino Library

[![Check Arduino status](https://github.com/arduino-libraries/Arduino_LSM6DSOX/actions/workflows/check-arduino.yml/badge.svg)](https://github.com/arduino-libraries/Arduino_LSM6DSOX/actions/workflows/check-arduino.yml)
[![Compile Examples](https://github.com/arduino-libraries/Arduino_LSM6DSOX/actions/workflows/compile-examples.yml/badge.svg)](https://github.com/arduino-libraries/Arduino_LSM6DSOX/actions/workflows/compile-examples.yml)
[![Spell Check](https://github.com/arduino-libraries/Arduino_LSM6DSOX/actions/workflows/spell-check.yml/badge.svg)](https://github.com/arduino-libraries/Arduino_LSM6DSOX/actions/workflows/spell-check.yml)

This library allows you to read accelerometer, gyroscope, and temperature values from the **LSM6DSOX** IMU.

## 🌟 Features

This library supports the API exposed by the LSM6DSOX sensor over I2C and SPI.

- 📏 **Accelerometer Control**
    - Read X, Y, and Z acceleration (in g)
    - Check available samples
    - Get sampling rate
- 🔄 **Gyroscope Control**
    - Read X, Y, and Z gyroscope values (in degrees/second)
    - Check available samples
    - Get sampling rate
- 🌡 **Temperature Sensor Control**
    - Read temperature (in °C, integer or float formats)
    - Check availability of temperature samples

## 📖 Documentation
For more information on the features of this library and how to use them, please read the API documentation [here](./docs/api.md).

## ✅ Supported Boards

The library should work on any board that uses a modern Arduino core. It requires either an I2C or SPI interface to connect to an LSM6DSOX sensor.

## 📞 Communication Interface

The default `IMU` variable uses the standard `Wire` interface. If the IMU is connected to a different interface, an instance of `LSM6DSOXClass` can be declared using custom I2C or SPI pins. e.g.

```cpp
// Custom I2C device
LSM6DSOXClass myIMU = LSM6DSOXClass(Wire1, 0x6A);

// Custom SPI device
LSM6DSOXClass myIMU_SPI = LSM6DSOXClass(SPI, 10, 2);
```

## ⚙️ Installation

The library is preinstalled in [**Arduino Cloud Editor**](https://docs.arduino.cc/arduino-cloud/guides/cloud-editor).

Arduino IDE users can install it via [Library Manager](https://docs.arduino.cc/software/ide-v2/tutorials/ide-v2-installing-a-library#installing-a-library).

Arduino CLI can install it with `arduino-cli lib install Arduino_LSM6DSOX`. You may need to update the index beforehand using `arduino-cli lib update-index`.

## 🧑‍💻 Developer Installation

Clone the repository and then run any example using the Arduino CLI. e.g. 
```shell
arduino-cli compile examples/SimpleAccelerometer/SimpleAccelerometer.ino -b arduino:mbed_nano:nanorp2040connect --library ./ -u -p /dev/cu.usbmodem14201
```

## 🐛 Reporting Issues

If you encounter any issue, please open a bug report [here](https://github.com/arduino-libraries/Arduino_LSM6DSOX/issues).

## 📕 Further Reading

- Datasheet:
    - [LSM6DSOX datasheet](https://www.st.com/resource/en/datasheet/lsm6dsox.pdf)

## 💪 Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## 🤙 Support

If you have questions about using the library, you can get assistance on Arduino Forum:

https://forum.arduino.cc/

## ⚖️ License

This library is released under the [GNU Lesser General Public License v2.1 (LGPL)](http://www.gnu.org/licenses/lgpl-2.1.html).
