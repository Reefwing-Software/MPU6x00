![version](https://img.shields.io/github/v/tag/Reefwing-Software/Reefwing-MPU6x00) ![license](https://img.shields.io/badge/license-MIT-green) ![release](https://img.shields.io/github/release-date/Reefwing-Software/Reefwing-MPU6x00?color="red") ![open source](https://badgen.net/badge/open/source/blue?icon=github)

## Reefwing MPU6x00 Arduino Library

This is a fork of the [MPU6x00 Library](https://github.com/simondlevy/MPU6x00) by Simon D. Levy. It is a simple, header-only Arduino library for the InvenSense/TDK MPU6000 and MPU6500 inertial measurement units using
the SPI bus.  

It has been modified to use the [Reefwing imuTypes library header](https://github.com/Reefwing-Software/Reefwing-imuTypes) to make it compatible with the [Reefwing AHRS Library](https://github.com/Reefwing-Software/Reefwing-AHRS) and allow easy graphing of IMU values via the [Reefwing xIMU3](https://github.com/Reefwing-Software/Reefwing-xIMU3) data visualisation library.

The other methods added include:

- `bool dataAvailable()` - Can be used to poll the IMU and detect if new sensor readings are available. This is useful if you haven't wired up the IMU interrupt pin.
- `void getTemp(float & t)` - Loads the IMU chip temperature in celsius into t).
- `void getCalibratedGyro(float & gx, float & gy, float & gz)` - Loads the calibrated gyro values for the x, y, and z-axis into gx, gy, and gz. You need to call `calibrateAccelGyro()` first.
- `void getCalibratedAccel(float & ax, float & ay, float & az)` - Loads the calibrated accelerometer values for the x, y, and z-axis into ax, ay, and az. You need to call `calibrateAccelGyro()` first.
- `BiasOffsets getGyroOffsets()` - Returns the x, y, and z gyro offsets calculated by the `calibrateAccelGyro()` method.
- `BiasOffsets getAccelOffsets()` - Returns the x, y, and z accelerometer offsets calculated by the `calibrateAccelGyro()` method.
- `SensorData getSensorData()` - Returns calibrated gyro and accelerometer readings with time stamps, in a format suitable for sending to the Reefwing AHRS library. Call `readSensor()` first.
- `TempData getTempData()` - Returns chip temperature and time stamp, in a format suitable for sending to the Reefwing xIMU3 data visualisation library. Call `readSensor()` first.
- `InertialMessage getInertial()` - Returns calibrated gyro and accelerometer readings with time stamps, in a format suitable for sending to the Reefwing xIMU3 data visualisation library. Call `readSensor()` first.
- `void calibrateAccelGyro()` - Calculates and load the zero bias offsets for the gyro and accelerometer. These are used in the methods that return a calibrated value. The IMU needs to be flat and still while this method is running.

You will need to install the Reefwing imuTypes library before using this library. This can be done via the Arduino IDE Library Manager or directly from the [Reefwing Software GitHub repository](https://github.com/Reefwing-Software).

## The MPU-6500 IMU

The MPU-6500 is a 6-axis IMU that combines a 3-axis gyroscope, and a 3-axis accelerometer. The gyroscope has a programmable full-scale range of ±250, ±500, ±1000, and ±2000 degrees/sec. The accelerometer has a user-programmable accelerometer full-scale range of ±2g, ±4g, ±8g, and ±16g. Communication with all registers of the device is performed using either I²C at 400kHz or SPI at 1MHz. Our library uses only SPI.

## Library Examples

As our MPU-6500 shield breaks out the interrupt pin to the Arduino, most of our examples use the interrupt method to detect when new sensor readings are available. If your shield doesn't provide access to this pin, use the polling method instead (e.g., in the library examples folder, look at `polledIMU.ino`). Communication between the IMU and the Arduino is done via SPI.

The examples folder contains the following sketches:

- `ARDUINO_MKR_MPU6500` - This minimal example demonstrates connecting an Arduino to the MPU-6500 IMU, using SPI and interrupts. The results printed to the Arduino IDE Serial Monitor are raw gyro values for the x, y, and z axis. These need to be scaled to be useful but it is handy during debugging to view the numbers coming from the IMU sensors. These values are **NOT** calibrated.
- `simpleIMU` - Displays data from all 3 sensors of the MPU-6500, Gyro, Accelerometer and chip temperature. The results are printed to the Arduino IDE Serial Monitor every `displayPeriod` (default is 1 second). This makes it easier to read the results. The loop frequency is also printed. The sensor values are **NOT** calibrated.
- `polledIMU` - If you don't want to use interrupts to determine when data is available, then have a look at this example. It uses the `imu.dataAvailable()` method to determine when new sensor data has been loaded into the registers. This example prints out **calibrated** results.
- `calibratingBiasOffset` - Displays data from MPU-6500 after calibration. Keep the IMU still and flat during bias offset calibration.
- `xIMU3_Data_Visualisation` - This sketch reads calibrated data from the MPU-6500 and packages it up for streaming to the xio xIMU3 application. You will need to install the Reefwing_xIMU3 library in order to compile this sketch.

## Dependencies

This library needs the [Reefwing_imuTypes](https://github.com/Reefwing-Software/Reefwing-imuTypes) library installed as well. If you want to run the xIMU3_Data_Visualisation sketch, you will also need to install the [Reefwing_xIMU3](https://github.com/Reefwing-Software/Reefwing-xIMU3) library.

## More Information

For more information raise an issue on the [GitHub Repository](https://github.com/Reefwing-Software/MPU6x00) or read our Medium article about writing this library.