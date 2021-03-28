# LSM6 (LSM6DS33) library for Arduino

Version: 2.0.0<br>
Release date: 2021 March 27<br>
Released here: [RevEng_LSM6DS33 GitHub](https://github.com/acrandal/RevEng_LSM6DS33)<br>
Based on the Pololu LSM6 driver: [www.pololu.com](https://www.pololu.com/)

## Summary

This is a library for the Arduino IDE/CLI that helps interface with ST Microelectronics' LSM6DS33 iNEMO 6DoF inertial measurement unit (IMU).
The library makes it simple to configure the LSM6DS33 and read the accelerometer and gyro data through I&sup2;C.
This RevEng version of the driver also allows for setting the full scale ranges of the accelerometer and gyroscope to use the sensor in a wide range of applications.


## Device documentation

ST Microelectronics Documentation:
* Main web page: https://www.st.com/en/mems-and-sensors/lsm6ds33.html
* Sensor device datasheet: https://www.st.com/resource/en/datasheet/lsm6ds33.pdf


## Supported platforms

This library is designed to work with the Arduino IDE/CLI versions 1.6.x or later.  It has not been tested with earlier versions.
This library should support any Arduino-compatible board.


## Getting started

### Hardware

An [LSM6DS33 carrier](https://www.pololu.com/product/2736) can be purchased from Pololu's website.  Before continuing, careful reading of the [product page](https://www.pololu.com/product/2736) as well as the LSM6 datasheet and application note is recommended.



Make the following connections between the Arduino and the LSM6 board:

#### 5V Arduino boards

(including Arduino Uno, Leonardo, Mega; Pololu A-Star 32U4)

    Arduino   LSM6 board
    -------   ----------
         5V - VIN
        GND - GND
        SDA - SDA
        SCL - SCL

#### 3.3V Arduino boards

(including Arduino Due)

    Arduino   LSM6 board
    -------   ----------
        3V3 - VIN
        GND - GND
        SDA - SDA
        SCL - SCL

### Software

If you are using version 1.6.2 or later of the [Arduino software (IDE)](http://www.arduino.cc/en/Main/Software), you can use the Library Manager to install this library:

1. In the Arduino IDE, open the "Sketch" menu, select "Include Library", then "Manage Libraries...".
2. Search for "LSM6".
3. Click the LSM6 entry in the list.
4. Click "Install".

If this does not work, you can manually install the library:

1. Download the [latest release archive from GitHub](https://github.com/pololu/lsm6-arduino/releases) and decompress it.
2. Rename the folder "lsm6-arduino-master" to "LSM6".
3. Move the "LSM6" folder into the "libraries" directory inside your Arduino sketchbook directory.  You can view your sketchbook location by opening the "File" menu and selecting "Preferences" in the Arduino IDE.  If there is not already a "libraries" folder in that location, you should make the folder yourself.
4. After installing the library, restart the Arduino IDE.

## Examples

Example sketches are available that shows how to use the library. You can access it from the Arduino IDE by opening the "File" menu, selecting "Examples", and then selecting "RevEng_LSM6D33". If you cannot find the example, the library was probably installed incorrectly and you should retry the installation instructions above.

## Library reference

* `vector<int16_t> a`<br>
  The last values read from the accelerometer.

* `vector<int16_t> g`<br>
  The last values read from the gyro.

* `vector<float> acc_g`<br>
  The last values read from the accelerometer converted to gravities.

* `vector<float> acc_mps2`<br>
  The last values read from the accelerometer converted to m/s^2.

* `vector<float> gyro_dps`<br>
  The last values read from the gyroscope converted to degress per second.

* `uint8_t last_status`<br>
  The status of the last I&sup2;C write transmission. See the [`Wire.endTransmission()` documentation](http://arduino.cc/en/Reference/WireEndTransmission) for return values.

* `LSM6(void)`<br>
  Constructor.

* `bool init(deviceType device, sa0State sa0)`<br>
  Initializes the library with the device being used (`device_DS33` or `device_auto`) and the state of the SA0 pin (`sa0_low`, `sa0_high`, or `sa0_auto`), which determines the least significant bit of the I&sup2;C slave address. Constants for these arguments are defined in LSM6.h. Both of these arguments are optional; if they are not specified, the library will try to automatically detect the device address. A boolean is returned indicating whether the type of LSM6 device was successfully determined (if necessary).

* `deviceType getDeviceType(void)`<br>
  Returns the device type specified to or detected by `init()`.

* `void enableDefault(void)`<br>
  Turns on the accelerometer and gyro and enables a consistent set of default settings.

  This function will reset the accelerometer to &plusmn;2&nbsp;g full scale and the gyro to &plusmn;245&nbsp;dps. See the comments in LSM6.cpp for a full explanation of the settings.

* `void setAccScale( accScale scale )`<br>
  Sets the acclerometer scale from the various &plusmn; gravity ranges available. The options for accScale are: ACC2g, ACC4g, ACC8g, ACC16g.

* `void setGyroScale( gyroScale scale )`<br>
  Sets the gyro scale from the various ranges for degress per second available. The options for gyroScale are: G125dps, G245dps, G500dps, G1000dps, G2000dps.

* `void writeReg(uint8_t reg, uint8_t value)`<br>
  Writes a sensor register with the given value.

  Register address constants are defined by the regAddr enumeration type in LSM6.h.<br>
  Example use: `imu.writeReg(LSM6::CTRL1_XL, 0x80);`

* `uint8_t readReg(uint8_t reg)`<br>
  Reads a sensor register and returns the value read.

* `void readAcc(void)`<br>
  Takes a reading from the accelerometer and stores the values in the vector `a`. Conversion of the readings to units of g depends on the accelerometer's selected gain (full scale setting).

* `void readGyro(void)`<br>
  Takes a reading from the gyro and stores the values in the vector `g`. Conversion of the readings to units of dps (degrees per second) depends on the gyro's selected gain (full scale setting).

* `void read(void)`<br>
  Takes a reading from both the accelerometer and gyro and stores the values in the vectors `a` and `g`.

* `void calcAccG(void)`<br>
  Uses the currently stored values in the vector `a` to calculate their equivalent values in gravities. This calculation takes into account the accelerometer's selected gain (full scale setting). These values are stored in a member vector called `acc_g`.

* `void calcAccMPS2(void)`<br>
  Uses the currently stored values in the vector `a` to calculate their equivalent values in meters per second per second (m/s^2). This calculation takes into account the accelerometer's selected gain (full scale setting). These values are stored in a member vector called `acc_mps2`.

* `void calcGyroDPS(void)`<br>
  Uses the currently stored values in the vector `g` to calculate their equivalent values in degrees per second (dps). This calculation takes into account the gyroscope's selected gain (full scale setting). These values are stored in a member vector called `gyro_dps`.

* `void readCalc(void)`<br>
  Takes a reading from both the accelerometer and gyro and stores the values in the vectors `a` and `g`, then it runs calculates Gravities, m/s^2, and degrees per second, which fill the `acc_g`, `acc_mps2,` and `gyro_dps` vectors.

* `void setTimeout(uint16_t timeout)`<br>
  Sets a timeout period in milliseconds after which the read functions will abort if the sensor is not ready. A value of 0 disables the timeout.

* `uint16_t getTimeout(void)`<br>
  Returns the current timeout period setting.

* `bool timeoutOccurred(void)`<br>
  Indicates whether a read timeout has occurred since the last call to `timeoutOccurred()`.


## Limitations

* This driver does not support a SPI bus interface with the LSM6.

## Version history

* 2.0.0 (2021 Mar 27): Initial RevEng updates with Full Scale APIs - Aaron S. Crandall \<crandall@gonzaga.edu>
* 1.0.0 (2016 Jan 19): Original release.
