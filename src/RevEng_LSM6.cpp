/**
 * RevEng_LSM6.cpp - Driver implementation for 
 *            LSM6DS33 iNEMO inertial module for Arduino
 *
 * Original code: Pololu  <inbox@pololu.com>
 * Full Scale API and SI unit conversion updates by:
 *       Aaron Crandall <acrandal@gmail.com>
 *
 */

#include <RevEng_LSM6.h>
#include <Wire.h>
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define DS33_SA0_HIGH_ADDRESS 0b1101011
#define DS33_SA0_LOW_ADDRESS  0b1101010

#define TEST_REG_ERROR -1

#define DS33_WHO_ID    0x69

// Accelerometer and Gyro scaling bitmasks for
//  CTRL1_XL and CTRL2_G registers
#define ACC_FS_XL2g  0b00000000   // FS_XL 00
#define ACC_FS_XL16g 0b00000100   // FS_XL 01
#define ACC_FS_XL4g  0b00001000   // FS_XL 10
#define ACC_FS_XL8g  0b00001100   // FS_XL 11

#define GYRO_FS_125dps  0b00000010 // FS_G 00
#define GYRO_FS_245dps  0b00000000 // FS_G 00
#define GYRO_FS_500dps  0b00000100 // FS_G 01
#define GYRO_FS_1000dps 0b00001000 // FS_G_10
#define GYRO_FS_2000dps 0b00001100 // FS_G 11

// Defined scaling factors for Gravities and Degrees Per second
//  -- found in LSM6D33 spec sheet, page 15
#define ACC_SCALE_FACTOR_2g  0.061    // Raw value * factor == mg
#define ACC_SCALE_FACTOR_4g  0.122
#define ACC_SCALE_FACTOR_8g  0.244
#define ACC_SCALE_FACTOR_16g 0.488

#define GYRO_SCALE_FACTOR_125dps   4.375  // Raw value * factor == mdps
#define GYRO_SCALE_FACTOR_245dps   8.750
#define GYRO_SCALE_FACTOR_500dps  17.500
#define GYRO_SCALE_FACTOR_1000dps 35.000
#define GYRO_SCALE_FACTOR_2000dps 70.000

#define G2MPS2 9.80665         // Scaling factor to convert Gravities to M/S^2


// Constructors ////////////////////////////////////////////////////////////////

LSM6::LSM6(void)
{
  _device = device_auto;

  io_timeout = 0;  // 0 = no timeout
  did_timeout = false;
}

// Public Methods //////////////////////////////////////////////////////////////

// Did a timeout occur in readAcc(), readGyro(), or read() since the last call to timeoutOccurred()?
bool LSM6::timeoutOccurred()
{
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;
}

void LSM6::setTimeout(uint16_t timeout)
{
  io_timeout = timeout;
}

uint16_t LSM6::getTimeout()
{
  return io_timeout;
}

bool LSM6::init(deviceType device, sa0State sa0)
{
  // perform auto-detection unless device type and SA0 state were both specified
  if (device == device_auto || sa0 == sa0_auto)
  {
    // check for LSM6DS33 if device is unidentified or was specified to be this type
    if (device == device_auto || device == device_DS33)
    {
      // check SA0 high address unless SA0 was specified to be low
      if (sa0 != sa0_low && testReg(DS33_SA0_HIGH_ADDRESS, WHO_AM_I) == DS33_WHO_ID)
      {
        sa0 = sa0_high;
        if (device == device_auto) { device = device_DS33; }
      }
      // check SA0 low address unless SA0 was specified to be high
      else if (sa0 != sa0_high && testReg(DS33_SA0_LOW_ADDRESS, WHO_AM_I) == DS33_WHO_ID)
      {
        sa0 = sa0_low;
        if (device == device_auto) { device = device_DS33; }
      }
    }

    // make sure device and SA0 were successfully detected; otherwise, indicate failure
    if (device == device_auto || sa0 == sa0_auto)
    {
      return false;
    }
  }

  _device = device;

  switch (device)
  {
    case device_auto:           // TODO: How should this be handled? - BUGFIX: done to pass CI now
    case device_DS33:
      address = (sa0 == sa0_high) ? DS33_SA0_HIGH_ADDRESS : DS33_SA0_LOW_ADDRESS;
      break;
  }

  return true;
}

/*
Enables the LSM6's accelerometer and gyro. Also:
- Sets sensor full scales (gain) to default power-on values, which are
  +/- 2 g for accelerometer and 245 dps for gyro
- Selects 1.66 kHz (high performance) ODR (output data rate) for accelerometer
  and 1.66 kHz (high performance) ODR for gyro. (These are the ODR settings for
  which the electrical characteristics are specified in the datasheet.)
- Enables automatic increment of register address during multiple byte access
Note that this function will also reset other settings controlled by
the registers it writes to.
*/
void LSM6::enableDefault(void)
{
  if (_device == device_DS33)
  {
    // Accelerometer

    // 0x80 = 0b10000000
    // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (+/-2 g full scale)
    writeReg(CTRL1_XL, 0x80);
    setAccScale( ACC2g );      // set accelerometer scale to +/-2 g full scale

    // Gyro

    // 0x80 = 0b010000000
    // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (245 dps)
    writeReg(CTRL2_G, 0x80);
    setGyroScale( G245dps );  // set gyroscope scale to 245 dps

    // Common

    // 0x04 = 0b00000100
    // IF_INC = 1 (automatically increment register address)
    writeReg(CTRL3_C, 0x04);
  }
}

/**
 * Set the Accelerometer Scale:
 *  ACC2g, ACC4g, ACC8g, ACC16g
 */
void LSM6::setAccScale( accScale scale )
{

  uint8_t curr_CTRL1_XL;
  curr_CTRL1_XL = readReg(CTRL1_XL);   // Read in current Accel config
                                       // curr_CTRL1_XL is a member variable to the LSM6 class
  curr_CTRL1_XL &= 0b11110011;         // Mask off the FS_XL bits

  switch( scale )                      // Choose new setting mask
  {
    case ACC2g:
      curr_AccScale = ACC_FS_XL2g;     // Set the FS_XL bits to 00
      curr_AccScaleFactor = ACC_SCALE_FACTOR_2g;
      break;
    case ACC16g:
      curr_AccScale = ACC_FS_XL16g;    // Set the FS_XL bits to 01
      curr_AccScaleFactor = ACC_SCALE_FACTOR_16g;
      break;
    case ACC4g:
      curr_AccScale = ACC_FS_XL4g;     // Set the FS_XL bits to 10
      curr_AccScaleFactor = ACC_SCALE_FACTOR_4g;
      break;
    case ACC8g:
      curr_AccScale = ACC_FS_XL8g;     // Set the FS_XL bits to 11
      curr_AccScaleFactor = ACC_SCALE_FACTOR_8g;
      break;
    default:
      Serial.println(" Invalid accelerometer scaling factor chosen.");
  }
  curr_CTRL1_XL |= curr_AccScale;
  writeReg(CTRL1_XL, curr_CTRL1_XL);   // Write new Accel configuration
}

/**
 *  Set the scale for the gyroscope. Options are:
 *   G125dps, G245dps, G500dps, G1000dps, G2000dps
 */
void LSM6::setGyroScale( gyroScale scale )
{
  uint8_t curr_CTRL2_G;
  curr_CTRL2_G = readReg(CTRL2_G);       // Read in current Gyro config
                                         // curr_CTRL2_G is a member variable to the LSM6 class
  curr_CTRL2_G &= 0b11110001;            // Mask off the FS_G bits

  switch( scale )                        // Choose new setting mask
  {
    case G125dps:
      curr_GyroScale = GYRO_FS_125dps;   // Set the FS_G bits to 001
      curr_GyroScaleFactor = GYRO_SCALE_FACTOR_125dps;
      break;
    case G245dps:
      curr_GyroScale = GYRO_FS_245dps;   // Set the FS_G bits to 000
      curr_GyroScaleFactor = GYRO_SCALE_FACTOR_245dps;
      break;
    case G500dps:
      curr_GyroScale = GYRO_FS_500dps;   // Set the FS_G bits to 010
      curr_GyroScaleFactor = GYRO_SCALE_FACTOR_500dps;
      break;
    case G1000dps:
      curr_GyroScale = GYRO_FS_1000dps;  // Set the FS_G bits to 100
      curr_GyroScaleFactor = GYRO_SCALE_FACTOR_1000dps;
      break;
    case G2000dps:
      curr_GyroScale = GYRO_FS_2000dps;  // Set the FS_G bits to 110
      curr_GyroScaleFactor = GYRO_SCALE_FACTOR_2000dps;
      break;
    default:
      Serial.println(" Invalid Gyroscope scaling factor chosen.");
  }
  curr_CTRL2_G |= curr_GyroScale;
  writeReg(CTRL2_G, curr_CTRL2_G);       // Write new Accel configuration
}


/**
 *  Calculate the accelerometer values as gravities (G)
 *   This is based on the currently set accelerometer scale (which gives mg)
 */
void LSM6::calcAccG(void)
{
  acc_g.x = a.x * curr_AccScaleFactor / 1000; // div 1000 to go from mg to g
  acc_g.y = a.y * curr_AccScaleFactor / 1000;
  acc_g.z = a.z * curr_AccScaleFactor / 1000; 
}


/**
 *  Fills the acc_mps2 vector with the current acceleration values in m/s^2
 *   ** Note: Could be made faster by copying the calcAccG code into here
 *            That would bypass 3 assignments and 3 floating point calculations
 */
void LSM6::calcAccMPS2(void)
{
  calcAccG();
  acc_mps2.x = acc_g.x * G2MPS2;  // Converts from Gravities to m/s^2
  acc_mps2.y = acc_g.y * G2MPS2;
  acc_mps2.z = acc_g.z * G2MPS2; 
}

/**
 *  Calculate the gyro values as degress per second
 *   This is based on the currently set gyro scale (which gives mdps)
 */
void LSM6::calcGyroDPS(void)
{
  gyro_dps.x = g.x * curr_GyroScaleFactor / 1000; // div 1000 -> mdps to dps
  gyro_dps.y = g.y * curr_GyroScaleFactor / 1000;  
  gyro_dps.z = g.z * curr_GyroScaleFactor / 1000;
}

/**
 *  Helper function to both read in IMU data, then calculate human-readable vals
 */
void LSM6::readCalc(void)
{
  read();        // Read in IMU
  calcAccMPS2(); // Calc m/s^2 - But also does Gs on the way
  calcGyroDPS(); // Calc DPS
}

/**
 *  Write to a single register at reg with a given byte
 */
void LSM6::writeReg(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  last_status = Wire.endTransmission();
}


/**
 *  Read out a single byte at a given register address
 */
uint8_t LSM6::readReg(uint8_t reg)
{
  uint8_t value;

  Wire.beginTransmission(address);
  Wire.write(reg);
  last_status = Wire.endTransmission();
  Wire.requestFrom(address, (uint8_t)1);
  value = Wire.read();
  Wire.endTransmission();

  return value;
}

// Reads the 3 accelerometer channels and stores them in vector a
void LSM6::readAcc(void)
{
  Wire.beginTransmission(address);
  // automatic increment of register address is enabled by default (IF_INC in CTRL3_C)
  Wire.write(OUTX_L_XL);
  Wire.endTransmission();
  Wire.requestFrom(address, (uint8_t)6);

  uint16_t millis_start = millis();
  while (Wire.available() < 6) {
    if (io_timeout > 0 && ((uint16_t)millis() - millis_start) > io_timeout)
    {
      did_timeout = true;
      return;
    }
  }

  uint8_t xla = Wire.read();
  uint8_t xha = Wire.read();
  uint8_t yla = Wire.read();
  uint8_t yha = Wire.read();
  uint8_t zla = Wire.read();
  uint8_t zha = Wire.read();

  // combine high and low bytes
  a.x = (int16_t)(xha << 8 | xla);
  a.y = (int16_t)(yha << 8 | yla);
  a.z = (int16_t)(zha << 8 | zla);
}

// Reads the 3 gyro channels and stores them in vector g
void LSM6::readGyro(void)
{
  Wire.beginTransmission(address);
  // automatic increment of register address is enabled by default (IF_INC in CTRL3_C)
  Wire.write(OUTX_L_G);
  Wire.endTransmission();
  Wire.requestFrom(address, (uint8_t)6);

  uint16_t millis_start = millis();
  while (Wire.available() < 6) {
    if (io_timeout > 0 && ((uint16_t)millis() - millis_start) > io_timeout)
    {
      did_timeout = true;
      return;
    }
  }

  uint8_t xlg = Wire.read();
  uint8_t xhg = Wire.read();
  uint8_t ylg = Wire.read();
  uint8_t yhg = Wire.read();
  uint8_t zlg = Wire.read();
  uint8_t zhg = Wire.read();

  // combine high and low bytes
  g.x = (int16_t)(xhg << 8 | xlg);
  g.y = (int16_t)(yhg << 8 | ylg);
  g.z = (int16_t)(zhg << 8 | zlg);
}

// Reads all 6 channels of the LSM6 and stores them in the object variables
void LSM6::read(void)
{
  readAcc();
  readGyro();
}

void LSM6::vector_normalize(vector<float> *a)
{
  float mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

// Private Methods //////////////////////////////////////////////////////////////

int16_t LSM6::testReg(uint8_t address, regAddr reg)
{
  Wire.beginTransmission(address);
  Wire.write((uint8_t)reg);
  if (Wire.endTransmission() != 0)
  {
    return TEST_REG_ERROR;
  }

  Wire.requestFrom(address, (uint8_t)1);
  if (Wire.available())
  {
    return Wire.read();
  }
  else
  {
    return TEST_REG_ERROR;
  }
}