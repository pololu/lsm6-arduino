#include <LSM6DS33.h>
#include <Wire.h>
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define SA0_HIGH_ADDRESS                0b1101011
#define SA0_LOW_ADDRESS                 0b1101010

#define TEST_REG_ERROR -1

#define WHO_ID    0x69

// Constructors ////////////////////////////////////////////////////////////////

LSM6DS33::LSM6DS33(void)
{
  _device = device_auto;

  io_timeout = 0;  // 0 = no timeout
  did_timeout = false;
}

// Public Methods //////////////////////////////////////////////////////////////

// Did a timeout occur in readAcc(), readGyro(), or read() since the last call to timeoutOccurred()?
bool LSM6DS33::timeoutOccurred()
{
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;
}

void LSM6DS33::setTimeout(unsigned int timeout)
{
  io_timeout = timeout;
}

unsigned int LSM6DS33::getTimeout()
{
  return io_timeout;
}

bool LSM6DS33::init(deviceType device, sa0State sa0)
{
  // perform auto-detection unless device type and SA0 state were both specified
  if (device == device_auto || sa0 == sa0_auto)
  {
	  
    // check for LSM303D if device is unidentified or was specified to be this type
    if (device == device_auto)
    {
      // check SA0 high address unless SA0 was specified to be low
      if (sa0 != sa0_low && testReg(SA0_HIGH_ADDRESS, WHO_AM_I) == WHO_ID)
      {
        // device responds to address 0b1101011; SA0 high
        device = device_DS33;
        sa0 = sa0_high;
      }
      // check SA0 low address unless SA0 was specified to be high
      else if (sa0 != sa0_high && testReg(SA0_LOW_ADDRESS, WHO_AM_I) == WHO_ID)
      {
        // device responds to address 0b1101010; SA0 low
        device = device_DS33;
        sa0 = sa0_low;
      }
    }
    
    // make sure device and SA0 were successfully detected; otherwise, indicate failure
    if (device == device_auto || sa0 == sa0_auto)
    {
      return false;
    }
  }
  
  _device = device;
  
  acc_address = gyro_address = (sa0 == sa0_high) ? SA0_HIGH_ADDRESS : SA0_LOW_ADDRESS;
  
  return true;
}

/*
Enables the LSM6DS33's accelerometer and gyro. Also:
- Sets sensor full scales (gain) to default power-on values, which are
  +/- 2 g for accelerometer and 245 dps for gyro
- Selects 1.66 kHz (high performance) ODR (output data rate) for accelerometer and 1.66 kHz (high performance)
  ODR for gyro. (These are the ODR settings for which the electrical characteristics are specified in
  the datasheets.)
- automatic increment of register address is enabled by default (IF_INC in CTRL3_C)
Note that this function will also reset other settings controlled by
the registers it writes to.
*/
void LSM6DS33::enableDefault(void)
{

  if (_device == device_DS33)
  {
    // Accelerometer

    // 0x80 = 0b10000000
    // ODR = 1000 1.66 kHz (high performance)
	// FS_XL = 00  ±2 g
    writeReg(CTRL1_XL, 0x80);
	
	//Gyro

    // 0x80 = 0b010000000
    // ODR = 1000 1.66 kHz (high performance)
	// FS_XL = 00  245 dps
    writeReg(CTRL2_G, 0x80);
	
	// 0x04 = 0b00000100
	// IF_INC = 1
	writeReg(CTRL3_C, 0x04);
  }
}

// Writes an accelerometer register
void LSM6DS33::writeAccReg(byte reg, byte value)
{
  Wire.beginTransmission(acc_address);
  Wire.write(reg);
  Wire.write(value);
  last_status = Wire.endTransmission();
}

// Reads an accelerometer register
byte LSM6DS33::readAccReg(byte reg)
{
  byte value;

  Wire.beginTransmission(acc_address);
  Wire.write(reg);
  last_status = Wire.endTransmission();
  Wire.requestFrom(acc_address, (byte)1);
  value = Wire.read();
  Wire.endTransmission();

  return value;
}

// Writes a gyro register
void LSM6DS33::writeGyroReg(byte reg, byte value)
{
  Wire.beginTransmission(gyro_address);
  Wire.write(reg);
  Wire.write(value);
  last_status = Wire.endTransmission();
}

// Reads a gyro register
byte LSM6DS33::readGyroReg(int reg)
{
  byte value;

  Wire.beginTransmission(gyro_address);
  Wire.write(reg);
  last_status = Wire.endTransmission();
  Wire.requestFrom(gyro_address, (byte)1);
  value = Wire.read();
  Wire.endTransmission();

  return value;
}

void LSM6DS33::writeReg(byte reg, byte value)
{
  // gyro address == acc_address so it doesn't really matter which one we use.
  writeAccReg(reg, value);
}

byte LSM6DS33::readReg(int reg)
{
  // gyro address == acc_address so it doesn't really matter which one we use.
  return readAccReg(reg);
}

// Reads the 3 accelerometer channels and stores them in vector a
void LSM6DS33::readAcc(void)
{
  Wire.beginTransmission(acc_address);
  // automatic increment of register address is enabled by default (IF_INC in CTRL3_C)
  Wire.write(OUTX_L_XL);
  Wire.endTransmission();
  Wire.requestFrom(acc_address, (byte)6);

  unsigned int millis_start = millis();
  while (Wire.available() < 6) {
    if (io_timeout > 0 && ((unsigned int)millis() - millis_start) > io_timeout)
    {
      did_timeout = true;
      return;
    }
  }

  byte xla = Wire.read();
  byte xha = Wire.read();
  byte yla = Wire.read();
  byte yha = Wire.read();
  byte zla = Wire.read();
  byte zha = Wire.read();

  // combine high and low bytes
  a.x = (int16_t)(xha << 8 | xla);
  a.y = (int16_t)(yha << 8 | yla);
  a.z = (int16_t)(zha << 8 | zla);
}

// Reads the 3 gyro channels and stores them in vector g
void LSM6DS33::readGyro(void)
{
  Wire.beginTransmission(gyro_address);
  // automatic increment of register address is enabled by default (IF_INC in CTRL3_C)
  Wire.write(OUTX_L_G);
  Wire.endTransmission();
  Wire.requestFrom(gyro_address, (byte)6);

  unsigned int millis_start = millis();
  while (Wire.available() < 6) {
    if (io_timeout > 0 && ((unsigned int)millis() - millis_start) > io_timeout)
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

// Reads all 6 channels of the LSM6DS33 and stores them in the object variables
void LSM6DS33::read(void)
{
  readAcc();
  readGyro();
}

void LSM6DS33::vector_normalize(vector<float> *a)
{
  float mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

// Private Methods //////////////////////////////////////////////////////////////

int LSM6DS33::testReg(byte address, regAddr reg)
{
  Wire.beginTransmission(address);
  Wire.write((byte)reg);
  if (Wire.endTransmission() != 0)
  {
    return TEST_REG_ERROR;
  }

  Wire.requestFrom(address, (byte)1);
  if (Wire.available())
  {
    return Wire.read();
  }
  else
  {
    return TEST_REG_ERROR;
  }
}