/**
 * RevEng_LSM6.h - API and defines for LSM6DS33 iNEMO inertial module for Arduino
 *
 * Original code: Pololu  <inbox@pololu.com>
 * Full Scale API and SI unit conversion updates by:
 *       Aaron S. Crandall <crandall@gonzaga.edu>
 *
 */


#ifndef __RE_LSM6DS33_h
#define __RE_LSM6DS33_h

#include <Arduino.h>

enum accScale { ACC2g, ACC4g, ACC8g, ACC16g }; 
enum gyroScale { G125dps, G245dps, G500dps, G1000dps, G2000dps };

class LSM6
{
  public:
    template <typename T> struct vector
    {
      T x, y, z;
    };

    enum deviceType { device_DS33, device_auto };
    enum sa0State { sa0_low, sa0_high, sa0_auto };

    // register addresses
    enum regAddr
    {
      FUNC_CFG_ACCESS   = 0x01,         // Enable embedded functions register

      FIFO_CTRL1        = 0x06,
      FIFO_CTRL2        = 0x07,
      FIFO_CTRL3        = 0x08,
      FIFO_CTRL4        = 0x09,
      FIFO_CTRL5        = 0x0A,
      ORIENT_CFG_G      = 0x0B,         // Gyroscope axis direction control (flip axis)

      INT1_CTRL         = 0x0D,         // HW int pin 1 control - what trips it: Step, SigMotion, Full, FIFO, Boot, Gyro, Accel
      INT2_CTRL         = 0x0E,         // HW int pin 2 control - what trips it: Temperature, Gyro, Accel data ready
      WHO_AM_I          = 0x0F,         // Device ID - set to 0x69
      CTRL1_XL          = 0x10,         // Linear accel control - output power data rate, full scale range, and filter bandwidth
      CTRL2_G           = 0x11,         // Angular gyro control - output power data rate, full scale range
      CTRL3_C           = 0x12,
      CTRL4_C           = 0x13,         // Control: inc gyro sleep mode, merge Int2 & Int 1, disable I2C
      CTRL5_C           = 0x14,
      CTRL6_C           = 0x15,         // Control: inc high performance op mode for accel
      CTRL7_G           = 0x16,         // Control: inc high performance op mode for gyro
      CTRL8_XL          = 0x17,
      CTRL9_XL          = 0x18,         // accel control: X,Y,Z axis enable/disable
      CTRL10_C          = 0x19,         // Control: Gyro X,Y,Z axis enable/disable, Embedded functions enable

      WAKE_UP_SRC       = 0x1B,         // Freefall detection flag, wake up events on X,Y,Z, sleep event
      TAP_SRC           = 0x1C,         // Tap, single, double detected - inc per axis
      D6D_SRC           = 0x1D,         // Orientation change detection (portrait, landscape, face-up/down)
      STATUS_REG        = 0x1E,         // Data available flags: temperature, gyro, accel

      OUT_TEMP_L        = 0x20,         // Temperature reading low byte
      OUT_TEMP_H        = 0x21,         // Temperature reading high byte
      OUTX_L_G          = 0x22,         // Gyro X
      OUTX_H_G          = 0x23,
      OUTY_L_G          = 0x24,         // Gyro Y
      OUTY_H_G          = 0x25,
      OUTZ_L_G          = 0x26,         // Gyro Z
      OUTZ_H_G          = 0x27,
      OUTX_L_XL         = 0x28,         // Accel X
      OUTX_H_XL         = 0x29,
      OUTY_L_XL         = 0x2A,         // Accel Y
      OUTY_H_XL         = 0x2B,
      OUTZ_L_XL         = 0x2C,         // Accel Z
      OUTZ_H_XL         = 0x2D,

      FIFO_STATUS1      = 0x3A,
      FIFO_STATUS2      = 0x3B,
      FIFO_STATUS3      = 0x3C,
      FIFO_STATUS4      = 0x3D,
      FIFO_DATA_OUT_L   = 0x3E,
      FIFO_DATA_OUT_H   = 0x3F,
      TIMESTAMP0_REG    = 0x40,
      TIMESTAMP1_REG    = 0x41,
      TIMESTAMP2_REG    = 0x42,

      STEP_TIMESTAMP_L  = 0x49,
      STEP_TIMESTAMP_H  = 0x4A,
      STEP_COUNTER_L    = 0x4B,
      STEP_COUNTER_H    = 0x4C,

      FUNC_SRC          = 0x53,         // Step counter status: steps Y/N, Significant motion Y/N, Tilt Y/N

      TAP_CFG           = 0x58,         // Pedometer en, tilt en, tap per axis enable
      TAP_THS_6D        = 0x59,         // Orientation change configs, inc degree change theshold
      INT_DUR2          = 0x5A,         // Tap detect timings: duration 4 double, quiet gap, shock overthreshold
      WAKE_UP_THS       = 0x5B,         // Wake events config: single/double tap, inactivity, threshold for wakeup
      WAKE_UP_DUR       = 0x5C,         // Freefall duration en, wake duration, sleep mode kickoff duration
      FREE_FALL         = 0x5D,         // Freefall duration setting, freefall threshold setting
      MD1_CFG           = 0x5E,         // INT 1 Register routing - inactive, single tap, wake, freefall, double tap, rotate, tilt, timer
      MD2_CFG           = 0x5F,         // INT 2 Register routing - ditto INT 1 options
    };

    vector<int16_t> a; // accelerometer readings
    vector<int16_t> g; // gyro readings

    vector<float> acc_g;    // accelerometer readings in G
    vector<float> acc_mps2; // accelerometer readings in m/s^2
    vector<float> gyro_dps; // gyroscope readings in Degress Per Second

    accScale curr_AccScale;     // Current Accelerometer Scale
    gyroScale curr_GyroScale;   // Current Gyroscope Scale

    float curr_AccScaleFactor;  // Scaling factor for current scale setting
    float curr_GyroScaleFactor;

    uint8_t last_status; // status of last I2C transmission

    LSM6(void);

    bool init(deviceType device = device_auto, sa0State sa0 = sa0_auto);
    deviceType getDeviceType(void) { return _device; }

    void enableDefault(void);
    void setAccScale( accScale scale );   // Set accelerometer scale
    void setGyroScale( gyroScale scale ); // Set gyro scale

    void writeReg(uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t reg);

    void readAcc(void);     // Read just the accelerometer into vector 'a'
    void readGyro(void);    // Read just the gyro into vector 'g'
    void read(void);        // Read both acc and gyro

    void calcAccG(void);    // Calculate accelerometer values in gravities
    void calcAccMPS2(void); // Calculate accelerometer values in m/s^2
    void calcGyroDPS(void); // Calculate gyroscope values in degrees per second
    void readCalc(void);    // Read in IMU and calculate G & DPS

    void setTimeout(uint16_t timeout);
    uint16_t getTimeout(void);
    bool timeoutOccurred(void);

    // vector functions
    template <typename Ta, typename Tb, typename To> static void vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out);
    template <typename Ta, typename Tb> static float vector_dot(const vector<Ta> *a, const vector<Tb> *b);
    static void vector_normalize(vector<float> *a);

  private:
    deviceType _device; // chip type
    uint8_t address;

    uint16_t io_timeout;
    bool did_timeout;

    int16_t testReg(uint8_t address, regAddr reg);
};


template <typename Ta, typename Tb, typename To> void LSM6::vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out)
{
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}

template <typename Ta, typename Tb> float LSM6::vector_dot(const vector<Ta> *a, const vector<Tb> *b)
{
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

#endif