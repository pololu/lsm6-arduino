/*
 * Arduino example code for the LSM6DS33 sensor.
 *  -- Example shows using the data calculation APIs in detail
 * 
 * The sensor outputs provided by the library are the raw
 * 16-bit values obtained by concatenating the 8-bit high and
 * low accelerometer and gyro data registers. They can be
 * converted to units of g and dps (degrees per second) using
 * the conversion factors specified in the datasheet for your
 * particular device and full scale setting (gain).
 * 
 * The actual scaling values can be found in the LSM6D33 spec document (pg 15):
 *  https://www.pololu.com/file/download/LSM6DS33.pdf?file_id=0J1087
 * 
 * The full API for this LMS6 library has several interfaces for data collection and calculation:
 * void readAcc(void);    // puts raw sensor data into vector called 'a'
 * void readGyro(void);   // puts raw sensor data into vector called 'g'
 * void read(void);       // Does readAcc, then readGyro
 * 
 * void calcAccG(void);    // Call read(), then calcAccG to get gravities in acc_g vector
 * void calcAccMPS2(void); // Call read(), then calcAccMPS2 to get m/s^2 in acc_mps2 vector
 * void calcGyroDPS(void); // Call read(), then calcGyroDPS to get degress per second into gyro_dps vector
 * void readCalc(void);    // Call will do read(), then calc of G, m/s^2, and dps
 * 
 * Original code from Pololu <inbox@pololu.com>
 * Updates for Full Scale API by:
 *  Aaron S. Crandall, 2017 - acrandal@gmail.com
 */

#include <Wire.h>
#include <RevEng_LSM6.h>

LSM6 imu;


void setup()
{
  Serial.begin(115200);
  Wire.begin();

  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }

  imu.enableDefault();  // Sets accelerometer to +/- 2g and gyro to 245 dps scales
}


void loop()
{
  Serial.println("-------------------------------------------------------");
  imu.read();         // Read in raw data to imu.a and imu.g vectors

  imu.calcAccG();     // Do the calculations for gravities in imu.acc_g
  imu.calcAccMPS2();  // Do the calculations for m/s^2 in imu.acc_mps2

  Serial.print(" Accel(raw):\t");
  Serial.print(imu.a.x);
  Serial.print(", ");
  Serial.print(imu.a.y);
  Serial.print(", ");
  Serial.println(imu.a.z);

  Serial.print(" Accel(g):\t");
  Serial.print(imu.acc_g.x);
  Serial.print(", ");
  Serial.print(imu.acc_g.y);
  Serial.print(", ");
  Serial.println(imu.acc_g.z);

  Serial.print(" Accel(m/s^2):\t");
  Serial.print(imu.acc_mps2.x);
  Serial.print(", ");
  Serial.print(imu.acc_mps2.y);
  Serial.print(", ");
  Serial.println(imu.acc_mps2.z);


  imu.calcGyroDPS();  // Do the calculations for degrees per second in imu.gyro_dps

  Serial.print(" Gyro(raw):\t");
  Serial.print(imu.g.x);
  Serial.print(", ");
  Serial.print(imu.g.y);
  Serial.print(", ");
  Serial.println(imu.g.z);

  Serial.print(" Gyro(dps):\t");
  Serial.print(imu.gyro_dps.x);
  Serial.print(", ");
  Serial.print(imu.gyro_dps.y);
  Serial.print(", ");
  Serial.println(imu.gyro_dps.z);

  // Optionally, data & calculations in one call:  imu.readCalc();

  delay(100);
}
