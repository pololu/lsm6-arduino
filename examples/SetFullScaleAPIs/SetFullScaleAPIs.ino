/*
 * Arduino example code for the LSM6DS33 sensor.
 *  -- Example shows how to use the scale setting APIs
 * 
 * The sensor outputs provided by the library are the raw
 * 16-bit values obtained by concatenating the 8-bit high and
 * low accelerometer and gyro data registers. They can be
 * converted to units of g and dps (degrees per second) using
 * the conversion factors specified in the datasheet for your
 * particular device and full scale setting (gain).
 * 
 * The API for this LMS6 library has an API to read and calculate the IMU data:
 * void readCalc(void);    // Call will do read(), then calc of G, m/s^2, and dps
 * 
 * The API can also set the range (max values & resolution) 
 * of the accelerometer & gyroscope. These settings are done
 * via calls on the LSM6 object with these pre-defined types:
 * 
 * enum accScale { ACC2g, ACC4g, ACC8g, ACC16g }; 
 * enum gyroScale { G125dps, G245dps, G500dps, G1000dps, G2000dps };
 * 
 * void enableDefault(void);             // Gives default settings for scale
 * void setAccScale( accScale scale );   // Set accelerometer scale
 * void setGyroScale( gyroScale scale ); // Set gyro scale
 *
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

void printAcc(void){
  Serial.print(" Accel(raw):\t");
  Serial.print(imu.a.x);
  Serial.print(", ");
  Serial.print(imu.a.y);
  Serial.print(", ");
  Serial.println(imu.a.z);
}

void printGyro(void)
{
  Serial.print(" Gyro(raw):\t");
  Serial.print(imu.g.x);
  Serial.print(", ");
  Serial.print(imu.g.y);
  Serial.print(", ");
  Serial.println(imu.g.z);
}


void loop()
{
  Serial.println("-------------------------------------------------------");
  
  imu.setAccScale( ACC2g );  // Set scale to +/- 2g (2 gravities)
  imu.readCalc();       // Read in NEW data and do ISO value calculations
  printAcc();

  imu.setAccScale( ACC4g );  // Set scale to +/- 4g
  imu.readCalc();       // Read in NEW data and do ISO value calculations
  printAcc();

  imu.setAccScale( ACC8g );  // Set scale to +/- 8g
  imu.readCalc();       // Read in NEW data and do ISO value calculations
  printAcc();

  imu.setAccScale( ACC16g ); // Set scale to +/- 16g 
  imu.readCalc();       // Read in NEW data and do ISO value calculations
  printAcc();

  Serial.println();

  imu.setGyroScale( G125dps ); // Set scale to 125 degrees per second
  imu.readCalc();     // Read in NEW data and do ISO value calculations
  printGyro();

  imu.setGyroScale( G245dps ); // Set scale to 245 degrees per second
  imu.readCalc();     // Read in NEW data and do ISO value calculations
  printGyro();

  imu.setGyroScale( G500dps ); // Set scale to 500 degrees per second
  imu.readCalc();     // Read in NEW data and do ISO value calculations
  printGyro();

  imu.setGyroScale( G1000dps ); // Set scale to 1000 degrees per second
  imu.readCalc();     // Read in NEW data and do ISO value calculations
  printGyro();

  imu.setGyroScale( G2000dps ); // Set scale to 2000 degrees per second
  imu.readCalc();     // Read in NEW data and do ISO value calculations
  printGyro();

  delay(1000);
}
