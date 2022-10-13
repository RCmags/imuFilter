/*
 This sketch shows to initialize the filter and update it with the imu output. It also shows how to get the euler angles of the estimated heading orientation.
 */

#include <imuFilter.h>
#include <basicMPU6050.h>       // Library for IMU sensor. See this link: https://github.com/RCmags/basicMPU6050

// Sensor fusion
constexpr float GAIN = 0.1;     // Fusion gain, value between 0 and 1 - Determines orientation correction with respect to gravity vector. 
imuFilter <&GAIN> fusion;       // If set to 1 the gyroscope is dissabled. If set to 0 the accelerometer is dissabled (equivant to gyro-only).

// Imu sensor
basicMPU6050<> imu;

// Enable sensor fusion. Setting to "true" enables gravity correction.
#define FUSION  false

void setup() {
   #if FUSION
    // Set quaternion with gravity vector
    fusion.setup( imu.ax(), imu.ay(), imu.az() );     
  #else 
    // Just use gyro
    fusion.setup();                                   
  #endif

  // Calibrate imu
  imu.setup();
  imu.setBias();
   
  Serial.begin(38400);
}

void loop() {
  // Update filter:
  
  #if FUSION
    //Fuse gyro and accelerometer
    fusion.update( imu.gx(), imu.gy(), imu.gz(), imu.ax(), imu.ay(), imu.az() );    
  #else
    // Only use gyroscope
    fusion.update( imu.gx(), imu.gy(), imu.gz() );
  #endif

  // Display angles:
  Serial.print( fusion.pitch() );
  Serial.print( " " );
  Serial.print( fusion.yaw() );
  Serial.print( " " );
  Serial.print( fusion.roll() );
  Serial.println(); 
}
