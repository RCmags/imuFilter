/*
 This sketch shows to initialize the filter and update it with the imu output. It also shows how to get the euler angles of the estimated heading orientation.
 */

#include <basicMPU6050.h>       // Library for IMU sensor. See this link: https://github.com/RCmags/basicMPU6050
#include <imuFilter.h>

basicMPU6050<> imu;
   
// =========== Settings ===========
imuFilter fusion;

#define GAIN          0.1     /* Fusion gain, value between 0 and 1 - Determines orientation correction with respect to gravity vector. 
                                 If set to 1 the gyroscope is dissabled. If set to 0 the accelerometer is dissabled (equivant to gyro-only) */

#define SCALE_GAIN    true    /* Scale gain by timestep, true or false. Defaults to true. Multiplies the gain by the refresh time. This allows 
                                 the filter to behave the same if even the main loop execuses faster or slower */                          
            
#define FUSION        false   /* Enable sensor fusion. Setting to "true" enables gravity correction */

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
    /*NOTE: GAIN and SCALE_GAIN are optional parameters */
    fusion.update( imu.gx(), imu.gy(), imu.gz(), imu.ax(), imu.ay(), imu.az(), GAIN, SCALE_GAIN );  
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
