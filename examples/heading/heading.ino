/*
 This sketch shows to perform vector products and rotate heading (yaw angle) of the estimated orientation.
 */

#include <basicMPU6050.h>       // Library for IMU sensor. See this link: https://github.com/RCmags/basicMPU6050
#include <imuFilter.h>

basicMPU6050<> imu;

// =========== Settings ===========
imuFilter fusion;
#define GAIN          0.1     /* Fusion gain, value between 0 and 1 - Determines orientation correction with respect to gravity vector */

void setup() {
  // Initialize filter: 
  fusion.setup( imu.ax(), imu.ay(), imu.az() );     

  // Calibrate imu
  imu.setup();
  imu.setBias();
                  
  // Rotate heading:
  float angle = 45 * DEG_TO_RAD;                // angle in radians to rotate heading about z-axis
  fusion.rotateHeading( angle, LARGE_ANGLE );   // Can choose LARGE_ANGLE or SMALL_ANGLE approximation

  Serial.begin(38400);
}

void loop() {  
  // Update filter:
  fusion.update( imu.gx(), imu.gy(), imu.gz(), imu.ax(), imu.ay(), imu.az(), GAIN );    

  // Display angles:
  Serial.print( fusion.pitch() );
  Serial.print( " " );
  Serial.print( fusion.yaw() );
  Serial.print( " " );
  Serial.print( fusion.roll() );
  Serial.println();
}
