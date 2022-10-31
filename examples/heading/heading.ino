/*
 This sketch shows to rotate the heading (yaw angle) of the estimated orientation.
 */

#include <basicMPU6050.h>       // Library for IMU sensor. See this link: https://github.com/RCmags/basicMPU6050
#include <imuFilter.h>

basicMPU6050<> imu;

imuFilter fusion;

void setup() {
  Serial.begin(38400);

  // Calibrate imu
  imu.setup();
  imu.setBias();
  
  // Initialize filter: 
  fusion.setup( imu.ax(), imu.ay(), imu.az() );     
                  
  // Rotate heading:
  float angle = 45 * DEG_TO_RAD;                // angle in radians to rotate heading about z-axis
  fusion.rotateHeading( angle, LARGE_ANGLE );   // Can choose LARGE_ANGLE or SMALL_ANGLE approximation
}

void loop() {  
  // Update filter:
  fusion.update( imu.gx(), imu.gy(), imu.gz(), imu.ax(), imu.ay(), imu.az() );    

  // Display angles:
  Serial.print( fusion.pitch() );
  Serial.print( " " );
  Serial.print( fusion.yaw() );
  Serial.print( " " );
  Serial.print( fusion.roll() );
  Serial.println();
}
