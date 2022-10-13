/*
 This sketch shows to perform vector products and rotate heading (yaw angle) of the estimated orientation.
 */

#include <imuFilter.h>
#include <basicMPU6050.h>       // Library for IMU sensor. See this link: https://github.com/RCmags/basicMPU6050

// Sensor fusion
constexpr float GAIN = 0.1;     // Fusion gain, value between 0 and 1 - Determines response of heading correction with respect to gravity.
imuFilter <&GAIN> fusion;

// Imu sensor
basicMPU6050<> imu;

void setup() {
  // Initialize filter: 
  fusion.setup( imu.ax(), imu.ay(), imu.az() );     

  // Calibrate imu
  imu.setup();
  imu.setBias();

  Serial.begin(38400);
                  
  // Rotate heading:
  float angle = 45 * DEG_TO_RAD;                // angle in radians to rotate heading about z-axis
  fusion.rotateHeading( LARGE_ANGLE, angle );   // Can choose LARGE_ANGLE or SMALL_ANGLE approximation
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
