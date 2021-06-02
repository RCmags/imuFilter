/*
 This sketch shows to get the axis projections from the orientation estimate. It also shows how to project a vector in the global or local reference frame.
 */

#include <imuFilter.h>
#include <basicMPU6050.h>       // Library for IMU sensor. See this link: https://github.com/RCmags/basicMPU6050

// Sensor fusion
constexpr float GAIN = 0.1;     // Fusion gain, value between 0 and 1 - Determines response of heading correction with respect to gravity.
imuFilter <&GAIN> fusion;

// Imu sensor
basicMPU6050<> imu;

// Display functions:
void printVector( float r[] ) {
  Serial.print( r[0], 2 );
  Serial.print( "," );
  Serial.print( r[1], 2 );
  Serial.print( "," );
  Serial.print( r[2], 2 );
}

void printQuat( float q[] ) {
  Serial.print( q[0] );
  Serial.print( "," );
  printVector( q + 1 );
}

void setup() {
  // Initialize filter: 
  fusion.setup( imu.ax(), imu.ay(), imu.az() );     

  // Calibrate imu
  imu.setup();
  imu.setBias();
  
  Serial.begin(38400);
}

void loop() {  
  // Update filter:
  fusion.update( imu.gx(), imu.gy(), imu.gz(), imu.ax(), imu.ay(), imu.az() );    

  float v[3] = { 1, 1, 0 };
  float x[3], y[3], z[3];

  // Unit vectors of rectangular coodinates
  #define TO_WORLD false         
                                // Project local axis to global reference frame = true 
                                // Project global axis to local reference frame = false
  
  fusion.getXaxis( TO_WORLD, x );
  fusion.getYaxis( TO_WORLD, y );
  fusion.getZaxis( TO_WORLD, z );
  fusion.projectVector( TO_WORLD, v );

  // Display vectors:
  Serial.print( " x = " );
  printVector( x );
  Serial.print( " | y = " );
  printVector( y );
  Serial.print( " | z = " );
  printVector( z );
  Serial.print( " | v = " );
  printVector( v );
  
  // Display quaternion
  float q[4];
  fusion.getQuat(q);
  
  Serial.print( " | q = " );
  printQuat( q );  
  Serial.println();
}
