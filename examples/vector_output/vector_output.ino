/*
 This sketch shows to get the axis projections from the orientation estimate. It also shows how to project a vector in the global or local reference frame.
 */

#include <basicMPU6050.h>       // Library for IMU sensor. See this link: https://github.com/RCmags/basicMPU6050
#include <imuFilter.h>

basicMPU6050<> imu;

// =========== Settings ===========
imuFilter fusion;
#define GAIN          0.1     /* Fusion gain, value between 0 and 1 - Determines orientation correction with respect to gravity vector */

// ========= functions ===========

void printVector( vec3_t r ) {
  Serial.print( r.x, 2 );
  Serial.print( "," );
  Serial.print( r.y, 2 );
  Serial.print( "," );
  Serial.print( r.z, 2 );
}

void printQuat( quat_t q ) {
  Serial.print( q.w );
  Serial.print( "," );
  printVector( q.v );
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
  fusion.update( imu.gx(), imu.gy(), imu.gz(), imu.ax(), imu.ay(), imu.az(), GAIN );    

  // Unit vectors of rectangular coodinates [Choose between GLOBAL_FRAME and LOCAL_FRAME]
  vec3_t x = fusion.getXaxis(GLOBAL_FRAME);
  vec3_t y = fusion.getYaxis(GLOBAL_FRAME);
  vec3_t z = fusion.getZaxis(GLOBAL_FRAME);
  
  const vec3_t VEC = {1, 1, 0};
  vec3_t v = fusion.projectVector(VEC, GLOBAL_FRAME);

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
  Serial.print( " | q = " );
  printQuat( fusion.getQuat() );  
  Serial.println();
}
