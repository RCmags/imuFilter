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

// Display function:
void printVector( float r[] ) {
  Serial.print( r[0], 2 );
  Serial.print( "," );
  Serial.print( r[1], 2 );
  Serial.print( "," );
  Serial.print( r[2], 2 );
  Serial.println();
}

void setup() {
  // Initialize filter: 
  fusion.setup( imu.ax(), imu.ay(), imu.az() );     

  // Calibrate imu
  imu.setup();
  imu.setBias();

  Serial.begin(38400);

  // Vector operations:
  float v1[] = { 3, 1, -1 };                    // Input Vector 
  float axis_y[3], axis_z[3];
  
  fusion.getYaxis( true, axis_y );              // Vectors to operate on [global axes] 
  fusion.getZaxis( true, axis_z );              

  fusion.crossProduct( v1, axis_y );            // Cross product: V = V cross R ; Output is stored in V 
  float v2[] = { v1[0], v1[1], v1[2] };         // Store product
  
  fusion.normalizeVector( v1 );                 // Norm: V = V/|V| ; Output is stored in V 
  
  float dot = fusion.dotProduct( v1, axis_z );  // Dot product: Input order does not matter   
                 
  // Rotate heading:
  #define SMALL_ANG false  
                          // Small angle approximation = true 
                          // Exact angle rotation = false 
  fusion.rotateHeading( SMALL_ANG, dot );

  // Display results:
  Serial.print( "y = " ); 
  printVector( axis_y );
  Serial.print( "v2 = " ); 
  printVector( v2 );
  Serial.print( "v1 = " ); 
  printVector( v1 );
  Serial.print( "dot = ");
  Serial.println( dot );

  // Wait for output to be read
  delay(10000);
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
