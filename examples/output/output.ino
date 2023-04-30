/*
 This sketch shows to initialize the filter and update it with the imu output. It also shows how to get the euler angles of the estimated heading orientation.
*/

/* NOTE: The accelerometer MUST be calibrated for the fusion to work. Adjust the 
   AX, AY, AND AZ offsets until the sensor reads (0,0,0) at rest. 
*/

#include <basicMPU6050.h>       // Library for IMU sensor. See this link: https://github.com/RCmags/basicMPU6050
#include <imuFilter.h>

// Gyro settings:
#define         LP_FILTER   3           // Low pass filter.                    Value from 0 to 6
#define         GYRO_SENS   0           // Gyro sensitivity.                   Value from 0 to 3
#define         ACCEL_SENS  0           // Accelerometer sensitivity.          Value from 0 to 3
#define         ADDRESS_A0  LOW         // I2C address from state of A0 pin.   A0 -> GND : ADDRESS_A0 = LOW
                                        //                                     A0 -> 5v  : ADDRESS_A0 = HIGH
// Accelerometer offset:
constexpr int   AX_OFFSET = 0;          // Use these values to calibrate the accelerometer. The sensor should output 1.0g if held level. 
constexpr int   AY_OFFSET = 0;          // These values are unlikely to be zero.
constexpr int   AZ_OFFSET = 0;

//-- Set the template parameters:

basicMPU6050<LP_FILTER,  GYRO_SENS,  ACCEL_SENS, ADDRESS_A0,
             AX_OFFSET,  AY_OFFSET,  AZ_OFFSET
            >imu;
   
// =========== Settings ===========
imuFilter fusion;

#define GAIN          0.5     /* Fusion gain, value between 0 and 1 - Determines orientation correction with respect to gravity vector. 
                                 If set to 1 the gyroscope is dissabled. If set to 0 the accelerometer is dissabled (equivant to gyro-only) */

#define SD_ACCEL      0.2     /* Standard deviation of acceleration. Accelerations relative to (0,0,1)g outside of this band are suppresed.
                                 Accelerations within this band are used to update the orientation. [Measured in g-force] */                          
            
#define FUSION        true    /* Enable sensor fusion. Setting to "true" enables gravity correction */

void setup() {
  Serial.begin(38400);
  
  // Calibrate imu
  imu.setup();
  imu.setBias();

  #if FUSION
    // Set quaternion with gravity vector
    fusion.setup( imu.ax(), imu.ay(), imu.az() );     
  #else 
    // Just use gyro
    fusion.setup();                                   
  #endif
}

void loop() {
  // Update filter:
  
  #if FUSION
    /* NOTE: GAIN and SD_ACCEL are optional parameters */
    fusion.update( imu.gx(), imu.gy(), imu.gz(), imu.ax(), imu.ay(), imu.az(), GAIN, SD_ACCEL );  
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
  
  // timestep
  Serial.print( " " );
  Serial.print( fusion.timeStep(), 6 );
  
  Serial.println(); 
}
