/*
 This sketch shows to integrate acceleration to obtain an estimate of velocity
*/

/* NOTE: The accelerometer MUST be calibrated to integrate its output. Adjust the 
   AX, AY, AND AZ offsets until the sensor reads (0,0,0) at rest. 
*/

#include <basicMPU6050.h>   // Library for IMU sensor. See this link: https://github.com/RCmags/basicMPU6050
#include <accIntegral.h>

// ========== IMU sensor ==========

// Gyro settings:
#define         LP_FILTER   3           // Low pass filter.                    Value from 0 to 6
#define         GYRO_SENS   0           // Gyro sensitivity.                   Value from 0 to 3
#define         ACCEL_SENS  0           // Accelerometer sensitivity.          Value from 0 to 3
#define         ADDRESS_A0  LOW         // I2C address from state of A0 pin.   A0 -> GND : ADDRESS_A0 = LOW
                                        //                                     A0 -> 5v  : ADDRESS_A0 = HIGH
// Accelerometer offset:
constexpr int   AX_OFFSET =  552;       // Use these values to calibrate the accelerometer. The sensor should output 1.0g if held level. 
constexpr int   AY_OFFSET = -241;       // These values are unlikely to be zero.
constexpr int   AZ_OFFSET = -3185;

//-- Set template parameters:

basicMPU6050<LP_FILTER,  GYRO_SENS,  ACCEL_SENS, ADDRESS_A0,
             AX_OFFSET,  AY_OFFSET,  AZ_OFFSET
            >imu;

// =========== Settings ===========
accIntegral fusion;
                                            //  Unit            
constexpr float GRAVITY = 9.81e3;           //  mm/s^2          Magnitude of gravity at rest. Determines units of velocity.
constexpr float SD_ACC  = 200;              //  mm/s^2          Standard deviation of acceleration. Deviations from zero are suppressed.
constexpr float SD_VEL  = 200;              //  mm/s            Standard deviation of velocity. Deviations from target value are suppressed.

void setup() {
  Serial.begin(38400);

  // calibrate IMU sensor
  imu.setup();
  imu.setBias();

  // initialize sensor fusion
  fusion.setup( imu.ax(), imu.ay(), imu.az() );   // set initial heading 
  fusion.reset();                                 // zero velocity estimate
}

void loop() {   
  /* NOTE: The heading must be updated along with the velocity estimate for accurate results.
           Use the following steps to ensure proper integration */
  
  // 1. measure state
  
  imu.updateBias(); 
  vec3_t accel = { imu.ax(), imu.ay(), imu.az() };
  vec3_t gyro = { imu.gx(), imu.gy(), imu.gz() };

  // 2. update heading

  fusion.update( gyro, accel );

  // 3. update velocity estimate

  vec3_t vel_t = {0,0,0};   // Known measured velocity (target state)
  vec3_t vel;               // placeholder variable 
  
  fusion.updateVel( gyro, accel, vel_t, SD_ACC, SD_VEL, GRAVITY );
  vel = fusion.getVel();
  
  // Display velocity components [view with serial plotter]
 
  Serial.print( vel.x, 2 );
  Serial.print( " " );
  Serial.print( vel.y, 2 );
  Serial.print( " " );
  Serial.print( vel.z, 2 );
  Serial.println();
}
