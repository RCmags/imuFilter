/*
 This sketch shows to integrate acceleration to obtain an estimate of velocity
*/

/* NOTE: 
   1. The accelerometer MUST be calibrated to integrate its output. Adjust the 
   AX, AY, AND AZ offsets until the sensor reads (0,0,1) at rest.

   2. REQUIRED UNITS:
      I. The gyroscope must output: radians/second
      II. The accelerometer output: g-force         [outputs 1 when sensor is static]
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
constexpr int   AX_OFFSET = 0;       // Use these values to calibrate the accelerometer. The sensor should output 1.0g if held level. 
constexpr int   AY_OFFSET = 0;       // These values are unlikely to be zero.
constexpr int   AZ_OFFSET = 0;

//-- Set template parameters:

basicMPU6050<LP_FILTER,  GYRO_SENS,  ACCEL_SENS, ADDRESS_A0,
             AX_OFFSET,  AY_OFFSET,  AZ_OFFSET
            >imu;

// =========== Settings ===========
accIntegral fusion;

// Filter coefficients                       //  Unit           
constexpr float GRAVITY = 9.81e3;            //  mm/s^2             Magnitude of gravity at rest. Determines units of velocity. [UNITS MUST MATCH ACCELERATION]
constexpr float SD_ACC  = 1000 / GRAVITY;    //  mm/s^2 / g-force   Standard deviation of acceleration. Deviations from zero are suppressed.
constexpr float SD_VEL  = 200  / GRAVITY;    //  mm/s   / g-force   Standard deviation of velocity. Deviations from target value are suppressed.
constexpr float ALPHA   = 0.5;               //                     Gain of heading update - See example "output" for more information.

void setup() {
  Serial.begin(38400);
  delay(1000);

  // calibrate IMU sensor
  imu.setup();
  imu.setBias();

  // initialize sensor fusion
  //fusion.setup( imu.ax(), imu.ay(), imu.az() );   // ALWAYS set initial heading with acceleration 
  fusion.setup();
  
  //fusion.reset();  /* Use this function to zero velocity estimate */                               
}

void loop() {   
  imu.updateBias(); 
  
  // Measure state:  
  vec3_t accel = { imu.ax(), imu.ay(), imu.az() };    // g-unit
  vec3_t gyro = { imu.gx(), imu.gy(), imu.gz() };     // radians/second
 
  // Update heading and velocity estimate:
  
      // known measured velocity (target state). Estimate will be forced towards this vector
  vec3_t vel_t = {0,0,0};

  vel_t /= GRAVITY;                         // must have unit: g-force * second
  
      /* note: all coefficients are optional and have default values */
  fusion.update( gyro, accel, vel_t, SD_ACC, SD_VEL, ALPHA ); 

      // obtain velocity estimate
  vec3_t vel = fusion.getVel() * GRAVITY;   // scale by gravity for desired units
  
  // Display velocity components: [view with serial plotter]
  Serial.print( vel.x, 2 );
  Serial.print( " " );
  Serial.print( vel.y, 2 );
  Serial.print( " " );
  Serial.print( vel.z, 2 );  
  Serial.println();
}
