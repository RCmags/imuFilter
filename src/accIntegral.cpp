#include "accIntegral.h"

//----------------- Initialization ------------------- 

void accIntegral::reset() {
  // vectors
  vel = {0,0,0};      
  accel_mean = {0,0,0};  
  accel_last = {0,0,0};
  
  // scalars
  var_vel = 0;
  var_acc = 0;
}

//---------------- Velocity estimate ----------------- 

vec3_t accIntegral::getVel() {
  return vel;
}

// update velocity with imu measurements 
void accIntegral::update( vec3_t angvel, 
                          vec3_t accel, 
                          vec3_t vel_t, 
                          const float SD_ACC, 
                          const float SD_VEL,
                          const float ALPHA ) {
  // 0. Constants:
  const float VAR_ACC = SD_ACC*SD_ACC;
  const float VAR_VEL = SD_VEL*SD_VEL;
  const float VAR_COMB = VAR_ACC*VAR_VEL;
  
  // 1. Remove acceleration bias:
  imuFilter::update( angvel, accel, ALPHA, SD_ACC );
  quat_t qt = imuFilter::getQuat(); 
    
    // Remove centrifugal
  vec3_t vel_local = qt.rotate(vel, LOCAL_FRAME);
  accel -= vel_local.cross(angvel);
  
    // remove global gravity vector [normalized]
  accel = qt.rotate(accel, GLOBAL_FRAME);
  accel.z -= 1; 		
  
    // Update variance
  accel -= accel_mean;
    
  float acc_err = accel.dot(accel);
  float gain_acc = VAR_ACC/(VAR_ACC + var_acc);
  
  var_acc = acc_err + gain_acc*var_acc;        
  accel_mean += accel*gain_acc;
  
  // 2. Integrate acceleration:
    // Update Variance
  vec3_t dvel = vel_t - vel;
  
  float dvel_mag = dvel.dot(dvel);
                // gain affected by velocity and acceleration
  float gain_vel = VAR_COMB/(VAR_COMB + var_vel*var_acc); 
  
  var_vel = dvel_mag + gain_vel*var_vel;        
  
    // Integrate       
  float dt = imuFilter::timeStep();
  vel += 0.5*(accel + accel_last)*dt;     // trapezoidal rule
  vel += dvel*gain_vel;                   // update with target velocity
  
  accel_last = accel;
}

// verbose update
void accIntegral::update( float gx, float gy, float gz, 
                          float ax, float ay, float az, 
                          float vx, float vy, float vz, 
                          const float SD_ACC, 
                          const float SD_VEL,
                          const float ALPHA ) {                  
  vec3_t angvel = {gx, gy, gz};
  vec3_t accel  = {ax, ay, az};
  vec3_t vel_t  = {vx, vy, vz};
  return update( angvel, accel, vel_t, SD_ACC, SD_VEL, ALPHA );
}
