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
  time_last = micros();
}

//---------------------- Other -----------------------  

vec3_t accIntegral::getVel() {
  return vel;
}

float accIntegral::updateTimer() {
  uint32_t time_now = micros();
  uint32_t change_time = time_now - time_last;
  time_last = time_now;
  return float(change_time)*1e-6;         
}

//---------------- Velocity estimate ----------------- 

// update velocity with imu measurements 
void accIntegral::updateVel( vec3_t angvel, 
                             vec3_t accel, 
                             vec3_t vel_t, 
                             const float SD_ACC, 
                             const float SD_VEL, 
                             const float GRAVITY ) {
  // 0. Constants:
  const float VAR_ACC = SD_ACC*SD_ACC;
  const float VAR_VEL = SD_VEL*SD_VEL;
  const float VAR_COMB = VAR_ACC*VAR_VEL;
  
  // 1. Remove acceleration bias:
  quat_t qt = this->getQuat(); 
  accel *= GRAVITY;               // convert to given units
  
    // Remove centrifugal
  vec3_t vel_local = qt.rotate(vel, LOCAL_FRAME);
  accel -= vel_local.cross(angvel);
  
    // remove global gravity vector 
  accel = qt.rotate(accel, GLOBAL_FRAME);
  accel.z -= GRAVITY;
  
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
  float gain_vel = VAR_COMB/(VAR_COMB + var_vel*var_acc); // gain affected by velocity and acceleration
  
  var_vel = dvel_mag + gain_vel*var_vel;        
  
    // Integrate       
  float dt = updateTimer();
  vel += 0.5*(accel + accel_last)*dt;     // trapezoidal rule
  vel += dvel*gain_vel;                   // update with target velocity
  
  accel_last = accel;
}

// verbose update
void accIntegral::updateVel( float gx, float gy, float gz, 
                             float ax, float ay, float az, 
                             float vx, float vy, float vz, 
                             const float SD_ACC, 
                             const float SD_VEL, 
                             const float GRAVITY ) {                  
  vec3_t angvel = {gx, gy, gz};
  vec3_t accel  = {ax, ay, az};
  vec3_t vel_t  = {vx, vy, vz};
  return updateVel( angvel, accel, vel_t, SD_ACC, SD_VEL, GRAVITY );
}
