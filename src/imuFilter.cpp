#include "imuFilter.h"

//----------------- Initialization ------------------- 

float imuFilter::timeStep() {
  return dt;
}

void imuFilter::updateTimer() {
  uint32_t time_now = micros();
  uint32_t change_time = time_now - last_time;
  last_time = time_now;
  dt = float(change_time)*1e-6;         
}

void imuFilter::setup() {
  var = 0;
  q = {1,0,0,0};
  last_time = micros();
}

void imuFilter::setup( vec3_t accel ) { 
  setup();
  // set quaternion as vertical vector
  vec3_t v = accel.norm();                          // gravity vector
 
  float norm = v.x*v.x + v.y*v.y;                   
  float cosine = v.z/sqrt( norm + v.z*v.z ) * 0.5;  // vertical angle
  norm = sqrt( (0.5 - cosine)/norm );               // sine of half angle
 
  q.w = sqrt(0.5 + cosine);                         // quaternion components
  q.v = vec3_t(v.y*norm, -v.x*norm, 0);
}

void imuFilter::setup( float ax, float ay, float az ) {
  setup( vec3_t(ax, ay, az) );
} 

//---------------- Heading estimate ------------------ 

// Update heading with gyro:

void imuFilter::update( float gx, float gy, float gz ) {
  // Update Timer
  updateTimer();

  // Rotation increment
  vec3_t da = vec3_t(gx, gy, gz)*dt;
  quat_t dq; dq.setRotation(da, SMALL_ANGLE); 
      
  // Multiply and normalize Quaternion
  q *= dq;  
  q = q.norm();
}

// Update heading with gyro and accelerometer:

void imuFilter::update( float gx, float gy, float gz, 
                        float ax, float ay, float az, 
                        const float ALPHA, 
                        const float SD_ACC ) {  
  // Update Timer
  updateTimer();

  // check global acceleration [normalized]:
  vec3_t accel = {ax, ay, az};
  vec3_t acrel = q.rotate(accel, GLOBAL_FRAME);
  acrel.z -= 1; 
  
    // kalmal filter:
  const float INV_VAR = 1.0/( SD_ACC * SD_ACC );
  float error = acrel.dot(acrel);     // deviation from vertical
  
  float gain = 1.0/(1 + var*INV_VAR); // kalman gain
  var = error + var*gain;             // variance of error
    
  // error about vertical
  vec3_t vz = q.axisZ(LOCAL_FRAME);   
  vec3_t ve = accel.norm();
  ve = ve.cross(vz);

  // Rotation increment  
  gain *= ALPHA * dt;   
  vec3_t da = vec3_t(gx, gy, gz)*dt + ve*gain;       
  quat_t dq; dq.setRotation(da, SMALL_ANGLE);

  // Multiply and normalize Quaternion  
  q *= dq;
  q = q.norm();
}

// vector inputs:
void imuFilter::update( vec3_t gyro ) {
  update(gyro.x, gyro.y, gyro.z);
}

void imuFilter::update( vec3_t gyro, vec3_t accel,  
                        const float ALPHA, 
                        const float SD_ACC ) { 
  update( gyro.x , gyro.y , gyro.z, 
          accel.x, accel.y, accel.z,
          ALPHA, SD_ACC ); 
}

// Rotate heading by a large or small angle

void imuFilter::rotateHeading( float angle, const bool SMALL_ANG ) {
  // rotation about vertical
  vec3_t vp = q.axisZ(LOCAL_FRAME);  
  quat_t dq; dq.setRotation(vp, angle, SMALL_ANG);
  
  // Rotate quaternion    
  q *= dq;
}

//----------------- Fusion outputs ------------------- 

// Quaternion
quat_t imuFilter::getQuat() { 
  return q;
}

// Axis projections:

vec3_t imuFilter::getXaxis( const bool TO_WORLD ) { 
  return q.axisX(TO_WORLD);
}


vec3_t imuFilter::getYaxis( const bool TO_WORLD ) {
  return q.axisY(TO_WORLD);
}

vec3_t imuFilter::getZaxis( const bool TO_WORLD ) {
  return q.axisZ(TO_WORLD);
}

vec3_t imuFilter::projectVector( vec3_t vec, const bool TO_WORLD ) {
  return q.rotate(vec, TO_WORLD);
}

//------------------ Euler Angles ------------------- 

float imuFilter::roll() { // x-axis
  vec3_t v = q.v;
  float y = 2*( q.w*v.x + v.y*v.z );                        
  float x = 1 - 2*( v.x*v.x + v.y*v.y );
  return atan2( y, x );
}

float imuFilter::pitch() { // y-axis
  constexpr float PI_2 = PI*0.5;    
  vec3_t v = q.v;
  float a = 2*( v.y*q.w - v.z*v.x );    
  if( a > 1 ) {
    return PI_2; 
  } else if ( a < -1 ) {
    return -PI_2;
  } else {
    return asin(a);
  }
}

float imuFilter::yaw() { // z-axis
  vec3_t v = q.v;
  float y = 2*( v.z*q.w + v.x*v.y );
  float x = 1 - 2*( v.y*v.y + v.z*v.z );    
  return atan2( y, x );
}
