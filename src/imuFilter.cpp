#include "imuFilter.h"

//----------------- Initialization ------------------- 

float imuFilter::updateTimer() {
  uint32_t change_time = uint32_t( micros() - last_time );
  last_time = micros();
  return float(change_time)*1e-6;         
}

void imuFilter::setup() {
  q = {1,0,0,0};
  last_time = micros();
}

void imuFilter::setup( float ax, float ay, float az ) { 
  setup();
  // set quaternion as vertical vector
  vec3_t v = vec3_t(ax, ay, az).norm();				// gravity vector
 
  float norm = v.x*v.x + v.y*v.y;					
  float cosine = v.z/sqrt( norm + v.z*v.z ) * 0.5;  // vertical angle
  norm = sqrt( (0.5 - cosine)/norm );               // sine of half angle
 
  q.w = sqrt(0.5 + cosine);							// quaternion components
  q.v = vec3_t(v.y*norm, -v.x*norm, 0);
}

//---------------- Heading estimate ------------------ 

// Update heading with gyro:

void imuFilter::update( float gx, float gy, float gz ) {
  // Update Timer
  float dt = updateTimer();

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
                        float alpha /*=DEFAULT_GAIN*/, 
                        const bool SCALE_GAIN /*=true*/ ) {  
  // Update Timer
  float dt = updateTimer();

  // error about vertical
  vec3_t vz = q.axisZ(LOCAL_FRAME);   
  vec3_t ve = vec3_t(ax, ay, az).norm();
  ve = ve.cross(vz);

  // filter gains
  float kp = (alpha*alpha);          // spring constant
  if( SCALE_GAIN ) {
    kp *= dt;
  }   
  float kc = sqrt(INV_Q_FACTOR*kp);  // damping constant           
  
  // Rotation increment
  s += ve*kp - s*kc;                 // target angular rate   
  vec3_t da = vec3_t(gx, gy, gz)*dt + s;       
  quat_t dq; dq.setRotation(da, SMALL_ANGLE);

  // Multiply and normalize Quaternion  
  q *= dq;
  q = q.norm();
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

float imuFilter::roll() {
  vec3_t v = q.v;
  float y = 2*( q.w*v.x + v.y*v.z );
  float x = 1 - 2*( v.x*v.x + v.y*v.y );
  return atan2( y, x );
}

float imuFilter::pitch() {
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

float imuFilter::yaw() {
  vec3_t v = q.v;
  float y = 2*( v.z*q.w + v.x*v.y );
  float x = 1 - 2*( v.y*v.y + v.z*v.z );    
  return atan2( y, x );
}
