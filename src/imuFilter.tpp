#include <quaternion_type.h>

//----------------- Initialization ------------------- 

template<TEMPLATE_TYPE>
float imuFilter<TEMPLATE_INPUTS>
::updateTimer() {
    uint32_t change_time = uint32_t( micros() - last_time );
    last_time = micros();
    return float(change_time)*1e-6;         
}

template<TEMPLATE_TYPE>
void imuFilter<TEMPLATE_INPUTS>
::setup() {
    // Reset quaternion
    q = {1,0,0,0};

    // Initialize timer
    last_time = micros();
}

template<TEMPLATE_TYPE>
void imuFilter<TEMPLATE_INPUTS>
::setup( float ax, float ay, float az ) { 
    // Reset quaternion
    q = {1,0,0,0};

    // Initialize timer
    last_time = micros();

    // Normalize acceleration
    vec3_t v = vec3_t(ax, ay, az).norm();
  
    // Rotate quaternion onto vertical
    float norm   = v.x*v.x + v.y*v.y;
    float cosine = v.z/sqrt( norm + v.z*v.z ) * 0.5;
    norm = sqrt( (0.5 - cosine)/norm );
    q.w = sqrt(0.5 + cosine);
    q.v = vec3_t(v.y*norm, -v.x*norm, 0);
}

//---------------- Heading estimate ------------------ 

// Update heading with gyro:

template<TEMPLATE_TYPE>
void imuFilter<TEMPLATE_INPUTS>
::update( float gx, float gy, float gz ) {
    // Update Timer
    float dt = updateTimer()*0.5;

    // Rotation increment
    vec3_t da = vec3_t(gx, gy, gz)*dt;
    float  w  = 1 - 0.5*da.dot(da);
    quat_t dq = quat_t(w, da);
        
    // Multiply and normalize Quaternion
    q *= dq;  
    q = q.norm();
}

// Update heading with gyro and accelerometer:

template<TEMPLATE_TYPE>
void imuFilter<TEMPLATE_INPUTS>
::update( float gx, float gy, float gz, float ax, float ay, float az ) {  
    // Update Timer
    float dt = updateTimer();

    // Normalize vector
    vec3_t v = vec3_t(ax, ay, az).norm();

    // Project vertical vector
    vec3_t vp = q.axisZ(LOCAL_FRAME);
    
    // Cross product [ in Local frame ]     
    v = v.cross(vp);

    // Rotation increment
    constexpr float KP = (*ALPHA)*(*ALPHA);
    constexpr float KC = (*ALPHA)*INV_Q_VAL;
    
        // low pass filter
    float KP_t = KP*dt;
    float KC_t = KC*sqrt(dt);       // scale coefficients

    dt *= 0.5;
    s += v*KP_t - s*KC_t;           // update filter rate
        
    vec3_t da = {gx, gy, gz};       
    da = da*dt + s*0.5;             // angle change
    
    float   w = 1 - 0.5*da.dot(da); 
    quat_t dq = quat_t(w, da);

    //-- Multiply and normalize Quaternion  
    q *= dq;
    q = q.norm();
}

// Rotate heading by a large or small angle

template<TEMPLATE_TYPE>
void imuFilter<TEMPLATE_INPUTS>
::rotateHeading( const bool SMALL_ANG, float angle ) {
  // rotation about vertical
  vec3_t vp = q.axisZ(LOCAL_FRAME);  
  quat_t dq; dq.setRotation(vp, angle, SMALL_ANG);
  
  // Rotate quaternion    
  q *= dq;
}

//----------------- Fusion outputs ------------------- 

// Quaternion
template<TEMPLATE_TYPE>
quat_t imuFilter<TEMPLATE_INPUTS>
::getQuat() { 
    return q;
}

// Axis projections:

template<TEMPLATE_TYPE>
vec3_t imuFilter<TEMPLATE_INPUTS>
::getXaxis( const bool TO_WORLD ) { 
    return q.axisX(TO_WORLD);
}

template<TEMPLATE_TYPE>
vec3_t imuFilter<TEMPLATE_INPUTS>
::getYaxis( const bool TO_WORLD ) {
    return q.axisY(TO_WORLD);
}

template<TEMPLATE_TYPE>
vec3_t imuFilter<TEMPLATE_INPUTS>
::getZaxis( const bool TO_WORLD ) {
    return q.axisZ(TO_WORLD);
}

template<TEMPLATE_TYPE>
vec3_t imuFilter<TEMPLATE_INPUTS>
::projectVector( const bool TO_WORLD, vec3_t vec ) {
    return q.rotate(vec, TO_WORLD);
}

//------------------ Euler Angles ------------------- 

template<TEMPLATE_TYPE>
float imuFilter<TEMPLATE_INPUTS>
::roll() {
    vec3_t v = q.v;
    float y = 2*( q.w*v.x + v.y*v.z );
    float x = 1 - 2*( v.x*v.x + v.y*v.y );
    return atan2( y, x );
}

template<TEMPLATE_TYPE>
float imuFilter<TEMPLATE_INPUTS>
::pitch() {
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

template<TEMPLATE_TYPE>
float imuFilter<TEMPLATE_INPUTS>
::yaw() {
    vec3_t v = q.v;
    float y = 2*( v.z*q.w + v.x*v.y );
    float x = 1 - 2*( v.y*v.y + v.z*v.z );    
    return atan2( y, x );
}
