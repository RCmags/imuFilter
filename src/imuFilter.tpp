//------------- Quaternion operations ----------------
 
template<TEMPLATE_TYPE>
void imuFilter<TEMPLATE_INPUTS>
::multiplyQuaternion( float r[] ) {
    float q0 = q[0];
    float q1 = q[1];
    float q2 = q[2];   
    q[0] = q0*r[0] - q1*r[1] - q2*r[2] - q[3]*r[3];
    q[1] = q0*r[1] + q1*r[0] + q2*r[3] - q[3]*r[2];
    q[2] = q0*r[2] - q1*r[3] + q2*r[0] + q[3]*r[1];
    q[3] = q0*r[3] + q1*r[2] - q2*r[1] + q[3]*r[0];
}

template<TEMPLATE_TYPE>
void imuFilter<TEMPLATE_INPUTS>
::normalizeQuaternion() {
    float factor = 1.0/sqrt( q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3] );
    q[0] *= factor;
    q[1] *= factor;
    q[2] *= factor;
    q[3] *= factor;       
}

template<TEMPLATE_TYPE>
float imuFilter<TEMPLATE_INPUTS>
::updateTimer() {
    uint32_t change_time = uint32_t( micros() - last_time );
    last_time = micros();
    return float(change_time)*1e-6;         
}

//---------------- Vector functions ------------------ 

template<TEMPLATE_TYPE>
void imuFilter<TEMPLATE_INPUTS>
::crossProduct( float v[], float r[] ) {
    float v0 = v[0];
    float v1 = v[1];
    v[0] = v1*r[2] - v[2]*r[1];
    v[1] = -v0*r[2] + v[2]*r[0];
    v[2] = v0*r[1] - v1*r[0];
}

template<TEMPLATE_TYPE>
float imuFilter<TEMPLATE_INPUTS>
::dotProduct( float v[], float r[] ) {
    return v[0]*r[0] + v[1]*r[1] + v[2]*r[2];
}

template<TEMPLATE_TYPE>
void imuFilter<TEMPLATE_INPUTS>
::normalizeVector( float v[] ) {
    float factor = 1.0/sqrt( dotProduct( v, v ) );
    v[0] *= factor;
    v[1] *= factor;
    v[2] *= factor;
}

//----------------- Initialization ------------------- 

template<TEMPLATE_TYPE>
void imuFilter<TEMPLATE_INPUTS>
::setup() {
    // Reset quaternion
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;

    // Initialize timer
    last_time = micros();
}

template<TEMPLATE_TYPE>
void imuFilter<TEMPLATE_INPUTS>
::setup( float ax, float ay, float az ) { 
    // Reset quaternion
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;

    // Initialize timer
    last_time = micros();

    // Normalize acceleration
    float v[VEC_DIM] = {ax, ay, az};
    normalizeVector( v );

    // Rotate quaternion onto vertical
    float norm   = v[0]*v[0] + v[1]*v[1];
    float cosine = v[2]/sqrt( norm + v[2]*v[2] ) * 0.5;
    norm = sqrt( (0.5 - cosine)/norm );
    //
    q[0] = sqrt( 0.5 + cosine );
    q[1] = v[1]*norm;
    q[2] = -v[0]*norm;
    q[3] = 0;
}

//---------------- Heading estimate ------------------ 

// Update heading with gyro:

template<TEMPLATE_TYPE>
void imuFilter<TEMPLATE_INPUTS>
::update( float gx, float gy, float gz ) {
    //-- Update Timer
    float dt = updateTimer()*0.5;

    //-- Rotation increment
    float dq[QUAT_DIM];
    float* dqv = dq + 1;
    dq[1] = gx*dt;
    dq[2] = gy*dt;
    dq[3] = gz*dt;
    dq[0] = 1 - 0.5*dotProduct( dqv, dqv );

    //-- Multiply Quaternion
    multiplyQuaternion( dq );

    //-- Normalize Quaternion
    normalizeQuaternion();      
}

// Update heading with gyro and accelerometer:

template<TEMPLATE_TYPE>
void imuFilter<TEMPLATE_INPUTS>
::update( float gx, float gy, float gz, float ax, float ay, float az ) {  
    //-- Update Timer
    float dt = updateTimer();

    //-- Normalize vector
    float v[VEC_DIM] = {ax, ay, az};
    normalizeVector( v );

    //-- Project vertical vector [ in Local frame ]
    float vp[VEC_DIM];
    getZaxis( false, vp );

    //-- Cross product [ in Local frame ]    
    crossProduct( v, vp ); 

    //-- Rotation increment
    constexpr float KP = (*ALPHA)*(*ALPHA);
    constexpr float KC = (*ALPHA)*INV_Q_VAL;
    //
    float KP_t = KP*dt;
    float KC_t = KC*sqrt(dt);
    s[0] += v[0]*KP_t - s[0]*KC_t; 
    s[1] += v[1]*KP_t - s[1]*KC_t; 
    s[2] += v[2]*KP_t - s[2]*KC_t;       
    //
    float dq[QUAT_DIM];
    float* dqv = dq + 1;
    dt *= 0.5;
    dq[1] = gx*dt + s[0]*0.5;
    dq[2] = gy*dt + s[1]*0.5;
    dq[3] = gz*dt + s[2]*0.5;
    dq[0] = 1 - 0.5*dotProduct( dqv, dqv );

    //-- Multiply Quaternion
    multiplyQuaternion( dq );

    //-- Normalize Quaternion
    normalizeQuaternion();
}

// Rotate heading by a large or small angle

template<TEMPLATE_TYPE>
void imuFilter<TEMPLATE_INPUTS>
::rotateHeading( const bool SMALL_ANG, float angle ) {
  float vp[VEC_DIM];
  getZaxis( false, vp );
  
  float dq[QUAT_DIM];
  angle *= 0.5;
  //
  if( SMALL_ANG ) {
    dq[1] = vp[0]*angle;
    dq[2] = vp[1]*angle;
    dq[3] = vp[2]*angle;
    dq[0] = 1 - 0.5*( angle*angle );
  } else {
    float sine = sin(angle);
    dq[1] = vp[0]*sine;
    dq[2] = vp[1]*sine;
    dq[3] = vp[2]*sine;
    dq[0] = cos(angle);
  }
  // Rotate quaternion
  multiplyQuaternion( dq );    
}

//----------------- Fusion outputs ------------------- 

// Quaternion
template<TEMPLATE_TYPE>
void imuFilter<TEMPLATE_INPUTS>
::getQuat( float r[] ) {
    r[0] = q[0];
    r[1] = q[1];
    r[2] = q[2];
    r[3] = q[3];
}

// Axis projections:

template<TEMPLATE_TYPE>
void imuFilter<TEMPLATE_INPUTS>
::getXaxis( const bool TO_WORLD, float v[] ) {
    float q0q3 = q[0]*q[3];
    float q0q2 = q[0]*q[2];

    // Project in World frame
    if( TO_WORLD == true ) {
    q0q3 = -q0q3;
    q0q2 = -q0q2;
    }
    v[0] = 2*( q[1]*q[1] + q[0]*q[0] ) - 1;
    v[1] = 2*( q[1]*q[2] - q0q3 );
    v[2] = 2*( q[1]*q[3] + q0q2 );
}

template<TEMPLATE_TYPE>
void imuFilter<TEMPLATE_INPUTS>
::getYaxis( const bool TO_WORLD, float v[] ) {
    float q0q3 = q[0]*q[3];
    float q0q1 = q[0]*q[1];

    // Project in world frame
    if( TO_WORLD == true ) {
    q0q3 = -q0q3;
    q0q1 = -q0q1;
    }
    v[0] = 2*( q[2]*q[1] + q0q3 );
    v[1] = 2*( q[2]*q[2] + q[0]*q[0] ) - 1;
    v[2] = 2*( q[2]*q[3] - q0q1 );
}

template<TEMPLATE_TYPE>
void imuFilter<TEMPLATE_INPUTS>
::getZaxis( const bool TO_WORLD, float v[] ) {
    float q0q2 = q[0]*q[2];
    float q0q1 = q[0]*q[1];

    // Project in world frame
    if( TO_WORLD == true ) {
    q0q2 = -q0q2;
    q0q1 = -q0q1;  
    }
    v[0] = 2*( q[3]*q[1] - q0q2 );
    v[1] = 2*( q[3]*q[2] + q0q1 );
    v[2] = 2*( q[3]*q[3] + q[0]*q[0] ) - 1;      
}

template<TEMPLATE_TYPE>
void imuFilter<TEMPLATE_INPUTS>
::projectVector( const bool TO_WORLD, float v[] ) {
    float vc[VEC_DIM] = { v[0], v[1], v[2] };
    float* qv = q + 1;

    // Vector products              
    float dot = 2*dotProduct( qv, v );
    float cross = 2*q[0];
    float vec = cross*q[0] - 1;
    crossProduct( vc, qv );

    // Project to world frame
    if( TO_WORLD == true ) {
    cross = -cross;
    }
    v[0] = cross*vc[0] + dot*qv[0] + vec*v[0];
    v[1] = cross*vc[1] + dot*qv[1] + vec*v[1];
    v[2] = cross*vc[2] + dot*qv[2] + vec*v[2];
}

//------------------ Euler Angles ------------------- 

template<TEMPLATE_TYPE>
float imuFilter<TEMPLATE_INPUTS>
::roll() {
    float y = 2*( q[0]*q[1] + q[2]*q[3] );
    float x = 1 - 2*( q[1]*q[1] + q[2]*q[2] );
    return atan2( y, x );
}

template<TEMPLATE_TYPE>
float imuFilter<TEMPLATE_INPUTS>
::pitch() {
    constexpr float PI_2 = PI*0.5;
    float a = 2*( q[2]*q[0] - q[3]*q[1] );
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
    float y = 2*( q[3]*q[0] + q[1]*q[2] );
    float x = 1 - 2*( q[2]*q[2] + q[3]*q[3] );
    return atan2( y, x );
}
