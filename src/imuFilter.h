#include <Arduino.h>
#include <quaternion_type.h>

#ifndef imuFilter_h
#define imuFilter_h

//------------------ Common terms -------------------- 

#define INV_Q_VAL   1.414213            // Damping behavior of filter. A larger value leads to faster response but more oscillations.
#define QUAT_DIM    4
#define VEC_DIM     3

//--------------- Template Parameters ---------------- [ No characters after backlash! ]
 
#define TEMPLATE_TYPE   const float *ALPHA

#define TEMPLATE_INPUTS              ALPHA

//---------------- Class definition ------------------ 
                         
template<TEMPLATE_TYPE>
class imuFilter {
  private: 
    //float s[VEC_DIM] = {0};
    vec3_t s;
    //float q[QUAT_DIM] = {1, 0, 0, 0};
    quat_t q;
    
    uint32_t last_time = 0;
  
    // Quaternion operations
    //void multiplyQuaternion( float [] );
    //void normalizeQuaternion();
    float updateTimer();
      
  public:
    //imuFilter();
    //void setGain();
      
    // Vector functions:
    //void crossProduct( float [], float [] );
    //float dotProduct( float [], float [] );
    //void normalizeVector( float [] );

    // Initialization:
    void setup();
    void setup( float, float, float );

    // Heading estimate:
    void update( float, float, float );
    void update( float, float, float, float, float, float );
    void rotateHeading( const bool, float );

    //-- Fusion outputs:
    
    // Quaternion
    quat_t getQuat();

    // Axis projections:
    vec3_t getXaxis( const bool );
    vec3_t getYaxis( const bool );
    vec3_t getZaxis( const bool );
    vec3_t projectVector( const bool, vec3_t );
    
    // Euler Angles:
    float roll();
    float pitch();
    float yaw();
};

#include "imuFilter.tpp"

//----------------- Clearing labels ------------------

#undef TEMPLATE_TYPE
#undef TEMPLATE_INPUTS

#endif
