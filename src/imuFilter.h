#include "Arduino.h"

#ifndef imuFilter_h
#define imuFilter_h

//------------------ Common terms -------------------- 

#define INV_Q_VAL   1.414213            // Damping behavior of filter. A larger value leads to faster response but more oscillations.
#define QUAT_DIM    4
#define VEC_DIM     3

//--------------- Template Parameters ---------------- [ No characters after backlash! ]
 
#define TEMPLATE_TYPE           \
        const float     *ALPHA

#define TEMPLATE_INPUTS         \
                         ALPHA

//---------------- Class definition ------------------ 
                         
template<TEMPLATE_TYPE>
class imuFilter {
  private: 
    float s[VEC_DIM] = {0};
    float q[QUAT_DIM] = {1, 0, 0, 0};
    uint32_t last_time = 0;
  
    // Quaternion operations
    void multiplyQuaternion( float [] );
    void normalizeQuaternion();
    float updateTimer();
      
  public:   
    // Vector functions:
    void crossProduct( float [], float [] );
    float dotProduct( float [], float [] );
    void normalizeVector( float [] );

    // Initialization:
    void setup();
    void setup( float, float, float );

    // Heading estimate:
    void update( float, float, float );
    void update( float, float, float, float, float, float );
    void rotateHeading( const bool, float );

    //-- Fusion outputs:
    
    // Quaternion
    void getQuat( float r[] );

    // Axis projections:
    void getXaxis( const bool, float [] );
    void getYaxis( const bool, float [] );
    void getZaxis( const bool, float [] );
    void projectVector( const bool, float [] );
    
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
