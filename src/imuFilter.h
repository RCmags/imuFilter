#include <Arduino.h>
#include <quaternion_type.h>

#ifndef imuFilter_h
#define imuFilter_h

//------------------ Coefficients -------------------- 

#define INV_Q_FACTOR        2           // Filter damping. A smaller value leads to faster response but more oscillations.
#define DEFAULT_GAIN        0.1         // Default filter gain. A faster value 
#define DEFAULT_SD          0.1         // Default standard deviation in acceleration. [g-force]

//---------------- Class definition ------------------ 
                         
class imuFilter {
  private: 
    vec3_t s;
    quat_t q;
    float var;
    uint32_t last_time;
    
    float updateTimer();
      
  public:
    // Initialization:
    void setup();
    void setup( float, float, float );

    // Heading estimate:
    void update( float, float, float );
    
    void update( float, float, float, 
                 float, float, float, 
                 const float=DEFAULT_GAIN, 
                 const float=DEFAULT_SD  );
                 
    void rotateHeading( float, const bool );

    //-- Fusion outputs:
    
    // Quaternion
    quat_t getQuat();

    // Axis projections:
    vec3_t getXaxis( const bool );
    vec3_t getYaxis( const bool );
    vec3_t getZaxis( const bool );
    vec3_t projectVector( vec3_t, const bool );
    
    // Euler Angles:
    float roll();
    float pitch();
    float yaw();
};

#endif
