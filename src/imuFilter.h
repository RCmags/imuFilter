#include <Arduino.h>
#include <quaternion_type.h>

#ifndef imuFilter_h
#define imuFilter_h

//------------------ Coefficients -------------------- 

#define INV_Q_FACTOR        2           // Filter damping. A smaller value leads to faster response but more oscillations.
#define DEFAULT_GAIN        0.5         // Default filter gain. 
#define DEFAULT_SD_ACC      0.2         // Default standard deviation in acceleration. [g-force]

//---------------- Class definition ------------------ 
                         
class imuFilter {
  private: 
    quat_t q = {1,0,0,0};
    float var = 0;
    uint32_t last_time = 0;
    float dt;

    void updateTimer();
      
  public:
    // Initialization:
    void setup();
    void setup( float, float, float );
    void setup( vec3_t );

    // Heading estimate:
    void update( float, float, float );
    
    void update( float, float, float, 
                 float, float, float, 
                 const float=DEFAULT_GAIN, 
                 const float=DEFAULT_SD_ACC );

    void update( vec3_t );               
    void update( vec3_t, vec3_t, 
                 const float=DEFAULT_GAIN, 
                 const float=DEFAULT_SD_ACC );
                 
    void rotateHeading( float, const bool );

    float timeStep();

    //-- Fusion outputs:
    
    // Quaternion
    quat_t getQuat();

    // Axis projections:arduino
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
