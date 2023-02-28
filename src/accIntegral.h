#include <imuFilter.h>

#ifndef accIntegral_h
#define accIntegral_h

//---------------- Class definition ------------------ 

class accIntegral: public imuFilter {
  private:
    // vectors
    vec3_t   vel = 0;           
    vec3_t   accel_mean = {0,0,0};     
    vec3_t   accel_last = {0,0,0};
    
    // scalars
    float    var_vel = 0;       
    float    var_acc = 0;     
    
  public:  
    void reset();
    
    vec3_t getVel();

    void update( vec3_t, vec3_t, vec3_t,
                 const float, 
                 const float, 
                 const float,
                 const float=DEFAULT_GAIN );

    void update( float, float, float, 
                 float, float, float, 
                 float, float, float, 
                 const float, 
                 const float, 
                 const float,
                 const float=DEFAULT_GAIN );
};

#endif
