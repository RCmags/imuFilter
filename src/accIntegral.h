#include <imuFilter.h>

#ifndef accIntegral_h
#define accIntegral_h

//------------------ Coefficients -------------------- 

#define DEFAULT_SD_VEL     0.1         // Default standard deviation in velocity. [g-force * second]

//---------------- Class definition ------------------ 

class accIntegral: public imuFilter {
  private:
    // vectors
    vec3_t   vel = {0,0,0};           
    vec3_t   accel_mean = {0,0,0};     
    vec3_t   accel_last = {0,0,0};
    
    // scalars
    float    var_vel = 0;       
    float    var_acc = 0;     
    
  public:  
    void reset();
    
    vec3_t getVel();

    void update( vec3_t, vec3_t, vec3_t,
                 const float=DEFAULT_SD_ACC, 
                 const float=DEFAULT_SD_VEL,
                 const float=DEFAULT_GAIN );

    void update( float, float, float, 
                 float, float, float, 
                 float, float, float,  
                 const float=DEFAULT_SD_ACC, 
                 const float=DEFAULT_SD_VEL,
                 const float=DEFAULT_GAIN );
};

#endif
