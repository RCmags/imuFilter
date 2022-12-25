#include <imuFilter.h>

#ifndef accIntegral_h
#define accIntegral_h

//------------------ Coefficients -------------------- 

#define DEFAULT_GRAVITY     9.80665     // Default gravity magnitude [ m/s^2 ]

//---------------- Class definition ------------------ 

class accIntegral: public imuFilter {
  private:
    // vectors
    vec3_t   vel;           
    vec3_t   accel_mean;     
    vec3_t   accel_last;
    
    // scalars
    float    var_vel;       
    float    var_acc; 
    uint32_t time_last;      
    
    // functions
    float updateTimer();

  public:  
    void reset();
    vec3_t getVel();

    void updateVel( vec3_t, vec3_t, vec3_t, 
                    const float, 
                    const float, 
                    const float=DEFAULT_GRAVITY );

    void updateVel( float, float, float, 
                    float, float, float, 
                    float, float, float, 
                    const float, 
                    const float, 
                    const float=DEFAULT_GRAVITY );
};

#endif
