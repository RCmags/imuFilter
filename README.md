# imuFilter

This library contains a sensor fusion algorithm to combine the outputs of a 6-axis inertial measurement unit (IMU). It's based on a modified version of the Mahony filter that replaces the PI controller with something akin to a 2nd order low pass filter. The proportional term was removed and the integral term has been forced to decay in order to damp the system. The correction steps of each filter are shown below:

- Mahony:  
_integral += error.dt   
dtheta = theta_dot.dt + kp.error + ki.integral_  

- Modified Mahony:  
_integral += error.kp - integral.kc    
dtheta = theta_dot.dt + integral_  

The behavior of the modified filter is analogous to spring-mass system. Kp (stiffness) and Kc (damping) are related by the damping ratio Q which is held constant. This allows the behavior of the filter to be controlled via a single parameter.  

The filter uses a quaternion to encode rotations. This makes it easy to perform coordinate transformations. These include:  
- Transfor a vector from the local frame to the global (and vice versa)
- Get unit vectors of the X, Y and Z axes in the local or global frame.

Since a 6-axis IMU has no absolute reference for heading there is a function to rotate the orientation estimate about the yaw axis. Basic vector operations have been included to easily implement a heading correction algorithm should one have an additional sensor (such a magnetometer or some other absolute heading sensor).

For more information on the Mahony filter see these links:
- [IMU Data Fusing: Complementary, Kalman, and Mahony Filter](http://www.olliw.eu/2013/imu-data-fusing/#chapter23)
- [Mahony Filter](https://nitinjsanket.github.io/tutorials/attitudeest/mahony)
