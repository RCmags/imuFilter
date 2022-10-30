# imuFilter
This library fuses the outputs of an inertial measurement unit (IMU) and stores the heading as a quaternion. It uses a _modified Mahony_ filter that replaces the PI controller with a damped 2nd-order system. The proportional term was removed and the integral term was forced to decay to damp the system. The correction steps are as follows:

__Mahony:__  
$\ E_{k} = \theta_{accel} - \theta_{k-1} $

$\ I_{k} = I_{k-1} + {E_{k}}{\Delta t} $

$\ \theta_{k} = \theta_{k-1} + \dot{\theta}{\Delta t} + K_{p}E_{k} + K_{i}I_{k} $

__Modified Mahony:__ (this filter)  
$\ E_{k} = \theta_{accel} - \theta_{k-1} $

$\ I_{k} = K_{p}{E_{k}} + (1 - K_{c})I_{k-1} $

$\ \theta_{k} = \theta_{k-1} + \dot{\theta}{\Delta t} + I_{k} $  

The behavior of the modified filter is analogous to spring-mass system. Kp (stiffness) and Kc (damping) are related by the [Q-factor](https://en.wikipedia.org/wiki/Q_factor). This value is held constant, so the behavior of the filter is controlled by a single parameter (Kp):  

$\ K_{c} = \sqrt{ K_{p}/Q } $

To improve the response of the filter, the acceleration is checked to see if it lies within a given deviation from vertical. If the acceleration is within this band it will be used to re-adjust the heading. However, if the acceleration lies outside of this band the correction is suppressed. This is accomplished with a kalman filter that uses the deviation from vertical to update the acceleration's variance. This value then is used to set the gain needed to correct the heading:  

$\ \overrightarrow{a_{rel}} = \overrightarrow{a_{local}} - (0,0,1) $

$\ K_{\sigma} = 1/(1 + \frac{ {\sigma}^2 }{ {\sigma}_{acc}^2 } ) $

$\ {\sigma}^2 = | \overrightarrow{a_{rel}} |^2 + K_{\sigma}{\sigma}^2 $ 

$\ E_{k} = K_{\sigma} E_{k} $

As the filter uses a quaternion to encode rotations, it's easy to perform coordinate transformations. The library has functions to:
- Transfor a vector to the local or global frame.
- Get the unit vectors of the X, Y and Z axes in the local or global frame.

Moreover, since a 6-axis IMU (gyro-accelerometer) cannot measure an absolute heading, a function is included to rotate the orientation about the vertical (yaw) axis. One can use vector operations to correct the heading with an additional sensor such a magnetometer.

# Dependecies
This library depends on the [vector_datatype](https://github.com/RCmags/vector_datatype) library.

# References
See these links for more information on the Mahony filter:
- [Nonlinear Complementary Filters on the Special
Orthogonal Group](https://hal.archives-ouvertes.fr/hal-00488376/document) (original paper)
- [IMU Data Fusing: Complementary, Kalman, and Mahony Filter](http://www.olliw.eu/2013/imu-data-fusing/#chapter23)
- [Mahony Filter](https://nitinjsanket.github.io/tutorials/attitudeest/mahony)
