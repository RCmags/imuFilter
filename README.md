# imuFilter
This library fuses the outputs of an inertial measurement unit (IMU) and stores the heading as a quaternion. It uses a _kalman-like_ filter to check the acceleration and see if it lies within a given deviation from 1g vertical. If the acceleration is within this band, it will strongly correct the orientation. However, if the acceleration lies outside of this band, it will barely affect the orientation. To this end, the deviation from vertical is used to update the variance and set the kalman gain: 

$\ \overrightarrow{a_{rel}} = \overrightarrow{a_{local}} - (0,0,1) $

$\ K_{\sigma} = {\alpha}/(1 + \frac{ {\sigma}^2 }{ {\sigma}_{acc}^2 } ) $

$\ {\sigma}^2 = | \overrightarrow{a_{rel}} |^2 + K_{\sigma}{\sigma}^2 $ 

The kalman gain then scaled by a delay parameter and used to determine the attitude correction. This allows the filter to act like a 1rst-order low pass filter that smoothens the acceleration at the cost of slower response: 

$\ E_{k} = \theta_{accel} - \theta_{k-1} $

$\ \theta_{k} = \theta_{k-1} + \dot{\theta}{\Delta t} + K_{\sigma}E_{k} $

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
