# imuFilter
This library fuses the outputs of an inertial measurement unit (IMU) and stores the heading as a quaternion. It uses a _kalman-like_ filter to check the acceleration and see if it lies within a deviation from (0,0,1)g. If the acceleration is within this band, it will strongly correct the orientation. However, if the acceleration lies outside of this band, it will barely affect the orientation. To this end, the deviation from vertical is used to update the variance and kalman gain: 

$\ \overrightarrow{a_{rel}} = \overrightarrow{a_{local}} - (0,0,1) $

$\ K_{\sigma} = {\alpha}/(1 + \frac{ {\sigma}^2 }{ {\sigma}_{acc}^2 } ) $

$\ {\sigma}^2 = | \overrightarrow{a_{rel}} |^2 + K_{\sigma}{\sigma}^2 $ 

The kalman gain then scaled by a delay parameter and used to correct the attitude. This scaling allows the filter to act like a 1rst-order low pass filter that smoothens the acceleration at the cost of slower response: 

$\ E_{k} = \theta_{accel} - \theta_{k-1} $

$\ \theta_{k} = \theta_{k-1} + \dot{\theta}{\Delta t} + K_{\sigma}E_{k} $

As the filter uses a quaternion to encode rotations, it's easy to perform coordinate transformations. The library has functions to:
- Transfor a vector to the local or global frame.
- Get the unit vectors of the X, Y and Z axes in the local or global frame.

Moreover, since a 6-axis IMU (gyro-accelerometer) cannot measure an absolute heading, a function is included to rotate the orientation about the vertical (yaw) axis. One can use vector operations to correct the heading with an additional sensor like a magnetometer.

# Velocity estimate
The library can integrate acceleration to obtain a short-term estimate of velocity. This is accomplished by using a set of kalman-like filters like the one shown above. First, the acceleration is checked against a rest condition of (0,0,0)g, and any deviations from this state that lie within specified uncertainty band are suppresed. This eliminates much of the bias due to gravity or miscalibration:

$\ K_{acc} = 1/(1 + \frac{ S_{acc}^2 }{ {\sigma}_{acc}^2 } ) $

$\ S_{acc}^2 = | \overrightarrow{a_{rel}} |^2 + K_{acc}S_{acc}^2 $ 

$\ a_{k} = a_{k} - \overline{a_{k}} $

$\ \overline{a_{k}} = \overline{a_{k-1}} + K_{acc}{a_{k}} $

Afterwords, the deviation of the velocity (relative to a known target velocity) along with the variance of the acceleration are used to determine the kalman gain of the velocity. This relationship causes small velocity deviations or small accelerations to reduce the gain, and the velocity estimate is forced to match the known target velocity:  

$\ \Delta{V} = V_{k-1} - V_{target} $

$\ S_{vel}^2 = | \overrightarrow{ {\Delta}V } |^2 + K_{vel}S_{vel}^2 $ 

$\ K_{vel} = 1/(1 + ( \frac{ S_{vel} S_{acc} }{ \sigma_{vel} \sigma_{acc} } )^2 ) $

The velocity is then updated using the trapezoidal rule to integrate the corrected acceleration:

$\ V_{k} = V_{k-1} + K_{vel}{\Delta}{V} + \frac{dt}{2}( a_{k} + a_{k-1} ) $

# Dependecies
This library depends on the [vector_datatype](https://github.com/RCmags/vector_datatype) library.

# References
See these links for more information on the Mahony filter:
- [Nonlinear Complementary Filters on the Special
Orthogonal Group](https://hal.archives-ouvertes.fr/hal-00488376/document) (original paper)
- [IMU Data Fusing: Complementary, Kalman, and Mahony Filter](http://www.olliw.eu/2013/imu-data-fusing/#chapter23)
- [Mahony Filter](https://nitinjsanket.github.io/tutorials/attitudeest/mahony)
