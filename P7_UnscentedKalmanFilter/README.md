# Unscented Kalman Filter

## Usage and Dependencies

* Please visit this [repository](https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project) by Udacity.

* Replace the files in the folder `src`.

## Model Description

### CTRV Model (constant turn rate and velocity magnitude)

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/UKF/CTRVmodel.png" alt="CTRV model" width="388">

### State Vector (5 dimension)

* `p_x` is the value of the x-coordinate of the vehicle

* `p_y` is the value of the y-coordinate of the vehicle

* `v` is the velocity of the vehicle in direction in the front of the vehicle

* `psi` is the yaw angle of the vehicle

* `psi_p` is the change rate of the yaw angle of the vehicle

### Measurements from Radar and Lidar
All the raw data received from the sensors use the world coordinates.

* The measurement data from Radar uses the polar coordinate system, which means, the first measurement is the distance to the original point, and the second measurement is the angle.

* The data of measurement from Lidar uses the 2d cartesian coordinate system.

### Kalman Filter

```
<initialization> ----> [prediction] ----> [update]
                            ^               |
                            |               |
                            |---------------|
```

Following diagram is for the linear Kalman filter. (Unscented Kalman Filter is non-linear.)

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/UKF/lin_KF.png" alt="linear kalman filter" width="666">


### Prediction

#### 1. Generate Sigma Points

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/UKF/gen_sigma_ps.png" alt="generate sigma points" width="488">

* The first term generates only one sigma point, which is the mean point (center). The second term and the third term can create two sets of sigma points.

* `lambda` is normally be chosen as `3 - n_x`.

* `n_x` is the dimension of the states.

* `sqrt(P)` can be calculated as the **square root matrix** of the matrix `P`.

#### 2. Predict Sigma Points

In the prediction of the sigma points the non-linearity occurs. The functions for prediction is shown below:

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/UKF/prediction_sigma_ps.png" alt="prediction" width="666">

* Vector `x_k` is the state at this time point.

* The second vector is from solution of the differential equation `integral{x_p}` from time point `t_k` to `t_{k+1}`, whereby `x_p = derivative{x}` and `x` is the state vector.

* The third vector is the noise values. 

  1. `ny_a` is the longitudinal acceleration noise and `ny_psypp` is the yaw acceleration noise, the two variables are normally distributed as `ny_a = N(0, sigma_a ^2)` and `ny_psypp = N(0, sigma_psypp ^2)`.
  
  2. The noise of the yaw rate `noise_{psy_p} = dt * ny_psypp`.

  3. The noise of the velocity `noise_{v} = dt * ny_a` 

  4. The noise of the yaw `noise_{psy} = integral(noise_{psy_p})` in the time interval `t_k` to `t_{k+1}`.

  5. When calculating the new positions `p_x` and `p_y` the influence of the yaw acceleration noise `ny_psypp` is ignored. The vehicle is assumed to be driven straightforward as long as the yaw rate is not too high. The noise of the position would be `noise_{p_x} = integral(noise_{v_x})` and `noise_{p_y} = integral(noise_{v_y}]` in the time interval `t_k` to `t_{k+1}`.

#### UKF Augmentation

The two noises, which are longitudinal and yaw acceleration noises, are incorporated in the states vector and covariance matrix.

The two noises are independent of each other. The covariance matrix `Q` of the noises can be calculated as below:

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/UKF/cov_max_noise.png" alt="covariance matrix of the noises" width="233">

The new states and the new covariance matrix is shown below:

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/UKF/aug_UKF.png" alt="covariance matrix of the noises" width="333">

#### 3. Predict Mean and Covariance

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/UKF/predictedMeanCovariance.png" alt="Predict Mean and Covariance" width="448">

* `n_a` is the dimension of the augmented states.

* `lambda` is normally be set as `3 - n_a`.

### Update

#### 1. Predict Measurement

The predicted measurements and the calculated corresponding matrix is used to update the Kalman filter, to let the filter be more sure about the localization of the vehicle.

##### Predict Radar Measurement

The predicted state should be changed into the measurement vector:

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/UKF/radar_1.png" alt="state->measurement" width="188">

The equations for the changing is shown below:

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/UKF/radar_2.png" alt="measurement mean/covariance" width="488">

##### Predict Lidar Measurement

* The predicted measurement vector `Z_{lidar}` has the components `p_x_{k+1|k}` and `p_y_{k+1|k}` of the vehicle, which means that the predicted measurement vector of lidar is much easier to calculate as that of the radar.

* Other matrix and variables are calculated using the same equations as that of radar.


#### 2. Update State

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/UKF/UKFupdate.png" alt="UKF update" width="388">

Whereby `z_{k+1}` is the data of true measurements at the time point `t_{k+1}`.

### Parameters and Consistency

We used **Normalized Innovation Squared** (NIS) to estimate the prediction of the measurements.

Following is the equation of the NIS:

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/UKF/NIS.png" alt="NIS" width="388">

We used **Chi-squared Distribution** to judge how well the prediction is. The table of the **χ2 values vs p-values** is shown below:

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/UKF/chisquredDistr.png" alt="ChisquaredDistr" width="666">

#### Explanation of the table with a simple example

1. We have three values in the measurement vector, which means that the degree of dimensions is `3`.

2. Choose a P value `0.10`.

3. The corresponding χ2 value is `6.25`.

4. This means that in this case, `10%` of all the NIS values should be bigger than `6.25`. 

5. For example, in this case, when `50%` of all the NIS values are bigger than `6.25`. This means that we underestimate the uncertainty (noise) of the system.

6. For example, in this case, when `0.1%` of all the NIS values are bigger than `6.25`. This means that we overestimate the uncertainty (noise) of the system.

## Result

The video is uploaded on the Youtube. [[link]](https://www.youtube.com/watch?v=vCsr6XV9qgQ&index=7&list=PLNDTbGbATLcED0iX8K-zY3vrNbwhxV8gC)

* In this simulator, the green points represent the predicted positions of the vehicle.

* The blue and red points are the measured positions with noise added from radar and lidar.

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/UKF/res1.png" alt="res" width="333">

## References

1. Udacity Self-Driving Car Engineer Nanodegree

2. [Chi-squared distribution (Wikipedia)](https://en.wikipedia.org/wiki/Chi-squared_distribution)

3. The Unscented Kalman Filter for Nonlinear Estimation, Eric A. Wan, and Rudolph van der Merwe, Oregon Graduate Institute of Science & Technology

4. [Kalman Filter (Wikipedia)](https://en.wikipedia.org/wiki/Kalman_filter)