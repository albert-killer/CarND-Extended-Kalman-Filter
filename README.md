# CarND - Extended-Kalman-Filter

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This is the project repository for **Project No. 1 Extended Kalman Filter**, part of Term 2 _Sensor Fusion_ of Udacity Self-Driving Car Nanodegree program, submitted by Albert Killer in April 2017. 

An Extended Kalman Filter (EKF) was implemented in C++ to process simulated LIDAR and RADAR measurements provided by Udacity. In doing so the position and velocity of an object, moving around a vehicle equiped with those sensors, can be detected with high accuracy:

```
Accuracy - RMSE:
px: 0.0972256
py: 0.0853761
vx: 0.4508550
vy: 0.4395880
```
We use a Kalman Filter for LIDAR data. In order to handle the nonlinear measurement function _h_(_x_) of RADAR data we have to apply an Extended Kalmand Filter (EKF). The EKF uses a linear approximation of _h_(_x_) and applies First Order Taylor Expansion in order to get a Gaussian distribution again.    

You want to know more about Extended Kalman Filters? Have a look: https://en.wikipedia.org/wiki/Extended_Kalman_filter
