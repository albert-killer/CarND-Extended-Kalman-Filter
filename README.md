# CarND - Extended-Kalman-Filter

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This is the project repository for Udacity Self-Driving Car Nanodegree program's Project No. 2 Behavioral Cloning of Term 2 Sensor Fusion by Albert Killer, April 2017. 

An Extended Kalman Filter (EKF) was implemented in C++ to process simulated LIDA and RADAR measurements provided by Udacity. In doing so the position and velocity of an object, which is moving around the vehicle equiped with those sensors, is detected with high accuracy:

```
Accuracy - RMSE:
0.0972256
0.0853761
0.4508550
0.4395880
```
We use a Kalman Filter for LIDAR data. In order to handle the nonlinear measurement function _h_(_x_) of RADAR data we have to apply an Extended Kalmand Filter (EKF). The EKF uses a linear approximation of _h_(_x_) and applies First Order Taylor Expansion in order to get a Gaussian distribution again.    

You want to know more about Extended Kalman Filters? Have a look: https://en.wikipedia.org/wiki/Extended_Kalman_filter
