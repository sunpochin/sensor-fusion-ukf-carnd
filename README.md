
# Unscented Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

## Introduction:
In this project I implemented Unscented Kalman filter to do sensor fusion, estimate the state of a moving object of interest with noisy lidar and radar measurements.

By correctly implementing sensor fusion, this implementation obtains RMSE values that are lower that the tolerance outlined in the project rubric: px, py, vx, vy output coordinates must have an RMSE <= [.09, .10, .40, .30] .

## Results Demo:
Simulation results are recorded as following youtube videos.

[Video with Dataset 1](https://www.youtube.com/watch?v=nvqw2kU47xM)

## Setting up and Usage
* git clone this repository.
* Install `uWebSocketIO`. By running `install-ubuntu.sh` or `install-mac.sh`
* Build main program:
  Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.
 1. mkdir build
 2. cd build
 3. cmake ..
 4. make
 5. ./UnscentedKF

* Download and run simulator: This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

## Reflection:
* `Parameter tuning` is by intuition, should use `NIS` for a more systemic way.

