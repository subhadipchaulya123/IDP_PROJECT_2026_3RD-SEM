## Project Overview

GPS and IMU are commonly used for navigation and localization.

GPS provides absolute position measurements, while an Inertial Measurement Unit (IMU) provides high-frequency motion information.

However, both sensors have limitations:

- GPS measurements can be noisy and have a relatively low update frequency.
- IMU measurements can accumulate errors over time.
- Accelerometer and gyroscope biases can cause position and orientation drift.

This project combines GPS and IMU data using GTSAM to estimate a trajectory using information from both sensors.

The project uses Python for data processing and GTSAM for the fusion algorithm.

---


## PROBLEM STATEMENT:

GPS measurements are noisy and relatively low-frequency, while IMU measurements provide high-frequency motion information but suffer from accumulated drift. This project addresses the problem by fusing GPS and IMU data using GTSAM to estimate a consistent trajectory


## Objectives :

The main objectives of this project are:

1. Process accelerometer, gyroscope, and GPS data collected from a smartphone.
2. Synchronize accelerometer and gyroscope measurements using timestamps.
3. Convert geographic GPS coordinates into a local metric coordinate system.
4. Combine IMU motion information and GPS position constraints using GTSAM.
5. Account for accelerometer and gyroscope biases in the estimation model.
6. Generate a trajectory from the fusion pipeline.
7. Visualize the GPS and fused trajectories for comparison.



## Solving Approach:
The system synchronizes accelerometer and gyroscope data, converts GPS coordinates into a local metric frame, applies IMU preintegration and GPS position factors in a GTSAM factor graph, estimates IMU biases, performs incremental ISAM2 optimization, and generates the fused trajectory for visualization.



## Proposed Solution

The proposed solution uses **GTSAM-based sensor fusion**.

The pipeline consists of the following stages:

1. Collect raw sensor data.
2. Synchronize accelerometer and gyroscope measurements.
3. Convert GPS coordinates into local metric coordinates.
4. Process IMU measurements using the GTSAM fusion implementation.
5. Incorporate GPS position measurements.
6. Estimate IMU biases and update the state estimate.
7. Generate the trajectory output.
8. Visualize the result using Matplotlib.
