INPUT
  |
  +-- AccelerometerUncalibrated.csv
  |
  +-- GyroscopeUncalibrated.csv
  |
  +-- Location.csv
          |
          v
PREPROCESSING
  |
  +-- synchronize.py
  |       |
  |       v
  |   Synchronized_IMU.csv
  |
  +-- gps_to_enu.py
          |
          v
      GPS_ENU.csv
          |
          v
GTSAM FUSION
  |
  +-- Read IMU measurements
  |
  +-- Read GPS measurements
  |
  +-- Create/update motion states
  |
  +-- Add IMU and GPS factors
  |
  +-- Model IMU bias and measurement noise
  |
  +-- Optimize using ISAM2
          |
          v
OUTPUT
  |
  +-- Fused_Trajectory.csv
  |
  +-- Fused_Trajectory.png
