               RAW SENSOR DATA
                         |
          +--------------+--------------+
          |              |              |
          v              v              v
    Accelerometer     Gyroscope         GPS
          |              |              |
          +------+-------+              |
                 |                      |
                 v                      |
         IMU Synchronization            |
         (synchronize.py)               |
                 |                      |
                 v                      v
        Synchronized_IMU.csv       GPS Conversion
                                   (gps_to_enu.py)
                                        |
                                        v
                                    GPS_ENU.csv
                 |                      |
                 +----------+-----------+
                            |
                            v
                    +---------------+
                    |     GTSAM     |
                    |               |
                    | IMU Processing|
                    | GPS Factors   |
                    | Bias Estimation
                    | ISAM2 Updates |
                    +-------+-------+
                            |
                            v
                  Fused_Trajectory.csv
                            |
                            v
                       Matplotlib
                            |
                            v
                  Fused_Trajectory.png
