`
             RAW SENSOR DATA
                    |
          +---------+---------+
          |                   |
   Accelerometer CSV      Gyroscope CSV
          |                   |
          +---------+---------+
                    |
                    v
           synchronize.py
                    |
                    v
           Synchronized_IMU.csv
                    |
                    |                  GPS Location.csv
                    |                         |
                    |                         v
                    |                  gps_to_enu.py
                    |                         |
                    |                         v
                    |                     GPS_ENU.csv
                    |                         |
                    +------------+------------+
                                 |
                                 v
                         gtsam_fusion_csv.py
                                 |
                                 v
                         GTSAM Fusion Core
                                 |
                    +------------+------------+
                    |                         |
                    v                         v
              IMU constraints            GPS constraints
                    |                         |
                    +------------+------------+
                                 |
                                 v
                           Factor Graph
                                 |
                                 v
                        ISAM2 Optimization
                                 |
                                 v
                        Fused_Trajectory.csv
                                 |
                                 v
                        plot_trajectory.py
                                 |
                                 v
                       Fused_Trajectory.png
                       `
