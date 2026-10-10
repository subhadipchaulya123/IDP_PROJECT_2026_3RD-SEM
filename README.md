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

  **Problem:**

How can GPS position measurements and high-frequency IMU measurements be combined using GTSAM to estimate a more consistent trajectory while accounting for GPS noise and IMU drift?

---


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



## Technologies Used

| Technology | Purpose |

| Python | Main programming language |
| GTSAM | Factor-graph-based sensor fusion |
| NumPy | Numerical computation |
| Pandas | CSV processing and timestamp synchronization |
| UTM | Projection of geographic GPS coordinates |
| ENU | Local East-North-Up coordinate representation |
| Matplotlib | Trajectory visualization |
| ISAM2 | Incremental optimization supported by GTSAM |



## Project Structure

```text
imu_gps_gtsam/
│
├── data/
│   ├── AccelerometerUncalibrated.csv
│   ├── GyroscopeUncalibrated.csv
│   └── Location.csv
│
├── output/
│   ├── Synchronized_IMU.csv
│   ├── GPS_ENU.csv
│   ├── Fused_Trajectory.csv
│   └── Fused_Trajectory.png
│
├── scripts/
│   ├── synchronize.py
│   ├── gps_to_enu.py
│   ├── gtsam_fusion_core.py
│   ├── gtsam_fusion_csv.py
│   └── plot_trajectory.py
│
├── README.md
└── requirements.txt
```

*Note: The structure above represents the expected project layout. Generated files appear in `output/` after running the corresponding scripts.*


## Working Process
### 1. Sensor Data Collection
 The Sensor Logger App

Sensor Logger is a free, easy-to-use, cross-platform data logger that logs readings from common motion-related sensors on smartphones. Once completed, recordings can be exported as a zipped CSV file, JSON, SQLite, KML or be viewed within the app via interactive plots. There is also a free companion watch app (WatchOS and WearOS) that allows you to log heart rate & wrist motion, and control recording sessions remotely from your wrist. Supported sensors and measurements include:

    Device Acceleration (Accelerometer; Raw & Calibrated), G-Force
    Gravity Vector (Accelerometer)
    Device Rotation Rate (Gyroscope)
    Device Orientation (Gyroscope; Raw & Calibrated)
    Magnetic Field (Magnetometer; Raw & Calibrated)
    Compass
    Barometric Altitude (Barometer), Atmospheric Pressure
    GPS: Altitude, Speed, Heading, Latitude, Longitude
    Audio (Microphone)
    Loudness (Microphone), Sound Meter
    Camera Images (Front & Back, Foreground)
    Camera Depth (Front & Back, Foreground, iOS Only)
    Camera Video (Front & Back, Foreground)
    Pedometer
    Activity
    Heart Rate
    Wrist Motion
    Watch Location
    Watch Barometer
    Watch Magnetometer
    Watch Compass
    Watch Microphone
    Light Sensor (Android Only)
    Annotations (Timestamp and Text)
    Device Battery Level and State
    Device Screen Brightness Level
    Nearby Bluetooth Beacons (All Advertised Data)
    Nearby WiFi Networks (Android Only)
    Battery Temperature (Android Only)
    Headphone Motion (Requires AirPods)
    Network

Learn more and download Sensor Logger at www.tszheichoi.com/sensorlogger.


The project uses three input CSV files.

**Accelerometer**

Measures acceleration along three axes:

```text
ax, ay, az
```

**Gyroscope**

Measures angular velocity around three axes:

```text
gx, gy, gz
```

**GPS**

Provides geographic position measurements, including latitude, longitude, altitude, and timestamps.



### 2. IMU Synchronization

The accelerometer and gyroscope may record measurements at slightly different timestamps.

The `synchronize.py` script matches measurements using nearby timestamps and creates a synchronized IMU dataset.

Output:

```text
output/Synchronized_IMU.csv
```

### 3. GPS Coordinate Conversion

GPS provides latitude and longitude in geographic coordinates.

The `gps_to_enu.py` script uses UTM projection to obtain metric Easting and Northing coordinates. It then subtracts the initial position to create a local reference frame.

The output represents:

- **East:** local eastward displacement in meters.
- **North:** local northward displacement in meters.
- **Up:** relative altitude in meters.

Output:

```text
output/GPS_ENU.csv
```


### 4. GTSAM Sensor Fusion

The `gtsam_fusion_core.py` module implements the main fusion logic.

The fusion pipeline uses IMU measurements and GPS position measurements to estimate the trajectory.

The implementation includes components such as:

- IMU preintegration
- GPS position factors
- IMU bias modeling
- Incremental updates using ISAM2

The exact behavior depends on the implementation and its configuration.


### 5. Trajectory Generation

The `gtsam_fusion_csv.py` script loads the processed CSV files and passes measurements to the fusion core.

It saves the resulting trajectory to:

```text
output/Fused_Trajectory.csv
```
