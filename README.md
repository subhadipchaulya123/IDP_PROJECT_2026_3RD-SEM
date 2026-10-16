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


## System Architecture
<img width="1080" height="1796" alt="Screenshot_20261016_223335" src="https://github.com/user-attachments/assets/e5ebb393-19a2-4a4e-ad82-c5188c7ee553" />



## Technologies Used

| Technology | Purpose |
|---|---|
|`Python` | Main programming language |  
|`GTSAM` | Factor-graph-based sensor fusion |
|`NumPy `| Numerical computation |
|`Pandas `| CSV processing and timestamp synchronization |
|`UTM `| Projection of geographic GPS coordinates |
|`ENU `| Local East-North-Up coordinate representation |
|`Matplotlib` | Trajectory visualization |
|`ISAM2` | Incremental optimization supported by GTSAM |



## Project Structure

```text
imu_gps_gtsam/
├── .venv/
├── data/
│   ├── AccelerometerUncalibrated.csv
│   ├── GyroscopeUncalibrated.csv
│   └── Location.csv
├── output/
│   ├── Synchronized_IMU.csv
│   ├── GPS_ENU.csv
│   ├── Fused_Trajectory.csv
│   └── Fused_Trajectory.png
├── scripts/
│   ├── synchronize.py
│   ├── gps_to_enu.py
│   ├── gtsam_fusion_core.py
│   ├── gtsam_fusion_csv.py
│   └── plot_trajectory.py
├── README.md
└── requirements.txt
```

*Note: The structure above represents the expected project layout. Generated files appear in `output/` after running the corresponding scripts.*


## Working Process
## 1. Sensor Data Collection

### Sensor Logger App

Sensor Logger is a free, easy-to-use, cross-platform application for recording data from various sensors available on smartphones. It supports exporting recorded data in formats such as CSV, JSON, SQLite, and KML, making it useful for sensor-based research and development.

For this project, Sensor Logger was used to collect Inertial Measurement Unit (IMU) and GPS data for an IMU–GPS sensor fusion system using GTSAM.

### Sensors Used

The following sensors were used for data collection:

- **Accelerometer:** Records linear acceleration along the X, Y, and Z axes.
- **Gyroscope:** Records angular velocity along the X, Y, and Z axes.
- **GPS:** Provides location information, including latitude and longitude, for position estimation.

### Data Collection Configuration

The IMU recording was configured with a target sampling frequency of **250 Hz**. GPS data was recorded separately at a lower sampling frequency.

The collected sensor data was exported into CSV files for further processing, timestamp synchronization, and integration into the sensor fusion pipeline.

The accelerometer and gyroscope measurements were processed to prepare synchronized IMU data, while GPS measurements were organized separately for use in the localization system.

### Purpose of Data Collection

The collected sensor data serves as the input for the IMU–GPS fusion pipeline. The IMU measurements provide information about device motion, while GPS measurements provide position references.

These data sources are used together by the GTSAM-based fusion system to estimate the device's position, orientation, and velocity over time.

### Supported Sensors and Measurements

Sensor Logger supports recording data from a wide range of smartphone sensors and device features, including:

- Accelerometer and gravity vector
- Gyroscope and device orientation
- Magnetometer and compass
- GPS location, altitude, speed, and heading
- Barometer and atmospheric pressure
- Microphone and audio measurements
- Camera images, video, and depth (where supported)
- Pedometer and activity data
- Device battery level and state
- Additional sensors and device information, depending on the platform

### Application Link

Learn more and download Sensor Logger:

[https://www.tszheichoi.com/sensorlogger](https://www.tszheichoi.com/sensorlogger)


The project uses three input CSV files(uncalibrated gyroscope,accelerometer csv file and location file) .
## Why Use Uncalibrated IMU Data?

In this project, we use **uncalibrated accelerometer and gyroscope data** collected using the Sensor Logger smartphone application. Uncalibrated data provides raw sensor measurements along with information about the sensors' estimated biases.

### Reasons for Using Uncalibrated Data

1. **Realistic Sensor Measurements:** Uncalibrated data represents sensor readings before the operating system's calibration corrections are applied. This helps us study the behavior of IMU sensors under realistic conditions.

2. **IMU Bias Estimation:** Accelerometers and gyroscopes may contain measurement biases. Using uncalibrated measurements allows us to investigate how GTSAM estimates and accounts for these biases during sensor fusion.

3. **Understanding Sensor Errors:** Raw measurements may contain biases, noise, and other imperfections. Working with these measurements helps us understand how sensor errors affect motion estimation.

4. **Improved Understanding of Sensor Fusion:** Using uncalibrated data helps us study how IMU measurements and GPS position observations complement each other during trajectory estimation.

5. **Testing GTSAM's IMU Preintegration:** GTSAM supports IMU preintegration and bias estimation. Uncalibrated sensor measurements provide a useful input for evaluating these capabilities when appropriate sensor noise parameters and bias models are configured.

### How Uncalibrated Data Is Used in Our Project

The project follows these steps:

1. Collect uncalibrated accelerometer and gyroscope data using Sensor Logger.
2. Export the recorded sensor measurements into CSV files.
3. Synchronize the accelerometer and gyroscope measurements using their timestamps.
4. Prepare the synchronized IMU data and GPS measurements for the sensor fusion pipeline.
5. Use GTSAM to combine IMU motion constraints and GPS position measurements.
6. Estimate the device's trajectory using the configured optimization and sensor models.

### Important Consideration

Uncalibrated data does not automatically improve localization accuracy. It may contain larger biases and errors than calibrated data, so appropriate IMU noise parameters, bias initialization, and bias estimation are important.

Also, uncalibrated sensor outputs can include estimated bias values in addition to the measured values, depending on the device and recording application. The exact CSV fields must be checked to determine which values are available.

**Project Objective:** To investigate how uncalibrated smartphone IMU measurements can be combined with GPS data using GTSAM for trajectory estimation and sensor fusion.

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

##**Understanding Units**

See [UNITS](https://github.com/subhadipchaulya123/imu_gps_gtsam_____IDP_PROJECT_2026_3RD_SEM/blob/main/UNITS.md)


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

### 6. Visualization

The `plot_trajectory.py` script uses Matplotlib to display the GPS trajectory and the trajectory produced by the fusion pipeline.

It saves:

```text
output/Fused_Trajectory.png
```

Matplotlib is used for visualization; it does not perform sensor fusion.

## Cordinate System

See [Coordinate System](https://github.com/subhadipchaulya123/imu_gps_gtsam_____IDP_PROJECT_2026_3RD_SEM/blob/main/Coordinate.md)  for the full documentation of coordinate systems.

## Installation

### Prerequisites

- Ubuntu Linux 24.04 LTS
- Python 3
- Git
- A working Python virtual environment

### 1. Clone the repository

```bash
https://github.com/subhadipchaulya123/imu_gps_gtsam
cd imu_gps_gtsam
```

### 2. Create a virtual environment

```bash
python3 -m venv .venv
```

Activate it:

```bash
source .venv/bin/activate
```


### 3. Install dependencies

```bash
python -m pip install --upgrade pip
python -m pip install numpy pandas matplotlib utm gtsam
```

Verify GTSAM:

```bash
python -c "import gtsam; print('GTSAM installed successfully')"
```

Verify the plotting library:

```bash
python -c "import matplotlib; print('Matplotlib installed successfully')"
```

---


## Input Data

This project uses recorded IMU and GPS measurements stored in CSV files.

The sensor data is stored in the following directory:

```text
imu_gps_gtsam/
│
├── data/
│   ├── AccelerometerUncalibrated.csv
│   ├── GyroscopeUncalibrated.csv
│   └── Location.csv
```

### 1. Accelerometer Data

**File:** `AccelerometerUncalibrated.csv`

Contains accelerometer measurements along three axes:

- X-axis acceleration
- Y-axis acceleration
- Z-axis acceleration

### 2. Gyroscope Data

**File:** `GyroscopeUncalibrated.csv`

Contains angular velocity measurements along three axes:

- X-axis angular velocity
- Y-axis angular velocity
- Z-axis angular velocity

### 3. GPS Data

**File:** `Location.csv`

Contains timestamped GPS measurements, including latitude and longitude.

### 4. Synchronized IMU Data

**File:** `Synchronized_IMU.csv`

This file contains synchronized Inertial Measurement Unit (IMU) data, including accelerometer and gyroscope measurements. The data is organized and aligned using timestamps to maintain the correct temporal relationship between the sensor readings.

The accelerometer provides linear acceleration measurements along the X, Y, and Z axes, while the gyroscope provides angular velocity measurements along the same three axes. These measurements help describe the device's motion and orientation over time.

Before using the data for sensor fusion, the accelerometer and gyroscope measurements are synchronized to ensure that the readings correspond to the appropriate timestamps. This synchronization is important for maintaining temporal consistency during motion estimation.

The synchronized IMU data serves as an input to the IMU–GPS sensor fusion pipeline. GTSAM uses IMU measurements with other available sensor information to estimate the device's position, orientation, and velocity over time.

**Key Features:**
- Contains accelerometer and gyroscope measurements.
- Organizes sensor readings according to their timestamps.
- Maintains temporal alignment between different IMU measurements.
- Supports motion estimation and orientation tracking.
- Provides input data for IMU–GPS sensor fusion using GTSAM.
- Helps reduce synchronization-related inconsistencies during sensor data processing.

**Purpose:** The primary purpose of this file is to provide temporally synchronized IMU measurements for subsequent processing, motion estimation, and integration with GPS data in the sensor fusion system.

The measured IMU sampling rate is approximately 250 Hz.

---



## How to Run

Run the scripts in the following order.

### Step 1: Open the project

```bash
cd ~/imu_gps_gtsam
```

### Step 2: Activate the Python Virtual Environment

Activate the existing virtual environment:

```bash
source .venv/bin/activate
```

Ensure that `(venv)` appears in your terminal prompt.

### Step 3: Verify GTSAM Installation

Run:

```bash
python -c "import gtsam; print('GTSAM imported successfully')"
```

Expected output:

```text
GTSAM imported successfully
```

If the import fails, verify that GTSAM is installed in the active virtual environment.

### Step 4: Synchronize IMU Measurements

```bash
python scripts/synchronize.py
```

Expected output:

```text
output/Synchronized_IMU.csv
```

### Step 5: Convert GPS coordinates

```bash
python scripts/gps_to_enu.py
```

Expected output:

```text
output/GPS_ENU.csv
```

### Step 6: Run GTSAM fusion

```bash
python scripts/gtsam_fusion_csv.py
```

Expected output:

```text
output/Fused_Trajectory.csv
```

The exact number of trajectory points depends on the input dataset and processing configuration.

### Step 7: Generate the trajectory plot

```bash
python scripts/plot_trajectory.py
```

Expected output:

```text
output/Fused_Trajectory.png
```

### Step 8: Open the plot

```bash
xdg-open output/Fused_Trajectory.png
```

---

## Run the Complete Pipeline

After installation and placing the input files in `data/`, run:

```bash
cd ~/imu_gps_gtsam

source .venv/bin/activate

python scripts/synchronize.py
python scripts/gps_to_enu.py
python scripts/gtsam_fusion_csv.py
python scripts/plot_trajectory.py

xdg-open output/Fused_Trajectory.png
```

---



## Output Files

| File | Description |
|---|---|
| `Synchronized_IMU.csv` | Synchronized accelerometer and gyroscope measurements |
| `GPS_ENU.csv` | GPS positions represented in local metric coordinates |
| `Fused_Trajectory.csv` | Trajectory generated by the current fusion pipeline |
| `Fused_Trajectory.png` | Visualization comparing GPS and fusion output |


### View My Dataset Output:
   
 To see the actual output and results of my IMU–GPS sensor fusion project, visit the [README.md – Output and Results](https://github.com/subhadipchaulya123/IDP_PROJECT_2026_3RD-SEM/blob/main/output.md) section in my GitHub repository.




# Why GTSAM?

GTSAM provides tools for nonlinear estimation using factor graphs.

For GPS-IMU fusion, it supports modeling relationships between states and measurements.

The project uses a GTSAM-based approach rather than replacing the fusion algorithm with a conventional Kalman-filter implementation.

### Main concepts

**IMU Preintegration**

Summarizes high-frequency IMU motion between states for use in the estimation problem.

**GPS Position Factors**

Introduce GPS position measurements as constraints.

**IMU Bias Estimation**

Models accelerometer and gyroscope biases that can contribute to accumulated error.

**ISAM2**

Supports incremental smoothing and optimization as new factors are processed.

---


## How GTSAM Works in This Project

GTSAM (Georgia Tech Smoothing and Mapping) is a library for state estimation using factor graphs and optimization.

The objective of this project is to combine IMU measurements with GPS position observations to estimate a movement trajectory.

### 1. IMU Measurements

The accelerometer measures acceleration along three axes, while the gyroscope measures angular velocity.

These measurements provide high-frequency information about the motion of the device.

### 2. GPS Measurements

GPS provides geographic position measurements, including latitude and longitude.

These measurements provide an external position reference for the trajectory estimation process.

### 3. Factor-Graph-Based Estimation

GTSAM represents the estimation problem using variables and factors that describe relationships between states and sensor measurements.

The resulting estimate depends on the factors, noise models, and optimization methods implemented in the fusion code.

### 4. IMU Preintegration and Bias Estimation

If implemented in the fusion core, IMU preintegration summarizes high-frequency inertial measurements between states, while bias estimation accounts for systematic accelerometer and gyroscope errors.

### 5. Incremental Optimization

If configured in the implementation, ISAM2 can update the state estimate incrementally as new factors are processed.

> **Implementation note:** IMU preintegration, GPS factors, bias estimation, and ISAM2 should only be described as implemented features when they are present in the actual source code.




For more gtsam information see [GTSAM_WORKING_PROCESS](https://github.com/subhadipchaulya123/IDP_PROJECT_2026_3RD-SEM/blob/main/gtsam_working_process.md)
---

## Limitations

- GPS quality depends on the environment and receiver.
- Uncalibrated IMU measurements may contain bias and noise.
- Incorrect sensor units can produce physically invalid estimates.
- Timestamp errors can affect synchronization and fusion.
- Coordinate-frame and gravity conventions must be consistent.
- A successful program run does not by itself establish that the estimated trajectory is accurate.
- The trajectory output should be validated against GPS measurements and, ideally, an independent ground-truth trajectory.

---

## Future Improvements

Potential improvements include:

- Calibrate accelerometer and gyroscope measurements.
- Validate sensor units and coordinate frames.
- Evaluate trajectory error using ground-truth data.
- Tune noise models and bias parameters.
- Add quantitative trajectory metrics.
- Improve visualization with time-series and error plots.
- Integrate live sensor data for real-time localization.
- Evaluate the method on longer datasets and different environments.

---

## References

- [GTSAM](https://github.com/borglab/gtsam)
- [PaulKemppi/gtsam_fusion](https://github.com/PaulKemppi/gtsam_fusion)
- [Awesome Sensor Logger](https://github.com/tszheichoi/awesome-sensor-logger)

---

## Project Summary

**Problem:** GPS measurements can be noisy and relatively low-frequency, while IMU measurements can accumulate drift.

**Approach:** Synchronize IMU data, convert GPS into a local metric coordinate system, combine GPS and IMU measurements using GTSAM, and generate a trajectory for visualization.

**Result:** A CSV-based GPS-IMU processing pipeline that produces a trajectory output and a visualization for further evaluation.

   ### View My Dataset Output:
   To see the actual output and results of my IMU–GPS sensor fusion project, visit the [README.md – Output and Results](https://github.com/subhadipchaulya123/IDP_PROJECT_2026_3RD-SEM/blob/main/output.md) section in my GitHub repository.


---

## Author

**Subhadip Chaulya**

- **University:** Adamas University
- **Department:** Computer Science and Engineering
- **Specialization:** Artificial Intelligence and Machine Learning
- **Project:** IMU–GPS Sensor Fusion Using GTSAM

### Project Repository

[View My Project on GitHub](https://github.com/subhadipchaulya123/IDP_PROJECT_2026_3RD-SEM)

### Dataset Output

To explore the dataset processing results, IMU synchronization, GPS coordinate conversion, and fused trajectory, visit the [Output.md section](https://github.com/subhadipchaulya123/IDP_PROJECT_2026_3RD-SEM/blob/main/output.md).

---

*Developed as part of an academic project at Adamas University.*
