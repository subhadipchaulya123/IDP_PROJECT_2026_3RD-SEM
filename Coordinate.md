## Coordinate Systems and Sensor Conventions (Android)

This project uses sensor measurements collected using Sensor Logger on an Android device. Understanding the coordinate system is essential for correctly processing accelerometer, gyroscope, and GPS measurements during sensor fusion.

Reference: [Sensor Logger Coordinate Systems](https://github.com/tszheichoi/awesome-sensor-logger/blob/main/COORDINATES.md)

### 1. Accelerometer Coordinate System

On Android, the accelerometer uses a device-local coordinate system:

- **X-axis:** Horizontal axis across the phone's screen.
- **Y-axis:** Vertical axis along the phone's screen.
- **Z-axis:** Perpendicular to the screen, pointing outward from the front of the phone under the standard Android device coordinate convention.

The coordinate axes are fixed relative to the device. They rotate with the phone when it changes orientation.

The accelerometer measures acceleration along these three axes. The reported values depend on the sensor convention, phone orientation, and whether gravity is included in the measurements.

For example, when a phone lies flat with its screen facing upward, the Android gravity vector typically points along the positive Z-axis.

**Important:** Confirm whether your CSV contains raw, uncalibrated, or gravity-compensated acceleration before using the measurements in GTSAM.

<img width="1080" height="992" alt="IMG_20261016_220726" src="https://github.com/user-attachments/assets/50305495-726a-40a1-a3bd-bc2e6dd88982" />

### 2. Gyroscope Coordinate System

The gyroscope measures angular velocity around the same device-local X, Y, and Z axes.

- **X-axis:** Rotation around the phone's X-axis.
- **Y-axis:** Rotation around the phone's Y-axis.
- **Z-axis:** Rotation around the phone's Z-axis.

Angular velocity is typically expressed in radians per second (rad/s), but the actual units must be verified from the recorded data.

The gyroscope's axes rotate with the device. Therefore, gyroscope measurements must be interpreted using the phone's orientation and the coordinate-frame conventions of the fusion algorithm.

<img width="1080" height="948" alt="IMG_20261016_220800" src="https://github.com/user-attachments/assets/fd479175-d40a-49b6-84d0-7c5ce59f371d" />


### 3. Android Orientation Convention

Sensor Logger can report orientation values derived from rotation measurements or other orientation-estimation methods.

According to the Sensor Logger documentation:

- **Roll:** Rotation around the Y-axis.
- **Pitch:** Rotation around the X-axis.
- **Yaw:** Rotation around the Z-axis.

Sensor Logger describes its orientation convention using an intrinsic Z-X′-Y″ Tait–Bryan rotation sequence.

Orientation conventions can differ between operating systems and sensor implementations. The orientation values should therefore not be mixed with other coordinate conventions without checking their definitions.

**Note:** This project should use Android-compatible conventions consistently throughout preprocessing and sensor fusion.

### 4. Gravity and Acceleration

Gravity is an important consideration when processing accelerometer measurements.

For an Android device lying flat with its screen facing upward, the gravity vector typically points along the positive Z-axis under the standard Android device coordinate convention.

However, the sign and magnitude of recorded acceleration depend on the sensor type and measurement convention.

Before integrating acceleration to estimate motion, determine whether the data represents:

- Raw acceleration including gravity.
- Linear acceleration with gravity removed.
- Uncalibrated acceleration measurements that may include sensor bias.

Incorrect handling of gravity can introduce substantial errors in estimated position and velocity.

### 5. Coordinate Transformation for GPS–IMU Fusion

GPS provides geographic coordinates, such as latitude and longitude, while the IMU measures motion in the phone's local coordinate frame.

These measurements cannot be directly combined as if they used the same coordinate system.

For sensor fusion, the implementation should establish consistent reference frames.

A typical workflow is:

1. Read the accelerometer and gyroscope measurements.
2. Verify their units and device-local coordinate conventions.
3. Synchronize the IMU measurements using timestamps.
4. Convert GPS latitude and longitude into a local metric coordinate system, such as East-North-Up (ENU), when required.
5. Apply the appropriate rotation and coordinate transformations between the device, navigation, and world frames.
6. Use consistent coordinate conventions in the GTSAM estimation model.

The transformations must account for the device's orientation and the relationship between its local axes and the navigation frame.

### 6. Importance for GTSAM

GTSAM uses mathematical models and optimization to estimate states from measurement constraints.

Incorrect coordinate conventions can produce:

- Incorrect acceleration directions.
- Incorrect rotation estimates.
- Inconsistent gravity compensation.
- Incorrect GPS and IMU alignment.
- Unreliable trajectory estimates.

Therefore, verifying the Android sensor conventions is an important preprocessing step before performing IMU–GPS fusion.

### Reference

- [Sensor Logger Coordinate Systems](https://github.com/tszheichoi/awesome-sensor-logger)
- [Sensor Logger Units Documentation](https://github.com/tszheichoi/awesome-sensor-logger/blob/main/UNITS.md)
- [Android Sensors Overview](https://developer.android.com/develop/sensors-and-location/sensors/sensors_overview)
