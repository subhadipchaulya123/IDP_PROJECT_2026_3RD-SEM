# How GTSAM Works in IMU–GPS Fusion

The most important part of this project is understanding the **factor graph** used by GTSAM.

## A. What Is a Factor Graph?

A factor graph represents unknown states and the measurements that constrain those states.

In this project, a state can contain:

- **Position:** p
- **Orientation:** R
- **Velocity:** v
- **IMU bias:** b

At time k, the state can be represented as:

X_k = {p_k, R_k, v_k, b_k}

Where:

- p_k is the estimated 3D position.
- R_k is the device's orientation.
- v_k is its velocity.
- b_k represents biases or offsets in the IMU measurements.

The factor graph connects these unknown states through measurement constraints.

## B. What Are the Factors in This Project?

### 1. IMU Factor

The IMU provides acceleration and angular velocity. GTSAM uses **IMU preintegration** to summarize motion measurements between consecutive states.

This constrains how position, velocity, and orientation change over time.

For example, if the device accelerates forward, the IMU measurements help estimate the resulting change in velocity and position.

IMU-only estimates tend to drift because small measurement errors accumulate over time.

### 2. GPS Factor

The GPS factor constrains the estimated position using GPS position measurements.

For example, a GPS position might be represented in a local coordinate frame as:

East  = 10.5 m
North = 4.2 m
Up    = 0.3 m

GTSAM uses the GPS observation to help correct the estimated position.

GPS is generally less frequent than IMU data, but it provides an important external position reference.

**Note:** GPS latitude and longitude usually need to be converted into a suitable local coordinate frame before being used as metric position measurements.

### 3. IMU Bias Constraints

Accelerometers and gyroscopes can have biases. GTSAM can estimate these biases and model how they evolve over time.

This helps the optimizer avoid treating every sensor reading as perfectly accurate.

## C. How Does GTSAM Combine the Factors?

Conceptually, GTSAM seeks a state estimate that best satisfies the available measurement constraints.

A simplified objective is:

X* = arg min [E_IMU(X) + E_GPS(X) + E_bias(X) + E_prior(X)]

Where:

- E_IMU measures disagreement with the IMU motion constraints.
- E_GPS measures disagreement with GPS position measurements.
- E_bias represents IMU bias constraints.
- E_prior represents initial-state constraints, when included.

The errors are weighted using noise models. Measurements considered more reliable can exert greater influence on the estimate.

**Key idea:** GTSAM does not simply average GPS and IMU positions. It optimizes a set of connected states to find a solution consistent with the measurements and their uncertainties.

## D. What Is ISAM2?

This project uses **iSAM2 (incremental Smoothing and Mapping)**, an incremental optimizer.

As new measurements arrive, the system can add new variables and factors, then update its estimate incrementally.

Conceptually, the process is:

Initial state
     |
     v
Add IMU constraints
     |
     v
Add GPS position constraint when available
     |
     v
Update iSAM2
     |
     v
Estimate motion states
     |
     v
Process next measurements
     |
     v
Update estimate again

iSAM2 is useful when measurements arrive continuously because it can update the solution without rebuilding and solving the entire optimization problem from scratch each time.

## E. Important Implementation Note

The output of an IMU prediction method is not necessarily the same as exporting every optimized state directly from iSAM2.

Therefore, the precise contents of `Fused_Trajectory.csv` depend on how the project's CSV driver obtains and saves the trajectory.

To describe the exact output behavior, inspect the code that reads the iSAM2 estimate or performs IMU prediction and writes `Fused_Trajectory.csv`.
