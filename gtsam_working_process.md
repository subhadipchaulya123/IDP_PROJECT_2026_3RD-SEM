3. How GTSAM actually works
The most important part of your project is understanding the factor graph.
A. What is a factor graph?
A factor graph represents unknown states and the measurements that constrain those states.
In your project, a state can contain:
- Position: \(p\)
- Orientation: \(R\)
- Velocity: \(v\)
- IMU bias: \(b\)
At time \(k\), we can represent the state as:
\[
X_k = \{p_k,R_k,v_k,b_k\}
\]
Here:
- \(p_k\) represents the estimated 3D position.
- \(R_k\) represents the device's orientation.
- \(v_k\) represents its velocity.
- \(b_k\) represents errors or offsets in the IMU measurements.
The factor graph connects these unknown states through measurement constraints.
B. What are the factors in your project?
1. IMU factor
The IMU provides acceleration and angular velocity. GTSAM uses IMU preintegration to summarize the motion measurements between consecutive states.
This constrains how position, velocity and orientation change over time.
For example, if the device accelerates forward, the IMU measurements help estimate the resulting change in velocity and position.
IMU estimates alone tend to drift because small measurement errors accumulate over time.
2. GPS factor
The GPS factor constrains the estimated position using the GPS position measurement.
For example:
GPS position:
East  = 10.5 m
North = 4.2 m
Up    = 0.3 m

GTSAM uses the GPS observation to help correct the position estimate.
GPS is generally less frequent than IMU data, but it provides an important external position reference.
3. IMU bias constraints
Accelerometers and gyroscopes can have biases. GTSAM can estimate these biases and model how they evolve over time.
This helps prevent the optimizer from treating every sensor reading as perfectly accurate.
C. How does GTSAM combine the factors?
Conceptually, GTSAM seeks a state estimate that best satisfies the available measurement constraints.
A simplified objective is:
\[
X^*=\arg\min_X
\left(
E_{\mathrm{IMU}}(X)
+
E_{\mathrm{GPS}}(X)
+
E_{\mathrm{bias}}(X)
+
E_{\mathrm{prior}}(X)
\right)
\]
Where:
- \(E_{\mathrm{IMU}}\) measures disagreement with the IMU motion constraints.
- \(E_{\mathrm{GPS}}\) measures disagreement with GPS position measurements.
- \(E_{\mathrm{bias}}\) represents the IMU bias constraints.
- \(E_{\mathrm{prior}}\) represents initial-state constraints, when included.
The errors are weighted using noise models. Measurements considered more reliable can exert greater influence on the estimate.
This is the key idea: GTSAM does not simply average the GPS and IMU positions. It optimizes a set of connected states to find a solution consistent with the measurements and their uncertainties.
D. What is ISAM2?
Your fusion core uses ISAM2, an incremental smoothing and mapping optimizer.
As new measurements arrive, the system can add new variables and factors and update its estimate incrementally.

