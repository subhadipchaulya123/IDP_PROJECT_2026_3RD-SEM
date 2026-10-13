#!/usr/bin/python3

"""FUSION core.

Provides functionality to fuse IMU, GPS and 6DOF
(absolute / relative) pose data using GTSAM's
incremental smoothing and mapping based on ISAM2.
"""

from gtsam import gtsam
import numpy as np
from collections import deque
import heapq
import time


# =========================================================
# GTSAM POSE -> NUMPY
# =========================================================

def gtsam_pose_to_numpy(gtsam_pose):
    """Convert GTSAM Pose3 to numpy arrays.

    Returns:
        position    : [x, y, z]
        orientation : [x, y, z, w]
    """

    position = np.array([
        gtsam_pose.x(),
        gtsam_pose.y(),
        gtsam_pose.z()
    ])

    # Your installed GTSAM exposes matrix(), while
    # quaternion() is not available as a getter.
    R = np.asarray(
        gtsam_pose.rotation().matrix()
    )

    trace = np.trace(R)

    if trace > 0.0:

        s = 2.0 * np.sqrt(trace + 1.0)

        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s

    elif (
        R[0, 0] > R[1, 1]
        and R[0, 0] > R[2, 2]
    ):

        s = 2.0 * np.sqrt(
            1.0
            + R[0, 0]
            - R[1, 1]
            - R[2, 2]
        )

        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s

    elif R[1, 1] > R[2, 2]:

        s = 2.0 * np.sqrt(
            1.0
            + R[1, 1]
            - R[0, 0]
            - R[2, 2]
        )

        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s

    else:

        s = 2.0 * np.sqrt(
            1.0
            + R[2, 2]
            - R[0, 0]
            - R[1, 1]
        )

        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    orientation = np.array([
        x,
        y,
        z,
        w
    ])

    return position, orientation


# =========================================================
# NUMPY -> GTSAM POSE
# =========================================================

def numpy_pose_to_gtsam(position, orientation):
    """Convert numpy arrays to GTSAM Pose3.

    orientation format:
        [rx, ry, rz, rw]
    """

    return gtsam.Pose3(
        gtsam.Rot3.Quaternion(
            orientation[3],
            orientation[0],
            orientation[1],
            orientation[2]
        ),
        np.asarray(
            position,
            dtype=float
        )
    )


# =========================================================
# GTSAM FUSION CORE
# =========================================================

class GtsamFusionCore():

    """Core functions for ISAM2 fusion."""

    def __init__(self, params):

        # -------------------------------------------------
        # Measurement buffers
        # -------------------------------------------------

        self.__imu_measurements_predict = []

        self.__imu_measurements_optimize = []

        self.__imu_samples = []

        self.__opt_measurements = []

        self.__opt_meas_buffer_time = (
            params['opt_meas_buffer_time']
        )

        # -------------------------------------------------
        # ISAM2 initialization
        # -------------------------------------------------

        isam2_params = gtsam.ISAM2Params()

        isam2_params.setRelinearizeThreshold(
            params['relinearize_th']
        )

        # Compatibility with installed GTSAM
        isam2_params.relinearizeSkip = (
            params['relinearize_skip']
        )

        isam2_params.setFactorization(
            params['factorization']
        )

        self.__isam2 = gtsam.ISAM2(
            isam2_params
        )

        self.__graph = (
            gtsam.NonlinearFactorGraph()
        )

        self.__initial_estimate = (
            gtsam.Values()
        )

        self.__prev_relative_pose = None

        self.__prev_pose_key = None

        self.__min_imu_sample_count_for_integration = 2

        # -------------------------------------------------
        # ISAM2 keys
        # -------------------------------------------------

        # Installed GTSAM expects a string here,
        # not ord('x').
        self.__pose_key = gtsam.symbol(
            'x',
            0
        )

        self.__vel_key = gtsam.symbol(
            'v',
            0
        )

        self.__bias_key = gtsam.symbol(
            'b',
            0
        )

        # -------------------------------------------------
        # IMU preintegration
        # -------------------------------------------------

        self.__pre_integration_params = (
            gtsam.PreintegrationParams(
                np.asarray(
                    params['g']
                )
            )
        )

        self.__pre_integration_params.setAccelerometerCovariance(
            np.eye(3)
            * np.power(
                params['sigma_accelerometer'],
                2
            )
        )

        self.__pre_integration_params.setGyroscopeCovariance(
            np.eye(3)
            * np.power(
                params['sigma_gyroscope'],
                2
            )
        )

        self.__pre_integration_params.setIntegrationCovariance(
            np.eye(3)
            * params['sigma_integration'] ** 2
        )

        if hasattr(self.__pre_integration_params, "setUse2ndOrderCoriolis"):
            self.__pre_integration_params.setUse2ndOrderCoriolis(
                params["use_2nd_order_coriolis"]
            )

        self.__pre_integration_params.setOmegaCoriolis(
            np.array(
                params['omega_coriolis']
            )
        )

        self.__pre_integration_params.setBodyPSensor(
            numpy_pose_to_gtsam(
                params['b2s_pos'],
                params['b2s_ori']
            )
        )

        # -------------------------------------------------
        # Initial state
        # -------------------------------------------------

        self.__current_time = 0

        self.__current_pose = (
            numpy_pose_to_gtsam(
                params['init_pos'],
                params['init_ori']
            )
        )

        self.__predicted_pose = (
            self.__current_pose
        )

        self.__current_vel = np.asarray(
            params['init_vel']
        )

        # Installed GTSAM API
        self.__current_bias = (
            gtsam.imuBias.ConstantBias(
                np.asarray(
                    params['init_acc_bias']
                ),
                np.asarray(
                    params['init_gyr_bias']
                )
            )
        )

        # -------------------------------------------------
        # Store state for prediction
        # -------------------------------------------------

        self.__last_opt_time = (
            self.__current_time
        )

        self.__last_opt_pose = (
            self.__current_pose
        )

        self.__last_opt_vel = (
            self.__current_vel
        )

        self.__last_opt_bias = (
            self.__current_bias
        )

        self.__imu_accum = (
            gtsam.PreintegratedImuMeasurements(
                self.__pre_integration_params
            )
        )

        # -------------------------------------------------
        # Initial uncertainty
        # -------------------------------------------------

        # Installed GTSAM API
        self.__sigma_init_pose = (
            gtsam.noiseModel.Isotropic.Sigmas(
                np.array([
                    params['sigma_init_ori'],
                    params['sigma_init_ori'],
                    params['sigma_init_ori'],

                    params['sigma_init_pos'],
                    params['sigma_init_pos'],
                    params['sigma_init_pos']
                ])
            )
        )

        self.__sigma_init_vel = (
            gtsam.noiseModel.Isotropic.Sigmas(
                np.array([
                    params['sigma_init_vel'],
                    params['sigma_init_vel'],
                    params['sigma_init_vel']
                ])
            )
        )

        self.__sigma_init_bias = (
            gtsam.noiseModel.Isotropic.Sigmas(
                np.array([
                    params['sigma_acc_init_bias'],
                    params['sigma_acc_init_bias'],
                    params['sigma_acc_init_bias'],

                    params['sigma_gyr_init_bias'],
                    params['sigma_gyr_init_bias'],
                    params['sigma_gyr_init_bias']
                ])
            )
        )

        # -------------------------------------------------
        # Measurement noise
        # -------------------------------------------------

        self.__pose_noise = (
            gtsam.noiseModel.Isotropic.Sigmas(
                np.concatenate((
                    params['sigma_pose_rot'],
                    params['sigma_pose_pos']
                ))
            )
        )

        self.__gps_noise = (
            gtsam.noiseModel.Isotropic.Sigmas(
                np.asarray(
                    params['sigma_gps']
                )
            )
        )

        self.__bias_noise = (
            gtsam.noiseModel.Isotropic.Sigmas(
                np.concatenate((
                    params['sigma_acc_bias_evol'],
                    params['sigma_gyr_bias_evol']
                ))
            )
        )

        # -------------------------------------------------
        # Prior pose factor
        # -------------------------------------------------

        prior_pose_factor = (
            gtsam.PriorFactorPose3(
                self.__pose_key,
                self.__current_pose,
                self.__sigma_init_pose
            )
        )

        self.__graph.add(
            prior_pose_factor
        )

        # -------------------------------------------------
        # Prior velocity factor
        # -------------------------------------------------

        prior_vel_factor = (
            gtsam.PriorFactorVector(
                self.__vel_key,
                self.__current_vel,
                self.__sigma_init_vel
            )
        )

        self.__graph.add(
            prior_vel_factor
        )

        # -------------------------------------------------
        # Prior bias factor
        # -------------------------------------------------

        prior_bias_factor = (
            gtsam.PriorFactorConstantBias(
                self.__bias_key,
                self.__current_bias,
                self.__sigma_init_bias
            )
        )

        self.__graph.add(
            prior_bias_factor
        )

        # -------------------------------------------------
        # Initial estimates
        # -------------------------------------------------

        self.__initial_estimate.insert(
            self.__pose_key,
            self.__current_pose
        )

        self.__initial_estimate.insert(
            self.__vel_key,
            self.__current_vel
        )

        self.__initial_estimate.insert(
            self.__bias_key,
            self.__current_bias
        )

    # =====================================================
    # RELATIVE POSE
    # =====================================================

    def add_relative_pose_measurement(
        self,
        time,
        position,
        orientation
    ):

        heapq.heappush(
            self.__opt_measurements,
            (
                time,
                'relative_pose',
                position,
                orientation
            )
        )

        buffer_time = (
            heapq.nlargest(
                1,
                self.__opt_measurements
            )[0][0]
            -
            heapq.nsmallest(
                1,
                self.__opt_measurements
            )[0][0]
        )

        if (
            buffer_time
            >= self.__opt_meas_buffer_time
        ):

            self.__trigger_update(
                heapq.heappop(
                    self.__opt_measurements
                )
            )

    # =====================================================
    # ABSOLUTE POSE
    # =====================================================

    def add_absolute_pose_measurement(
        self,
        time,
        position,
        orientation
    ):

        heapq.heappush(
            self.__opt_measurements,
            (
                time,
                'absolute_pose',
                position,
                orientation
            )
        )

        buffer_time = (
            heapq.nlargest(
                1,
                self.__opt_measurements
            )[0][0]
            -
            heapq.nsmallest(
                1,
                self.__opt_measurements
            )[0][0]
        )

        if (
            buffer_time
            >= self.__opt_meas_buffer_time
        ):

            self.__trigger_update(
                heapq.heappop(
                    self.__opt_measurements
                )
            )

    # =====================================================
    # GPS
    # =====================================================

    def add_gps_measurement(
        self,
        time,
        position
    ):

        heapq.heappush(
            self.__opt_measurements,
            (
                time,
                'gps',
                position
            )
        )

        buffer_time = (
            heapq.nlargest(
                1,
                self.__opt_measurements
            )[0][0]
            -
            heapq.nsmallest(
                1,
                self.__opt_measurements
            )[0][0]
        )

        if (
            buffer_time
            >= self.__opt_meas_buffer_time
        ):

            self.__trigger_update(
                heapq.heappop(
                    self.__opt_measurements
                )
            )

    # =====================================================
    # IMU
    # =====================================================

    def add_imu_measurement(
        self,
        time,
        linear_acceleration,
        angular_velocity,
        dt
    ):

        heapq.heappush(
            self.__imu_measurements_predict,
            (
                time,
                linear_acceleration,
                angular_velocity,
                dt
            )
        )

        heapq.heappush(
            self.__imu_measurements_optimize,
            (
                time,
                linear_acceleration,
                angular_velocity,
                dt
            )
        )

        return self.__imu_predict()

    # =====================================================
    # TRIGGER UPDATE
    # =====================================================

    def __trigger_update(
        self,
        measurement
    ):

        meas_time = measurement[0]

        meas_type = measurement[1]

        imu_samples = []

        while True:

            if not self.__imu_measurements_optimize:
                break

            imu_sample = (
                heapq.heappop(
                    self.__imu_measurements_optimize
                )
            )

            if imu_sample[0] < meas_time:

                imu_samples.append(
                    imu_sample
                )

            else:

                # IMPORTANT FIX:
                # Keep the first IMU sample at/after
                # the measurement time for the next
                # integration window.

                heapq.heappush(
                    self.__imu_measurements_optimize,
                    imu_sample
                )

                break

        if (
            len(imu_samples)
            < self.__min_imu_sample_count_for_integration
        ):

            # Put samples back.

            for imu_sample in imu_samples:

                heapq.heappush(
                    self.__imu_measurements_optimize,
                    imu_sample
                )

            print(
                "Ignoring measurement at: {}".format(
                    meas_time
                )
            )

            return

        # -------------------------------------------------
        # Relative pose initialization
        # -------------------------------------------------

        if (
            meas_type == 'relative_pose'
            and
            not self.__prev_relative_pose
        ):

            pose = numpy_pose_to_gtsam(
                measurement[2],
                measurement[3]
            )

            self.__prev_relative_pose = pose

            self.__prev_pose_key = (
                self.__pose_key
            )

        else:

            # -------------------------------------------------
            # New pose / velocity / bias keys
            # -------------------------------------------------

            self.__pose_key += 1

            self.__vel_key += 1

            self.__bias_key += 1

            # -------------------------------------------------
            # GPS
            # -------------------------------------------------

            if meas_type == 'gps':

                position = measurement[2]

                # Installed GTSAM does not expose Point3
                # in this Python API. GPSFactor accepts
                # a 3-element vector.

                point = np.asarray(
                    position,
                    dtype=float
                )

                gps_factor = (
                    gtsam.GPSFactor(
                        self.__pose_key,
                        point,
                        self.__gps_noise
                    )
                )

                self.__graph.add(
                    gps_factor
                )

            # -------------------------------------------------
            # Relative pose
            # -------------------------------------------------

            elif meas_type == 'relative_pose':

                pose = numpy_pose_to_gtsam(
                    measurement[2],
                    measurement[3]
                )

                between_pose_factor = (
                    gtsam.BetweenFactorPose3(
                        self.__prev_pose_key,
                        self.__pose_key,
                        self.__prev_relative_pose.between(
                            pose
                        ),
                        self.__pose_noise
                    )
                )

                self.__graph.add(
                    between_pose_factor
                )

                self.__prev_relative_pose = pose

                self.__prev_pose_key = (
                    self.__pose_key
                )

            # -------------------------------------------------
            # Absolute pose
            # -------------------------------------------------

            elif meas_type == 'absolute_pose':

                pose = numpy_pose_to_gtsam(
                    measurement[2],
                    measurement[3]
                )

                pose_factor = (
                    gtsam.PriorFactorPose3(
                        self.__pose_key,
                        pose,
                        self.__pose_noise
                    )
                )

                self.__graph.add(
                    pose_factor
                )

            # -------------------------------------------------
            # Optimize
            # -------------------------------------------------

            self.__optimize(
                meas_time,
                imu_samples
            )

    # =====================================================
    # IMU PREDICTION
    # =====================================================

    def __imu_predict(self):

        if (
            self.__current_time
            > self.__last_opt_time
        ):

            # -------------------------------------------------
            # Store optimized state
            # -------------------------------------------------

            self.__last_opt_time = (
                self.__current_time
            )

            self.__last_opt_pose = (
                self.__current_pose
            )

            self.__last_opt_vel = (
                self.__current_vel
            )

            self.__last_opt_bias = (
                self.__current_bias
            )

            # -------------------------------------------------
            # Reset integration
            # -------------------------------------------------

            self.__imu_accum.resetIntegration()

            new_imu_samples = []

            for sample in self.__imu_samples:

                if (
                    sample[0]
                    > self.__last_opt_time
                ):

                    self.__imu_accum.integrateMeasurement(
                        sample[1],
                        sample[2],
                        sample[3]
                    )

                    new_imu_samples.append(
                        sample
                    )

            self.__imu_samples = (
                new_imu_samples
            )

        # -----------------------------------------------------
        # Get oldest prediction sample
        # -----------------------------------------------------

        (
            time_stamp,
            linear_acceleration,
            angular_velocity,
            dt
        ) = heapq.heappop(
            self.__imu_measurements_predict
        )

        # -----------------------------------------------------
        # Store sample
        # -----------------------------------------------------

        self.__imu_samples.append(
            (
                time_stamp,
                linear_acceleration,
                angular_velocity,
                dt
            )
        )

        # -----------------------------------------------------
        # Integrate
        # -----------------------------------------------------

        self.__imu_accum.integrateMeasurement(
            linear_acceleration,
            angular_velocity,
            dt
        )

        # -----------------------------------------------------
        # Predict
        # -----------------------------------------------------

        predicted_nav_state = (
            self.__imu_accum.predict(
                gtsam.NavState(
                    self.__last_opt_pose,
                    self.__last_opt_vel
                ),
                self.__last_opt_bias
            )
        )

        pos, ori = gtsam_pose_to_numpy(
            predicted_nav_state.pose()
        )

        return (
            pos,
            ori,
            self.__last_opt_vel,
            self.__last_opt_bias.accelerometer(),
            self.__last_opt_bias.gyroscope()
        )

    # =====================================================
    # IMU UPDATE
    # =====================================================

    def __imu_update(
        self,
        imu_samples
    ):

        # -------------------------------------------------
        # New preintegration
        # -------------------------------------------------

        imu_accum = (
            gtsam.PreintegratedImuMeasurements(
                self.__pre_integration_params
            )
        )

        # -------------------------------------------------
        # Integrate samples
        # -------------------------------------------------

        for imu_sample in imu_samples:

            imu_accum.integrateMeasurement(
                imu_sample[1],
                imu_sample[2],
                imu_sample[3]
            )

        # -------------------------------------------------
        # Last optimized state
        # -------------------------------------------------

        last_opt_pose = (
            self.__current_pose
        )

        last_opt_vel = (
            self.__current_vel
        )

        last_opt_bias = (
            self.__current_bias
        )

        # -------------------------------------------------
        # Predict
        # -------------------------------------------------

        predicted_nav_state = (
            imu_accum.predict(
                gtsam.NavState(
                    last_opt_pose,
                    last_opt_vel
                ),
                last_opt_bias
            )
        )

        # -------------------------------------------------
        # IMU factor
        # -------------------------------------------------

        imu_factor = (
            gtsam.ImuFactor(
                self.__pose_key - 1,
                self.__vel_key - 1,
                self.__pose_key,
                self.__vel_key,
                # IMU measurements are preintegrated using the bias
                # at the START of this interval (bias_i), not bias_j.
                self.__bias_key - 1,
                imu_accum
            )
        )

        self.__graph.add(
            imu_factor
        )

        return predicted_nav_state

    # =====================================================
    # OPTIMIZATION
    # =====================================================

    def __optimize(
        self,
        meas_time,
        imu_samples
    ):

        # -------------------------------------------------
        # IMU preintegration
        # -------------------------------------------------

        predicted_nav_state = (
            self.__imu_update(
                imu_samples
            )
        )

        # -------------------------------------------------
        # Initial estimates
        # -------------------------------------------------

        self.__initial_estimate.insert(
            self.__pose_key,
            predicted_nav_state.pose()
        )

        self.__initial_estimate.insert(
            self.__vel_key,
            predicted_nav_state.velocity()
        )

        self.__initial_estimate.insert(
            self.__bias_key,
            self.__current_bias
        )

        # -------------------------------------------------
        # Bias evolution
        # -------------------------------------------------

        bias_factor = (
            gtsam.BetweenFactorConstantBias(
                self.__bias_key - 1,
                self.__bias_key,
                gtsam.imuBias.ConstantBias(),
                self.__bias_noise
            )
        )

        self.__graph.add(
            bias_factor
        )

        # -------------------------------------------------
        # ISAM2
        # -------------------------------------------------

        result = self.__isam2_update()

        if result:

            # -------------------------------------------------
            # Update state
            # -------------------------------------------------

            self.__current_time = (
                meas_time
            )

            self.__current_pose = (
                result.atPose3(
                    self.__pose_key
                )
            )

            self.__current_vel = (
                result.atVector(
                    self.__vel_key
                )
            )

            self.__current_bias = (
                result.atConstantBias(
                    self.__bias_key
                )
            )

    # =====================================================
    # ISAM2 UPDATE
    # =====================================================

    def __isam2_update(self):

        result = None

        try:

            self.__isam2.update(
                self.__graph,
                self.__initial_estimate
            )

            result = (
                self.__isam2.calculateEstimate()
            )

        except RuntimeError as e:

            print(
                "Runtime error in optimization: {}".format(
                    e
                )
            )

        except IndexError as e:

            print(
                "Index error in optimization: {}".format(
                    e
                )
            )

        except TypeError as e:

            print(
                "Type error in optimization: {}".format(
                    e
                )
            )

        # -------------------------------------------------
        # Reset
        # -------------------------------------------------

        self.__graph.resize(0)

        self.__initial_estimate.clear()

        return result
