import os
import sys
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gtsam_fusion_core import GtsamFusionCore


IMU_FILE = "output/Synchronized_IMU.csv"
GPS_FILE = "output/GPS_ENU.csv"
OUTPUT_FILE = "output/Fused_Trajectory.csv"


params = {
    "opt_meas_buffer_time": 0.0,

    "relinearize_th": 0.01,
    "relinearize_skip": 1,
    "factorization": "CHOLESKY",

    "g": [0.0, 0.0, -9.81],

    "sigma_accelerometer": 0.1,
    "sigma_gyroscope": 0.1,
    "sigma_integration": 1e-4,

    "use_2nd_order_coriolis": False,
    "omega_coriolis": [0.0, 0.0, 0.0],

    "b2s_pos": [0.0, 0.0, 0.0],
    "b2s_ori": [0.0, 0.0, 0.0, 1.0],

    "init_pos": [0.0, 0.0, 0.0],
    "init_ori": [0.0, 0.0, 0.0, 1.0],
    "init_vel": [0.0, 0.0, 0.0],

    "init_acc_bias": [0.0, 0.0, 0.0],
    "init_gyr_bias": [0.0, 0.0, 0.0],

    "sigma_init_ori": 0.1,
    "sigma_init_pos": 1.0,
    "sigma_init_vel": 1.0,

    "sigma_acc_init_bias": 0.1,
    "sigma_gyr_init_bias": 0.1,

    "sigma_pose_rot": [0.1, 0.1, 0.1],
    "sigma_pose_pos": [1.0, 1.0, 1.0],

    "sigma_gps": [1.0, 1.0, 1.0],

    "sigma_acc_bias_evol": [0.01, 0.01, 0.01],
    "sigma_gyr_bias_evol": [0.01, 0.01, 0.01],
}


imu = pd.read_csv(IMU_FILE)
gps = pd.read_csv(GPS_FILE)

imu = imu.sort_values("time_sec").reset_index(drop=True)
gps = gps.sort_values("time_sec").reset_index(drop=True)

print("IMU samples:", len(imu))
print("GPS samples:", len(gps))

fusion = GtsamFusionCore(params)

events = []

for _, row in imu.iterrows():
    events.append((float(row["time_sec"]), "imu", row))

for _, row in gps.iterrows():
    events.append((float(row["time_sec"]), "gps", row))

events.sort(key=lambda e: e[0])


# ---------------------------------------------------------
# IMPORTANT:
# Do NOT save the high-rate IMU prediction as the fused
# trajectory. Those predictions can be corrected later by
# GTSAM when a GPS factor is added.
#
# Instead, save the optimized GTSAM state after every GPS
# update. This represents the actual GPS+IMU fused solution.
# ---------------------------------------------------------

trajectory = []

previous_imu_time = None

for time_sec, event_type, row in events:

    if event_type == "imu":

        if previous_imu_time is None:
            previous_imu_time = time_sec
            continue

        dt = time_sec - previous_imu_time

        if dt <= 0:
            continue

        acceleration = np.array([
            float(row["ax"]),
            float(row["ay"]),
            float(row["az"])
        ])

        gyro = np.array([
            float(row["gx"]),
            float(row["gy"]),
            float(row["gz"])
        ])

        fusion.add_imu_measurement(
            time_sec,
            acceleration,
            gyro,
            dt
        )

        previous_imu_time = time_sec

    elif event_type == "gps":

        gps_position = np.array([
            float(row["east_m"]),
            float(row["north_m"]),
            float(row["up_m"])
        ])

        fusion.add_gps_measurement(
            time_sec,
            gps_position
        )

        # Read the CURRENT optimized GTSAM state.
        # These are Python name-mangled private members
        # of GtsamFusionCore.
        try:
            pose = fusion._GtsamFusionCore__current_pose

            position = pose.translation()

            trajectory.append([
                time_sec,
                float(position[0]),
                float(position[1]),
                float(position[2])
            ])

        except Exception as e:
            print("Could not read optimized GTSAM state:", e)


trajectory_df = pd.DataFrame(
    trajectory,
    columns=[
        "time_sec",
        "east_m",
        "north_m",
        "up_m"
    ]
)

trajectory_df.to_csv(
    OUTPUT_FILE,
    index=False
)

print()
print("Fusion finished.")
print("Optimized trajectory points:", len(trajectory_df))
print("Saved:", OUTPUT_FILE)

if len(trajectory_df) > 0:

    east_range = (
        trajectory_df["east_m"].max()
        - trajectory_df["east_m"].min()
    )

    north_range = (
        trajectory_df["north_m"].max()
        - trajectory_df["north_m"].min()
    )

    distance = np.sqrt(
        trajectory_df["east_m"]**2
        + trajectory_df["north_m"]**2
    )

    print()
    print("Trajectory statistics")
    print("---------------------")
    print("East range :", east_range, "m")
    print("North range:", north_range, "m")
    print("Maximum distance from origin:",
          distance.max(), "m")
