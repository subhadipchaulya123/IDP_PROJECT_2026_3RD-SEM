import pandas as pd

# ============================================================
# FILE PATHS
# ============================================================

ACC_FILE = "data/AccelerometerUncalibrated.csv"
GYRO_FILE = "data/GyroscopeUncalibrated.csv"
GPS_FILE = "data/Location.csv"

OUTPUT_FILE = "output/Synchronized_IMU.csv"


# ============================================================
# READ DATA
# ============================================================

acc = pd.read_csv(ACC_FILE)
gyro = pd.read_csv(GYRO_FILE)
gps = pd.read_csv(GPS_FILE)


# ============================================================
# RENAME SENSOR AXES
# ============================================================

acc = acc.rename(columns={
    "x": "ax",
    "y": "ay",
    "z": "az"
})

gyro = gyro.rename(columns={
    "x": "gx",
    "y": "gy",
    "z": "gz"
})


# ============================================================
# CONVERT TIMESTAMP
# ============================================================
# Original timestamp is nanoseconds.
# Convert nanoseconds -> seconds.
# ============================================================

acc["time_sec"] = acc["time"] / 1e9
gyro["time_sec"] = gyro["time"] / 1e9
gps["time_sec"] = gps["time"] / 1e9


# ============================================================
# SORT BY TIME
# ============================================================

acc = acc.sort_values("time_sec").reset_index(drop=True)
gyro = gyro.sort_values("time_sec").reset_index(drop=True)


# ============================================================
# SYNCHRONIZE ACCELEROMETER + GYROSCOPE
# ============================================================
# Nearest timestamp matching.
#
# 0.01 seconds = 10 milliseconds.
# ============================================================

imu = pd.merge_asof(
    acc[[
        "time_sec",
        "seconds_elapsed",
        "ax",
        "ay",
        "az"
    ]],
    gyro[[
        "time_sec",
        "gx",
        "gy",
        "gz"
    ]],
    on="time_sec",
    direction="nearest",
    tolerance=0.01
)


# ============================================================
# REMOVE UNMATCHED SAMPLES
# ============================================================

imu = imu.dropna(
    subset=["gx", "gy", "gz"]
).reset_index(drop=True)


# ============================================================
# CALCULATE TIME FROM START
# ============================================================

imu["time_from_start"] = (
    imu["time_sec"] - imu["time_sec"].iloc[0]
)


# ============================================================
# REORDER COLUMNS
# ============================================================

imu = imu[[
    "time_sec",
    "time_from_start",
    "ax",
    "ay",
    "az",
    "gx",
    "gy",
    "gz"
]]


# ============================================================
# SAVE
# ============================================================

imu.to_csv(
    OUTPUT_FILE,
    index=False
)


# ============================================================
# GPS INFORMATION
# ============================================================

print("\n========================================")
print("       IMU SYNCHRONIZATION")
print("========================================")

print(f"Accelerometer readings : {len(acc)}")
print(f"Gyroscope readings     : {len(gyro)}")
print(f"Synchronized IMU       : {len(imu)}")
print(f"GPS readings           : {len(gps)}")

print("\nIMU time range:")
print(f"Start: {imu['time_from_start'].iloc[0]:.6f} s")
print(f"End  : {imu['time_from_start'].iloc[-1]:.6f} s")

print("\nFirst 5 synchronized samples:")
print(imu.head().to_string(index=False))

print("\nSaved:")
print(OUTPUT_FILE)
