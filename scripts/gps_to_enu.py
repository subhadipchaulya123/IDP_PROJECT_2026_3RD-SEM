import pandas as pd
import numpy as np
import utm

GPS_FILE = "data/Location.csv"
OUTPUT_FILE = "output/GPS_ENU.csv"

# Read GPS data
gps = pd.read_csv(GPS_FILE)

# Convert nanoseconds to seconds
gps["time_sec"] = gps["time"] / 1e9

# Remove invalid GPS coordinates
gps = gps.dropna(
    subset=["latitude", "longitude"]
).copy()

# Sort by timestamp
gps = gps.sort_values("time_sec").reset_index(drop=True)

# First GPS point is the origin
lat0 = gps["latitude"].iloc[0]
lon0 = gps["longitude"].iloc[0]

x0, y0, zone_number, zone_letter = utm.from_latlon(
    lat0,
    lon0
)

# Convert GPS to local East/North coordinates
east = []
north = []

for lat, lon in zip(
    gps["latitude"],
    gps["longitude"]
):
    x, y, zone, letter = utm.from_latlon(
        lat,
        lon
    )

    east.append(x - x0)
    north.append(y - y0)

# Altitude
if "altitude" in gps.columns:
    altitude = gps["altitude"].fillna(
        gps["altitude"].iloc[0]
    )

    up = altitude - altitude.iloc[0]
else:
    up = np.zeros(len(gps))

# Create output
gps_enu = pd.DataFrame({
    "time_sec": gps["time_sec"],
    "time_from_start": (
        gps["time_sec"] -
        gps["time_sec"].iloc[0]
    ),
    "latitude": gps["latitude"],
    "longitude": gps["longitude"],
    "east_m": east,
    "north_m": north,
    "up_m": up
})

# Save
gps_enu.to_csv(
    OUTPUT_FILE,
    index=False
)

print("\n========================================")
print("         GPS → LOCAL ENU")
print("========================================")

print("GPS readings:", len(gps_enu))

print("\nOrigin:")
print("Latitude :", lat0)
print("Longitude:", lon0)

print("\nFirst 5 points:")
print(
    gps_enu.head().to_string(index=False)
)

print("\nSaved:")
print(OUTPUT_FILE)
