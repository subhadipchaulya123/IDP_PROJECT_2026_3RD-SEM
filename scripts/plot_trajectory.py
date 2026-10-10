import pandas as pd
import matplotlib.pyplot as plt

gps = pd.read_csv("output/GPS_ENU.csv")
fused = pd.read_csv("output/Fused_Trajectory.csv")

print("GPS points:", len(gps))
print("Fused points:", len(fused))

plt.figure(figsize=(10, 8))

plt.plot(
    gps["east_m"],
    gps["north_m"],
    "o--",
    markersize=4,
    label="GPS"
)

plt.plot(
    fused["east_m"],
    fused["north_m"],
    linewidth=2,
    label="GTSAM GPS + IMU"
)

plt.scatter(
    fused["east_m"].iloc[0],
    fused["north_m"].iloc[0],
    s=100,
    marker="o",
    label="Start"
)

plt.scatter(
    fused["east_m"].iloc[-1],
    fused["north_m"].iloc[-1],
    s=100,
    marker="x",
    label="End"
)

plt.xlabel("East (m)")
plt.ylabel("North (m)")
plt.title("GPS vs GTSAM GPS-IMU Fused Trajectory")

plt.grid(True)
plt.axis("equal")
plt.legend()
plt.tight_layout()

plt.savefig(
    "output/Fused_Trajectory.png",
    dpi=300
)

plt.show()

print("Saved: output/Fused_Trajectory.png")
