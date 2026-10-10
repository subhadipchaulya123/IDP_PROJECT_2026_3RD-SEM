
flowchart TD
    A["📡 Sensor Data Collection"]

    A --> B["Accelerometer CSV"]
    A --> C["Gyroscope CSV"]
    A --> D["GPS Location CSV"]

    B --> E["synchronize.py"]
    C --> E

    E --> F["Synchronized_IMU.csv"]

    D --> G["gps_to_enu.py"]
    G --> H["GPS_ENU.csv"]

    F --> I["gtsam_fusion_csv.py"]
    H --> I

    I --> J["GTSAM Fusion Core"]

    J --> K["IMU Preintegration"]
    J --> L["GPS Position Factor"]
    J --> M["IMU Bias Estimation"]

    K --> N["Factor Graph"]
    L --> N
    M --> N

    N --> O["ISAM2 Optimization"]

    O --> P["Fused_Trajectory.csv"]

    P --> Q["plot_trajectory.py"]

    Q --> R["Fused_Trajectory.png"]

    style A fill:#1565C0,color:#FFFFFF,stroke:#0D47A1
    style J fill:#6A1B9A,color:#FFFFFF,stroke:#4A148C
    style N fill:#00897B,color:#FFFFFF,stroke:#004D40
    style O fill:#EF6C00,color:#FFFFFF,stroke:#E65100
    style R fill:#2E7D32,color:#FFFFFF,stroke:#1B5E20
