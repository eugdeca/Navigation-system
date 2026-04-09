# 🛩️ Multi-Sensor Navigation System for UAV

![MATLAB](https://img.shields.io/badge/MATLAB-Data_Fusion-blue.svg)

## 📌 Project Overview
This repository contains the implementation of a navigation system for an Unmanned Aerial Vehicle (UAV) operating near the "Polo A" of the Engineering Department in Pisa. The flight scenario covers a localized area of a few tens of meters.

The core objective is the joint estimation of the UAV's Position, Velocity, and Attitude using a single **Extended Kalman Filter (EKF)**. The system relies on a flat-earth model, where the North-East-Down (NED) navigation frame is considered inertial.

## ⚙️ System Architecture & Sensors
The system fuses data from multiple simulated sensors operating at different sampling frequencies:

* **UWB (Ultra-Wideband) Sensor:** Provides distance-only measurements (no triangulation) from 6 base stations located at known positions. Runs at **50 Hz**.
* **Accelerometer:** Measures the 3 acceleration components in the body frame at **100 Hz**. The sensor is assumed to have no bias.
* **Gyroscope:** Measures the 3 angular velocity components in the body frame at **100 Hz**. The measurements are affected by a constant bias, which is continuously estimated by the filter.
* **Magnetometer:** Provides the direction of the magnetic north (defined via WMM) in the body frame at **100 Hz**.

Additionally, the **TRIAD** (Two-Vector Attitude Determination) algorithm is used to pre-calculate attitude quaternions from the acceleration and magnetic field measurements.

### EKF State Vector
The Extended Kalman Filter is designed to continuously estimate the following variables:
* 3x Position (North, East, Down)
* 3x Velocity 
* 3x Acceleration (Body frame)
* 4x Attitude Quaternions
* 3x Angular Velocity (Body frame)
* 3x Gyroscope Bias

## 📊 Results & Performance
System observability is slightly affected by the UWB sensors running at half the system's sampling frequency and by the geometric distribution of the 6 UWB stations (which impacts the estimation along the *down* axis).

To evaluate the system's robustness, two datasets were generated via waypoint interpolation:
1. **High Dynamics:** A flight path with strong acceleration changes, which highlights the operational limits of the TRIAD algorithm.
2. **Low Dynamics:** A flight path with near-zero accelerations, resulting in significantly improved attitude and position estimation.

### Position Estimation
<img width="1840" height="925" alt="image" src="https://github.com/user-attachments/assets/10cd0b65-ea66-48fc-8743-706d168a3ad5" />
*Comparison between the true 3D position and the EKF point-by-point estimation.*

### Attitude Estimation
<img width="1908" height="959" alt="image" src="https://github.com/user-attachments/assets/5e5671a5-016e-464e-ab5c-a4ba03ef7dc8" />
*Comparison between the true attitude and the EKF point-by-point estimation.*


## 👤 Author
* **Eugenio Delli Carri**
