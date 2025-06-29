# 🌋 ESP32-Based Smart Sensor Node System for Real-Time Monitoring & Anomaly Detection

This project is a full-stack embedded + software system designed for **real-time landslide/environmental monitoring** using ESP32 nodes, CH32 microcontrollers, and Python-based GUIs with built-in anomaly detection.

---

## 📁 File Breakdown

### `1. node.ino` – **ESP32 Sensor Node Firmware**

This file contains the C++ code running on an ESP32 microcontroller, functioning as a smart sensor node. It handles a variety of critical tasks:

- **🔋 Ultra-Low Power (ULP) Co-processor Integration**  
  - Enables deep sleep and wake-up via ULP for energy efficiency.

- **📡 ESP-NOW Communication**  
  - Mesh-style communication between ESP32 nodes with **failover/failback** support.

- **📊 PWM Decoding**  
  - Reads sensor data from an external **CH32V003** via PWM.

- **💾 Non-Volatile Storage (NVS)**  
  - Persists mode (Standby/Data Collection) and target node ID across reboots.

- **📈 Data Collection Mode**  
  - Enables high-frequency sensor data acquisition.

- **🖥️ TCP Server with REST-style Endpoints**  
  - Exposes:
    - `GET /sensor_data`: Returns sensor data (Rain, Soil, Vibration, Tilt) as JSON.
    - `POST /set_mode`: Remotely switch between standby and data collection modes.

---

### `2. gui.py` – **Basic Sensor Monitoring GUI (PyQt5)**

A desktop GUI for **visualizing real-time sensor data** using PyQt5 and pyqtgraph.

- **🔌 TCP Client**  
  - Connects to ESP32 Node 1 (`192.168.4.1`) via TCP.

- **📊 Real-time Graphs**  
  - Displays live plots for:
    - Rain sensor
    - Soil moisture
    - Vibration
    - Tilt

- **🔀 Mode Switch**  
  - GUI buttons for toggling between **Data Collection** and **Standby** modes.

- **📁 CSV Logging**  
  - Auto-logs data to CSV for offline analysis and validation.

---

### `3. ensemble_hybrid.py` – **Anomaly Detection GUI with Ensemble ML**

An advanced GUI client that builds upon `gui.py` and integrates **real-time anomaly detection** using a hybrid ensemble approach.

- **📡 Real-Time Sensor Monitoring**  
  - Same live plots and data fetching as `gui.py`.

- **🤖 Machine Learning Integration**  
  - **Isolation Forest (Offline)**: Pre-trained model loaded from `.joblib`.
  - **River HalfSpaceTrees (Online)**: Online-learning model adapts on-the-fly.

- **⚠️ Ensemble Warning Level**  
  - Fusion of both models to compute a **real-time anomaly score**.

- **🧼 Preprocessing**  
  - Uses pre-trained `StandardScaler` and `LabelEncoder` for:
    - Feature normalization
    - MAC address encoding

---

## 🔗 System Workflow Overview

       +-------------------+
       |    node.ino       |
       |-------------------|
       | ESP32 + CH32V003  |
       | Collects Rain,    |
       | Soil, Tilt, Vib.  |
       |-------------------|
       | Exposes TCP +     |
       | ESP-NOW comms     |
       +--------+----------+
                |
       +--------v----------+
       |     gui.py        |
       |-------------------|
       | TCP Client GUI    |
       | Real-time plots   |
       | CSV Logging       |
       | Mode Control Btns |
       +--------+----------+
                |
       +--------v----------+
       | ensemble_hybrid.py |
       |--------------------|
       | Inherits GUI Flow  |
       | Adds ML Inference  |
       | Anomaly Detection  |
       | Warning Levels     |
       +--------------------+


## 🚀 Getting Started

### Prerequisites

- Arduino IDE or PlatformIO for ESP32
- Python 3.x
- Required Python packages:
  ```bash
  pip install pyqt5 pyqtgraph scikit-learn river joblib
