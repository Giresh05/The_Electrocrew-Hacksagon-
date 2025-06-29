# 🌋 Landslide Detection System – ElectroCrew

**A resilient edge-AI system for early landslide detection using ESP32 and CH32 microcontrollers.**  
Now with TCP-to-PC communication and real-time data plotting — plus an ensemble of Isolation Forest + River's HalfSpaceTrees for hybrid anomaly detection.

---

## ⚙️ System Overview

This multi-node, low-power sensor mesh detects landslide-related anomalies by fusing data from environmental and motion sensors. It communicates with a PC using a custom TCP server and visualizes sensor trends in real time.

---

## 🧠 Machine Learning Pipeline

| Phase        | Model              | Purpose                                |
|--------------|--------------------|----------------------------------------|
| Offline      | Isolation Forest   | Trained on collected data for static profiling. |
| Online       | HalfSpaceTrees     | Live adaptation using `river`.         |
| Final Output | Hybrid Ensemble    | Combines both for robust detection.    |

### ✅ Ensemble Hybrid Validation  
- Outputs from both models are fused.  
- Ensures lower false positive/negative rate than using either alone.  
- Output: `"normal"`, `"suspicious"`, or `"likely"` anomaly.

---

## 🔋 Hardware

| Component              | Role                        |
|------------------------|-----------------------------|
| ESP32-WROOM-32U        | Main node + ULP + TCP       |
| CH32V003               | Secondary analog sensor node|
| MPU9250                | Accel + Gyro                |
| Rain Sensor            | Weather sensing (AO)        |
| Soil Moisture Sensor   | Ground moisture (AO)        |
| SD Card Module         | Data buffer + blackbox      |
| TP4056 + Li-ion        | Battery system              |
| AMS1117               | 3.3V LDO regulator           |

---

## 📶 TCP Communication with PC

The ESP32 runs a **TCP server** on port `80`. The Python script connects to it to:

- Request live sensor data from `/sensor_data?nodeId=x`
- Control the system via `/set_mode` endpoint

### API Endpoints

| Endpoint         | Method | Description                            |
|------------------|--------|----------------------------------------|
| `/sensor_data`   | GET    | Returns latest sensor payload          |
| `/set_mode`      | POST   | Set mode: `Standby` / `Data Collection`|

---

## 📊 Real-Time Plotting (Python)

Python script polls sensor data over TCP and plots:

- Accelerometer (X, Y, Z)
- Rain/Soil analog values
- Live anomaly score
- Decision result (annotated)

