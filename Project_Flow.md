🚧 Landslide Detection System – ESP32-Based Multi-Node Sensor Network
This repository contains the source code for a distributed landslide detection system using ESP32 microcontrollers, with real-time monitoring and hybrid anomaly detection via machine learning.

🧠 System Overview
The project is structured into three major components:

node.ino: Sensor node firmware for ESP32

gui.py: Basic data monitoring GUI

ensemble_hybrid.py: Advanced GUI with real-time anomaly detection

📟 1. node.ino – ESP32 Sensor Node Firmware
This is the embedded C++ firmware for an ESP32 microcontroller, acting as a power-efficient smart sensor node with the following features:

✅ Key Features:
ULP (Ultra-Low Power) Co-processor Integration
Enables deep sleep with minimal power consumption, waking only on critical pin state changes or timer triggers.

ESP-NOW Communication Protocol
Enables robust, peer-to-peer wireless communication between ESP32 nodes with failover/failback logic for redundancy.

PWM Decoding
Parses pulse-width modulated (PWM) signals from a secondary CH32V003 MCU to extract sensor values.

Non-Volatile Storage (NVS)
Retains node state (e.g. standby vs data collection mode) and target node ID across reboots.

TCP Server
Exposes a lightweight TCP server to:

/sensor_data: Return all current sensor values (tilt, vibration, rain, soil) as a JSON array

/set_mode: Allow remote mode switching (Standby / Data Collection)

🖥️ 2. gui.py – Sensor Monitoring GUI
A Python GUI made with PyQt5 for real-time monitoring and control of the ESP32 nodes.

✅ Core Capabilities:
Sensor Data Fetching
Periodically connects to ESP32 Node 1 (192.168.4.1) over TCP to retrieve sensor data.

Real-Time Plotting
Uses pyqtgraph to plot:

Rain Sensor

Soil Moisture Sensor

Tilt

Vibration

Node Mode Switching
UI buttons toggle ESP32 mode via the /set_mode TCP endpoint.

CSV Logging
Sensor data is saved locally for further offline analysis.

🔍 3. ensemble_hybrid.py – Anomaly Detection + Monitoring GUI
An advanced monitoring client that extends gui.py with real-time hybrid anomaly detection using machine learning models.

✅ Key Additions:
Sensor Data Fetching + Plotting
Same as gui.py, with additional UI plots for anomaly detection.

Hybrid Anomaly Detection:

Offline Model: Isolation Forest (loaded from offline_isolation_forest.joblib)

Online Model: River's HalfSpaceTrees (streaming anomaly detection)

Ensemble Warning System
Combines outputs from both models into a unified "warning level" indicator.

Preprocessing Pipeline
Uses pretrained:

StandardScaler for normalization

LabelEncoder to encode MAC addresses

🔗 How Everything Connects
mermaid
Copy
Edit
graph TD
  A[node.ino - ESP32 Sensor Node] --> B[/sensor_data TCP Endpoint]
  B --> C(gui.py - Basic Monitor)
  B --> D(ensemble_hybrid.py - Advanced Monitor)
  A --> E[ESP-NOW Mesh]
node.ino – The data source. It reads from sensors and distributes the data over TCP and ESP-NOW.

gui.py – A lightweight GUI that visualizes sensor data and controls ESP32 mode.

ensemble_hybrid.py – A smarter GUI that integrates ML models for anomaly detection and gives early warnings.

📁 Project Folder Structure
bash
Copy
Edit
.
├── node.ino                  # ESP32 firmware for sensor node
├── gui.py                    # Real-time PyQt5 GUI for monitoring
├── ensemble_hybrid.py        # Advanced GUI with ML-based anomaly detection
├── offline_isolation_forest.joblib
├── standard_scaler.pkl
├── label_encoder.pkl
└── README.md                 # (You are here)
⚙️ Dependencies
For GUI tools (gui.py, ensemble_hybrid.py):

bash
Copy
Edit
pip install pyqt5 pyqtgraph scikit-learn river joblib
For ESP32 code:

PlatformIO / Arduino IDE

Required libraries: WiFi, ESP-NOW, Preferences, WiFiClient, etc.

🧪 Future Enhancements
Add LoRaWAN fallback

Push data to cloud DB (e.g., InfluxDB, Firebase)

Auto-calibration routines

OTA firmware update capability
