import sys
import requests
import json
import csv
import time
from datetime import datetime
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from PyQt5.QtCore import QTimer, Qt
import pyqtgraph as pg
import numpy as np
import joblib
from sklearn.ensemble import IsolationForest
from river import anomaly, preprocessing, compose
from sklearn.preprocessing import LabelEncoder # Explicitly import LabelEncoder for type hinting/clarity

# --- Configuration ---
ESP32_IP = "192.168.4.1"  # IP address of your ESP32 Active Receiver (Node 1)
SENSOR_DATA_ENDPOINT = f"http://{ESP32_IP}/sensor_data"
SET_MODE_ENDPOINT = f"http://{ESP32_IP}/set_mode"
FETCH_INTERVAL_MS = 500  # How often to fetch data from ESP32 (in milliseconds)
FORWARD_FILL_TIMEOUT_SECONDS = 60  # 1 minute timeout for forward filling in Standby Mode
CSV_FILENAME_PREFIX = "anomaly_log_"
MAX_NODES = 3 # Define the maximum number of nodes for color generation

# File paths for pre-trained models and encoders
ISOLATION_FOREST_MODEL_PATH = 'offline_isolation_forest.joblib'
SCALER_PATH = 'offline_scaler.joblib'
MAC_ENCODER_PATH = 'mac_encoder.joblib'

# --- PyQtGraph Global Configuration ---
pg.setConfigOption('background', 'w') # White background
pg.setConfigOption('foreground', 'k') # Black foreground (text, axes)

class EnsembleAnomalyApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Ensemble Hybrid Anomaly Detection System")
        self.setGeometry(100, 100, 1400, 900) # Increased window size for more plots

        self.current_mode = "UNKNOWN" # Will be updated upon first fetch
        self.csv_file = None
        self.csv_writer = None
        self.csv_filename = None

        # Stores the last received data for each node, used for forward filling
        # Format: {node_id: {'timestamp': python_timestamp, 'rain': val, 'soil': val, 'vibration': val, 'tilt': val, 'mac': mac_address}}
        self.last_known_data = {}

        # Data for plotting: {sensor_type: {node_id: {'x': [], 'y': []}}}
        self.plot_data = {
            'rain': {},
            'soil': {},
            'vibration': {},
            'tilt': {},
            'iso_score': {}, # For Isolation Forest scores
            'river_score': {}, # For River HalfSpaceTree scores
            'warning_level': {} # For ensemble warning level (e.g., numerical representation of High/Mid/Low)
        }
        # Keep track of the plot items for each node
        self.plot_items = {
            'rain': {},
            'soil': {},
            'vibration': {},
            'tilt': {},
            'iso_score': {},
            'river_score': {},
            'warning_level': {}
        }
        # Keep track of alert markings (scatter plots)
        self.alert_scatter_items = {
            'iso_score_alert': {},
            'river_score_alert': {},
            'ensemble_alert': {}
        }

        # --- Load Pre-trained Models ---
        self.if_model: IsolationForest = None
        self.scaler: preprocessing.StandardScaler = None
        self.mac_encoder: LabelEncoder = None
        self.load_models()

        # --- Initialize River HalfSpaceTrees Model (online learning) ---
        # Note: HalfSpaceTrees is an online learning model and does not directly "base" itself
        # on a pre-trained Isolation Forest model in terms of parameters.
        # It learns incrementally from the incoming data stream.
        # The number of features will be inferred from the first data point it learns.
        self.river_model = anomaly.HalfSpaceTrees( # Corrected from HalfSpaceTree to HalfSpaceTrees
            seed=42 # Removed n_features argument
        )
        # For simplicity, we will feed the river model the raw sensor values + encoded MAC.
        # River's HalfSpaceTrees has internal mechanisms to handle feature ranges.

        self.init_ui()
        self.start_data_timer()

    def load_models(self):
        """Loads the pre-trained Isolation Forest model, scaler, and MAC encoder."""
        try:
            self.if_model = joblib.load(ISOLATION_FOREST_MODEL_PATH)
            print(f"Successfully loaded Isolation Forest model from {ISOLATION_FOREST_MODEL_PATH}")
        except FileNotFoundError:
            print(f"Error: Isolation Forest model not found at {ISOLATION_FOREST_MODEL_PATH}. Please ensure it exists.")
            self.if_model = None
        except Exception as e:
            print(f"Error loading Isolation Forest model: {e}")
            self.if_model = None
        
        try:
            self.scaler = joblib.load(SCALER_PATH)
            print(f"Successfully loaded Scaler from {SCALER_PATH}")
        except FileNotFoundError:
            print(f"Error: Scaler not found at {SCALER_PATH}. Please ensure it exists.")
            self.scaler = None
        except Exception as e:
            print(f"Error loading Scaler: {e}")
            self.scaler = None

        try:
            self.mac_encoder = joblib.load(MAC_ENCODER_PATH)
            print(f"Successfully loaded MAC Encoder from {MAC_ENCODER_PATH}")
        except FileNotFoundError:
            print(f"Error: MAC Encoder not found at {MAC_ENCODER_PATH}. Please ensure it exists.")
            self.mac_encoder = None
        except Exception as e:
            print(f"Error loading MAC Encoder: {e}")
            self.mac_encoder = None

        if not (self.if_model and self.scaler and self.mac_encoder):
            print("Warning: Not all models/preprocessors loaded. Anomaly detection might be limited.")


    def init_ui(self):
        main_layout = QVBoxLayout()

        # --- Mode Control ---
        mode_layout = QHBoxLayout()
        self.mode_label = QLabel(f"Current Mode: {self.current_mode}")
        mode_layout.addWidget(self.mode_label)

        self.data_collection_btn = QPushButton("Set Data Collection Mode")
        self.data_collection_btn.clicked.connect(lambda: self.set_mode("data_collection"))
        mode_layout.addWidget(self.data_collection_btn)

        self.standby_btn = QPushButton("Set Standby Mode")
        self.standby_btn.clicked.connect(lambda: self.set_mode("standby"))
        mode_layout.addWidget(self.standby_btn)
        
        main_layout.addLayout(mode_layout)

        # --- Plotting Area (Sensor Data) ---
        self.sensor_plot_widget = pg.GraphicsLayoutWidget(parent=self)
        main_layout.addWidget(self.sensor_plot_widget)

        # Create plots for each sensor type
        self.plot_rain = self.sensor_plot_widget.addPlot(row=0, col=0, title="Rain Sensor (ADC)")
        self.plot_rain.addLegend()
        self.plot_rain.setLabel('bottom', 'Time (seconds)')
        self.plot_rain.setLabel('left', 'ADC Value')
        self.plot_rain.setYRange(0, 4095) # Set Y-axis range for Rain sensor

        self.plot_soil = self.sensor_plot_widget.addPlot(row=0, col=1, title="Soil Sensor (ADC)")
        self.plot_soil.addLegend()
        self.plot_soil.setLabel('bottom', 'Time (seconds)')
        self.plot_soil.setLabel('left', 'ADC Value')
        self.plot_soil.setYRange(0, 4095) # Set Y-axis range for Soil sensor

        self.plot_vibration = self.sensor_plot_widget.addPlot(row=1, col=0, title="Vibration (g)")
        self.plot_vibration.addLegend()
        self.plot_vibration.setLabel('bottom', 'Time (seconds)')
        self.plot_vibration.setLabel('left', 'Vibration (g)')
        self.plot_vibration.setYRange(0.0, 10.0) # Set Y-axis range for Vibration

        self.plot_tilt = self.sensor_plot_widget.addPlot(row=1, col=1, title="Tilt (degrees)")
        self.plot_tilt.addLegend()
        self.plot_tilt.setLabel('bottom', 'Time (seconds)')
        self.plot_tilt.setLabel('left', 'Tilt (degrees)')
        self.plot_tilt.setYRange(0.0, 60.0) # Set Y-axis range for Tilt

        # --- Plotting Area (Anomaly Scores and Warning Levels) ---
        self.anomaly_plot_widget = pg.GraphicsLayoutWidget(parent=self)
        main_layout.addWidget(self.anomaly_plot_widget)

        self.plot_iso_score = self.anomaly_plot_widget.addPlot(row=0, col=0, title="Isolation Forest Anomaly Score")
        self.plot_iso_score.addLegend()
        self.plot_iso_score.setLabel('bottom', 'Time (seconds)')
        self.plot_iso_score.setLabel('left', 'Score')
        # Isolation Forest scores typically range from -0.5 to 0.5. Anomalies are typically < 0
        self.plot_iso_score.setYRange(-0.5, 0.5) 

        self.plot_river_score = self.anomaly_plot_widget.addPlot(row=0, col=1, title="River HalfSpaceTree Anomaly Score")
        self.plot_river_score.addLegend()
        self.plot_river_score.setLabel('bottom', 'Time (seconds)')
        self.plot_river_score.setLabel('left', 'Score')
        # HalfSpaceTrees scores are distances, typically > 0. Higher means more anomalous.
        self.plot_river_score.setYRange(0, 100) # Adjust as per typical score distribution

        self.plot_warning_level = self.anomaly_plot_widget.addPlot(row=1, col=0, title="Ensemble Warning Level")
        self.plot_warning_level.addLegend()
        self.plot_warning_level.setLabel('bottom', 'Time (seconds)')
        self.plot_warning_level.setLabel('left', 'Warning Level (0=Low, 1=Mid, 2=High)')
        self.plot_warning_level.setYRange(-0.5, 2.5) # For 0, 1, 2 levels
        # Set custom tick labels for the warning level plot
        self.plot_warning_level.getAxis('left').setTicks([[(0, 'Low'), (1, 'Mid'), (2, 'High')]])


        self.setLayout(main_layout)

    def start_data_timer(self):
        self.timer = QTimer()
        self.timer.setInterval(FETCH_INTERVAL_MS)
        self.timer.timeout.connect(self.fetch_and_update_data)
        self.timer.start()

    def set_mode(self, mode):
        """Sends a TCP request to the ESP32 to change its operating mode."""
        try:
            response = requests.get(f"{SET_MODE_ENDPOINT}?mode={mode}", timeout=5)
            response.raise_for_status() # Raise an exception for HTTP errors (4xx or 5xx)
            data = response.json()
            if data.get("status") == "success":
                self.current_mode = mode.upper() + "_MODE"
                self.mode_label.setText(f"Current Mode: {self.current_mode}")
                print(f"Successfully set ESP32 to {self.current_mode}")
                # Clear existing plot data when mode changes to avoid misleading graphs
                self.clear_plot_data()
                # Re-initialize CSV for new mode
                self.close_csv()
                self.open_csv()
            else:
                print(f"Failed to set mode: {data.get('error', 'Unknown error')}")
        except requests.exceptions.RequestException as e:
            print(f"Error setting mode: {e}")

    def open_csv(self):
        """Opens a new CSV file with a timestamp in its name for anomaly logs."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f"{CSV_FILENAME_PREFIX}{timestamp}.csv"
        try:
            self.csv_file = open(self.csv_filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            # Write header for anomaly log
            header = ["Timestamp", "MAC Address", "Rain", "Soil", "Vibration", "Tilt",
                      "IsoForest_Score", "River_Score", "Warning_Level_Numerical", "Warning_Level_Text"]
            self.csv_writer.writerow(header)
            print(f"Started logging anomaly data to {self.csv_filename}")
        except IOError as e:
            print(f"Error opening CSV file: {e}")
            self.csv_file = None
            self.csv_writer = None

    def close_csv(self):
        """Closes the current CSV file."""
        if self.csv_file:
            self.csv_file.close()
            print(f"Closed CSV file: {self.csv_filename}")
            self.csv_file = None
            self.csv_writer = None
            self.csv_filename = None

    def clear_plot_data(self):
        """Clears all historical plot data and plot items."""
        # Clear sensor plots
        for sensor_type in self.plot_data:
            self.plot_data[sensor_type].clear()
            for node_id in self.plot_items[sensor_type]:
                plot_map = {
                    'rain': self.plot_rain, 'soil': self.plot_soil,
                    'vibration': self.plot_vibration, 'tilt': self.plot_tilt,
                    'iso_score': self.plot_iso_score, 'river_score': self.plot_river_score,
                    'warning_level': self.plot_warning_level
                }
                if sensor_type in plot_map:
                    plot_map[sensor_type].removeItem(self.plot_items[sensor_type][node_id])
            self.plot_items[sensor_type].clear()
        
        # Clear scatter alert items
        for alert_type in self.alert_scatter_items:
            for node_id in self.alert_scatter_items[alert_type]:
                # Scatter plot items are added to specific plots, so we need to remove them from there
                if alert_type == 'iso_score_alert':
                    self.plot_iso_score.removeItem(self.alert_scatter_items['iso_score_alert'][node_id])
                elif alert_type == 'river_score_alert':
                    self.plot_river_score.removeItem(self.alert_scatter_items['river_score_alert'][node_id])
                elif alert_type == 'ensemble_alert':
                    self.plot_warning_level.removeItem(self.alert_scatter_items['ensemble_alert'][node_id])
            self.alert_scatter_items[alert_type].clear()

        self.last_known_data.clear() # Also clear last known data for forward filling


    def get_warning_level(self, iso_score, river_score):
        """
        Applies ensemble logic to determine the warning level.
        This is a placeholder for your Logic Voting / Weighted Average / Thresholding.
        Returns: (numerical_level, text_level)
        """
        # Example Thresholds (These should be tuned based on your data and models)
        # For Isolation Forest: Lower score means more anomalous.
        # For HalfSpaceTrees: Higher score means more anomalous.

        iso_anomaly_threshold = -0.05 # Values below this are considered anomalous by IF
        river_anomaly_threshold = 20   # Values above this are considered anomalous by River

        is_iso_anomaly = iso_score is not None and iso_score < iso_anomaly_threshold
        is_river_anomaly = river_score is not None and river_score > river_anomaly_threshold

        if is_iso_anomaly and is_river_anomaly:
            return (2, "High") # Both models agree
        elif is_iso_anomaly or is_river_anomaly:
            return (1, "Mid") # One model indicates anomaly
        else:
            return (0, "Low") # Neither indicates anomaly


    def fetch_and_update_data(self):
        """Fetches data from ESP32, logs to CSV, and updates plots."""
        current_python_time = time.time() # Current time in seconds since epoch

        if not self.csv_file: # Ensure CSV is open if not already
            self.open_csv()

        try:
            response = requests.get(SENSOR_DATA_ENDPOINT, timeout=FETCH_INTERVAL_MS / 1000.0 + 1) # Add a buffer to timeout
            response.raise_for_status()
            received_data = response.json()

            # Convert received_data from list of dicts to a dict keyed by nodeId for easier lookup
            received_data_map = {item['nodeId']: item for item in received_data}

            # Update last_known_data with newly received data
            for node_id, data in received_data_map.items():
                self.last_known_data[node_id] = {
                    'timestamp': current_python_time, # Use Python's time for consistency
                    'rain': data.get('rain'),
                    'soil': data.get('soil'),
                    'vibration': data.get('vibration'),
                    'tilt': data.get('tilt'),
                    'mac': data.get('mac') # Store MAC address
                }

            # Process data for all known nodes (including those not currently sending)
            all_node_ids = set(self.last_known_data.keys()).union(set(received_data_map.keys()))
            
            for node_id in sorted(list(all_node_ids)): # Sort for consistent plotting order
                data_to_process = {
                    'nodeId': node_id,
                    'rain': None,
                    'soil': None,
                    'vibration': None,
                    'tilt': None,
                    'mac': None,
                    'iso_score': None,
                    'river_score': None,
                    'warning_level_numerical': 0, # Default to Low
                    'warning_level_text': "Low"
                }

                # --- Handle data acquisition (direct or forward-filled) ---
                if node_id in received_data_map:
                    # Use directly received data
                    data_to_process['rain'] = received_data_map[node_id]['rain']
                    data_to_process['soil'] = received_data_map[node_id]['soil']
                    data_to_process['vibration'] = received_data_map[node_id]['vibration']
                    data_to_process['tilt'] = received_data_map[node_id]['tilt']
                    data_to_process['mac'] = received_data_map[node_id]['mac']
                elif self.current_mode == "STANDBY_MODE" and node_id in self.last_known_data:
                    # Apply forward filling for standby mode if data is within timeout
                    last_update_time = self.last_known_data[node_id].get('timestamp')
                    if last_update_time is not None and \
                       (current_python_time - last_update_time) <= FORWARD_FILL_TIMEOUT_SECONDS:
                        # Use last known values
                        data_to_process['rain'] = self.last_known_data[node_id]['rain']
                        data_to_process['soil'] = self.last_known_data[node_id]['soil']
                        data_to_process['vibration'] = self.last_known_data[node_id]['vibration']
                        data_to_process['tilt'] = self.last_known_data[node_id]['tilt']
                        data_to_process['mac'] = self.last_known_data[node_id]['mac'] # Forward fill MAC too
                    else:
                        # Data is too old for forward filling, set to None (NaN in plot)
                        # MAC address might still be known from last_known_data, even if sensor data is stale
                        data_to_process['mac'] = self.last_known_data[node_id].get('mac')


                # --- Anomaly Detection ---
                # Only proceed if all sensor data (rain, soil, vibration, tilt) is available and models are loaded
                if all(data_to_process[k] is not None for k in ['rain', 'soil', 'vibration', 'tilt']) and \
                   self.if_model and self.scaler and self.mac_encoder:
                    
                    # Create input feature array for models
                    # Note: Vibration and Tilt are already floats from ESP32 JSON, but ensure consistency
                    features_list = [
                        float(data_to_process['rain']),
                        float(data_to_process['soil']),
                        float(data_to_process['vibration']),
                        float(data_to_process['tilt'])
                    ]
                    
                    # Encode MAC Address for the models
                    encoded_mac = None
                    if data_to_process['mac'] is not None:
                        try:
                            # Handle new MACs not seen during training for LabelEncoder
                            if data_to_process['mac'] not in self.mac_encoder.classes_:
                                # Assign a new, unique integer for this new MAC.
                                # This is a simple approach; for robust production, consider
                                # retraining the encoder or using a hashing trick.
                                # For now, we'll append to classes and transform.
                                self.mac_encoder.classes_ = np.append(self.mac_encoder.classes_, data_to_process['mac'])
                                encoded_mac = self.mac_encoder.transform([data_to_process['mac']])[0]
                                print(f"Info: Added new MAC '{data_to_process['mac']}' to encoder classes for Node {node_id}. New encoded value: {encoded_mac}")
                            else:
                                encoded_mac = self.mac_encoder.transform([data_to_process['mac']])[0]
                        except Exception as e:
                            print(f"Error encoding MAC '{data_to_process['mac']}': {e}. Defaulting encoded_mac to 0.")
                            encoded_mac = 0 # Fallback if encoding fails
                    else:
                        # If MAC is None (e.g., during forward fill where MAC wasn't known yet)
                        # or if there's a problem, use a default/placeholder for encoded_mac.
                        # This might affect model accuracy for such points.
                        encoded_mac = 0 # Default if MAC is missing

                    features_list.append(float(encoded_mac)) # Add encoded MAC as a feature

                    input_array = np.array(features_list).reshape(1, -1)

                    # --- Isolation Forest Score ---
                    # Apply the same scaler used during training
                    if self.scaler:
                        scaled_input = self.scaler.transform(input_array)
                        data_to_process['iso_score'] = self.if_model.decision_function(scaled_input)[0]
                    else:
                        print("Warning: Scaler not loaded. Cannot calculate Isolation Forest score.")

                    # --- River HalfSpaceTrees Score (predict and learn) ---
                    # HalfSpaceTrees expects a dictionary input, not scaled numpy array,
                    # and it learns incrementally.
                    # It's better to provide raw features to river and let it do its internal preprocessing.
                    river_input = {
                        'rain': data_to_process['rain'],
                        'soil': data_to_process['soil'],
                        'vibration': data_to_process['vibration'],
                        'tilt': data_to_process['tilt'],
                        'encoded_mac': float(encoded_mac) # Use encoded MAC for river as well
                    }
                    # Make a prediction (score) first, then learn from the observation
                    data_to_process['river_score'] = self.river_model.score_one(river_input)
                    self.river_model.learn_one(river_input)
                else:
                    # If data is incomplete or models not loaded, scores remain None
                    pass # Already initialized to None or default values

                # --- Ensemble Logic (Warning Level) ---
                (data_to_process['warning_level_numerical'],
                 data_to_process['warning_level_text']) = self.get_warning_level(
                                                                data_to_process['iso_score'],
                                                                data_to_process['river_score']
                                                            )

                # --- Log to CSV ---
                if self.csv_writer:
                    row = [
                        datetime.fromtimestamp(current_python_time).strftime("%Y-%m-%d %H:%M:%S.%f"),
                        data_to_process['mac'] if data_to_process['mac'] is not None else f"Node {data_to_process['nodeId']}",
                        data_to_process['rain'] if data_to_process['rain'] is not None else '',
                        data_to_process['soil'] if data_to_process['soil'] is not None else '',
                        data_to_process['vibration'] if data_to_process['vibration'] is not None else '',
                        data_to_process['tilt'] if data_to_process['tilt'] is not None else '',
                        data_to_process['iso_score'] if data_to_process['iso_score'] is not None else '',
                        data_to_process['river_score'] if data_to_process['river_score'] is not None else '',
                        data_to_process['warning_level_numerical'],
                        data_to_process['warning_level_text']
                    ]
                    self.csv_writer.writerow(row)
                    self.csv_file.flush() # Ensure data is written to disk immediately

                # --- Update Plot Data & Plot Items ---
                for sensor_type, plot_obj in {
                    'rain': self.plot_rain, 'soil': self.plot_soil,
                    'vibration': self.plot_vibration, 'tilt': self.plot_tilt,
                    'iso_score': self.plot_iso_score, 'river_score': self.plot_river_score,
                    'warning_level': self.plot_warning_level
                }.items():
                    plot_label = data_to_process['mac'] if data_to_process['mac'] is not None else f"Node {node_id}"

                    if node_id not in self.plot_items[sensor_type]:
                        self.plot_data[sensor_type][node_id] = {'x': [], 'y': []}
                        color = pg.intColor(node_id * 10, hues=MAX_NODES)
                        self.plot_items[sensor_type][node_id] = plot_obj.plot(
                            pen=pg.mkPen(color=color, width=2),
                            name=plot_label
                        )
                    else:
                        current_plot_item = self.plot_items[sensor_type][node_id]
                        if current_plot_item.name() != plot_label:
                            current_plot_item.opts['name'] = plot_label
                            current_plot_item.updateLabel()

                    # Value for plotting (use np.nan if None)
                    plot_value = data_to_process.get(sensor_type)
                    if plot_value is None:
                        plot_value = np.nan
                    # Special handling for warning_level to ensure it's plotted as number
                    if sensor_type == 'warning_level':
                        plot_value = data_to_process['warning_level_numerical']
                    
                    self.plot_data[sensor_type][node_id]['x'].append(current_python_time)
                    self.plot_data[sensor_type][node_id]['y'].append(plot_value)
                    
                    max_points = 500 # Keep only the last N points
                    if len(self.plot_data[sensor_type][node_id]['x']) > max_points:
                        self.plot_data[sensor_type][node_id]['x'] = self.plot_data[sensor_type][node_id]['x'][-500:] # Fixed slice to 500
                        self.plot_data[sensor_type][node_id]['y'] = self.plot_data[sensor_type][node_id]['y'][-500:] # Fixed slice to 500

                    self.plot_items[sensor_type][node_id].setData(
                        np.array(self.plot_data[sensor_type][node_id]['x']) - self.plot_data[sensor_type][node_id]['x'][0],
                        np.array(self.plot_data[sensor_type][node_id]['y'])
                    )
                
                # --- Add Alert Markings ---
                # Scatter plot for anomalies on score plots
                relative_time = (current_python_time - self.plot_data['rain'][node_id]['x'][0]) if self.plot_data['rain'][node_id]['x'] else 0
                
                # For Isolation Forest (score < threshold is anomalous)
                if data_to_process['iso_score'] is not None and data_to_process['iso_score'] < -0.05: # Example threshold
                    if node_id not in self.alert_scatter_items['iso_score_alert']:
                        self.alert_scatter_items['iso_score_alert'][node_id] = pg.ScatterPlotItem(
                            symbol='o', size=10, brush=pg.mkBrush(255, 0, 0, 150), name=f'IF Anomaly {plot_label}'
                        )
                        self.plot_iso_score.addItem(self.alert_scatter_items['iso_score_alert'][node_id])
                    self.alert_scatter_items['iso_score_alert'][node_id].addPoints(
                        [relative_time], [data_to_process['iso_score']]
                    )
                
                # For River HalfSpaceTrees (score > threshold is anomalous)
                if data_to_process['river_score'] is not None and data_to_process['river_score'] > 20: # Example threshold
                    if node_id not in self.alert_scatter_items['river_score_alert']:
                        self.alert_scatter_items['river_score_alert'][node_id] = pg.ScatterPlotItem(
                            symbol='s', size=10, brush=pg.mkBrush(0, 0, 255, 150), name=f'River Anomaly {plot_label}'
                        )
                        self.plot_river_score.addItem(self.alert_scatter_items['river_score_alert'][node_id])
                    self.alert_scatter_items['river_score_alert'][node_id].addPoints(
                        [relative_time], [data_to_process['river_score']]
                    )

                # For Ensemble Warning Level
                if data_to_process['warning_level_numerical'] > 0: # If Mid or High
                    symbol = 't' if data_to_process['warning_level_numerical'] == 1 else 'd' # Triangle for Mid, Diamond for High
                    brush_color = pg.mkBrush(255, 165, 0, 150) if data_to_process['warning_level_numerical'] == 1 else pg.mkBrush(255, 0, 0, 200) # Orange for Mid, Red for High
                    
                    if node_id not in self.alert_scatter_items['ensemble_alert']:
                        self.alert_scatter_items['ensemble_alert'][node_id] = pg.ScatterPlotItem(
                            symbol=symbol, size=12, brush=brush_color, name=f'Ensemble Alert {plot_label}'
                        )
                        self.plot_warning_level.addItem(self.alert_scatter_items['ensemble_alert'][node_id])
                    else:
                        # Update existing scatter plot item attributes if necessary (e.g., symbol/color for different levels)
                        self.alert_scatter_items['ensemble_alert'][node_id].setSymbol(symbol)
                        self.alert_scatter_items['ensemble_alert'][node_id].setBrush(brush_color)

                    self.alert_scatter_items['ensemble_alert'][node_id].addPoints(
                        [relative_time], [data_to_process['warning_level_numerical']]
                    )
            
            # Update the mode label if it's currently unknown (first fetch)
            if self.current_mode == "UNKNOWN" and received_data:
                self.mode_label.setText(f"Current Mode: {self.current_mode} (Data Received)")


        except requests.exceptions.RequestException as e:
            print(f"Error fetching data: {e}")
            # If data fetch fails, apply forward filling for all nodes if in standby mode
            if self.current_mode == "STANDBY_MODE":
                for node_id in sorted(list(self.last_known_data.keys())):
                    data_to_process = {
                        'nodeId': node_id,
                        'rain': None, 'soil': None, 'vibration': None, 'tilt': None, 'mac': None,
                        'iso_score': None, 'river_score': None,
                        'warning_level_numerical': 0, 'warning_level_text': "Low"
                    }
                    last_update_time = self.last_known_data[node_id].get('timestamp')
                    if last_update_time is not None and \
                       (current_python_time - last_update_time) <= FORWARD_FILL_TIMEOUT_SECONDS:
                        data_to_process['rain'] = self.last_known_data[node_id]['rain']
                        data_to_process['soil'] = self.last_known_data[node_id]['soil']
                        data_to_process['vibration'] = self.last_known_data[node_id]['vibration']
                        data_to_process['tilt'] = self.last_known_data[node_id]['tilt']
                        data_to_process['mac'] = self.last_known_data[node_id]['mac']
                    else:
                        data_to_process['mac'] = self.last_known_data[node_id].get('mac')
                    
                    # Log to CSV (even if sensor data is None, log placeholder scores/levels)
                    if self.csv_writer:
                        row = [
                            datetime.fromtimestamp(current_python_time).strftime("%Y-%m-%d %H:%M:%S.%f"),
                            data_to_process['mac'] if data_to_process['mac'] is not None else f"Node {data_to_process['nodeId']}",
                            data_to_process['rain'] if data_to_process['rain'] is not None else '',
                            data_to_process['soil'] if data_to_process['soil'] is not None else '',
                            data_to_process['vibration'] if data_to_process['vibration'] is not None else '',
                            data_to_process['tilt'] if data_to_process['tilt'] is not None else '',
                            data_to_process['iso_score'] if data_to_process['iso_score'] is not None else '',
                            data_to_process['river_score'] if data_to_process['river_score'] is not None else '',
                            data_to_process['warning_level_numerical'],
                            data_to_process['warning_level_text']
                        ]
                        self.csv_writer.writerow(row)
                        self.csv_file.flush()

                    # Update plot data (even if sensor data is None, plots will show NaNs or last known)
                    for sensor_type, plot_obj in {
                        'rain': self.plot_rain, 'soil': self.plot_soil,
                        'vibration': self.plot_vibration, 'tilt': self.plot_tilt,
                        'iso_score': self.plot_iso_score, 'river_score': self.plot_river_score,
                        'warning_level': self.plot_warning_level
                    }.items():
                        plot_label = data_to_process['mac'] if data_to_process['mac'] is not None else f"Node {node_id}"

                        if node_id not in self.plot_items[sensor_type]:
                            self.plot_data[sensor_type][node_id] = {'x': [], 'y': []}
                            color = pg.intColor(node_id * 10, hues=MAX_NODES)
                            self.plot_items[sensor_type][node_id] = plot_obj.plot(
                                pen=pg.mkPen(color=color, width=2),
                                name=plot_label
                            )
                        else:
                            current_plot_item = self.plot_items[sensor_type][node_id]
                            if current_plot_item.name() != plot_label:
                                current_plot_item.opts['name'] = plot_label
                                current_plot_item.updateLabel()
                        
                        plot_value = data_to_process.get(sensor_type)
                        if plot_value is None:
                            plot_value = np.nan
                        if sensor_type == 'warning_level':
                            plot_value = data_to_process['warning_level_numerical']
                        
                        self.plot_data[sensor_type][node_id]['x'].append(current_python_time)
                        self.plot_data[sensor_type][node_id]['y'].append(plot_value)
                        
                        max_points = 500
                        if len(self.plot_data[sensor_type][node_id]['x']) > max_points:
                            self.plot_data[sensor_type][node_id]['x'] = self.plot_data[sensor_type][node_id]['x'][-500:] # Fixed slice to 500
                            self.plot_data[sensor_type][node_id]['y'] = self.plot_data[sensor_type][node_id]['y'][-500:] # Fixed slice to 500

                        self.plot_items[sensor_type][node_id].setData(
                            np.array(self.plot_data[sensor_type][node_id]['x']) - self.plot_data[sensor_type][node_id]['x'][0],
                            np.array(self.plot_data[sensor_type][node_id]['y'])
                        )
            else:
                # If not in STANDBY_MODE, and fetch fails, just print error and don't forward fill
                pass


    def closeEvent(self, event):
        """Ensures the CSV file is closed when the application exits."""
        self.close_csv()
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = EnsembleAnomalyApp()
    window.show()
    sys.exit(app.exec_())
