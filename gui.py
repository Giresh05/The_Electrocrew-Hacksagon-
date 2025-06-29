import sys
import requests
import json
import csv
import time
from datetime import datetime
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QComboBox
from PyQt5.QtCore import QTimer, Qt
import pyqtgraph as pg
import numpy as np

# --- Configuration ---
ESP32_IP = "192.168.4.1"  # IP address of your ESP32 Active Receiver (Node 1)
SENSOR_DATA_ENDPOINT = f"http://{ESP32_IP}/sensor_data"
SET_MODE_ENDPOINT = f"http://{ESP32_IP}/set_mode"
FETCH_INTERVAL_MS = 500  # How often to fetch data from ESP32 (in milliseconds)
FORWARD_FILL_TIMEOUT_SECONDS = 60  # 1 minute timeout for forward filling in Standby Mode
CSV_FILENAME_PREFIX = "sensor_data_"
MAX_NODES = 3 # Define the maximum number of nodes for color generation

# --- PyQtGraph Global Configuration ---
pg.setConfigOption('background', 'w') # White background
pg.setConfigOption('foreground', 'k') # Black foreground (text, axes)

class SensorDataApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32 Sensor Monitor")
        self.setGeometry(100, 100, 1200, 800) # Increased window size

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
            'tilt': {}
        }
        # Keep track of the plot items for each node
        self.plot_items = {
            'rain': {},
            'soil': {},
            'vibration': {},
            'tilt': {}
        }

        self.init_ui()
        self.start_data_timer()

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

        # --- Plotting Area ---
        self.plot_widget = pg.GraphicsLayoutWidget(parent=self)
        main_layout.addWidget(self.plot_widget)

        # Create plots for each sensor type
        self.plot_rain = self.plot_widget.addPlot(row=0, col=0, title="Rain Sensor (ADC)")
        self.plot_rain.addLegend()
        self.plot_rain.setLabel('bottom', 'Time (seconds)')
        self.plot_rain.setLabel('left', 'ADC Value')
        self.plot_rain.setYRange(0, 4095) # Set Y-axis range for Rain sensor

        self.plot_soil = self.plot_widget.addPlot(row=0, col=1, title="Soil Sensor (ADC)")
        self.plot_soil.addLegend()
        self.plot_soil.setLabel('bottom', 'Time (seconds)')
        self.plot_soil.setLabel('left', 'ADC Value')
        self.plot_soil.setYRange(0, 4095) # Set Y-axis range for Soil sensor

        self.plot_vibration = self.plot_widget.addPlot(row=1, col=0, title="Vibration (g)")
        self.plot_vibration.addLegend()
        self.plot_vibration.setLabel('bottom', 'Time (seconds)')
        self.plot_vibration.setLabel('left', 'Vibration (g)')
        self.plot_vibration.setYRange(0.0, 10.0) # Set Y-axis range for Vibration

        self.plot_tilt = self.plot_widget.addPlot(row=1, col=1, title="Tilt (degrees)")
        self.plot_tilt.addLegend()
        self.plot_tilt.setLabel('bottom', 'Time (seconds)')
        self.plot_tilt.setLabel('left', 'Tilt (degrees)')
        self.plot_tilt.setYRange(0.0, 60.0) # Set Y-axis range for Tilt

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
        """Opens a new CSV file with a timestamp in its name."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f"{CSV_FILENAME_PREFIX}{timestamp}.csv"
        try:
            self.csv_file = open(self.csv_filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            # Write header, now with "MAC Address" instead of "NodeID"
            header = ["Timestamp", "MAC Address", "Rain", "Soil", "Vibration", "Tilt"]
            self.csv_writer.writerow(header)
            print(f"Started logging data to {self.csv_filename}")
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
        for sensor_type in self.plot_data:
            self.plot_data[sensor_type].clear()
            for node_id in self.plot_items[sensor_type]:
                # Remove plot items from their respective plots
                if sensor_type == 'rain': self.plot_rain.removeItem(self.plot_items[sensor_type][node_id])
                elif sensor_type == 'soil': self.plot_soil.removeItem(self.plot_items[sensor_type][node_id])
                elif sensor_type == 'vibration': self.plot_vibration.removeItem(self.plot_items[sensor_type][node_id])
                elif sensor_type == 'tilt': self.plot_tilt.removeItem(self.plot_items[sensor_type][node_id])
            self.plot_items[sensor_type].clear()
        self.last_known_data.clear() # Also clear last known data for forward filling

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
            # This loop handles both direct received data and forward filling
            all_node_ids = set(self.last_known_data.keys()).union(set(received_data_map.keys()))
            
            for node_id in sorted(list(all_node_ids)): # Sort for consistent plotting order
                data_to_log = {
                    'nodeId': node_id, # Keep nodeId for internal logic if needed, but use mac for display
                    'rain': None,
                    'soil': None,
                    'vibration': None,
                    'tilt': None,
                    'mac': None # Initialize mac
                }

                if node_id in received_data_map:
                    # Use directly received data
                    data_to_log['rain'] = received_data_map[node_id]['rain']
                    data_to_log['soil'] = received_data_map[node_id]['soil']
                    data_to_log['vibration'] = received_data_map[node_id]['vibration']
                    data_to_log['tilt'] = received_data_map[node_id]['tilt']
                    data_to_log['mac'] = received_data_map[node_id]['mac']
                elif self.current_mode == "STANDBY_MODE" and node_id in self.last_known_data:
                    # Apply forward filling for standby mode if data is within timeout
                    last_update_time = self.last_known_data[node_id].get('timestamp')
                    if last_update_time is not None and \
                       (current_python_time - last_update_time) <= FORWARD_FILL_TIMEOUT_SECONDS:
                        # Use last known values
                        data_to_log['rain'] = self.last_known_data[node_id]['rain']
                        data_to_log['soil'] = self.last_known_data[node_id]['soil']
                        data_to_log['vibration'] = self.last_known_data[node_id]['vibration']
                        data_to_log['tilt'] = self.last_known_data[node_id]['tilt']
                        data_to_log['mac'] = self.last_known_data[node_id]['mac'] # Forward fill MAC too
                    else:
                        # Data is too old for forward filling, set to None (NaN in plot)
                        # MAC address might still be known from last_known_data, even if sensor data is stale
                        data_to_log['mac'] = self.last_known_data[node_id].get('mac')


                # Log to CSV
                if self.csv_writer:
                    row = [
                        datetime.fromtimestamp(current_python_time).strftime("%Y-%m-%d %H:%M:%S.%f"),
                        data_to_log['mac'] if data_to_log['mac'] is not None else f"Node {data_to_log['nodeId']}", # Use MAC for CSV
                        data_to_log['rain'] if data_to_log['rain'] is not None else '',
                        data_to_log['soil'] if data_to_log['soil'] is not None else '',
                        data_to_log['vibration'] if data_to_log['vibration'] is not None else '',
                        data_to_log['tilt'] if data_to_log['tilt'] is not None else ''
                    ]
                    self.csv_writer.writerow(row)
                    self.csv_file.flush() # Ensure data is written to disk immediately

                # Update plot data
                for sensor_type, plot_obj in {
                    'rain': self.plot_rain,
                    'soil': self.plot_soil,
                    'vibration': self.plot_vibration,
                    'tilt': self.plot_tilt
                }.items():
                    # Determine the label for the plot legend
                    plot_label = data_to_log['mac'] if data_to_log['mac'] is not None else f"Node {node_id}"

                    # Ensure the plot item and its corresponding data storage exist for this node and sensor type
                    if node_id not in self.plot_items[sensor_type]:
                        self.plot_data[sensor_type][node_id] = {'x': [], 'y': []} # Initialize data storage
                        color = pg.intColor(node_id * 10, hues=MAX_NODES) # Use node_id for distinct colors
                        self.plot_items[sensor_type][node_id] = plot_obj.plot(
                            pen=pg.mkPen(color=color, width=2),
                            name=plot_label # Use MAC for plot legend
                        )
                    else:
                        # Update the legend name if MAC was not available initially but now is
                        current_plot_item = self.plot_items[sensor_type][node_id]
                        if current_plot_item.name() != plot_label:
                            current_plot_item.opts['name'] = plot_label
                            current_plot_item.updateLabel() # Force legend update

                    # Add current time (relative to start) and sensor value
                    # Use np.nan for missing data for pyqtgraph to break lines
                    sensor_value = data_to_log[sensor_type] if data_to_log[sensor_type] is not None else np.nan
                    
                    self.plot_data[sensor_type][node_id]['x'].append(current_python_time)
                    self.plot_data[sensor_type][node_id]['y'].append(sensor_value)
                    
                    # Keep only the last N points to prevent memory issues and keep plots responsive
                    max_points = 500 # Adjust as needed
                    if len(self.plot_data[sensor_type][node_id]['x']) > max_points:
                        self.plot_data[sensor_type][node_id]['x'] = self.plot_data[sensor_type][node_id]['x'][-max_points:]
                        self.plot_data[sensor_type][node_id]['y'] = self.plot_data[sensor_type][node_id]['y'][-max_points:]

                    # Update plot item data
                    self.plot_items[sensor_type][node_id].setData(
                        np.array(self.plot_data[sensor_type][node_id]['x']) - self.plot_data[sensor_type][node_id]['x'][0],
                        np.array(self.plot_data[sensor_type][node_id]['y'])
                    )
            
            # Update the mode label if it's currently unknown (first fetch)
            if self.current_mode == "UNKNOWN" and received_data:
                # The ESP32 code doesn't send its current mode in the /sensor_data response directly.
                # We'll rely on the user setting it via buttons or assume an initial state.
                # For now, if data is received, we know it's active.
                self.mode_label.setText(f"Current Mode: {self.current_mode} (Data Received)")


        except requests.exceptions.RequestException as e:
            print(f"Error fetching data: {e}")
            # If data fetch fails, apply forward filling for all nodes if in standby mode
            if self.current_mode == "STANDBY_MODE":
                for node_id in sorted(list(self.last_known_data.keys())):
                    data_to_log = {
                        'nodeId': node_id,
                        'rain': None,
                        'soil': None,
                        'vibration': None,
                        'tilt': None,
                        'mac': None # Initialize mac
                    }
                    last_update_time = self.last_known_data[node_id].get('timestamp')
                    if last_update_time is not None and \
                       (current_python_time - last_update_time) <= FORWARD_FILL_TIMEOUT_SECONDS:
                        data_to_log['rain'] = self.last_known_data[node_id]['rain']
                        data_to_log['soil'] = self.last_known_data[node_id]['soil']
                        data_to_log['vibration'] = self.last_known_data[node_id]['vibration']
                        data_to_log['tilt'] = self.last_known_data[node_id]['tilt']
                        data_to_log['mac'] = self.last_known_data[node_id]['mac']
                    else:
                        # MAC address might still be known from last_known_data, even if sensor data is stale
                        data_to_log['mac'] = self.last_known_data[node_id].get('mac')
                    
                    # Log to CSV
                    if self.csv_writer:
                        row = [
                            datetime.fromtimestamp(current_python_time).strftime("%Y-%m-%d %H:%M:%S.%f"),
                            data_to_log['mac'] if data_to_log['mac'] is not None else f"Node {data_to_log['nodeId']}", # Use MAC for CSV
                            data_to_log['rain'] if data_to_log['rain'] is not None else '',
                            data_to_log['soil'] if data_to_log['soil'] is not None else '',
                            data_to_log['vibration'] if data_to_log['vibration'] is not None else '',
                            data_to_log['tilt'] if data_to_log['tilt'] is not None else ''
                        ]
                        self.csv_writer.writerow(row)
                        self.csv_file.flush()

                    # Update plot data with potentially forward-filled values
                    for sensor_type, plot_obj in {
                        'rain': self.plot_rain,
                        'soil': self.plot_soil,
                        'vibration': self.plot_vibration,
                        'tilt': self.plot_tilt
                    }.items():
                        # Determine the label for the plot legend
                        plot_label = data_to_log['mac'] if data_to_log['mac'] is not None else f"Node {node_id}"

                        if node_id not in self.plot_items[sensor_type]:
                            # This case should ideally not happen if last_known_data is populated
                            # but added for robustness.
                            self.plot_data[sensor_type][node_id] = {'x': [], 'y': []}
                            color = pg.intColor(node_id * 10, hues=MAX_NODES)
                            self.plot_items[sensor_type][node_id] = plot_obj.plot(
                                pen=pg.mkPen(color=color, width=2),
                                name=plot_label # Use MAC for plot legend
                            )
                        else:
                            # Update the legend name if MAC was not available initially but now is
                            current_plot_item = self.plot_items[sensor_type][node_id]
                            if current_plot_item.name() != plot_label:
                                current_plot_item.opts['name'] = plot_label
                                current_plot_item.updateLabel() # Force legend update
                        
                        sensor_value = data_to_log[sensor_type] if data_to_log[sensor_type] is not None else np.nan
                        self.plot_data[sensor_type][node_id]['x'].append(current_python_time)
                        self.plot_data[sensor_type][node_id]['y'].append(sensor_value)
                        
                        max_points = 500
                        if len(self.plot_data[sensor_type][node_id]['x']) > max_points:
                            self.plot_data[sensor_type][node_id]['x'] = self.plot_data[sensor_type][node_id]['x'][-max_points:]
                            self.plot_data[sensor_type][node_id]['y'] = self.plot_data[sensor_type][node_id]['y'][-max_points:]

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
    window = SensorDataApp()
    window.show()
    sys.exit(app.exec_())
