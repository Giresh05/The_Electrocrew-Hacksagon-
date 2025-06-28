// --- ESP32 Sensor Bridge Comprehensive Workflow ---

// This code integrates the Ultra-Low Power (ULP) co-processor for deep sleep wakeups,
// ESP-NOW for robust wireless communication with failover/failback mechanisms,
// and PWM decoding for sensor data from a CH32V003 microcontroller.
// It also adds NVS (Non-Volatile Storage) for persisting operating mode and target node ID,
// and implements a "Data Collection Mode" for continuous high-rate data acquisition.
//
// Author: Gemini
// Date: June 23, 2025 (Modified: June 24, 2025 for NVS and Data Collection Mode, wakeup sync fix, and ACK-based AR announcements)
// Modified: June 24, 2025 for AR Takeback Logic
// Modified: June 24, 2025 for TCP Server functionality
// Modified: June 24, 2025 to fix LoadProhibited error by reordering WiFi and ESP-NOW initialization
// Modified: June 24, 2025 to enable TCP server to update with data from other nodes
// Modified: June 26, 2025 to add PC-based mode switching via TCP endpoint

// --- Includes ---
#include <esp_now.h>
#include <WiFi.h>
#include <nvs_flash.h> // For NVS (Non-Volatile Storage)
#include <nvs.h>       // For NVS operations
#include "hulp_arduino.h" // Assuming this library is available for ULP interactions
#include <WebServer.h> // New include for WebServer (TCP functionality)

// --- Global Constants and Defines for ESP-NOW and System ---
#define WIFI_CHANNEL 1              // WiFi channel for ESP-NOW communication
#define PROBE_INTERVAL 7000         // Interval (ms) for receivers to probe higher-priority nodes for AR Takeback
#define FAILOVER_THRESHOLD 3        // Number of consecutive send failures to trigger failover
#define RETRY_DELAY_MS 500          // Delay (ms) before retrying a failed ESP-NOW send
#define LOCAL_SENSOR_CHECK_INTERVAL 1000 // Interval (ms) for receiver to check its local sensors
#define SENDER_MODE_SYNC_WAIT_MS 1000 // Time (ms) sender waits after sending/waking to sync mode with AR

// --- NVS Specific Defines ---
#define NVS_NAMESPACE "sensor_bridge"    // Namespace for NVS keys
#define NVS_KEY_MODE_FLAG "mode_flag"    // NVS key for operating mode
#define NVS_KEY_TARGET_NODE_ID "target_node_id" // NVS key for target node ID

// --- ULP Specific Defines (from fin_ulp.txt) ---
#define PIN_ADC_RAIN    GPIO_NUM_36  // ADC1_CH0, pad_idx = 0 (Rain sensor analog input)
#define PIN_ADC_SOIL    GPIO_NUM_39  // ADC1_CH3, pad_idx = 3 (Soil sensor analog input)
#define PIN_ADC_READY   GPIO_NUM_34  // ADC1_CH6, pad_idx = 6 (CH32 "data ready" analog input)
#define ULP_WAKEUP_INTERVAL_MS 1000 // ULP program execution interval in milliseconds
#define SENSOR_WINDOW_MARGIN 100    // Margin for delta-threshold wakeups (for rain/soil)
#define DATA_READY_THRESHOLD 2000   // Analog threshold for DATA_READY_PIN to trigger ULP wakeup

// --- Dynamic Mode Switching (Push Button) ---
#define PUSH_BUTTON_PIN GPIO_NUM_0 // Example GPIO for push button on Alpha Node (e.g., ESP32 onboard button)

// --- TCP Server Specific Defines ---
const char* softAP_ssid = "ESP32_SensorBridge";  // WiFi AP SSID for the TCP server
const char* softAP_password = "password";      // WiFi AP password for the TCP server

// --- RTC_DATA_ATTR Variables (persist across deep sleep) ---
// These variables are stored in RTC memory and retain their values when the ESP32 enters deep sleep.
RTC_DATA_ATTR struct {
    ulp_var_t rain_lower;      // Lower threshold for rain ADC (ULP)
    ulp_var_t rain_upper;      // Upper threshold for rain ADC (ULP)
    ulp_var_t last_rain_val;   // Last recorded rain ADC value (ULP)
    ulp_var_t soil_lower;      // Lower threshold for soil ADC (ULP)
    ulp_var_t soil_upper;      // Upper threshold for soil ADC (ULP)
    ulp_var_t last_soil_val;   // Last recorded soil ADC value (ULP)
    ulp_var_t last_ready_val;  // Last recorded DATA_READY ADC value (ULP)
    ulp_var_t triggered_count; // Counter for ULP wakeups (ULP)
} ulp_vars;

RTC_DATA_ATTR int sendCounter = 0;          // Counter for messages sent by this node
RTC_DATA_ATTR bool failedLastSend = false;   // Flag indicating if the last ESP-NOW send failed
RTC_DATA_ATTR int lastAnnouncedAR = 0;      // ID of the last known Active Receiver
RTC_DATA_ATTR int consecutiveSendFailures = 0; // Consecutive failures to send to target AR
RTC_DATA_ATTR int lastFailedTargetId = 0;    // ID of the node that last caused a send failure
RTC_DATA_ATTR bool probingForFailover = false; // Flag to indicate if probing for a new AR is in progress
RTC_DATA_ATTR bool ulpJustWoke = false;       // Flag to indicate if woken by ULP
RTC_DATA_ATTR unsigned long lastLocalSensorReadTime = 0; // Timestamp for last local sensor read on receiver
RTC_DATA_ATTR unsigned long lastButtonPressTime = 0; // Timestamp for button debounce
const unsigned long BUTTON_DEBOUNCE_DELAY = 200; // Debounce delay for push button

// --- Node MAC Addresses ---
// MAC addresses of the nodes in the mesh.
// These should be unique and known to all nodes.
uint8_t nodeMacs[][ESP_NOW_ETH_ALEN] = {
    {0x10, 0x06, 0x1C, 0x82, 0x70, 0x54}, // Node 1 MAC (Highest Priority)
    {0x5C, 0x01, 0x3B, 0x4E, 0x08, 0xFC}, // Node 2 MAC
    {0xF0, 0x24, 0xF9, 0x59, 0xD3, 0xBC}   // Node 3 MAC (Lowest Priority)
};
#define MAX_NODES (sizeof(nodeMacs) / ESP_NOW_ETH_ALEN) // Calculate the number of nodes
uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Standard Broadcast MAC address

// --- Enumerations ---
typedef enum { ROLE_SENDER, ROLE_RECEIVER } node_role_t; // Defines node roles
typedef enum { STANDBY_MODE, DATA_COLLECTION_MODE } operating_mode_t; // Defines operating modes
typedef enum { MSG_TYPE_DATA, MSG_TYPE_PROBE, MSG_TYPE_ANNOUNCE_AR, MSG_TYPE_MODE_CHANGE } message_type_t; // Defines message types

// --- ESP-NOW Message Structure ---
// This structure defines the format of data sent over ESP-NOW.
// __attribute__((packed)) ensures no padding is added, maintaining consistent size.
typedef struct __attribute__((packed)) {
    message_type_t type;              // Type of message (data, probe, announce AR, mode change)
    uint8_t senderId;                 // ID of the sender node (1-based)
    int wakeCounter;                  // Counter from ULP wakeups (for data messages)
    long tilt_centi_deg;              // Decoded tilt data from CH32 (in centi-degrees)
    long vibration_milli_g;           // Decoded vibration data from CH32 (in milli-g)
    int rain_adc_value;               // Current Rain ADC reading (direct analog sensor)
    int soil_adc_value;               // Current Soil ADC reading (direct analog sensor)
    operating_mode_t newMode;          // New mode for MSG_TYPE_MODE_CHANGE, also used to announce AR's current mode
    uint8_t dummy_payload[4];          // Placeholder to align structure size, if needed
} struct_message;

// --- Structure to hold current sensor readings for TCP server ---
typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    int rain_adc_value;
    int soil_adc_value;
    long vibration_milli_g;
    long tilt_centi_deg;
    unsigned long lastUpdated; // Timestamp of last update
    bool initialized; // Flag to indicate if this entry has valid data
} CurrentSensorReadings;

// Global array to store the latest sensor readings for each node (index 0 unused)
RTC_DATA_ATTR CurrentSensorReadings latestNodeData[MAX_NODES + 1];

// --- Global Variables for Current Node State ---
int myNodeId = 0;             // This device's node ID (1-based)
node_role_t myRole = ROLE_SENDER; // Current role of this node
int targetNodeId = 0;         // The ID of the node this sender is currently targeting (persisted in NVS)
operating_mode_t currentOperatingMode = STANDBY_MODE; // Current operating mode (persisted in NVS)
unsigned long lastProbeTime = 0;      // Timestamp for last probe (for receiver role to check for higher-priority ARs)
unsigned long lastSendAttemptTime = 0; // Timestamp for the last data send attempt
struct_message messageToSend; // Global message structure to avoid re-creation on each send

// NEW: Array to track if each node (by ID) has received the current AR operating mode
// This is used by the AR (Node 1) to manage which nodes need a mode announcement ACK.
bool nodeModeSynced[MAX_NODES + 1]; // Index 0 unused, nodes 1 to MAX_NODES

// --- Global Constants and Defines for PWM Decoder (from fin_pwm_decode.txt) ---
const int TILT_PWM_PIN = 13;      // ESP32 GPIO connected to CH32 PC4 (Tilt PWM output)
const int VIBRATION_PWM_PIN = 14; // ESP32 GPIO connected to CH32 PD4 (Vibration PWM output)

// Constants from the CH32V003 code for accurate re-mapping.
// CH32's ATRLR is 4799, so duty cycle range is 0 to 4799 (4800 distinct values).
const long CH32_PWM_MAX_DUTY_VALUE = 4799;

// Maximum expected values for the sensor proxies from the CH32V003 code (for reverse mapping).
#define MAX_TILT_CENTI_DEG      (90 * 100)      // Max tilt in centi-degrees (90.00 degrees)
#define MAX_VIBRATION_MDPS      750000          // Upper limit for vibration proxy in milli-deg/s

// Constants for the secondary mapping of vibration data (from CH32V003 code).
#define MIN_DISPLAY_VIBRATION_MILLI_G 50    // Corresponds to 0.050g
#define MAX_DISPLAY_VIBRATION_MILLI_G 25000 // Corresponds to 25.000g

// PWM Frequencies from the CH32V003 code.
const long TILT_PWM_FREQUENCY_HZ = 10;
const long VIBRATION_PWM_FREQUENCY_HZ = 1;

// Calculate PWM periods in microseconds.
const long TILT_PWM_PERIOD_US = 1000000L / TILT_PWM_FREQUENCY_HZ;         // 100,000 us
const long VIBRATION_PWM_PERIOD_US = 1000000L / VIBRATION_PWM_FREQUENCY_HZ; // 1,000,000 us

// WebServer instance for TCP communication
WebServer server(80);

// --- Helper Functions ---
// Function to print MAC addresses in a human-readable format.
void printMac(const uint8_t *mac) {
    for (int i = 0; i < 6; i++) {
        if (i > 0) Serial.print(":");
        Serial.printf("%02X", mac[i]);
    }
}

// Function to get a node's ID (1-based) from its MAC address.
// Returns 0 if the MAC is not found in the `nodeMacs` list.
int getNodeIdFromMac(const uint8_t *mac_addr) {
    for (int i = 0; i < MAX_NODES; i++) {
        if (memcmp(mac_addr, nodeMacs[i], ESP_NOW_ETH_ALEN) == 0) return i + 1; // Node IDs are 1-based
    }
    return 0; // Not a recognized node
}

// --- NVS Helper Functions ---
// Reads the operating mode from NVS. Returns STANDBY_MODE if not found.
operating_mode_t readModeFromNVS() {
    nvs_handle_t nvs_handle;
    operating_mode_t mode = STANDBY_MODE; // Default mode
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        Serial.printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        int8_t storedMode;
        err = nvs_get_i8(nvs_handle, NVS_KEY_MODE_FLAG, &storedMode);
        if (err == ESP_OK) {
            mode = (operating_mode_t)storedMode;
            Serial.printf("NVS: Read mode_flag = %s\n", (mode == STANDBY_MODE ? "STANDBY_MODE" : "DATA_COLLECTION_MODE"));
        } else if (err == ESP_ERR_NVS_NOT_FOUND) {
            Serial.println("NVS: mode_flag not found. Using default STANDBY_MODE.");
        } else {
            Serial.printf("Error (%s) reading mode_flag from NVS!\n", esp_err_to_name(err));
        }
        nvs_close(nvs_handle);
    }
    return mode;
}

// Writes the operating mode to NVS.
void writeModeToNVS(operating_mode_t mode) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        Serial.printf("Error (%s) opening NVS handle for write!\n", esp_err_to_name(err));
        return;
    }
    err = nvs_set_i8(nvs_handle, NVS_KEY_MODE_FLAG, (int8_t)mode);
    if (err == ESP_OK) {
        Serial.printf("NVS: Wrote mode_flag = %s\n", (mode == STANDBY_MODE ? "STANDBY_MODE" : "DATA_COLLECTION_MODE"));
        err = nvs_commit(nvs_handle);
        if (err != ESP_OK) {
            Serial.printf("Error (%s) committing NVS write!\n", esp_err_to_name(err));
        }
    } else {
        Serial.printf("Error (%s) writing mode_flag to NVS!\n", esp_err_to_name(err));
    }
    nvs_close(nvs_handle);
}

// Reads the target node ID from NVS. Returns 0 if not found.
int readTargetNodeIdFromNVS() {
    nvs_handle_t nvs_handle;
    int storedTargetId = 0;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        Serial.printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        err = nvs_get_i8(nvs_handle, NVS_KEY_TARGET_NODE_ID, (int8_t*)&storedTargetId);
        if (err == ESP_OK) {
            Serial.printf("NVS: Read target_node_id = %d\n", storedTargetId);
        } else if (err == ESP_ERR_NVS_NOT_FOUND) {
            Serial.println("NVS: target_node_id not found.");
        } else {
            Serial.printf("Error (%s) reading target_node_id from NVS!\n", esp_err_to_name(err));
        }
        nvs_close(nvs_handle);
    }
    return storedTargetId;
}

// Writes the target node ID to NVS.
void writeTargetNodeIdToNVS(int id) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        Serial.printf("Error (%s) opening NVS handle for write!\n", esp_err_to_name(err));
        return;
    }
    err = nvs_set_i8(nvs_handle, NVS_KEY_TARGET_NODE_ID, (int8_t)id);
    if (err == ESP_OK) {
        Serial.printf("NVS: Wrote target_node_id = %d\n", id);
        err = nvs_commit(nvs_handle);
        if (err != ESP_OK) {
            Serial.printf("Error (%s) committing NVS write!\n", esp_err_to_name(err));
        }
    } else {
        Serial.printf("Error (%s) writing target_node_id to NVS!\n", esp_err_to_name(err));
    }
    nvs_close(nvs_handle);
}

// --- TCP Server Handler Functions ---

// This function handles requests to the /sensor_data endpoint, providing current sensor values.
// It now supports a nodeId parameter to fetch data for other nodes.
void handleSensorData() {
    int requestedNodeId = myNodeId; // Default to this node's data
    if (server.hasArg("nodeId")) {
        requestedNodeId = server.arg("nodeId").toInt();
    }

    // Check if the requested nodeId is valid and if data for it is initialized
    if (requestedNodeId >= 1 && requestedNodeId <= MAX_NODES && latestNodeData[requestedNodeId].initialized) {
        String macString = "";
        for (int i = 0; i < ESP_NOW_ETH_ALEN; i++) {
            if (i > 0) macString += ":";
            char hex[3];
            sprintf(hex, "%02X", latestNodeData[requestedNodeId].mac_addr[i]);
            macString += hex;
        }

        String json = "{";
        json += "\"nodeId\": " + String(requestedNodeId) + ",";
        json += "\"mac\": \"" + macString + "\",";
        json += "\"rain\": " + String(latestNodeData[requestedNodeId].rain_adc_value) + ",";
        json += "\"soil\": " + String(latestNodeData[requestedNodeId].soil_adc_value) + ",";
        json += "\"vibration\": " + String(latestNodeData[requestedNodeId].vibration_milli_g) + ",";
        json += "\"tilt\": " + String(latestNodeData[requestedNodeId].tilt_centi_deg) + ",";
        json += "\"lastUpdated\": " + String(latestNodeData[requestedNodeId].lastUpdated);
        json += "}";
        server.send(200, "application/json", json);
        Serial.printf("Served /sensor_data request for Node %d with: %s\n", requestedNodeId, json.c_str());
    } else {
        // Send a 404 Not Found or an error message if data is not available or nodeId is invalid
        server.send(404, "application/json", "{\"error\":\"Sensor data for requested nodeId not found or invalid.\"}");
        Serial.printf("Failed to serve /sensor_data request for Node %d: Data not available or invalid nodeId.\n", requestedNodeId);
    }
}

// New handler for setting operating mode via PC.
void handleSetMode() {
    if (server.hasArg("mode")) {
        String modeStr = server.arg("mode");
        operating_mode_t newMode;
        bool validMode = false;

        if (modeStr.equalsIgnoreCase("standby")) {
            newMode = STANDBY_MODE;
            validMode = true;
        } else if (modeStr.equalsIgnoreCase("data_collection")) {
            newMode = DATA_COLLECTION_MODE;
            validMode = true;
        } else {
            server.send(400, "application/json", "{\"error\":\"Invalid mode specified. Use 'standby' or 'data_collection'.\"}");
            Serial.printf("PC Mode Change: Invalid mode '%s' received.\n", modeStr.c_str());
            return;
        }

        if (validMode && currentOperatingMode != newMode) {
            Serial.printf("[Node %d] PC request to change mode to %s.\n",
                          myNodeId, (newMode == STANDBY_MODE ? "STANDBY_MODE" : "DATA_COLLECTION_MODE"));
            
            currentOperatingMode = newMode;
            writeModeToNVS(currentOperatingMode); // Persist the new mode

            // Broadcast MSG_TYPE_MODE_CHANGE to other nodes
            messageToSend.type = MSG_TYPE_MODE_CHANGE;
            messageToSend.senderId = myNodeId;
            messageToSend.newMode = currentOperatingMode;
            for (int i = 0; i < 5; ++i) { // Send multiple times for robustness
                esp_now_send(broadcastMac, (uint8_t *)&messageToSend, sizeof(messageToSend));
                delay(10); // Small delay between broadcasts
            }
            Serial.println("Mode change broadcasted.");

            // Immediately reconfigure ULP based on the new mode on this node
            if (currentOperatingMode == STANDBY_MODE) {
                ulp_init(); // Enable ULP for standby
                Serial.println("ULP re-initialized for STANDBY_MODE on this node.");
            } else {
                hulp_ulp_end(); // Disable ULP for data collection
                Serial.println("ULP stopped for DATA_COLLECTION_MODE on this node.");
            }

            // If this node is the AR, reset sync flags for other nodes
            if (myNodeId == 1) {
                for (int i = 0; i <= MAX_NODES; ++i) {
                    nodeModeSynced[i] = false;
                }
                Serial.println("[Node 1 AR] Mode changed, resetting nodeModeSynced flags for all nodes.");
                // Also, send an initial ANNOUNCE_AR broadcast to alert any awake nodes immediately.
                messageToSend.type = MSG_TYPE_ANNOUNCE_AR;
                messageToSend.senderId = myNodeId;
                messageToSend.newMode = currentOperatingMode;
                esp_now_send(broadcastMac, (uint8_t *)&messageToSend, sizeof(messageToSend));
                Serial.println("[Node 1 AR] Sent ANNOUNCE_AR broadcast after PC mode change.");
            }
            
            server.send(200, "application/json", "{\"status\":\"success\", \"newMode\":\"" + modeStr + "\"}");
            Serial.printf("PC Mode Change: Mode successfully changed to %s.\n", modeStr.c_str());
        } else if (validMode && currentOperatingMode == newMode) {
            server.send(200, "application/json", "{\"status\":\"unchanged\", \"currentMode\":\"" + modeStr + "\", \"message\":\"Mode already set to " + modeStr + ".\"}");
            Serial.printf("PC Mode Change: Mode is already %s. No change needed.\n", modeStr.c_str());
        }
    } else {
        server.send(400, "application/json", "{\"error\":\"Mode parameter missing. Use /set_mode?mode=standby or /set_mode?mode=data_collection.\"}");
        Serial.println("PC Mode Change: Mode parameter missing in request.");
    }
}

// --- Role Management Functions ---
// Transitions the current node to an Active Receiver (AR) role.
// An AR stays awake to receive data from senders and periodically probes.
void becomeActiveReceiver() {
    myRole = ROLE_RECEIVER;
    targetNodeId = 0; // ARs do not have a specific target sender
    lastAnnouncedAR = myNodeId; // Announce self as the new AR
    consecutiveSendFailures = 0; // Reset failures counter on role change
    writeTargetNodeIdToNVS(targetNodeId); // Store targetNodeId in NVS
    Serial.printf("[Node %d] Became ACTIVE RECEIVER\n", myNodeId);
    
    // Configure WiFi as Access Point and Station for AR
    // This is done after initial ESP-NOW setup in setup()
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(softAP_ssid, softAP_password);
    IPAddress ip(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(ip, gateway, subnet);
    Serial.printf("AP Started: %s (IP: %s)\n", softAP_ssid, ip.toString().c_str());

    // Register web server routes
    server.on("/sensor_data", HTTP_GET, handleSensorData);
    server.on("/set_mode", HTTP_GET, handleSetMode); // Register the new mode control endpoint
    server.begin(); // Start the web server
    Serial.println("Web server started.");

    // NEW: When AR becomes active or its mode changes, reset sync status for all other nodes
    for (int i = 0; i <= MAX_NODES; ++i) { // Reset all entries, including unused index 0 for clarity
        nodeModeSynced[i] = false;
    }
    Serial.println("[Node 1 AR] Resetting nodeModeSynced flags for all nodes.");
    // Initial broadcast of ANNOUNCE_AR to inform all other nodes.
    // This is crucial for initial discovery if other nodes are already awake.
    messageToSend.type = MSG_TYPE_ANNOUNCE_AR;
    messageToSend.senderId = myNodeId;
    messageToSend.newMode = currentOperatingMode; // IMPORTANT: Include current operating mode
    esp_now_send(broadcastMac, (uint8_t *)&messageToSend, sizeof(messageToSend));
    Serial.println("[Node 1 AR] Sent initial ANNOUNCE_AR broadcast.");
}

// Transitions the current node to a Sender role, targeting a specific AR.
// Senders primarily deep sleep and wake to send data to their target AR.
void becomeSender(int newTargetId) {
    // Validate the new target ID to ensure it's within the valid range.
    if (newTargetId < 1 || newTargetId > MAX_NODES) {
        Serial.printf("[Node %d] Invalid target node: %d. Defaulting to Node 1\n", myNodeId, newTargetId);
        newTargetId = 1; // Fallback to Node 1 if the provided ID is invalid
    }
    myRole = ROLE_SENDER;
    targetNodeId = newTargetId; // Set the new target Active Receiver
    lastAnnouncedAR = newTargetId; // Update last announced AR based on our new target
    consecutiveSendFailures = 0; // Reset failures counter on role change
    writeTargetNodeIdToNVS(targetNodeId); // Store targetNodeId in NVS
    Serial.printf("[Node %d] Became SENDER targeting Node %d\n", myNodeId, newTargetId);

    // Set WiFi mode to station for senders and stop AP/web server if previously running
    // This is done after initial ESP-NOW setup in setup()
    WiFi.mode(WIFI_STA);
    WiFi.softAPdisconnect(true); // Disconnect SoftAP if it was running
    server.stop(); // Stop the web server if it was running
    Serial.println("WiFi mode set to STA. Web server stopped.");
}

// Initiates the failover process to find a new Active Receiver.
// Called when a sender experiences consecutive failures to reach its current target AR.
void findNewTargetAndFailover() {
    int failedNodeId = lastFailedTargetId;
    // If the failed target ID is unknown, reset to default and target Node 1.
    if (failedNodeId == 0) {
        Serial.printf("!!! [%d] FAILOVER Triggered without known failed target. Resetting.\n", myNodeId);
        consecutiveSendFailures = 0;
        lastFailedTargetId = 0;
        becomeSender(1); // Default to Node 1 as the target
        return;
    }
    Serial.printf("!!! [%d] FAILOVER Triggered: Node %d seems down\n", myNodeId, failedNodeId);
    // Determine the next potential AR by incrementing the failed node's ID.
    int expectedNewAR = failedNodeId + 1;
    // If the next expected AR is beyond the maximum number of nodes or is myself,
    // then this node should promote itself to become the new Active Receiver.
    if (expectedNewAR > MAX_NODES || expectedNewAR == myNodeId) {
        becomeActiveReceiver();
    } else {
        // Otherwise, become a sender targeting the next node in sequence.
        becomeSender(expectedNewAR);
    }
}

// --- ESP-NOW Callbacks ---
// Callback function executed when an ESP-NOW message is sent.
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    int remoteId = getNodeIdFromMac(mac_addr);
    if (status == ESP_NOW_SEND_SUCCESS) {
        Serial.printf("[Node %d] Send OK to Node %d\n", myNodeId, remoteId);
        consecutiveSendFailures = 0; // Reset failures on successful send
        probingForFailover = false;  // Stop probing if send was successful
        failedLastSend = false;      // Reset the failed send flag
        ulpJustWoke = false;         // Reset ulpJustWoke after successful send
    } else {
        Serial.printf("[Node %d] Send FAIL to Node %d\n", myNodeId, remoteId);
        failedLastSend = true; // Set flag for the loop to stay awake and retry/probe
        // Only increment failures if the failed send was to our current target AR.
        if (remoteId == targetNodeId) {
            consecutiveSendFailures++;
            lastFailedTargetId = remoteId; // Record the node that caused the failure
            // If consecutive failures reach the threshold, initiate failover.
            if (consecutiveSendFailures >= FAILOVER_THRESHOLD) {
                findNewTargetAndFailover(); // Trigger the failover process
                failedLastSend = false; // Reset to allow deep sleep after failover action
                return;
            }
        }
    }
}

// Callback function executed when an ESP-NOW message is received.
void onDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
    int senderId = getNodeIdFromMac(mac_addr);
    // --- DEBUGGING START ---
    Serial.printf("[Node %d RECEIVE DEBUG] Incoming message from MAC: ", myNodeId);
    for(int i=0; i<6; ++i) {
        Serial.printf("%02X", mac_addr[i]);
        if(i < 5) Serial.print(":");
    }
    Serial.printf(". Sender ID: %d, Length: %d, Expected Length: %d\n", senderId, len, sizeof(struct_message));
    // --- DEBUGGING END ---
    // Validate sender ID and message length.
    if (senderId == 0 || len != sizeof(struct_message)) {
        Serial.printf("[Node %d RECEIVE DEBUG] Message validation FAILED. senderId: %d, len: %d, expected: %d\n", myNodeId, senderId, len, sizeof(struct_message));
        return;
    }
    struct_message msg;
    memcpy(&msg, data, sizeof(msg)); // Copy received data into message structure
    Serial.printf("[Node %d] Got message from Node %d, Type=%d\n", myNodeId, msg.senderId, msg.type);
    if (msg.type == MSG_TYPE_ANNOUNCE_AR) {
        // An ANNOUNCE_AR message indicates a new Active Receiver.
        lastAnnouncedAR = msg.senderId;
        // --- AR Takeback Logic ---
        // If I am an AR, and a higher-priority node (lower ID) announced itself as AR,
        // I should yield to the higher-priority node.
        if (myRole == ROLE_RECEIVER && senderId < myNodeId) {
            Serial.printf("[Node %d] Yielding AR role to Node %d (higher priority AR).\n", myNodeId, senderId);
            becomeSender(senderId); // Demote self and become a sender targeting the new AR.
            // When demoting from AR, also re-initialize ULP if in STANDBY_MODE
            if (currentOperatingMode == STANDBY_MODE) {
                ulp_init(); // Re-enable ULP for deep sleep
                Serial.println("ULP re-initialized for STANDBY_MODE after AR demotion.");
            } else {
                hulp_ulp_end(); // Ensure ULP is off for DATA_COLLECTION_MODE
                Serial.println("ULP stopped for DATA_COLLECTION_MODE after AR demotion.");
            }
        }
        // If I am a sender and a different AR was announced, switch my target.
        else if (myRole == ROLE_SENDER && senderId != targetNodeId) {
            Serial.printf("[Node %d] Switching to new AR Node %d\n", myNodeId, senderId);
            becomeSender(senderId); // Update target to the newly announced AR.
        }
        // Handle mode synchronization if the AR's mode is different from ours
        if (currentOperatingMode != msg.newMode) {
            Serial.printf("[Node %d] RECEIVED ANNOUNCE_AR with new mode %s from Node %d. Updating my mode.\n",
                            myNodeId, (msg.newMode == STANDBY_MODE ? "STANDBY_MODE" : "DATA_COLLECTION_MODE"), msg.senderId);
            Serial.printf("[Node %d] OLD MODE: %s, NEW MODE FROM AR: %s\n", myNodeId,
                            (currentOperatingMode == STANDBY_MODE ? "STANDBY_MODE" : "DATA_COLLECTION_MODE"),
                            (msg.newMode == STANDBY_MODE ? "STANDBY_MODE" : "DATA_COLLECTION_MODE"));
            currentOperatingMode = msg.newMode;
            writeModeToNVS(currentOperatingMode); // Persist the new mode
            // Re-initialize ULP based on the new mode immediately
            if (currentOperatingMode == STANDBY_MODE) {
                ulp_init(); // Enable ULP for standby
                Serial.println("ULP re-initialized for STANDBY_MODE.");
            } else {
                hulp_ulp_end(); // Disable ULP for data collection
                Serial.println("ULP stopped for DATA_COLLECTION_MODE.");
            }
            Serial.printf("[Node %d] MODE AFTER UPDATE IN CALLBACK: %s\n", myNodeId,
                            (currentOperatingMode == STANDBY_MODE ? "STANDBY_MODE" : "DATA_COLLECTION_MODE"));
        } else {
             Serial.printf("[Node %d] Received ANNOUNCE_AR with mode %s, my mode already matches.\n",
                                 myNodeId, (msg.newMode == STANDBY_MODE ? "STANDBY_MODE" : "DATA_COLLECTION_MODE"));
        }
    } else if (msg.type == MSG_TYPE_PROBE && myRole == ROLE_RECEIVER) {
        // If a probe is received and I am an AR, respond with an ANNOUNCE_AR
        // to confirm my presence and role to the probing node.
        Serial.printf("[Node %d AR] Responding to PROBE from Node %d with ANNOUNCE_AR.\n", myNodeId, senderId);
        messageToSend.type = MSG_TYPE_ANNOUNCE_AR;
        messageToSend.senderId = myNodeId;
        messageToSend.wakeCounter = 0; // Probes do not carry wake counter data
        messageToSend.newMode = currentOperatingMode; // IMPORTANT: Include current operating mode
        esp_now_send(mac_addr, (uint8_t *)&messageToSend, sizeof(messageToSend));
        nodeModeSynced[senderId] = true; // Mark this node as synced with AR's current mode
    } else if (msg.type == MSG_TYPE_DATA && myRole == ROLE_RECEIVER) {
            // If I am a receiver and received a DATA message, process the sensor data.
            Serial.printf("[Node %d] Received Data from Node %d (WakeCount: %d):\n", myNodeId, msg.senderId, msg.wakeCounter);
            Serial.printf("    Tilt: %ld.%02ld deg | Vibration: %ld.%03ld g\n",
                          msg.tilt_centi_deg / 100, abs(msg.tilt_centi_deg % 100), // Format for two decimal places
                          msg.vibration_milli_g / 1000, abs(msg.vibration_milli_g % 1000)); // Format for three decimal places with abs
            Serial.printf("    Raw ADC: Rain=%d, Soil=%d\n", msg.rain_adc_value, msg.soil_adc_value);
            // Here you would typically log the received data to storage (e.g., SD card, server)
            // or perform further processing based on the application's needs.

            // Store the received data in the global latestNodeData array
            if (senderId >= 1 && senderId <= MAX_NODES) {
                memcpy(latestNodeData[senderId].mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
                latestNodeData[senderId].rain_adc_value = msg.rain_adc_value;
                latestNodeData[senderId].soil_adc_value = msg.soil_adc_value;
                latestNodeData[senderId].vibration_milli_g = msg.vibration_milli_g;
                latestNodeData[senderId].tilt_centi_deg = msg.tilt_centi_deg;
                latestNodeData[senderId].lastUpdated = millis();
                latestNodeData[senderId].initialized = true;
                Serial.printf("[Node %d AR] Stored data for Node %d.\n", myNodeId, senderId);
            }

            // NEW: Send ANNOUNCE_AR back as an acknowledgment for received data if not already synced
            // This also implicitly provides the AR's current operating mode.
            if (!nodeModeSynced[senderId]) {
                Serial.printf("[Node %d AR] Sending ANNOUNCE_AR ACK to Node %d (unsynced) after receiving data.\n", myNodeId, senderId);
                messageToSend.type = MSG_TYPE_ANNOUNCE_AR;
                messageToSend.senderId = myNodeId;
                messageToSend.wakeCounter = 0; // ANNOUNCE_AR messages don't use wakeCounter
                messageToSend.newMode = currentOperatingMode; // Crucial: Include current operating mode
                esp_now_send(mac_addr, (uint8_t *)&messageToSend, sizeof(messageToSend));
                nodeModeSynced[senderId] = true; // Mark this node as synced with AR's current mode
            } else {
                Serial.printf("[Node %d AR] Received data from Node %d (already synced). No ANNOUNCE_AR ACK sent.\n", myNodeId, senderId);
            }
    } else if (msg.type == MSG_TYPE_MODE_CHANGE) {
        Serial.printf("[Node %d] Received MODE_CHANGE to %s from Node %d\n",
                              myNodeId,
                              (msg.newMode == STANDBY_MODE ? "STANDBY_MODE" : "DATA_COLLECTION_MODE"),
                              msg.senderId);
        if (currentOperatingMode != msg.newMode) {
            currentOperatingMode = msg.newMode;
            writeModeToNVS(currentOperatingMode); // Persist the new mode
            Serial.printf("[Node %d] Mode changed to %s. Reconfiguring...\n",
                              myNodeId,
                              (currentOperatingMode == STANDBY_MODE ? "STANDBY_MODE" : "DATA_COLLECTION_MODE"));
            // Re-initialize ULP based on the new mode
            if (currentOperatingMode == STANDBY_MODE) {
                ulp_init(); // Enable ULP for standby
                Serial.println("ULP re-initialized for STANDBY_MODE.");
            } else {
                hulp_ulp_end(); // Changed from hulp_ulp_stop() to hulp_ulp_end()
                Serial.println("ULP stopped for DATA_COLLECTION_MODE.");
            }
            // If the AR (myNodeId == 1) receives a mode change (e.g., from button press on itself),
            // it needs to reset the synced flags for other nodes so they are forced to re-sync.
            if (myNodeId == 1) {
                for (int i = 0; i <= MAX_NODES; ++i) {
                    nodeModeSynced[i] = false;
                }
                Serial.println("[Node 1 AR] Mode changed, resetting nodeModeSynced flags for all nodes.");
                // Also, send an initial broadcast to alert any awake nodes immediately.
                messageToSend.type = MSG_TYPE_ANNOUNCE_AR;
                messageToSend.senderId = myNodeId;
                messageToSend.newMode = currentOperatingMode;
                esp_now_send(broadcastMac, (uint8_t *)&messageToSend, sizeof(messageToSend));
                Serial.println("[Node 1 AR] Sent ANNOUNCE_AR broadcast after mode change.");
            }
        }
    }
}

// --- PWM Decoding Function (from fin_pwm_decode.txt) ---
// This function measures the incoming PWM pulse widths from the CH32V003
// and reverse-maps them to reconstruct the original tilt and vibration values.
void decodePwmSignals(long& decoded_tilt, long& decoded_vibration_milli_g) {
    // --- Decode Tilt PWM (from PC4/TIM1, 10Hz) ---
    // Measure the high pulse duration for Tilt PWM in microseconds.
    // A timeout (2x period) prevents indefinite blocking if no pulse is detected.
    long tilt_pulse_us = pulseIn(TILT_PWM_PIN, HIGH, TILT_PWM_PERIOD_US * 2);
    decoded_tilt = 0; // Initialize decoded tilt value
    if (tilt_pulse_us > 0) {
        // Calculate the equivalent CH32 PWM duty value from the measured pulse width.
        // The CH32 maps 0-4799 to its timer's pulse width range. We reverse this.
        long decoded_pwm_val_tilt = (tilt_pulse_us * (CH32_PWM_MAX_DUTY_VALUE + 1)) / TILT_PWM_PERIOD_US;
        // Map this decoded PWM value back to the original tilt centi-degrees range.
        decoded_tilt = map(decoded_pwm_val_tilt,
                               0, CH32_PWM_MAX_DUTY_VALUE,
                               0, MAX_TILT_CENTI_DEG);
    } else {
        // Serial.print("Warning: No Tilt PWM signal detected or timeout on pin "); // Commented out by Gemini
        // Serial.println(TILT_PWM_PIN); // Commented out by Gemini
    }
    // --- Decode Vibration PWM (from PD4/TIM2, 1Hz) ---
    // Measure the high pulse duration for Vibration PWM in microseconds.
    long vibration_pulse_us = pulseIn(VIBRATION_PWM_PIN, HIGH, VIBRATION_PWM_PERIOD_US * 2);
    long decoded_vibration_mdps = 0;
    decoded_vibration_milli_g = 0; // Initialize final display value
    if (vibration_pulse_us > 0) {
        // Calculate the equivalent CH32 PWM duty value for vibration.
        long decoded_pwm_val_vibration = (vibration_pulse_us * (CH32_PWM_MAX_DUTY_VALUE + 1)) / VIBRATION_PWM_PERIOD_US;
        // Map this decoded PWM value back to the original vibration milli-degrees per second range.
        decoded_vibration_mdps = map(decoded_pwm_val_vibration,
                                         0, CH32_PWM_MAX_DUTY_VALUE,
                                         0, MAX_VIBRATION_MDPS);
        // Now, map the mdps value to milli-g, consistent with the CH32's display logic.
        decoded_vibration_milli_g = map(decoded_vibration_mdps,
                                          0, MAX_VIBRATION_MDPS,
                                          MIN_DISPLAY_VIBRATION_MILLI_G, MAX_DISPLAY_VIBRATION_MILLI_G);
    } else {
        // Serial.print("Warning: No Vibration PWM signal detected or timeout on pin "); // Commented out by Gemini
        // Serial.println(VIBRATION_PWM_PIN); // Commented out by Gemini
    }
}

// --- ULP (Ultra-Low Power) Co-processor Initialization (from fin_ulp.txt) ---
// This function sets up and loads the ULP program onto the ESP32's ULP co-processor.
// The ULP runs autonomously in deep sleep to monitor sensors and wake the main CPU
// based on delta thresholds for rain/soil or a high signal on the data ready pin.
void ulp_init() {
    enum {
        LBL_SOIL_CHECK,
        LBL_READY_CHECK,
        LBL_TRIGGERED,
        LBL_HALT,
    };
    const ulp_insn_t program[] = {
        I_MOVI(R3, 0), // R3 = 0, base for I_PUT/I_GET
        // --- RAIN SENSOR CHECK ---
        I_ANALOG_READ(R1, PIN_ADC_RAIN),        // Read current Rain ADC value into R1
        I_PUT(R1, R3, ulp_vars.last_rain_val), // Store current Rain ADC value in RTC memory
        I_GET(R0, R3, ulp_vars.rain_lower),   // Get lower threshold for Rain into R0
        I_SUBR(R2, R1, R0),                   // R2 = Current Rain ADC - Lower Threshold
        M_BXF(LBL_TRIGGERED),                 // If R2 is negative (Current < Lower), trigger wakeup
        I_GET(R0, R3, ulp_vars.rain_upper),   // Get upper threshold for Rain into R0
        I_SUBR(R2, R0, R1),                   // R2 = Upper Threshold - Current Rain ADC
        M_BXF(LBL_TRIGGERED),                 // If R2 is negative (Current > Upper), trigger wakeup
        // --- SOIL SENSOR CHECK ---
        M_LABEL(LBL_SOIL_CHECK),              // Label for Soil Sensor check
        I_ANALOG_READ(R1, PIN_ADC_SOIL),        // Read current Soil ADC value into R1
        I_PUT(R1, R3, ulp_vars.last_soil_val), // Store current Soil ADC value in RTC memory
        I_GET(R0, R3, ulp_vars.soil_lower),   // Get lower threshold for Soil into R0
        I_SUBR(R2, R1, R0),                   // R2 = Current Soil ADC - Lower Threshold
        M_BXF(LBL_TRIGGERED),                 // If R2 is negative (Current < Lower), trigger wakeup
        I_GET(R0, R3, ulp_vars.soil_upper),   // Get upper threshold for Soil into R0
        I_SUBR(R2, R0, R1),                   // R2 = Upper Threshold - Current Soil ADC
        M_BXF(LBL_TRIGGERED),                 // If R2 is negative (Current > Upper), trigger wakeup
        // --- DATA READY CHECK (CH32) ---
        M_LABEL(LBL_READY_CHECK),             // Label for Data Ready check
        I_ANALOG_READ(R1, PIN_ADC_READY),       // Read current Data Ready ADC value into R1
        I_PUT(R1, R3, ulp_vars.last_ready_val),// Store current Data Ready ADC value in RTC memory
        I_MOVI(R0, DATA_READY_THRESHOLD),     // Load DATA_READY_THRESHOLD into R0
        I_SUBR(R2, R1, R0),                   // R2 = Current Ready ADC - Threshold
        M_BXF(LBL_HALT),                      // If R2 is negative (Current < Threshold), don't wake, go to HALT
        M_BX(LBL_TRIGGERED),                  // If R2 is non-negative (Current >= Threshold), trigger wakeup
        M_BX(LBL_HALT), // Fallback if no conditions met (should ideally go to HALT without explicit branch)
        M_LABEL(LBL_TRIGGERED),               // Label for wakeup sequence
            I_GET(R0, R3, ulp_vars.triggered_count), // Get current triggered count
            I_ADDI(R0, R0, 1),                    // Increment count
            I_PUT(R0, R3, ulp_vars.triggered_count), // Store back updated count
            M_WAKE_WHEN_READY(),                  // Wake up the main CPU
        M_LABEL(LBL_HALT),                    // Label for ULP halt
            I_HALT(),
    };
    // Configure analog pins for ADC readings with specified attenuation and bit width.
    ESP_ERROR_CHECK(hulp_configure_analog_pin(PIN_ADC_RAIN, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12));
    ESP_ERROR_CHECK(hulp_configure_analog_pin(PIN_ADC_SOIL, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12));
    ESP_ERROR_CHECK(hulp_configure_analog_pin(PIN_ADC_READY, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12));
    // Load the ULP program and set the wakeup interval.
    ESP_ERROR_CHECK(hulp_ulp_load(program, sizeof(program), 1000UL * ULP_WAKEUP_INTERVAL_MS, 0));
    // Run the ULP program from the beginning (offset 0).
    ESP_ERROR_CHECK(hulp_ulp_run(0));
}

// --- Arduino Setup Function ---
void setup() {
    Serial.begin(115200); // Initialize serial communication for debugging
    delay(500); // Give time for serial monitor to connect

    // Initialize NVS (Non-Volatile Storage)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        Serial.println("NVS: Erasing flash...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    Serial.println("NVS initialized.");

    // Determine myNodeId early based on MAC address
    uint8_t mac[6];
    WiFi.macAddress(mac); // Get this device's MAC address
    myNodeId = getNodeIdFromMac(mac); // Determine this device's node ID from its MAC
    Serial.print("This device MAC: "); printMac(mac); Serial.printf(" -> Node ID: %d\n", myNodeId);

    // If MAC address is not recognized in the `nodeMacs` list, halt execution.
    if (!myNodeId) {
        Serial.println("Error: Device MAC not found in nodeMacs list. Halting.");
        while (1); // Infinite loop to stop execution
    }

    // Set initial WiFi mode for all nodes before ESP-NOW initialization.
    // ESP-NOW typically operates on the STA interface.
    WiFi.mode(WIFI_STA);
    Serial.println("Initial WiFi mode set to WIFI_STA.");
    
    // Initialize ESP-NOW AFTER setting the initial WiFi mode
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        while (1); // Infinite loop on failure
    }

    // Register ESP-NOW send and receive callbacks.
    // These functions will be called automatically when messages are sent or received.
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    // Add all other nodes in the mesh as ESP-NOW peers.
    // This allows direct communication between nodes.
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer)); // Initialize peer structure
    for (int i = 0; i < MAX_NODES; i++) {
        // Skip adding self as peer
        if (memcmp(mac, nodeMacs[i], ESP_NOW_ETH_ALEN) == 0) continue;
        
        memcpy(peer.peer_addr, nodeMacs[i], ESP_NOW_ETH_ALEN);
        peer.channel = WIFI_CHANNEL;
        peer.encrypt = false;         // Encryption set to false for simplicity
        peer.ifidx = WIFI_IF_STA;     // Always add peers for STA interface for ESP-NOW communication
        if (esp_now_add_peer(&peer) != ESP_OK) {
            Serial.printf("Failed to add peer for node %d (MAC: ", i+1); printMac(nodeMacs[i]); Serial.println(")");
        } else {
            Serial.printf("Added peer for node %d (MAC: ", i+1); printMac(nodeMacs[i]); Serial.println(")");
        }
    }

    // Read operating mode and target node ID from NVS
    currentOperatingMode = readModeFromNVS();
    targetNodeId = readTargetNodeIdFromNVS();
    if (targetNodeId == 0) { // If targetNodeId was not found in NVS or is 0
        targetNodeId = 1; // Default to Node 1
        writeTargetNodeIdToNVS(targetNodeId); // Store default in NVS
    }

    // Initialize myCurrentSensorReadings MAC address (it's for this node)
    // and also initialize the latestNodeData array
    for (int i = 0; i <= MAX_NODES; ++i) {
        latestNodeData[i].initialized = false; // Mark all entries as uninitialized
    }
    memcpy(latestNodeData[myNodeId].mac_addr, mac, ESP_NOW_ETH_ALEN);
    latestNodeData[myNodeId].initialized = true; // Mark my own data as initialized


    // Configure sensor input pins
    pinMode(PIN_ADC_RAIN, INPUT);
    pinMode(PIN_ADC_SOIL, INPUT);
    pinMode(PIN_ADC_READY, INPUT);
    pinMode(TILT_PWM_PIN, INPUT);
    pinMode(VIBRATION_PWM_PIN, INPUT);
    pinMode(PUSH_BUTTON_PIN, INPUT_PULLUP); // Configure push button with pull-up


    // Check if wakeup was from ULP deep sleep.
    if (hulp_is_deep_sleep_wakeup()) {
        Serial.println("\n🔔 Woken up by ULP!");
        ulpJustWoke = true; // Set flag if woken by ULP
        Serial.print("Last Rain ADC: "); Serial.println(ulp_vars.last_rain_val.val);
        Serial.print("Last Soil ADC: "); Serial.println(ulp_vars.last_soil_val.val);
        Serial.print("Last Ready ADC: "); Serial.println(ulp_vars.last_ready_val.val);
        Serial.print("Triggered count: "); Serial.println(ulp_vars.triggered_count.val);
        // Update ULP thresholds based on last read values for next deep sleep cycle
        uint16_t rain = ulp_vars.last_rain_val.val;
        ulp_vars.rain_lower.val = (rain > SENSOR_WINDOW_MARGIN) ? rain - SENSOR_WINDOW_MARGIN : 0;
        ulp_vars.rain_upper.val = rain + SENSOR_WINDOW_MARGIN;
        uint16_t soil = ulp_vars.last_soil_val.val;
        ulp_vars.soil_lower.val = (soil > SENSOR_WINDOW_MARGIN) ? soil - SENSOR_WINDOW_MARGIN : 0;
        ulp_vars.soil_upper.val = soil + SENSOR_WINDOW_MARGIN;
        // Based on current operating mode, decide initial role after ULP wakeup
        if (currentOperatingMode == STANDBY_MODE) {
            // Normal standby mode wakeup logic
            if (myNodeId == 1) {
                becomeActiveReceiver();
            } else {
                Serial.printf("[Node %d] Waking with %d send fails to Node %d. Last AR: %d\n",
                                 myNodeId, consecutiveSendFailures, lastFailedTargetId, lastAnnouncedAR);
                if (consecutiveSendFailures >= FAILOVER_THRESHOLD) {
                    Serial.printf("[Node %d] Woke up with %d consecutive failures. Triggering failover.\n",
                                     myNodeId, consecutiveSendFailures);
                    findNewTargetAndFailover();
                } else if (lastAnnouncedAR != 0 && lastAnnouncedAR != myNodeId) {
                    Serial.printf("[Node %d] Woke up, last known AR is Node %d. Attempting to send to it.\n",
                                     myNodeId, lastAnnouncedAR);
                    becomeSender(lastAnnouncedAR);
                } else {
                    Serial.printf("[Node %d] Woke up, no clear AR. Defaulting to Node 1 as sender target.\n", myNodeId);
                    becomeSender(1);
                }
                // Immediately probe for AR's mode after wakeup if a sender, and actively wait for response
                if (myRole == ROLE_SENDER) { // Ensure this block only applies to senders
                    Serial.printf("[Node %d] ULP woke and became sender. Immediately sending PROBE to get current AR mode.\n", myNodeId);
                    messageToSend.type = MSG_TYPE_PROBE;
                    messageToSend.senderId = myNodeId;
                    messageToSend.wakeCounter = 0;
                    esp_now_send(broadcastMac, (uint8_t *)&messageToSend, sizeof(messageToSend));
                    // Active wait for AR mode sync in setup before proceeding to loop()
                    Serial.printf("[Node %d] Waiting for AR mode sync in setup() (%dms max). Initial mode: %s\n",
                                  myNodeId, SENDER_MODE_SYNC_WAIT_MS,
                                  (currentOperatingMode == STANDBY_MODE ? "STANDBY_MODE" : "DATA_COLLECTION_MODE"));
                    unsigned long startWaitTime = millis();
                    while (millis() - startWaitTime < SENDER_MODE_SYNC_WAIT_MS) {
                        // Yield to allow ESP-NOW callbacks to process received messages
                        delay(10); // Small delay to allow other tasks (like ESP-NOW receive) to run
                        // If mode has changed by onDataRecv callback, break out of this wait
                        if (currentOperatingMode == DATA_COLLECTION_MODE) {
                            Serial.printf("[Node %d] Mode updated during setup() wait to DATA_COLLECTION_MODE. Breaking wait.\n", myNodeId);
                            break; // Exit the while loop early
                        }
                    }
                    Serial.printf("[Node %d] Exited AR mode sync wait in setup(). Final mode: %s\n",
                                  myNodeId, (currentOperatingMode == STANDBY_MODE ? "STANDBY_MODE" : "DATA_COLLECTION_MODE"));
                }
            }
        } else { // DATA_COLLECTION_MODE
            Serial.printf("[Node %d] Woke up in DATA_COLLECTION_MODE (shouldn't deep sleep usually). Re-evaluating...\n", myNodeId);
            // In data collection mode, ULP should ideally be off.
            // If we woke from ULP, it implies a bug or a forced ULP enable.
            // For robustness, we proceed as if we just booted into DATA_COLLECTION_MODE.
            if (myNodeId == 1) becomeActiveReceiver();
            else becomeSender(targetNodeId);
        }
    } else {
        // If not a deep sleep wakeup (e.g., first boot or reset),
        // initialize ULP variables and assign initial roles based on NVS mode.
        Serial.println("First boot or reset. Initializing ULP and roles based on NVS mode.");
        ulpJustWoke = false; // Ensure flag is false on first boot
        lastLocalSensorReadTime = millis(); // Initialize for AR local sensor check
        // Initialize nodeModeSynced array for AR (Node 1)
        if (myNodeId == 1) {
            for (int i = 0; i <= MAX_NODES; ++i) {
                nodeModeSynced[i] = false;
            }
            Serial.println("[Node 1 AR] Initializing nodeModeSynced flags for all nodes on first boot.");
        }
        // --- DEBUGGING: Read raw ADC values directly on first boot ---
        Serial.println("--- Raw ADC Readings on First Boot (Main Core) ---");
        int rawRainADC = analogRead(PIN_ADC_RAIN);
        int rawSoilADC = analogRead(PIN_ADC_SOIL);
        int rawReadyADC = analogRead(PIN_ADC_READY);
        Serial.printf("Raw Rain ADC (PIN_ADC_RAIN %d): %d\n", PIN_ADC_RAIN, rawRainADC);
        Serial.printf("Raw Soil ADC (PIN_ADC_SOIL %d): %d\n", PIN_ADC_SOIL, rawSoilADC);
        Serial.printf("Raw Ready ADC (PIN_ADC_READY %d): %d\n", PIN_ADC_READY, rawReadyADC);
        Serial.println("------------------------------------");
        // --- END DEBUGGING ---
        // Initialize ULP variables based on current raw readings or arbitrary defaults
        uint16_t initial_rain_val = rawRainADC > 0 ? rawRainADC : 1000;
        uint16_t initial_soil_val = rawSoilADC > 0 ? rawSoilADC : 2000;
        ulp_vars.rain_lower.val = (initial_rain_val > SENSOR_WINDOW_MARGIN) ? initial_rain_val - SENSOR_WINDOW_MARGIN : 0;
        ulp_vars.rain_upper.val = initial_rain_val + SENSOR_WINDOW_MARGIN;
        ulp_vars.last_rain_val.val = initial_rain_val; // Initialize last read with current value
        ulp_vars.soil_lower.val = (initial_soil_val > SENSOR_WINDOW_MARGIN) ? initial_soil_val - SENSOR_WINDOW_MARGIN : 0;
        ulp_vars.soil_upper.val = initial_soil_val + SENSOR_WINDOW_MARGIN; // Corrected initialization
        ulp_vars.last_soil_val.val = initial_soil_val; // Initialize last read with current value
        ulp_vars.triggered_count.val = 0; // Reset ULP wakeup counter
        ulp_vars.last_ready_val.val = rawReadyADC; // Initialize with current reading
        Serial.println("--- Initialized ULP Variables (Main Core Assignment) ---");
        Serial.printf("ulp_vars.last_rain_val.val: %d\n", ulp_vars.last_rain_val.val);
        Serial.printf("ulp_vars.last_soil_val.val: %d\n", ulp_vars.last_soil_val.val);
        Serial.printf("ulp_vars.last_ready_val.val: %d\n", ulp_vars.last_ready_val.val);
        Serial.printf("ulp_vars.rain_lower.val: %d\n", ulp_vars.rain_lower.val);
        Serial.printf("ulp_vars.rain_upper.val: %d\n", ulp_vars.rain_upper.val);
        Serial.printf("ulp_vars.soil_lower.val: %d\n", ulp_vars.soil_lower.val);
        Serial.printf("ulp_vars.soil_upper.val: %d\n", ulp_vars.soil_upper.val);
        Serial.println("--------------------------------------------------");
        sendCounter = 0;
        failedLastSend = false;
        consecutiveSendFailures = 0;
        lastFailedTargetId = 0;
        probingForFailover = false;
        
        if (myNodeId == 1) {
            becomeActiveReceiver();
        } else {
            becomeSender(targetNodeId);
        }
        
        if (currentOperatingMode == STANDBY_MODE) {
            Serial.printf("[Node %d] Calling ulp_init()...\n", myNodeId);
            ulp_init();
            Serial.printf("[Node %d] ulp_init() completed.\n", myNodeId);
        } else {
            Serial.printf("[Node %d] Not calling ulp_init() as in DATA_COLLECTION_MODE.\n", myNodeId);
        }
    }
    Serial.printf("[Node %d] My role is: %s, Operating Mode: %s (after setup)\n",
                      myNodeId,
                      myRole == ROLE_RECEIVER ? "RECEIVER" : "SENDER",
                      currentOperatingMode == STANDBY_MODE ? "STANDBY_MODE" : "DATA_COLLECTION_MODE");
    delay(100); // Give time for serial buffer to flush after setup prints
}

// --- Arduino Loop Function ---
void loop() {
    // Handle push button for mode switching (only on Active Receiver nodes)
    if (myRole == ROLE_RECEIVER && digitalRead(PUSH_BUTTON_PIN) == LOW && (millis() - lastButtonPressTime > BUTTON_DEBOUNCE_DELAY)) {
        lastButtonPressTime = millis(); // Update button press time for debounce
        operating_mode_t newMode = (currentOperatingMode == STANDBY_MODE) ? DATA_COLLECTION_MODE : STANDBY_MODE;
        
        Serial.printf("[Node %d] Push button pressed. Toggling mode to %s.\n",
                              myNodeId, (newMode == STANDBY_MODE ? "STANDBY_MODE" : "DATA_COLLECTION_MODE"));
        currentOperatingMode = newMode;
        writeModeToNVS(currentOperatingMode); // Persist the new mode
        // Broadcast MSG_TYPE_MODE_CHANGE multiple times for robustness
        messageToSend.type = MSG_TYPE_MODE_CHANGE;
        messageToSend.senderId = myNodeId;
        messageToSend.newMode = currentOperatingMode;
        for (int i = 0; i < 5; ++i) { // Send 5 times
            esp_now_send(broadcastMac, (uint8_t *)&messageToSend, sizeof(messageToSend));
            delay(10); // Small delay between broadcasts
        }
        Serial.println("Mode change broadcasted.");
        // Immediately reconfigure ULP based on the new mode on this node
        if (currentOperatingMode == STANDBY_MODE) {
            ulp_init(); // Enable ULP for standby
            Serial.println("ULP re-initialized for STANDBY_MODE on this node.");
        } else {
            hulp_ulp_end(); // Changed from hulp_ulp_stop() to hulp_ulp_end()
            Serial.println("ULP stopped for DATA_COLLECTION_MODE on this node.");
        }
        // When the AR's mode changes (e.g., via button), reset sync flags for other nodes
        // so they will receive an ANNOUNCE_AR ACK the next time they send data/probe.
        for (int i = 0; i <= MAX_NODES; ++i) {
            nodeModeSynced[i] = false;
        }
        Serial.println("[Node 1 AR] Mode changed, resetting nodeModeSynced flags for all nodes.");
        // Also, send an initial broadcast to alert any awake nodes immediately.
        messageToSend.type = MSG_TYPE_ANNOUNCE_AR;
        messageToSend.senderId = myNodeId;
        messageToSend.newMode = currentOperatingMode;
        esp_now_send(broadcastMac, (uint8_t *)&messageToSend, sizeof(messageToSend));
        Serial.println("[Node 1 AR] Sent ANNOUNCE_AR broadcast after mode change.");
    }

    if (currentOperatingMode == STANDBY_MODE) {
        // --- STANDBY_MODE LOGIC (existing, power-efficient, deep sleep enabled) ---
        if (myRole == ROLE_SENDER) {
            bool shouldSend = false;
            Serial.printf("[Node %d Sender Loop - STANDBY_MODE] ulpJustWoke: %s\n", myNodeId, ulpJustWoke ? "true" : "false"); // NEW DEBUG
            // Condition 1: ULP just woke up the main core due to a sensor change. (Prioritized)
            if (ulpJustWoke) {
                Serial.println("[Sender - STANDBY_MODE] ULP just woke up main core. Sending data.");
                shouldSend = true;
            }
            // Condition 2: CH32 Data Ready signal is active (high).
            // Note: This is a digital read of the same pin used for ULP's analog read.
            // Ensure the CH32 keeps this pin high for a sufficient duration after new data is ready.
            else if (digitalRead(PIN_ADC_READY) == HIGH) {
                Serial.println("[Sender - STANDBY_MODE] CH32 Data Ready detected.");
                shouldSend = true;
            }
            // Condition 3: Last send failed and it's time to retry.
            // This ensures that `consecutiveSendFailures` can increment even if
            // the data ready pin doesn't go high again, leading to eventual failover.
            else if (failedLastSend && (millis() - lastSendAttemptTime >= RETRY_DELAY_MS)) {
                Serial.println("[Sender - STANDBY_MODE] Retrying failed send due to previous failure.");
                shouldSend = true;
            }
            else {
                Serial.println("[Sender - STANDBY_MODE] Not sending: No new CH32 data, no failed retry pending, and not a fresh ULP wake.");
            }
            if (shouldSend) {
                lastSendAttemptTime = millis(); // Update the timestamp of the last send attempt
                sendCounter++; // Increment the message send counter
                // Prepare the ESP-NOW message structure.
                messageToSend.type = MSG_TYPE_DATA;
                messageToSend.senderId = myNodeId;
                messageToSend.wakeCounter = ulp_vars.triggered_count.val; // Use ULP's trigger count for wakeCounter
                // Fill dummy payload (if any, otherwise remove or set to 0)
                memset(messageToSend.dummy_payload, 0xAA, sizeof(messageToSend.dummy_payload));
                // *** INTEGRATE PWM DECODING HERE ***
                // Call the PWM decoding function to get the latest tilt and vibration values.
                long current_tilt_centi_deg;
                long current_vibration_milli_g;
                decodePwmSignals(current_tilt_centi_deg, current_vibration_milli_g);
                messageToSend.tilt_centi_deg = current_tilt_centi_deg;
                messageToSend.vibration_milli_g = current_vibration_milli_g;
                // Read analog sensor values that have been updated by ULP and stored in RTC memory.
                messageToSend.rain_adc_value = ulp_vars.last_rain_val.val;
                messageToSend.soil_adc_value = ulp_vars.last_soil_val.val;
                Serial.printf("[Node %d] Sending DATA to Node %d...\n", myNodeId, targetNodeId);
                Serial.printf("    Tilt: %ld.%02ld deg, Vib: %ld.%03ld g, Rain: %d, Soil: %d\n",
                                  messageToSend.tilt_centi_deg / 100, abs(messageToSend.tilt_centi_deg % 100),
                                  messageToSend.vibration_milli_g / 1000, abs(messageToSend.vibration_milli_g % 1000),
                                  messageToSend.rain_adc_value, messageToSend.soil_adc_value);
                // Send the data message to the current target node.
                esp_now_send(nodeMacs[targetNodeId - 1], (uint8_t *)&messageToSend, sizeof(messageToSend));
                delay(100); // Small delay after sending to prevent rapid re-sends
                // After sending, pause briefly to allow AR to respond with its current mode
                Serial.printf("[Node %d] Waiting for AR mode sync (%dms max). Current mode: %s\n",
                                  myNodeId, SENDER_MODE_SYNC_WAIT_MS,
                                  (currentOperatingMode == STANDBY_MODE ? "STANDBY_MODE" : "DATA_COLLECTION_MODE"));
                unsigned long startWaitTime = millis();
                while (millis() - startWaitTime < SENDER_MODE_SYNC_WAIT_MS) { // Wait for up to 1 second
                    // Yield to allow ESP-NOW callbacks to process
                    delay(10);
                    // If mode has changed by onDataRecv callback, break out of this wait
                    if (currentOperatingMode == DATA_COLLECTION_MODE) {
                        Serial.printf("[Node %d] Mode updated during post-send wait to DATA_COLLECTION_MODE. Bypassing deep sleep.\n", myNodeId);
                        return; // Immediately re-evaluate loop in new mode
                    }
                }
                Serial.printf("[Node %d] Exited AR mode sync wait. Final mode after wait: %s\n",
                                  myNodeId, (currentOperatingMode == STANDBY_MODE ? "STANDBY_MODE" : "DATA_COLLECTION_MODE"));
            }
            // If the last send failed and we are not already probing for a new AR, start probing.
            if (failedLastSend && !probingForFailover) {
                Serial.printf("[Node %d] Probing for new AR...\n", myNodeId);
                // Send probe messages to all nodes with lower IDs than myself.
                // This is a strategy to find a potential new Active Receiver if the current one is down.
                for (int i = 0; i < myNodeId - 1; ++i) {
                    messageToSend.type = MSG_TYPE_PROBE;
                    messageToSend.senderId = myNodeId;
                    messageToSend.wakeCounter = 0; // Probes do not carry wake counter data
                    esp_now_send(nodeMacs[i], (uint8_t *)&messageToSend, sizeof(messageToSend));
                }
                probingForFailover = true; // Set flag to avoid repeated probing during this phase
            }
            // If the last send was successful and this node is still a sender,
            // enter deep sleep to conserve power.
            if (!failedLastSend && myRole == ROLE_SENDER) {
                // Check if the operating mode has changed (e.g., via onDataRecv callback)
                // This is the last line of defense before deep sleep.
                if (currentOperatingMode == DATA_COLLECTION_MODE) {
                    Serial.printf("[Node %d] Last check: Mode is DATA_COLLECTION_MODE. Bypassing deep sleep. \n", myNodeId);
                    return; // Exit current loop iteration to re-evaluate operating mode
                }
                Serial.printf("[Node %d] Deep sleep now. Current mode: %s\n", myNodeId,
                                  (currentOperatingMode == STANDBY_MODE ? "STANDBY_MODE" : "DATA_COLLECTION_MODE"));
                esp_sleep_enable_ulp_wakeup(); // Enable ULP to wake up from deep sleep
                delay(50); // Give time for serial buffer to clear before sleeping
                esp_deep_sleep_start(); // Enter deep sleep
            } else if (failedLastSend) {
                // If send failed, stay awake to retry or continue probing.
                Serial.printf("[Node %d] Send failed. Staying awake to retry or probe.\n", myNodeId);
            }
        } else if (myRole == ROLE_RECEIVER) {
            // AR TCP server client handler
            server.handleClient(); // Process incoming web requests

            // If this node is an Active Receiver, it periodically probes higher-priority
            // (lower ID) nodes to check if they have recovered and are now available.
            // This supports the failback mechanism (AR Takeback).
            if (millis() - lastProbeTime >= PROBE_INTERVAL) {
                lastProbeTime = millis(); // Update last probe timestamp
                Serial.printf("[Node %d AR] Periodically probing higher priority nodes for AR Takeback...\n", myNodeId);
                for (int i = 0; i < myNodeId - 1; ++i) { // Loop through nodes with lower IDs (higher priority)
                    messageToSend.type = MSG_TYPE_PROBE;
                    messageToSend.senderId = myNodeId;
                    messageToSend.wakeCounter = 0; // Probes do not carry wake counter data
                    esp_now_send(nodeMacs[i], (uint8_t *)&messageToSend, sizeof(messageToSend));
                }
            }
            // Check local sensors for changes and print if significant, and update for TCP server
            if (millis() - lastLocalSensorReadTime >= LOCAL_SENSOR_CHECK_INTERVAL) {
                lastLocalSensorReadTime = millis(); // Update last local sensor read time
                int currentRainADC = analogRead(PIN_ADC_RAIN);
                int currentSoilADC = analogRead(PIN_ADC_SOIL);
                int currentReadyADC = analogRead(PIN_ADC_READY);

                long current_tilt_centi_deg;
                long current_vibration_milli_g;
                decodePwmSignals(current_tilt_centi_deg, current_vibration_milli_g);

                // Determine if a significant change occurred based on the defined conditions
                bool rainChanged = abs(currentRainADC - ulp_vars.last_rain_val.val) > SENSOR_WINDOW_MARGIN;
                bool soilChanged = abs(currentSoilADC - ulp_vars.last_soil_val.val) > SENSOR_WINDOW_MARGIN;
                // For DATA_READY, we check if it's currently above the threshold
                bool readyChanged = currentReadyADC >= DATA_READY_THRESHOLD;
                if (rainChanged || soilChanged || readyChanged) {
                    Serial.printf("[Node %d LOCAL SENSOR CHANGE - STANDBY_MODE] Detected change! Printing local data:\n", myNodeId);
                    Serial.printf("    Rain ADC: %d (Last: %d, Lower: %d, Upper: %d)\n", currentRainADC, ulp_vars.last_rain_val.val, ulp_vars.rain_lower.val, ulp_vars.rain_upper.val);
                    Serial.printf("    Soil ADC: %d (Last: %d, Lower: %d, Upper: %d)\n", currentSoilADC, ulp_vars.last_soil_val.val, ulp_vars.soil_lower.val, ulp_vars.soil_upper.val);
                    Serial.printf("    Ready ADC: %d (Threshold: %d, IsReady: %s)\n", currentReadyADC, DATA_READY_THRESHOLD, readyChanged ? "YES" : "NO");
                    Serial.printf("    Tilt: %ld.%02ld deg, Vib: %ld.%03ld g\n",
                                  current_tilt_centi_deg / 100, abs(current_tilt_centi_deg % 100),
                                  current_vibration_milli_g / 1000, abs(current_vibration_milli_g % 1000));
                    // Update stored ULP variables and thresholds for future comparisons
                    ulp_vars.last_rain_val.val = currentRainADC;
                    ulp_vars.rain_lower.val = (currentRainADC > SENSOR_WINDOW_MARGIN) ? currentRainADC - SENSOR_WINDOW_MARGIN : 0;
                    ulp_vars.rain_upper.val = currentRainADC + SENSOR_WINDOW_MARGIN;
                    ulp_vars.last_soil_val.val = currentSoilADC;
                    ulp_vars.soil_lower.val = (currentSoilADC > SENSOR_WINDOW_MARGIN) ? currentSoilADC - SENSOR_WINDOW_MARGIN : 0;
                    ulp_vars.soil_upper.val = currentSoilADC + SENSOR_WINDOW_MARGIN;
                    ulp_vars.last_ready_val.val = currentReadyADC; // Update last ready value
                }
                // Update latestNodeData for my own local readings for TCP server
                latestNodeData[myNodeId].rain_adc_value = currentRainADC;
                latestNodeData[myNodeId].soil_adc_value = currentSoilADC;
                latestNodeData[myNodeId].vibration_milli_g = current_vibration_milli_g;
                latestNodeData[myNodeId].tilt_centi_deg = current_tilt_centi_deg;
                latestNodeData[myNodeId].lastUpdated = millis();
                latestNodeData[myNodeId].initialized = true; // Ensure it's marked as initialized
            }
            delay(20); // Keep receiver node alive and responsive by yielding CPU periodically
        }
    } else if (currentOperatingMode == DATA_COLLECTION_MODE) {
        // --- DATA_COLLECTION_MODE LOGIC (continuous, high-rate) ---
        // In this mode, deep sleep is bypassed. Nodes continuously acquire and send data.
        // Receivers continuously receive and log data, and also monitor their own local sensors.
        Serial.printf("[Node %d Loop - DATA_COLLECTION_MODE] My role is %s\n", myNodeId, myRole == ROLE_RECEIVER ? "RECEIVER" : "SENDER");
        long current_tilt_centi_deg;
        long current_vibration_milli_g;
        // All nodes (senders and receivers) continuously read their own sensors
        // and decode PWM signals.
        decodePwmSignals(current_tilt_centi_deg, current_vibration_milli_g);
        int currentRainADC = analogRead(PIN_ADC_RAIN);
        int currentSoilADC = analogRead(PIN_ADC_SOIL);
        int currentReadyADC = analogRead(PIN_ADC_READY);
        // Update RTC variables for consistency, even if ULP is not active
        ulp_vars.last_rain_val.val = currentRainADC;
        ulp_vars.rain_lower.val = (currentRainADC > SENSOR_WINDOW_MARGIN) ? currentRainADC - SENSOR_WINDOW_MARGIN : 0;
        ulp_vars.rain_upper.val = currentRainADC + SENSOR_WINDOW_MARGIN;
        ulp_vars.last_soil_val.val = currentSoilADC;
        ulp_vars.soil_lower.val = (currentSoilADC > SENSOR_WINDOW_MARGIN) ? currentSoilADC - SENSOR_WINDOW_MARGIN : 0;
        ulp_vars.soil_upper.val = currentSoilADC + SENSOR_WINDOW_MARGIN;
        ulp_vars.last_ready_val.val = currentReadyADC;

        // Update latestNodeData for my own local readings for TCP server
        latestNodeData[myNodeId].rain_adc_value = currentRainADC;
        latestNodeData[myNodeId].soil_adc_value = currentSoilADC;
        latestNodeData[myNodeId].vibration_milli_g = current_vibration_milli_g;
        latestNodeData[myNodeId].tilt_centi_deg = current_tilt_centi_deg;
        latestNodeData[myNodeId].lastUpdated = millis();
        latestNodeData[myNodeId].initialized = true;

        if (myRole == ROLE_SENDER) {
            // Senders in DATA_COLLECTION_MODE continuously send their data
            lastSendAttemptTime = millis(); // Update the timestamp of the last send attempt
            sendCounter++; // Increment the message send counter
            messageToSend.type = MSG_TYPE_DATA;
            messageToSend.senderId = myNodeId;
            messageToSend.wakeCounter = 0; // ULP is not active in this mode, so wakeCounter is 0
            memset(messageToSend.dummy_payload, 0xAA, sizeof(messageToSend.dummy_payload));
            messageToSend.tilt_centi_deg = current_tilt_centi_deg;
            messageToSend.vibration_milli_g = current_vibration_milli_g;
            messageToSend.rain_adc_value = currentRainADC;
            messageToSend.soil_adc_value = currentSoilADC;
            Serial.printf("[Node %d] Sending DATA to Node %d (Continuous Mode)...\n", myNodeId, targetNodeId);
            Serial.printf("    Tilt: %ld.%02ld deg, Vib: %ld.%03ld g, Rain: %d, Soil: %d\n",
                                  messageToSend.tilt_centi_deg / 100, abs(messageToSend.tilt_centi_deg % 100),
                                  messageToSend.vibration_milli_g / 1000, abs(messageToSend.vibration_milli_g % 1000),
                                  messageToSend.rain_adc_value, messageToSend.soil_adc_value);
            
            esp_now_send(nodeMacs[targetNodeId - 1], (uint8_t *)&messageToSend, sizeof(messageToSend));
            delay(100); // Small delay to avoid flooding ESP-NOW
            // Failover logic remains the same, but no deep sleep.
            // The onDataSent callback will handle consecutiveSendFailures and call findNewTargetAndFailover().
            if (failedLastSend && !probingForFailover) {
                Serial.printf("[Node %d] Probing for new AR (Continuous Mode)...\n", myNodeId);
                for (int i = 0; i < myNodeId - 1; ++i) {
                    messageToSend.type = MSG_TYPE_PROBE;
                    messageToSend.senderId = myNodeId;
                    messageToSend.wakeCounter = 0;
                    esp_now_send(nodeMacs[i], (uint8_t *)&messageToSend, sizeof(messageToSend));
                }
                probingForFailover = true;
            }
        } else if (myRole == ROLE_RECEIVER) {
            // AR TCP server client handler
            server.handleClient(); // Process incoming web requests
            
            // Receivers in DATA_COLLECTION_MODE log their own sensor data
            // in addition to receiving from others.
            // The periodic broadcast logic has been removed from here.
            // If this node is an Active Receiver, it periodically probes higher-priority
            // (lower ID) nodes to check if they have recovered and are now available.
            // This supports the failback mechanism (AR Takeback).
            if (millis() - lastProbeTime >= PROBE_INTERVAL) {
                lastProbeTime = millis(); // Update last probe timestamp
                Serial.printf("[Node %d AR] Periodically probing higher priority nodes for AR Takeback (Continuous Mode)...\n", myNodeId);
                for (int i = 0; i < myNodeId - 1; ++i) { // Loop through nodes with lower IDs (higher priority)
                    messageToSend.type = MSG_TYPE_PROBE;
                    messageToSend.senderId = myNodeId;
                    messageToSend.wakeCounter = 0; // Probes do not carry wake counter data
                    esp_now_send(nodeMacs[i], (uint8_t *)&messageToSend, sizeof(messageToSend));
                }
            }
            if (millis() - lastLocalSensorReadTime >= LOCAL_SENSOR_CHECK_INTERVAL) {
                lastLocalSensorReadTime = millis();
                Serial.printf("[Node %d LOCAL SENSOR READ - DATA_COLLECTION_MODE] Own data: Tilt: %ld.%02ld deg, Vib: %ld.%03ld g, Rain: %d, Soil: %d\n",
                                  myNodeId,
                                  current_tilt_centi_deg / 100, abs(current_tilt_centi_deg % 100),
                                  current_vibration_milli_g / 1000, abs(current_vibration_milli_g % 1000),
                                  currentRainADC, currentSoilADC);
            }
            delay(20); // Keep receiver node alive and responsive
        }
    }
}
