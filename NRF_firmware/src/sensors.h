#include <Arduino.h>

// Sensor data structure to hold all readings
struct SensorData {
  // Timestamp
  unsigned long timestamp;
  
  // Light sensor
  float lux;
  
  // Battery
  float battery_voltage;
  
  // Temperature and humidity
  float case_temperature;
  float case_humidity;
  
  // Pressure sensor
  float pressure;
  float pressure_sensor_temp;
  
  // Accelerometers
  float acc1_x;
  float acc1_y;
  float acc1_z;
  float acc2_x;
  float acc2_y;
  float acc2_z;
  
  // System info
  int32_t wakeup_reason;
  int32_t watchdog_counter;
  
  // GPS data (when available)
  float gps_latitude;
  float gps_longitude;
  float gps_speed;
  float gps_altitude;
  int gps_visible_satellites;
  int gps_used_satellites;
  float gps_accuracy;
  
  // Modem information
  String modem_id;
  float modem_temperature;
  String sim_card_id;
  String network_name;
  String network_id;
  
  // Network connection information
  String link_type;        // LTE, 2G, 3G, etc.
  int signal_quality;      // Signal quality (0-31, where 31 is best)
  int signal_strength;     // Signal strength in dBm
  int network_registration_status; // Network registration status
  String network_band;     // Network band (e.g., "LTE BAND 20")
  int network_channel;     // Network channel number
  String network_operator_code; // Network operator code
  
  // WiFi scan data
  String wifi_scan_data;   // JSON string of WiFi scan results
  
  // BLE scan data
  String ble_scan_data;    // JSON string of BLE beacon scan results
  
  // Constructor to initialize all values
  SensorData() {
    timestamp = 0;
    lux = -1;
    battery_voltage = 0.0;
    case_temperature = 0.0;
    case_humidity = 0.0;
    pressure = 0.0;
    pressure_sensor_temp = 0.0;
    acc1_x = 0.0;
    acc1_y = 0.0;
    acc1_z = 0.0;
    acc2_x = 0.0;
    acc2_y = 0.0;
    acc2_z = 0.0;
    wakeup_reason = 0;
    watchdog_counter = 0;
    gps_latitude = 0.0;
    gps_longitude = 0.0;
    gps_speed = 0.0;
    gps_altitude = 0.0;
    gps_visible_satellites = 0;
    gps_used_satellites = 0;
    gps_accuracy = 0.0;
    modem_id = "";
    modem_temperature = 0.0;
    sim_card_id = "";
    network_name = "";
    network_id = "";
    link_type = "";
    signal_quality = 0;
    signal_strength = 0;
    network_registration_status = 0;
    network_band = "";
    network_channel = 0;
    network_operator_code = "";
    wifi_scan_data = "";
    ble_scan_data = "";
  }
  
  // Method to check if data is fresh (within last 5 minutes)
  bool isFresh() const {
    return (millis() - timestamp) < 300000; // 5 minutes
  }
  
  // Method to get JSON representation for HTTP transmission
  String toJSON() const;
};

// Global sensor data instance
extern SensorData sensorData;

// Data collection functions
void collectAllSensorData();