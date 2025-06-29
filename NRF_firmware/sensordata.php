<?php
// Sensor Data Receiver - POST JSON Version
// This script receives sensor data via POST with JSON payload and returns structured JSON response

header('Content-Type: application/json');
header('Access-Control-Allow-Origin: *');
header('Access-Control-Allow-Methods: POST');
header('Access-Control-Allow-Headers: Content-Type');

// Get JSON input
$json_input = file_get_contents('php://input');
$data = json_decode($json_input, true);

// Create compact response array with Unix timestamp
$response = array(
    'status' => 'success',
    'ts' => time(),
    'processed' => false
);

// Check if JSON was parsed successfully
if ($data === null) {
    $response['status'] = 'error';
    $response['error'] = 'Invalid JSON';
    http_response_code(400);
} else {
    // Parse and expand short field names to longer descriptive names
    $processed_data = array();
    
    // Map short field names to longer descriptive names
    $field_mapping = array(
        // Timestamp and system data
        'ts' => 'timestamp',
        'wkr' => 'wakeup_reason',
        'wdc' => 'watchdog_counter',
        
        // Sensor data
        'lx' => 'light_lux',
        'bat' => 'battery_voltage',
        'tmp' => 'temperature_celsius',
        'hum' => 'humidity_percent',
        'prs' => 'pressure_hpa',
        'pt' => 'pressure_sensor_temperature',
        
        // Accelerometer data
        'a1x' => 'accelerometer1_x',
        'a1y' => 'accelerometer1_y',
        'a1z' => 'accelerometer1_z',
        'a2x' => 'accelerometer2_x',
        'a2y' => 'accelerometer2_y',
        'a2z' => 'accelerometer2_z',
        
        // GPS data
        'glt' => 'gps_latitude',
        'gln' => 'gps_longitude',
        'gsp' => 'gps_speed',
        'gal' => 'gps_altitude',
        'gsv' => 'gps_visible_satellites',
        'gsu' => 'gps_used_satellites',
        'gac' => 'gps_accuracy',
        
        // Modem data
        'mid' => 'modem_id',
        'mt' => 'modem_temperature',
        'sid' => 'sim_card_id',
        'nn' => 'network_name',
        'nid' => 'network_id',
        'lt' => 'link_type',
        'sq' => 'signal_quality',
        'ss' => 'signal_strength',
        'nrs' => 'network_registration_status',
        'nb' => 'network_band',
        'nc' => 'network_channel',
        'noc' => 'network_operator_code',
        
        // Scan data
        'wifi' => 'wifi_scan_data',
        'ble' => 'ble_scan_data'
    );
    
    // Process each field in the received data
    foreach ($data as $key => $value) {
        if (isset($field_mapping[$key])) {
            // Use the longer descriptive name
            $processed_data[$field_mapping[$key]] = $value;
        } else {
            // Keep original name if no mapping exists
            $processed_data[$key] = $value;
        }
    }
    
    // Log the processed data to a JSON file (minimal logging)
    $log_filename = 'sensor_data_' . date('Y-m-d') . '.json';
    $log_entry = array(
        'ts' => time(),
        'data' => $processed_data
    );
    
    // Append to daily log file
    file_put_contents($log_filename, json_encode($log_entry) . "\n", FILE_APPEND | LOCK_EX);
    
    // Update response with minimal info
    $response['processed'] = true;
    $response['fields'] = count($processed_data);
    $response['log'] = $log_filename;
}

// Output compact response as JSON
echo json_encode($response);
?> 