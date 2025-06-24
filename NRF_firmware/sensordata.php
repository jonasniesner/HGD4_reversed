<?php
// Sensor Data Receiver - POST JSON Version
// This script receives sensor data via POST with JSON payload

header('Content-Type: application/json');

// Get JSON input
$json_input = file_get_contents('php://input');
$data = json_decode($json_input, true);

// Create response array
$response = array(
    'status' => 'success',
    'message' => 'Data received successfully',
    'timestamp' => time(),
    'received_data' => $data
);

// Check if JSON was parsed successfully
if ($data === null) {
    $response['status'] = 'error';
    $response['message'] = 'Invalid JSON data received';
    $response['raw_input'] = $json_input;
    http_response_code(400);
} else {
    // Log the received data to a JSON file
    $log_filename = 'sensor_data_' . date('Y-m-d') . '.json';
    $log_entry = array(
        'timestamp' => date('Y-m-d H:i:s'),
        'data' => $data
    );
    
    // Append to daily log file
    file_put_contents($log_filename, json_encode($log_entry, JSON_PRETTY_PRINT) . "\n", FILE_APPEND | LOCK_EX);
    
    // Also log to a general log file
    $general_log = 'sensor_data_all.json';
    file_put_contents($general_log, json_encode($log_entry, JSON_PRETTY_PRINT) . "\n", FILE_APPEND | LOCK_EX);
    
    $response['logged_to'] = $log_filename;
}

// Output response as JSON
echo json_encode($response, JSON_PRETTY_PRINT);
?> 