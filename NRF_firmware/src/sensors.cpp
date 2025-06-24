#include "sensors.h"

// Global sensor data instance
SensorData sensorData;

// Implementation of toJSON method
String SensorData::toJSON() const {
  String json = "{";
  json += "\"timestamp\":" + String(timestamp) + ",";
  json += "\"sensors\":{";
  json += "\"light\":{\"lux\":" + String(lux) + "},";
  json += "\"battery\":{\"voltage\":" + String(battery_voltage, 2) + "},";
  json += "\"environment\":{";
  json += "\"temperature\":" + String(case_temperature, 1) + ",";
  json += "\"humidity\":" + String(case_humidity, 1) + ",";
  json += "\"pressure\":" + String(pressure, 2) + ",";
  json += "\"pressure_sensor_temp\":" + String(pressure_sensor_temp, 2);
  json += "},";
  json += "\"accelerometers\":{";
  json += "\"acc1\":{\"x\":" + String(acc1_x) + ",\"y\":" + String(acc1_y) + ",\"z\":" + String(acc1_z) + "},";
  json += "\"acc2\":{\"x\":" + String(acc2_x) + ",\"y\":" + String(acc2_y) + ",\"z\":" + String(acc2_z) + "}";
  json += "},";
  json += "\"gps\":{";
  json += "\"latitude\":" + String(gps_latitude, 8) + ",";
  json += "\"longitude\":" + String(gps_longitude, 8) + ",";
  json += "\"speed\":" + String(gps_speed) + ",";
  json += "\"altitude\":" + String(gps_altitude) + ",";
  json += "\"visible_satellites\":" + String(gps_visible_satellites) + ",";
  json += "\"used_satellites\":" + String(gps_used_satellites) + ",";
  json += "\"accuracy\":" + String(gps_accuracy);
  json += "},";
  json += "\"system\":{";
  json += "\"wakeup_reason\":" + String(wakeup_reason) + ",";
  json += "\"watchdog_counter\":" + String(watchdog_counter);
  json += "}";
  json += "}";
  json += "}";
  return json;
} 