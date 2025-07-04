#include "main.h"

// Global configuration - will be loaded from modem storage
ModemConfig deviceConfig;

//set if config should be saved to modem
bool saveConfig = true;

void setup() {
  rtt.trimDownBufferFull();
  rtt.println("|x-------XStarting setupX-------x|");
  gpioinit(); 
  sensorData.wakeup_reason = NRF_POWER->RESETREAS;
  delay(200);
  initializeModem();
  sensorpwr(true);
  blinkLED(1, 200, 200, true);
  //checkModemStorageSpace();
  LoadDeviceConfig();
  if (deviceConfig.ble_scan_enabled) {
    initializeBLE();
  }
}

void loop() {
  lastWaitCheck = millis();
  rtt.println("|---------Starting loop---------|");
  rtt.println("Collecting sensor data");
  collectAllSensorData();
  rtt.print("WD counter ");
  rtt.print(sensorData.watchdog_counter);
  rtt.print(" Wakeup reason: ");
  rtt.println(sensorData.wakeup_reason);
  displaySensorDataSummary();
  rtt.println("Scanning for beacons");
  scanforbeacons();
  rtt.println("Scanning for WiFi networks");
  ScanForWiFiNetworks();
  rtt.println("Managing modem lifecycle");
  manageModemLifecycle();
  rtt.println("Waiting for next transmission");
  rtt.println("Loop cycle completed.");
  globalMotionDetected = false;
  readAccelerometers(lastAcc1X, lastAcc1Y, lastAcc1Z, lastAcc2X, lastAcc2Y, lastAcc2Z);
  while (!waitForNextTransmission(deviceConfig.transmission_interval)) {
    performBackgroundTasks();
  }
}

void ble_scan_callback(ble_gap_evt_adv_report_t* report){
  if (!bleScanning) {
    Bluefruit.Scanner.resume();
    return;
  }
  
  // Check if we've reached the maximum number of beacons
  if (bleBeaconsFound >= deviceConfig.ble_scan_max_beacons) {
    Bluefruit.Scanner.resume();
    return;
  }
  
  // Check if scan duration has expired
  if (millis() - bleScanStartTime > deviceConfig.ble_scan_duration) {
    Bluefruit.Scanner.resume();
    return;
  }
  
  JsonObject beacon = bleBeacons.createNestedObject();
  if (beacon.isNull()) {
    rtt.println("BLE: Failed to create beacon object");
    Bluefruit.Scanner.resume();
    return;
  }
  
  // Add timestamp
  beacon["ts"] = millis();
  
  // Add RSSI
  beacon["rssi"] = report->rssi;
  
  // Add MAC address (little endian, so reverse for display)
  char macStr[18];
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
          report->peer_addr.addr[5], report->peer_addr.addr[4], 
          report->peer_addr.addr[3], report->peer_addr.addr[2], 
          report->peer_addr.addr[1], report->peer_addr.addr[0]);
  beacon["mac"] = String(macStr);
  
  // Add advertisement type
  beacon["ty"] = report->type.scan_response ? "sr" : "adv";
  beacon["con"] = report->type.connectable;
  beacon["dir"] = report->type.directed;
  
  // Add payload length
  beacon["plen"] = report->data.len;
  
  // Parse device name if available
  uint8_t buffer[32];
  memset(buffer, 0, sizeof(buffer));
  
  // Try to get complete local name first
  if(Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, buffer, sizeof(buffer))) {
    beacon["name"] = String((char*)buffer);
  } else if(Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, buffer, sizeof(buffer))) {
    beacon["name"] = String((char*)buffer);
  }
  
  // Parse TX power if available
  memset(buffer, 0, sizeof(buffer));
  if (Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_TX_POWER_LEVEL, buffer, sizeof(buffer))) {
    beacon["tx_power"] = (int8_t)buffer[0];
  }
  
  // Parse UUIDs if available
  uint8_t uuidBuffer[32];
  
  // 16-bit UUIDs
  int uuidLen = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE, uuidBuffer, sizeof(uuidBuffer));
  if (uuidLen > 0) {
    JsonArray uuids = beacon.createNestedArray("uuids_16");
    for(int i = 0; i < uuidLen; i += 2) {
      uint16_t uuid16;
      memcpy(&uuid16, uuidBuffer + i, 2);
      uuids.add(String(uuid16, HEX));
    }
  }
  
  // 128-bit UUIDs
  memset(uuidBuffer, 0, sizeof(uuidBuffer));
  uuidLen = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE, uuidBuffer, sizeof(uuidBuffer));
  if (uuidLen > 0) {
    JsonArray uuids = beacon.createNestedArray("uuids_128");
    for(int i = 0; i < uuidLen; i += 16) {
      String uuid128 = "";
      for(int j = 0; j < 16 && (i + j) < uuidLen; j++) {
        if (j > 0 && (j % 2 == 0)) uuid128 += "-";
        uuid128 += String(uuidBuffer[i + j], HEX);
      }
      uuids.add(uuid128);
    }
  }
  
  // Service Data
  memset(uuidBuffer, 0, sizeof(uuidBuffer));
  uuidLen = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_SERVICE_DATA, uuidBuffer, sizeof(uuidBuffer));
  if (uuidLen > 0) {
    JsonArray serviceData = beacon.createNestedArray("service_data");
    for(int i = 0; i < uuidLen; i++) {
      serviceData.add(uuidBuffer[i]);
    }
  }
  
  // Parse manufacturer data if available
  int manuLen = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, buffer, sizeof(buffer));
  if (manuLen > 0) {
    JsonArray manuData = beacon.createNestedArray("md");
    for(int i = 0; i < manuLen; i++) {
      manuData.add(buffer[i]);
    }
  }
  
  // Parse flags if available
  memset(buffer, 0, sizeof(buffer));
  if (Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_FLAGS, buffer, sizeof(buffer))) {
    beacon["flags"] = buffer[0];
  }
  
  // Parse appearance if available
  memset(buffer, 0, sizeof(buffer));
  if (Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_APPEARANCE, buffer, sizeof(buffer))) {
    uint16_t appearance;
    memcpy(&appearance, buffer, 2);
    beacon["appearance"] = appearance;
  }
  
  bleBeaconsFound++;
  //rtt.print("BLE: Beacon "); rtt.print(bleBeaconsFound); rtt.print("/"); rtt.println(deviceConfig.ble_scan_max_beacons);
  
  // For Softdevice v6: after received a report, scanner will be paused
  // We need to call Scanner resume() to continue scanning
  Bluefruit.Scanner.resume();
}

// BLE initialization function - called once in setup
bool initializeBLE() {
  rtt.println("Initializing BLE...");
  
  // Initialize BLE scanning
  Bluefruit.begin(0, 1);
  Bluefruit.setTxPower(4);

  // Set the device name
  Bluefruit.setName("Bluefruit52");

  // Set the LED interval for blinky pattern on BLUE LED
  Bluefruit.setConnLedInterval(250);
  
  rtt.println("BLE initialization completed");
  return true;
}

void scanforbeacons(){
  if (!deviceConfig.ble_scan_enabled) {
    rtt.println("BLE scanning disabled in configuration");
    return;
  }

  rtt.println("Starting BLE beacon scan");
  rtt.print("Max beacons: "); rtt.println(deviceConfig.ble_scan_max_beacons);
  rtt.print("Scan duration: "); rtt.print(deviceConfig.ble_scan_duration); rtt.println(" ms");

  // Initialize JSON document for BLE scan data
  bleScanDoc.clear();
  bleBeacons = bleScanDoc.createNestedArray("beacons");
  if (bleBeacons.isNull()) {
    rtt.println("BLE: Failed to create beacons array");
    return;
  }

  // Initialize scanning variables
  bleScanning = true;
  bleScanStartTime = millis();
  bleBeaconsFound = 0;

  // Start Central Scanning
  Bluefruit.Scanner.setRxCallback(ble_scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.filterRssi(-80);
  Bluefruit.Scanner.setInterval(160, 80);       // in units of 0.625 ms
  Bluefruit.Scanner.useActiveScan(true);        // Request scan response data
  Bluefruit.Scanner.start(0);                   // 0 = Don't stop scanning after n seconds

  rtt.println("BLE: Scanning started...");

  // Wait for scan to complete (either max beacons found or timeout)
  unsigned long scanStartTime = millis();
  while (bleScanning && 
         bleBeaconsFound < deviceConfig.ble_scan_max_beacons && 
         (millis() - scanStartTime) < deviceConfig.ble_scan_duration) {
    delay(100);
    
    // Check if scan duration has expired
    if (millis() - bleScanStartTime > deviceConfig.ble_scan_duration) {
      rtt.println("BLE: Scan duration expired");
      break;
    }
  }

  // Stop scanning
  Bluefruit.Scanner.stop();
  
  bleScanning = false;

  // Add summary information to JSON
  bleScanDoc["count"] = bleBeaconsFound;
  bleScanDoc["scan_duration"] = millis() - bleScanStartTime;
  bleScanDoc["max_beacons"] = deviceConfig.ble_scan_max_beacons;
  bleScanDoc["ts"] = millis();

  // Serialize to string
  String bleScanJson;
  serializeJson(bleScanDoc, bleScanJson);

  // Save to sensor data
  sensorData.ble_scan_data = bleScanJson;

  rtt.print("BLE: Scan completed. Found "); rtt.print(bleBeaconsFound); rtt.println(" beacons");
  rtt.print("BLE: JSON length: "); rtt.println(bleScanJson.length());
}

void blinkLED(int times, int delayon, int delayoff, bool green){
  for(int i = 0; i < times; i++){
    if(green){
      digitalWrite(GREEN_LED,LOW);
      delay(delayon);
      digitalWrite(GREEN_LED,HIGH);
    }
    else{
      digitalWrite(RED_LED,LOW);
      delay(delayon);
      digitalWrite(RED_LED,HIGH);
    }
    delay(delayoff);
  }
}

void displaySensorDataSummary() {
  rtt.print("Sensors: ");
  rtt.print("B="); rtt.print(sensorData.battery_voltage, 2);
  rtt.print("V, L="); rtt.print(sensorData.lux);
  rtt.print("lux, T="); rtt.print(sensorData.case_temperature, 1);
  rtt.print("°C, H="); rtt.print(sensorData.case_humidity, 1);
  rtt.print("%RH, P="); rtt.print(sensorData.pressure, 1);
  rtt.print("hPa, GPS="); rtt.print(sensorData.gps_used_satellites);
  rtt.println(" sats");
}

void resetI2CBus() {
  rtt.println("Resetting I2C bus...");
  Wire.end();
  delay(100);
  Wire.begin();
  delay(100);
  rtt.println("I2C bus reset completed");
}

void sensorpwr(bool onoff){
  if(!onoff){
  rtt.println("Power down sensors");
  Wire.end();
  pinMode(I2C_SCL,INPUT);
  pinMode(I2C_SDA,INPUT);
  pinMode(SPI_CLK,INPUT);
  pinMode(SPI_MISO,INPUT);
  pinMode(SPI_MOSI,INPUT);
  pinMode(ACCL1_CS,INPUT);
  pinMode(ACCL2_CS,INPUT);
  pinMode(SENSOR_PWR,INPUT);
  }
  else{
  rtt.println("Power up sensors");
  pinMode(SENSOR_PWR,OUTPUT);
  digitalWrite(SENSOR_PWR,HIGH);
  delay(100);
  Wire.begin();
  delay(50);
  }
}

void wd_handler() {
  //digitalWrite(GREEN_LED,LOW);
  digitalWrite(DONE,HIGH);
  delayMicroseconds(160);
  digitalWrite(DONE,LOW);
  //digitalWrite(GREEN_LED,HIGH);
  wdcounter++;
}

void collectAllSensorData(){
  sensorData.timestamp = millis();
  sensorData.wakeup_reason = NRF_POWER->RESETREAS;
  sensorData.watchdog_counter = wdcounter;
  sensorData.battery_voltage = readBatteryVoltage();
  sensorData.lux = readLightSensor(0.5, 200);
  if (sensorData.lux < 0) {
    rtt.println("Light sensor failed, resetting I2C bus and retrying...");
    resetI2CBus();
    sensorData.lux = readLightSensor(0.5, 200);
    if (sensorData.lux < 0) {
      rtt.println("Light sensor failed after retry, using default value");
      sensorData.lux = 0.0;
    }
  }
  float temp, hum;
  if (readTemperatureHumidity(temp, hum)) {
    sensorData.case_temperature = temp;
    sensorData.case_humidity = hum;
  }
  float pressure, pressureTemp;
  if (readPressure(pressure, pressureTemp)) {
    sensorData.pressure = pressure;
    sensorData.pressure_sensor_temp = pressureTemp;
  }
  float acc1_x, acc1_y, acc1_z, acc2_x, acc2_y, acc2_z;
  if (readAccelerometers(acc1_x, acc1_y, acc1_z, acc2_x, acc2_y, acc2_z)) {
    sensorData.acc1_x = acc1_x;
    sensorData.acc1_y = acc1_y;
    sensorData.acc1_z = acc1_z;
    sensorData.acc2_x = acc2_x;
    sensorData.acc2_y = acc2_y;
    sensorData.acc2_z = acc2_z;
  }
}

void powerdownmodem(){
  if(!modemon){
    rtt.println("Modem already off");
    return;
  }
  rtt.println("Powering down modem");
  rtt.println("Pressing modem power button");
  digitalWrite(MODEM_PWRKEY,HIGH);
  delay(600);
  digitalWrite(MODEM_PWRKEY,LOW);
  delay(10);
  modemon = false;
  if(!espon){
  rtt.println("Powering down espmodem rail");
  digitalWrite(MODEM_ESP_PWR,LOW);
  delay(50);
  espmodemrail = false;
  }
}

void powerdownesp(){
  if(!espon){
    rtt.println("ESP already off");
    return;
  }
  rtt.println("Powering esp down");
  digitalWrite(MODEM_ESP_PWR,LOW);
  delay(50);
  espon = false;
  if(!modemon){
  rtt.println("Powering down espmodem rail");
  digitalWrite(MODEM_ESP_PWR,LOW);
  delay(50);
  espmodemrail = false;
  }
}

void softpwrup(){
  digitalWrite(MODEM_ESP_PWR,HIGH);
  delay(1);
  digitalWrite(MODEM_ESP_PWR,LOW);
  delay(2);
  digitalWrite(MODEM_ESP_PWR,HIGH);
  delay(1);
  digitalWrite(MODEM_ESP_PWR,LOW);
  delay(2);
  digitalWrite(MODEM_ESP_PWR,HIGH);
  delay(1);
  digitalWrite(MODEM_ESP_PWR,LOW);
  delay(2);
  digitalWrite(MODEM_ESP_PWR,HIGH);
  delay(500);
}

void gpioinit(){
  //latch on the main power
  pinMode(PWR_LATCH , OUTPUT);
  digitalWrite(PWR_LATCH , LOW);
  //setup watchdog feeding
  pinMode(WAKE, INPUT);
  pinMode(DONE, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(WAKE), wd_handler, RISING);
  //setup I2C bus
  Wire.begin();
  //power up sensors
  pinMode(SENSOR_PWR, OUTPUT);
  digitalWrite(SENSOR_PWR, HIGH);
  //set up leds
  pinMode(RED_LED,OUTPUT);
  pinMode(GREEN_LED,OUTPUT);
  digitalWrite(RED_LED,HIGH);
  digitalWrite(GREEN_LED,HIGH);
  //set all other pins
  pinMode(MODEM_ESP_PWR,OUTPUT);
  digitalWrite(MODEM_ESP_PWR,LOW);
  pinMode(ESP_PWR,OUTPUT);
  digitalWrite(ESP_PWR,LOW);
  pinMode(MODEM_PWRKEY,OUTPUT);
  digitalWrite(MODEM_PWRKEY,LOW);
  pinMode(PWR_SW_IN, INPUT);
  pinMode(MODEM_CON_A,OUTPUT);
  pinMode(MODEM_CON_B,OUTPUT);
  digitalWrite(MODEM_CON_A,LOW);
  digitalWrite(MODEM_CON_B,LOW);
  //this is not nice and needs to be done get one initial watchdog pulse. This is also time sensitive so dont mess around with it
  delay(600);
  for (size_t i = 0; i < 50; i++)
  {
    digitalWrite(DONE,HIGH);
    delay(20);
    digitalWrite(DONE,LOW);
    delay(20);
  }

}

void powerupesp(){
  if(espon){
    rtt.println("ESP already on");
    return;
  }
  rtt.println("Powering up esp");
  if(espmodemrail){
    rtt.println("Power rail alredy up");
  }
  else{
  rtt.println("Powering espmodem power rail");
  softpwrup();
  }
  digitalWrite(ESP_PWR,HIGH);
  delay(50);
  espmodemrail = true;
  espon = true;
}

void powerupmodem(){
  if(modemon){
    rtt.println("Modem already on");
    return;
  }
  rtt.println("powering up modem");
  if(espmodemrail){
  rtt.println("Power rail alredy up");
  }
  else{
    rtt.println("Powering espmodem power rail");
    softpwrup();
  }
  espmodemrail = true;
  rtt.println("Pressing modem power button");
  digitalWrite(MODEM_PWRKEY,HIGH);
  delay(600);
  digitalWrite(MODEM_PWRKEY,LOW);
  delay(10);
  modemon = true;
}

float readBatteryVoltage(){
  return ((float)analogRead(VBAT_DIV) / 1000.0) * 6.0;
}

float readLightSensor(float gain, int integrationTime){
  unsigned long startTime = millis();
  bool beginSuccess = false;
  while (millis() - startTime < 1000) {
    if (light.begin()) {
      beginSuccess = true;
      break;
    }
    delay(10);
  }
  if (!beginSuccess) {
    rtt.println("Light sensor begin failed");
    return -1.0;
  }
  light.setGain(gain);
  light.setIntegTime(integrationTime);
  startTime = millis();
  bool readSuccess = false;
  long luxVal = 0;
  while (millis() - startTime < 2000) {
  luxVal = light.readLight();
    if (luxVal >= 0) {
      readSuccess = true;
      break;
    }
    delay(50);
  }
  if (!readSuccess) {
    rtt.println("Light sensor read failed");
  light.enablePowSave();
    return -1.0;
  }
  light.enablePowSave();
  return (float)luxVal;
}

bool readTemperatureHumidity(float& temperature, float& humidity) {
  ens210.begin();
  int t_data, t_status, h_data, h_status;
  ens210.measure(&t_data, &t_status, &h_data, &h_status);
  temperature = ens210.toCelsius(t_data, 10) / 10.0;
  humidity = ens210.toPercentageH(h_data, 1);
  ens210.lowpower(true);
  return (temperature != 0.0 || humidity != 0.0);
}

bool readPressure(float& pressure, float& sensorTemp) {
  lps22hb.begin();
  lps22hb.GetPressure(&pressure);
  lps22hb.GetTemperature(&sensorTemp);
  lps22hb.end();
  return (pressure != 0.0 || sensorTemp != 0.0);
}

bool readAccelerometers(float& acc1_x, float& acc1_y, float& acc1_z, float& acc2_x, float& acc2_y, float& acc2_z) {
  if (!acc1.begin() || !acc2.begin()) {
    return false;
  }
  acc1.setRange(LIS3DH_RANGE_2_G);
  acc2.setRange(LIS3DH_RANGE_2_G);
  acc1.read();
  acc2.read();
  acc1_x = acc1.x;
  acc1_y = acc1.y;
  acc1_z = acc1.z;
  acc2_x = acc2.x;
  acc2_y = acc2.y;
  acc2_z = acc2.z;
  return true;
}

bool readGPS(float& latitude, float& longitude, float& speed, float& altitude, int& visibleSatellites, int& usedSatellites, float& accuracy) {
  if (!modemon) {
    return false;
  }
  unsigned long startTime = millis();
  modem.enableGPS();
  delay(5000L); 
  int gps_year, gps_month, gps_day, gps_hour, gps_minute, gps_second;
  for (int8_t i = 3; i; i--) {
    if (millis() - startTime > deviceConfig.gps_timeout) {
      rtt.println("GPS overall timeout reached");
      modem.disableGPS();
      return false;
    }
    rtt.printf("Requesting current GPS/GNSS/GLONASS location");
    if (modem.getGPS(&latitude, &longitude, &speed, &altitude,
                     &visibleSatellites, &usedSatellites, &accuracy, 
                     &gps_year, &gps_month, &gps_day, &gps_hour, &gps_minute, &gps_second)) {
      modem.disableGPS();
      return true;
    } else {
      rtt.println("Couldn't get GPS/GNSS/GLONASS location, retrying in 5s.");
      delay(5000L);
    }
  }
  modem.disableGPS();
  return false;
}

bool initializeModem() {
  if (modemInitialized) {
    return true;
  }
  powerupmodem();
  SerialAT.begin(115200);
  delay(10);
  if (!modem.init()) {
    rtt.println("Modem init failed");
    powerdownmodem();
    return false;
  }
  modemInitialized = true;
  return true;
}

bool connectToNetwork() {
  if (!modemInitialized) {
    rtt.println("Modem not initialized");
    return false;
  }
  modem.gprsConnect(deviceConfig.apn.c_str(), deviceConfig.username.c_str(), deviceConfig.password.c_str());
  if (!modem.waitForNetwork(60000L, true)) {
    rtt.println("Network connection failed");
    return false;
  }
  rtt.println("Network connection test successful");
  rtt.println("Modem ready for data transmission");
  
  // Test network information
  rtt.println("Testing network information...");
  readNetworkInfo();
  return true;
}

bool disconnectFromNetwork() {
  if (!modemInitialized) {
    return true;
  }
  rtt.println("Disconnecting from network...");
  modem.gprsDisconnect();
  rtt.println("Network disconnected");
  return true;
}

// Helper function to validate modem connection before large transfers
bool validateModemConnection() {
  if (!modemInitialized) {
    return false;
  }
  
  // Check if modem is still responsive
  if (!modem.isNetworkConnected()) {
    rtt.println("Network connection lost, reconnecting...");
    if (!connectToNetwork()) {
      return false;
    }
  }
  
  // Check signal quality
  int signalQuality = modem.getSignalQuality();
  if (signalQuality == 99) {
    rtt.println("Poor signal quality, may affect large transfers");
  }
  
  return true;
}

bool sendSensorDataToServer(const SensorData& data, const ServerConfig& config) {
  rtt.println("Starting HTTP transmission...");
  
  if (!modemInitialized) {
    rtt.println("Modem not initialized");
    return false;
  }
  
  // Validate modem connection
  if (!validateModemConnection()) {
    rtt.println("Modem connection validation failed");
    return false;
  }
  
  if (!modem.isGprsConnected()) {
    rtt.println("Not connected to network");
    return false;
  }

  // Try up to 3 times
  for (int attempt = 1; attempt <= 3; attempt++) {
    rtt.print("HTTP attempt "); rtt.print(attempt); rtt.println("/3");
    
    TinyGsmClient client(modem);
    HttpClient http(client, config.server, config.port);
    
    // Set longer timeout for large requests
    http.setTimeout(60000); // 60 second timeout
    
    // Create JSON document with larger size for complex data
    DynamicJsonDocument doc(32000); // Increased size for large payloads
    
    // Build JSON object efficiently
    doc["ts"] = data.timestamp;
    doc["lx"] = data.lux;
    doc["bat"] = round(data.battery_voltage * 100) / 100.0;
    doc["tmp"] = round(data.case_temperature * 10) / 10.0;
    doc["hum"] = round(data.case_humidity * 10) / 10.0;
    doc["prs"] = round(data.pressure * 100) / 100.0;
    doc["pt"] = round(data.pressure_sensor_temp * 100) / 100.0;
    doc["a1x"] = data.acc1_x;
    doc["a1y"] = data.acc1_y;
    doc["a1z"] = data.acc1_z;
    doc["a2x"] = data.acc2_x;
    doc["a2y"] = data.acc2_y;
    doc["a2z"] = data.acc2_z;
    doc["glt"] = round(data.gps_latitude * 100000000) / 100000000.0;
    doc["gln"] = round(data.gps_longitude * 100000000) / 100000000.0;
    doc["gsp"] = data.gps_speed;
    doc["gal"] = data.gps_altitude;
    doc["gsv"] = data.gps_visible_satellites;
    doc["gsu"] = data.gps_used_satellites;
    doc["gac"] = data.gps_accuracy;
    doc["wkr"] = data.wakeup_reason;
    doc["wdc"] = data.watchdog_counter;
    doc["mid"] = data.modem_id.length() > 0 ? data.modem_id : "unknown";
    doc["mt"] = round(data.modem_temperature * 10) / 10.0;
    doc["sid"] = data.sim_card_id.length() > 0 ? data.sim_card_id : "unknown";
    doc["nn"] = data.network_name.length() > 0 ? data.network_name : "unknown";
    doc["nid"] = data.network_id.length() > 0 ? data.network_id : "unknown";
    doc["lt"] = data.link_type.length() > 0 ? data.link_type : "unknown";
    doc["sq"] = data.signal_quality;
    doc["ss"] = data.signal_strength;
    doc["nrs"] = data.network_registration_status;
    doc["nb"] = data.network_band.length() > 0 ? data.network_band : "unknown";
    doc["nc"] = data.network_channel;
    doc["noc"] = data.network_operator_code.length() > 0 ? data.network_operator_code : "unknown";
    
    // Add WiFi scan data if available
    if (data.wifi_scan_data.length() > 0) {
      doc["wifi"] = data.wifi_scan_data;
    }
    
    // Add BLE scan data if available
    if (data.ble_scan_data.length() > 0) {
      doc["ble"] = data.ble_scan_data;
    }
    
    // Calculate JSON size first to check if it fits
    size_t jsonSize = measureJson(doc);
    rtt.print("JSON size: "); rtt.print(jsonSize); rtt.println(" bytes");
    
    // Optimize scan data if JSON is too large
    optimizeScanData(doc, 30000); // 30KB limit
    
    // Recalculate size after optimization
    jsonSize = measureJson(doc);
    rtt.print("Final JSON size: "); rtt.print(jsonSize); rtt.println(" bytes");
    
    if (jsonSize > 30000) { // Safety check
      rtt.println("JSON too large, truncating scan data");
      // Remove large scan data if JSON is too big
      if (doc.containsKey("wifi")) doc.remove("wifi");
      if (doc.containsKey("ble")) doc.remove("ble");
      jsonSize = measureJson(doc);
      rtt.print("Truncated JSON size: "); rtt.print(jsonSize); rtt.println(" bytes");
    }
    
    // Serialize JSON to string with error checking
    String jsonData;
    jsonData.reserve(jsonSize + 100); // Reserve memory to avoid fragmentation
    
    if (serializeJson(doc, jsonData) == 0) {
      rtt.println("JSON serialization failed");
      http.stop();
      return false;
    }
    
    // Validate JSON format
    if (jsonData.length() == 0) {
      rtt.println("Error: Empty JSON data");
      http.stop();
      return false;
    }
    
    if (!jsonData.startsWith("{") || !jsonData.endsWith("}")) {
      rtt.println("Error: Invalid JSON format");
      http.stop();
      return false;
    }
    
    rtt.print("Final JSON length: "); rtt.println(jsonData.length());
    
    // Send request using the optimized function for large payloads
    if (!sendLargeJsonData(http, doc, config.endpoint)) {
      rtt.println("Failed to send JSON data");
      http.stop();
      if (attempt < 3) {
        delay(2000);
        continue;
      }
      return false;
    }
    
    // Wait for response with proper timeout handling
    unsigned long startTime = millis();
    int statusCode = -1;
    bool responseReceived = false;
    
    while (millis() - startTime < 45000) { // 45 second timeout
      if (http.available()) {
        statusCode = http.responseStatusCode();
        responseReceived = true;
        break;
      }
      delay(50); // Shorter delay for faster response
    }
    
    if (!responseReceived) {
      rtt.println("HTTP request timeout");
      http.stop();
      if (attempt < 3) {
        delay(2000);
        continue;
      }
      return false;
    }
    
    // Read complete HTTP response with proper handling of chunked transfer
    String responseBody = readCompleteHttpResponse(http, 30000);
    
    rtt.print("Response length: "); rtt.println(responseBody.length());
    // Only print first 200 characters to avoid flooding the log
    if (responseBody.length() > 200) {
      rtt.print("Response (first 200 chars): "); rtt.println(responseBody.substring(0, 200));
    } else {
      rtt.print("Response: "); rtt.println(responseBody);
    }
    
    http.stop();
    
    if (statusCode >= 200 && statusCode < 300) {
      lastTransmissionTime = millis();
      rtt.print("HTTP success: "); rtt.println(statusCode);
      
      // Parse server response if it contains JSON
      if (responseBody.indexOf('{') != -1) {
        DynamicJsonDocument responseDoc(12288); // 12KB for response parsing (increased from 2KB)
        if (parseServerResponse(responseBody, responseDoc)) {
          // Extract useful information from server response
          if (responseDoc.containsKey("status")) {
            String status = responseDoc["status"];
            rtt.print("Server status: "); rtt.println(status);
          }
          if (responseDoc.containsKey("ts")) {
            unsigned long serverTime = responseDoc["ts"];
            rtt.print("Server timestamp: "); rtt.println(serverTime);
          }
          if (responseDoc.containsKey("processed")) {
            bool dataProcessed = responseDoc["processed"];
            rtt.print("Data processed: "); rtt.println(dataProcessed ? "Yes" : "No");
          }
          if (responseDoc.containsKey("fields")) {
            int fieldCount = responseDoc["fields"];
            rtt.print("Fields processed: "); rtt.println(fieldCount);
          }
          if (responseDoc.containsKey("log")) {
            String logFile = responseDoc["log"];
            rtt.print("Data logged to: "); rtt.println(logFile);
          }
          if (responseDoc.containsKey("error")) {
            String error = responseDoc["error"];
            rtt.print("Server error: "); rtt.println(error);
          }
        } else {
          rtt.println("Failed to parse server JSON response");
        }
      } else {
        rtt.println("No JSON data found in server response");
      }
      
      return true;
    } else {
      rtt.print("HTTP error: "); rtt.println(statusCode);
      if (attempt < 3) {
        rtt.println("Retrying...");
        delay(3000); // Longer delay between retries
        continue;
      }
    }
  }
  
  rtt.println("All HTTP attempts failed");
  return false;
}

bool sendSensorDataToServer(const SensorData& data) {
  // Create ServerConfig from loaded device configuration
  ServerConfig config = {
    deviceConfig.server_url.c_str(),
    deviceConfig.server_endpoint.c_str(),
    deviceConfig.server_port,
    deviceConfig.apn.c_str(),
    deviceConfig.username.c_str(),
    deviceConfig.password.c_str()
  };
  return sendSensorDataToServer(data, config);
}

void manageModemLifecycle() {
  unsigned long currentTime = millis();
    rtt.println("Transmitting sensor data...");
    if (!initializeModemIfNeeded()) {
      rtt.println("Failed to initialize modem");
      return;
    }
    bool networkConnected = modem.isGprsConnected();
    if (!networkConnected) {
      rtt.println("Network not connected, connecting...");
      if (!connectToNetwork()) {
        rtt.println("Failed to connect to network");
        return;
      }
    }
    collectAllSensorData();
    readModemInfo();
    if (deviceConfig.enable_gps) {
      rtt.println("Acquiring GPS data...");
      unsigned long gpsStartTime = millis();
      float latitude, longitude, speed, altitude;
      int visibleSatellites, usedSatellites;
      float accuracy;
      if (readGPS(latitude, longitude, speed, altitude, visibleSatellites, usedSatellites, accuracy)) {
        sensorData.gps_latitude = latitude;
        sensorData.gps_longitude = longitude;
        sensorData.gps_speed = speed;
        sensorData.gps_altitude = altitude;
        sensorData.gps_visible_satellites = visibleSatellites;
        sensorData.gps_used_satellites = usedSatellites;
        sensorData.gps_accuracy = accuracy;
        rtt.println("GPS data acquired");
      } else {
        rtt.println("GPS acquisition failed");
      }
      if (millis() - gpsStartTime > deviceConfig.gps_timeout) {
        rtt.println("GPS timeout, proceeding without GPS");
      }
    }
    readNetworkInfo();
    if (sendSensorDataToServer(sensorData)) {
      rtt.println("Data transmission successful");
      blinkLED(4, 200, 400, true);
    } else {
      rtt.println("Data transmission failed");
      blinkLED(4, 200, 400, false);
    }
    
    // Shutdown modem between loops if configured
    if (deviceConfig.modem_shutdown_between_loops) {
      // Check if we should keep modem on due to motion
      if (shouldKeepModemPowered()) {
        rtt.println("Keeping modem powered on due to motion or configuration");
        logPowerManagementStatus();
      } else {
        rtt.println("Shutting down modem between loops...");
        shutdownModem();
      }
    }
}

bool readModemInfo() {
  if (!modemInitialized) {
    return false;
  }
  sensorData.modem_id = modem.getIMEI();
  sensorData.modem_temperature = modem.getTemperature();
  sensorData.sim_card_id = modem.getSimCCID();
  sensorData.network_name = modem.getOperator();
  sensorData.network_id = modem.getIMSI();
  return true;
}

bool readNetworkInfo() {
  if (!modemInitialized) {
    return false;
  }
  sensorData.signal_quality = modem.getSignalQuality();
  if (sensorData.signal_quality == 99) {
    sensorData.signal_strength = -999; 
  } else if (sensorData.signal_quality >= 0 && sensorData.signal_quality <= 31) {
    // Convert to dBm: 0 = -113 dBm, 31 = -51 dBm
    sensorData.signal_strength = -113 + (sensorData.signal_quality * 2);
  } else {
    sensorData.signal_strength = -999; // Invalid value
  }
  sensorData.network_registration_status = modem.getRegistrationStatus();
  sensorData.link_type = "Unknown";
  modem.sendAT(GF("+QNWINFO"));
  String response = modem.stream.readString();
  if (response.indexOf("+QNWINFO:") != -1) {
    int start = response.indexOf("+QNWINFO:") + 9;
    String data = response.substring(start);
    // Parse the comma-separated values
    int pos1 = data.indexOf(",");
    int pos2 = data.indexOf(",", pos1 + 1);
    int pos3 = data.indexOf(",", pos2 + 1);
    if (pos1 > 0 && pos2 > pos1 && pos3 > pos2) {
      // Extract access technology (remove quotes)
      String act = data.substring(0, pos1);
      act.replace("\"", "");
      // Extract operator code (remove quotes)
      String oper = data.substring(pos1 + 1, pos2);
      oper.replace("\"", "");
      // Extract band (remove quotes)
      String band = data.substring(pos2 + 1, pos3);
      band.replace("\"", "");
      // Extract channel
      String channel = data.substring(pos3 + 1);
      channel.replace("\"", "");
      channel.replace("\r", "");
      channel.replace("\n", "");
      // Set the values
      sensorData.network_operator_code = oper;
      sensorData.network_band = band;
      sensorData.network_channel = channel.toInt();
      // Map access technology to readable format
      if (act == " eMTC" || act == "LTE") {
        sensorData.link_type = "LTE";
      } else if (act == "GSM" || act == "GPRS" || act == "EDGE") {
        sensorData.link_type = "GSM (2G)";
      } else if (act == "UMTS" || act == "HSPA") {
        sensorData.link_type = "UTRAN (3G)";
      } else if (act == "CDMA") {
        sensorData.link_type = "CDMA2000";
      } else if (act == "NB-IoT") {
        sensorData.link_type = "NB-IoT";
      } else {
        sensorData.link_type = act;
      }
    }
  }
  if (sensorData.link_type == "Unknown") {
    modem.sendAT(GF("+CREG?"));
    response = modem.stream.readString();
    // Try to infer from registration status and signal quality
    if (sensorData.network_registration_status == 1 || sensorData.network_registration_status == 5) {
      // Registered, home network or roaming
      if (sensorData.signal_quality > 0 && sensorData.signal_quality <= 31) {
        // Good signal quality suggests LTE or 3G
        if (sensorData.signal_quality >= 20) {
          sensorData.link_type = "E-UTRAN (LTE)";
        } else if (sensorData.signal_quality >= 15) {
          sensorData.link_type = "UTRAN (3G)";
        } else {
          sensorData.link_type = "GSM (2G)";
        }
      }
    }
  }
  // Method 3: Try +QCSQ (signal quality command that might give more info)
  if (sensorData.link_type == "Unknown") {
    modem.sendAT(GF("+QCSQ"));
    response = modem.stream.readString();
  }
  rtt.print("Network Info - Link: "); rtt.print(sensorData.link_type);
  rtt.print(", Band: "); rtt.print(sensorData.network_band);
  rtt.print(", Channel: "); rtt.print(sensorData.network_channel);
  rtt.print(", Operator: "); rtt.print(sensorData.network_operator_code);
  rtt.print(", Quality: "); rtt.print(sensorData.signal_quality);
  rtt.print(", Strength: "); rtt.print(sensorData.signal_strength);
  rtt.print(" dBm, Reg Status: "); rtt.println(sensorData.network_registration_status);
  return true;
}

bool modemFileOpen(const String& filename, int mode) {
  if (!modemInitialized) {
    rtt.println("Modem not initialized");
    return false;
  }
  if (fileOpen) {
    rtt.println("Closing previously opened file");
    modemFileClose();
    delay(100); // Wait for close to complete
  }
  
  // Flush buffer before command
  while (modem.stream.available()) { modem.stream.read(); }

  // rtt.print("Opening file: "); rtt.print(filename); rtt.print(" mode: "); rtt.println(mode);
  modem.sendAT(GF("+QFOPEN="), filename, GF(","), mode);
  
  // Add delay before reading response
  delay(200);
  
  String response = modem.stream.readString();
  // rtt.println("Open response: " + response);
  
  if (response.indexOf("+QFOPEN:") != -1) {
    // Correctly parse file handle from response: +QFOPEN: <fhandle>
    int handle_start = response.indexOf(":") + 1;
    String handle_str = response.substring(handle_start);
    handle_str.trim();
    currentFileHandle = handle_str.toInt();
    
    // rtt.print("Parsed file handle: "); rtt.println(currentFileHandle);
    
    if (currentFileHandle >= 0) { // Note: 0 is a valid file handle
        fileOpen = true;
        // rtt.println("File opened successfully, handle: " + String(currentFileHandle));
        return true;
    } else {
        rtt.println("Failed to parse valid file handle.");
        return false;
    }
  } else if (response.indexOf("ERROR") != -1) {
    rtt.println("Failed to open file - ERROR response");
    return false;
  } else {
    rtt.println("Failed to open file - unexpected response");
    return false;
  }
}

bool modemFileWrite(const String& data) {
  if (!fileOpen || currentFileHandle < 0) {
    rtt.println("No file open for writing");
    return false;
  }
  
  // Flush buffer before command
  while (modem.stream.available()) { modem.stream.read(); }
  
  // rtt.print("Writing "); rtt.print(data.length()); rtt.print(" bytes to file handle: "); rtt.println(currentFileHandle);
  modem.sendAT(GF("+QFWRITE="), currentFileHandle, GF(","), data.length());
  
  // Add delay before reading response
  delay(100);
  
  String response = modem.stream.readString();
  // rtt.println("Write command response: " + response);
  
  if (response.indexOf("CONNECT") != -1) {
    // Send the actual data
    delay(50); // Longer delay to ensure modem is ready
    modem.stream.print(data);
    
    // Add delay before reading response
    delay(100);
    
    response = modem.stream.readString();
    // rtt.println("Write data response: " + response);
    
    if (response.indexOf("OK") != -1) {
      // rtt.println("Data written successfully");
      return true;
    } else if (response.indexOf("ERROR") != -1) {
      rtt.println("Write failed - ERROR response");
      return false;
    } else {
      rtt.println("Write failed - unexpected response");
      return false;
    }
  } else if (response.indexOf("ERROR") != -1) {
    rtt.println("Write failed - ERROR response");
    return false;
  } else {
    rtt.println("Write failed - no CONNECT response");
    return false;
  }
}

bool modemFilePosition(int& position) {
  if (!fileOpen || currentFileHandle < 0) {
    rtt.println("No file open for position query");
    return false;
  }
  
  // Flush buffer before command
  while (modem.stream.available()) { modem.stream.read(); }
  
  // rtt.print("Querying current file position for handle: "); rtt.println(currentFileHandle);
  modem.sendAT(GF("+QFPOSITION="), currentFileHandle);
  
  // Add delay before reading response
  delay(100);
  
  String response = modem.stream.readString();
  // rtt.println("Position response: " + response);
  
  if (response.indexOf("+QFPOSITION:") != -1) {
    // Parse position from response: +QFPOSITION: <position>
    int pos_start = response.indexOf(":") + 1;
    String pos_str = response.substring(pos_start);
    pos_str.trim();
    position = pos_str.toInt();
    // rtt.print("Current file position: "); rtt.println(position);
    return true;
  } else if (response.indexOf("ERROR") != -1) {
    rtt.println("Position query failed - ERROR response");
    return false;
  } else {
    rtt.println("Position query failed - unexpected response");
    return false;
  }
}

bool modemFileSeek(int offset, int origin) {
  if (!fileOpen || currentFileHandle < 0) {
    rtt.println("No file open for seeking");
    return false;
  }
  
  // Flush buffer before command
  while (modem.stream.available()) { modem.stream.read(); }
  
  // rtt.print("Seeking to offset: "); rtt.print(offset); rtt.print(" origin: "); rtt.print(origin); rtt.print(" in file handle: "); rtt.println(currentFileHandle);
  modem.sendAT(GF("+QFSEEK="), currentFileHandle, GF(","), offset, GF(","), origin);
  
  // Add delay before reading response
  delay(100);
  
  String response = modem.stream.readString();
  // rtt.println("Seek response: " + response);
  
  if (response.indexOf("OK") != -1) {
    // rtt.println("File seek successful");
    return true;
  } else if (response.indexOf("ERROR") != -1) {
    rtt.println("Seek failed - ERROR response");
    return false;
  } else {
    rtt.println("Seek failed - unexpected response");
    return false;
  }
}

bool modemFileRead(String& data, int length) {
  if (!fileOpen || currentFileHandle < 0) {
    rtt.println("No file open for reading");
    return false;
  }
  if (length <= 0) {
    length = 1024;
  }
  if (!modemFileSeek(0, 0)) {
    rtt.println("Failed to seek to beginning of file");
    return false;
  }
  while (modem.stream.available()) { modem.stream.read(); }
  //int lastwd = wdcounter;
  //while(lastwd == wdcounter){
  //  nrf_delay_us(200);
  //}
  //delay(20);
  // rtt.print("Reading "); rtt.print(length); rtt.println(" bytes");
  modem.sendAT(GF("+QFREAD="), currentFileHandle, GF(","), length);
  String response = "";
  unsigned long startTime = millis();
  unsigned long timeout = 10000; // 10 second timeout
  while ((millis() - startTime) < timeout) {
    if (modem.stream.available()) {
      char c = modem.stream.read();
      response += c;
    } else {
      delay(1); 
      if (response.indexOf("OK") != -1 || response.indexOf("ERROR") != -1) {
      //  // Wait a bit more to ensure we get all data
      while (modem.stream.available()) {
        response += (char)modem.stream.read();
        }
         delay(10);
        while (modem.stream.available()) {
          response += (char)modem.stream.read();
        }
       break;
      }
    }
  }
  if (response.indexOf("CONNECT") != -1) {
    int connectIndex = response.indexOf("CONNECT");
    if (connectIndex != -1) {
      int spaceIndex = response.indexOf(" ", connectIndex);
      if (spaceIndex != -1) {
        String lengthStr = response.substring(spaceIndex + 1);
        lengthStr.trim();
        int endIndex = lengthStr.indexOf("\r");
        if (endIndex != -1) {
          lengthStr = lengthStr.substring(0, endIndex);
        }
        endIndex = lengthStr.indexOf("\n");
        if (endIndex != -1) {
          lengthStr = lengthStr.substring(0, endIndex);
        }
      }
    }
    int dataStart = response.indexOf("CONNECT") + 7; // Skip "CONNECT "
    if (dataStart > 7) {
      // Find the end of the length number and the start of the actual data
      int lengthEnd = response.indexOf("\r", dataStart);
      if (lengthEnd == -1) lengthEnd = response.indexOf("\n", dataStart);
      if (lengthEnd == -1) lengthEnd = response.length();
      unsigned int actualDataStart = lengthEnd;
      while (actualDataStart < response.length() && 
             (response.charAt(actualDataStart) == '\r' || 
              response.charAt(actualDataStart) == '\n' || 
              response.charAt(actualDataStart) == ' ' || 
              response.charAt(actualDataStart) == '\t')) {
        actualDataStart++;
      }
      // Find the end of the data (before the OK response)
      unsigned int dataEnd = response.indexOf("OK", actualDataStart);
        // Make sure we don't include any whitespace before OK
        while (dataEnd > actualDataStart && 
               (response.charAt(dataEnd - 1) == '\r' || 
                response.charAt(dataEnd - 1) == '\n' || 
                response.charAt(dataEnd - 1) == ' ' || 
                response.charAt(dataEnd - 1) == '\t')) {
          dataEnd--;
        }
      data = response.substring(actualDataStart, dataEnd);
      data.trim();
      if (response.indexOf("OK") != -1) {
        return true;
      } else {
        delay(100);
        String okResponse = modem.stream.readString();
        if (okResponse.indexOf("OK") != -1) {
          return true;
        } else if (okResponse.indexOf("ERROR") != -1) {
          rtt.println("Read failed - ERROR response");
          return false;
        } else {
          if (data.length() > 0) {
            return true;
          } else {
            rtt.println("Read failed - no data and no OK response");
            return false;
          }
        }
      }
    } else {
      rtt.println("Failed to find data start in response");
      return false;
    }
  } else if (response.indexOf("ERROR") != -1) {
    rtt.println("Read failed - ERROR response");
    return false;
  } else {
    rtt.println("Read failed - no CONNECT response");
    return false;
  }
}

bool modemFileClose() {
  if (!fileOpen || currentFileHandle < 0) {
    rtt.println("No file to close or invalid file handle");
    return true;
  }
  modem.sendAT(GF("+QFCLOSE="), currentFileHandle);
  delay(100);
  String response = modem.stream.readString();
  if (response.indexOf("OK") != -1) {
    fileOpen = false;
    currentFileHandle = -1;
    return true;
  } else if (response.indexOf("ERROR") != -1) {
    rtt.println("Failed to close file - ERROR response");
    return false;
  } else {
    rtt.println("Failed to close file - unexpected response");
    return false;
  }
}

bool modemFileDelete(const String& filename) {
  if (!modemInitialized) {
    return false;
  }
  while (modem.stream.available()) { modem.stream.read(); }
  rtt.print("Deleting file: "); rtt.println(filename);
  modem.sendAT(GF("+QFDEL="), filename);
  String response = modem.stream.readString();
  rtt.println("Delete response: " + response);
  return response.indexOf("OK") != -1;
}

bool modemFileExists(const String& filename) {
  if (!modemInitialized) {
    return false;
  }
  while(modem.stream.available()) { modem.stream.read(); }
  modem.sendAT(GF("+QFOPEN="), filename, GF(",0"));
  String response = modem.stream.readString();
  if (response.indexOf("+QFOPEN:") != -1) {
    int handle_start = response.indexOf(":") + 1;
    String handle_str = response.substring(handle_start);
    handle_str.trim();
    int fhandle = handle_str.toInt();
    if (fhandle >= 0) { 
      modem.sendAT(GF("+QFCLOSE="), fhandle);
      modem.stream.readString();
      return true;
    }
  }
  return false;
}

int modemFileSize(const String& filename) {
  if (!modemInitialized) {
    return -1;
  }
  return -1;
}

bool saveConfigToModem(const ModemConfig& config) {
  rtt.println("Saving configuration to modem...");
  DynamicJsonDocument doc(16000);
  doc["device_name"] = config.device_name;
  doc["transmission_interval"] = config.transmission_interval;
  doc["transmission_interval_motion"] = config.transmission_interval_motion;
  doc["enable_gps"] = config.enable_gps;
  doc["gps_timeout"] = config.gps_timeout;
  doc["server_url"] = config.server_url;
  doc["server_endpoint"] = config.server_endpoint;
  doc["server_port"] = config.server_port;
  doc["apn"] = config.apn;
  doc["username"] = config.username;
  doc["password"] = config.password;
  doc["enable_storage"] = config.enable_storage;
  doc["max_stored_files"] = config.max_stored_files;
  doc["wifi_scan_enabled"] = config.wifi_scan_enabled;
  doc["wifi_scan_max_networks"] = config.wifi_scan_max_networks;
  doc["ble_scan_enabled"] = config.ble_scan_enabled;
  doc["ble_scan_max_beacons"] = config.ble_scan_max_beacons;
  doc["ble_scan_duration"] = config.ble_scan_duration;
  doc["modem_shutdown_between_loops"] = config.modem_shutdown_between_loops;
  doc["motion_detection_enabled"] = config.motion_detection_enabled;
  doc["motion_threshold"] = config.motion_threshold;
  doc["modem_shutdown_on_motion"] = config.modem_shutdown_on_motion;
  String configJson = "";
  serializeJson(doc, configJson);
  delay(10);
  const char* filename = "config.json";
  if (!modemFileOpen(filename, 1)) {
    rtt.println("Failed to open config file for writing");
    return false;
  }
  if (!modemFileWrite(configJson)) {
    rtt.println("Failed to write config data");
    modemFileClose();
    return false;
  }
  if (!modemFileClose()) {
    rtt.println("Failed to close config file");
    return false;
  }
  rtt.println("Configuration saved successfully");
  return true;
}

bool loadConfigFromModem(ModemConfig& config) {
  rtt.println("Loading configuration from modem...");
  const char* filename = "config.json";
  if (!modemFileExists(filename)) {
    rtt.println("Config file not found, using defaults");
    return false;
  }
  if (!modemFileOpen(filename, 0)) { // Mode 0 = read only
    rtt.println("Failed to open config file for reading");
    return false;
  }
  String configData;
  if (!modemFileRead(configData, 512)) { // Read up to 512 bytes
    rtt.println("Failed to read config data");
    modemFileClose();
    return false;
  }
  modemFileClose();
  DynamicJsonDocument doc(16000);
  DeserializationError error = deserializeJson(doc, configData);
  if (error) {
    rtt.print("JSON parsing failed: ");
    rtt.println(error.c_str());
    rtt.print("Error code: "); rtt.println(error.code());
    return false;
  }
  if (doc.containsKey("device_name")) {
    config.device_name = doc["device_name"].as<String>();
  }
  if (doc.containsKey("transmission_interval")) {
    config.transmission_interval = doc["transmission_interval"].as<unsigned long>();
  }
  if (doc.containsKey("transmission_interval_motion")) {
    config.transmission_interval_motion = doc["transmission_interval_motion"].as<unsigned long>();
  }
  if (doc.containsKey("enable_gps")) {
    config.enable_gps = doc["enable_gps"].as<bool>();
  }
  if (doc.containsKey("gps_timeout")) {
    config.gps_timeout = doc["gps_timeout"].as<unsigned long>();
  }
  if (doc.containsKey("server_url")) {
    config.server_url = doc["server_url"].as<String>();
  }
  if (doc.containsKey("server_endpoint")) {
    config.server_endpoint = doc["server_endpoint"].as<String>();
  }
  if (doc.containsKey("server_port")) {
    config.server_port = doc["server_port"].as<int>();
  }
  if (doc.containsKey("apn")) {
    config.apn = doc["apn"].as<String>();
  }
  if (doc.containsKey("username")) {
    config.username = doc["username"].as<String>();
  }
  if (doc.containsKey("password")) {
    config.password = doc["password"].as<String>();
  }
  if (doc.containsKey("enable_storage")) {
    config.enable_storage = doc["enable_storage"].as<bool>();
  }
  if (doc.containsKey("max_stored_files")) {
    config.max_stored_files = doc["max_stored_files"].as<int>();
  }
  if (doc.containsKey("wifi_scan_enabled")) {
    config.wifi_scan_enabled = doc["wifi_scan_enabled"].as<bool>();
  }
  if (doc.containsKey("wifi_scan_max_networks")) {
    config.wifi_scan_max_networks = doc["wifi_scan_max_networks"].as<int>();
  }
  if (doc.containsKey("ble_scan_enabled")) {
    config.ble_scan_enabled = doc["ble_scan_enabled"].as<bool>();
  }
  if (doc.containsKey("ble_scan_max_beacons")) {
    config.ble_scan_max_beacons = doc["ble_scan_max_beacons"].as<int>();
  }
  if (doc.containsKey("ble_scan_duration")) {
    config.ble_scan_duration = doc["ble_scan_duration"].as<unsigned long>();
  }
  if (doc.containsKey("modem_shutdown_between_loops")) {
    config.modem_shutdown_between_loops = doc["modem_shutdown_between_loops"].as<bool>();
  }
  if (doc.containsKey("motion_detection_enabled")) {
    config.motion_detection_enabled = doc["motion_detection_enabled"].as<bool>();
  }
  if (doc.containsKey("motion_threshold")) {
    config.motion_threshold = doc["motion_threshold"].as<float>();
  }
  if (doc.containsKey("modem_shutdown_on_motion")) {
    config.modem_shutdown_on_motion = doc["modem_shutdown_on_motion"].as<bool>();
  }
  //rtt.println("Configuration loaded successfully");
  return true;
}

void printConfig(const ModemConfig& config) {
  rtt.print("Device Name: "); rtt.println(config.device_name);
  rtt.print("Transmission Interval: "); rtt.print(config.transmission_interval); rtt.println(" ms");
  rtt.print("Transmission Interval (Motion): "); rtt.print(config.transmission_interval_motion); rtt.println(" ms");
  rtt.print("GPS Enabled: "); rtt.println(config.enable_gps ? "Yes" : "No");
  rtt.print("GPS Timeout: "); rtt.print(config.gps_timeout); rtt.println(" ms");
  rtt.print("WiFi Scan Enabled: "); rtt.println(config.wifi_scan_enabled ? "Yes" : "No");
  rtt.print("WiFi Max Networks: "); rtt.println(config.wifi_scan_max_networks);
  rtt.print("BLE Scan Enabled: "); rtt.println(config.ble_scan_enabled ? "Yes" : "No");
  rtt.print("BLE Max Beacons: "); rtt.println(config.ble_scan_max_beacons);
  rtt.print("BLE Scan Duration: "); rtt.print(config.ble_scan_duration); rtt.println(" ms");
  rtt.print("Modem Shutdown Between Loops: "); rtt.println(config.modem_shutdown_between_loops ? "Yes" : "No");
  rtt.print("Motion Detection Enabled: "); rtt.println(config.motion_detection_enabled ? "Yes" : "No");
  rtt.print("Motion Threshold: "); rtt.print(config.motion_threshold, 3); rtt.println(" g");
  rtt.print("Modem Shutdown On Motion: "); rtt.println(config.modem_shutdown_on_motion ? "Yes" : "No");
  rtt.print("Server URL: "); rtt.println(config.server_url);
  rtt.print("Server Endpoint: "); rtt.println(config.server_endpoint);
  rtt.print("Server Port: "); rtt.println(config.server_port);
  rtt.print("APN: "); rtt.println(config.apn);
  rtt.print("Storage Enabled: "); rtt.println(config.enable_storage ? "Yes" : "No");
  rtt.print("Max Stored Files: "); rtt.println(config.max_stored_files);
}

void LoadDeviceConfig() {
  if(saveConfig) {
    ModemConfig newconfig;
    rtt.println("Saving new configuration:");
    if (saveConfigToModem(newconfig)) {
      rtt.println("config saved successfully");
      printConfig(newconfig);
    } else {
      rtt.println("Failed to save test config");
    }
  }
  // Load configuration from modem storage
  if(loadConfigFromModem(deviceConfig)) {
    rtt.println("Configuration loaded successfully:");
    printConfig(deviceConfig);
  }
  else {
    rtt.println("Failed to load configuration, using defaults");
  }
}

bool initializeESPSerial() {
  if (espSerialInitialized) {
    return true;
  }
  rtt.println("Initializing ESP serial communication...");
  Serial2.begin(115200);
  //pinMode(ESP_TXD, OUTPUT);
  //pinMode(ESP_RXD, INPUT);
  powerupESP();
  delay(2000);
  //digitalWrite(ESP_TXD, HIGH);
  espSerialInitialized = true;
  return true;
}

void powerupESP() {
  if (espon) {
    rtt.println("ESP already powered on");
    return;
  }
  rtt.println("Powering up ESP microcontroller...");
  if (!espmodemrail) {
    rtt.println("Powering up ESP/Modem power rail...");
    softpwrup();
    espmodemrail = true;
  }
  pinMode(ESP_GPIO0, OUTPUT);
  digitalWrite(ESP_GPIO0, HIGH);
  digitalWrite(ESP_PWR, HIGH);
  delay(50);
  espon = true;
}

void powerdownESP() {
  if (!espon) {
    rtt.println("ESP already powered down");
    return;
  }
  rtt.println("Powering down ESP microcontroller...");
  digitalWrite(ESP_PWR, LOW);
  delay(50);
  espon = false;
  // Power down the rail if modem is also off
  if (!modemon) {
    rtt.println("Powering down ESP/Modem power rail...");
    digitalWrite(MODEM_ESP_PWR, LOW);
    espmodemrail = false;
  }
  espSerialInitialized = false;
  rtt.println("ESP powered down successfully");
}

bool sendCommandToESP(const String& command) {
  if (!espSerialInitialized) {
    rtt.println("ESP serial not initialized");
    return false;
  }
  rtt.print("Sending to ESP: "); rtt.println(command);
  String cmd = command + "\r\n";
  delayMicroseconds(100);
  Serial2.write(cmd.c_str(), cmd.length());
  return true;
}

void ScanForWiFiNetworks() {
  if (!deviceConfig.wifi_scan_enabled) {
   return;
  }
  if (initializeESPSerial()) {
    String response;
    if (sendESPCommandAndGetResponse("AT+CWLAP", response, 16000)) {
      rtt.println("WiFi scan successful");
      sensorData.wifi_scan_data = parseCWLAPData(response);
      rtt.print("Length: "); rtt.println(sensorData.wifi_scan_data.length());
      //rtt.print("JSON: "); rtt.println(sensorData.wifi_scan_data);
    } else {
      rtt.println("WiFi scan failed or incomplete response");
      rtt.println("Attempting to parse anyway:");
      sensorData.wifi_scan_data = parseCWLAPData(response);
      rtt.print("Length: "); rtt.println(sensorData.wifi_scan_data.length());
      //rtt.print("JSON: "); rtt.println(sensorData.wifi_scan_data);
    }
  } else {
    rtt.println("ESP serial initialization failed");
  }
  powerdownESP();
}

bool readESPResponse(String& response, unsigned long timeout) {
  response = "";
  unsigned long startTime = millis();
  bool foundOK = false;
  bool foundError = false;
  
  while (millis() - startTime < timeout && !foundOK && !foundError) {
    while (Serial2.available()) {
      char c = Serial2.read();
      response += c;
      
      // Check for \r\nOK\r\n pattern
      if (response.length() >= 6) {
        int lastIndex = response.length() - 1;
        if (response.charAt(lastIndex) == '\n' && 
            response.charAt(lastIndex - 1) == '\r' &&
            response.charAt(lastIndex - 2) == 'K' &&
            response.charAt(lastIndex - 3) == 'O' &&
            response.charAt(lastIndex - 4) == '\n' &&
            response.charAt(lastIndex - 5) == '\r') {
          foundOK = true;
          break;
        }
      }
      
      // Check for ERROR pattern
      if (response.length() >= 7) {
        int lastIndex = response.length() - 1;
        if (response.charAt(lastIndex) == '\n' && 
            response.charAt(lastIndex - 1) == '\r' &&
            response.charAt(lastIndex - 2) == 'R' &&
            response.charAt(lastIndex - 3) == 'O' &&
            response.charAt(lastIndex - 4) == 'R' &&
            response.charAt(lastIndex - 5) == 'R' &&
            response.charAt(lastIndex - 6) == 'E') {
          foundError = true;
          break;
        }
      }
    }
    
    // Small delay to prevent busy waiting
    if (!foundOK && !foundError) {
      delayMicroseconds(100);
    }
  }
  
  // Return true if we found OK, false if we found ERROR or timed out
  return foundOK;
}

bool parseESPResponse(const String& rawResponse, String& filteredResponse, bool& hasOK) {
  filteredResponse = "";
  hasOK = false;
  String lines[40];
  int lineCount = 0;
  size_t startPos = 0;
  for (size_t i = 0; i < rawResponse.length() && lineCount < 40; i++) {
    if (rawResponse.charAt(i) == '\n' || rawResponse.charAt(i) == '\r') {
      if (i > startPos) {
        lines[lineCount] = rawResponse.substring(startPos, i);
        lineCount++;
      }
      startPos = i + 1;
    }
  }
  if (startPos < rawResponse.length()) {
    lines[lineCount] = rawResponse.substring(startPos);
    lineCount++;
  }
  for (int i = 0; i < lineCount; i++) {
    String line = lines[i];
    line.trim();
    delay(10);
    if (line.length() == 0) {
      continue;
    }
    bool isEcho = false;
    if (line.startsWith("AT")) {
      if (line.indexOf("+") == -1 && line.indexOf(":") == -1) {
        isEcho = true;
      } else if (line.indexOf("?") != -1) {
        isEcho = true;
      }
    }
    if (line == "OK") {
      hasOK = true;
      continue;
    }
    if (!isEcho && line.length() > 0) {
      if (filteredResponse.length() > 0) {
        filteredResponse += "\n";
      }
      filteredResponse += line;
    }
  }
  return true;
}

bool sendESPCommandAndGetResponse(const String& command, String& response, unsigned long timeout) {
  String rawResponse;
  Serial2.flush();
  sendCommandToESP(command);
  readESPResponse(rawResponse, timeout);
  String filteredResponse;
  bool hasOK;
  if (!parseESPResponse(rawResponse, filteredResponse, hasOK)) {
    return false;
  }
  response = filteredResponse;
  return hasOK;
}

void checkModemStorageSpace() {
  if (!modemInitialized) {
    rtt.println("Modem not initialized, cannot check storage space");
    return;
  }
  
  rtt.println("Checking Modem Storage Space");
  
  // Check UFS storage space
  modem.sendAT(GF("+QFLDS=\"UFS\""));
  delay(100); // Give modem time to process
  String response = modem.stream.readString();
  
  if (response.indexOf("+QFLDS:") != -1) {
    // Parse the response: +QFLDS: <freesize>,<total_size>
    // Example: +QFLDS: 1249984,1562624
    int start = response.indexOf("+QFLDS:") + 7;
    int comma = response.indexOf(",", start);
    int end = response.indexOf("\r", comma);
    
    if (start > 6 && comma > start && end > comma) {
      String freeSizeStr = response.substring(start, comma);
      String totalSizeStr = response.substring(comma + 1, end);
      
      unsigned long freeSize = freeSizeStr.toInt();
      unsigned long totalSize = totalSizeStr.toInt();
      unsigned long usedSize = totalSize - freeSize;
      float usedPercent = (float)usedSize / totalSize * 100.0;
  
      rtt.print("UFS Storage - Free: "); rtt.print(freeSize);
      rtt.print(" bytes, Total: "); rtt.print(totalSize);
      rtt.print(" bytes, Used: "); rtt.print(usedSize);
      rtt.print(" bytes ("); rtt.print(usedPercent, 1);
      rtt.println("%)");
    } else {
      rtt.println("Failed to parse UFS storage response");
    }
  } else {
    rtt.println("UFS storage command failed or not supported");
  }
  // Check EUFS storage space (Extended UFS on AP side)
  modem.sendAT(GF("+QFLDS=\"EUFS\""));
  delay(100); // Give modem time to process
  if (response.indexOf("+QFLDS:") != -1) {
    // Parse the response: +QFLDS: <freesize>,<total_size>
    int start = response.indexOf("+QFLDS:") + 7;
    int comma = response.indexOf(",", start);
    int end = response.indexOf("\r", comma);
    if (start > 6 && comma > start && end > comma) {
      String freeSizeStr = response.substring(start, comma);
      String totalSizeStr = response.substring(comma + 1, end);
      unsigned long freeSize = freeSizeStr.toInt();
      unsigned long totalSize = totalSizeStr.toInt();
      unsigned long usedSize = totalSize - freeSize;
      float usedPercent = (float)usedSize / totalSize * 100.0;
      rtt.print("EUFS Storage - Free: "); rtt.print(freeSize);
      rtt.print(" bytes, Total: "); rtt.print(totalSize);
      rtt.print(" bytes, Used: "); rtt.print(usedSize);
      rtt.print(" bytes ("); rtt.print(usedPercent, 1);
      rtt.println("%)");
    } else {
      rtt.println("Failed to parse EUFS storage response");
    }
  } else {
    rtt.println("EUFS storage command failed or not supported");
  }
  modem.sendAT(GF("+QFLDS"));
  delay(100); // Give modem time to process
  response = modem.stream.readString();
  if (response.indexOf("+QFLDS:") != -1) {
    // Parse the response: +QFLDS: <UFS_file_size>,<UFS_file_number>
    // Example: +QFLDS: 2330,5
    int start = response.indexOf("+QFLDS:") + 7;
    int comma = response.indexOf(",", start);
    int end = response.indexOf("\r", comma);
    if (start > 6 && comma > start && end > comma) {
      String fileSizeStr = response.substring(start, comma);
      String fileNumberStr = response.substring(comma + 1, end);
      unsigned long fileSize = fileSizeStr.toInt();
      int fileNumber = fileNumberStr.toInt();
      rtt.print("UFS Files - Total size: "); rtt.print(fileSize);
      rtt.print(" bytes, Number of files: "); rtt.println(fileNumber);
    } else {
      rtt.println("Failed to parse UFS file information response");
    }
  } else {
    rtt.println("UFS file information command failed or not supported");
  }
  rtt.println("|----------------------------------|");
}

// WiFi scan data parser
String parseCWLAPData(const String& cwlapResponse) {
  rtt.print("Input length: "); rtt.println(cwlapResponse.length());
  delay(10);
  
  // Check for empty input
  if (cwlapResponse.length() < 26) {
    rtt.println("parseCWLAPData: Empty input, returning empty result");
    rtt.println("parseCWLAPData: Input: "); rtt.println(cwlapResponse);
    return "{\"count\":0,\"ts\":" + String(millis()) + "}";
  }
  
  DynamicJsonDocument doc(16000);
  
  JsonArray networks = doc.createNestedArray("nets");
  if (networks.isNull()) {
    rtt.println("parseCWLAPData: Failed to create networks array");
    return "{\"count\":0,\"ts\":" + String(millis()) + "}";
  }
  // Process response line by line without storing all lines
  int networksAdded = 0;
  size_t lineStart = 0;

  for (size_t i = 0; i < cwlapResponse.length(); i++) {
    char c = cwlapResponse.charAt(i);
    
    // Check for line end
    if (c == '\n' || c == '\r') {
      if (i > lineStart) {
        // Extract current line
        String line = cwlapResponse.substring(lineStart, i);
        line.trim();
        
        // Check if this is a CWLAP line
        if (line.startsWith("+CWLAP:(")) {
          //rtt.println("parseCWLAPData: Found CWLAP line");
          //delay(10);
          
          // Check line length before substring
          if (line.length() < 9) {
            rtt.println("parseCWLAPData: Line too short, skipping");
            delay(10);
            lineStart = i + 1;
            continue;
          }
          
          // Remove the +CWLAP:( prefix and ) suffix
          String data = line.substring(8, line.length() - 1);

          // Parse the comma-separated values
          String values[12]; // Max 12 values per network
          int valueCount = 0;
          bool inQuotes = false;
          size_t valueStart = 0;
          
          //rtt.println("parseCWLAPData: Starting value parsing");
          delay(10);
          
          for (size_t j = 0; j < data.length() && valueCount < 12; j++) {
            char dataChar = data.charAt(j);
            
            if (dataChar == '"') {
              inQuotes = !inQuotes;
            } else if (dataChar == ',' && !inQuotes) {
              if (j > valueStart) {
                values[valueCount] = data.substring(valueStart, j);
                //rtt.print("parseCWLAPData: Value "); rtt.print(valueCount); rtt.print(": "); rtt.println(values[valueCount]);
                valueCount++;
              }
              valueStart = j + 1;
            }
          }
          
          // Add the last value
          if (valueStart < data.length()) {
            values[valueCount] = data.substring(valueStart);
            valueCount++;
          }
          
          //rtt.print("parseCWLAPData: Total values parsed: "); rtt.println(valueCount);
          
          // Create network object if we have enough values
          if (valueCount >= 4) {
            
            JsonObject network = networks.createNestedObject();
            if (network.isNull()) {
              rtt.println("parseCWLAPData: Failed to create network object");
              continue;
            }
            
            // Parse encryption type
            int enc = values[0].toInt();
            network["enc"] = enc;
            
            // Parse SSID (remove quotes)
            String ssid = values[1];
            if (ssid.startsWith("\"") && ssid.endsWith("\"") && ssid.length() > 2) {
              ssid = ssid.substring(1, ssid.length() - 1);
            }
            network["ssid"] = ssid;
            
            // Parse RSSI
            int rssi = values[2].toInt();
            network["rssi"] = rssi;
            
            // Parse MAC address (remove quotes)
            String mac = values[3];
            if (mac.startsWith("\"") && mac.endsWith("\"") && mac.length() > 2) {
              mac = mac.substring(1, mac.length() - 1);
            }
            network["mac"] = mac;
            
            // Parse channel
            if (valueCount > 4) {
              network["ch"] = values[4].toInt();
            }
            
            // Parse authentication mode
            if (valueCount > 5) {
              network["auth"] = values[5].toInt();
            }
            
            // Parse additional fields if available (only store if non-zero)
            if (valueCount > 6 && values[6].toInt() != 0) network["f6"] = values[6].toInt();
            if (valueCount > 7 && values[7].toInt() != 0) network["f7"] = values[7].toInt();
            if (valueCount > 8 && values[8].toInt() != 0) network["f8"] = values[8].toInt();
            if (valueCount > 9 && values[9].toInt() != 0) network["f9"] = values[9].toInt();
            if (valueCount > 10 && values[10].toInt() != 0) network["f10"] = values[10].toInt();
            
            networksAdded++;
            // Check if we've reached the maximum
            if (networksAdded >= deviceConfig.wifi_scan_max_networks) {
              rtt.println("parseCWLAPData: Reached maximum networks, stopping");
              break;
            }
          } else {
            rtt.println("parseCWLAPData: Not enough values for network object");
          }
        }
      }
      lineStart = i + 1;
    }
  }
  
  rtt.println("parseCWLAPData: Line processing completed");
  
  // Add summary information
  doc["count"] = networks.size();
  doc["ts"] = millis();
  
  rtt.print("parseCWLAPData: Final network count: "); rtt.println(networks.size());
  
  // Serialize to string (compact format)
  String jsonOutput = "";
  serializeJson(doc, jsonOutput);
  
  rtt.print("parseCWLAPData: JSON output length: "); rtt.println(jsonOutput.length());
  rtt.println("parseCWLAPData: Function completed successfully");
  
  return jsonOutput;
}

// Function to send large JSON data in a more memory-efficient way
bool sendLargeJsonData(HttpClient& http, const DynamicJsonDocument& doc, const String& endpoint) {
  // First, try to serialize the entire document
  String jsonData;
  jsonData.reserve(measureJson(doc) + 100);
  
  if (serializeJson(doc, jsonData) == 0) {
    rtt.println("JSON serialization failed");
    return false;
  }
  
  // If JSON is very large (>20KB), use chunked transfer
  if (jsonData.length() > 20000) {
    rtt.println("Using chunked transfer for large payload");
    
    http.beginRequest();
    http.post(endpoint);
    http.sendHeader("Content-Type", "application/json");
    http.sendHeader("Transfer-Encoding", "chunked");
    http.sendHeader("Accept", "application/json");
    http.sendHeader("User-Agent", "NRF-Firmware/1.0");
    http.sendHeader("Connection", "close");
    
    // Send data in chunks
    const int chunkSize = 512; // 2KB chunks
    int totalSent = 0;
    
    for (size_t i = 0; i < jsonData.length(); i += chunkSize) {
      size_t chunkLength = min(chunkSize, jsonData.length() - i);
      String chunk = jsonData.substring(i, i + chunkLength);
      
      // Send chunk length in hex
      http.print(String(chunkLength, HEX));
      http.print("\r\n");
      
      // Send chunk data
      if (http.print(chunk) != chunkLength) {
        rtt.println("Failed to send chunk");
        return false;
      }
      
      http.print("\r\n");
      totalSent += chunkLength;
      
      // Small delay to prevent overwhelming the modem
      delay(20);
    }
    
    // Send end chunk
    http.print("0\r\n\r\n");
    rtt.print("Sent "); rtt.print(totalSent); rtt.println(" bytes in chunks");
    
  } else {
    // For smaller payloads, use regular transfer
    http.beginRequest();
    http.post(endpoint);
    http.sendHeader("Content-Type", "application/json");
    http.sendHeader("Content-Length", jsonData.length());
    http.sendHeader("Accept", "application/json");
    http.sendHeader("User-Agent", "NRF-Firmware/1.0");
    http.sendHeader("Connection", "close");
    
    http.beginBody();
    
    // Send data in smaller chunks for reliability
    const int chunkSize = 1024;
    int totalSent = 0;
    
    for (size_t i = 0; i < jsonData.length(); i += chunkSize) {
      size_t chunkLength = min(chunkSize, jsonData.length() - i);
      String chunk = jsonData.substring(i, i + chunkLength);
      
      if (http.print(chunk) != chunkLength) {
        rtt.println("Failed to send chunk");
        return false;
      }
      totalSent += chunkLength;
      delay(10);
    }
    
    rtt.print("Sent "); rtt.print(totalSent); rtt.println(" bytes");
  }
  
  http.endRequest();
  return true;
}

// Function to intelligently handle large scan data
void optimizeScanData(DynamicJsonDocument& doc, size_t maxSize) {
  size_t currentSize = measureJson(doc);
  
  if (currentSize <= maxSize) {
    return; // No optimization needed
  }
  
  rtt.print("JSON size "); rtt.print(currentSize); rtt.print(" exceeds limit "); rtt.print(maxSize); rtt.println(", optimizing...");
  
  // First, try to truncate BLE data (usually larger)
  if (doc.containsKey("ble")) {
    rtt.println("Truncating BLE scan data");
    doc.remove("ble");
    currentSize = measureJson(doc);
    rtt.print("After BLE removal: "); rtt.print(currentSize); rtt.println(" bytes");
  }
  
  // If still too large, truncate WiFi data
  if (currentSize > maxSize && doc.containsKey("wifi")) {
    rtt.println("Truncating WiFi scan data");
    doc.remove("wifi");
    currentSize = measureJson(doc);
    rtt.print("After WiFi removal: "); rtt.print(currentSize); rtt.println(" bytes");
  }
  
  // If still too large, reduce precision of sensor data
  if (currentSize > maxSize) {
    rtt.println("Reducing sensor data precision");
    
    // Reduce GPS precision
    if (doc.containsKey("glt")) doc["glt"] = round(doc["glt"].as<float>() * 1000000) / 1000000.0;
    if (doc.containsKey("gln")) doc["gln"] = round(doc["gln"].as<float>() * 1000000) / 1000000.0;
    
    // Reduce accelerometer precision
    if (doc.containsKey("a1x")) doc["a1x"] = round(doc["a1x"].as<float>() * 100) / 100.0;
    if (doc.containsKey("a1y")) doc["a1y"] = round(doc["a1y"].as<float>() * 100) / 100.0;
    if (doc.containsKey("a1z")) doc["a1z"] = round(doc["a1z"].as<float>() * 100) / 100.0;
    if (doc.containsKey("a2x")) doc["a2x"] = round(doc["a2x"].as<float>() * 100) / 100.0;
    if (doc.containsKey("a2y")) doc["a2y"] = round(doc["a2y"].as<float>() * 100) / 100.0;
    if (doc.containsKey("a2z")) doc["a2z"] = round(doc["a2z"].as<float>() * 100) / 100.0;
    
    currentSize = measureJson(doc);
    rtt.print("After precision reduction: "); rtt.print(currentSize); rtt.println(" bytes");
  }
  
  if (currentSize <= maxSize) {
    rtt.println("Optimization successful");
  } else {
    rtt.println("Warning: JSON still exceeds size limit after optimization");
  }
}

bool waitForNextTransmission(unsigned long interval) {
  unsigned long currentTime = millis();
  
  // Choose interval based on motion detection
  unsigned long effectiveInterval = interval;
  if (deviceConfig.motion_detection_enabled && globalMotionDetected) {
    effectiveInterval = deviceConfig.transmission_interval_motion;
  }
  // Check if it's time for the next transmission
  if (currentTime - lastWaitCheck >= effectiveInterval) {
    lastWaitCheck = currentTime;
    return true; // Time to transmit
  }
  
  return false; // Still waiting
}

void performBackgroundTasks() {
  // Check battery voltage periodically
  static unsigned long lastBatteryCheck = 0;
  if (millis() - lastBatteryCheck > 30000) { // Check every 30 seconds
    sensorData.battery_voltage = readBatteryVoltage();
    lastBatteryCheck = millis();
  }
  
  if (deviceConfig.motion_detection_enabled && millis() - lastMotionCheck > 100) {
    // Read current accelerometer values
    
    if (readAccelerometers(currentAcc1X, currentAcc1Y, currentAcc1Z, currentAcc2X, currentAcc2Y, currentAcc2Z)) {
      // Calculate acceleration differences
      float acc1Diff = sqrt(pow(currentAcc1X - lastAcc1X, 2) + 
                           pow(currentAcc1Y - lastAcc1Y, 2) + 
                           pow(currentAcc1Z - lastAcc1Z, 2));
      float acc2Diff = sqrt(pow(currentAcc2X - lastAcc2X, 2) + 
                           pow(currentAcc2Y - lastAcc2Y, 2) + 
                           pow(currentAcc2Z - lastAcc2Z, 2));
      
      // Check if motion is detected using configured threshold
      if (acc1Diff > deviceConfig.motion_threshold || acc2Diff > deviceConfig.motion_threshold) {
        if (!globalMotionDetected) {
          globalMotionDetected = true;
          globalLastMotionTime = millis();
          rtt.println("Motion detected!");
          rtt.print("Acc1 diff: "); rtt.print(acc1Diff, 3);
          rtt.print(" Acc2 diff: "); rtt.println(acc2Diff, 3);
        }
      }
      lastAcc1X = currentAcc1X;
      lastAcc1Y = currentAcc1Y;
      lastAcc1Z = currentAcc1Z;
      lastAcc2X = currentAcc2X;
      lastAcc2Y = currentAcc2Y;
      lastAcc2Z = currentAcc2Z;
    }
    
    lastMotionCheck = millis();
  }
  delay(10);
}

// Function to parse JSON response from server
bool parseServerResponse(const String& response, JsonDocument& doc) {
  // Find the start of JSON data (after headers)
  int jsonStart = response.indexOf('{');
  if (jsonStart == -1) {
    rtt.println("No JSON data found in response");
    return false;
  }
  
  // Extract JSON part
  String jsonData = response.substring(jsonStart);
  
  // Parse JSON
  DeserializationError error = deserializeJson(doc, jsonData);
  if (error) {
    rtt.print("JSON parsing failed: ");
    rtt.println(error.c_str());
    return false;
  }
  
  return true;
}

// Function to read complete HTTP response with larger buffer
String readCompleteHttpResponse(HttpClient& http, unsigned long timeout) {
  String response;
  response.reserve(8192); // Reserve 8KB for response
  
  unsigned long startTime = millis();
  bool readingBody = false;
  bool chunkedTransfer = false;
  int expectedLength = 0;
  unsigned long lastDataTime = millis();
  
  while (millis() - startTime < timeout) {
    if (http.available()) {
      char c = http.read();
      response += c;
      lastDataTime = millis();
      
      // Check for chunked transfer encoding
      if (!chunkedTransfer && response.indexOf("Transfer-Encoding: chunked") != -1) {
        chunkedTransfer = true;
        rtt.println("Detected chunked transfer encoding");
      }
      
      // Check for content length
      if (!readingBody && response.indexOf("Content-Length:") != -1) {
        int lengthStart = response.indexOf("Content-Length:") + 15;
        int lengthEnd = response.indexOf("\r\n", lengthStart);
        if (lengthEnd != -1) {
          String lengthStr = response.substring(lengthStart, lengthEnd);
          lengthStr.trim();
          expectedLength = lengthStr.toInt();
          rtt.print("Expected content length: "); rtt.println(expectedLength);
        }
      }
      
      // Check for end of headers
      if (!readingBody && response.indexOf("\r\n\r\n") != -1) {
        readingBody = true;
        rtt.println("Headers complete, reading body");
      }
      
      // For chunked transfer, look for end chunk
      if (chunkedTransfer && readingBody) {
        if (response.indexOf("0\r\n\r\n") != -1) {
          rtt.println("Chunked transfer complete");
          break;
        }
      }
      
      // For regular transfer, check content length
      else if (!chunkedTransfer && readingBody && expectedLength > 0) {
        int bodyStart = response.indexOf("\r\n\r\n") + 4;
        if (bodyStart > 4) {
          int bodyLength = response.length() - bodyStart;
          if (bodyLength >= expectedLength) {
            rtt.print("Content length reached: "); rtt.println(bodyLength);
            break;
          }
        }
      }
    } else {
      // If no data for 2 seconds and we have some response, assume it's complete
      if (readingBody && (millis() - lastDataTime) > 2000 && response.length() > 100) {
        rtt.println("No more data for 2 seconds, assuming response complete");
        break;
      }
      delay(10); // Small delay to prevent busy waiting
    }
  }
  
  rtt.print("Total response length: "); rtt.println(response.length());
  
  // Check if response seems complete
  if (response.length() < 50) {
    rtt.println("Warning: Response seems too short");
  }
  
  return response;
}

// Function to properly shutdown modem with cleanup
void shutdownModem() {
  rtt.println("Performing complete modem shutdown...");
  
  // Disconnect from network first
  if (modemInitialized && modem.isGprsConnected()) {
    rtt.println("Disconnecting from network...");
    disconnectFromNetwork();
  }
  
  // Close any open files
  if (fileOpen) {
    rtt.println("Closing open files...");
    modemFileClose();
  }
  
  // Power down modem
  if (modemon) {
    rtt.println("Powering down modem...");
    powerdownmodem();
  }
  
  // Reset initialization flags
  modemInitialized = false;
  
  rtt.println("Modem shutdown complete");
}

// Function to initialize modem if needed
bool initializeModemIfNeeded() {
  if (!modemInitialized) {
    rtt.println("Modem not initialized, initializing...");
    return initializeModem();
  }
  return true;
}
// Function to check if modem should be kept powered on based on motion
bool shouldKeepModemPowered() {
  if (!deviceConfig.modem_shutdown_between_loops) {
    return true; // Always keep modem on if shutdown is disabled
  }
  
  if (deviceConfig.modem_shutdown_on_motion && globalMotionDetected) {
    return true; // Keep modem on if motion detected and configured to do so
  }
  
  return false; // Shutdown modem normally
}

// Function to get current power management status
void logPowerManagementStatus() {
  rtt.print("Power Management - Modem Shutdown: ");
  rtt.print(deviceConfig.modem_shutdown_between_loops ? "Enabled" : "Disabled");
  rtt.print(", Motion Shutdown: ");
  rtt.print(deviceConfig.modem_shutdown_on_motion ? "Enabled" : "Disabled");
  rtt.print(", Motion Detected: ");
  rtt.print(globalMotionDetected ? "Yes" : "No");
  rtt.print(", Keep Modem On: ");
  rtt.println(shouldKeepModemPowered() ? "Yes" : "No");
}