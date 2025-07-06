#include "main.h"

// Global configuration - will be loaded from modem storage
ModemConfig deviceConfig;

//set if config should be saved to modem
bool saveConfig = false;

void setup() {
  rtt.trimDownBufferFull();
  rtt.println("|---------Starting setup---------|");
  gpioinit(); 
  scanI2C();
  collectAllSensorData();
  sensorData.wakeup_reason = NRF_POWER->RESETREAS;
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
  if (bleBeaconsFound >= deviceConfig.ble_scan_max_beacons) {
    Bluefruit.Scanner.resume();
    return;
  }
  if (millis() - bleScanStartTime > deviceConfig.ble_scan_duration) {
    Bluefruit.Scanner.resume();
    return;
  }
  JsonObject beacon = bleBeacons.createNestedObject();
  if (beacon.isNull()) {
    rtt.println("Failed to create beacon object");
    Bluefruit.Scanner.resume();
    return;
  }
  beacon["r"] = report->rssi;
  char macStr[18];
  sprintf(macStr, "%02X%02X%02X%02X%02X%02X", report->peer_addr.addr[5], report->peer_addr.addr[4], report->peer_addr.addr[3], report->peer_addr.addr[2], report->peer_addr.addr[1], report->peer_addr.addr[0]);
  beacon["m"] = String(macStr);
  beacon["t"] = report->type.scan_response ? "s" : "a";
  beacon["c"] = report->type.connectable;
  beacon["d"] = report->type.directed;
  beacon["l"] = report->data.len;
  uint8_t buffer[32];
  memset(buffer, 0, sizeof(buffer));
  if(Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, buffer, sizeof(buffer))) {
    beacon["n"] = String((char*)buffer);
  } else if(Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, buffer, sizeof(buffer))) {
    beacon["n"] = String((char*)buffer);
  }
  memset(buffer, 0, sizeof(buffer));
  if (Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_TX_POWER_LEVEL, buffer, sizeof(buffer))) {
    beacon["p"] = (int8_t)buffer[0];
  }
  uint8_t uuidBuffer[32];
  int uuidLen = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE, uuidBuffer, sizeof(uuidBuffer));
  if (uuidLen > 0) {
    JsonArray uuids = beacon.createNestedArray("u6");
    for(int i = 0; i < uuidLen; i += 2) {
      uint16_t uuid16;
      memcpy(&uuid16, uuidBuffer + i, 2);
      uuids.add(String(uuid16, HEX));
    }
  }
  memset(uuidBuffer, 0, sizeof(uuidBuffer));
  uuidLen = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE, uuidBuffer, sizeof(uuidBuffer));
  if (uuidLen > 0) {
    JsonArray uuids = beacon.createNestedArray("u8");
    for(int i = 0; i < uuidLen; i += 16) {
      String uuid128 = "";
      for(int j = 0; j < 16 && (i + j) < uuidLen; j++) {
        if (j > 0 && (j % 2 == 0)) uuid128 += "-";
        uuid128 += String(uuidBuffer[i + j], HEX);
      }
      uuids.add(uuid128);
    }
  }
  memset(uuidBuffer, 0, sizeof(uuidBuffer));
  uuidLen = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_SERVICE_DATA, uuidBuffer, sizeof(uuidBuffer));
  if (uuidLen > 0) {
    JsonArray serviceData = beacon.createNestedArray("sd");
    for(int i = 0; i < uuidLen; i++) {
      serviceData.add(uuidBuffer[i]);
    }
  }
  int manuLen = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, buffer, sizeof(buffer));
  if (manuLen > 0) {
    JsonArray manuData = beacon.createNestedArray("md");
    for(int i = 0; i < manuLen; i++) {
      manuData.add(buffer[i]);
    }
  }
  memset(buffer, 0, sizeof(buffer));
  if (Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_FLAGS, buffer, sizeof(buffer))) {
    beacon["f"] = buffer[0];
  }
  memset(buffer, 0, sizeof(buffer));
  if (Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_APPEARANCE, buffer, sizeof(buffer))) {
    uint16_t appearance;
    memcpy(&appearance, buffer, 2);
    beacon["a"] = appearance;
  }
  bleBeaconsFound++;
  Bluefruit.Scanner.resume();
}

bool initializeBLE() {
  rtt.println("Initializing BLE...");
  Bluefruit.begin(0, 1);
  Bluefruit.setTxPower(4);
  Bluefruit.setName("HGD4");
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
  bleScanDoc.clear();
  bleBeacons = bleScanDoc.createNestedArray("beacons");
  if (bleBeacons.isNull()) {
    rtt.println("Failed to create beacons array");
    return;
  }
  bleScanning = true;
  bleScanStartTime = millis();
  bleBeaconsFound = 0;
  Bluefruit.Scanner.setRxCallback(ble_scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.filterRssi(-80);
  Bluefruit.Scanner.setInterval(160, 80);       // in units of 0.625 ms
  Bluefruit.Scanner.useActiveScan(true);        // Request scan response data
  Bluefruit.Scanner.start(0);                   // 0 = Don't stop scanning after n seconds
  rtt.println("Scanning started...");
  unsigned long scanStartTime = millis();
  while (bleScanning && 
         bleBeaconsFound < deviceConfig.ble_scan_max_beacons && 
         (millis() - scanStartTime) < deviceConfig.ble_scan_duration) {
    delay(10);
    if (millis() - bleScanStartTime > deviceConfig.ble_scan_duration) {
      rtt.println("Scan duration expired");
      break;
    }
  }
  Bluefruit.Scanner.stop();
  bleScanning = false;
  bleScanDoc["sb"] = millis() - bleScanStartTime;
  bleScanDoc["mb"] = deviceConfig.ble_scan_max_beacons;
  String bleScanJson;
  serializeJson(bleScanDoc, bleScanJson);
  sensorData.ble_scan_data = bleScanJson;
  rtt.print("Scan completed. Found "); rtt.print(bleBeaconsFound); rtt.println(" beacons");
  rtt.print("JSON length: "); rtt.println(bleScanJson.length());
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
  rtt.print("C, H="); rtt.print(sensorData.case_humidity, 1);
  rtt.print("%RH, P="); rtt.print(sensorData.pressure, 1);
  rtt.print("hPa, GPS="); rtt.print(sensorData.gps_used_satellites);
  rtt.println(" sats");
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
  delay(20);
  Wire.begin();
  }
}

void wd_handler() {
  digitalWrite(DONE,HIGH);
  delayMicroseconds(160);
  digitalWrite(DONE,LOW);
  wdcounter++;
}

void collectAllSensorData(){
  sensorData.timestamp = millis();
  sensorData.wakeup_reason = NRF_POWER->RESETREAS;
  sensorData.watchdog_counter = wdcounter;
  sensorData.battery_voltage = readBatteryVoltage();
  if(has_veml60350)sensorData.lux = readLightSensor(0.5, 200);
  if(has_ens210){
  float temp, hum;
  if (readTemperatureHumidity(temp, hum)) {
    sensorData.case_temperature = temp;
    sensorData.case_humidity = hum;
  }
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
  SerialAT.flush();
  SerialAT.end();
  rtt.println("Powering down modem...");
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
  delay(200);
}

void gpioinit(){
  pinMode(PWR_LATCH , OUTPUT);
  digitalWrite(PWR_LATCH , LOW);
  pinMode(WAKE, INPUT);
  pinMode(DONE, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(WAKE), wd_handler, RISING);
  Wire.begin();
  pinMode(SENSOR_PWR, OUTPUT);
  digitalWrite(SENSOR_PWR, HIGH);
  pinMode(RED_LED,OUTPUT);
  pinMode(GREEN_LED,OUTPUT);
  digitalWrite(RED_LED,HIGH);
  digitalWrite(GREEN_LED,HIGH);
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
  delay(20);
  for (size_t i = 0; i < 10; i++)
  {
    digitalWrite(DONE,HIGH);
    delay(2);
    digitalWrite(DONE,LOW);
    delay(2);
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
  if(has_lpS22hb){
  barometricSensor.init();
  pressure = barometricSensor.readPressureHPA();
  sensorTemp = barometricSensor.readTemperature();
  return (pressure != 0.0 || sensorTemp != 0.0);
  }
  if(has_ens220){
    ens220.begin(&Wire, 0x20);
    ens220.singleShotMeasure(ENS220_SENSOR_TEMPERATURE_AND_PRESSURE);
    ens220.waitSingleShot();
    ens220.update();
    pressure = ens220.getPressureHectoPascal();
    sensorTemp = ens220.getTempCelsius();
    return (pressure != 0.0 || sensorTemp != 0.0);
  }
  return false;
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
  acc1.enableDRDY(false);
  acc2.enableDRDY(false); 
  SPI.end();
  return true;
}

bool readGPS(float& latitude, float& longitude, float& speed, float& altitude, int& visibleSatellites, int& usedSatellites, float& accuracy) {
  if (!modemon) {
    return false;
  }
  unsigned long startTime = millis();
  modem.enableGPS();
  int gps_year, gps_month, gps_day, gps_hour, gps_minute, gps_second;
  for (int8_t i = 3; i; i--) {
    if (millis() - startTime > deviceConfig.gps_timeout) {
      rtt.println("GPS overall timeout reached");
      modem.disableGPS();
      return false;
    }
    rtt.println("Requesting current GPS/GNSS/GLONASS location");
    if (modem.getGPS(&latitude, &longitude, &speed, &altitude,
                     &visibleSatellites, &usedSatellites, &accuracy, 
                     &gps_year, &gps_month, &gps_day, &gps_hour, &gps_minute, &gps_second)) {
      modem.disableGPS();
      return true;
    } else {
      rtt.println("Couldn't get GPS/GNSS/GLONASS location");
      delay(2000);
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

bool validateModemConnection() {
  if (!modemInitialized) {
    return false;
  }
  if (!modem.isNetworkConnected()) {
    rtt.println("Network connection lost, reconnecting...");
    if (!connectToNetwork()) {
      return false;
    }
  }
  return true;
}

bool sendSensorDataToServer(const SensorData& data, const ServerConfig& config) {
  rtt.println("Starting HTTP transmission...");
  if (!modemInitialized) {
    rtt.println("Modem not initialized");
    return false;
  }
  if (!validateModemConnection()) {
    rtt.println("Modem connection validation failed");
    return false;
  }
  if (!modem.isGprsConnected()) {
    rtt.println("Not connected to network");
    return false;
  }
  for (int attempt = 1; attempt <= 3; attempt++) {
    rtt.print("HTTP attempt "); rtt.print(attempt); rtt.println("/3");
    TinyGsmClient client(modem);
    HttpClient http(client, config.server, config.port);
    http.setTimeout(60000);
    DynamicJsonDocument doc(32000);
    doc["ts"] = data.timestamp;
    if(has_veml60350)doc["lx"] = data.lux;
    doc["bat"] = data.battery_voltage;
    if(has_ens210){
    doc["tmp"] = data.case_temperature;
    doc["hum"] = data.case_humidity;
    }
    if(has_ens220 || has_lpS22hb){
    doc["prs"] = data.pressure;
    doc["pt"] = data.pressure_sensor_temp;
    }
    doc["a1x"] = data.acc1_x;
    doc["a1y"] = data.acc1_y;
    doc["a1z"] = data.acc1_z;
    doc["a2x"] = data.acc2_x;
    doc["a2y"] = data.acc2_y;
    doc["a2z"] = data.acc2_z;
    if(data.gps_visible_satellites > 1){
    doc["glt"] = data.gps_latitude;
    doc["gln"] = data.gps_longitude;
    doc["gsp"] = data.gps_speed;
    doc["gal"] = data.gps_altitude;
    doc["gsv"] = data.gps_visible_satellites;
    doc["gsu"] = data.gps_used_satellites;
    doc["gac"] = data.gps_accuracy;
    }
    doc["wkr"] = data.wakeup_reason;
    doc["wdc"] = data.watchdog_counter;
    doc["mid"] = data.modem_id;
    doc["mt"] = data.modem_temperature;
    doc["sid"] = data.sim_card_id;
    doc["nn"] = data.network_name;
    doc["nid"] = data.network_id;
    doc["lt"] = data.link_type;
    doc["sq"] = data.signal_quality;
    doc["ss"] = data.signal_strength;
    doc["nrs"] = data.network_registration_status;
    doc["nb"] = data.network_band;
    doc["nc"] = data.network_channel;
    doc["noc"] = data.network_operator_code;
    if (data.wifi_scan_data.length() > 0) {
      DynamicJsonDocument wifiDoc(16000);
      DeserializationError wifiError = deserializeJson(wifiDoc, data.wifi_scan_data);
      if (!wifiError) {
        doc["wifi"] = wifiDoc;
      } else {
        rtt.println("Failed to parse WiFi scan data JSON");
        doc["wifi"] = data.wifi_scan_data; // Fallback to string if parsing fails
      }
    }
    if (data.ble_scan_data.length() > 0) {
      DynamicJsonDocument bleDoc(16000);
      DeserializationError bleError = deserializeJson(bleDoc, data.ble_scan_data);
      if (!bleError) {
        doc["ble"] = bleDoc;
      } else {
        rtt.println("Failed to parse BLE scan data JSON");
        doc["ble"] = data.ble_scan_data; // Fallback to string if parsing fails
      }
    }
    size_t jsonSize = measureJson(doc);
    rtt.print("JSON size: "); rtt.print(jsonSize); rtt.println(" bytes");
    String jsonData;
    jsonData.reserve(jsonSize + 100); // Reserve memory to avoid fragmentation
    if (serializeJson(doc, jsonData) == 0) {
      rtt.println("JSON serialization failed");
      http.stop();
      return false;
    }
    rtt.print("Final JSON length: "); rtt.println(jsonData.length());
    if (!sendLargeJsonData(http, doc, config.endpoint)) {
      rtt.println("Failed to send JSON data");
      http.stop();
      if (attempt < 3) {
        delay(2000);
        continue;
      }
      return false;
    }
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
    String responseBody = readCompleteHttpResponse(http, 30000);
    rtt.print("Response length: "); rtt.println(responseBody.length());
    http.stop();
    if (statusCode >= 200 && statusCode < 300) {
      lastTransmissionTime = millis();
      rtt.print("HTTP success: "); rtt.println(statusCode);
      if (responseBody.indexOf('{') != -1) {
        DynamicJsonDocument responseDoc(16000);
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
        delay(2000);
        continue;
      }
    }
  }
  rtt.println("All HTTP attempts failed");
  return false;
}

bool sendSensorDataToServer(const SensorData& data) {
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
      }
      if (millis() - gpsStartTime > deviceConfig.gps_timeout) {
        rtt.println("GPS timeout, proceeding without GPS");
      }
    }
    readNetworkInfo();
    if (sendSensorDataToServer(sensorData)) {
      rtt.println("Data transmission successful");
    } else {
      rtt.println("Data transmission failed");
      blinkLED(4, 200, 400, false);
    }
      if (shouldKeepModemPowered()) {
        rtt.println("Keeping modem powered on due to motion or configuration");
      } else {
        rtt.println("Shutting down modem between loops...");
        shutdownModem();
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
    int pos1 = data.indexOf(",");
    int pos2 = data.indexOf(",", pos1 + 1);
    int pos3 = data.indexOf(",", pos2 + 1);
    if (pos1 > 0 && pos2 > pos1 && pos3 > pos2) {
      String act = data.substring(0, pos1);
      act.replace("\"", "");
      String oper = data.substring(pos1 + 1, pos2);
      oper.replace("\"", "");
      String band = data.substring(pos2 + 1, pos3);
      band.replace("\"", "");
      String channel = data.substring(pos3 + 1);
      channel.replace("\"", "");
      channel.replace("\r", "");
      channel.replace("\n", "");
      sensorData.network_operator_code = oper;
      sensorData.network_band = band;
      sensorData.network_channel = channel.toInt();
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
    if (sensorData.network_registration_status == 1 || sensorData.network_registration_status == 5) {
      if (sensorData.signal_quality > 0 && sensorData.signal_quality <= 31) {
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
  while (modem.stream.available()) { modem.stream.read(); }
  modem.sendAT(GF("+QFOPEN="), filename, GF(","), mode);
  String response = modem.stream.readString();
  if (response.indexOf("+QFOPEN:") != -1) {
    int handle_start = response.indexOf(":") + 1;
    String handle_str = response.substring(handle_start);
    handle_str.trim();
    currentFileHandle = handle_str.toInt();
    if (currentFileHandle >= 0) { // Note: 0 is a valid file handle
        fileOpen = true;
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
  while (modem.stream.available()) { modem.stream.read(); }
  modem.sendAT(GF("+QFWRITE="), currentFileHandle, GF(","), data.length());
  delay(100);
  String response = modem.stream.readString();
  if (response.indexOf("CONNECT") != -1) {
    delay(50); // Longer delay to ensure modem is ready
    modem.stream.print(data);
    delay(100);
    response = modem.stream.readString();
    if (response.indexOf("OK") != -1) {
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
  while (modem.stream.available()) { modem.stream.read(); }
  modem.sendAT(GF("+QFPOSITION="), currentFileHandle);
  delay(100);
  String response = modem.stream.readString();
  if (response.indexOf("+QFPOSITION:") != -1) {
    int pos_start = response.indexOf(":") + 1;
    String pos_str = response.substring(pos_start);
    pos_str.trim();
    position = pos_str.toInt();
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
  while (modem.stream.available()) { modem.stream.read(); }
  modem.sendAT(GF("+QFSEEK="), currentFileHandle, GF(","), offset, GF(","), origin);
  delay(100);
  String response = modem.stream.readString();
  if (response.indexOf("OK") != -1) {
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
      unsigned int dataEnd = response.indexOf("OK", actualDataStart);
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

bool saveConfigToModem(const ModemConfig& config) {
  rtt.println("Saving configuration to modem...");
  DynamicJsonDocument doc(8000);
  doc["dn"] = config.device_name;
  doc["ti"] = config.transmission_interval;
  doc["tim"] = config.transmission_interval_motion;
  doc["eg"] = config.enable_gps;
  doc["gt"] = config.gps_timeout;
  doc["su"] = config.server_url;
  doc["se"] = config.server_endpoint;
  doc["sp"] = config.server_port;
  doc["apn"] = config.apn;
  doc["un"] = config.username;
  doc["pw"] = config.password;
  doc["es"] = config.enable_storage;
  doc["msf"] = config.max_stored_files;
  doc["wse"] = config.wifi_scan_enabled;
  doc["wsmn"] = config.wifi_scan_max_networks;
  doc["bse"] = config.ble_scan_enabled;
  doc["bsmb"] = config.ble_scan_max_beacons;
  doc["bsd"] = config.ble_scan_duration;
  doc["msbl"] = config.modem_shutdown_between_loops;
  doc["mde"] = config.motion_detection_enabled;
  doc["mt"] = config.motion_threshold;
  doc["msom"] = config.modem_shutdown_on_motion;
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
  if (!modemFileOpen(filename, 0)) {
    rtt.println("Failed to open config file for reading");
    return false;
  }
  String configData;
  if (!modemFileRead(configData, 1024)) {
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
  if (doc.containsKey("dn")) {
    config.device_name = doc["dn"].as<String>();
  }
  if (doc.containsKey("ti")) {
    config.transmission_interval = doc["ti"].as<unsigned long>();
  }
  if (doc.containsKey("tim")) {
    config.transmission_interval_motion = doc["tim"].as<unsigned long>();
  }
  if (doc.containsKey("eg")) {
    config.enable_gps = doc["eg"].as<bool>();
  }
  if (doc.containsKey("gt")) {
    config.gps_timeout = doc["gt"].as<unsigned long>();
  }
  if (doc.containsKey("su")) {
    config.server_url = doc["su"].as<String>();
  }
  if (doc.containsKey("se")) {
    config.server_endpoint = doc["se"].as<String>();
  }
  if (doc.containsKey("sp")) {
    config.server_port = doc["sp"].as<int>();
  }
  if (doc.containsKey("apn")) {
    config.apn = doc["apn"].as<String>();
  }
  if (doc.containsKey("un")) {
    config.username = doc["un"].as<String>();
  }
  if (doc.containsKey("pw")) {
    config.password = doc["pw"].as<String>();
  }
  if (doc.containsKey("es")) {
    config.enable_storage = doc["es"].as<bool>();
  }
  if (doc.containsKey("msf")) {
    config.max_stored_files = doc["msf"].as<int>();
  }
  if (doc.containsKey("wse")) {
    config.wifi_scan_enabled = doc["wse"].as<bool>();
  }
  if (doc.containsKey("wsmn")) {
    config.wifi_scan_max_networks = doc["wsmn"].as<int>();
  }
  if (doc.containsKey("bse")) {
    config.ble_scan_enabled = doc["bse"].as<bool>();
  }
  if (doc.containsKey("bsmb")) {
    config.ble_scan_max_beacons = doc["bsmb"].as<int>();
  }
  if (doc.containsKey("bsd")) {
    config.ble_scan_duration = doc["bsd"].as<unsigned long>();
  }
  if (doc.containsKey("msbl")) {
    config.modem_shutdown_between_loops = doc["msbl"].as<bool>();
  }
  if (doc.containsKey("mde")) {
    config.motion_detection_enabled = doc["mde"].as<bool>();
  }
  if (doc.containsKey("mt")) {
    config.motion_threshold = doc["mt"].as<float>();
  }
  if (doc.containsKey("msom")) {
    config.modem_shutdown_on_motion = doc["msom"].as<bool>();
  }
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
    } else {
      rtt.println("Failed to save test config");
    }
  }
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
  powerupESP();
  delay(200);
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
  Serial2.flush();
  Serial2.end();
  pinMode(ESP_GPIO0, INPUT);
  digitalWrite(ESP_PWR, LOW);
  espon = false;
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
    } else {
      rtt.println("WiFi scan failed or incomplete response");
      rtt.println("Attempting to parse anyway:");
      sensorData.wifi_scan_data = parseCWLAPData(response);
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
    if (!foundOK && !foundError) {
      delayMicroseconds(100);
    }
  }
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
  modem.sendAT(GF("+QFLDS=\"UFS\""));
  delay(100); // Give modem time to process
  String response = modem.stream.readString();
  if (response.indexOf("+QFLDS:") != -1) {
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
  modem.sendAT(GF("+QFLDS=\"EUFS\""));
  delay(100); // Give modem time to process
  if (response.indexOf("+QFLDS:") != -1) {
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

String parseCWLAPData(const String& cwlapResponse) {
  rtt.print("Input length: "); rtt.println(cwlapResponse.length());
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
  int networksAdded = 0;
  size_t lineStart = 0;
  for (size_t i = 0; i < cwlapResponse.length(); i++) {
    char c = cwlapResponse.charAt(i);
    if (c == '\n' || c == '\r') {
      if (i > lineStart) {
        String line = cwlapResponse.substring(lineStart, i);
        line.trim();
        if (line.startsWith("+CWLAP:(")) {
          if (line.length() < 9) {
            rtt.println("parseCWLAPData: Line too short, skipping");
            delay(10);
            lineStart = i + 1;
            continue;
          }
          String data = line.substring(8, line.length() - 1);
          String values[12]; // Max 12 values per network
          int valueCount = 0;
          bool inQuotes = false;
          size_t valueStart = 0;
          for (size_t j = 0; j < data.length() && valueCount < 12; j++) {
            char dataChar = data.charAt(j);
            
            if (dataChar == '"') {
              inQuotes = !inQuotes;
            } else if (dataChar == ',' && !inQuotes) {
              if (j > valueStart) {
                values[valueCount] = data.substring(valueStart, j);
                valueCount++;
              }
              valueStart = j + 1;
            }
          }
          if (valueStart < data.length()) {
            values[valueCount] = data.substring(valueStart);
            valueCount++;
          }
          if (valueCount >= 4) {
            JsonObject network = networks.createNestedObject();
            if (network.isNull()) {
              rtt.println("parseCWLAPData: Failed to create network object");
              continue;
            }
            int enc = values[0].toInt();
            network["e"] = enc;
            String ssid = values[1];
            if (ssid.startsWith("\"") && ssid.endsWith("\"") && ssid.length() > 2) {
              ssid = ssid.substring(1, ssid.length() - 1);
            }
            network["s"] = ssid;
            int rssi = values[2].toInt();
            network["r"] = rssi;
            String mac = values[3];
            if (mac.startsWith("\"") && mac.endsWith("\"") && mac.length() > 2) {
              mac = mac.substring(1, mac.length() - 1);
            }
            mac.replace(":", "");
            network["m"] = mac;
            if (valueCount > 4) {
              network["c"] = values[4].toInt();
            }
            if (valueCount > 5) {
              network["a"] = values[5].toInt();
            }
            if (valueCount > 6 && values[6].toInt() != 0) network["6"] = values[6].toInt();
            if (valueCount > 7 && values[7].toInt() != 0) network["7"] = values[7].toInt();
            if (valueCount > 8 && values[8].toInt() != 0) network["8"] = values[8].toInt();
            if (valueCount > 9 && values[9].toInt() != 0) network["9"] = values[9].toInt();
            if (valueCount > 10 && values[10].toInt() != 0) network["1"] = values[10].toInt();
            networksAdded++;
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
  rtt.print("parseCWLAPData: Final network count: "); rtt.println(networks.size());
  String jsonOutput = "";
  serializeJson(doc, jsonOutput);
  rtt.print("parseCWLAPData: JSON output length: "); rtt.println(jsonOutput.length());
  return jsonOutput;
}

bool sendLargeJsonData(HttpClient& http, const DynamicJsonDocument& doc, const String& endpoint) {
  String jsonData;
  jsonData.reserve(measureJson(doc) + 100);
  if (serializeJson(doc, jsonData) == 0) {
    rtt.println("JSON serialization failed");
    return false;
  }
    http.beginRequest();
    http.post(endpoint);
    http.sendHeader("Content-Type", "application/json");
    http.sendHeader("Transfer-Encoding", "chunked");
    http.sendHeader("Accept", "application/json");
    http.sendHeader("User-Agent", "NRF-Firmware/1.0");
    http.sendHeader("Connection", "close");
    const int chunkSize = 512;
    int totalSent = 0;
    for (size_t i = 0; i < jsonData.length(); i += chunkSize) {
      size_t chunkLength = min(chunkSize, jsonData.length() - i);
      String chunk = jsonData.substring(i, i + chunkLength);
      http.print(String(chunkLength, HEX));
      http.print("\r\n");
      if (http.print(chunk) != chunkLength) {
        rtt.println("Failed to send chunk");
        return false;
      }
      http.print("\r\n");
      totalSent += chunkLength;
      delay(20);
    }
    http.print("0\r\n\r\n");
    rtt.print("Sent "); rtt.print(totalSent); rtt.println(" bytes in chunks");
  http.endRequest();
  return true;
}

bool waitForNextTransmission(unsigned long interval) {
  unsigned long currentTime = millis();
  unsigned long effectiveInterval = interval;
  if (deviceConfig.motion_detection_enabled && globalMotionDetected) {
    effectiveInterval = deviceConfig.transmission_interval_motion;
  }
  if (currentTime - lastWaitCheck >= effectiveInterval) {
    lastWaitCheck = currentTime;
    return true; // Time to transmit
  }
  return false; // Still waiting
}

void performBackgroundTasks() {
  if(digitalRead(PWR_SW_IN)){
    rtt.println("Button pressed");
    digitalWrite(GREEN_LED,LOW);
    delay(2000);
    if(digitalRead(PWR_SW_IN)){
      rtt.println("Button short press, powering down");
      digitalWrite(GREEN_LED,HIGH);
      digitalWrite(RED_LED,LOW);
      delay(10000);
      digitalWrite(PWR_LATCH,HIGH);
      delay(10000);
    }
    else{
      digitalWrite(GREEN_LED,HIGH);
      rtt.println("Button short press, transmitting now");
      lastWaitCheck = 0;
    }
  }
    if (readAccelerometers(currentAcc1X, currentAcc1Y, currentAcc1Z, currentAcc2X, currentAcc2Y, currentAcc2Z)) {
      float acc1Diff = sqrt(pow(currentAcc1X - lastAcc1X, 2) + pow(currentAcc1Y - lastAcc1Y, 2) + pow(currentAcc1Z - lastAcc1Z, 2));
      float acc2Diff = sqrt(pow(currentAcc2X - lastAcc2X, 2) + pow(currentAcc2Y - lastAcc2Y, 2) + pow(currentAcc2Z - lastAcc2Z, 2));
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
  delay(100);
}

bool parseServerResponse(const String& response, JsonDocument& doc) {
  int jsonStart = response.indexOf('{');
  if (jsonStart == -1) {
    rtt.println("No JSON data found in response");
    return false;
  }
  String jsonData = response.substring(jsonStart);
  DeserializationError error = deserializeJson(doc, jsonData);
  if (error) {
    rtt.print("JSON parsing failed: ");
    rtt.println(error.c_str());
    return false;
  }
  return true;
}

String readCompleteHttpResponse(HttpClient& http, unsigned long timeout) {
  String response;
  response.reserve(8192);
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
      if (!chunkedTransfer && response.indexOf("Transfer-Encoding: chunked") != -1) {
        chunkedTransfer = true;
        rtt.println("Detected chunked transfer encoding");
      }
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
      if (!readingBody && response.indexOf("\r\n\r\n") != -1) {
        readingBody = true;
        rtt.println("Headers complete, reading body");
      }
      if (chunkedTransfer && readingBody) {
        if (response.indexOf("0\r\n\r\n") != -1) {
          rtt.println("Chunked transfer complete");
          break;
        }
      }
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
      if (readingBody && (millis() - lastDataTime) > 2000 && response.length() > 100) {
        rtt.println("No more data for 2 seconds, assuming response complete");
        break;
      }
      delay(10); // Small delay to prevent busy waiting
    }
  }
  return response;
}

void shutdownModem() {
  rtt.println("Performing complete modem shutdown...");
  if (modemInitialized && modem.isGprsConnected())disconnectFromNetwork();
  if (fileOpen)modemFileClose();
  if (modemon)powerdownmodem();
  modemInitialized = false;
  rtt.println("Modem shutdown complete");
}

bool initializeModemIfNeeded() {
  if (!modemInitialized) {
    rtt.println("Modem not initialized, initializing...");
    return initializeModem();
  }
  return true;
}

bool shouldKeepModemPowered() {
  if (!deviceConfig.modem_shutdown_between_loops) {
    return true; // Always keep modem on if shutdown is disabled
  }
  if (deviceConfig.modem_shutdown_on_motion && globalMotionDetected) {
    return true; // Keep modem on if motion detected and configured to do so
  }
  return false;
}

void checkAllInterruptFlags() {
    rtt.println("=== Interrupt Status Check ===");
    bool hasPending = false;
    for (int i = 0; i < 8; i++) {
        if (NVIC->ISPR[i] != 0) {
            rtt.print("NVIC pending in word "); rtt.print(i); rtt.print(": 0x"); rtt.println(NVIC->ISPR[i], HEX);
            uint32_t pending = NVIC->ISPR[i];
            for (int bit = 0; bit < 32; bit++) {
                if (pending & (1 << bit)) {
                    int irq_num = i * 32 + bit;
                    rtt.print("    IRQ "); rtt.print(irq_num); rtt.print(" (bit "); rtt.print(bit); rtt.print("): ");
                    switch (irq_num) {
                        case 38: rtt.println("UARTE0"); break;
                        case 39: rtt.println("UARTE1"); break;
                        case 40: rtt.println("UARTE2"); break;
                        case 41: rtt.println("UARTE3"); break;
                        case 42: rtt.println("SWI2 (BLE)"); break;
                        case 43: rtt.println("SWI3 (BLE)"); break;
                        case 44: rtt.println("SWI4 (BLE)"); break;
                        case 45: rtt.println("SWI5 (BLE)"); break;
                        case 6: rtt.println("GPIOTE"); break;
                        case 7: rtt.println("SAADC"); break;
                        case 8: rtt.println("TIMER0"); break;
                        case 9: rtt.println("TIMER1"); break;
                        case 10: rtt.println("TIMER2"); break;
                        case 11: rtt.println("RTC0"); break;
                        case 12: rtt.println("TEMP"); break;
                        case 13: rtt.println("RNG"); break;
                        case 14: rtt.println("ECB"); break;
                        case 15: rtt.println("CCM_AAR"); break;
                        case 16: rtt.println("WDT"); break;
                        case 17: rtt.println("RTC1"); break;
                        case 18: rtt.println("QDEC"); break;
                        case 19: rtt.println("COMP_LPCOMP"); break;
                        case 20: rtt.println("SWI0_EGU0"); break;
                        case 21: rtt.println("SWI1_EGU1"); break;
                        case 22: rtt.println("SWI2_EGU2"); break;
                        case 23: rtt.println("SWI3_EGU3"); break;
                        case 24: rtt.println("SWI4_EGU4"); break;
                        case 25: rtt.println("SWI5_EGU5"); break;
                        case 26: rtt.println("TIMER3"); break;
                        case 27: rtt.println("TIMER4"); break;
                        case 28: rtt.println("PWM0"); break;
                        case 29: rtt.println("PDM"); break;
                        case 30: rtt.println("MWU"); break;
                        case 31: rtt.println("PWM1"); break;
                        case 32: rtt.println("PWM2"); break;
                        case 33: rtt.println("SPIM2_SPIS2_SPI2"); break;
                        case 34: rtt.println("RTC2"); break;
                        case 35: rtt.println("I2S"); break;
                        case 36: rtt.println("FPU"); break;
                        case 37: rtt.println("USBD"); break;
                        default: rtt.println("Unknown peripheral"); break;
                    }
                }
            }
            hasPending = true;
        }
    }
    if(hasPending){
    if (NRF_UARTE0->INTENSET != 0) {
        rtt.print("UARTE0 interrupts enabled: 0x"); rtt.println(NRF_UARTE0->INTENSET, HEX);
        rtt.println("UARTE0 Event Analysis:");
        if (NRF_UARTE0->EVENTS_ENDRX) {
            rtt.println("  - ENDRX event: PENDING");
        }
        if (NRF_UARTE0->EVENTS_ENDTX) {
            rtt.println("  - ENDTX event: PENDING");
        }
        if (NRF_UARTE0->EVENTS_ERROR) {
            rtt.println("  - ERROR event: PENDING");
            rtt.print("    Error source: 0x"); rtt.println(NRF_UARTE0->ERRORSRC, HEX);
        }
        if (NRF_UARTE0->EVENTS_RXTO) {
            rtt.println("  - RXTO event: PENDING");
        }
        if (NRF_UARTE0->EVENTS_RXSTARTED) {
            rtt.println("  - RXSTARTED event: PENDING");
        }
        if (NRF_UARTE0->EVENTS_TXSTARTED) {
            rtt.println("  - TXSTARTED event: PENDING");
        }
        if (NRF_UARTE0->EVENTS_TXSTOPPED) {
            rtt.println("  - TXSTOPPED event: PENDING");
        }
        rtt.print("  Enabled interrupts: ");
        if (NRF_UARTE0->INTENSET & UARTE_INTENSET_ENDRX_Msk) rtt.print("ENDRX ");
        if (NRF_UARTE0->INTENSET & UARTE_INTENSET_ENDTX_Msk) rtt.print("ENDTX ");
        if (NRF_UARTE0->INTENSET & UARTE_INTENSET_ERROR_Msk) rtt.print("ERROR ");
        if (NRF_UARTE0->INTENSET & UARTE_INTENSET_RXTO_Msk) rtt.print("RXTO ");
        if (NRF_UARTE0->INTENSET & UARTE_INTENSET_RXSTARTED_Msk) rtt.print("RXSTARTED ");
        if (NRF_UARTE0->INTENSET & UARTE_INTENSET_TXSTARTED_Msk) rtt.print("TXSTARTED ");
        if (NRF_UARTE0->INTENSET & UARTE_INTENSET_TXSTOPPED_Msk) rtt.print("TXSTOPPED ");
        rtt.println();
        rtt.print("  Pending events: ");
        if (NRF_UARTE0->EVENTS_ENDRX) rtt.print("ENDRX ");
        if (NRF_UARTE0->EVENTS_ENDTX) rtt.print("ENDTX ");
        if (NRF_UARTE0->EVENTS_ERROR) rtt.print("ERROR ");
        if (NRF_UARTE0->EVENTS_RXTO) rtt.print("RXTO ");
        if (NRF_UARTE0->EVENTS_RXSTARTED) rtt.print("RXSTARTED ");
        if (NRF_UARTE0->EVENTS_TXSTARTED) rtt.print("TXSTARTED ");
        if (NRF_UARTE0->EVENTS_TXSTOPPED) rtt.print("TXSTOPPED ");
        rtt.println();
    }
    if (NRF_UARTE1->INTENSET != 0) {
        rtt.print("UARTE1 interrupts enabled: 0x"); rtt.println(NRF_UARTE1->INTENSET, HEX);
        rtt.println("UARTE1 Event Analysis:");
        if (NRF_UARTE1->EVENTS_ENDRX) rtt.println("  - ENDRX event: PENDING");
        if (NRF_UARTE1->EVENTS_ENDTX) rtt.println("  - ENDTX event: PENDING");
        if (NRF_UARTE1->EVENTS_ERROR) rtt.println("  - ERROR event: PENDING");
        if (NRF_UARTE1->EVENTS_RXTO) rtt.println("  - RXTO event: PENDING");
        if (NRF_UARTE1->EVENTS_RXSTARTED) rtt.println("  - RXSTARTED event: PENDING");
        if (NRF_UARTE1->EVENTS_TXSTARTED) rtt.println("  - TXSTARTED event: PENDING");
        if (NRF_UARTE1->EVENTS_TXSTOPPED) rtt.println("  - TXSTOPPED event: PENDING");
    }
    if (NRF_TWIM0->INTENSET != 0) {
        rtt.print("TWIM0 interrupts enabled: 0x"); rtt.println(NRF_TWIM0->INTENSET, HEX);
    }
    if (NRF_GPIOTE->INTENSET != 0) {
        rtt.print("GPIOTE interrupts enabled: 0x"); rtt.println(NRF_GPIOTE->INTENSET, HEX);
        if (hasPending) {
            rtt.println("GPIOTE Event Analysis:");
            for (int ch = 0; ch < 8; ch++) {
                if (NRF_GPIOTE->EVENTS_IN[ch]) {
                    rtt.print("  - Channel "); rtt.print(ch); rtt.println(" event: PENDING");
                    uint32_t config = NRF_GPIOTE->CONFIG[ch];
                    rtt.print("    Mode: ");
                    if ((config & GPIOTE_CONFIG_MODE_Msk) == GPIOTE_CONFIG_MODE_Disabled) {
                        rtt.print("Disabled");
                    } else if ((config & GPIOTE_CONFIG_MODE_Msk) == GPIOTE_CONFIG_MODE_Event) {
                        rtt.print("Event");
                    } else if ((config & GPIOTE_CONFIG_MODE_Msk) == GPIOTE_CONFIG_MODE_Task) {
                        rtt.print("Task");
                    }
                    rtt.print(", Pin: "); rtt.print((config & GPIOTE_CONFIG_PSEL_Msk) >> GPIOTE_CONFIG_PSEL_Pos);
                    
                    rtt.print(", Polarity: ");
                    if ((config & GPIOTE_CONFIG_POLARITY_Msk) == GPIOTE_CONFIG_POLARITY_None) {
                        rtt.print("None");
                    } else if ((config & GPIOTE_CONFIG_POLARITY_Msk) == GPIOTE_CONFIG_POLARITY_LoToHi) {
                        rtt.print("LoToHi");
                    } else if ((config & GPIOTE_CONFIG_POLARITY_Msk) == GPIOTE_CONFIG_POLARITY_HiToLo) {
                        rtt.print("HiToLo");
                    } else if ((config & GPIOTE_CONFIG_POLARITY_Msk) == GPIOTE_CONFIG_POLARITY_Toggle) {
                        rtt.print("Toggle");
                    }
                    
                    rtt.print(", OutInit: ");
                    if ((config & GPIOTE_CONFIG_OUTINIT_Msk) == GPIOTE_CONFIG_OUTINIT_Low) {
                        rtt.print("Low");
                    } else {
                        rtt.print("High");
                    }
                    rtt.println();
                }
            }
            rtt.print("  Enabled channels: ");
            for (int ch = 0; ch < 8; ch++) {
                if (NRF_GPIOTE->INTENSET & (1 << ch)) {
                    rtt.print(ch); rtt.print(" ");
                }
            }
            rtt.println();
            rtt.print("  Pending channels: ");
            for (int ch = 0; ch < 8; ch++) {
                if (NRF_GPIOTE->EVENTS_IN[ch]) {
                    rtt.print(ch); rtt.print(" ");
                }
            }
            rtt.println();
        }
    }
    uint32_t evt_id;
    if (sd_evt_get(&evt_id) == NRF_SUCCESS) {
        rtt.print("SoftDevice event pending: 0x"); rtt.println(evt_id, HEX);
    } else {
        rtt.println("No SoftDevice events pending");
    }
    uint8_t ble_evt_buffer[sizeof(ble_evt_t)];
    uint16_t ble_evt_len = sizeof(ble_evt_buffer);
    if (sd_ble_evt_get(ble_evt_buffer, &ble_evt_len) == NRF_SUCCESS) {
        rtt.print("BLE event pending, length: "); rtt.println(ble_evt_len);
    } else {
        rtt.println("No BLE events pending");
    }
    }
    if (!hasPending) {
        rtt.println("No NVIC pending interrupts");
    }
    rtt.print("SysTick enabled: "); rtt.println((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) ? "Yes" : "No");
    rtt.print("SysTick pending: "); rtt.println((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) ? "Yes" : "No");
    rtt.print("RTC0 enabled: "); rtt.println(NRF_RTC0->INTENSET ? "Yes" : "No");
    rtt.print("RTC0 running: "); rtt.println(NRF_RTC0->TASKS_START ? "Yes" : "No");
    rtt.print("RTC0 prescaler: "); rtt.println(NRF_RTC0->PRESCALER);
    rtt.print("RTC0 compare[0]: "); rtt.println(NRF_RTC0->CC[0]);
    rtt.print("RTC1 enabled: "); rtt.println(NRF_RTC1->INTENSET ? "Yes" : "No");
    rtt.print("RTC1 running: "); rtt.println(NRF_RTC1->TASKS_START ? "Yes" : "No");
    rtt.print("RTC1 prescaler: "); rtt.println(NRF_RTC1->PRESCALER);
    rtt.print("TIMER0 enabled: "); rtt.println(NRF_TIMER0->INTENSET ? "Yes" : "No");
    rtt.print("TIMER0 running: "); rtt.println(NRF_TIMER0->TASKS_START ? "Yes" : "No");
    rtt.print("TIMER0 prescaler: "); rtt.println(NRF_TIMER0->PRESCALER);
    rtt.print("TIMER1 enabled: "); rtt.println(NRF_TIMER1->INTENSET ? "Yes" : "No");
    rtt.print("TIMER1 running: "); rtt.println(NRF_TIMER1->TASKS_START ? "Yes" : "No");
    rtt.print("TIMER2 enabled: "); rtt.println(NRF_TIMER2->INTENSET ? "Yes" : "No");
    rtt.print("TIMER2 running: "); rtt.println(NRF_TIMER2->TASKS_START ? "Yes" : "No");
    rtt.println("=== End Interrupt Check ===");
}

void scanI2C(){
    rtt.println("Scanning I2C bus for devices...");
    int deviceCount = 0;
    for (byte address = 8; address < 120; address++) {
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();
        if (error == 0) {
            deviceCount++;
            rtt.print(address);
            switch (address) {
                case 0x20: rtt.println("(ENS220 Pres)"); has_ens220 = true; break;
                case 0x29: rtt.println("(VEML6035 Light)"); has_veml60350 = true; break;
                case 0x43: rtt.println("(ENS210 Temp/Hum)"); has_ens210 = true; break;
                case 0x5D: rtt.println("(LPS22HB Pres)"); has_lpS22hb = true; break;
                default: rtt.println("(Unknown)"); break;
            }
        }
        delay(1);
        if (address % 32 == 0) {
            yield();
        }
    }
    rtt.print("I2C scan completed. Found "); rtt.print(deviceCount); rtt.println(" device(s).");
}