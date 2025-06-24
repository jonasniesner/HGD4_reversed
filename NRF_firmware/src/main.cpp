#include "main.h"

// Global configuration - will be loaded from modem storage
ModemConfig deviceConfig;

//set if config should be saved to modem
bool saveConfig = false;

void setup() {
  rtt.trimDownBufferFull();
  rtt.println("|---------Starting setup---------|");
  gpioinit(); 
  sensorData.wakeup_reason = NRF_POWER->RESETREAS;
  delay(200);
  initializeModem();
  blinkLED(1, 200, 200, true);
  //checkModemStorageSpace();
  LoadDeviceConfig();
  testESPCommunication();
  powerdownESP();
}

void loop() {
  rtt.println();
  rtt.println("|---------Starting loop---------|");
  sensorpwr(true);
  sampleallsensors();
  rtt.print("WD counter ");
  rtt.print(sensorData.watchdog_counter);
  rtt.print(" Wakeup reason: ");
  rtt.println(sensorData.wakeup_reason);
  displaySensorDataSummary();
  manageModemLifecycle();
  sensorpwr(false);
  delay(100000);
  rtt.println("Loop cycle completed.");
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
  if (modemInitialized) {
    rtt.print("Modem: ");
    rtt.print("ID="); rtt.print(sensorData.modem_id);
    rtt.print(", Temp="); rtt.print(sensorData.modem_temperature, 1);
    rtt.print("°C, SIM="); rtt.print(sensorData.sim_card_id);
    rtt.print(", Net="); rtt.print(sensorData.network_name);
    rtt.print(", Link="); rtt.print(sensorData.link_type);
    rtt.print(", Band="); rtt.print(sensorData.network_band);
    rtt.print(", Channel="); rtt.print(sensorData.network_channel);
    rtt.print(", Quality="); rtt.print(sensorData.signal_quality);
    rtt.print(", Strength="); rtt.print(sensorData.signal_strength);
    rtt.print(" dBm");
    rtt.println();
  }
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
  nrf_delay_us(160);
  digitalWrite(DONE,LOW);
  //digitalWrite(GREEN_LED,HIGH);
  wdcounter++;
}

void sampleallsensors(){
  collectAllSensorData();
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
  nrf_delay_us(500);
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
  lps22hb.Enable();
  lps22hb.GetPressure(&pressure);
  lps22hb.GetTemperature(&sensorTemp);
  lps22hb.end();
  return (pressure != 0.0 || sensorTemp != 0.0);
}

bool readAccelerometers(float& acc1_x, float& acc1_y, float& acc1_z, float& acc2_x, float& acc2_y, float& acc2_z) {
  if (!acc1.begin() || !acc2.begin()) {
    return false;
  }
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

bool sendSensorDataToServer(const SensorData& data, const ServerConfig& config) {
  if (!modemInitialized) {
    rtt.println("Modem not initialized");
    return false;
  }
  if (!modem.isGprsConnected()) {
    rtt.println("Not connected to network");
    return false;
  }
  
  TinyGsmClient client(modem);
  HttpClient http(client, config.server, config.port);
  
  // Create JSON document with estimated size
  StaticJsonDocument<1024> doc;
  
  // Build JSON object efficiently
  doc["timestamp"] = data.timestamp;
  doc["lux"] = data.lux;
  doc["battery"] = round(data.battery_voltage * 100) / 100.0; // Round to 2 decimal places
  doc["temp"] = round(data.case_temperature * 10) / 10.0; // Round to 1 decimal place
  doc["humidity"] = round(data.case_humidity * 10) / 10.0; // Round to 1 decimal place
  doc["pressure"] = round(data.pressure * 100) / 100.0; // Round to 2 decimal places
  doc["pressure_temp"] = round(data.pressure_sensor_temp * 100) / 100.0; // Round to 2 decimal places
  doc["acc1_x"] = data.acc1_x;
  doc["acc1_y"] = data.acc1_y;
  doc["acc1_z"] = data.acc1_z;
  doc["acc2_x"] = data.acc2_x;
  doc["acc2_y"] = data.acc2_y;
  doc["acc2_z"] = data.acc2_z;
  doc["gps_lat"] = round(data.gps_latitude * 100000000) / 100000000.0; // Round to 8 decimal places
  doc["gps_lon"] = round(data.gps_longitude * 100000000) / 100000000.0; // Round to 8 decimal places
  doc["gps_speed"] = data.gps_speed;
  doc["gps_alt"] = data.gps_altitude;
  doc["gps_sat_vis"] = data.gps_visible_satellites;
  doc["gps_sat_used"] = data.gps_used_satellites;
  doc["gps_accuracy"] = data.gps_accuracy;
  doc["wakeup_reason"] = data.wakeup_reason;
  doc["watchdog_counter"] = data.watchdog_counter;
  doc["modem_id"] = data.modem_id.length() > 0 ? data.modem_id : "unknown";
  doc["modem_temp"] = round(data.modem_temperature * 10) / 10.0; // Round to 1 decimal place
  doc["sim_id"] = data.sim_card_id.length() > 0 ? data.sim_card_id : "unknown";
  doc["network_name"] = data.network_name.length() > 0 ? data.network_name : "unknown";
  doc["network_id"] = data.network_id.length() > 0 ? data.network_id : "unknown";
  doc["link_type"] = data.link_type.length() > 0 ? data.link_type : "unknown";
  doc["signal_quality"] = data.signal_quality;
  doc["signal_strength"] = data.signal_strength;
  doc["network_registration_status"] = data.network_registration_status;
  doc["network_band"] = data.network_band.length() > 0 ? data.network_band : "unknown";
  doc["network_channel"] = data.network_channel;
  doc["network_operator_code"] = data.network_operator_code.length() > 0 ? data.network_operator_code : "unknown";
  
  // Serialize JSON to string efficiently
  String jsonData;
  serializeJson(doc, jsonData);
  
  rtt.print("JSON data length: "); rtt.println(jsonData.length());
  
  http.beginRequest();
  http.post(config.endpoint);
  http.sendHeader("Content-Type", "application/json");
  http.sendHeader("Content-Length", jsonData.length());
  http.beginBody();
  http.print(jsonData);
  http.endRequest();
  
  int statusCode = http.responseStatusCode();
  http.stop();
  
  if (statusCode >= 200 && statusCode < 300) {
    lastTransmissionTime = millis();
    return true;
  } else {
    rtt.print("HTTP error: "); rtt.println(statusCode);
    return false;
  }
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
  if (lastTransmissionTime == 0 || (currentTime - lastTransmissionTime >= deviceConfig.transmission_interval)) {
    rtt.println("Transmitting sensor data...");
    if (!modemInitialized) {
      if (!initializeModem()) {
        rtt.println("Failed to initialize modem");
        return;
      }
    }
    bool networkConnected = modem.isGprsConnected();
    if (!networkConnected) {
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
  int currentPos;
  if (!modemFileSeek(0, 0)) {
    rtt.println("Failed to seek to beginning of file");
    return false;
  }
  while (modem.stream.available()) { modem.stream.read(); }
  int lastwd = wdcounter;
  while(lastwd == wdcounter){
    nrf_delay_us(200);
  }
  delay(20);
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
  StaticJsonDocument<1024> doc;
  doc["device_name"] = config.device_name;
  doc["transmission_interval"] = config.transmission_interval;
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
  StaticJsonDocument<512> doc;
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
  rtt.println("Configuration loaded successfully");
  return true;
}

void printConfig(const ModemConfig& config) {
  rtt.println("|----------------------------------|");
  rtt.println("| Current Configuration           |");
  rtt.println("|----------------------------------|");
  rtt.print("Device Name: "); rtt.println(config.device_name);
  rtt.print("Transmission Interval: "); rtt.print(config.transmission_interval); rtt.println(" ms");
  rtt.print("GPS Enabled: "); rtt.println(config.enable_gps ? "Yes" : "No");
  rtt.print("GPS Timeout: "); rtt.print(config.gps_timeout); rtt.println(" ms");
  rtt.print("Server URL: "); rtt.println(config.server_url);
  rtt.print("Server Endpoint: "); rtt.println(config.server_endpoint);
  rtt.print("Server Port: "); rtt.println(config.server_port);
  rtt.print("APN: "); rtt.println(config.apn);
  rtt.print("Storage Enabled: "); rtt.println(config.enable_storage ? "Yes" : "No");
  rtt.print("Max Stored Files: "); rtt.println(config.max_stored_files);
  rtt.println("|----------------------------------|");
}

void LoadDeviceConfig() {
  rtt.println("|----------------------------------|");
  rtt.println("| Loading device configuration     |");
  rtt.println("|----------------------------------|");
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
  pinMode(ESP_TXD, OUTPUT);
  pinMode(ESP_RXD, INPUT);
  powerupESP();
  delay(1000);
  digitalWrite(ESP_TXD, HIGH);
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

void uartSendByte(uint8_t byte) {
  digitalWrite(ESP_TXD, LOW);
  delayMicroseconds(8);
  for (int i = 0; i < 8; i++) {
    digitalWrite(ESP_TXD, (byte >> i) & 0x01);
    delayMicroseconds(8);
  }
  digitalWrite(ESP_TXD, HIGH);
  delayMicroseconds(8);
}

bool sendCommandToESP(const String& command) {
  if (!espSerialInitialized) {
    rtt.println("ESP serial not initialized");
    return false;
  }
  rtt.print("Sending to ESP: "); rtt.println(command);
  String cmd = command + "\r\n";
  delayMicroseconds(100);
  for (size_t i = 0; i < cmd.length(); i++) {
    uartSendByte(cmd.charAt(i));
  }
  return true;
}

bool testESPConnection() {
  if (!initializeESPSerial()) {
    rtt.println("Failed to initialize ESP serial");
    return false;
  }
  rtt.println("Sending test command to ESP...");
  String response;
  if (sendESPCommandAndGetResponse("AT", response, 5000)) {
    return true;
  } else {
    rtt.println("No valid response from ESP");
    return false;
  }
}

void testESPCommunication() {
  rtt.println("|----------------------------------|");
  rtt.println("| Testing ESP Communication       |");
  rtt.println("|----------------------------------|");
  powerupESP();
  delay(200);
  if (initializeESPSerial()) {
    if (testESPConnection()) {
      rtt.println("ESP communication test successful!");
      rtt.println("Testing additional ESP commands...");
      String response;
      if (sendESPCommandAndGetResponse("AT+GMR", response, 5000)) {
        rtt.println("ESP Version command successful:");
        rtt.println(response);
      } else {
        rtt.println("ESP Version command failed");
        rtt.println(response);
      }
    } else {
      rtt.println("ESP communication test failed");
    }
  } else {
    rtt.println("ESP serial initialization failed");
  }
  powerdownESP();
  rtt.println("|----------------------------------|");
}

bool readESPResponseDebug(String& response, unsigned long timeout) {
  delayMicroseconds(10);
  const long BAUD_RATE = 115200;
  const int BIT_PERIOD_US = 1000000 / BAUD_RATE;
  response = "";
  unsigned long startTime = millis();
  unsigned long interCharTimeout = BIT_PERIOD_US * 20;
  unsigned long lastBitTime = micros();
  uint8_t samples[200];
  uint8_t pos = 0;
  for (int i = 0; i < 200; i++) {
    samples[i] = 0;
  }
  while (millis() - startTime < timeout) {
    while (digitalRead(ESP_RXD) == HIGH) {
      if (micros() - lastBitTime > interCharTimeout) {
        delayMicroseconds(100);
        for(int i = 0; i < pos; i++) {
          response += (char)samples[i];
          delayMicroseconds(10);
        }
        delayMicroseconds(100);
        return response.length() > 0;
      }
      if (millis() - startTime > timeout) {
        rtt.println("Overall timeout");
        return response.length() > 0;
      }
    }
    unsigned long byteStartTime = micros();
    lastBitTime = byteStartTime;
    uint8_t currentByte = 0;
    delayMicroseconds(BIT_PERIOD_US + BIT_PERIOD_US / 2 - 2);
    for (int i = 0; i < 8; ++i) {
      if (digitalRead(ESP_RXD) == HIGH) {
        currentByte |= (1 << i);
      }
      if(i != 7)delayMicroseconds(BIT_PERIOD_US);
    }
    nrf_delay_us(4);
    lastBitTime = micros();
    samples[pos] = currentByte;
    pos++;
    if (!(digitalRead(ESP_RXD) == HIGH)) {
      rtt.println("Frame error: Stop bit was LOW");
      rtt.println((char)samples[pos - 1]);
    }
  }
  rtt.println("Timeout");
  rtt.println(pos);
  for (int i = 0; i < pos; i++) {
    response += (char)samples[i];
  }
  rtt.println(response);
  return response.length() > 0;
}

bool parseESPResponse(const String& rawResponse, String& filteredResponse, bool& hasOK) {
  filteredResponse = "";
  hasOK = false;
  String lines[20];
  int lineCount = 0;
  size_t startPos = 0;
  for (size_t i = 0; i < rawResponse.length() && lineCount < 20; i++) {
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
  int lastwd = wdcounter;
  while(lastwd == wdcounter){
    nrf_delay_us(200);
  }
  delay(20);
  String rawResponse;
  sendCommandToESP(command);
  readESPResponseDebug(rawResponse, timeout);
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
  
  rtt.println("|----------------------------------|");
  rtt.println("| Checking Modem Storage Space    |");
  rtt.println("|----------------------------------|");
  
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