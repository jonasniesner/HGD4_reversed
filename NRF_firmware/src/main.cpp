#include "main.h"

const char server[]   = "api.64eng.de";
const char resource[] = "ok.txt";

HttpClient    http(client, server, 80);

void setup() {
  rtt.println("|---------Starting setup---------|");
  gpioinit();
  initmodem();
  testmodem();
  powerdownmodem();
  delay(2000);
}

void loop() {
  rtt.println();
  rtt.println("|---------Starting loop---------|");
  measureacc();
  measuretemp();
  measurepressure();
  lightsense();
  ButtonState();
  measureBattery();
  delay(10000);
}

void wd_handler() {
  digitalWrite(DONE,HIGH);
  delay(1);
  digitalWrite(DONE,LOW);
}

void initmodem(){
  powerupmodem();
  rtt.println("Initialising modem");
  SerialAT.begin(115200);
  delay(10);
  modem.init();
  rtt.print("Modem Info: ");
  rtt.println(modem.getModemInfo());
  rtt.print("Modem SN: ");
  rtt.println(modem.getModemSerialNumber());
  rtt.print("Modem isconnected: ");
  rtt.println(modem.isGprsConnected());
}

void connectmodem(){
  rtt.println("Connecting modem");
  modem.gprsConnect("internet", "", "");
  rtt.println("Waiting for network...");
  if (!modem.waitForNetwork(600000L, true)) {
    delay(10000);
    return;
  }
  rtt.println("Connection state");
  rtt.println(modem.isGprsConnected());
}

void disconnectmodem(){
  rtt.println("Disconnecting modem");
  modem.gprsDisconnect();
  rtt.println("Connection state");
  rtt.println(modem.isGprsConnected());
}

void httpreq(){
  rtt.print(F("Performing HTTP GET request... "));
  int err = http.get(resource);
  if (err != 0) {
    rtt.println(F("failed to connect"));
    return;
  }

  int status = http.responseStatusCode();
  rtt.print(F("Response status code: "));
  rtt.println(status);

  rtt.println(F("Response Headers:"));
  while (http.headerAvailable()) {
    String headerName  = http.readHeaderName();
    String headerValue = http.readHeaderValue();
    rtt.println("    " + headerName + " : " + headerValue);
  }

  int length = http.contentLength();
  if (length >= 0) {
    rtt.print(F("Content length is: "));
    rtt.println(length);
  }
  if (http.isResponseChunked()) {
    rtt.println(F("The response is chunked"));
  }

  String body = http.responseBody();
  rtt.println(F("Response:"));
  rtt.println(body);

  rtt.print(F("Body length is: "));
  rtt.println(body.length());

  http.stop();
  rtt.println(F("Server disconnected"));
}

void getgps(){
  rtt.printf("Enabling GPS/GNSS/GLONASS and waiting 15s for warm-up");
  modem.enableGPS();
  delay(15000L);
  float gps_latitude  = 0;
  float gps_longitude = 0;
  float gps_speed     = 0;
  float gps_altitude  = 0;
  int   gps_vsat      = 0;
  int   gps_usat      = 0;
  float gps_accuracy  = 0;
  int   gps_year      = 0;
  int   gps_month     = 0;
  int   gps_day       = 0;
  int   gps_hour      = 0;
  int   gps_minute    = 0;
  int   gps_second    = 0;
  for (int8_t i = 15; i; i--) {
    rtt.printf("Requesting current GPS/GNSS/GLONASS location");
    if (modem.getGPS(&gps_latitude, &gps_longitude, &gps_speed, &gps_altitude,
                     &gps_vsat, &gps_usat, &gps_accuracy, &gps_year, &gps_month,
                     &gps_day, &gps_hour, &gps_minute, &gps_second)) {
      rtt.printf("Latitude:", String(gps_latitude, 8), "\tLongitude:", String(gps_longitude, 8));
      rtt.printf("Speed:", gps_speed, "\tAltitude:", gps_altitude);
      rtt.printf("Visible Satellites:", gps_vsat, "\tUsed Satellites:", gps_usat);
      rtt.printf("Accuracy:", gps_accuracy);
      rtt.printf("Year:", gps_year, "\tMonth:", gps_month, "\tDay:", gps_day);
      rtt.printf("Hour:", gps_hour, "\tMinute:", gps_minute, "\tSecond:", gps_second);
      break;
    } else {
      rtt.printf("Couldn't get GPS/GNSS/GLONASS location, retrying in 15s.");
      delay(15000L);
    }
  }
  rtt.printf("Retrieving GPS/GNSS/GLONASS location again as a string");
  String gps_raw = modem.getGPSraw();
  rtt.printf("GPS/GNSS Based Location String:", gps_raw);
  rtt.printf("Disabling GPS");
  modem.disableGPS();
}

void testmodem(){
  connectmodem();
  rtt.println("testing modem");

  rtt.print("Modem CCID: ");
  rtt.println(modem.getSimCCID());
  rtt.print("Modem IMEI: ");
  rtt.println(modem.getIMEI());
  rtt.print("Sim Statue: ");
  rtt.println(modem.getSimStatus());
  rtt.print("Modem IMSI: ");
  rtt.println(modem.getIMSI());
  rtt.print("Modem Operator: ");
  rtt.println(modem.getOperator());
  rtt.print("Modem LocalIP: ");
  rtt.println(modem.localIP());
  rtt.print("Modem SQ: ");
  rtt.println(modem.getSignalQuality());

  int8_t  chargeState   = -99;
  int8_t  chargePercent = -99;
  int16_t milliVolts    = -9999;
  modem.getBattStats(chargeState, chargePercent, milliVolts);
  rtt.println("Battery chargeState");
  rtt.println(chargeState);
  rtt.println("Battery chargePercent");
  rtt.println(chargePercent);
  rtt.println("Battery milliVolts");
  rtt.println(milliVolts);

  rtt.println("Chip temperature");
  rtt.println(modem.getTemperature());

  httpreq();

  //getgps();

  disconnectmodem();
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
  //needed for some boards
  wd_handler();
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

void ButtonState() {
  int buttonState = digitalRead(PWR_SW_IN);
  rtt.println("|-----------------|");
  rtt.println("| Button (SW)    |");
  rtt.print("| State: "); rtt.println(buttonState ? "Pressed" : "Released");
  rtt.println("|-----------------|");
}

void measureBattery() {
  float voltage = ((float)analogRead(VBAT_DIV) / 1000.0) * 6.0;
  rtt.println("|-------------------|");
  rtt.println("| Battery Voltage  |");
  rtt.print("| "); rtt.print(voltage, 2); rtt.println(" V");
  rtt.println("|-------------------|");
}

void lightsense(){
float gain = .25;
int time = 100;
long luxVal = 0; 
  if(!light.begin())
    rtt.println("Could not communicate with the light sensor!");
  light.setGain(gain);
  light.setIntegTime(time);

  rtt.println("|-----------------------------|");
  rtt.println("| Sensor: VEML6035            |");
  rtt.println("| Gain    | Integration Time  |");
  rtt.print("| ");
  rtt.print(gain, 3);
  rtt.print("   | ");
  rtt.print(time);
  rtt.println("               |");
  
  luxVal = light.readLight();
  
  rtt.println("|-----------------------------|");
  rtt.println("| Ambient Light Reading       |");
  rtt.print("| ");
  rtt.print(luxVal);
  rtt.println(" Lux");
  rtt.println("|-----------------------------|");  
}

void measuretemp(){
  ens210.begin();
  int t_data, t_status, h_data, h_status;
  ens210.measure(&t_data, &t_status, &h_data, &h_status );

  rtt.println("|-------------------------------|");
  rtt.println("| Sensor: ENS210                |");
  rtt.println("| Temperature  | Humidity       |");
  rtt.print("| ");
  rtt.print(ens210.toCelsius(t_data, 10) / 10.0, 1);
  rtt.print(" C       | ");
  rtt.print(ens210.toPercentageH(h_data, 1));
  rtt.println(" %RH         |");
  rtt.println("|-------------------------------|");
}

void measurepressure(){
  lps22hb.begin();
  lps22hb.Enable();
  float pressure, temperature;
  lps22hb.GetPressure(&pressure);
  lps22hb.GetTemperature(&temperature);

  rtt.println("|----------------------------------|");
  rtt.println("| Sensor: LPS22HB                  |");
  rtt.println("| Pressure[hPa]  | Temperature[C]  |");
  rtt.print("| ");
  rtt.print(pressure, 2);
  rtt.print("        | ");
  rtt.print(temperature, 2);
  rtt.println("           |");
  rtt.println("|----------------------------------|");
}

void measureacc(){
  rtt.println("|----------------------------------|");
  rtt.println("| Sensor: LIS3DH                   |");
  if (!acc1.begin()) rtt.println("Could not start acc1");
  if (!acc2.begin()) rtt.println("Could not start acc2");

  rtt.print("| Range = "); rtt.print(2 << acc1.getRange());
  rtt.println("G                       |");

  // lis.setPerformanceMode(LIS3DH_MODE_LOW_POWER);
  rtt.print("| Performance mode set to: ");
  switch (acc1.getPerformanceMode()) {
    case LIS3DH_MODE_NORMAL: rtt.println("Normal 10bit"); break;
    case LIS3DH_MODE_LOW_POWER: rtt.println("Low Power 8bit"); break;
    case LIS3DH_MODE_HIGH_RESOLUTION: rtt.println("High Resolution 12bit"); break;
  }

  // lis.setDataRate(LIS3DH_DATARATE_50_HZ);
  rtt.print("| Data rate set to: ");
  switch (acc1.getDataRate()) {
    case LIS3DH_DATARATE_1_HZ: rtt.println("1 Hz"); break;
    case LIS3DH_DATARATE_10_HZ: rtt.println("10 Hz"); break;
    case LIS3DH_DATARATE_25_HZ: rtt.println("25 Hz"); break;
    case LIS3DH_DATARATE_50_HZ: rtt.println("50 Hz"); break;
    case LIS3DH_DATARATE_100_HZ: rtt.println("100 Hz"); break;
    case LIS3DH_DATARATE_200_HZ: rtt.println("200 Hz"); break;
    case LIS3DH_DATARATE_400_HZ: rtt.println("400 Hz"); break;

    case LIS3DH_DATARATE_POWERDOWN: rtt.println("Powered Down"); break;
    case LIS3DH_DATARATE_LOWPOWER_5KHZ: rtt.println("5 Khz Low Power"); break;
    case LIS3DH_DATARATE_LOWPOWER_1K6HZ: rtt.println("1.6 Khz Low Power"); break;
  }
  rtt.println("| Acc1 (X,Y,Z) | Acc2 (X,Y,Z)      |");
  acc1.read();
  acc2.read();

  rtt.print("| ");
  rtt.print(acc1.x); rtt.print(", ");
  rtt.print(acc1.y); rtt.print(", ");
  rtt.print(acc1.z); rtt.print(" | ");
  rtt.print(acc2.x); rtt.print(", ");
  rtt.print(acc2.y); rtt.print(", ");
  rtt.print(acc2.z); rtt.println("|");
  rtt.println("|----------------------------------|");
}