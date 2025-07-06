#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <Sodaq_LPS22HB.h>
#include <TinyGsmClient.h>
#include <RTTStream.h>
#include <ArduinoHttpClient.h>
#include <ArduinoJson.h>
#include <bluefruit.h>
#include <ens220.h>
#include "SparkFun_VEML6030_Ambient_Light_Sensor.h"
#include "sensors.h"
#include "ens210.h"

SparkFun_Ambient_Light light(AL_ADDR);
Sodaq_LPS22HB barometricSensor;
Adafruit_LIS3DH acc1 = Adafruit_LIS3DH(ACCL1_CS);
Adafruit_LIS3DH acc2 = Adafruit_LIS3DH(ACCL2_CS);
ENS210 ens210;
RTTStream rtt;
ENS220 ens220;
TinyGsm       modem(SerialAT);
TinyGsmClient client(modem);

BLEDfu  bledfu;
BLEDis  bledis;
BLEUart bleuart;
BLEBas  blebas;

struct ModemConfig {
  String device_name;
  unsigned long transmission_interval;
  unsigned long transmission_interval_motion;
  bool enable_gps;
  unsigned long gps_timeout;
  String server_url;
  String server_endpoint;
  int server_port;
  String apn;
  String username;
  String password;
  bool enable_storage;
  int max_stored_files;
  bool wifi_scan_enabled;
  int wifi_scan_max_networks;
  bool ble_scan_enabled;
  int ble_scan_max_beacons;
  unsigned long ble_scan_duration;
  bool modem_shutdown_between_loops;
  bool motion_detection_enabled;
  float motion_threshold;
  bool modem_shutdown_on_motion;
  
  ModemConfig() {
    device_name = "Dev 1";
    transmission_interval = 3600000;
    transmission_interval_motion = 60000;
    enable_gps = true;
    gps_timeout = 15000;
    server_url = "api.64eng.de";
    server_endpoint = "/sensordata.php";
    server_port = 80;
    apn = "internet";
    username = "";
    password = "";
    enable_storage = true;
    max_stored_files = 10;
    wifi_scan_enabled = true;
    wifi_scan_max_networks = 20;
    ble_scan_enabled = true;
    ble_scan_max_beacons = 20;
    ble_scan_duration = 5000;
    modem_shutdown_between_loops = true;
    motion_detection_enabled = true;
    motion_threshold = 500;
    modem_shutdown_on_motion = false;
  }
};

struct ServerConfig {
  const char* server;
  const char* endpoint;
  int port;
  const char* apn;
  const char* username;
  const char* password;
};

DynamicJsonDocument bleScanDoc(16000);
JsonArray bleBeacons;
bool bleScanning = false;
unsigned long bleScanStartTime = 0;
int bleBeaconsFound = 0;
static bool globalMotionDetected = false;
static unsigned long globalLastMotionTime = 0;
static float lastAcc1X = 0, lastAcc1Y = 0, lastAcc1Z = 0;
static float lastAcc2X = 0, lastAcc2Y = 0, lastAcc2Z = 0;
float currentAcc1X, currentAcc1Y, currentAcc1Z;
float currentAcc2X, currentAcc2Y, currentAcc2Z;
bool espmodemrail = 0;
bool espon = 0;
bool modemon = 0;
bool has_ens220 = false;
bool has_veml60350 = false;
bool has_ens210 = false;
bool has_lpS22hb = false;
bool espSerialInitialized = false;
int currentFileHandle = -1;
bool fileOpen = false;
volatile int32_t wdcounter = 0;
unsigned long lastTransmissionTime = 0;
bool modemInitialized = false;
unsigned long lastWaitCheck = 0;
extern unsigned long lastTransmissionTime;
extern unsigned long transmissionInterval;
extern bool modemInitialized;
extern bool enableGPS;
extern unsigned long gpsTimeout;

extern ModemConfig deviceConfig;

void ble_scan_callback(ble_gap_evt_adv_report_t* report);
void scanforbeacons();
bool initializeBLE();
void gpioinit();
void softpwrup();
void sensorpwr(bool onoff);
void collectAllSensorData();
float readBatteryVoltage();
float readLightSensor(float gain = 0.25, int integrationTime = 100);
bool readTemperatureHumidity(float& temperature, float& humidity);
bool readPressure(float& pressure, float& sensorTemp);
bool readAccelerometers(float& acc1_x, float& acc1_y, float& acc1_z, float& acc2_x, float& acc2_y, float& acc2_z);
bool readGPS(float& latitude, float& longitude, float& speed, float& altitude, int& visibleSatellites, int& usedSatellites, float& accuracy);
bool readModemInfo();
bool readNetworkInfo();
void checkModemStorageSpace();
bool initializeModem();
bool connectToNetwork();
bool disconnectFromNetwork();
bool sendSensorDataToServer(const SensorData& data, const ServerConfig& config);
bool sendSensorDataToServer(const SensorData& data);
void testModemAndNetwork();
void manageModemLifecycle();
void shutdownModem();
bool initializeModemIfNeeded();
bool modemFileOpen(const String& filename, int mode);
bool modemFileWrite(const String& data);
bool modemFileRead(String& data, int length = 1024);
bool modemFileSeek(int offset, int origin);
bool modemFilePosition(int& position);
bool modemFileClose();
bool modemFileDelete(const String& filename);
bool modemFileExists(const String& filename);
int modemFileSize(const String& filename);
String modemFileList();
bool checkModemReady();
void blinkLED(int times, int delayon, int delayoff, bool green);
bool saveConfigToModem(const ModemConfig& config);
bool loadConfigFromModem(ModemConfig& config);
void printConfig(const ModemConfig& config);
void LoadDeviceConfig();
void displaySensorDataSummary();
bool initializeESPSerial();
void powerupESP();
void powerdownESP();
bool sendCommandToESP(const String& command);
bool readESPResponse(String& response, unsigned long timeout = 5000);
void ScanForWiFiNetworks();
bool parseESPResponse(const String& rawResponse, String& filteredResponse, bool& hasOK);
bool sendESPCommandAndGetResponse(const String& command, String& response, unsigned long timeout);
String parseCWLAPData(const String& cwlapResponse);
bool sendLargeJsonData(HttpClient& http, const DynamicJsonDocument& doc, const String& endpoint);
void optimizeScanData(DynamicJsonDocument& doc, size_t maxSize);
bool parseServerResponse(const String& response, JsonDocument& doc);
String readCompleteHttpResponse(HttpClient& http, unsigned long timeout);
bool waitForNextTransmission(unsigned long interval);
void performBackgroundTasks();
bool isMotionDetected();
void triggerMotionTransmission();
bool shouldKeepModemPowered();
void logPowerManagementStatus();
void checkAllInterruptFlags();
void clearInterruptFlags();
void check1msInterruptSources();
void scanI2C();