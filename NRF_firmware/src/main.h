#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <LPS22HBSensor.h>
#include <TinyGsmClient.h>
#include <RTTStream.h>
#include <ArduinoHttpClient.h>
#include "ens210.h"
#include "SparkFun_VEML6030_Ambient_Light_Sensor.h"
#include "sensors.h"

SparkFun_Ambient_Light light(AL_ADDR);

ENS210 ens210;
LPS22HBSensor lps22hb(&Wire);

Adafruit_LIS3DH acc1 = Adafruit_LIS3DH(ACCL1_CS);
Adafruit_LIS3DH acc2 = Adafruit_LIS3DH(ACCL2_CS);

RTTStream rtt;

TinyGsm        modem(SerialAT);
TinyGsmClient client(modem);

bool espmodemrail = 0;
bool espon = 0;
bool modemon = 0;

void measuretemp();
void measurepressure();
void ButtonState();
void measureBattery();
void lightsense();
void powerupesp();
void wd_handler();
void gpioinit();
void measureacc();
void powerupmodem();
void softpwrup();
void powerdownesp();
void powerdownmodem();
void testmodem();
void getgps();
void httpreq();
void disconnectmodem();
void connectmodem();
void initmodem();
void sampleallsensors();
void sensorpwr(bool onoff);