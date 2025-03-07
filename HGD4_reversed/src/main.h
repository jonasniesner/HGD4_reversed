#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "ens210.h"
#include "SparkFun_VEML6030_Ambient_Light_Sensor.h"

void busscan();
void measuretemp();
void lightsense();
void powerupesp();
void wd_handler();
void gpioinit();