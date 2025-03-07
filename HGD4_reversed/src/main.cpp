#include <Arduino.h>
#include <Wire.h>
#include "ens210.h" // ENS210 library
#include "SparkFun_VEML6030_Ambient_Light_Sensor.h"
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

#define LIS3DH_CS 6

#define AL_ADDR 0x29

SparkFun_Ambient_Light light(AL_ADDR);

ENS210 ens210;

Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);

void testallpins();
void test1pin(int i,int d);
void busscan();
void measuretemp();
void lightsense();

void in1_handler() {
  digitalWrite(10,HIGH);
  delay(1);
  digitalWrite(10,LOW);
  delay(1);
}

void setup() {
  //latch on the main power
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  //setup watchdog feeding
  pinMode(10, OUTPUT);
  pinMode(9, INPUT);
  attachInterrupt(digitalPinToInterrupt(9), in1_handler, RISING);
  //setup Serial
  Serial.begin(115200);
  //setup I2C bus
  Wire.begin();
  //set i2c pins 
  pinMode(15, OUTPUT);
  //needed for some boards
  in1_handler();
}

int ptt = 34;

void loop() {
  Serial.println("loop");
  pinMode(29,OUTPUT);
  digitalWrite(29,HIGH);

  pinMode(34,OUTPUT);
  digitalWrite(34,HIGH);
  delay(600);
  digitalWrite(34,LOW);
  delay(600);
  //testallpins();
  //measuretemp();
  //lightsense();
  //pinMode(ptt,OUTPUT);
  //digitalWrite(ptt,HIGH);
  //delay(100);
  //digitalWrite(ptt,LOW);
  //delay(100);
}

void test1pin(int i,int d){
    Serial.println("Testing Pin with number");
    Serial.println(i);
    pinMode(i, OUTPUT);
    for (size_t j = 0; j < 100; j++)
    {
      delay(d);
      digitalWrite(i, HIGH);
      delay(d);
      digitalWrite(i, LOW);
    }
}

void lightsense(){

// Possible values: .125, .25, 1, 2
// Both .125 and .25 should be used in most cases except darker rooms.
// A gain of 2 should only be used if the sensor will be covered by a dark
// glass.
float gain = .125;

// Possible integration times in milliseconds: 800, 400, 200, 100, 50, 25
// Higher times give higher resolutions and should be used in darker light. 
int time = 100;
long luxVal = 0; 
  if(light.begin())
    Serial.println("Ready to sense some light!"); 
  else
    Serial.println("Could not communicate with the sensor!");

  // Again the gain and integration times determine the resolution of the lux
  // value, and give different ranges of possible light readings. Check out
  // hoookup guide for more info. 
  light.setGain(gain);
  light.setIntegTime(time);

  Serial.println("Reading settings..."); 
  Serial.print("Gain: ");
  float gainVal = light.readGain();
  Serial.print(gainVal, 3); 
  Serial.print(" Integration Time: ");
  int timeVal = light.readIntegTime();
  Serial.println(timeVal);
  luxVal = light.readLight();
  Serial.print("Ambient Light Reading: ");
  Serial.print(luxVal);
  Serial.println(" Lux");  
}

void measuretemp(){
  ens210.begin();
  int t_data, t_status, h_data, h_status;
  ens210.measure(&t_data, &t_status, &h_data, &h_status );

  Serial.print( ens210.toCelsius(t_data,10)/10.0, 1 ); Serial.print(" C, ");
  Serial.print( ens210.toPercentageH(h_data,1)      ); Serial.print(" %RH");
  Serial.println();

}

void busscan(){
   byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

void testallpins(){
  for (size_t i = 36; i < 64; i++){
    if(i != 2 && i != 29){
    Serial.println(String(i));
    for (size_t  j= 0; j < 50; j++)
    {
      pinMode(i, OUTPUT);
      digitalWrite(i, HIGH);
      delay(50);
      digitalWrite(i, LOW);
      delay(50);
    }
    }
  }
}