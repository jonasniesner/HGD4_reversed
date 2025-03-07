#include "main.h"

SparkFun_Ambient_Light light(AL_ADDR);

ENS210 ens210;

Adafruit_LIS3DH acc1 = Adafruit_LIS3DH(ACCL1_CS);
Adafruit_LIS3DH acc2 = Adafruit_LIS3DH(ACCL2_CS);

void setup() {
  gpioinit();
}

void loop() {
  Serial.println("loop");
  busscan();
  //powerupesp();
  //powerupmodem();
  measuretemp();
  lightsense();
  delay(1000);
}

void wd_handler() {
  digitalWrite(DONE,HIGH);
  delay(1);
  digitalWrite(DONE,LOW);
}


void gpioinit(){
  //latch on the main power
  pinMode(PWR_LATCH , OUTPUT);
  digitalWrite(PWR_LATCH , HIGH);
  //setup watchdog feeding
  pinMode(WAKE, INPUT);
  pinMode(DONE, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(WAKE), wd_handler, RISING);
  //setup Serial
  Serial.begin(115200);
  //setup I2C bus
  Wire.begin();
  //set i2c pins 
  pinMode(I2C_SDA, OUTPUT);
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
}

void powerupesp(){
  Serial.println("Powering up esp");
  digitalWrite(MODEM_ESP_PWR,HIGH);
  digitalWrite(ESP_PWR,HIGH);
  delay(500);
}

void powerupmodem(){
  Serial.println("powering up modem");
  digitalWrite(MODEM_ESP_PWR,HIGH);
  delay(600);
  digitalWrite(MODEM_PWRKEY,HIGH);
  delay(600);
  digitalWrite(MODEM_PWRKEY,LOW);
  delay(600);
}

void lightsense(){
float gain = .25;
int time = 100;
long luxVal = 0; 
  if(!light.begin())
    Serial.println("Could not communicate with the light sensor!");
  light.setGain(gain);
  light.setIntegTime(time);
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