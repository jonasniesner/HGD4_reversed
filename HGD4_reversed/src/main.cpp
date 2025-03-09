#include "main.h"

SparkFun_Ambient_Light light(AL_ADDR);

ENS210 ens210;
LPS22HBSensor lps22hb(&Wire);

Adafruit_LIS3DH acc1 = Adafruit_LIS3DH(ACCL1_CS);
Adafruit_LIS3DH acc2 = Adafruit_LIS3DH(ACCL2_CS);


void testmodem(){
  //SerialAT.begin(9600);
}

void setup() {
  gpioinit();
}

void loop() {
  Serial.println();
  Serial.println("|---------Starting loop---------|");
  //powerupesp();
  //powerupmodem();
  measureacc();
  measuretemp();
  measurepressure();
  lightsense();
  Serial.println("vbat");
  Serial.println(((float)analogRead(VBAT_DIV) / 1000.0) * 6.0);
  Serial.println("SW");
  Serial.println(digitalRead(PWR_SW_IN));
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
  digitalWrite(PWR_LATCH , LOW);
  //setup watchdog feeding
  pinMode(WAKE, INPUT);
  pinMode(DONE, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(WAKE), wd_handler, RISING);
  //setup Serial
  Serial.begin(115200);
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

  Serial.println("|-----------------------------|");
  Serial.println("| Sensor: VEML6035            |");
  Serial.println("| Gain    | Integration Time  |");
  Serial.print("| ");
  Serial.print(gain, 3);
  Serial.print("   | ");
  Serial.print(time);
  Serial.println("               |");
  
  luxVal = light.readLight();
  
  Serial.println("|-----------------------------|");
  Serial.println("| Ambient Light Reading       |");
  Serial.print("| ");
  Serial.print(luxVal);
  Serial.println(" Lux");
  Serial.println("|-----------------------------|");  
}

void measuretemp(){
  ens210.begin();
  int t_data, t_status, h_data, h_status;
  ens210.measure(&t_data, &t_status, &h_data, &h_status );

  Serial.println("|-------------------------------|");
  Serial.println("| Sensor: ENS210                |");
  Serial.println("| Temperature  | Humidity       |");
  Serial.print("| ");
  Serial.print(ens210.toCelsius(t_data, 10) / 10.0, 1);
  Serial.print(" C       | ");
  Serial.print(ens210.toPercentageH(h_data, 1));
  Serial.println(" %RH         |");
  Serial.println("|-------------------------------|");
}

void measurepressure(){
  lps22hb.begin();
  lps22hb.Enable();
  float pressure, temperature;
  lps22hb.GetPressure(&pressure);
  lps22hb.GetTemperature(&temperature);

  Serial.println("|----------------------------------|");
  Serial.println("| Sensor: LPS22HB                  |");
  Serial.println("| Pressure[hPa]  | Temperature[C]  |");
  Serial.print("| ");
  Serial.print(pressure, 2);
  Serial.print("        | ");
  Serial.print(temperature, 2);
  Serial.println("           |");
  Serial.println("|----------------------------------|");
}

void measureacc(){
  if (! acc1.begin()) {
    Serial.println("Couldnt start acc1");
  }
  if (! acc2.begin()) {
    Serial.println("Couldnt start acc2");
  }
  Serial.print("Range = "); Serial.print(2 << acc1.getRange());
  Serial.println("G");

  // lis.setPerformanceMode(LIS3DH_MODE_LOW_POWER);
  Serial.print("Performance mode set to: ");
  switch (acc1.getPerformanceMode()) {
    case LIS3DH_MODE_NORMAL: Serial.println("Normal 10bit"); break;
    case LIS3DH_MODE_LOW_POWER: Serial.println("Low Power 8bit"); break;
    case LIS3DH_MODE_HIGH_RESOLUTION: Serial.println("High Resolution 12bit"); break;
  }

  // lis.setDataRate(LIS3DH_DATARATE_50_HZ);
  Serial.print("Data rate set to: ");
  switch (acc1.getDataRate()) {
    case LIS3DH_DATARATE_1_HZ: Serial.println("1 Hz"); break;
    case LIS3DH_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3DH_DATARATE_25_HZ: Serial.println("25 Hz"); break;
    case LIS3DH_DATARATE_50_HZ: Serial.println("50 Hz"); break;
    case LIS3DH_DATARATE_100_HZ: Serial.println("100 Hz"); break;
    case LIS3DH_DATARATE_200_HZ: Serial.println("200 Hz"); break;
    case LIS3DH_DATARATE_400_HZ: Serial.println("400 Hz"); break;

    case LIS3DH_DATARATE_POWERDOWN: Serial.println("Powered Down"); break;
    case LIS3DH_DATARATE_LOWPOWER_5KHZ: Serial.println("5 Khz Low Power"); break;
    case LIS3DH_DATARATE_LOWPOWER_1K6HZ: Serial.println("1.6 Khz Low Power"); break;
  }
  Serial.println("Reading");
  acc1.read();
  Serial.print("X:  "); Serial.print(acc1.x);
  Serial.print("  \tY:  "); Serial.print(acc1.y);
  Serial.print("  \tZ:  "); Serial.print(acc1.z);
  acc2.read();
  Serial.print("X:  "); Serial.print(acc2.x);
  Serial.print("  \tY:  "); Serial.print(acc2.y);
  Serial.print("  \tZ:  "); Serial.print(acc2.z);
}