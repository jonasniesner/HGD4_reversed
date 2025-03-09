# HGD4_reversed
Information about the LOCO Track Primary HGD4 GPS tracker

![pinout](https://github.com/jonasniesner/HGD4_reversed/blob/main/pinout.webp?raw=true)


| Component                      | Type                      |  Note                                        | Marking |
|--------------------------------|--------------------------|-----------------------------------------------|--------------|
| nRF52840                       | SoC                       | Main SoC, 1MB Flash, 256KB RAM               | U1           |
| ESP8266                        | WiFi SoC                  | 2MB external Flash, running AT firmware      | IC8          |
| Quectel BG95-M3                | LTE Modem with GPS        | USB exposed to debug pads                    | M1           |
| LIS3DH                         | Acceleration Sensor       | Two units present                            | U2           |
| LIS3DH                         | Acceleration Sensor       | Two units present                            | U3           |
| VEML6035                       | Light Sensor              | Next to push button, used for wake-up        | IC7          |
| ENS210                         | Temperature + Humidity Sensor | -                                        | IC3          |
| TPL5010                        | Watchdog                  | needs to be fed                              | IC4          |
| LPS22HB                        | Pressure Sensor           | -                                            | IC5          |
| Red LED                        | LED                       | -                                            | LED1         |
| Green LED                      | LED                       | -                                            | LED2         |
| Button                         | Push Button               | Wakes the device power off                   | SW1          |

What we know about the pins of the nRF52840 so far:

| Pin | Description                                   |
|----:|-----------------------------------------------|
|  0  | !LED Green                                    |
|  1  | !RED Green                                    |
|  2  | Power Latch - High Z * High -> Stay on -> Low hard power off |
|  3  | NC                                           |
|  4  | CS ACCL 2                                    |
|  5  | NC                                           |
|  6  | CS ACCL 1                                    |
|  7  | NC                                           |
|  8  | MISO                                         |
|  9  | TPL5010 Wake input                           |
| 10  | TPL5010 Done output                          |
| 11  | Sensor VDD                                   |
| 12  | SPI CLK                                      |
| 13  | SCL                                          |
| 14  | bridged to 16 and 18                         |
| 15  | SDA                                          |
| 16  | bridged to 14 and 18                         |
| 17  | Modem Rxd GPIO                               |
| 18  | RESET                                        |
| 19  | NC                                           |
| 20  | connects to R29 R                            |
| 21  | NC                                           |
| 22  | Modem Txd GPIO                               |
| 23  | NC                                           |
| 24  | Connects to Q5 U R                           |
| 25  | NC                                           |
| 26  | ACC U3 int                                   |
| 27  | NC                                           |
| 28  | NC                                           |
| 29  | Modem + ESP power, switches the actual rail  |
| 30  | NC                                           |
| 31  | VBAT measurement ? (R48 b)                   |
| 32  | Uart logging Rxd (broken out gpio)           |
| 33  | Uart logging Txd (broken out gpio)           |
| 34  | Modem !pwrkey                                |
| 35  | NC                                           |
| 36  | VEML6035 INT                                 |
| 37  | NC                                           |
| 38  | Switch input                                 |
| 39  | NC                                           |
| 40  | NC                                           |
| 41  | MOSI                                         |
| 42  | Wifi Rxd   Baud 115200                       |
| 43  | Wifi Txd   Baud 115200                       |
| 44  | NC                                           |
| 45  | ESP power pin                                |
| 46  | NC                                           |
| 47  | ESP GPIO 0                                   |
