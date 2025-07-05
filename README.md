# HGD4_reversed
Information about the LOCO Track Primary HGD4 GPS tracker

![pinout](https://github.com/jonasniesner/HGD4_reversed/blob/main/pinout.webp?raw=true)

The following tables where reversed from the hardware version 1.1, we know of the following version 1.1 1.4 1.6 1.6a. The other versions still have to be reversed properly.


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

| Pin | Description                                  |
|----:|----------------------------------------------|
|  0  | !LED Green                                   |
|  1  | !RED Green                                   |
|  2  | VBAT measurement                             |
|  4  | CS ACCL 2                                    |
|  6  | CS ACCL 1                                    |
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
| 20  | MODEM_CON_A                                  |
| 22  | Modem Txd GPIO                               |
| 24  | MODEM_CON_B                                  |
| 26  | ACC U3 int                                   |
| 29  | Modem + ESP power, switches the actual rail  |
| 31  | !Power Latch                                 |
| 32  | Uart logging Rxd (broken out gpio)           |
| 33  | Uart logging Txd (broken out gpio)           |
| 34  | Modem !pwrkey                                |
| 36  | VEML6035 INT                                 |
| 38  | Switch input                                 |
| 41  | MOSI                                         |
| 42  | Wifi Rxd   Baud 115200                       |
| 43  | Wifi Txd   Baud 115200                       |
| 45  | ESP power pin                                |
| 47  | ESP GPIO 0                                   |
