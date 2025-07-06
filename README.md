# HGD4_reversed
Custom firmware and information for the LOCO Track Primary HGD4 GPS tracker

![pinout](https://github.com/jonasniesner/HGD4_reversed/blob/main/pinout.webp?raw=true)

To get startet, connect a jlink programmer to GND and SWDIO and SWCLK pins. Then connect a 5-6V power supply or battery to the battery port and press the power button.
After that you can upload the NRF firmware part. The esp does not need custom firmware at the moment.
You will most likely have to replace the sim card and adjust the config for it and the backend server you intend to use.

The following tables where mainly reversed from the hardware version 1.1, we know of the following version 1.1 1.4 1.6 1.6a. All of those versions should be compatible


| Component                      | Type                      |  Note                                        | Marking |
|--------------------------------|--------------------------|-----------------------------------------------|--------------|
| nRF52840                       | SoC                       | Main SoC, 1MB Flash, 256KB RAM               | U1           |
| ESP8266                        | WiFi SoC                  | 2MB external Flash, running AT firmware      | IC8          |
| Quectel BG95-M3                | LTE Modem with GPS        | USB exposed to debug pads                    | M1           |
| LIS3DH                         | Acceleration Sensor       | Two units present                            | U2           |
| LIS3DH                         | Acceleration Sensor       | Two units present                            | U3           |
| VEML6035                       | Light Sensor              | Next to push button, used for wake-up        | IC7          |
| ENS210                         | Temperature + Humidity Sensor | only on hw version 1.1 and 1.4           | IC3          |
| TPL5010                        | Watchdog                  | needs to be fed                              | IC4          |
| LPS22HB                        | Pressure Sensor           | only on hw version 1.1 and 1.4               | IC5          |
| ENS220                         | Pressure Sensor           | only on hw version 1.6 and 1.6a              | IC5          |
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
