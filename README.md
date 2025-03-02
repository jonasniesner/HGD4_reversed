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
| ENS210                         | Temperature + Humidity Sensor | -                                        | -            |
| TPL5010                        | Watchdog                  | needs to be fed                              | IC4          |
| Unspecified Pressure Sensor    | Pressure Sensor           | -                                            | IC5          |
| Red LED                        | LED                       | -                                            | LED1         |
| Green LED                      | LED                       | -                                            | LED2         |

What we know about the pins of the nRF52840 so far:

| Pin | Description                                   |
|----:|-----------------------------------------------|
|  2  | Power Latch - High Z * High -> Stay on -> Low hard power off |
|  4  | CS ACCL 2 ?                                  |
|  6  | CS ACCL 1 ?                                  |
|  8  | MISO?                                        |
|  9  | TPL5010 Wake input                           |
| 10  | TPL5010 Done output                          |
| 12  | SPI CLK ?                                    |
| 13  | SCL                                          |
| 15  | SDA                                          |
| 17  | Modem Rxd GPIO ?                             |
| 22  | Modem Txd GPIO ?                             |
| 29  | Modem power pin ??                           |
| 32  | Uart logging Rxd ?                           |
| 33  | Uart logging Txd ?                           |
| 34  | Modem power pin ??                           |
| 41  | MOSI ?                                       |
| 42  | Wifi Rxd ? Baud 115200                       |
| 43  | Wifi Txd ? Baud 115200                       |
| 45  | Modem power pin ??                           |
