# HGD4_reversed
Information about the LOCO Track Primary HGD4 GPS tracker

The main soc of the device is the nRF52840 with 1MB of Flash 256KB RAM, it is not locked or read protected with exposed debug pads.

It also contains an ESP8266 WIFI SoC with 2MB of external Flash, running AT firmware and has exposed debug pads.

The modem is a Quectel BG95-M3 LTE MODEM with GPS. Both antennas are fitted and the usb of the modem is likely exposed to debug pads.

The other sensors are 2 Acceleration Sensors(LIS3DH), a Light Sensor(VEML6035) next to the push button(that will allow the device to wake up from a low power state), a Temperature + Humidity Sensor(ENS210) and Pressure sensor and a red and green led.

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
