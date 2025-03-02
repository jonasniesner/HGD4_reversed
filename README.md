# HGD4_reversed
Information about the LOCO Track Primary HGD4 GPS tracker

The main soc of the device is the nRF52840 with 1MB of Flash 256KB RAM, it is not locked or read protected with exposed debug pads.

It also contains an ESP8266 WIFI SoC with 2MB of external Flash, running AT firmware and has exposed debug pads.

The modem is a Quectel BG95-M3 LTE MODEM with GPS. Both antennas are fitted and the usb of the modem is likely exposed to debug pads.

The other sensors are 2 Acceleration Sensors(LIS3DH), a Light Sensor(VEML6035) next to the push button(that will allow the device to wake up from a low power state), a Temperature + Humidity Sensor(ENS210) and Pressure sensor and a red and green led.

The full schematic of the device will still need to be reversed and the sensors need to be properly identifyed. PRs are welcome. 

What we know so far

0
1
2    Power Latch - High Z * High -> Stay on -> low hard power off
3
4    CS ACCL 2 ? 
5
6     CS ACCL 1 ?
7
8     MISO?
9     TPL5010 Wake input
10    TPL5010 Done output
11
12   SPI CLK ?
13     SCL 
14
15    SDA
16
17  Modem Rxd GPIO 17 ?
18
19
20
21
22  Modem Txd GPIO ?
23
24
25
26
27
28
29  Modem power pin ??
30
31
32   Uart loggig Rxd ? 
33   Uart loggig Txd ? 
34   Modem power pin ??
35
36
37
38
39
40
41   MOSI ? 
42   Wifi Rxd ?   Baud 115200
43   WIFI Txd ?   Baud 115200
44
45   Modem power pin ??
46
47
48
49
50
51
52
53
54
55
56
57
58
59
60
61
62
63

For now we know the following(thanks to atc1441)

I2C for most sensors(Temp, Pressure, Light):
SCL = GPIO 13
SDA = GPIO 15

ACCL via SPI:
CS ACCL 1 = GPIO 6
CS ACCL 2 = GPIO 4
SCL = GPIO 12
MOSI = GPIO 41
MISO = GPIO 8

UART  Wifi:
Baud 115200
Txd GPIO 43
Rxd GPIO 42

UART  Modem:
Power enable: 
GPIO 45
GPIO 29
GPIO 34
Baud 115200
Txd GPIO 22
Rxd GPIO 17

UART Logging:
Baud 115200
Txd GPIO 33
Rxd GPIO 32
