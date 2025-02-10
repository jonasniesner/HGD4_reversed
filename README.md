# HGD4_reversed
Information about the LOCO Track Primary HGD4 GPS tracker

The main soc of the device is the nRF52840 with 1MB of Flash 256KB RAM, it is not locked or read protected with exposed debug pads.

It also contains an ESP8266 WIFI SoC with 2MB of external Flash, running AT firmware and has exposed debug pads.

The modem is a Quectel BG95-M3 LTE MODEM with GPS. Both antennas are fitted and the usb of the modem is likely exposed to debug pads.

The other sensors are 2 Acceleration Sensors, a Light Sensor net to the push button(that will allow the device to wake up from a low power state), a Temperature Sensor and Pressure sensor and a red and green led.

The full schematic of the device will still need to be reversed and the sensors need to be properly identifyed. PRs are welcome. 
