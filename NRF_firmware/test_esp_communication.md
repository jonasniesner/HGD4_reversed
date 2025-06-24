# ESP Communication Test Guide

## Overview
This implementation establishes serial communication between the nRF52 and ESP microcontroller using:
- **ESP_TXD** (Pin 43): nRF52 TX → ESP RX
- **ESP_RXD** (Pin 42): ESP TX → nRF52 RX
- **ESP_PWR** (Pin 45): Power control for ESP
- **ESP_GPIO0** (Pin 47): ESP boot mode control (not used in this implementation)

## Implementation Details

### Bit-Banging UART at 115200 Baud
- Uses direct pin manipulation for UART communication
- Timing: 8.68 microseconds per bit (1/115200)
- 8N1 format: 8 data bits, no parity, 1 stop bit
- LSB first transmission

### Functions Implemented

1. **`initializeESPSerial()`**
   - Powers up ESP microcontroller
   - Configures pins for UART communication
   - Sets up bit-banging timing

2. **`powerupESP()`**
   - Powers up ESP/Modem rail if needed
   - Activates ESP power pin
   - Handles power rail management

3. **`powerdownESP()`**
   - Safely powers down ESP
   - Manages power rail state
   - Cleans up serial state

4. **`uartSendByte(uint8_t byte)`**
   - Sends single byte using bit-banging
   - Handles start bit, 8 data bits, stop bit
   - Precise timing for 115200 baud

5. **`uartReceiveByte()`**
   - Receives single byte using bit-banging
   - Waits for start bit, samples in middle
   - Returns received byte

6. **`sendCommandToESP(const String& command)`**
   - Sends AT command to ESP
   - Adds CR+LF termination
   - Uses bit-banging for transmission

7. **`readResponseFromESP(String& response, unsigned long timeout)`**
   - Reads response from ESP
   - Handles timeout
   - Detects complete responses (OK/ERROR)

8. **`testESPConnection()`**
   - Tests basic ESP communication
   - Sends "AT" command
   - Verifies "OK" response

9. **`testESPCommunication()`**
   - Comprehensive ESP communication test
   - Tests power up/down
   - Tests multiple AT commands
   - Tests WiFi functionality

## Testing Commands

The implementation tests these ESP AT commands:

1. **`AT`** - Basic communication test
2. **`AT+GMR`** - Get firmware version
3. **`AT+CWMODE?`** - Query WiFi mode
4. **`AT+CWLAP`** - Scan for WiFi networks

## Expected Behavior

### Successful Communication
```
|----------------------------------|
| Testing ESP Communication       |
|----------------------------------|
Powering up ESP microcontroller...
Powering up ESP/Modem power rail...
ESP powered up successfully
Initializing ESP serial communication...
ESP serial initialized (pins 42/43 at 115200 baud)
ESP serial initialization successful
|----------------------------------|
| Testing ESP Connection          |
|----------------------------------|
Sending test command to ESP...
Sending to ESP: AT
Command sent successfully
Reading response from ESP...
Received complete response: [OK]
ESP Response: OK
SUCCESS: ESP communication established!
ESP communication test successful!
Testing additional ESP commands...
Sending to ESP: AT+GMR
Command sent successfully
Reading response from ESP...
ESP Version: [version info]
Sending to ESP: AT+CWMODE?
Command sent successfully
Reading response from ESP...
ESP WiFi Mode: [mode info]
Sending to ESP: AT+CWLAP
Command sent successfully
Reading response from ESP...
ESP WiFi Scan: [network list]
Powering down ESP microcontroller...
ESP powered down successfully
|----------------------------------|
```

### Potential Issues

1. **No Response**
   - Check ESP power supply
   - Verify pin connections
   - Check ESP firmware (should respond to AT commands)

2. **Garbled Data**
   - Check baud rate timing
   - Verify pin assignments
   - Check for electrical interference

3. **Timeout Issues**
   - ESP may need more boot time
   - Check ESP firmware state
   - Verify power rail stability

## Hardware Requirements

- ESP microcontroller (ESP8266/ESP32)
- Proper power supply for ESP
- Correct pin connections
- ESP firmware that responds to AT commands

## Next Steps

1. Compile and upload to nRF52
2. Monitor RTT output for communication status
3. Verify ESP responds to AT commands
4. Test WiFi functionality if needed
5. Integrate with sensor data transmission

## Notes

- Bit-banging is used due to nRF52 UART configuration complexity
- Timing is critical for reliable communication
- Power management ensures proper ESP operation
- Error handling includes timeouts and response validation 