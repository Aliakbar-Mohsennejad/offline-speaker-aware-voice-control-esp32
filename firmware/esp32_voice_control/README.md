# Firmware (ESP32)

## Requirements
- Arduino IDE
- ESP32 board support installed
- INMP441 wired via I2S

## Install Edge Impulse Library
Sketch → Include Library → Add .ZIP Library  
Select: `ei-model/ei-esp32_voice_onoff_aliakbar-arduino-1.0.84.zip`

## Build & Upload
1. Open `esp32_voice_control.ino`
2. Select board: **ESP32 Dev Module**
3. Select correct COM port
4. Upload
5. Serial Monitor: **115200 baud**

## Pin Mapping
- GPIO14: I2S SCK
- GPIO25: I2S WS
- GPIO32: I2S SD
- GPIO2: Green LED
- GPIO4: Red LED

## Tuning
Thresholds and timing parameters are located in `config.h`.
