# MicroMethod - Arduino Pro Micro + NRF24L01 Wireless RC Controller

A wireless RC controller system using Arduino Pro Micro (sender) and Arduino Uno (receiver) with NRF24L01 2.4GHz modules.

## System Overview

```
vJoy (sim ratt + pedaler)
        ↓
Python Script (reads joystick)
        ↓
Serial (USB) → Arduino Pro Micro
        ↓
NRF24L01 Wireless (2.4GHz)
        ↓
Arduino Uno → PPM Signal → Servo/Motor Control
```

## Hardware Requirements

### Sender (Controller)
- Arduino Pro Micro (USB-C or Micro-USB)
- NRF24L01 + SMA Antenna
- USB cable to PC

### Receiver (Vehicle)
- Arduino Uno
- NRF24L01 + SMA Antenna
- Motor/Servo control circuit
- Power supply (5V for Arduino, separate 3.3V for NRF24)

## Wiring

### Both Arduino boards - NRF24L01 Connection:
```
NRF24L01    Arduino
─────────────────────
GND    →    GND
VCC    →    3.3V ⚠️ (with 10µF capacitor to GND)
CE     →    Pin 9
CSN    →    Pin 10
MOSI   →    Pin 11
MISO   →    Pin 12
SCK    →    Pin 13
```

### Arduino Uno - Motor/Servo Output:
```
PPM Output → Pin 5 (connect to servo/ESC signal)
```

## Installation

### 1. Arduino IDE Setup

Install required library:
- Sketch → Include Library → Manage Libraries
- Search for `RF24` by **TMRh20**
- Click Install

### 2. Upload Code

**Arduino Pro Micro (Sender):**
1. Open `ProMicroSender.ino`
2. Select Board: **Arduino Pro Micro**
3. Select Port: Your Pro Micro's COM port
4. Upload

**Arduino Uno (Receiver):**
1. Open `UnoReceiver.ino`
2. Select Board: **Arduino Uno**
3. Select Port: Your Uno's COM port
4. Upload

### 3. Python Setup

Install Python dependencies:
```bash
pip install pygame pyserial
```

## Usage

### On Windows
1. Plug in Arduino Pro Micro (USB-C)
2. Run Python script:
   ```bash
   python MicroMethodController.py
   ```
   or with specific COM port:
   ```bash
   python MicroMethodController.py --port COM7
   ```

### Joystick Input
- **Steering**: Left stick X-axis (or ratt simulation)
- **Throttle/Brake**: Right trigger or Y-axis

## Troubleshooting

### "NRF24 init failed!"
- Check wiring (especially CE, CSN, SPI pins)
- Verify 3.3V capacitor is connected
- Try both Arduino boards independently

### No wireless connection
- Make sure both boards have same **address**: `"00001"`
- Check antenna is securely connected
- Reduce distance to 1-2 meters for initial testing
- Check serial output with Arduino Serial Monitor

### Python script not finding joystick
- Run `python MicroMethodController.py` to see available joysticks
- Select correct index
- Verify joystick is recognized by Windows (Check Device Manager)

### "COM port not found"
- Check Arduino is connected
- Verify correct driver installed (Pro Micro drivers if needed)
- Run `python MicroMethodController.py --port COM7` (adjust COM port)

## Configuration

### Change Update Rate
Edit in `MicroMethodController.py`:
```python
UPDATE_HZ = 50  # 50Hz = 20ms per update
```

### Change RF Power Level
In Arduino code, change:
```cpp
radio.setPALevel(RF24_PA_MIN);  // Options: PA_MIN, PA_LOW, PA_HIGH, PA_MAX
```

### Change Frequency Channel
In Arduino code, add:
```cpp
radio.setChannel(76);  // 0-125 (2.4GHz channels)
```

## Safety Notes

⚠️ **Important:**
- NRF24L01 requires 3.3V power with 10µF capacitor - **never connect directly to 5V**
- Use separate power supply for NRF24 if possible
- Arduino Uno has built-in voltage regulator suitable for NRF24
- Pro Micro's 3.3V pin has limited current - consider external regulator for 2 modules
- Test range before flying/driving
- Implement failsafe on vehicle (return to neutral if signal lost)

## Performance

- **Refresh Rate**: 50Hz (20ms latency)
- **Range**: ~30-50 meters line-of-sight (depends on antenna)
- **Reliability**: Good with AMA antenna, excellent with SMA
- **Power Consumption**: ~10mA active, <1mA sleep

## Files

- `ProMicroSender.ino` - Arduino Pro Micro transmitter code
- `UnoReceiver.ino` - Arduino Uno receiver code
- `MicroMethodController.py` - Python controller script
- `README.md` - This file

## Author

Created for wireless RC vehicle control with sim racing peripherals.

## License

MIT License
