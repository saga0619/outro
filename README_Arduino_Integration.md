# Arduino Worker Integration Documentation

## Overview
This document describes the integration of Arduino communication functionality as a threaded worker in the main PyQt5 application.

## Files Modified/Created

### 1. `ardu_worker.py` (New)
- **ArduinoWorker class**: Thread-safe Arduino communication worker
- **Features**:
  - Serial communication with CRC-8 validation
  - Sine wave brightness modulation (0.05 Hz frequency)
  - Thread-safe status monitoring with locks
  - Automatic reconnection on communication errors
  - Configurable COM port and baud rate

### 2. `main.py` (Modified)
- **Added Arduino worker integration**
- **New UI elements support**:
  - `label_led1` ~ `label_led6`: LED status indicators
  - `label_sw1`, `label_sw2`: Switch status indicators  
  - `pushButton_motoron`: Motor on/off control button
  - `textBrowser`: Logging display widget

### 3. `test_arduino_thread.py` (New)
- Standalone test script for Arduino worker functionality

## UI Elements Connected

### LED Status Labels (`label_led1` ~ `label_led6`)
- **Purpose**: Display LED brightness status from Arduino
- **Behavior**: 
  - Green background when LED brightness > 50% (ON)
  - Gray background when LED brightness ≤ 50% (OFF)
  - Text shows "LEDx: ON/OFF"

### Switch Status Labels (`label_sw1`, `label_sw2`)
- **Purpose**: Display switch states from Arduino
- **Behavior**:
  - Blue background when switch is pressed (ON)
  - Gray background when switch is released (OFF)
  - Text shows "SWx: ON/OFF"

### Motor Control Button (`pushButton_motoron`)
- **Purpose**: Control Arduino motor signal
- **Behavior**:
  - Green background with "MOTOR ON" text when off
  - Red background with "MOTOR OFF" text when on
  - Sends signal=1 when motor on, signal=0 when motor off

### Logging Display (`textBrowser`)
- **Purpose**: Display all application logs in real-time
- **Features**:
  - Custom QTextBrowserHandler for logging
  - Auto-scroll to latest messages
  - Timestamp formatting: HH:MM:SS
  - All logging levels supported (INFO, WARNING, ERROR, DEBUG)

## Arduino Communication Protocol

### Data Packet Structure
```
[Signal][Brightness1][Brightness2][Brightness3][Brightness4][Brightness5][Brightness6][CRC]
```
- **Signal**: 1 byte (0=motor off, 1=motor on)
- **Brightness**: 6 bytes (0-255 brightness values for LEDs)
- **CRC**: 1 byte CRC-8 checksum

### Response Packet Structure
```
[DigitalOutput][Brightness1][Brightness2][Brightness3][Brightness4][Brightness5][SwitchStates][CRC]
```
- **DigitalOutput**: 1 byte digital output status
- **Brightness**: 6 bytes received brightness values
- **SwitchStates**: 1 byte switch states (bit 0=SW1, bit 1=SW2)
- **CRC**: 1 byte CRC-8 checksum

## Key Features

### Thread Safety
- Uses `threading.Lock` for safe communication between GUI and worker threads
- Arduino worker runs independently from motor worker
- Status updates are thread-safe

### Error Handling
- Robust error handling with automatic reconnection attempts
- CRC validation for data integrity
- Connection status monitoring
- Error count tracking

### Real-time Updates
- 10Hz GUI update rate for real-time status display
- Continuous Arduino communication at configurable rate (default 10Hz)
- Sine wave brightness modulation for visual feedback

### Logging System
- All logs displayed in QTextBrowser widget
- Configurable log levels
- Timestamped messages
- Auto-scroll functionality

## Usage

### Starting the Application
```bash
python main.py
```

### Arduino Worker Behavior
1. **Auto-start**: Arduino worker starts automatically when application launches
2. **Connection**: Attempts to connect to COM4 (configurable)
3. **Communication**: Sends data packets at 10Hz with sine wave brightness modulation
4. **Status Updates**: Updates GUI elements in real-time
5. **Auto-cleanup**: Properly stops when application closes

### Motor Control
- Click "MOTOR ON" button to send signal=1 to Arduino
- Click "MOTOR OFF" button to send signal=0 to Arduino
- Button color changes to indicate current state

### Monitoring
- LED status updates automatically based on Arduino response
- Switch states update automatically based on Arduino input
- All communication logged to textBrowser widget

## Configuration

### COM Port
Default: COM4
To change: Modify the `port` parameter in `start_arduino_worker()` call

### Communication Rate
Default: 10Hz (100ms interval)
To change: Modify the `tick` parameter in ArduinoWorker constructor

### Brightness Modulation
Default: 0.05Hz sine wave
To change: Modify the `frequency` variable in ArduinoWorker.run()

## Troubleshooting

### Arduino Not Connected
- Check COM port availability
- Verify Arduino is connected and powered
- Check baud rate (default: 115200)
- Look for connection errors in log display

### Communication Errors
- CRC mismatch: Check data integrity
- Timeout errors: Check Arduino response timing
- Connection lost: Worker will attempt automatic reconnection

### UI Elements Not Found
- Ensure UI elements exist in mainwindow.ui file
- Check element names match exactly (case-sensitive)
- Missing elements will be logged as warnings
