# dogArm Firmware

This directory contains the embedded firmware for the dogArm calligraphy robot.

## Structure

```
firmware/
├── src/
│   ├── main.cpp              # Main program entry point
│   ├── config.h              # Hardware configuration
│   ├── motor_control.h/cpp   # Motor control module
│   ├── kinematics.h/cpp      # Kinematics calculations
│   └── communication.h/cpp   # Serial communication
├── lib/                      # Custom libraries (if any)
└── platformio.ini            # PlatformIO configuration
```

## Supported Platforms

- **ESP32** (recommended)
- **Arduino Mega 2560**

## Building and Uploading

### Using PlatformIO

```bash
# Build for ESP32
pio run -e esp32dev

# Upload to ESP32
pio run -e esp32dev -t upload

# Build for Arduino Mega
pio run -e megaatmega2560

# Upload to Arduino Mega
pio run -e megaatmega2560 -t upload

# Open serial monitor
pio device monitor
```

### Using Arduino IDE

1. Install required libraries:
   - AccelStepper (v1.64 or higher)
   - Servo (v1.2.1 or higher)

2. Open `src/main.cpp` in Arduino IDE

3. Select your board (ESP32 or Arduino Mega)

4. Select your port

5. Click Upload

## Configuration

Edit `src/config.h` to match your hardware:

### Pin Configuration
- Motor step/direction/enable pins
- Servo pin
- Limit switch pins

### Mechanical Parameters
- Link lengths (L1, L2)
- Base width
- Workspace limits

### Motion Parameters
- Maximum speed and acceleration
- Homing speed
- Pen up/down angles

## Serial Protocol

Baudrate: 115200

### Commands

| Command | Description | Format | Response |
|---------|-------------|--------|----------|
| HOME | Home robot | `HOME` | `OK:Homed` |
| MOVE | Move to position | `MOVE:x,y,z` | `OK:Moving` |
| SPEED | Set speed | `SPEED:value` | `OK:Speed set` |
| PEN | Control pen | `PEN:UP/DOWN` | `OK:Pen up/down` |
| STATUS | Get status | `STATUS` | `STATUS:...` |
| POS | Get position | `POS` | `POS:x,y,z` |
| STOP | Emergency stop | `STOP` | `OK:Stopped` |

## Modules

### motor_control
- Controls stepper motors and servo
- Handles acceleration profiles
- Provides high-level motion commands

### kinematics
- Forward kinematics calculation
- Inverse kinematics solver
- Workspace validation

### communication
- Serial protocol parser
- Command buffering
- Response formatting

## Customization

### Adding New Commands

1. Add command handler in `processCommand()` in `main.cpp`
2. Implement command logic
3. Send response using `comm.sendResponse()`

### Modifying Kinematics

Edit `kinematics.cpp` to implement your specific mechanism's kinematics equations.

### Changing Motor Configuration

Update `motor_control.cpp` to support different motor types (e.g., servos instead of steppers).

## Debugging

Enable verbose output by uncommenting debug prints in the code.

Monitor serial output:
```bash
pio device monitor
```

## License

MIT License
