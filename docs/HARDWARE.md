# Hardware Setup Guide

## Components List

### Electronic Components

1. **Microcontroller**
   - ESP32 Development Board (recommended) OR
   - Arduino Mega 2560

2. **Motors**
   - 2x NEMA 17 Stepper Motors (1.8° step angle, 200 steps/rev)
   - 2x A4988 or DRV8825 Stepper Motor Drivers
   - 1x SG90 or MG996R Servo Motor (for brush control)

3. **Power Supply**
   - 12V DC Power Supply (minimum 3A for stepper motors)
   - 5V regulator (if not provided by microcontroller board)

4. **Sensors**
   - 2x Limit Switches (for homing)

5. **Miscellaneous**
   - Jumper wires
   - Breadboard or PCB
   - USB cable (for programming)
   - Power cables

### Mechanical Components

1. **Frame Structure**
   - Aluminum extrusion or acrylic sheets
   - Double parallel linkage arms (4 links total)
   - Pivot joints (bearings or bushings)

2. **Mounting Hardware**
   - Motor mounts
   - Limit switch mounts
   - Base plate

3. **End Effector**
   - Brush holder assembly
   - Servo mount
   - Brush clamp mechanism

## Wiring Diagram

### ESP32 Pinout

```
ESP32 Pin Configuration:
------------------------
Motor 1 (Left):
  - STEP: GPIO 2
  - DIR:  GPIO 3
  - EN:   GPIO 4

Motor 2 (Right):
  - STEP: GPIO 5
  - DIR:  GPIO 6
  - EN:   GPIO 7

Servo (Brush):
  - PWM:  GPIO 9

Limit Switches:
  - SW1:  GPIO 10 (with pullup)
  - SW2:  GPIO 11 (with pullup)

Power:
  - VIN:  12V input (for stepper drivers)
  - 5V:   Servo power
  - GND:  Common ground
```

### Stepper Driver Connections

Each A4988/DRV8825 driver:
```
Driver Pins:
-----------
- STEP → Microcontroller STEP pin
- DIR  → Microcontroller DIR pin
- EN   → Microcontroller EN pin
- VDD  → 5V (logic power)
- GND  → Ground
- VMOT → 12V (motor power)
- GND  → Ground
- 1A, 1B, 2A, 2B → Stepper motor coils
```

**Important**: Set motor current using the potentiometer on the driver!
- Formula: Vref = Current × 8 × Rcs (for A4988)
- Typical Vref: 0.4V - 0.8V for NEMA 17

### Servo Connection

```
Servo Pins:
----------
- Signal (Orange/Yellow) → GPIO 9
- VCC (Red)              → 5V
- GND (Brown/Black)      → Ground
```

## Mechanical Assembly

### 1. Base Frame

1. Cut aluminum extrusion or acrylic to size
2. Create base plate with mounting holes
3. Install motor mounts at calculated base width (default: 100mm apart)

### 2. Linkage Arms

1. **Upper Arms** (Link 1):
   - Length: 200mm (adjust in config.h if different)
   - Attach to motor shafts using couplers
   - Install bearings at far end

2. **Forearms** (Link 2):
   - Length: 200mm (adjust in config.h if different)
   - Connect to upper arms via pivot joints
   - Connect both forearms at end effector point

### 3. End Effector Assembly

1. Mount servo to end effector plate
2. Attach brush holder to servo arm
3. Ensure brush can move vertically (pen up/down)
4. Adjust servo angles in config.h for proper up/down positions

### 4. Limit Switches

1. Position switches at home position of each motor
2. Wire with pull-up resistors (or use internal pull-ups)
3. Test triggering before final mounting

## Calibration

### 1. Motor Current

1. Measure stepper motor rated current
2. Calculate Vref for driver
3. Adjust driver potentiometer with multimeter
4. Start with lower current and increase if needed

### 2. Link Lengths

1. Measure actual link lengths precisely
2. Update `LINK_LENGTH_1` and `LINK_LENGTH_2` in `config.h`
3. Measure base width between motor axes
4. Update `BASE_WIDTH` in `config.h`

### 3. Servo Angles

1. Connect servo and upload firmware
2. Manually test angles:
   ```
   PEN:UP
   PEN:DOWN
   ```
3. Adjust `PEN_UP_ANGLE` and `PEN_DOWN_ANGLE` in `config.h`
4. Recompile and upload

### 4. Workspace Limits

1. Manually move robot to extreme positions
2. Record maximum and minimum reachable coordinates
3. Update workspace limits in `config.h`:
   - `MIN_X`, `MAX_X`
   - `MIN_Y`, `MAX_Y`
   - `MIN_Z`, `MAX_Z`

### 5. Speed and Acceleration

1. Start with conservative values (already set in config.h)
2. Gradually increase if motors can handle it
3. Watch for:
   - Motor stalling
   - Skipped steps
   - Excessive vibration
4. Reduce if any issues occur

## Testing

### 1. Power-On Test

1. Connect power (motors off)
2. Check voltages: 12V, 5V
3. Enable motors
4. Check for heating (normal: slightly warm)

### 2. Single Motor Test

1. Disconnect one motor from driver
2. Test other motor with simple moves
3. Verify direction and step count
4. Repeat for second motor

### 3. Home Test

1. Manually position robot near limit switches
2. Send `HOME` command
3. Verify both motors move to switches
4. Check for proper offset movement

### 4. Movement Test

1. Home robot
2. Command simple moves: `MOVE:100,200,10`
3. Verify smooth motion
4. Check position accuracy

### 5. Pen Control Test

1. `PEN:UP` - verify pen lifts
2. `PEN:DOWN` - verify pen lowers
3. Adjust angles if needed

## Troubleshooting

**Motors not moving**:
- Check power supply
- Verify enable pins (LOW = enabled)
- Check wiring
- Verify current setting

**Motors skipping steps**:
- Increase motor current (carefully)
- Reduce speed/acceleration
- Check mechanical binding

**Erratic movement**:
- Check for loose wires
- Verify stepper driver configuration (microstepping)
- Check power supply capacity

**Servo jittering**:
- Check power supply (servos need stable 5V)
- Add capacitor near servo (100µF-1000µF)
- Verify signal wire connection

**Limit switches not working**:
- Verify wiring and pull-up configuration
- Test with multimeter in continuity mode
- Check pin configuration in firmware
