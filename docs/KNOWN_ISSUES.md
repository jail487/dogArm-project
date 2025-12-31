# Known Issues and Future Improvements

This document tracks known issues and planned improvements for the dogArm project.

## Current Limitations

### 1. Kinematics Implementation
**Status**: Placeholder implementation

**Description**: The current inverse and forward kinematics implementations use simplified equations for a basic 2-link serial arm. These equations serve as a starting point but do NOT accurately represent a true double parallel linkage mechanism.

**Impact**: 
- Position calculations will not be accurate for a parallel linkage
- End effector position may not match commanded positions

**Action Required**:
- Measure and document the exact geometry of your parallel linkage mechanism
- Derive the correct kinematic equations based on your mechanism
- Update both firmware (`kinematics.cpp`) and software (`inverse_kinematics.py`) with correct equations

**References**:
- For parallel linkage kinematics, consult robotics textbooks like "Introduction to Robotics" by Craig
- Consider using constraint-based approaches where both motors contribute to the same end effector position

### 2. Blocking Delays in Motor Control
**Status**: Acceptable for initial testing, should be improved

**Description**: The `penUp()` and `penDown()` functions use blocking `delay()` calls (200ms each) which pause the entire control loop.

**Impact**:
- Robot cannot process commands or update motor positions during servo movements
- May cause timing issues in real-time control scenarios

**Action Required**:
- Implement non-blocking servo control using state machines
- Track servo movement timing separately from main control loop
- Allow motor updates to continue during servo transitions

**Example approach**:
```cpp
// Non-blocking servo control
unsigned long servoMoveStartTime = 0;
bool servoMoving = false;
int servoTargetAngle = 0;

void setServoNonBlocking(int angle) {
    penServo.write(angle);
    servoTargetAngle = angle;
    servoMoveStartTime = millis();
    servoMoving = true;
}

void updateServo() {
    if (servoMoving && (millis() - servoMoveStartTime > 200)) {
        servoMoving = false;
    }
}
```

### 3. Simplified Character Data
**Status**: Examples only

**Description**: Only three simple characters (一, 十, 木) are included with basic stroke data.

**Action Required**:
- Add more characters
- Consider integrating with existing Chinese character databases like Makemeahanzi
- Implement stroke order and calligraphic styling

## Future Enhancements

### High Priority
1. **Accurate Kinematics**: Implement mechanism-specific equations
2. **Non-blocking Control**: Remove blocking delays from control loop
3. **Safety Features**: Add position limit checking, emergency stop button
4. **Calibration Routine**: Automated workspace calibration

### Medium Priority
1. **Path Optimization**: Smooth velocity profiles for calligraphy
2. **Brush Pressure Control**: Dynamic z-axis control based on stroke type
3. **Character Database**: Integration with existing character stroke databases
4. **GUI Application**: Desktop application for easier control

### Low Priority
1. **Network Control**: WiFi/Ethernet control in addition to serial
2. **Path Recording**: Record and replay brush movements
3. **Multi-character Composition**: Automatic layout for multiple characters
4. **Vision Feedback**: Camera-based verification of written characters

## Contributing

If you fix any of these issues or implement enhancements, please update this document and submit a pull request!

## Testing Checklist

Before using the robot with your specific hardware:

- [ ] Verify all pin assignments in `config.h`
- [ ] Measure and update link lengths
- [ ] Test motor directions and step counts
- [ ] Calibrate servo angles for pen up/down
- [ ] Verify workspace limits through manual testing
- [ ] Test homing sequence with limit switches
- [ ] Start with slow speeds and gradually increase
- [ ] Implement and test your mechanism-specific kinematics

## Version History

### v1.0.0 (Current)
- Initial project structure
- Placeholder kinematics implementation
- Basic serial control protocol
- Example characters and documentation
- Known issues documented
