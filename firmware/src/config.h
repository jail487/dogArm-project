/**
 * Configuration file for dogArm firmware
 * 
 * Adjust these values based on your hardware setup
 */

#ifndef CONFIG_H
#define CONFIG_H

// Serial communication
#define SERIAL_BAUDRATE 115200

// Motor pins (adjust based on your wiring)
// Motor 1 (left arm)
#define MOTOR1_STEP_PIN 2
#define MOTOR1_DIR_PIN 3
#define MOTOR1_ENABLE_PIN 4

// Motor 2 (right arm)
#define MOTOR2_STEP_PIN 5
#define MOTOR2_DIR_PIN 6
#define MOTOR2_ENABLE_PIN 7

// Servo pin (brush z-axis)
#define SERVO_PIN 9

// Limit switch pins (for homing)
#define LIMIT_SWITCH_1_PIN 10
#define LIMIT_SWITCH_2_PIN 11

// Motor settings
#define STEPS_PER_REVOLUTION 200  // Standard for most stepper motors
#define MICROSTEPS 16              // Microstepping configuration
#define GEAR_RATIO 1.0             // If using gears, adjust this value
#define MAX_SPEED 1000.0           // Steps per second
#define MAX_ACCELERATION 500.0     // Steps per second^2
#define DEFAULT_SPEED 500.0        // Steps per second

// Servo settings
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define PEN_UP_ANGLE 120    // Angle when pen is up
#define PEN_DOWN_ANGLE 90   // Angle when pen is down

// Kinematic parameters (in mm)
// These define the geometry of the double parallel linkage
#define LINK_LENGTH_1 200.0  // Length of first link (upper arm)
#define LINK_LENGTH_2 200.0  // Length of second link (forearm)
#define BASE_WIDTH 100.0     // Distance between the two parallel linkage bases

// Workspace limits (in mm)
#define MIN_X -300.0
#define MAX_X 300.0
#define MIN_Y 50.0   // Minimum reach
#define MAX_Y 400.0  // Maximum reach
#define MIN_Z 0.0
#define MAX_Z 100.0

// Homing settings
#define HOMING_SPEED 200.0  // Speed during homing (steps per second)
#define HOMING_OFFSET 5.0   // Offset from limit switch (in degrees)

// Safety settings
#define ENABLE_SOFT_LIMITS true
#define EMERGENCY_STOP_PIN 12

#endif // CONFIG_H
