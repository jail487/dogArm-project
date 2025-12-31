/**
 * dogArm - Calligraphy Robot Arm Firmware
 * 
 * This firmware controls a double parallel linkage robotic arm
 * designed for writing Chinese calligraphy.
 * 
 * Hardware:
 * - ESP32 or Arduino Mega microcontroller
 * - 2x stepper motors for the parallel linkage
 * - 1x servo motor for brush z-axis control
 * 
 * Communication:
 * - Serial protocol at 115200 baud
 * - Command format: "CMD:param1,param2\n"
 */

#include <Arduino.h>
#include "config.h"
#include "motor_control.h"
#include "kinematics.h"
#include "communication.h"

// Global objects
MotorControl motors;
Kinematics kinematics;
Communication comm;

// System state
bool isHomed = false;
bool isMoving = false;

void setup() {
    // Initialize serial communication
    Serial.begin(SERIAL_BAUDRATE);
    while (!Serial && millis() < 3000); // Wait for serial on USB boards
    
    Serial.println(F("dogArm Firmware v1.0"));
    Serial.println(F("Initializing..."));
    
    // Initialize motors
    motors.init();
    Serial.println(F("Motors initialized"));
    
    // Initialize kinematics
    kinematics.init();
    Serial.println(F("Kinematics initialized"));
    
    // Initialize communication
    comm.init();
    Serial.println(F("Communication initialized"));
    
    Serial.println(F("Ready! Waiting for commands..."));
    Serial.println(F("Use 'HOME' to home the robot"));
}

void loop() {
    // Process incoming commands
    if (comm.hasCommand()) {
        String cmd = comm.getCommand();
        processCommand(cmd);
    }
    
    // Update motor positions
    if (isMoving) {
        motors.update();
        
        // Check if movement is complete
        if (motors.isAtTarget()) {
            isMoving = false;
            comm.sendResponse("OK", "Movement complete");
        }
    }
    
    // Small delay to prevent CPU hogging
    delay(1);
}

void processCommand(String cmd) {
    cmd.trim();
    
    if (cmd.startsWith("HOME")) {
        // Home the robot
        Serial.println(F("Homing..."));
        motors.home();
        isHomed = true;
        comm.sendResponse("OK", "Homed");
        
    } else if (cmd.startsWith("MOVE:")) {
        // Move to Cartesian coordinates
        // Format: MOVE:x,y,z
        if (!isHomed) {
            comm.sendResponse("ERROR", "Not homed. Use HOME first");
            return;
        }
        
        String params = cmd.substring(5);
        float x, y, z;
        if (parseCoordinates(params, x, y, z)) {
            // Calculate inverse kinematics
            float theta1, theta2;
            if (kinematics.inverseKinematics(x, y, theta1, theta2)) {
                motors.moveTo(theta1, theta2, z);
                isMoving = true;
                comm.sendResponse("OK", "Moving");
            } else {
                comm.sendResponse("ERROR", "Invalid position (out of reach)");
            }
        } else {
            comm.sendResponse("ERROR", "Invalid coordinates format");
        }
        
    } else if (cmd.startsWith("SPEED:")) {
        // Set movement speed
        // Format: SPEED:value
        String params = cmd.substring(6);
        float speed = params.toFloat();
        if (speed > 0 && speed <= MAX_SPEED) {
            motors.setSpeed(speed);
            comm.sendResponse("OK", "Speed set");
        } else {
            comm.sendResponse("ERROR", "Invalid speed value");
        }
        
    } else if (cmd.startsWith("PEN:")) {
        // Control pen up/down
        // Format: PEN:UP or PEN:DOWN
        String params = cmd.substring(4);
        params.trim();
        if (params == "UP") {
            motors.penUp();
            comm.sendResponse("OK", "Pen up");
        } else if (params == "DOWN") {
            motors.penDown();
            comm.sendResponse("OK", "Pen down");
        } else {
            comm.sendResponse("ERROR", "Invalid pen command (use UP or DOWN)");
        }
        
    } else if (cmd.startsWith("STATUS")) {
        // Get current status
        String status = "Homed:" + String(isHomed ? "1" : "0") + 
                       ",Moving:" + String(isMoving ? "1" : "0");
        comm.sendResponse("STATUS", status);
        
    } else if (cmd.startsWith("POS")) {
        // Get current position
        float x, y, z;
        motors.getCurrentPosition(x, y, z);
        String pos = String(x) + "," + String(y) + "," + String(z);
        comm.sendResponse("POS", pos);
        
    } else if (cmd.startsWith("STOP")) {
        // Emergency stop
        motors.stop();
        isMoving = false;
        comm.sendResponse("OK", "Stopped");
        
    } else {
        comm.sendResponse("ERROR", "Unknown command");
    }
}

bool parseCoordinates(String params, float &x, float &y, float &z) {
    int comma1 = params.indexOf(',');
    int comma2 = params.indexOf(',', comma1 + 1);
    
    if (comma1 == -1 || comma2 == -1) {
        return false;
    }
    
    x = params.substring(0, comma1).toFloat();
    y = params.substring(comma1 + 1, comma2).toFloat();
    z = params.substring(comma2 + 1).toFloat();
    
    return true;
}
