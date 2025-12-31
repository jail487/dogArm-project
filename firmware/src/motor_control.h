/**
 * Motor Control Module
 * 
 * Handles stepper motor control for the parallel linkage
 * and servo control for the brush z-axis
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>
#include "config.h"

class MotorControl {
public:
    MotorControl();
    
    // Initialize motors and servo
    void init();
    
    // Move to specified angles (in degrees)
    void moveTo(float theta1, float theta2, float z);
    
    // Update motor positions (call this in loop)
    void update();
    
    // Check if motors have reached target position
    bool isAtTarget();
    
    // Home the robot
    void home();
    
    // Stop all motors
    void stop();
    
    // Set movement speed (steps per second)
    void setSpeed(float speed);
    
    // Pen control
    void penUp();
    void penDown();
    
    // Get current position
    void getCurrentPosition(float &x, float &y, float &z);
    
private:
    AccelStepper motor1;
    AccelStepper motor2;
    Servo penServo;
    
    float currentTheta1;
    float currentTheta2;
    float currentZ;
    
    float currentSpeed;
    
    // Convert degrees to steps
    long degreesToSteps(float degrees);
    
    // Convert steps to degrees
    float stepsToDegrees(long steps);
};

#endif // MOTOR_CONTROL_H
