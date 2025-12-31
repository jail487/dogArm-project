/**
 * Kinematics Module
 * 
 * Handles forward and inverse kinematics calculations
 * for the double parallel linkage mechanism
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>
#include "config.h"

class Kinematics {
public:
    Kinematics();
    
    // Initialize kinematics
    void init();
    
    // Calculate inverse kinematics
    // Input: target x, y position (mm)
    // Output: theta1, theta2 joint angles (degrees)
    // Returns: true if solution exists, false if out of reach
    bool inverseKinematics(float x, float y, float &theta1, float &theta2);
    
    // Calculate forward kinematics
    // Input: theta1, theta2 joint angles (degrees)
    // Output: x, y position (mm)
    void forwardKinematics(float theta1, float theta2, float &x, float &y);
    
    // Check if position is within workspace
    bool isInWorkspace(float x, float y);
    
private:
    float linkLength1;
    float linkLength2;
    float baseWidth;
    
    // Helper function to calculate angle using law of cosines
    float calculateAngle(float a, float b, float c);
};

#endif // KINEMATICS_H
