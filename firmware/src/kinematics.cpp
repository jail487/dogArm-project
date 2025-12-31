/**
 * Kinematics Module Implementation
 */

#include "kinematics.h"
#include <math.h>

Kinematics::Kinematics() 
    : linkLength1(LINK_LENGTH_1),
      linkLength2(LINK_LENGTH_2),
      baseWidth(BASE_WIDTH) {
}

void Kinematics::init() {
    // Nothing to initialize for now
}

bool Kinematics::inverseKinematics(float x, float y, float &theta1, float &theta2) {
    // Check if position is within workspace
    if (!isInWorkspace(x, y)) {
        return false;
    }
    
    // For a 2-DOF planar parallel linkage mechanism
    // This is a simplified implementation
    // Real implementation depends on exact mechanism geometry
    
    // Calculate distance from origin to target
    float distance = sqrt(x * x + y * y);
    
    // Check if reachable
    if (distance > (linkLength1 + linkLength2) || 
        distance < abs(linkLength1 - linkLength2)) {
        return false;
    }
    
    // Use law of cosines to find elbow angle
    float cosAngle = (linkLength1 * linkLength1 + linkLength2 * linkLength2 - distance * distance) /
                     (2.0 * linkLength1 * linkLength2);
    
    // Check if angle is valid
    if (cosAngle < -1.0 || cosAngle > 1.0) {
        return false;
    }
    
    float elbowAngle = acos(cosAngle);
    
    // Calculate base angle
    float baseAngle = atan2(y, x);
    float angle2 = asin((linkLength2 * sin(elbowAngle)) / distance);
    
    // Convert to degrees
    theta1 = (baseAngle - angle2) * 180.0 / PI;
    theta2 = (baseAngle + angle2) * 180.0 / PI;
    
    return true;
}

void Kinematics::forwardKinematics(float theta1, float theta2, float &x, float &y) {
    // Convert degrees to radians
    float theta1Rad = theta1 * PI / 180.0;
    float theta2Rad = theta2 * PI / 180.0;
    
    // For a simple 2-link arm
    // This is simplified - actual implementation depends on mechanism
    x = linkLength1 * cos(theta1Rad) + linkLength2 * cos(theta2Rad);
    y = linkLength1 * sin(theta1Rad) + linkLength2 * sin(theta2Rad);
}

bool Kinematics::isInWorkspace(float x, float y) {
    // Check against defined workspace limits
    if (x < MIN_X || x > MAX_X) {
        return false;
    }
    if (y < MIN_Y || y > MAX_Y) {
        return false;
    }
    
    // Check if within reachable radius
    float distance = sqrt(x * x + y * y);
    float maxReach = linkLength1 + linkLength2;
    float minReach = abs(linkLength1 - linkLength2);
    
    return (distance <= maxReach && distance >= minReach);
}

float Kinematics::calculateAngle(float a, float b, float c) {
    // Law of cosines: cos(C) = (a² + b² - c²) / (2ab)
    float cosAngle = (a * a + b * b - c * c) / (2.0 * a * b);
    
    // Clamp to valid range
    cosAngle = constrain(cosAngle, -1.0, 1.0);
    
    return acos(cosAngle);
}
