/**
 * Motor Control Module Implementation
 */

#include "motor_control.h"
#include "kinematics.h"

extern Kinematics kinematics;

MotorControl::MotorControl() 
    : motor1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN),
      motor2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN),
      currentTheta1(0),
      currentTheta2(0),
      currentZ(PEN_UP_ANGLE),
      currentSpeed(DEFAULT_SPEED) {
}

void MotorControl::init() {
    // Initialize motor 1
    motor1.setMaxSpeed(MAX_SPEED);
    motor1.setAcceleration(MAX_ACCELERATION);
    motor1.setSpeed(currentSpeed);
    pinMode(MOTOR1_ENABLE_PIN, OUTPUT);
    digitalWrite(MOTOR1_ENABLE_PIN, LOW); // Enable motor
    
    // Initialize motor 2
    motor2.setMaxSpeed(MAX_SPEED);
    motor2.setAcceleration(MAX_ACCELERATION);
    motor2.setSpeed(currentSpeed);
    pinMode(MOTOR2_ENABLE_PIN, OUTPUT);
    digitalWrite(MOTOR2_ENABLE_PIN, LOW); // Enable motor
    
    // Initialize servo
    penServo.attach(SERVO_PIN);
    penUp();
    
    // Initialize limit switches
    pinMode(LIMIT_SWITCH_1_PIN, INPUT_PULLUP);
    pinMode(LIMIT_SWITCH_2_PIN, INPUT_PULLUP);
}

void MotorControl::moveTo(float theta1, float theta2, float z) {
    currentTheta1 = theta1;
    currentTheta2 = theta2;
    currentZ = z;
    
    // Convert angles to steps and command motors
    long steps1 = degreesToSteps(theta1);
    long steps2 = degreesToSteps(theta2);
    
    motor1.moveTo(steps1);
    motor2.moveTo(steps2);
    
    // Set servo position based on z
    int servoAngle = map(z, MIN_Z, MAX_Z, PEN_DOWN_ANGLE, PEN_UP_ANGLE);
    servoAngle = constrain(servoAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    penServo.write(servoAngle);
}

void MotorControl::update() {
    motor1.run();
    motor2.run();
}

bool MotorControl::isAtTarget() {
    return (motor1.distanceToGo() == 0) && (motor2.distanceToGo() == 0);
}

void MotorControl::home() {
    // Simple homing routine - move to limit switches
    motor1.setSpeed(HOMING_SPEED);
    motor2.setSpeed(HOMING_SPEED);
    
    // Move until limit switches are triggered
    while (digitalRead(LIMIT_SWITCH_1_PIN) == HIGH) {
        motor1.move(-1);
        motor1.run();
    }
    
    while (digitalRead(LIMIT_SWITCH_2_PIN) == HIGH) {
        motor2.move(-1);
        motor2.run();
    }
    
    // Set current position as zero
    motor1.setCurrentPosition(0);
    motor2.setCurrentPosition(0);
    
    // Move to offset position
    long offsetSteps = degreesToSteps(HOMING_OFFSET);
    motor1.moveTo(offsetSteps);
    motor2.moveTo(offsetSteps);
    
    while (!isAtTarget()) {
        update();
    }
    
    currentTheta1 = HOMING_OFFSET;
    currentTheta2 = HOMING_OFFSET;
    
    // Restore speed
    motor1.setSpeed(currentSpeed);
    motor2.setSpeed(currentSpeed);
    
    // Pen up after homing
    penUp();
}

void MotorControl::stop() {
    motor1.stop();
    motor2.stop();
}

void MotorControl::setSpeed(float speed) {
    currentSpeed = speed;
    motor1.setSpeed(speed);
    motor2.setSpeed(speed);
}

void MotorControl::penUp() {
    penServo.write(PEN_UP_ANGLE);
    currentZ = MAX_Z;
    delay(200); // Wait for servo to move
}

void MotorControl::penDown() {
    penServo.write(PEN_DOWN_ANGLE);
    currentZ = MIN_Z;
    delay(200); // Wait for servo to move
}

void MotorControl::getCurrentPosition(float &x, float &y, float &z) {
    // Use forward kinematics to get Cartesian position
    kinematics.forwardKinematics(currentTheta1, currentTheta2, x, y);
    z = currentZ;
}

long MotorControl::degreesToSteps(float degrees) {
    return (long)((degrees / 360.0) * STEPS_PER_REVOLUTION * MICROSTEPS * GEAR_RATIO);
}

float MotorControl::stepsToDegrees(long steps) {
    return (float)(steps) / (STEPS_PER_REVOLUTION * MICROSTEPS * GEAR_RATIO) * 360.0;
}
