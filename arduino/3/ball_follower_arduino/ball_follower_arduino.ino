/*
 * Ball Follower Robot - Arduino Uno Firmware
 * Using AFMotor.h (Adafruit Motor Shield v1 with L293D)
 * 
 * Hardware:
 *   - Arduino Uno + Adafruit Motor Shield v1 (L293D)
 *   - 2x DC Motors on M1 (left) and M2 (right)
 *   - 1x Servo on shield Servo2 pin (pin 10)
 *   - 1x HC-SR04 Ultrasonic on A0 (TRIG) and A1 (ECHO)
 * 
 * AFMotor Shield pin usage:
 *   Pins 3, 4, 5, 6, 7, 8, 11, 12 → used by motor shield
 *   Pin 10                         → Servo2 header (our servo)
 *   Pin 9                          → Servo1 header (free)
 *   Pin 2, 13                      → FREE digital pins → HC-SR04
 *   Pin 0, 1                       → Serial TX/RX (reserved)
 * 
 * Serial Protocol (115200 baud):
 *   Receive from Pi:  M,leftSpeed,rightSpeed\n   (speeds: -255 to 255)
 *   Send to Pi:        S,angle,distance_cm\n       (scan data point)
 */

#include <AFMotor.h>
#include <Servo.h>

// ========== MOTOR SETUP (AFMotor) ==========
// Motor 1 = Left, Motor 2 = Right
// Change port numbers (1-4) if you wired differently
AF_DCMotor motorLeft(1);    // M1 terminal on shield
AF_DCMotor motorRight(2);   // M2 terminal on shield

// ========== SERVO ==========
// Use shield's Servo2 header (pin 10)
// Pin 9 is Servo1, Pin 10 is Servo2 — both are free from motor PWM
Servo scanServo;
#define SERVO_PIN 10

// ========== ULTRASONIC (HC-SR04) ==========
// Using free digital pins (not used by shield or servo)
#define TRIG  2
#define ECHO  13

// ========== CONFIGURATION ==========
#define SERIAL_BAUD   115200
#define SWEEP_DELAY   30     // ms between servo steps during scan
#define SERVO_STEP    5      // degrees per step during sweep
#define SCAN_INTERVAL 100    // ms between scan cycles
#define CMD_TIMEOUT   1000   // ms - stop motors if no command received

// ========== GLOBALS ==========
int leftSpeed = 0;
int rightSpeed = 0;
unsigned long lastCmdTime = 0;
unsigned long lastScanTime = 0;
int servoAngle = 0;
int servoDirection = 1;  // 1 = increasing, -1 = decreasing

String inputBuffer = "";

// ========== SETUP ==========
void setup() {
  Serial.begin(SERIAL_BAUD);
  
  // Ultrasonic pins
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  
  // Servo
  scanServo.attach(SERVO_PIN);
  scanServo.write(90);  // Start centered
  servoAngle = 0;       // Start sweep from 0
  
  // Motors off
  motorLeft.setSpeed(0);
  motorLeft.run(RELEASE);
  motorRight.setSpeed(0);
  motorRight.run(RELEASE);
  
  delay(500);
  Serial.println("READY");
}

// ========== MAIN LOOP ==========
void loop() {
  // 1. Read serial commands from Raspberry Pi
  readSerialCommands();
  
  // 2. Safety timeout - stop if no commands received
  if (millis() - lastCmdTime > CMD_TIMEOUT && lastCmdTime > 0) {
    stopMotors();
  }
  
  // 3. Sweep ultrasonic sensor and send readings
  if (millis() - lastScanTime >= SCAN_INTERVAL) {
    performScanStep();
    lastScanTime = millis();
  }
}

// ========== SERIAL COMMAND PARSING ==========
void readSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        parseCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
      // Prevent buffer overflow
      if (inputBuffer.length() > 50) {
        inputBuffer = "";
      }
    }
  }
}

void parseCommand(String cmd) {
  cmd.trim();
  
  if (cmd.startsWith("M,")) {
    // Motor command: M,leftSpeed,rightSpeed
    int firstComma = cmd.indexOf(',');
    int secondComma = cmd.indexOf(',', firstComma + 1);
    
    if (firstComma > 0 && secondComma > 0) {
      leftSpeed = cmd.substring(firstComma + 1, secondComma).toInt();
      rightSpeed = cmd.substring(secondComma + 1).toInt();
      
      // Clamp speeds
      leftSpeed = constrain(leftSpeed, -255, 255);
      rightSpeed = constrain(rightSpeed, -255, 255);
      
      setMotors(leftSpeed, rightSpeed);
      lastCmdTime = millis();
    }
  }
  else if (cmd == "STOP") {
    stopMotors();
    lastCmdTime = millis();
  }
}

// ========== MOTOR CONTROL (AFMotor) ==========
void setMotors(int left, int right) {
  // Left Motor (M1)
  if (left > 0) {
    motorLeft.setSpeed(left);
    motorLeft.run(FORWARD);
  } else if (left < 0) {
    motorLeft.setSpeed(-left);
    motorLeft.run(BACKWARD);
  } else {
    motorLeft.setSpeed(0);
    motorLeft.run(RELEASE);
  }
  
  // Right Motor (M2)
  if (right > 0) {
    motorRight.setSpeed(right);
    motorRight.run(FORWARD);
  } else if (right < 0) {
    motorRight.setSpeed(-right);
    motorRight.run(BACKWARD);
  } else {
    motorRight.setSpeed(0);
    motorRight.run(RELEASE);
  }
}

void stopMotors() {
  leftSpeed = 0;
  rightSpeed = 0;
  motorLeft.setSpeed(0);
  motorLeft.run(RELEASE);
  motorRight.setSpeed(0);
  motorRight.run(RELEASE);
}

// ========== ULTRASONIC SCANNING ==========
void performScanStep() {
  // Move servo to current angle
  scanServo.write(servoAngle);
  delay(SWEEP_DELAY);  // Wait for servo to settle
  
  // Take distance measurement
  float distance = getDistance();
  
  // Send scan data to Raspberry Pi: S,angle,distance
  Serial.print("S,");
  Serial.print(servoAngle);
  Serial.print(",");
  Serial.println(distance, 1);
  
  // Send current motor power status: P,leftPWM,rightPWM
  Serial.print("P,");
  Serial.print(leftSpeed);
  Serial.print(",");
  Serial.println(rightSpeed);
  
  // Update servo angle for next step
  servoAngle += servoDirection * SERVO_STEP;
  
  // Reverse direction at limits
  if (servoAngle >= 180) {
    servoAngle = 180;
    servoDirection = -1;
  } else if (servoAngle <= 0) {
    servoAngle = 0;
    servoDirection = 1;
  }
}

float getDistance() {
  // Send trigger pulse
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  
  // Read echo with timeout (30ms = ~500cm max)
  long duration = pulseIn(ECHO, HIGH, 30000);
  
  if (duration == 0) {
    return 500.0;  // No echo = max range
  }
  
  // Convert to cm (speed of sound = 343 m/s)
  float distance = duration * 0.0343 / 2.0;
  
  // Clamp to reasonable range
  if (distance < 2.0) distance = 2.0;
  if (distance > 400.0) distance = 400.0;
  
  return distance;
}
