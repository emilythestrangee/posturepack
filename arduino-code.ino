#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <AccelStepper.h>

// MPU6050 sensor
Adafruit_MPU6050 mpu;

// Stepper motor pins
#define MOTOR1_IN1_PIN 4
#define MOTOR1_IN2_PIN 5
#define MOTOR1_IN3_PIN 6
#define MOTOR1_IN4_PIN 7
#define MOTOR2_IN1_PIN 8
#define MOTOR2_IN2_PIN 9
#define MOTOR2_IN3_PIN 10
#define MOTOR2_IN4_PIN 11

// Stepper motors
AccelStepper motor1(AccelStepper::FULL4WIRE, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, MOTOR1_IN3_PIN, MOTOR1_IN4_PIN);
AccelStepper motor2(AccelStepper::FULL4WIRE, MOTOR2_IN1_PIN, MOTOR2_IN2_PIN, MOTOR2_IN3_PIN, MOTOR2_IN4_PIN);

// Control parameters
const int speedForward = 500;  // Speed for forward movement
const int speedBackward = -500; // Speed for backward movement
const int speedStop = 0;       // Speed when stopping
unsigned long lastReadTime = 0;
const int readInterval = 50;  // Read MPU6050 every 50ms

void setup() {
    Serial.begin(9600);
    Wire.begin();

    // Initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip!");
        while (1);
    }
    Serial.println("MPU6050 Found!");

    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Stepper settings
    motor1.setMaxSpeed(1000);
    motor1.setAcceleration(5000);
    motor2.setMaxSpeed(1000);
    motor2.setAcceleration(5000);
}

void loop() {
    unsigned long currentTime = millis();

    // Read MPU6050 at intervals to avoid slowing loop
    if (currentTime - lastReadTime >= readInterval) {
        lastReadTime = currentTime;

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // Calculate roll angle using accelerometer data
        float roll = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;

        // Apply angle-based control
        if (roll > 50) {
            stopMotors();  // Stop motors if angle > 50
        } else if (roll < 25) {
            moveMotorsForward();  // Move forward if angle < 25
        } else {
            moveMotorsBackward();  // Move backward if angle is in between
        }

        Serial.print("Roll: ");
        Serial.print(roll);
        Serial.print(" | Motor Speed: ");
        Serial.println(motor1.speed());
    }

    // Keep motors running
    motor1.runSpeed();
    motor2.runSpeed();
}

// Stops the motors
void stopMotors() {
    motor1.setSpeed(speedStop);
    motor2.setSpeed(speedStop);
}

// Moves motors forward
void moveMotorsForward() {
    motor1.setSpeed(speedForward);
    motor2.setSpeed(speedForward);
}

// Moves motors backward
void moveMotorsBackward() {
    motor1.setSpeed(speedBackward);
    motor2.setSpeed(speedBackward);
}