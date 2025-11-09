#include <Arduino.h>
#include <ESP32Servo.h>

#define PAN_SERVO_PIN 19
#define TILT_SERVO_PIN 18

#define SERIAL_BAUD_RATE 115200
#define SERIAL_TIMEOUT 500

#define SERVO_STOP 90
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define DEAD_ZONE 5
#define MAX_SPEED 30
#define CONTROL_LOOP_INTERVAL 10

Servo panServo;
Servo tiltServo;

volatile int commandedPanAngle = 90;
volatile int currentPanAngle = 90;
volatile int commandedTiltAngle = 90;
volatile int currentTiltAngle = 90;

unsigned long lastPanUpdate = 0;
unsigned long lastTiltUpdate = 0;

char serialDataBuffer[64];
int bufferIndex = 0;

void updateContinuousServo(Servo &servo, int commandedAngle, volatile int &currentAngle, unsigned long &lastUpdate) {
    int error = commandedAngle - currentAngle;
    int speedValue = SERVO_STOP;

    if (abs(error) > DEAD_ZONE) {
        int speedAdjustment = map(abs(error), DEAD_ZONE, 180, 1, MAX_SPEED);

        if (error > 0) {
            speedValue = SERVO_STOP - speedAdjustment;
        } else {
            speedValue = SERVO_STOP + speedAdjustment;
        }

        unsigned long now = millis();
        if (now - lastUpdate >= CONTROL_LOOP_INTERVAL) {
            int estimatedMovement = (speedValue < SERVO_STOP) ? 1 : -1;
            currentAngle = constrain(currentAngle + estimatedMovement, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
            lastUpdate = now;
        }
    }

    servo.write(speedValue);
}

void parseAndExecuteCommand() {
    if (strstr(serialDataBuffer, ";vt") != serialDataBuffer) {
        Serial.println("Err: Invalid format. Wat.");
        return;
    }

    int pan, tilt;
    int count = sscanf(serialDataBuffer, ";vt %d %d", &pan, &tilt);

    if (count == 2) {
        commandedPanAngle = constrain(pan, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        commandedTiltAngle = constrain(tilt, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

        Serial.print("Cmd parsed: PAN="); Serial.print(commandedPanAngle);
        Serial.print(" TILT="); Serial.println(commandedTiltAngle);

    } else {
        Serial.println("Err: Failed to parse numbers. Sad.");
    }
}

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.setTimeout(SERIAL_TIMEOUT);

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);

    panServo.setPeriodHertz(50);
    panServo.attach(PAN_SERVO_PIN, 500, 2500);

    tiltServo.setPeriodHertz(50);
    tiltServo.attach(TILT_SERVO_PIN, 500, 2500);

    panServo.write(SERVO_STOP);
    tiltServo.write(SERVO_STOP);
    
    currentPanAngle = 90;
    currentTiltAngle = 90;

    Serial.println("Skyguard Turret Controller Ready. (Dual 360-Mode) Awaiting commands...");
}

void loop() {
    while (Serial.available() > 0) {
        char incomingByte = Serial.read();

        if (incomingByte == '\n') {
            serialDataBuffer[bufferIndex] = '\0';
            parseAndExecuteCommand();
            bufferIndex = 0;
        } else if (incomingByte != '\r') {
            if (bufferIndex < 63) {
                serialDataBuffer[bufferIndex++] = incomingByte;
            } else {
                bufferIndex = 0;
            }
        }
    }

    updateContinuousServo(panServo, commandedPanAngle, currentPanAngle, lastPanUpdate);
    updateContinuousServo(tiltServo, commandedTiltAngle, currentTiltAngle, lastTiltUpdate);
}