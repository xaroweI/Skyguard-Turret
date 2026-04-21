#include <Arduino.h>
#include <ESP32Servo.h>

#define PAN_SERVO_PIN 19
#define TILT_SERVO_PIN 18

#define SERIAL_BAUD_RATE 115200
#define SERIAL_TIMEOUT 500

#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

Servo panServo;
Servo tiltServo;

int commandedPanAngle = 90;
int commandedTiltAngle = 90;

char serialDataBuffer[64];
int bufferIndex = 0;

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

    panServo.write(90);
    tiltServo.write(90);

    Serial.println("Skyguard Turret Controller Ready. Awaiting commands...");
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

    panServo.write(commandedPanAngle);
    tiltServo.write(commandedTiltAngle);
}