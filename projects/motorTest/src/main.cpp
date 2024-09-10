#include <Arduino.h>

#include "pinout.h"

const int motorFreq = 1000;     // PWM frequency
const int motorResolution = 8;  // PWM resolution (bits)
const int motorChannelA = 0;    // PWM channel for motor A
const int motorChannelB = 1;    // PWM channel for motor B

// put function declarations here:
void motor(int, int);

void setup() {
    pinMode(M1INA, OUTPUT);
    pinMode(M1INB, OUTPUT);
    pinMode(M1PWM, OUTPUT);
    ledcSetup(motorChannelA, motorFreq, motorResolution);
    ledcAttachPin(M1PWM, motorChannelA);

    pinMode(M2INA, OUTPUT);
    pinMode(M2INB, OUTPUT);
    pinMode(M2PWM, OUTPUT);
    ledcSetup(motorChannelB, motorFreq, motorResolution);
    ledcAttachPin(M2PWM, motorChannelB);
}

void loop() {
    motor(50, 50);
    delay(2000);
    motor(0, 0);
    delay(500);
    motor(-50, -50);
    delay(2000);
    motor(0, 0);
    delay(500);
}

void motor(int vi, int vd) {
    digitalWrite(M1INA, vi >= 0 ? HIGH : LOW);
    digitalWrite(M1INB, vi >= 0 ? LOW : HIGH);

    digitalWrite(M2INA, vd >= 0 ? HIGH : LOW);
    digitalWrite(M2INB, vd >= 0 ? LOW : HIGH);

    ledcWrite(motorChannelA, constrain(abs(vi), 0, 255));
    ledcWrite(motorChannelB, constrain(abs(vd), 0, 255));
}