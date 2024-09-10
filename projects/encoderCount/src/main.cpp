#include <Arduino.h>

#include "pinout.h"

long ISRcounter = 0;

void doEncodeA();
void doEncodeB();

void setup() {
    pinMode(ENC2A, INPUT);
    pinMode(ENC2B, INPUT);
    // Si los declaramos como INPUT_PULLUP, al no recibir ningún pulso, el valor será HIGH

    attachInterrupt(digitalPinToInterrupt(ENC1A), doEncodeA, RISING);
    // attachInterrupt(digitalPinToInterrupt(ENC1B), doEncodeB, CHANGE);
    // Cuidado que hay que dividir entre 2, 4 según lo que contemos
}

void loop() {
    Serial.begin(115200);
    Serial.println("Encoder test");
    while (1) {
        Serial.print("ISRcounter: ");
        Serial.println(ISRcounter);
        delay(1000);
    }
}

// Funciones de interrupción para el encoder
void IRAM_ATTR doEncodeA() {
    ISRcounter++;
}

void IRAM_ATTR doEncodeB() {
    ISRcounter++;
}
