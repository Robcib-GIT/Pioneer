#include "encoder.h"


extern volatile long ISR1counter; 
extern volatile long ISR2counter; 

void encoder_setup(){
    pinMode(ENC1A, INPUT);
    pinMode(ENC1B, INPUT);
    pinMode(ENC2A, INPUT);
    pinMode(ENC2B, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENC1A), doEncode1A, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC1B), doEncode1B, RISING);

    attachInterrupt(digitalPinToInterrupt(ENC2A), doEncode2A, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC2B), doEncode2B, RISING);
}


// Funciones de interrupción para el encoder contando solo RISING
void IRAM_ATTR doEncode1A() {
    if(digitalRead(ENC1B)){
        ISR1counter--;
    } else {
        ISR1counter++;
    }
}

void IRAM_ATTR doEncode1B() {
    if(digitalRead(ENC1A)){
        ISR1counter++;
    } else {
        ISR1counter--;
    }
    
}

void IRAM_ATTR doEncode2A() {
    if(digitalRead(ENC2B)){
        ISR2counter--;
    } else {
        ISR2counter++;
    }
}

void IRAM_ATTR doEncode2B() {
    if(digitalRead(ENC2A)){
        ISR2counter++;
    } else {
        ISR2counter--;
    }
}
