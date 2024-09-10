#include <Arduino.h>

#include "pinout.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"

const float motorReduction = 38.3;
const float pulleyReduction = 1.3;
const float encoderResolution = 500;

volatile long ISR1counter = 0;
volatile long ISR2counter = 0;

unsigned long previousTime = 0;

// Variables para el filtro paso-bajo
float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

PID pid1;
PID pid2;

void setup() {
    motor_setup();
    encoder_setup();
    Serial.begin(57600);
    //Serial.begin(115200);  

    pid1.setParams(25, 100, 0.5);
    pid2.setParams(30, 110, 1);
    
}

void loop() {
    // Calcula el incremento de tiempo entre iteraciones del loop
    unsigned long currentTime = micros();
    float deltaT = ((float) (currentTime-previousTime))/1.0e6;
    previousTime = currentTime;

    // Calcula la velocidad de los motores en pulsos por unidad de tiempo
    noInterrupts();
    float velocity1 = ISR1counter/deltaT;
    float velocity2 = ISR2counter/deltaT;
    ISR1counter=0;
    ISR2counter=0;
    interrupts();

    // Convierte la velocidad de pulsos por unidad de tiempo a radianes por segundo
    float v1 = velocity1*2*PI/(encoderResolution* motorReduction* pulleyReduction * 2);
    float v2 = velocity2*2*PI/(encoderResolution* motorReduction* pulleyReduction * 2);

    // Aplica a la velocidad un filtro de primer orden paso-bajo con frecuencia de corte en 25Hz
    v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
    v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
    v1Prev = v1;
    v2Prev = v2;

    // Señal de referencia
    float vt = 8*(sin(currentTime/1e6)>0); 
    //float vt=5*sin(prevT/1e6);
    

    // Calcula la señal de control
    float u1=pid1.compute(v1Filt, vt, deltaT);
    float u2=pid2.compute(v2Filt, vt, deltaT);

    // Actúa sobre el motor
    motor((int)u1, (int)u2);
    
    // Para mostrar la gráfica del ajuste del PID en el Serial Plotter
    Serial.print(vt);
    Serial.print(" ");
    Serial.print(v1Filt);
    Serial.print(" ");
    Serial.print(v2Filt);
    Serial.println();

    //delay(1);

}

