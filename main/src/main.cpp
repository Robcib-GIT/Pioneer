#include <Arduino.h>

#include "pinout.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "microROS.h"

const float motorReduction = 38.3;
const float pulleyReduction = 1.3;
const float encoderResolution = 500;

volatile long ISR1counter = 0;
volatile long ISR2counter = 0;

unsigned long previousTime = 0;

float vel=0;

// Variables para el filtro paso-bajo
float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

TaskHandle_t microRosTask;

microRos microros;
PID pid1;
PID pid2;

void microRosTask_code(void * pvParameters){
    microros.initialize();
    while(1){
        microros.start();
    }
}

void setup() {
    motor_setup();
    encoder_setup();

    xTaskCreatePinnedToCore(microRosTask_code, "microROS_subscriber", 10000, NULL, 1, &microRosTask, 0);            

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
    microros.start();
    //float vt = 8*(sin(currentTime/1e6)>0); 
    //float vt=5*sin(prevT/1e6);
    

    // Calcula la señal de control
    float u1=pid1.compute(v1Filt, vel, deltaT);
    float u2=pid2.compute(v2Filt, vel, deltaT);

    // Actúa sobre el motor
    motor((int)u1, (int)u2);

    delay(1);

}

