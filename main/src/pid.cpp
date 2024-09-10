#include "pid.h"

// Constructor
PID::PID(){
    kp = 1.0;
    kd = 0.0;
    ki = 0.0;
    eprev = 0.0;
    eintegral = 0.0;
}

// Asigna los valores de los parámetros del controlador
void PID::setParams(float Kp, float Ki, float Kd){
    kp = Kp; 
    ki = Ki;
    kd = Kd;
}

// Calcula la señal de control
float PID::compute(float value, float target, float deltaT){
    float e = target - value; // error
    float dedt = (e-eprev)/(deltaT); // derivativo
    eintegral += e*deltaT; // integral
    eprev = e; // guarda el error previo para la siguiente iteración
    return kp*e + kd*dedt + ki*eintegral; //señal de control
}