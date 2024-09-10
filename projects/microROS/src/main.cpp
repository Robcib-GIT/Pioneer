#include <Arduino.h>

#include "motor.h"
#include "microROS.h"

float vel=0;

microRos ros;

void setup(){
    motor_setup();
    Serial.begin(115200);

    ros.initialize();   
}

void loop(){
    ros.start();
    motor(vel, vel);
}