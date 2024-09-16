#include <Arduino.h>

#include "microROS.h"

float vel=0;

microRos ros;

void setup(){
    Serial.begin(115200);
    ros.initialize();   
}

void loop(){
    ros.start();
    ros.publish();
    //delay(100);
}