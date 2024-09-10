#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID{
  private:
    float kp, ki, kd;
    float eprev, eintegral;
  public:
    PID();
    void setParams(float Kp, float Ki, float Kd); 
    float compute(float value, float target, float deltaT);  
};

#endif