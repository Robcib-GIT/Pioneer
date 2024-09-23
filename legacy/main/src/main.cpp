#include "main.h"

//parameters of the robot
const float wheels_y_distance = 0.41;
const float wheel_radius = 0.11;
const float wheel_circumference = 2 * 3.14 * wheel_radius;

const float motorReduction = 38.3;
const float pulleyReduction = 1.3;
const float encoderResolution = 500;
const int tickPerRevolution = motorReduction * pulleyReduction * encoderResolution;

extern std_msgs__msg__Int32 encodervalue1;
extern std_msgs__msg__Int32 encodervalue2;


motorController motor1(13);
motorController motor2(13);

void setup(){
    
    motor1.pinoutSetup(M1INA, M1INB, M1PWM, ENC1A, ENC1B);
    motor2.pinoutSetup(M2INA, M2INB, M2PWM, ENC2A, ENC2B);

    motor1.pwmSetup(0, 30000, 8);
    motor2.pwmSetup(1, 30000, 8);

    motor1.encoderSetup(tickPerRevolution, doEncode1);
    motor2.encoderSetup(tickPerRevolution, doEncode2);

    Serial.begin(115200);
    
    initialize();   
}

void loop(){
    start();
}


//interrupt function for left wheel encoder.
void doEncode1() {
    if (digitalRead(motor1.getPinEncoderB()))
        motor1.incrementEncoderCount();
    else
        motor1.decrementEncoderCount();
    encodervalue1 = motor1.getEncoderCount();
}

//interrupt function for right wheel encoder
void doEncode2() {
    if (digitalRead(motor2.getPinEncoderA()))
        motor2.incrementEncoderCount();
    else
        motor2.decrementEncoderCount();
    encodervalue2 = motor2.getEncoderCount();
}
