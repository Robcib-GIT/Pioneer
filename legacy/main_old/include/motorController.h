#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <std_msgs/msg/int32.h>

class motorController{
    private:
        //pinout
        u_int8_t pinForward;
        u_int8_t pinBackward;
        u_int8_t pinEnable;
        u_int8_t pinEncoderA;
        u_int8_t pinEncoderB;

        //PWM
        int pwmChannel;
        int pwmFreq;
        int pwmResolution;

        //PID
        float kp;
        float ki;
        float kd;
        float eintegral;
        float previousError;
        unsigned long previousPIDTime;

        // Encoder
        std_msgs__msg__Int32 encoderCount;
        volatile long previousPosition;
        unsigned long previousEncoderTime;

        int tickPerRevolution;
        float rpmFilt;
        float rpmPrev;
        
        int threshold;

    public:
        motorController(int threshold);
        void pinoutSetup(u_int8_t pinForward, u_int8_t pinBackward, u_int8_t pinEnable, u_int8_t pinEncoderA, u_int8_t pinEncoderB);
        void pwmSetup(int pwmChannel, int pwmFreq, int pwmResolution);
        void encoderSetup(int tickPerRevolution, void (*ISR)());
        void setPIDParams(float kp , float ki, float kd);
        float calculateRPM();
        float computePID(float setpoint, float feedback);
        void move(float actuatingSignal);
        void stop();

        int getPinEncoderA(){return pinEncoderA;};
        int getPinEncoderB(){return pinEncoderB;};

        void incrementEncoderCount(){encoderCount.data++;};
        void decrementEncoderCount(){encoderCount.data--;};
        std_msgs__msg__Int32 getEncoderCount(){return encoderCount;};

};

#endif