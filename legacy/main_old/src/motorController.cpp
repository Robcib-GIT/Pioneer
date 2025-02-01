#include "motorController.h"


motorController::motorController(int threshold){
    pinMode(pinForward, OUTPUT);
    pinMode(pinBackward, OUTPUT);
    pinMode(pinEnable, OUTPUT);

    this->threshold = threshold;

    previousPosition = 0;
    previousEncoderTime = 0;
    previousPIDTime = 0;
    
}

void motorController::pinoutSetup(u_int8_t pinForward, u_int8_t pinBackward, u_int8_t pinEnable, u_int8_t pinEncoderA, u_int8_t pinEncoderB){
    this->pinForward = pinForward;
    this->pinBackward = pinBackward;
    this->pinEnable = pinEnable;
    this->pinEncoderA = pinEncoderA;
    this->pinEncoderB = pinEncoderB;
}

void motorController::pwmSetup(int pwmChannel, int pwmFreq, int pwmResolution){
    this->pwmChannel = pwmChannel;
    this->pwmFreq = pwmFreq;
    this->pwmResolution = pwmResolution;

    ledcSetup(pwmChannel, pwmFreq, pwmResolution);
    ledcAttachPin(pinEnable, pwmChannel);
}

void motorController::encoderSetup(int tickPerRevolution, void (*ISR)()){
    pinMode(pinEncoderA, INPUT_PULLUP);
    pinMode(pinEncoderB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pinEncoderA), ISR, RISING);

    this->tickPerRevolution = tickPerRevolution;

    encoderCount.data = 0;
}


void motorController::setPIDParams(float kp, float ki, float kd){
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;

    eintegral = 0;
    previousError = 0;
}

float motorController::calculateRPM(){
    long currentPosition = encoderCount.data;
    unsigned long currentEncoderTime = millis();
    float delta = ((float)currentEncoderTime - previousEncoderTime) / 1.0e3;
    float velocity = ((float)currentPosition - previousPosition) / delta;
    float rpm = (velocity / tickPerRevolution) * 60;
    
    rpmFilt = 0.854 * rpmFilt + 0.0728 * rpm + 0.0728 * rpmPrev;
    rpmPrev = rpm;
    previousPosition = currentPosition;
    previousEncoderTime = currentEncoderTime;
    return rpmFilt;
}

float motorController::computePID(float setpoint, float feedback){
    unsigned long currentPIDTime = millis();
    float delta = ((float)currentPIDTime - previousPIDTime) / 1.0e3;

    float error = setpoint - feedback;
    eintegral = eintegral + (error * delta);
    float ederivative = (error - previousError) / delta;
    float control_signal = (kp * error) + (ki * eintegral) + (kd * ederivative);

    previousError = error;
    previousPIDTime = currentPIDTime;
    return control_signal;
}

void motorController::move(float actuatingSignal){
    digitalWrite(pinForward, actuatingSignal >= 0 ? HIGH : LOW);
    digitalWrite(pinBackward, actuatingSignal >= 0 ? LOW : HIGH);

    int pwm = threshold + (int)fabs(actuatingSignal);

    if (pwm > 255)
      pwm = 255;
    ledcWrite(pwmChannel, pwm);
}

void motorController::stop(){
    digitalWrite(pinForward, LOW);
    digitalWrite(pinBackward, LOW);
    ledcWrite(pwmChannel, 0);
}
