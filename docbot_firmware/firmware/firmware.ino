#include <Arduino.h>


void motor_setup();
void encoder_setup();
void pid_setup();
void calculateWheelVelocity();
void resetAndPrintEncoderCounters();

void motor_loop();
void encoder_loop();
void calculatePID();
void applyCorrectedSpeedToMotors();

void setup()
{
    Serial.begin(115200);
    
    motor_setup();
    encoder_setup();
    pid_setup();

}

void loop()
{
    motor_loop();
    encoder_loop();
}

void interruptCallback()
{
    calculateWheelVelocity();
    sendWheelVelocity();
    calculatePID();
    applyCorrectedSpeedToMotors();
}
