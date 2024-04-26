/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/
#include <Arduino.h>
#include <PID_v1.h>

#define PIN_INPUT 0
#define PIN_OUTPUT 3

//연결할 변수 정의
double Setpoint, Input, Output;

//링크 및 초기 조정 매개변수 지정
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
  //initialize the variables we're linked to
  Input = analogRead(PIN_INPUT);
  Setpoint = 100;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  Input = analogRead(PIN_INPUT);
  myPID.Compute();
  analogWrite(PIN_OUTPUT, Output);
}


