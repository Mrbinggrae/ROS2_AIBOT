/********************************************************
 * PID 적응형 튜닝 예
 * PID 라이브러리의 장점 중 하나는 언제든지 튜닝 매개변수를 변경할 수 있다는 것입니다.
 * 이는 컨트롤러가 어떤 때는 공격적이지만 다른 때는 보수적이기를 원할 때 도움이 될 수 있습니다.
 * 아래 예에서는 설정점에 가까울 때 보수적인 튜닝 매개변수를 사용하고,
 * 멀리 있을 때 더 공격적인 튜닝 매개변수를 사용하도록 컨트롤러를 설정했습니다.
 ********************************************************/
#include <Arduino.h>
#include <PID_v1.h>

#define PIN_INPUT 0
#define PIN_OUTPUT 3

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//공격적이고 보수적인 튜닝 매개변수 정의
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

//링크 및 초기 조정 매개변수 지정
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

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

  double gap = abs(Setpoint-Input); //distance away from setpoint
  if (gap < 10)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }

  myPID.Compute();
  analogWrite(PIN_OUTPUT, Output);
}


