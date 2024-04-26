#include <Arduino.h>

int EN_R = 6;
int EN_L = 5; 

int IN_1 = 7;
int IN_2 = 8; 
int IN_3 = 9;
int IN_4 = 10; 

// 사용자 명령
// ROS에서 사용자가 입력한 명령을 전달함 
double right_wheel_cmd_vel = 0.0;   // 사용자가 입력한 속도 0~255
double left_wheel_cmd_vel = 0.0;  
bool is_right_wheel_cmd = false;    // 바퀴에 대한 명령 여부.
bool is_left_wheel_cmd = false;
bool is_right_wheel_forward = true; // 전진, 후진 여부
bool is_left_wheel_forward = true;
char value[] = "00.00";             // 사용자가 입력한 속도
uint8_t value_idx = 0;

extern double right_wheel_pidOutput;
extern double left_wheel_pidOutput;

extern String right_wheel_sign;  // 각 바퀴의 현재 상태 (전진이냐 후진이냐)
extern String left_wheel_sign;

void motor_setup()
{
  pinMode(EN_R, OUTPUT);
  pinMode(EN_L, OUTPUT);

  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);

  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, HIGH);
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, HIGH);
}

void motor_loop()
{
  // 모터 제어
  if(Serial.available())
  {

    // ros에서 사용자의 명령에 대한 모터 제어 명령을 문자열 형태로 전달한다. 
    char chr = Serial.read();

    if(chr == 'r' || chr =='l') setWheelCommand(chr);           // 각 바퀴에 대한 명령 여부 설정
    else if (chr == 'p' || chr == 'n')
    {
      // 엔코더 고장으로 인한 임시 방편.
      if(is_right_wheel_cmd) 
      {
        right_wheel_sign = chr;
      }

      if(is_left_wheel_cmd)
      {
        left_wheel_sign = chr;
      }

      setWheelDirection(chr);  // 전진, 후진 설정.
    } 
    else if (chr == ',') setWheelVelocity();                    // 속도 설정.
    else                                                        
    {
      if(value_idx < 5)
      {
          value[value_idx] = chr;
          value_idx++;
      }
    }
  }
}


void setWheelCommand(char chr)
{
    is_right_wheel_cmd = (chr == 'r') ? true : false;
    is_left_wheel_cmd = (chr == 'r') ? false : true;
    value_idx = 0;
}

void setWheelDirection(char chr)
{

  bool forward = (chr == 'p') ? true : false;

  if(is_right_wheel_cmd && (forward != is_right_wheel_forward))
    toggleWheelDirection(is_right_wheel_forward, IN_1, IN_2);
  
  if(is_left_wheel_cmd && (forward != is_left_wheel_forward))
    toggleWheelDirection(is_left_wheel_forward, IN_3, IN_4);
}

void toggleWheelDirection(bool& is_wheel_forward, int pin1, int pin2) {
    digitalWrite(pin1, HIGH - digitalRead(pin1));
    digitalWrite(pin2, HIGH - digitalRead(pin2));
    is_wheel_forward = !is_wheel_forward;
}

void setWheelVelocity() {
    if(is_right_wheel_cmd)
      right_wheel_cmd_vel = atof(value);
    
    else if(is_left_wheel_cmd)
      left_wheel_cmd_vel = atof(value);
    

    value_idx = 0;
    strcpy(value, "00.00");
}


void applyCorrectedSpeedToMotors()
{
  analogWrite(EN_R, right_wheel_pidOutput);
  analogWrite(EN_L, left_wheel_pidOutput);
}
