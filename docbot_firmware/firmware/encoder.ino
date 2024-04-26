#include <Arduino.h>

#define right_encoder_A 3 // Interrupt 
#define right_encoder_B 11

#define left_encoder_A 2 // Interrupt 
#define left_encoder_B 12
#define recution_ratio 120   // 기어박스
#define PPR 16            // 엔코더 PPR

unsigned int right_encoder_counter = 0;
unsigned int left_encoder_counter = 0;


const double RPM_TO_RAD_PER_SEC = 0.10472;  // RPM을 토대로 초당 라디안 값을 구하기 위한 상수

double right_wheel_meas_vel = 0.0;          // 측정 속도
double left_wheel_meas_vel = 0.0;  

String right_wheel_sign = "p";  // 각 바퀴의 현재 상태 (전진이냐 후진이냐)
String left_wheel_sign = "p";

unsigned long last_millis = 0;
const unsigned long interval = 100;


void interruptCallback();

void encoder_setup()
{
    pinMode(right_encoder_B, INPUT);
    pinMode(left_encoder_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(right_encoder_A), rightEncoderCallback, RISING);
    attachInterrupt(digitalPinToInterrupt(left_encoder_A), leftEncoderCallback, RISING);
}

void encoder_loop()
{
    unsigned long current_millis = millis();
    if(current_millis - last_millis >= interval)
    {
        last_millis = current_millis;
        interruptCallback();
    }
}



void calculateWheelVelocity()
{
    // PPR * recution_ratio = 한바퀴 회전에 발생하는 엔코더 펄스 수 / 기어박스가 100이면 바퀴가 1회 회전하면 모터는 100회 회전한다. 
    // right_encoder_counter / 한바퀴 회전에 발생하는 엔코더 펄스 수 = 바퀴가 회전한 수.
    right_wheel_meas_vel = (10 * right_encoder_counter * (60.0/(PPR * recution_ratio))) * RPM_TO_RAD_PER_SEC;
    left_wheel_meas_vel = (10 * left_encoder_counter * (60.0/(PPR * recution_ratio))) * RPM_TO_RAD_PER_SEC;
//    Serial.println("G, R, L");
//    Serial.print(1);
//    Serial.print(", ");
//    Serial.print(right_wheel_meas_vel);
//    Serial.print(", ");
//    Serial.println(left_wheel_meas_vel);

    right_encoder_counter = 0;
    left_encoder_counter = 0;
}

void sendWheelVelocity()
{
    String encoder_read = "r" + right_wheel_sign + String(right_wheel_meas_vel) + ",l" + left_wheel_sign + String(left_wheel_meas_vel) + ",";
    Serial.println(encoder_read);
}

void rightEncoderCallback()
{
    //Serial.println("1rightEncoderCallback");
    // if(digitalRead(right_encoder_B) == HIGH)
    // {
    //     right_wheel_sign = "p";
    // }
    // else
    // {
    //     right_wheel_sign = "n";
    // }

    right_encoder_counter++;
}
void leftEncoderCallback()
{
    // if(digitalRead(left_encoder_B) == HIGH)
    // {
    //     left_wheel_sign = "n";
    // }
    // else
    // {
    //     left_wheel_sign = "p";
    // }
    left_encoder_counter++;
}


// r * 각도 * (PI / 180) = 호의 길이

// r * 라디안 = 호의 길이 
