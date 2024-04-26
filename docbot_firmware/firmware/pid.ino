#include <PID_v1.h>

// PID에 의해 보정된 속도값
double right_wheel_pidOutput = 0.0;            
double left_wheel_pidOutput = 0.0;             

// PID Config
// P - 초기값을 목표값으로 도달하는 속도 조절.
// i - 목표치에 더 근접하게 조절.
// d - 오버슈팅 제어.
double Kp_r = 70;
double Ki_r = 25;
double Kd_r = 2;

double Kp_l = 70;
double Ki_l = 20; 
double Kd_l = 4;

extern double right_wheel_meas_vel;
extern double left_wheel_meas_vel;
extern double right_wheel_cmd_vel;
extern double left_wheel_cmd_vel;

PID rightMotor(&right_wheel_meas_vel, &right_wheel_pidOutput, &right_wheel_cmd_vel, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&left_wheel_meas_vel, &left_wheel_pidOutput, &left_wheel_cmd_vel, Kp_l, Ki_l, Kd_l, DIRECT);
void pid_setup()
{
    rightMotor.SetMode(AUTOMATIC);
    leftMotor.SetMode(AUTOMATIC);
}

void calculatePID()
{
    rightMotor.Compute();
    leftMotor.Compute();
   
    if(right_wheel_cmd_vel <= 0.10)
    {
        right_wheel_pidOutput = 0.0;
    }
    if(left_wheel_cmd_vel <= 0.10)
    {
        left_wheel_pidOutput = 0.0;
    }
}
