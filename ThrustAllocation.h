/*
ThrustAllocation.h - library for allocating (distributing)
pwm signal to four Mallard's thrusters. The input is velocity vector
[x_dot y_dot  psi_dot]. Output is puls width modulation: pwm1 to pwm4.
Created by Konrad Gornicki, September 27, 2019.
*/

//Construct - prevents including the library twice:
#ifndef ThrustAllocation_h
#define ThrustAllocation_h

#include "Arduino.h"
#include <Servo.h>
// #include "geometry_msgs/Twist.h"


class ThrustAllocation
{
 public:
   //Constructor variables - pin assignment:
   ThrustAllocation(int pin_1, int pin_2, int pin_3, int pin_4);
   // Function that takes vellocity command vector tau, converts it to
   // pwm output to all four thrusters and returns chosen pwm signal for testing.
   double output_pwm(float x_body_input, float y_body_input, float psi_body_input);
 private:
   // outputs converted to pwm signals:
   Servo _thruster_1,_thruster_2,_thruster_3,_thruster_4;
   // Scalled inputs and outputs:
   double _x=0,_y=0,_psi=0,_pwm1=0, _pwm2=0,_pwm3=0,_pwm4=0;
   // limits for linear and angular thrust:
   double _linear_scale=200, _angular_scale=100;
   // Coefficients from Moore-Penrose pseudoinverse matrix:
   double _a=1.0556,_b=1.1955;
};

#endif
