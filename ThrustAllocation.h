/*
ThrustAllocation.h - library for allocating (distributing)
pwm signal to four Mallard's thrusters. The input is velocity vector
[x_dot y_dot  psi_dot]. Output is pwm1 to pwm4.
Created by Konrad Gornicki, September 27, 2019.
*/

//Construct - prevents including the library twice:
#ifndef ThrustAllocation_h
#define ThrustAllocation_h

#include "Arduino.h"
#include <Servo.h>
#include "geometry_msgs/Twist.h"

class ThrustAllocation
{
 public:
   ThrustAllocation(int pin_1, int pin_2, int pin_3, int pin_4);
   double output_pwm(float x_body_input, float y_body_input, float psi_body_input);
 private:
   Servo _thruster_1,_thruster_2,_thruster_3,_thruster_4;
   double _x=0,_y=0,_psi=0,_pwm1=0, _pwm2=0,_pwm3=0,_pwm4=0;
   double _linear_scale=200, _angular_scale=100;
   double _a=1.0556,_b=1.1955;
};

#endif
