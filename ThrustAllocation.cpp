/*
ThrustAllocation.h - library for allocating (distributing)
pwm signal to four Mallard's thrusters. The input is velocity vector
[x_dot y_dot  psi_dot].  Output is pwm1 to pwm4.
Created by Konrad Gornicki, September 27, 2019.
*/

#include "Arduino.h"
#include "ThrustAllocation.h"
#include <Servo.h>
// #include "geometry_msgs/Twist.h"

// ----- Constructor -----
ThrustAllocation::ThrustAllocation(int pin_1, int pin_2, int pin_3, int pin_4)
{   
    // Assign output pins to thrusters:
    _thruster_1.attach(pin_1);
    _thruster_2.attach(pin_2);
    _thruster_3.attach(pin_3);
    _thruster_4.attach(pin_4);
    //Initialize each thruster with pwm value corresponding to 0 force = 1500:
    _thruster_1.writeMicroseconds(1500);
    _thruster_2.writeMicroseconds(1500);
    _thruster_3.writeMicroseconds(1500);
    _thruster_4.writeMicroseconds(1500);
}

// ----- Functions -----
double ThrustAllocation::output_pwm(float x_body_input, float y_body_input, float psi_body_input)
{   
    //Takes [U V R] commands and scales them according to limits.
    _x = (x_body_input)*_linear_scale;
    _y = (y_body_input)*_linear_scale;
    _psi = (-psi_body_input)*_angular_scale;

    // Multiply velocity commands by pseudo-inverse (B+ * tau).
    _pwm1 = 1500 + 0.5*_x + _a*_psi;
    _pwm2 = 1500 + 0.5*_x - _a*_psi;
    _pwm3 = 1500 - 0.5*_y + _b*_psi;
    _pwm4 = 1500 - 0.5*_y - _b*_psi;

    //Translate to pwm signal (servo command):
    _thruster_1.writeMicroseconds(_pwm1);
    _thruster_2.writeMicroseconds(_pwm2);
    _thruster_3.writeMicroseconds(_pwm3);
    _thruster_4.writeMicroseconds(_pwm4);
    
    //for testing only, send to publisher()
    return _pwm1; 
}