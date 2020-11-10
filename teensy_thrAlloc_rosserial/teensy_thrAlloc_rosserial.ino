
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <ThrustAllocationROS.h>

// ----- Variables -----
double pwm_out = 0; //return values for ThrustAllocation library
int state = 0; // to toggle LED
const int LED = 13;
std_msgs::Float64 test; // publisher's variables for testting


// ----- ROS and Calback declarations ------
void callback(const geometry_msgs::Twist&);
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub_twist("/mallard/thruster_command", &callback);
// ros::Publisher pub1("teensy_test_topic",&test); //for testing prurposes


// ----- Constructor ----- 
// Thruster initialization: Servo.attach() and Servo.writeMiliseconds(initial_value) 
// Each thruster gets its pin assignment (pin numbers) and initial pwm value. Must be outside of setup().
ThrustAllocationROS allocate(6,8,10,12); 

// ----- Setup -----
void setup()
{
  nh.initNode();
    
  //start subscriber and listen to incomming messages
  nh.subscribe(sub_twist); //if message present init callback and pass Twist message
  //adveriste on topic for testing
//   nh.advertise(pub1);

  // show output on led
  pinMode(LED,OUTPUT);
  digitalWrite(LED,HIGH);
  delay(1000);
}

// ----- Calback definition -----
void callback(const geometry_msgs::Twist& twist_msg)
{  

    u = twist_msg.linear.x;
    v = twist_msg.linear.y;
    r = twist_msg.angular.z;  

    // Thrust aloccation hapens here. Pass the vector of linear and angular velocities (u,v,r)
    // to get distributed vecor (pwm1,2,3 or 4) to all four thrusters of mallard. The pwm_out is 
    // for testing purposes only. Switch between return value of _pwm* inside ThrustAllocation.cpp to test different output.
    pwm_out = allocate.output_pwm(u,v,r);
    // This worked as well:
    // pwm_out = allocate.output_pwm(twist_msg);
 
    //Publish...
    // test.data = pwm_out;
    // pub1.publish(&test);
    
    //... and blink LED
    // digitalWrite(LED, (state) ? HIGH:LOW);
    // state = !state;
}

// ----- Main Loop ------
void loop()
{
  nh.spinOnce();
  delay(50); //miliseconds
}
