
 /* rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "DualVNH5019MotorShield.h"


DualVNH5019MotorShield md;

ros::NodeHandle  nh;


void motor_cb( const geometry_msgs::Twist& twist){

  
  int M1Speed = 100*twist.linear.x;
  int M2Speed = 100*twist.linear.x;
  
  int steering_angle= twist.angular.z;
  
  if (steering_angle > 1) {
      M1Speed = -M1Speed;
  }
  else if (steering_angle < -1) {
      M2Speed = -M2Speed;
  }
  else if ((steering_angle > -1) && (steering_angle < 1)) {
      M1Speed = M1Speed;
      M2Speed = M2Speed;
  }


    md.setM1Speed(M1Speed);
    md.setM2Speed(M2Speed);

}


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", motor_cb);

void setup(){
  pinMode(13, OUTPUT);
  md.init();
  nh.initNode();
  nh.subscribe(sub);
  delay(1);

}

void loop(){
 
 nh.spinOnce();

  delay(1);
}
