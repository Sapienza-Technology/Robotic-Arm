#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <Servo.h>
#include <math.h>

#define AXES 1

void setArmPos(const std_msgs::Float64MultiArray& arm_pos);
void setArmSpeed(const std_msgs::Float64MultiArray& arm_speed);

void servoWrite(float d);

inline int sgn(double x);

#endif