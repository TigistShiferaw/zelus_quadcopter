#ifndef ZEL_TO_HUB_H_
#define ZEL_TO_HUB_H_

/* 
    Communication layer between zelus c functions and the ROS hub
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <sstream>
#include "zqcopter/hub.h"
#include <boost/shared_ptr.hpp>

// ROS objects
ros::NodeHandle nh;

// refresh rate of communication (i.e. 10 would mean node runs updates at a rate of 10hz)
int cycle_rate;
int cycles;

// Ros publishers
ros::Publisher velocityLimit_pub;
ros::Publisher positionBounds_pub;
ros::Publisher state_pub;

// Ros subscribers
ros::Subscriber pose_sub;


// Initializes all publishers and subscribers
// Returns 0 if succesful and -1 if error occurred
int init(ros::NodeHandle *nodehandle);

// Spins node and handles all recurring update functions (not including callbacks)
void run();

//void publishDesiredPose(Pose &pose);
void publishXdotLimit(int xdot);
void publishYdotLimit(int ydot);
void publishZdotLimit(int zdot);
void publishVelocityLimits(int xdot, int ydot, int zdot);
void publishXBounds(int x_lower, int x_upper);
void publishYBounds(int y_lower, int y_upper);
void publishZBounds(int z_lower, int z_upper);
void publishDesiredMode(char mode)
void publishAutonDesiredState(char state);

void getPose(double *poseContainer);
void getLinearVelocities(double *linearVels);

#endif