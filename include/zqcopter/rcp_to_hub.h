/*
    This communication layer facilitates communication of states
    between rc_pilot codebase and the hub ( which is the main ROS state manager )
    This will be used both ways.
    1.) To communicate actual states to the hub
    2.) To accept desired states from the hub
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include "zqcopter/hub.h"
#include "zqcopter/transition_structs.h"

#include <iostream>
#include <sstream>

// ROS objects
ros::NodeHandle nh;

// refresh rate of communication (i.e. 10 would mean node runs updates at a rate of 10hz)
int cycle_rate;
int cycles;

// Ros publishers
ros::Publisher pose_pub; // actual pose
ros::Publisher lin_vel_pub;

// Initializes all publishers and subscribers
// Returns 0 if succesful and -1 if error occurred
int init(ros::NodeHandle *nodehandle);

// Spins node and handles all recurring update functions (not including callbacks)
void run();

void publishPose(Pose &pose);
void publishLinearVels(Vec3 lin_vels);
void publishState(char state);
void publishMode(char mode);