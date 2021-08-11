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

#include <iostream>
#include <sstream>

// Structs to temporarily hold the information until they can
// be converted to ROS messages
struct Point {
    double x;
    double y;
    double z;
};

struct Quat {
    double x;
    double y;
    double z;
    double w;
};

struct Pose {
    Point p;
    Quat q;
};

struct Twist {
    int xdot;
    int ydot;
    int zdot;
    // Ignore angular velocities for now
};

// ROS objects
ros::NodeHandle nh;

// refresh rate of communication (i.e. 10 would mean node runs updates at a rate of 10hz)
int cycle_rate;
int cycles;

// Ros publishers
ros::Publisher apose_pub; // actual pose
ros::Publisher dpose_pub; // desired pose
ros::Publisher twist_pub; // actual twist

// Initializes all publishers and subscribers
// Returns 0 if succesful and -1 if error occurred
int init(ros::NodeHandle *nodehandle);

// Spins node and handles all recurring update functions (not including callbacks)
void run();

void publishActualPose(Pose &pose);
void publishDesiredPose(Pose &pose);
void publishTwist(Twist &twist);