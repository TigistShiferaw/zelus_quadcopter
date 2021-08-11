/*
    Launches hub node that controls current desired and actual states
    Hub controls drone functions as well as communication between multiple froms of software
*/

#include <ros/ros.h>
#include "zqcopter/hub.h"

int main(int agrc, char **argv){
    ROS_INFO("Creating Hub...");

    ros::init(argc, argv, "hub");

    ros::NodeHandle nh;

    Hub hub;

    if(hub.init(&nh))
        ROS_INFO("SUCCESS");
    else
        ROS_INFO("FAILURE");

    return 0;
}