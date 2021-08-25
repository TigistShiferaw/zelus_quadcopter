#include "zqcopter/rcp_to_hub.h"

int init(ros::NodeHandle *nodehandle){
    nh = *nodehandle;
    cycle_rate = 10;

    // Publishers
    apose_pub = nh.advertise<geometry_msgs::Pose>("apose", 1);
    atwist_pub = nh.advertise<geometry_msgs::Twist>("atwist", 1);

    run();
}


void run(){
    // Sets refresh 
    ros::Rate loop_rate(cycle_rate);
    cycles = 0;

    while(ros::ok()){

        // End of cycle
        cycles++;
        loop_rate.sleep();
        ros::spinOnce();
    }

    ROS_INFO("Cleaning up rcp_to_hub layer...");
}