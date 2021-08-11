#include "zqcopter/hubcomm.h"

int init(ros::NodeHandle *nodehandle){
    nh = *nodehandle;
    cycle_rate = 10;

    // Publishers
    apose_pub = nh.advertise<geometry_msgs::Pose>("apose", 1);
    dpose_pub = nh.advertise<geometry_msgs::Pose>("dpose", 1);
    atwist_pub = nh.advertise<geometry_msgs::Twist>("atwist", 1);
    dtwist_pub = nh.advertise<geometry_msgs::Twist>("dtwist", 1);


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