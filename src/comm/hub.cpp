#include "zqcopter/hub.h"

int Hub::init(ros::NodeHandle *nodehandle){
    nh = *nodehandle;

    // Create all publishers

    // Create all subscribers
    apose_sub = nh.subscribe("apose_rcp", 1, &Hub::apose_callback, this);
    dpose_sub = nh.subscribe("dpose_zq", 1, &Hub::dpose_callback, this);
    

    run();
}

void Hub::run(){
    ros::Rate loop_rate(cycle_rate);
    cycles = 0;

    // Start Spinning while ros is stable
    while(ros::ok()){

        loop_rate.sleep();
        ros::spinOnce();
    }

    // Any items that need to be cleaned up
    cleanup();
}

void Hub::cleanup(){
    ros::shutdown();
}

void Hub::apose_callback(const geometry_msgs::Pose::ConstPtr &msg){

}

void Hub::dpose_callback(const geometry_msgs::Pose::ConstPtr &msg){

}