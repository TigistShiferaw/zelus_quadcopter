#include "zqcopter/rcp_to_hub.h"

int init(ros::NodeHandle *nodehandle){
    nh = *nodehandle;
    cycle_rate = 10;

    // Publishers
    pose_pub = nh.advertise<geometry_msgs::Pose>("pose_rcp", 100);
    lin_vel_pub = nh.advertise<geometry_msgs::Vector3>("lin_vel_rcp", 100);

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

void publishPose(Pose &pose){
    // Create real pose out of temp Pose
    geometry_msgs::Pose msg;
    msg.pose.position.x = pose.p.x;
    msg.pose.position.y = pose.p.y;
    msg.pose.position.z = pose.p.z;
    msg.pose.orientation.x = pose.q.x;
    msg.pose.orientation.y = pose.q.y;
    msg.pose.orientation.z = pose.q.z;
    msg.pose.orientation.w = pose.q.w;

    pose_pub.publish(msg);
}

void publishLinearVels()