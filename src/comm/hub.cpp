#include "zqcopter/hub.h"

int Hub::init(ros::NodeHandle *nodehandle){
    nh = *nodehandle;

    // Create all publishers
    pose_pub = nh.advertise<geometry_msgs::Pose>("pose_hub", 100);
    xbounds_pub = nh.advertises<std_msgs::String>("xbounds_hub", 100);
    ybounds_pub = nh.advertises<std_msgs::String>("ybounds_hub", 100);
    zbounds_pub = nh.advertises<std_msgs::String>("zbounds_hub", 100);
    linvel_pub = nh.advertise<geometry_msgs::Vector3>("linvel_hub", 100);
    mode_pub = nh.advertise<std_msgs::String>("mode_hub", 100);
    state_pub = nh.advertise<std_msgs::String>("state_hub", 100);
    maxvel_pub = nh.advertise<geometry_msgs::Vector3>("maxvel_hub", 100);

    

    // Create all subscribers
    pose_sub = nh.subscribe("pose_rcp", 1, &Hub::pose_callback, this);
    vel_lim_sub = nh.subscribe("vel_limit_zel", &Hub::vel_lim_callback, this);
    pos_bounds_sub = nh.subscribe("pos_bounds_zel", &Hub::pos_bounds_callback, this);
    desired_state_sub = nh.subscribe("state_zel", &Hub::desired_state_callback, this);

    run();
}

void Hub::run(){
    ros::Rate loop_rate(cycle_rate);
    cycles = 0;

    // Start Spinning while ros is stable
    while(ros::ok()){

        // Publish all
        publishAll();


        loop_rate.sleep();
        ros::spinOnce();
    }

    // Any items that need to be cleaned up
    cleanup();
}

void Hub::cleanup(){
    ros::shutdown();
}

void Hub::publishAll(){

    pose_pub.publish(this->pose);

    // Bounds
    std_msgs::String xb;
    stringstream xbss;
    xbss << "XB " << this->x_lowerbound << " " << this->x_upperbound;
    xb.data = xbss.c_str();
    xbounds_pub.publish(xb);

    std_msgs::String yb;
    stringstream ybss;
    ybss << "XB " << this->y_lowerbound << " " << this->y_upperbound;
    yb.data = ybss.c_str();
    ybounds_pub.publish(yb);

    std_msgs::String zb;
    stringstream zbss;
    zbss << "XB " << this->z_lowerbound << " " << this->z_upperbound;
    zb.data = zbss.c_str();
    zbounds_pub.publish(zb);

    // Velocities
    linvel_pub.publish(linear_vels);
    maxvel_pub.publish(max_vels);

    // Modes
    std_msgs::String mode_msg;
    stringstream mode_ss;
    mode_ss << "A " << this->actualMode << " D " << this->desiredMode;
    mode_msg.data = mode_ss.c_str();
    mode_pub.publish(mode_msg);

    // States
    std_msgs::String state_msg;
    stringstream state_ss;
    state_ss << "A " << this->actualState << " D " << this->desiredState;
    state_msg.data = state_ss.c_str();
    state_pub.publish(state_msg);

}

void Hub::desired_state_callback(const std_msgs::String::ConstPtr &msg){
    stringstream ss(msg->data);
    string code;
    ss >> code;

    string item;
    ss >> item;

    if(code == "auton"){
        desiredState = item;
    }
    else if(code == "mode"){
        desiredMode = item;
    }
}

void Hub::pos_bounds_callback(const std_msgs::String::ConstPtr &msg){
    stringstream ss(msg->data)
    string code;
    ss >> code;

    int upper, lower;
    ss >> lower >> upper;

    switch(code){
        case "XB":
            x_lowerbound = lower;
            x_upperbound = upper;
            break;
        case "YB":
            y_lowerbound = lower;
            y_upperbound = upper;
            break;
        case "ZB":
            z_lowerbound = lower;
            z_upperbound = upper;
            break;
    }
}

void Hub::vel_lim_callback(const geometry_msgs::Vector3::ConstPtr &msg){
    max_vels.x = msg->data.x;
    max_vels.y = msg->data.y;
    max_vels.z = msg->data.z;
}

void Hub::pose_callback(const geometry_msgs::Pose::ConstPtr &msg){
    // Update pose
    pose.position.x = msg->pose.position.x;
    pose.position.y = msg->pose.position.y;
    pose.position.z = msg->pose.position.z;
    pose.orientation.x = msg->pose.orientation.x;
    pose.orientation.y = msg->pose.orientation.y;
    pose.orientation.z = msg->pose.orientation.z;
    pose.orientation.w = msg->pose.orientation.w;
}