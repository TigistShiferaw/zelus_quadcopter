#include "zqcopter/zel_to_hub.h"

int init(ros::NodeHandle *nodehandle){
    nh = *nodehandle;
    cycle_rate = 10;

    // Publishers
    velocityLimit_pub = nh.advertise<std_msgs::String>("vel_limit_zel", 100);
    positionBounds_pub = nh.advertise<std_msgs::String>("pos_bounds_zel", 100);
    state_pub = nh.advertise<std_msgs::String>("state_zel", 100); 


    // Spin the node
    run();
}

void run(){
    // Sets refesh rates
    ros::Rate loop_rate(cycle_rate);
    cycles = 0;

    while(ros::ok()){
        // End of cycle
        cycle++;
        loop_rate.sleep();
        ros::spinOnce();
    }

    ROS_INFO("Cleaning up zel_to_hub layer...");
}

// Publishers
void publishXDotLimit(int xdot){
    geometry_msgs::Vector3 msg;
    
    msg.x = xdot;
    msg.y = -1;
    msg.z = -1;

    velocityLimit_pub.publish(msg);
}

void publishYDotLimit(int ydot){
    geometry_msgs::Vector3 msg;
    
    msg.x = -1;
    msg.y = ydot;
    msg.z = -1;

    velocityLimit_pub.publish(msg);
}

void publishZDotLimit(int zdot){
    geometry_msgs::Vector3 msg;
    
    msg.x = -1;
    msg.y = -1;
    msg.z = zdot;

    velocityLimit_pub.publish(msg);
}

void publishVelocityLimits(int xdot, int ydot, int zdot){
    geometry_msgs::Vector3 msg;
    
    msg.x = xdot;
    msg.y = ydot;
    msg.z = zdot;

    velocityLimit_pub.publish(msg);
}

void publishXBounds(int x_lower, int x_upper){
    std_msgs::String msg;

    std::stringstream ss;
    ss << "XB " << x_lower << " " << x_upper;
    msg.data = ss.str();

    positionBounds_pub.publish(msg);
}

void publishYBounds(int y_lower, int y_upper){
    std_msgs::String msg;

    std::stringstream ss;
    ss << "YB " << y_lower << " " << y_upper;
    msg.data = ss.str();

    positionBounds_pub.publish(msg);
}

void publishZBounds(int z_lower, int z_upper){
    std_msgs::String msg;

    std::stringstream ss;
    ss << "ZB " << z_lower << " " << z_upper;
    msg.data = ss.str();

    positionBounds_pub.publish(msg);
}

void publishDesiredMode(char mode){
    std_msgs::String;

    std::stringstream ss;
    ss << "mode " << state;
    msg.data = ss.str();

    state_pub.publish(msg);
}

void publishAutonDesiredState(char state){
    std_msgs::String;

    std::stringstream ss;
    ss << "auton " << state;
    msg.data = ss.str();

    state_pub.publish(msg);
}

void getPose(double *poseContainer){
    try {
        boost::shared_ptr<geometry_msgs::Pose const> sharedPose;
        sharedPose = ros::topic::waitForMessage<geometry_msgs::Pose>("pose_hub");
        if(sharedPose != NULL){
            poseContainer[0] = sharedPose->position->z;
            poseContainer[1] = sharedPose->position->y;
            poseContainer[2] = sharedPose->position->z;
            poseContainer[3] = sharedPose->orientation->x;
            poseContainer[4] = sharedPose->orientation->y;
            poseContainer[5] = sharedPose->orientation->z;
            poseContainer[6] = sharedPose->orientation->w;
        }
        else
            throw wmex;
    }
    catch(exception &e){
        cout << e.what() << '\n';
    }
}

void getLinearVelocities(double *linearVels){
    try {
        oost::shared_ptr<geometry_msgs::Vector3 const> sharedVels;
        sharedVels = ros::topic::waitForMessage<geometry_msgs::Vector3>("linvel_hub");
        if(sharedVels != NULL){
            linearVels[0] = sharedVels.x;
            linearVels[1] = sharedVels.y;
            linearVels[2] = sharedVels.z;
        }
        else
            throw wmex;
    }
    catch(exception &e){
        cout << e.what() << '\n';
    }
}

char getActualMode(){
    try {
        boost::shared_ptr<std_msgs::String const> sharedMode;
        sharedMode = ros::topic::waitForMessage<std_msgs::String>("mode_hub");
        if(sharedMode != NULL){
            stringstream ss(sharedMode->data);
            string dat;
            ss >> dat;
            ss >> dat;
            return (ss.str()).at(0);
        }

        throw wmex;
    }
    catch(exception &e){
        cout << e.what() << '\n';
    }
    
    return '\0';
}

char getActualState(){
    try {
        boost::shared_ptr<std_msgs::String const> sharedState;
        sharedState = ros::topic::waitForMessage<std_msgs::String>("state_hub");
        if(sharedState != NULL){
            stringstream ss(sharedState->data);
            string dat;
            ss >> dat;
            ss >> dat;
            return (ss.str()).at(0);
        }
        
        throw wmex;
    }
    catch(exception &e){
        cout << e.what() << '\n';
    }
}





