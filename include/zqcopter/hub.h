#ifndef HUB_H_
#define HUB_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

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

class Hub {
    public:
        // member functions
        Hub(){}

    private:

        // ------------ Member Functions ---------------

        // Starts spinning of node
        void run(); 

        // creates all publishers and subscribes as well as calls run
        // Returns 0 if creation of Hub is good and -1 if error occured
        int init(ros::NodeHandle *nodehandle);

        // Cleans up all allocated memory or other processes
        void cleanup();


        // ---------------- ROS Variables ---------------
        ros::NodeHandle nh;
        int cycle_rate = 10;
        int cycles;

        // Subscribers and respective callbacks
        ros::Subscriber apose_sub;
        void apose_callback(const geometry_msgs::Pose::ConstPtr &msg);

        ros::Subscriber dpose_sub;
        void dpose_callback(const geometry_msgs::Pose::ConstPtr &msg);

        // Publishers

        // --------------- State Variables --------------
        Pose actual_pose;
        Pose desired_pose;
        Twist twist;
        char mode; // 'L' -> LOITER; 'M' -> MANUAL; 'A' -> AUTONOMOUS

        // --------------- State Constraints ------------
        // Examples given
        // m/s
        int max_x_vel;
        int max_y_vel;
        int max_z_vel;
};


#endif