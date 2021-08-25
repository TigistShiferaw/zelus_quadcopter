#ifndef HUB_H_
#define HUB_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
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
        ros::Subscriber pose_sub;
        ros::Subscriber vel_lim_sub;
        ros::Subscriber pos_bounds_sub;
        ros::Subscriber state_sub;

        void pose_callback(const geometry_msgs::Pose::ConstPtr &msg);
        void vel_lim_callback(const geometry_msgs::Vector3::ConstPtr &msg);
        void pos_bounds_callback(const std_msgs::String::ConstPtr &msg);
        void state_zel_callback(const std_msgs::String::ConstPtr &msg);


        // Publishers
        ros::Publisher pose_pub;
        ros::Publisher xbounds_pub;
        ros::Publisher ybounds_pub;
        ros::Publisher zbounds_pub;
        ros::Publisher linvel_pub;
        ros::Publisher mode_pub;
        ros::Publisher state_pub;
        ros::Publisher maxvel_pub;

        // Publisher functions
        void publishAll();


        // --------------- State Variables --------------
        geometry_msgs::Pose pose;
        geometry_msgs::Vector3 linear_vels;
        geometry_msgs::Vector3 max_vels;
        char actualMode; // 'L' -> LOITER; 'M' -> MANUAL; 'A' -> AUTONOMOUS
        char desiredMode;
        char desiredState; // 'S' -> STANDBY; etc... TODO: Check lab computer and fill in
        char actualState;
        int x_upperbound;
        int x_lowerbound;
        int y_upperbound;
        int y_lowerbound;
        int z_upperbound;
        int z_lowerbound;
};


#endif