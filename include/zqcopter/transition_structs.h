#ifndef TRANSITION_STRUCTS_H_
#define TRANSITION_STRUCTS_H_

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

struct Vec3 {
    int xdot;
    int ydot;
    int zdot;
    // Ignore angular velocities for now
};

#endif