#include <stdio.h>
#include <caml/mlvalues.h>
#include "zqcopter/zel_to_hub.h"

CAMLprim value
publish_x_dot_limit(value xdot){
    // Call publishXDotLimit(int xdot)
}

CAMLprim value
publish_y_dot_limit(value ydot){
    // Call publishYDotLimit(int ydot)
}

CAMLprim value
publish_z_dot_limit(value zdot){
    // Call publishZDotLimit(int zdot)
}

CAMLprim value
publish_velocity_limits(value xdot, value ydot, value zdot){
    // Call publishVelocityLimits(int xdot, int ydot, int zdot)
}

CAMLprim value
publish_x_bounds(value x_lower, value x_upper){
    // Call publishXBounds(int x_lower, int x_upper)
}


CAMLprim value
publish_y_bounds(value y_lower, value y_upper){
    // Call publishYBounds(int y_lower, int y_upper)
}

CAMLprim value
publish_z_bounds(value z_lower, value z_upper){
    // Call publishZBounds(int z_lower, int z_upper)
}

CAMLprim value
publish_desired_mode(value mode){
    // Call publishDesiredMode(char mode)
}

CAMLprim value
publish_auton_desired_state(value state){
    // Call publishAutonDesiredState(char state)
}

CAMLprim value
get_pose(){
    // populates pose array with real pose
    double pose[7];
    getPose(pose);

    // Contains pose with 
    // Position: x, y, z
    // Orientation quaternion: x, y, z, w
    // Now to send back to zelus
}

CAMLprim value
get_linear_vels(){
    double linvel[3];
    getLinearVelocities(linvel);

    // send back to zelus
}

CAMLprim value
get_actual_mode(){
    char actual_mode = getActualMode();
}
