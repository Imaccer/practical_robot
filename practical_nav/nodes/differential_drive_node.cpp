#include <ros/ros.h>
#include "practical_nav/differential_drive_robot.h" 


int main(int argc, char** argv) { 

    ros::init(argc, argv, "differential_drive_node"); 
    ros::NodeHandle nh;
    DifferentialDriveRobot diff_drive_robot(nh); 

    diff_drive_robot.run(); 

    return 0; 

} 
