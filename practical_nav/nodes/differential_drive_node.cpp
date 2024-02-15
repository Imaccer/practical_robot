#include <ros/ros.h>
#include "simple_diff_drive.h" 


int main(int argc, char** argv) { 

    ros::init(argc, argv, "differential_drive_node"); 
 

    DifferentialDriveRobot diffDrive; 

    diffDrive.run(); 

    return 0; 

} 
