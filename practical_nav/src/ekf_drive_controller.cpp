/*
*simple_drive_controller.cpp is an example ROS node accompanying the book
*Practical Robotics in C++.
*
*This node subscribes to the robot_pose_ekf/odom_combine topic as well as waypoint_2d. When a new
*waypoint is recieved, this node calculates a straight course to the waypoint without obstacle avoidance
*and publishes cmd_vel msgs to first turn to the waypoint, then go forward. If the angle drifts
*outside of "close enough" while enroute, the robot will stop and correct is heading before continuing.
*Written to be a readable, but functional, example for all levels following along with the book.
*
*
*Author: Lloyd Brombach (lbrombach2@gmail.com)
*11/7/2019
*/

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <nav_msgs/Odometry.h>
#include <cstdlib>
#include <math.h>
#include <iostream>

using namespace std;

ros::Publisher pubVelocity;
nav_msgs::Odometry odom;
//geometry_msgs::PoseWithCovarianceStamped odom;
geometry_msgs::Twist cmdVel;
geometry_msgs::PoseStamped desired;
const double PI = 3.141592;
const double Ka =0.15;//.05;// .35;
const double Klv = .86;
const double initialX = 5.0;
const double initialY = 5.0;
const double angularTolerance = .15;//.1;
const double distanceTolerance = .05;//0.05
const double finalHeadingTolerance = 0.1;
const double MAX_LINEAR_VEL = 1;
bool waypointActive = false;


tf2::Quaternion createQuaternionFromPose(const geometry_msgs::Pose &pose){

  tf2::Quaternion quaternion;

  quaternion.setX(pose.orientation.x);
  quaternion.setY(pose.orientation.y);
  quaternion.setZ(pose.orientation.z);
  quaternion.setW(pose.orientation.w);
                
  return quaternion;
}

/*
double yawFromQuaternion(const tf2::Quaternion& quaternion) {
    // Extract yaw angle from the quaternion
    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    return yaw;
}
*/

//void update_pose(const nav_msgs::Odometry &currentOdom)
void update_pose(const geometry_msgs::PoseWithCovarianceStamped &currentOdom)
{
    //odom.header.frame_id = currentOdom.header.frame_id;

    odom.pose.pose.position.x = currentOdom.pose.pose.position.x;
    odom.pose.pose.position.y = currentOdom.pose.pose.position.y;
    odom.pose.pose.position.z = currentOdom.pose.pose.position.z;
    odom.pose.pose.orientation.x = currentOdom.pose.pose.orientation.x;
    odom.pose.pose.orientation.y = currentOdom.pose.pose.orientation.y;
    odom.pose.pose.orientation.z = currentOdom.pose.pose.orientation.z;
    odom.pose.pose.orientation.w = currentOdom.pose.pose.orientation.w;

    cout << "currentOdom msg header: " << currentOdom.header.frame_id << endl
         << "odom msg header: " << odom.header.frame_id << endl;

    //cout << "currentOdom msg child frame: " << currentOdom.child_frame_id << endl
    //     << "odom msg child frame: " << odom.child_frame_id << endl;
}

void update_goal(const geometry_msgs::PoseStamped &desiredPose)
{
cout<<"got new goal!"<<endl;
    desired.pose.position.x = desiredPose.pose.position.x;
    desired.pose.position.y = desiredPose.pose.position.y;
    desired.pose.position.z = desiredPose.pose.position.z;
    desired.pose.orientation.x = desiredPose.pose.orientation.x;
    desired.pose.orientation.y = desiredPose.pose.orientation.y;
    desired.pose.orientation.z = desiredPose.pose.orientation.z;
    desired.pose.orientation.w = desiredPose.pose.orientation.w;
    waypointActive = true;
    cout<<"waypoint active set true"<<endl;
}

double getDistanceError()
{
    double deltaX = desired.pose.position.x - odom.pose.pose.position.x;
    double deltaY = desired.pose.position.y - odom.pose.pose.position.y;
    cout<<"Distance error = "<<sqrt(pow(deltaX, 2) + pow(deltaY, 2))<<endl;
    return sqrt(pow(deltaX, 2) + pow(deltaY, 2));
}

double getAngularError()
{
    double deltaX = desired.pose.position.x - odom.pose.pose.position.x;
    double deltaY = desired.pose.position.y - odom.pose.pose.position.y;
    double thetaBearing = atan2(deltaY, deltaX);

    tf2::Quaternion quat = createQuaternionFromPose(odom.pose.pose);
    quat.normalize();
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll,pitch,yaw);   
    double currentYaw = yaw;
    //double angularError = thetaBearing - odom.pose.pose.orientation.z;
    double angularError = thetaBearing - currentYaw; 
    angularError = (angularError > PI)  ? angularError - (2*PI) : angularError;
    angularError = (angularError < -PI) ? angularError + (2*PI) : angularError;
cout<<"angular error within function call  = " <<angularError<<endl;
    return angularError;
}


void set_velocity()
{
    cmdVel.linear.x = 0;
    cmdVel.linear.y = 0;
    cmdVel.linear.z = 0;
    cmdVel.angular.x = 0;
    cmdVel.angular.y = 0;
    cmdVel.angular.z = 0;

    static bool angle_met = true;
    static bool location_met = true;

    //double final_desired_heading_error = desired.pose.orientation.z - odom.pose.pose.orientation.z;
    tf2::Quaternion desired_quat = createQuaternionFromPose(desired.pose);
    desired_quat.normalize();

    tf2::Quaternion current_quat = createQuaternionFromPose(odom.pose.pose);
    current_quat.normalize();
    
    double final_desired_heading_error = tf2::angleShortestPath(desired_quat, current_quat);

    cout << "Final desired heading error = " << final_desired_heading_error << endl;

    if(abs(getDistanceError()) >=distanceTolerance)//0.05
        {
        location_met = false;
        }
    else if (abs(getDistanceError()) < distanceTolerance)//.03)
        {
        location_met = true;
        }

    double angularError = (location_met == false) ? getAngularError() : final_desired_heading_error;

    cout << "Checking the ang error after conditional: " << angularError << endl;

    if (abs(angularError) >= angularTolerance)//.15)
        {
         angle_met = false;
        }
    else if (abs(angularError) < angularTolerance)
        {
         angle_met = true;
        }

    cout << "checking location_met: "<< location_met << endl;
    cout << "checking angle_met: " << angle_met << endl;

    if (waypointActive == true && angle_met == false)
        {
         cmdVel.angular.z = Ka * angularError;
         cmdVel.linear.x = 0;
        }
    else if (waypointActive == true && abs(getDistanceError()) >= distanceTolerance && location_met == false)
        {
         cmdVel.linear.x = Klv * getDistanceError();
         cmdVel.angular.z = 0;
        }
    else if (waypointActive == true&& abs(getDistanceError()-distanceTolerance)<0.02 && abs(getDistanceError()) >= distanceTolerance && location_met == false)
        {
         cmdVel.linear.x = 2*Klv * getDistanceError();
         cmdVel.angular.z = 0;
        }
    else 
    {
        cout << "********I'm HERE, now set final desired heading! **********"<<endl;
        location_met = true;
    }

    if (location_met==true && (abs(final_desired_heading_error) <= finalHeadingTolerance))
        {
         cout<<"Target Achieved"<<endl;
         angle_met = true;
         waypointActive = false;
        }
    else if (location_met==true && waypointActive==true && abs(getDistanceError())<distanceTolerance && abs(final_desired_heading_error)>finalHeadingTolerance)   
    {
         cmdVel.angular.z = Ka * final_desired_heading_error;
         cmdVel.linear.x = 0;
    }
    else if (location_met==true && abs(final_desired_heading_error-finalHeadingTolerance)<0.2 && waypointActive==true && abs(getDistanceError())<distanceTolerance && abs(final_desired_heading_error)>finalHeadingTolerance)   
    {
         cmdVel.angular.z = 1.5*Ka * final_desired_heading_error;
         cmdVel.linear.x = 0;
    }

    pubVelocity.publish(cmdVel);
    
    cout << "Location met: " << location_met << endl;
    cout << "Angle met: " << angle_met << endl;

    cout << "Desired pose orientation z: " << desired.pose.orientation.z << endl
         << "Current pose orientation z: " << odom.pose.pose.orientation.z << endl;

}


int main(int argc, char **argv)
{
    desired.pose.position.x = -1;
    ros::init(argc, argv, "ekf_drive_controller");
    ros::NodeHandle node;

    //Subscribe to topics
    //ros::Subscriber subCurrentPose = node.subscribe("encoder/odom", 10, update_pose, ros::TransportHints().tcpNoDelay());
    ros::Subscriber subCurrentPose = node.subscribe("robot_pose_ekf/odom_combined", 10, update_pose, ros::TransportHints().tcpNoDelay());
    ros::Subscriber subDesiredPose = node.subscribe("waypoint_2d", 1, update_goal, ros::TransportHints().tcpNoDelay());
    pubVelocity = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        if(desired.pose.position.x != -1)
        {
        set_velocity();
        }
        cout << "goal = " << desired.pose.position.x << ", " << desired.pose.position.y << endl
             << "current x,y = " << odom.pose.pose.position.x << ", " << odom.pose.pose.position.y << endl
             << "  Distance error = " << getDistanceError() << endl
             << "  Angular error = " << getAngularError() << endl;
        cout << "cmd_vel = " << cmdVel.linear.x <<" ,  "<<cmdVel.angular.z<< endl;
        loop_rate.sleep();
    }

    return 0;
}
