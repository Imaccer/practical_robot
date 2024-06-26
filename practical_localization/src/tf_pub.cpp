/*
*tf_pub.cpp is a simple ROS node that subscribes to an odometry message
*and published an odom to base_link transform based on that data.
*Unlike some of node and messages we use for examples in Practical Robotics in
*C++,
*tf_pub uses quaternions
*
*Author: Lloyd Brombach (lbrombach2@gmail.com)
*11/7/2019
*/

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h> // Include the correct message type

// callback function that broadcasts odom_combined message as transform
//void handle_odom(const geometry_msgs::PoseWithCovarianceStamped &odom) {
using namespace std;

/*
// callback function that broadcasts odom message as transform
void handle_odom(const nav_msgs::Odometry &odom) {
  static tf::TransformBroadcaster br;
  tf::Transform odom_base_tf;
  odom_base_tf.setOrigin(
      tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, 0.0));
  tf::Quaternion tf_q(
      odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
      odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);

  odom_base_tf.setRotation(tf_q);

//ROS_INFO_STREAM("Inside callback");  
    br.sendTransform(tf::StampedTransform(odom_base_tf, odom.header.stamp,
                                          "odom", "base_link"));
 //   br.sendTransform(tf::StampedTransform(odom_base_tf, odom.header.stamp,  "odom", "base_footprint"));
  
}
*/


// callback function that broadcasts odom_combined message as transform
void handle_odom(const geometry_msgs::PoseWithCovarianceStamped &odom_combined) {
  static tf::TransformBroadcaster br;
  tf::Transform odom_base_tf;
  odom_base_tf.setOrigin(
      tf::Vector3(odom_combined.pose.pose.position.x, odom_combined.pose.pose.position.y, 0.0));
  tf::Quaternion tf_q(
      odom_combined.pose.pose.orientation.x, odom_combined.pose.pose.orientation.y,
      odom_combined.pose.pose.orientation.z, odom_combined.pose.pose.orientation.w);

  odom_base_tf.setRotation(tf_q);

  //br.sendTransform(tf::StampedTransform(odom_base_tf, odom_combined.header.stamp,
   //                                      "odom", "base_link"));
  br.sendTransform(tf::StampedTransform(odom_base_tf, odom_combined.header.stamp,
                                         "odom", "base_footprint"));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_pub");
  ros::NodeHandle node;

  // ros::Subscriber subOdom = node.subscribe("encoder/odom_quat", 10,
  // handle_odom);
 // ros::Subscriber subOdom =
  //    node.subscribe("encoder/odom_quat", 10, handle_odom);
   ros::Subscriber subOdom = node.subscribe("robot_pose_ekf/odom_combined",
   10, handle_odom);

  ros::Rate loop_rate(30);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
