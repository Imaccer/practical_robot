//  Copyright 2024 <Ian McNally>

#ifndef PRACTICAL_LOCALIZATION_INCLUDE_PRACTICAL_LOCALIZATION_ENCODER_ODOM_PUBLISHER_H_
#define PRACTICAL_LOCALIZATION_INCLUDE_PRACTICAL_LOCALIZATION_ENCODER_ODOM_PUBLISHER_H_

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

class EncoderOdomPublisher {
 public:
  EncoderOdomPublisher(const ros::NodeHandle& nh);

  void run();

 private:
  void initializeFixedFields();
  void subscribeToTopics();
  void advertisePublishers();
  void setInitialPose(const geometry_msgs::PoseStamped& rvizClick);
  void calcLeft(const std_msgs::Int16& lCount);
  void calcRight(const std_msgs::Int16& rCount);
  void updateOdom();
  void publishQuat();

  ros::NodeHandle nh_;
  ros::Subscriber subRightCounts_;
  ros::Subscriber subLeftCounts_;
  ros::Subscriber subInitialPose_;
  ros::Publisher odomPub_;
  ros::Publisher pubQuat_;

  nav_msgs::Odometry newOdom_;
  nav_msgs::Odometry oldOdom_;

  const double initialX_;
  const double initialY_;
  const double initialTheta_;
  const double PI_;
  const double WHEEL_BASE_;
  const double TICKS_PER_M_;
  double leftDistance_;
  double rightDistance_;

  bool initialPoseReceived_;
};

#endif  // PRACTICAL_LOCALIZATION_INCLUDE_PRACTICAL_LOCALIZATION_ENCODER_ODOM_PUBLISHER_H_
