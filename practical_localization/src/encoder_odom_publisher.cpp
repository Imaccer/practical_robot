//  Copyright 2024 <Ian McNally>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/Int16.h"

#include "practical_localization/encoder_odom_publisher.h"

EncoderOdomPublisher::EncoderOdomPublisher(const ros::NodeHandle& nh)
    : nh_(nh),
      initialPoseReceived_(false),
      initialX_(0.0),
      initialY_(0.0),
      initialTheta_(0.00000000001),
      PI_(3.141592),
      WHEEL_BASE_(0.224),
      TICKS_PER_M_(2270),
      leftDistance_(0),
      rightDistance_(0) {
  initializeFixedFields();
  subscribeToTopics();
  advertisePublishers();
}

void EncoderOdomPublisher::run() {
  ros::Rate loop_rate(30);
  while (ros::ok()) {
    ros::spinOnce();
    if (initialPoseReceived_) {
      updateOdom();
      publishQuat();
    }
    loop_rate.sleep();
  }
}

void EncoderOdomPublisher::initializeFixedFields() {
  newOdom_.header.frame_id = "odom";
  newOdom_.pose.pose.position.z = 0;
  newOdom_.pose.pose.orientation.x = 0;
  newOdom_.pose.pose.orientation.y = 0;
  newOdom_.twist.twist.linear.x = 0;
  newOdom_.twist.twist.linear.y = 0;
  newOdom_.twist.twist.linear.z = 0;
  newOdom_.twist.twist.angular.x = 0;
  newOdom_.twist.twist.angular.y = 0;
  newOdom_.twist.twist.angular.z = 0;

  oldOdom_.pose.pose.position.x = initialX_;
  oldOdom_.pose.pose.position.y = initialY_;
  oldOdom_.pose.pose.orientation.z = initialTheta_;
  oldOdom_.pose.pose.orientation.w = 1;
}

void EncoderOdomPublisher::subscribeToTopics() {
  subRightCounts_ =
      nh_.subscribe("rightWheel", 100, &EncoderOdomPublisher::calcRight, this,
                    ros::TransportHints().tcpNoDelay());
  subLeftCounts_ =
      nh_.subscribe("leftWheel", 100, &EncoderOdomPublisher::calcLeft, this,
                    ros::TransportHints().tcpNoDelay());
  subInitialPose_ = nh_.subscribe("initial_2d", 1,
                                  &EncoderOdomPublisher::setInitialPose, this);
}

void EncoderOdomPublisher::advertisePublishers() {
  odomPub_ = nh_.advertise<nav_msgs::Odometry>("encoder/odom", 100);
  pubQuat_ = nh_.advertise<nav_msgs::Odometry>("encoder/odom_quat", 100);
}

void EncoderOdomPublisher::setInitialPose(
    const geometry_msgs::PoseStamped& rvizClick) {
  oldOdom_.pose.pose.position.x = rvizClick.pose.position.x;
  oldOdom_.pose.pose.position.y = rvizClick.pose.position.y;
  oldOdom_.pose.pose.position.z = rvizClick.pose.position.z;

  tf2::Quaternion q(rvizClick.pose.orientation.x, rvizClick.pose.orientation.y,
                    rvizClick.pose.orientation.z, rvizClick.pose.orientation.w);
  q.normalize();

  oldOdom_.pose.pose.orientation.x = q.x();
  oldOdom_.pose.pose.orientation.y = q.y();
  oldOdom_.pose.pose.orientation.z = q.z();
  oldOdom_.pose.pose.orientation.w = q.w();

  initialPoseReceived_ = true;
}

void EncoderOdomPublisher::calcLeft(const std_msgs::Int16& lCount) {
  static int lastCountL = 0;

  if (lCount.data != 0 && lastCountL != 0) {
    int leftTicks = (lCount.data - lastCountL);

    if (leftTicks > 10000) {
      leftTicks = 0 - (65535 - leftTicks);
    } else if (leftTicks < -10000) {
      leftTicks = 65535 - leftTicks;
    }
    leftDistance_ = leftTicks / TICKS_PER_M_;
  }
  lastCountL = lCount.data;
}

void EncoderOdomPublisher::calcRight(const std_msgs::Int16& rCount) {
  static int lastCountR = 0;

  if (rCount.data != 0 && lastCountR != 0) {
    int rightTicks = rCount.data - lastCountR;

    if (rightTicks > 10000) {
      rightTicks = (0 - (65535 - rightTicks)) / TICKS_PER_M_;
    } else if (rightTicks < -10000) {
      rightTicks = 65535 - rightTicks;
    }
    rightDistance_ = rightTicks / TICKS_PER_M_;
  }
  lastCountR = rCount.data;
}

void EncoderOdomPublisher::updateOdom() {
  double cycleDistance = (rightDistance_ + leftDistance_) / 2;
  double cycleAngle = asin((rightDistance_ - leftDistance_) / WHEEL_BASE_);

  double avgAngle = cycleAngle / 2 + oldOdom_.pose.pose.orientation.z;
  if (avgAngle > PI_) {
    avgAngle -= 2 * PI_;
  } else if (avgAngle < -PI_) {
    avgAngle += 2 * PI_;
  }

  newOdom_.pose.pose.position.x =
      oldOdom_.pose.pose.position.x + cos(avgAngle) * cycleDistance;
  newOdom_.pose.pose.position.y =
      oldOdom_.pose.pose.position.y + sin(avgAngle) * cycleDistance;
  newOdom_.pose.pose.orientation.z =
      cycleAngle + oldOdom_.pose.pose.orientation.z;

  if (std::isnan(newOdom_.pose.pose.position.x) ||
      std::isnan(newOdom_.pose.pose.position.y) ||
      std::isnan(newOdom_.pose.pose.position.z)) {
    newOdom_.pose.pose.position.x = oldOdom_.pose.pose.position.x;
    newOdom_.pose.pose.position.y = oldOdom_.pose.pose.position.y;
    newOdom_.pose.pose.orientation.z = oldOdom_.pose.pose.orientation.z;
  }

  if (newOdom_.pose.pose.orientation.z > PI_) {
    newOdom_.pose.pose.orientation.z -= 2 * PI_;
  } else if (newOdom_.pose.pose.orientation.z < -PI_) {
    newOdom_.pose.pose.orientation.z += 2 * PI_;
  }

  newOdom_.header.stamp = ros::Time::now();
  newOdom_.twist.twist.linear.x =
      cycleDistance /
      (newOdom_.header.stamp.toSec() - oldOdom_.header.stamp.toSec());
  newOdom_.twist.twist.angular.z = cycleAngle / (newOdom_.header.stamp.toSec() -
                                                 oldOdom_.header.stamp.toSec());

  oldOdom_.pose.pose.position.x = newOdom_.pose.pose.position.x;
  oldOdom_.pose.pose.position.y = newOdom_.pose.pose.position.y;
  oldOdom_.pose.pose.orientation.z = newOdom_.pose.pose.orientation.z;
  oldOdom_.header.stamp = newOdom_.header.stamp;

  odomPub_.publish(newOdom_);
}

void EncoderOdomPublisher::publishQuat() {
  tf2::Quaternion q;
  q.setRPY(0, 0, newOdom_.pose.pose.orientation.z);
  q.normalize();

  nav_msgs::Odometry quatOdom;
  quatOdom.header.stamp = newOdom_.header.stamp;
  quatOdom.header.frame_id = "odom";
  quatOdom.child_frame_id = "base_link";
  quatOdom.pose.pose.position.x = newOdom_.pose.pose.position.x;
  quatOdom.pose.pose.position.y = newOdom_.pose.pose.position.y;
  quatOdom.pose.pose.position.z = newOdom_.pose.pose.position.z;
  quatOdom.pose.pose.orientation.x = q.x();
  quatOdom.pose.pose.orientation.y = q.y();
  quatOdom.pose.pose.orientation.z = q.z();
  quatOdom.pose.pose.orientation.w = q.w();
  quatOdom.twist.twist.linear.x = newOdom_.twist.twist.linear.x;
  quatOdom.twist.twist.linear.y = newOdom_.twist.twist.linear.y;
  quatOdom.twist.twist.linear.z = newOdom_.twist.twist.linear.z;
  quatOdom.twist.twist.angular.x = newOdom_.twist.twist.angular.x;
  quatOdom.twist.twist.angular.y = newOdom_.twist.twist.angular.y;
  quatOdom.twist.twist.angular.z = newOdom_.twist.twist.angular.z;

  for (int i = 0; i < 36; i++) {
    if (i == 0 || i == 7 || i == 14) {
      quatOdom.pose.covariance[i] = 0.01;
    } else if (i == 21 || i == 28 || i == 35) {
      quatOdom.pose.covariance[i] += 0.165;
    } else {
      quatOdom.pose.covariance[i] = 0;
    }
  }

  pubQuat_.publish(quatOdom);
}
