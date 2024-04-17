// Copyright 2024 <Ian McNally>

#include "practical_nav/encoder_reader.h"
#include <ros/ros.h>
#include <std_msgs/Int16.h>

EncoderReader::EncoderReader()
    : ENCODER_RANGE_(65535),
      TICKS_PER_M_(2270),
      leftVelocity_(0),
      rightVelocity_(0) {}

void EncoderReader::calculateLeftVelocity(const std_msgs::Int16& leftCount) {
  static double lastTime = 0;
  static int lastCount = 0;
  int cycleDistance = 0;
  cycleDistance =
      (ENCODER_RANGE_ + leftCount.data - lastCount) % ENCODER_RANGE_;
  if (cycleDistance > 10000) {
    cycleDistance = 0 - (ENCODER_RANGE_ - cycleDistance);
  }
  leftVelocity_ =
      cycleDistance / TICKS_PER_M_ / (ros::Time::now().toSec() - lastTime);
  lastCount = leftCount.data;
  lastTime = ros::Time::now().toSec();
}

void EncoderReader::calculateRightVelocity(const std_msgs::Int16& rightCount) {
  static double lastTime = 0;
  static int lastCount = 0;
  int cycleDistance =
      (ENCODER_RANGE_ + rightCount.data - lastCount) % ENCODER_RANGE_;
  if (cycleDistance > 10000) {
    cycleDistance = 0 - (ENCODER_RANGE_ - cycleDistance);
  }
  rightVelocity_ =
      cycleDistance / TICKS_PER_M_ / (ros::Time::now().toSec() - lastTime);
  lastCount = rightCount.data;
  lastTime = ros::Time::now().toSec();
}

double EncoderReader::getLeftVelocity() const { return leftVelocity_; }

double EncoderReader::getRightVelocity() const { return rightVelocity_; }
