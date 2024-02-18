//  Copyright 2024 <Ian McNally>

#ifndef PRACTICAL_NAV_INCLUDE_PRACTICAL_NAV_ENCODER_READER_H_
#define PRACTICAL_NAV_INCLUDE_PRACTICAL_NAV_ENCODER_READER_H_

#include "ros/ros.h"
#include "std_msgs/Int16.h"

class EncoderReader {
 public:
  EncoderReader();

  void calculateLeftVelocity(const std_msgs::Int16& leftCount);
  void calculateRightVelocity(const std_msgs::Int16& rightCount);

  double getLeftVelocity() const;
  double getRightVelocity() const;
 private:
  const int ENCODER_RANGE_;
  const double TICKS_PER_M_;

  double leftVelocity_;
  double rightVelocity_;
};

#endif  // PRACTICAL_NAV_INCLUDE_PRACTICAL_NAV_ENCODER_READER_H_
