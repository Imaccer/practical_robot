#include <gtest/gtest.h>
#include <ros/ros.h>
#include "practical_sensors/encoder.h"
#include <pigpiod_if2.h>
#include <iostream>

namespace PinAssignments {
  const int leftEncoder = 22;
  const int rightEncoder = 23;
  const int leftReverse = 13;
  const int rightReverse = 20;
}

TEST(RosTest, testexample) {
  ros::NodeHandle nh;
  ros::Rate loop_rate(100);
  int pi = pigpio_start(nullptr, nullptr);
  if (pi<0) {
   std::cout << "Failed to initialize pigpiod: run sudo pigpiod" << std::endl;
  }
  Encoder leftEncoder(nh, pi, PinAssignments::leftEncoder, PinAssignments::leftReverse, "leftWheel");
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_encoder");

  return RUN_ALL_TESTS();
}
