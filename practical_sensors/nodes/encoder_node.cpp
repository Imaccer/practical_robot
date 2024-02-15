#include "practical_sensors/encoder.h"
#include <pigpiod_if2.h>
#include <ros/ros.h>
#include <iostream>

namespace PinAssignments {
const int leftEncoder = 22;
const int rightEncoder = 23;
const int leftReverse = 13;
const int rightReverse = 20;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "encoder_publisher");
  ros::NodeHandle nh;

  int pi = pigpio_start(nullptr, nullptr);

  if (pi < 0) {
    std::cout << "Failed to initialize pigpiod: run sudo pigpiod" << std::endl;
    return 1;
  }

  Encoder leftEncoder(nh, pi, PinAssignments::leftEncoder,
                      PinAssignments::leftReverse, "leftWheel");
  Encoder rightEncoder(nh, pi, PinAssignments::rightEncoder,
                       PinAssignments::rightReverse, "rightWheel");

  leftEncoder.setupCallback();
  rightEncoder.setupCallback();

  ros::Rate loop_rate(30);

  while (ros::ok()) {
    if (leftEncoder.isInitialized()) {
      leftEncoder.publishCount();
    } else {
      std::cerr << "Error: leftEncoder pointer is null!" << std::endl;
    }
    if (rightEncoder.isInitialized()) {
      rightEncoder.publishCount();
    } else {
      std::cerr << "Error: rightEncoder pointer is null!" << std::endl;
    }
    ros::spinOnce();
    loop_rate.sleep();
    std::cout << "Loop iteration completed successfully" << std::endl;
  }

  pigpio_stop(pi);

  return 0;
}
