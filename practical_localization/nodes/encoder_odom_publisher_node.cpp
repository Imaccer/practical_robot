//  Copyright 2024 <Ian McNally>

#include "practical_localization/encoder_odom_publisher.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "encoder_odom_publisher_node");
  ros::NodeHandle node;

  EncoderOdomPublisher encoderOdomPublisher(node);
  encoderOdomPublisher.run();

  return 0;
}
