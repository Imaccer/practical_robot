#include "practical_nav/differential_drive_robot.h"
#include <iostream>

// Definition and initialization of static constants
const int DifferentialDriveRobot::PWM_INCREMENT_ = 1;
const double DifferentialDriveRobot::TICKS_PER_WHEEL_REV_ = 254 * 2;

DifferentialDriveRobot::DifferentialDriveRobot()

    : node_(),

      subForRightCounts_(node_.subscribe(
          "rightWheel", 1000, &DifferentialDriveRobot::calcRightVel, this)),

      subForLeftCounts_(node_.subscribe(
          "leftWheel", 1000, &DifferentialDriveRobot::calcLeftVel, this)),

      subForVelocity_(node_.subscribe(
          "cmd_vel", 1, &DifferentialDriveRobot::setSpeeds, this)),

      loopRate_(100),

      pi_(pigpioSetup()),

      lastCmdMsgRcvd_(0) {
  // Other member variable initializations can be done here if needed.
}

DifferentialDriveRobot::~DifferentialDriveRobot() {
  gpio_write(pi_, MOTOR_L_FWD_, 1);
  gpio_write(pi_, MOTOR_L_REV_, 1);
  gpio_write(pi_, MOTOR_R_FWD_, 1);
  gpio_write(pi_, MOTOR_R_REV_, 1);
}

int DifferentialDriveRobot::pigpioSetup() {
  // ... (implementation)
}


void DifferentialDriveRobot::calcLeftVel(const std_msgs::Int16& lCount) {
  // ... (implementation)
}

void DifferentialDriveRobot::calcRightVel(const std_msgs::Int16& rCount) {
  // ... (implementation)
}

void DifferentialDriveRobot::setSpeeds(const geometry_msgs::Twist& cmdVel) {
  // ... (implementation)
}

void DifferentialDriveRobot::setPinValues() {
  // ... (implementation)
}

void DifferentialDriveRobot::run() {
  while (ros::ok()) {
    ros::spinOnce();

    if (ros::Time::now().toSec() - lastCmdMsgRcvd_ > 1) {
      std::cout
          << "NOT RECEIVING CMD_VEL - STOPPING MOTORS  --  time since last = "
          << ros::Time::now().toSec() - lastCmdMsgRcvd_ << std::endl;

      leftPwmReq_ = 0;

      rightPwmReq_ = 0;
    }

    setPinValues();

    loopRate_.sleep();
  }
}
