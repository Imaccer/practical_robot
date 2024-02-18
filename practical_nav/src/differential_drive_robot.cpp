//  Copyright 2024 <Ian McNally>

#include "practical_nav/differential_drive_robot.h"
#include <ros/ros.h>
#include <iostream>
#include "practical_nav/encoder_reader.h"

DifferentialDriveRobot::DifferentialDriveRobot(ros::NodeHandle& nh)

    : nh_(nh),
      loopRate_(100),
      CONTROL_KP_(20),
      DRIFT_MULTIPLIER_(250),
      TURN_PWM_(34),
      ANGULAR_VELOCITY_MIN_(0.01),
      LINEAR_VELOCITY_MIN_(0.055),
      LEFT_MOTOR_COMPENSATION_(0.9),
      leftPwmRequired_(0),
      rightPwmRequired_(0),
      lastCmdMsgRcvd_(0),
      encoderReader_(),
      motorController_(encoderReader_) {
  createSubscribers();
}

DifferentialDriveRobot::~DifferentialDriveRobot() {}

void DifferentialDriveRobot::createSubscribers() {
  subForRightWheelTicks_ =
      nh_.subscribe("rightWheel", 1000, &EncoderReader::calculateRightVelocity,
                    &encoderReader_);

  subForLeftWheelTicks_ =
      nh_.subscribe("leftWheel", 1000, &EncoderReader::calculateLeftVelocity,
                    &encoderReader_);

  subForVelocity_ =
      nh_.subscribe("cmd_vel", 1, &DifferentialDriveRobot::setSpeeds, this);
}

void DifferentialDriveRobot::setSpeeds(
    const geometry_msgs::Twist& cmdVelocity) {
  lastCmdMsgRcvd_ = ros::Time::now().toSec();
  calculatePwmRequired(cmdVelocity);
  straightDrivingCorrection();

  ROS_DEBUG_STREAM("CMD_VEL = " << cmdVelocity.linear.x);
  ROS_DEBUG_STREAM("ANG_VEL = " << cmdVelocity.angular.z);
}

void DifferentialDriveRobot::interactMotorGPIOs() {
  static int leftPwmOut = 0;
  static int rightPwmOut = 0;

  motorController_.setMotorsDirection(leftPwmOut, rightPwmOut, *this);
  motorController_.bumpStart(leftPwmOut, rightPwmOut, *this);
  motorController_.incrementPwm(leftPwmOut, rightPwmOut, *this);
  motorController_.capPwmOutputs(leftPwmOut, rightPwmOut, *this);
  motorController_.sendPwmSignals(leftPwmOut, rightPwmOut);
}

void DifferentialDriveRobot::run() {
  while (ros::ok()) {
    ros::spinOnce();

    if (ros::Time::now().toSec() - lastCmdMsgRcvd_ > 1) {
      ROS_INFO_STREAM(
          "NOT RECEIVING CMD_VEL - STOPPING MOTORS  --  time since last = "
          << ros::Time::now().toSec() - lastCmdMsgRcvd_);

      leftPwmRequired_ = 0;
      rightPwmRequired_ = 0;
    }

    interactMotorGPIOs();

    loopRate_.sleep();
  }
}

void DifferentialDriveRobot::calculatePwmRequired(
    const geometry_msgs::Twist& cmdVelocity) {
  int controlB = (abs(cmdVelocity.linear.x) > LINEAR_VELOCITY_MIN_ &&
                  abs(cmdVelocity.linear.x) < .082)
                     ? 30
                     : 40;

  if (abs(cmdVelocity.angular.z) > ANGULAR_VELOCITY_MIN_) {
    if (cmdVelocity.angular.z >= ANGULAR_VELOCITY_MIN_) {
      leftPwmRequired_ = -TURN_PWM_;
      rightPwmRequired_ = (1.0 / LEFT_MOTOR_COMPENSATION_) * TURN_PWM_;
    } else if (cmdVelocity.angular.z < -ANGULAR_VELOCITY_MIN_) {
      leftPwmRequired_ = TURN_PWM_;
      rightPwmRequired_ = -(1.0 / LEFT_MOTOR_COMPENSATION_) * TURN_PWM_;
    }
  } else if (abs(cmdVelocity.linear.x) <= LINEAR_VELOCITY_MIN_) {
    leftPwmRequired_ = 0;
    rightPwmRequired_ = 0;
  } else if (abs(cmdVelocity.linear.x) > LINEAR_VELOCITY_MIN_) {
    if (cmdVelocity.linear.x > 0) {
      leftPwmRequired_ =
          CONTROL_KP_ * LEFT_MOTOR_COMPENSATION_ * cmdVelocity.linear.x +
          controlB;
      rightPwmRequired_ = CONTROL_KP_ * (1.0 / LEFT_MOTOR_COMPENSATION_) *
                              cmdVelocity.linear.x +
                          controlB;
    } else if (cmdVelocity.linear.x <= -LINEAR_VELOCITY_MIN_) {
      leftPwmRequired_ = CONTROL_KP_ * cmdVelocity.linear.x - controlB;
      rightPwmRequired_ = CONTROL_KP_ * cmdVelocity.linear.x - controlB;
    }
  }
}

void DifferentialDriveRobot::straightDrivingCorrection() {
  // correct offset between left and right for straight-line driving
  double leftVelocity = getLeftVelocity();
  double rightVelocity = getRightVelocity();

  static double lastLrVelocityDelta = 0;
  static double penultimateLrVelocityDelta = 0;
  static double lastAvgLrVelocityDelta = 0;

  double lrVelocityDelta = leftVelocity - rightVelocity;
  double avgLrVelocityDelta =
      (lastLrVelocityDelta + penultimateLrVelocityDelta + lrVelocityDelta) / 3;

  ROS_DEBUG_STREAM("lastLrVelocityDelta = " << lastLrVelocityDelta);
  ROS_DEBUG_STREAM(
      "penultimateLrVelocityDelta = " << penultimateLrVelocityDelta);
  ROS_DEBUG_STREAM("lrVelocityDelta = " << lrVelocityDelta);
  ROS_DEBUG_STREAM("abs(avgLrVelocityDelta) =  " << abs(avgLrVelocityDelta));

  penultimateLrVelocityDelta = lastLrVelocityDelta;
  lastLrVelocityDelta = lrVelocityDelta;

  double lrVelocityCorrectionLimit = 0.01;

  if (abs(avgLrVelocityDelta) > lrVelocityCorrectionLimit) {
    ROS_DEBUG_STREAM("Incrementing for straight-line driving... ");
    ROS_DEBUG_STREAM(
        "leftPwmRequired_ (before incremented) = " << leftPwmRequired_);
    ROS_DEBUG_STREAM(
        "rightPwmReqired_ (before incremented) = " << rightPwmRequired_);

    leftPwmRequired_ -=
        static_cast<int>(avgLrVelocityDelta * DRIFT_MULTIPLIER_);
    rightPwmRequired_ +=
        static_cast<int>(avgLrVelocityDelta * DRIFT_MULTIPLIER_);

    ROS_DEBUG_STREAM(
        "leftPwmRequired_ (after incremented) = " << leftPwmRequired_);
    ROS_DEBUG_STREAM(
        "rightPwmRequired_ (after incremented) = " << rightPwmRequired_);

    lastAvgLrVelocityDelta = avgLrVelocityDelta;

  } else if (abs(avgLrVelocityDelta) <= lrVelocityCorrectionLimit) {
    leftPwmRequired_ -=
        static_cast<int>(lastAvgLrVelocityDelta * DRIFT_MULTIPLIER_);
    rightPwmRequired_ +=
        static_cast<int>(lastAvgLrVelocityDelta * DRIFT_MULTIPLIER_);
  }
}

double DifferentialDriveRobot::getLeftVelocity() {
  return encoderReader_.getLeftVelocity();
}

double DifferentialDriveRobot::getRightVelocity() {
  return encoderReader_.getRightVelocity();
}

double DifferentialDriveRobot::getLeftPwmRequired() const {
  return leftPwmRequired_;
}

double DifferentialDriveRobot::getRightPwmRequired() const {
  return rightPwmRequired_;
}

void DifferentialDriveRobot::setLeftPwmRequired(double leftPwmRequired) {
  leftPwmRequired_ = leftPwmRequired;
}

void DifferentialDriveRobot::setRightPwmRequired(double rightPwmRequired) {
  rightPwmRequired_ = rightPwmRequired;
}
