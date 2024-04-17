//  Copyright 2024 <Ian McNally>

#ifndef PRACTICAL_NAV_INCLUDE_PRACTICAL_NAV_DIFFERENTIAL_DRIVE_ROBOT_H_
#define PRACTICAL_NAV_INCLUDE_PRACTICAL_NAV_DIFFERENTIAL_DRIVE_ROBOT_H_

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "practical_nav/encoder_reader.h"
#include "practical_nav/motor_controller.h"

class DifferentialDriveRobot {
 public:
  DifferentialDriveRobot(ros::NodeHandle&);
  ~DifferentialDriveRobot();

  double getLeftPwmRequired() const;
  double getRightPwmRequired() const;

  void setLeftPwmRequired(double leftPwmRequired);
  void setRightPwmRequired(double rightPwmRequired);

  void run();

 private:
  int pigpioSetup();
  void createSubscribers();
  void setSpeeds(const geometry_msgs::Twist& cmdVelocity);
  void calculatePwmRequired(const geometry_msgs::Twist& cmdVelocity);
  void straightDrivingCorrection();
  void interactMotorGPIOs();
  void setMotorsDirection(int leftPwmOut, int rightPwmOut);
  void bumpStart(int leftPwmOut, int rightPwmOut);
  void incrementPwm(int& leftPwmOut, int& rightPwmOut);
  void capPwmOutputs(int& leftPwmOut, int& rightPwmOut);
  double getLeftVelocity();
  double getRightVelocity();

  ros::NodeHandle nh_;
  ros::Rate loopRate_;
  ros::Subscriber subForRightWheelTicks_;
  ros::Subscriber subForLeftWheelTicks_;
  ros::Subscriber subForVelocity_;

  const int CONTROL_KP_;
  const int DRIFT_MULTIPLIER_;
  const int TURN_PWM_;
  const double ANGULAR_VELOCITY_MIN_;
  const double LINEAR_VELOCITY_MIN_;
  const double LEFT_MOTOR_COMPENSATION_;

  double leftPwmRequired_;
  double rightPwmRequired_;
  double lastCmdMsgRcvd_;

  EncoderReader encoderReader_;
  MotorController motorController_;
};

#endif  // PRACTICAL_NAV_INCLUDE_PRACTICAL_NAV_DIFFERENTIAL_DRIVE_ROBOT_H_
