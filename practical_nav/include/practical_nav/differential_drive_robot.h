
#ifndef PRACTICAL_NAV_INCLUDE_PRACTICAL_NAV_DIFFERENTIAL_DRIVE_ROBOT_H_
#define PRACTICAL_NAV_INCLUDE_PRACTICAL_NAV_DIFFERENTIAL_DRIVE_ROBOT_H_

#include <pigpiod_if2.h>
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "std_msgs/Int16.h"

class DifferentialDriveRobot {
 public:
  DifferentialDriveRobot(ros::NodeHandle&);
  ~DifferentialDriveRobot();

  void run();

 private:
  int pigpioSetup();
  void createSubscribers();
  void calculateLeftVelocity(const std_msgs::Int16& leftCount);
  void calculateRightVelocity(const std_msgs::Int16& rightCount);
  void setSpeeds(const geometry_msgs::Twist& cmdVelocity);
  void setInitialPwms(const geometry_msgs::Twist& cmdVelocity);
  void straightDrivingCorrection();
  void setPinValues();
  void setMotorsDirection(int leftPwmOut, int rightPwmOut);
  void bumpStart(int leftPwmOut, int rightPwmOut);
  void incrementPwm(int& leftPwmOut,int& rightPwmOut);
  void capPwmOutputs(int& leftPwmOut, int& rightPwmOut);

  ros::NodeHandle nh_;
  ros::Rate loopRate_;
  ros::Subscriber subForRightWheelTicks_;
  ros::Subscriber subForLeftWheelTicks_;
  ros::Subscriber subForVelocity_;

  const int ENCODER_RANGE_;
  const int PWM_INCREMENT_;
  const double WHEEL_RADIUS_;
  const double WHEELBASE_;
  const double TICKS_PER_M_;
  const int CONTROL_KP_;              
  const int DRIFT_MULTIPLIER_;
  const int TURN_PWM_;
  const int MAX_TURN_PWM_;
  const int MIN_PWM_;
  const int MAX_PWM_;        
  const double ANGULAR_VELOCITY_MIN_;
  const double LINEAR_VELOCITY_MIN_; 
  const double LEFT_MOTOR_COMPENSATION_;  
  const int LEFT_PWM_PIN_;
  const int LEFT_MOTOR_FWD_PIN_;
  const int LEFT_MOTOR_REV_PIN_; 
  const int RIGHT_PWM_PIN_;
  const int RIGHT_MOTOR_FWD_PIN_;
  const int RIGHT_MOTOR_REV_PIN_;
  const int LEFT_PWM_FREQ_;
  const int RIGHT_PWM_FREQ_;

  double leftVelocity_;
  double rightVelocity_;
  double leftPwmRequired_;
  double rightPwmRequired_;
  double lastCmdMsgRcvd_;
  int pi_;
};

#endif  // PRACTICAL_NAV_INCLUDE_PRACTICAL_NAV_DIFFERENTIAL_DRIVE_ROBOT_H_
