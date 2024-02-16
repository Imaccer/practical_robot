
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
  void setSpeeds(const geometry_msgs::Twist& cmdVel);
  void setPinValues();

  ros::NodeHandle nh_;
  ros::Rate loopRate_;
  ros::Subscriber subForRightCounts_;
  ros::Subscriber subForLeftCounts_;
  ros::Subscriber subForVelocity_;

  const int ENCODER_RANGE_;
  const int PWM_INCREMENT_;
  const double WHEEL_RADIUS_;
  const double WHEELBASE_;
  const double TICKS_PER_M_;
  const int KP_;              
  const int DRIFT_MULTIPLIER_;
  const int TURN_PWM_;
  const int MAX_TURN_PWM_;
  const int MIN_PWM_;
  const int MAX_PWM_;        
  const double VEL_MIN_; 
  const double L_MOTOR_COMP_;  
  const int PWM_L_;
  const int MOTOR_L_FWD_;
  const int MOTOR_L_REV_; 
  const int PWM_R_;
  const int MOTOR_R_FWD_;
  const int MOTOR_R_REV_;
  const int LEFT_PWM_FREQ_;
  const int RIGHT_PWM_FREQ_;

  double leftVelocity_;
  double rightVelocity_;
  double leftPwmReq_;
  double rightPwmReq_;
  double lastCmdMsgRcvd_;
  int pi_;
};

#endif  // PRACTICAL_NAV_INCLUDE_PRACTICAL_NAV_DIFFERENTIAL_DRIVE_ROBOT_H_
