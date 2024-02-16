#include "practical_nav/differential_drive_robot.h"
#include <ros/ros.h>
#include <iostream>

DifferentialDriveRobot::DifferentialDriveRobot(ros::NodeHandle& nh)

    : nh_(nh),
      loopRate_(100),
      ENCODER_RANGE_(65535),
      PWM_INCREMENT_(1),
      WHEEL_RADIUS_(0.03575),
      WHEELBASE_(0.224),
      TICKS_PER_M_(2270),
      KP_(20),
      DRIFT_MULTIPLIER_(250),
      TURN_PWM_(34),
      MAX_TURN_PWM_(55),
      MIN_PWM_(25),
      MAX_PWM_(90),
      LINEAR_VELOCITY_MIN_(0.055),
      LEFT_MOTOR_COMPENSATION_(0.9),
      LEFT_PWM_PIN_(21),
      LEFT_MOTOR_FWD_PIN_(26),
      LEFT_MOTOR_REV_PIN_(13),
      RIGHT_PWM_PIN_(19),
      RIGHT_MOTOR_FWD_PIN_(12),
      RIGHT_MOTOR_REV_PIN_(20),
      LEFT_PWM_FREQ_(100),
      RIGHT_PWM_FREQ_(100),
      leftVelocity_(0),
      rightVelocity_(0),
      leftPwmReq_(0),
      rightPwmReq_(0),
      lastCmdMsgRcvd_(0),
      pi_(-1) {
  pi_ = pigpioSetup();
  createSubscribers();
}

DifferentialDriveRobot::~DifferentialDriveRobot() {
  //  Ensure motors are in OFF state
  gpio_write(pi_, LEFT_MOTOR_FWD_PIN_, 1);
  gpio_write(pi_, LEFT_MOTOR_REV_PIN_, 1);
  gpio_write(pi_, RIGHT_MOTOR_FWD_PIN_, 1);
  gpio_write(pi_, RIGHT_MOTOR_REV_PIN_, 1);
}

int DifferentialDriveRobot::pigpioSetup() {
  char* addrStr = NULL;
  char* portStr = NULL;
  int pi_ = pigpio_start(addrStr, portStr);
  
  // drive in1 or in2 low to start the output to motor
  set_mode(pi_, LEFT_PWM_PIN_, PI_OUTPUT);
  set_mode(pi_, LEFT_MOTOR_FWD_PIN_, PI_OUTPUT);
  set_mode(pi_, LEFT_MOTOR_REV_PIN_, PI_OUTPUT);
  set_mode(pi_, RIGHT_PWM_PIN_, PI_OUTPUT);
  set_mode(pi_, RIGHT_MOTOR_FWD_PIN_, PI_OUTPUT);
  set_mode(pi_, RIGHT_MOTOR_REV_PIN_, PI_OUTPUT);

  int setRightPwmFreq = set_PWM_frequency(pi_, RIGHT_PWM_PIN_, RIGHT_PWM_FREQ_);
  int setLeftPwmFreq = set_PWM_frequency(pi_, LEFT_PWM_PIN_, LEFT_PWM_FREQ_);

  if (setRightPwmFreq < 0) {
    ROS_ERROR_STREAM("Error setting PWM_R frequency: " << setRightPwmFreq);
  }

  if (setLeftPwmFreq < 0) {
    ROS_ERROR_STREAM("Error setting PWM_L frequency: " << setLeftPwmFreq);
  }

  // initialize motors off
  gpio_write(pi_, LEFT_MOTOR_FWD_PIN_, 1);
  gpio_write(pi_, LEFT_MOTOR_REV_PIN_, 1);
  gpio_write(pi_, RIGHT_MOTOR_FWD_PIN_, 1);
  gpio_write(pi_, RIGHT_MOTOR_REV_PIN_, 1);

  if (pi_ >= 0) {
    ROS_INFO_STREAM("Daemon interface started ok, pi: " << pi_);
    return 0;
  } else {
    ROS_ERROR_STREAM("Failed to connect to PIGPIO Daemon - is it running?");
    return -1;
  }
}

void DifferentialDriveRobot::createSubscribers() {
  subForRightWheelTicks_ =
      nh_.subscribe("rightWheel", 1000,
                    &DifferentialDriveRobot::calculateRightVelocity, this);

  subForLeftWheelTicks_ = nh_.subscribe(
      "leftWheel", 1000, &DifferentialDriveRobot::calculateLeftVelocity, this);

  subForVelocity_ =
      nh_.subscribe("cmd_vel", 1, &DifferentialDriveRobot::setSpeeds, this);
}

void DifferentialDriveRobot::calculateLeftVelocity(
    const std_msgs::Int16& leftCount) {
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

void DifferentialDriveRobot::calculateRightVelocity(
    const std_msgs::Int16& rightCount) {
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

void DifferentialDriveRobot::setSpeeds(const geometry_msgs::Twist& cmdVel) {
  lastCmdMsgRcvd_ = ros::Time::now().toSec();
  int b = (abs(cmdVel.linear.x) > LINEAR_VELOCITY_MIN_ && abs(cmdVel.linear.x) < .082) ? 30
                                                                           : 40;
  double cmdVelEpsilon = 0.01;
  double cmdAngVelEpsilon = 0.001;

  if (abs(cmdVel.angular.z) > 0.01) {
    if (cmdVel.angular.z >= .01) {
      leftPwmReq_ = -TURN_PWM_;
      rightPwmReq_ = (1.0 / LEFT_MOTOR_COMPENSATION_) * TURN_PWM_;
    } else if (cmdVel.angular.z < -.01) {
      leftPwmReq_ = TURN_PWM_;
      rightPwmReq_ = -(1.0 / LEFT_MOTOR_COMPENSATION_) * TURN_PWM_;
    }

    static double prevRotDiff = 0;
    static double prevPrevRotDiff = 0;
    static double prevAvgAngularRotDiff = 0;
    double angularVelRotDifference =
        leftVelocity_ +
        rightVelocity_;  // how much faster one wheel is actually turning
    double avgAngularRotDiff =
        (prevRotDiff + prevPrevRotDiff + angularVelRotDifference) /
        3;  // average several cycles
    prevPrevRotDiff = prevRotDiff;
    prevRotDiff = angularVelRotDifference;

    if (abs(avgAngularRotDiff) > cmdAngVelEpsilon) {
      leftPwmReq_ -= (int)(avgAngularRotDiff * DRIFT_MULTIPLIER_);
      rightPwmReq_ -= (int)(avgAngularRotDiff * DRIFT_MULTIPLIER_);

      prevAvgAngularRotDiff = avgAngularRotDiff;

    } else if (abs(avgAngularRotDiff) <= cmdAngVelEpsilon) {
      leftPwmReq_ -= (int)(prevAvgAngularRotDiff * DRIFT_MULTIPLIER_);
      rightPwmReq_ -= (int)(prevAvgAngularRotDiff * DRIFT_MULTIPLIER_);
    }
  } else if (abs(cmdVel.linear.x) <= LINEAR_VELOCITY_MIN_) {
    leftPwmReq_ = 0;
    rightPwmReq_ = 0;
  } else if (abs(cmdVel.linear.x) > LINEAR_VELOCITY_MIN_) {
    if (cmdVel.linear.x > 0) {
      leftPwmReq_ = KP_ * LEFT_MOTOR_COMPENSATION_ * cmdVel.linear.x + b;
      rightPwmReq_ = KP_ * (1.0 / LEFT_MOTOR_COMPENSATION_) * cmdVel.linear.x + b;
    } else if (cmdVel.linear.x < 0) {
      leftPwmReq_ = KP_ * cmdVel.linear.x - b;
      rightPwmReq_ = KP_ * cmdVel.linear.x - b;
    }

    static double prevDiff = 0;
    static double prevPrevDiff = 0;
    static double prevAvgAngularDiff = 0;

    double angularVelDifference = leftVelocity_ - rightVelocity_;
    double avgAngularDiff =
        (prevDiff + prevPrevDiff + angularVelDifference) / 3;

    ROS_DEBUG_STREAM("prev_diff = " << prevDiff);
    ROS_DEBUG_STREAM("prev_prev_diff = " << prevPrevDiff);
    ROS_DEBUG_STREAM("angular_velocity_diff = " << angularVelDifference);
    ROS_DEBUG_STREAM("avg_vel_diff = " << abs(avgAngularDiff));

    prevPrevDiff = prevDiff;
    prevDiff = angularVelDifference;

    if (abs(avgAngularDiff) > cmdVelEpsilon) {
      ROS_DEBUG_STREAM("in linear control loop: ");
      ROS_DEBUG_STREAM("ang_vel_diff = " << abs(avgAngularDiff));
      ROS_DEBUG_STREAM("leftPwmReq_before_inc = " << leftPwmReq_);
      ROS_DEBUG_STREAM("rightPwmReq_before_inc = " << rightPwmReq_);

      leftPwmReq_ -= (int)(avgAngularDiff * DRIFT_MULTIPLIER_);
      rightPwmReq_ += (int)(avgAngularDiff * DRIFT_MULTIPLIER_);

      ROS_DEBUG_STREAM("leftPwmReq_after_inc = " << leftPwmReq_);
      ROS_DEBUG_STREAM("rightPwmReq_after_inc = " << rightPwmReq_);

      prevAvgAngularDiff = avgAngularDiff;

    } else if (abs(avgAngularDiff) <= cmdVelEpsilon) {
      leftPwmReq_ -= (int)(prevAvgAngularDiff * DRIFT_MULTIPLIER_);
      rightPwmReq_ += (int)(prevAvgAngularDiff * DRIFT_MULTIPLIER_);
    }
  }

  if (abs(leftPwmReq_) < MIN_PWM_) {
    leftPwmReq_ = 0;
  }

  if (abs(rightPwmReq_) < MIN_PWM_) {
    rightPwmReq_ = 0;
  }
  ROS_DEBUG_STREAM("CMD_VEL = " << cmdVel.linear.x);
  ROS_DEBUG_STREAM("ANG_VEL = " << cmdVel.angular.z);
}

void DifferentialDriveRobot::setPinValues() {
  static int leftPwmOut = 0;
  static int rightPwmOut = 0;

  // if PwmReq*PwmOut is negative, that means the wheel is switching
  // directions and we should bring to a stop before switching directions
  static bool stopped = false;
  if ((leftPwmReq_ * leftVelocity_ < 0 && leftPwmOut != 0) ||
      (rightPwmReq_ * rightVelocity_ < 0 && rightPwmOut != 0)) {
    ROS_INFO_STREAM("Resetting pwms to 0");
    leftPwmReq_ = 0;
    rightPwmReq_ = 0;
  }

  // set motor driver direction pins
  if (leftPwmReq_ > 0) {  // left fwd
    ROS_DEBUG_STREAM("leftPwmReq : " << leftPwmReq_);
    ROS_DEBUG_STREAM("left forward");
    gpio_write(pi_, LEFT_MOTOR_REV_PIN_, 1);
    gpio_write(pi_, LEFT_MOTOR_FWD_PIN_, 0);
  } else if (leftPwmReq_ < 0) {  // left rev
    gpio_write(pi_, LEFT_MOTOR_FWD_PIN_, 1);
    gpio_write(pi_, LEFT_MOTOR_REV_PIN_, 0);
  } else if (leftPwmReq_ == 0 && leftPwmOut == 0) {  // left stop
    gpio_write(pi_, LEFT_MOTOR_FWD_PIN_, 1);
    gpio_write(pi_, LEFT_MOTOR_REV_PIN_, 1);
  }

  if (rightPwmReq_ > 0) {  // right fwd
    gpio_write(pi_, RIGHT_MOTOR_REV_PIN_, 1);
    gpio_write(pi_, RIGHT_MOTOR_FWD_PIN_, 0);
  } else if (rightPwmReq_ < 0) {  // right rev
    gpio_write(pi_, RIGHT_MOTOR_FWD_PIN_, 1);
    gpio_write(pi_, RIGHT_MOTOR_REV_PIN_, 0);
  } else if (rightPwmReq_ == 0 && rightPwmOut == 0) {
    gpio_write(pi_, RIGHT_MOTOR_FWD_PIN_, 1);
    gpio_write(pi_, RIGHT_MOTOR_REV_PIN_, 1);
  }

  const double vel_eps = 1e-6;

  if (leftPwmReq_ != 0 && (abs(leftVelocity_) < vel_eps)) {
    if (abs(leftPwmReq_) < MAX_PWM_ && leftPwmOut >= MIN_PWM_) {
      leftPwmReq_ *= 1.4;
      ROS_DEBUG_STREAM("After bump leftpwmreq: " << leftPwmReq_);
      ROS_DEBUG_STREAM("After bump leftpwmout: " << leftPwmOut);
    }
  }
  if (rightPwmReq_ != 0 && (abs(rightVelocity_) < vel_eps)) {
    if (abs(rightPwmReq_) < MAX_PWM_ && leftPwmOut >= MIN_PWM_) {
      rightPwmReq_ *= 1.4;
      ROS_DEBUG_STREAM("After bump rightpwmreq: " << rightPwmReq_);
      ROS_DEBUG_STREAM("After bump rightpwmout: " << rightPwmOut);
    }
  }

  // this section increments PWM changes instead of jarring/dangeroud sudden big
  // changes
  if (abs(leftPwmReq_) > leftPwmOut) {
    leftPwmOut += PWM_INCREMENT_;
  } else if (abs(leftPwmReq_) < leftPwmOut) {
    leftPwmOut -= PWM_INCREMENT_;
  }
  if (abs(rightPwmReq_) > rightPwmOut) {
    rightPwmOut += PWM_INCREMENT_;
  } else if (abs(rightPwmReq_) < rightPwmOut) {
    rightPwmOut -= PWM_INCREMENT_;
  }
  // cap output at max defined in constants
  leftPwmOut = (leftPwmOut > MAX_PWM_) ? MAX_PWM_ : leftPwmOut;
  rightPwmOut = (rightPwmOut > MAX_PWM_) ? MAX_PWM_ : rightPwmOut;

  if ((leftPwmReq_ < 0) || (rightPwmReq_ < 0)) {
    leftPwmOut = (leftPwmOut > MAX_TURN_PWM_) ? MAX_TURN_PWM_ : leftPwmOut;
    rightPwmOut = (rightPwmOut > MAX_TURN_PWM_) ? MAX_TURN_PWM_ : rightPwmOut;
  }

  if ((leftPwmOut < 0) || (rightPwmOut < 0)) {
    ROS_DEBUG_STREAM("PwmOut values -ve");
  }
  // limit output to a low of zero
  leftPwmOut = (leftPwmOut < 0) ? 0 : leftPwmOut;
  rightPwmOut = (rightPwmOut < 0) ? 0 : rightPwmOut;
  // write the pwm values to the pins
  set_PWM_dutycycle(pi_, LEFT_PWM_PIN_, leftPwmOut);
  set_PWM_dutycycle(pi_, RIGHT_PWM_PIN_, rightPwmOut);

  ROS_INFO_STREAM("PWM OUT LEFT AND RIGHT            "
                  << leftPwmOut << "           " << rightPwmOut << std::endl);
}

void DifferentialDriveRobot::run() {
  while (ros::ok()) {
    ros::spinOnce();

    if (ros::Time::now().toSec() - lastCmdMsgRcvd_ > 1) {
      ROS_INFO_STREAM(
          "NOT RECEIVING CMD_VEL - STOPPING MOTORS  --  time since last = "
          << ros::Time::now().toSec() - lastCmdMsgRcvd_);

      leftPwmReq_ = 0;
      rightPwmReq_ = 0;
    }

    setPinValues();

    loopRate_.sleep();
  }
}
