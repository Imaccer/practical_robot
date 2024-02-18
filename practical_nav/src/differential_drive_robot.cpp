//  Copyright 2024 <Ian McNally>

#include "practical_nav/differential_drive_robot.h"
#include "practical_nav/encoder_reader.h"
#include <ros/ros.h>
#include <iostream>

DifferentialDriveRobot::DifferentialDriveRobot(ros::NodeHandle& nh)

    : nh_(nh),
      loopRate_(100),
      PWM_INCREMENT_(1),
      CONTROL_KP_(20),
      DRIFT_MULTIPLIER_(250),
      TURN_PWM_(34),
      MAX_TURN_PWM_(55),
      MIN_PWM_(25),
      MAX_PWM_(90),
      ANGULAR_VELOCITY_MIN_(0.01),
      LINEAR_VELOCITY_MIN_(0.055),
      LEFT_MOTOR_COMPENSATION_(0.9),
      LEFT_PWM_PIN_(21),
      LEFT_MOTOR_FWD_PIN_(26),
      LEFT_MOTOR_REV_PIN_(13),
      LEFT_PWM_FREQ_(100),
      RIGHT_PWM_PIN_(19),
      RIGHT_MOTOR_FWD_PIN_(12),
      RIGHT_MOTOR_REV_PIN_(20),
      RIGHT_PWM_FREQ_(100),
      leftPwmRequired_(0),
      rightPwmRequired_(0),
      lastCmdMsgRcvd_(0),
      pi_(-1) {
  pi_ = pigpioSetup();
  EncoderReader encoderReader_;
  createSubscribers();
}

DifferentialDriveRobot::~DifferentialDriveRobot() {
  //  Ensure motors are in OFF state
  gpio_write(pi_, LEFT_MOTOR_FWD_PIN_, 1);
  gpio_write(pi_, LEFT_MOTOR_REV_PIN_, 1);
  gpio_write(pi_, RIGHT_MOTOR_FWD_PIN_, 1);
  gpio_write(pi_, RIGHT_MOTOR_REV_PIN_, 1);
}

void DifferentialDriveRobot::createSubscribers() {
  subForRightWheelTicks_ =
      nh_.subscribe("rightWheel", 1000,
                    &EncoderReader::calculateRightVelocity, &encoderReader_);

  subForLeftWheelTicks_ = nh_.subscribe(
      "leftWheel", 1000, &EncoderReader::calculateLeftVelocity, &encoderReader_);

  subForVelocity_ =
      nh_.subscribe("cmd_vel", 1, &DifferentialDriveRobot::setSpeeds, this);
}

void DifferentialDriveRobot::setSpeeds(
    const geometry_msgs::Twist& cmdVelocity) {
  lastCmdMsgRcvd_ = ros::Time::now().toSec();
  setInitialPwms(cmdVelocity);
  straightDrivingCorrection();

  ROS_DEBUG_STREAM("CMD_VEL = " << cmdVelocity.linear.x);
  ROS_DEBUG_STREAM("ANG_VEL = " << cmdVelocity.angular.z);
}

void DifferentialDriveRobot::setPinValues() {
  static int leftPwmOut = 0;
  static int rightPwmOut = 0;

  setMotorsDirection(leftPwmOut, rightPwmOut);
  bumpStart(leftPwmOut, rightPwmOut);
  incrementPwm(leftPwmOut, rightPwmOut);
  capPwmOutputs(leftPwmOut, rightPwmOut);

  // write the pwm values to the pins
  set_PWM_dutycycle(pi_, LEFT_PWM_PIN_, leftPwmOut);
  set_PWM_dutycycle(pi_, RIGHT_PWM_PIN_, rightPwmOut);

  ROS_INFO_STREAM("PWM OUT LEFT : " << leftPwmOut << std::endl
                                    << "PWM OUT RIGHT: " << rightPwmOut
                                    << std::endl);
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

    setPinValues();

    loopRate_.sleep();
  }
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

  // adjust pwm frequency to avoid noise
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

void DifferentialDriveRobot::setInitialPwms(
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
  double leftVelocity =  getLeftVelocity();
  double rightVelocity =  getRightVelocity();
 
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

  if (abs(leftPwmRequired_) < MIN_PWM_) {
    leftPwmRequired_ = 0;
  }

  if (abs(rightPwmRequired_) < MIN_PWM_) {
    rightPwmRequired_ = 0;
  }
}

void DifferentialDriveRobot::setMotorsDirection(int leftPwmOut,
                                                int rightPwmOut) {
  // if PwmReq*velocity is negative, means wheel switching
  // directions and should be stopped first
  double leftVelocity = getLeftVelocity();
  double rightVelocity = getRightVelocity();

  if ((leftPwmRequired_ * leftVelocity < 0 && leftPwmOut != 0) ||
      (rightPwmRequired_ * rightVelocity < 0 && rightPwmOut != 0)) {
    ROS_INFO_STREAM("Resetting pwms to 0");
    leftPwmRequired_ = 0;
    rightPwmRequired_ = 0;
  }

  // set motor driver direction pins
  if (leftPwmRequired_ > 0) {  // left fwd
    ROS_DEBUG_STREAM("leftPwmRequired : " << leftPwmRequired_);
    ROS_DEBUG_STREAM("left forward");
    gpio_write(pi_, LEFT_MOTOR_REV_PIN_, 1);
    gpio_write(pi_, LEFT_MOTOR_FWD_PIN_, 0);
  } else if (leftPwmRequired_ < 0) {  // left rev
    gpio_write(pi_, LEFT_MOTOR_FWD_PIN_, 1);
    gpio_write(pi_, LEFT_MOTOR_REV_PIN_, 0);
  } else if (leftPwmRequired_ == 0 && leftPwmOut == 0) {  // left stop
    gpio_write(pi_, LEFT_MOTOR_FWD_PIN_, 1);
    gpio_write(pi_, LEFT_MOTOR_REV_PIN_, 1);
  }

  if (rightPwmRequired_ > 0) {  // right fwd
    gpio_write(pi_, RIGHT_MOTOR_REV_PIN_, 1);
    gpio_write(pi_, RIGHT_MOTOR_FWD_PIN_, 0);
  } else if (rightPwmRequired_ < 0) {  // right rev
    gpio_write(pi_, RIGHT_MOTOR_FWD_PIN_, 1);
    gpio_write(pi_, RIGHT_MOTOR_REV_PIN_, 0);
  } else if (rightPwmRequired_ == 0 && rightPwmOut == 0) {
    gpio_write(pi_, RIGHT_MOTOR_FWD_PIN_, 1);
    gpio_write(pi_, RIGHT_MOTOR_REV_PIN_, 1);
  }
}

void DifferentialDriveRobot::bumpStart(int leftPwmOut, int rightPwmOut) {
  //  give robot extra bump to get moving if needed
  double leftVelocity = getLeftVelocity();
  double rightVelocity =  getRightVelocity();

  const double velocityTol = 1e-6;
  if (leftPwmRequired_ != 0 && (abs(leftVelocity) < velocityTol)) {
    if (abs(leftPwmRequired_) < MAX_PWM_ && leftPwmOut >= MIN_PWM_) {
      leftPwmRequired_ *= 1.4;
      ROS_DEBUG_STREAM("After bump leftPwmRequired_: " << leftPwmRequired_);
    }
  }
  if (rightPwmRequired_ != 0 && (abs(rightVelocity) < velocityTol)) {
    if (abs(rightPwmRequired_) < MAX_PWM_ && leftPwmOut >= MIN_PWM_) {
      rightPwmRequired_ *= 1.4;
      ROS_DEBUG_STREAM("After bump rightPwmRequired_: " << rightPwmRequired_);
    }
  }
}

void DifferentialDriveRobot::incrementPwm(int& leftPwmOut, int& rightPwmOut) {
  // increments PWM changes instead of jarring/dangerous sudden big
  // changes
  double leftVelocity = getLeftVelocity();
  double rightVelocity = getRightVelocity();

  if (abs(leftPwmRequired_) > leftPwmOut) {
    leftPwmOut += PWM_INCREMENT_;
  } else if (abs(leftPwmRequired_) < leftPwmOut) {
    leftPwmOut -= PWM_INCREMENT_;
  }
  if (abs(rightPwmRequired_) > rightPwmOut) {
    rightPwmOut += PWM_INCREMENT_;
  } else if (abs(rightPwmRequired_) < rightPwmOut) {
    rightPwmOut -= PWM_INCREMENT_;
  }
}

void DifferentialDriveRobot::capPwmOutputs(int& leftPwmOut, int& rightPwmOut) {
  // cap output at max defined in constants
  leftPwmOut = (leftPwmOut > MAX_PWM_) ? MAX_PWM_ : leftPwmOut;
  rightPwmOut = (rightPwmOut > MAX_PWM_) ? MAX_PWM_ : rightPwmOut;

  if ((leftPwmRequired_ < 0) || (rightPwmRequired_ < 0)) {
    leftPwmOut = (leftPwmOut > MAX_TURN_PWM_) ? MAX_TURN_PWM_ : leftPwmOut;
    rightPwmOut = (rightPwmOut > MAX_TURN_PWM_) ? MAX_TURN_PWM_ : rightPwmOut;
  }

  if ((leftPwmOut < 0) || (rightPwmOut < 0)) {
    ROS_DEBUG_STREAM("PwmOut values -ve");
  }
  // limit output to a low of zero
  leftPwmOut = (leftPwmOut < 0) ? 0 : leftPwmOut;
  rightPwmOut = (rightPwmOut < 0) ? 0 : rightPwmOut;
}

double DifferentialDriveRobot::getLeftVelocity() {
   return encoderReader_.getLeftVelocity();
}

double DifferentialDriveRobot::getRightVelocity() {
   return encoderReader_.getRightVelocity();
}
