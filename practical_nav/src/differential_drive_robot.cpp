//  Copyright 2024 <Ian McNally>

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
      leftVelocity_(0),
      rightVelocity_(0),
      leftPwmRequired_(0),
      rightPwmRequired_(0),
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

void DifferentialDriveRobot::setSpeeds(
    const geometry_msgs::Twist& cmdVelocity) {
  lastCmdMsgRcvd_ = ros::Time::now().toSec();
  double lrVelocityCorrectionLimit = 0.01;
  int controlB = (abs(cmdVelocity.linear.x) > LINEAR_VELOCITY_MIN_ &&
                  abs(cmdVelocity.linear.x) < .082)
                     ? 30
                     : 40;

  // Set Pwm initial values depending on cmdVelocity components
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

    // correct offset between left and right for straight-line driving
    static double lastLrVelocityDelta = 0;
    static double penultimateLrVelocityDelta = 0;
    static double lastAvgLrVelocityDelta = 0;

    double lrVelocityDelta = leftVelocity_ - rightVelocity_;
    double avgLrVelocityDelta =
        (lastLrVelocityDelta + penultimateLrVelocityDelta + lrVelocityDelta) /
        3;

    ROS_DEBUG_STREAM("lastLrVelocityDelta = " << lastLrVelocityDelta);
    ROS_DEBUG_STREAM(
        "penultimateLrVelocityDelta = " << penultimateLrVelocityDelta);
    ROS_DEBUG_STREAM("lrVelocityDelta = " << lrVelocityDelta);
    ROS_DEBUG_STREAM("abs(avgLrVelocityDelta) =  " << abs(avgLrVelocityDelta));

    penultimateLrVelocityDelta = lastLrVelocityDelta;
    lastLrVelocityDelta = lrVelocityDelta;

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

  if (abs(leftPwmRequired_) < MIN_PWM_) {
    leftPwmRequired_ = 0;
  }

  if (abs(rightPwmRequired_) < MIN_PWM_) {
    rightPwmRequired_ = 0;
  }
  ROS_DEBUG_STREAM("CMD_VEL = " << cmdVelocity.linear.x);
  ROS_DEBUG_STREAM("ANG_VEL = " << cmdVelocity.angular.z);
}

void DifferentialDriveRobot::setPinValues() {
  static int leftPwmOut = 0;
  static int rightPwmOut = 0;

  // if PwmReq*PwmOut is negative, that means the wheel is switching
  // directions and we should bring to a stop before switching directions
  static bool stopped = false;
  if ((leftPwmRequired_ * leftVelocity_ < 0 && leftPwmOut != 0) ||
      (rightPwmRequired_ * rightVelocity_ < 0 && rightPwmOut != 0)) {
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

  const double velocityTol = 1e-6;

  if (leftPwmRequired_ != 0 && (abs(leftVelocity_) < velocityTol)) {
    if (abs(leftPwmRequired_) < MAX_PWM_ && leftPwmOut >= MIN_PWM_) {
      leftPwmRequired_ *= 1.4;
      ROS_DEBUG_STREAM("After bump leftpwmreq: " << leftPwmRequired_);
      ROS_DEBUG_STREAM("After bump leftpwmout: " << leftPwmOut);
    }
  }
  if (rightPwmRequired_ != 0 && (abs(rightVelocity_) < velocityTol)) {
    if (abs(rightPwmRequired_) < MAX_PWM_ && leftPwmOut >= MIN_PWM_) {
      rightPwmRequired_ *= 1.4;
      ROS_DEBUG_STREAM("After bump rightpwmreq: " << rightPwmRequired_);
      ROS_DEBUG_STREAM("After bump rightpwmout: " << rightPwmOut);
    }
  }

  // this section increments PWM changes instead of jarring/dangeroud sudden big
  // changes
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

      leftPwmRequired_ = 0;
      rightPwmRequired_ = 0;
    }

    setPinValues();

    loopRate_.sleep();
  }
}
