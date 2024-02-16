#include "practical_nav/differential_drive_robot.h"
#include <ros/ros.h>
#include <iostream>

using std::cout;
using std::endl;

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
      VEL_MIN_(0.055),
      L_MOTOR_COMP_(0.9),
      PWM_L_(21),
      MOTOR_L_FWD_(26),
      MOTOR_L_REV_(13),
      PWM_R_(19),
      MOTOR_R_FWD_(12),
      MOTOR_R_REV_(20),
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
  gpio_write(pi_, MOTOR_L_FWD_, 1);
  gpio_write(pi_, MOTOR_L_REV_, 1);
  gpio_write(pi_, MOTOR_R_FWD_, 1);
  gpio_write(pi_, MOTOR_R_REV_, 1);
}

int DifferentialDriveRobot::pigpioSetup() {
  char* addrStr = NULL;
  char* portStr = NULL;
  int pi_ = pigpio_start(addrStr, portStr);
  // next 10 lines sets up our pins. Remember that high is "off"
  // and we must drive in1 or in2 low to start the output to motor
  set_mode(pi_, PWM_L_, PI_OUTPUT);
  set_mode(pi_, MOTOR_L_FWD_, PI_OUTPUT);
  set_mode(pi_, MOTOR_L_REV_, PI_OUTPUT);
  set_mode(pi_, PWM_R_, PI_OUTPUT);
  set_mode(pi_, MOTOR_R_FWD_, PI_OUTPUT);
  set_mode(pi_, MOTOR_R_REV_, PI_OUTPUT);
  int setRightPwmFreq = set_PWM_frequency(pi_, PWM_R_, RIGHT_PWM_FREQ_);
  if (setRightPwmFreq < 0) {
    printf("Error setting PWM_R frequency: %d\n", setRightPwmFreq);
  }

  int setLeftPwmFreq = set_PWM_frequency(pi_, PWM_L_, LEFT_PWM_FREQ_);
  if (setLeftPwmFreq < 0) {
    printf("Error setting PWM_L frequency: %d\n", setLeftPwmFreq);
  }

  // initialize motors off
  gpio_write(pi_, MOTOR_L_FWD_, 1);
  gpio_write(pi_, MOTOR_L_REV_, 1);
  gpio_write(pi_, MOTOR_R_FWD_, 1);
  gpio_write(pi_, MOTOR_R_REV_, 1);

  if (pi_ >= 0) {
    cout << "daemon interface started ok at " << pi_ << endl;
    return 0;
  } else {
    cout << "Failed to connect to PIGPIO Daemon - is it running?" << endl;
    return -1;
  }
}

void DifferentialDriveRobot::createSubscribers() {
  subForRightCounts_ =
      nh_.subscribe("rightWheel", 1000,
                      &DifferentialDriveRobot::calculateRightVelocity, this);

  subForLeftCounts_ = nh_.subscribe(
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
  int b = (abs(cmdVel.linear.x) > VEL_MIN_ && abs(cmdVel.linear.x) < .082) ? 30
                                                                           : 40;
  double cmdVelEpsilon = 0.01;
  double cmdAngVelEpsilon = 0.001;

  if (abs(cmdVel.angular.z) > 0.01) {
    if (cmdVel.angular.z >= .01) {
      leftPwmReq_ = -TURN_PWM_;
      rightPwmReq_ = (1.0 / L_MOTOR_COMP_) * TURN_PWM_;
    } else if (cmdVel.angular.z < -.01) {
      leftPwmReq_ = TURN_PWM_;
      rightPwmReq_ = -(1.0 / L_MOTOR_COMP_) * TURN_PWM_;
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
  } else if (abs(cmdVel.linear.x) <= VEL_MIN_) {
    leftPwmReq_ = 0;
    rightPwmReq_ = 0;
  } else if (abs(cmdVel.linear.x) > VEL_MIN_) {
    if (cmdVel.linear.x > 0) {
      leftPwmReq_ = KP_ * L_MOTOR_COMP_ * cmdVel.linear.x + b;
      rightPwmReq_ = KP_ * (1.0 / L_MOTOR_COMP_) * cmdVel.linear.x + b;
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

    cout << "prev_diff = " << prevDiff << endl;
    cout << "prev_prev_diff = " << prevPrevDiff << endl;
    cout << "angular_velocity_diff = " << angularVelDifference << endl;
    cout << "avg_vel_diff = " << abs(avgAngularDiff) << endl;

    prevPrevDiff = prevDiff;
    prevDiff = angularVelDifference;

    if (abs(avgAngularDiff) > cmdVelEpsilon) {
      cout << "in linear control loop: " << endl;
      cout << "ang_vel_diff = " << abs(avgAngularDiff) << endl;
      cout << "leftPwmReq_before_inc = " << leftPwmReq_ << endl;
      cout << "rightPwmReq_before_inc = " << rightPwmReq_ << endl;

      leftPwmReq_ -= (int)(avgAngularDiff * DRIFT_MULTIPLIER_);
      rightPwmReq_ += (int)(avgAngularDiff * DRIFT_MULTIPLIER_);

      cout << "leftPwmReq_after_inc = " << leftPwmReq_ << endl;
      cout << "rightPwmReq_after_inc = " << rightPwmReq_ << endl;

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
  cout << "CMD_VEL = " << cmdVel.linear.x << endl;
  cout << "ANG_VEL = " << cmdVel.angular.z << endl;
}

void DifferentialDriveRobot::setPinValues() {
  static int leftPwmOut = 0;
  static int rightPwmOut = 0;

  // if PwmReq*PwmOut is negative, that means the wheel is switching
  // directions and we should bring to a stop before switching directions
  static bool stopped = false;
  if ((leftPwmReq_ * leftVelocity_ < 0 && leftPwmOut != 0) ||
      (rightPwmReq_ * rightVelocity_ < 0 && rightPwmOut != 0)) {
    cout << "Resetting pwms to 0" << endl;
    leftPwmReq_ = 0;
    rightPwmReq_ = 0;
  }

  // set motor driver direction pins
  if (leftPwmReq_ > 0) {  // left fwd
    cout << "leftPwmReq : " << leftPwmReq_ << endl;
    cout << "left forward" << endl;
    gpio_write(pi_, MOTOR_L_REV_, 1);
    gpio_write(pi_, MOTOR_L_FWD_, 0);
  } else if (leftPwmReq_ < 0) {  // left rev
    gpio_write(pi_, MOTOR_L_FWD_, 1);
    gpio_write(pi_, MOTOR_L_REV_, 0);
  } else if (leftPwmReq_ == 0 && leftPwmOut == 0) {  // left stop
    gpio_write(pi_, MOTOR_L_FWD_, 1);
    gpio_write(pi_, MOTOR_L_REV_, 1);
  }

  if (rightPwmReq_ > 0) {  // right fwd
    gpio_write(pi_, MOTOR_R_REV_, 1);
    gpio_write(pi_, MOTOR_R_FWD_, 0);
  } else if (rightPwmReq_ < 0) {  // right rev
    gpio_write(pi_, MOTOR_R_FWD_, 1);
    gpio_write(pi_, MOTOR_R_REV_, 0);
  } else if (rightPwmReq_ == 0 && rightPwmOut == 0) {
    gpio_write(pi_, MOTOR_R_FWD_, 1);
    gpio_write(pi_, MOTOR_R_REV_, 1);
  }

  const double vel_eps = 1e-6;

  if (leftPwmReq_ != 0 && (abs(leftVelocity_) < vel_eps)) {
    if (abs(leftPwmReq_) < MAX_PWM_ && leftPwmOut >= MIN_PWM_) {
      leftPwmReq_ *= 1.4;
      cout << "After bump leftpwmreq: " << leftPwmReq_ << endl;
      cout << "After bump leftpwmout: " << leftPwmOut << endl;
    }
  }
  if (rightPwmReq_ != 0 && (abs(rightVelocity_) < vel_eps)) {
    if (abs(rightPwmReq_) < MAX_PWM_ && leftPwmOut >= MIN_PWM_) {
      rightPwmReq_ *= 1.4;
      cout << "After bump rightpwmreq: " << rightPwmReq_ << endl;
      cout << "After bump rightpwmout: " << rightPwmOut << endl;
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
    cout << "PwmOut values -ve" << endl;
  }
  // limit output to a low of zero
  leftPwmOut = (leftPwmOut < 0) ? 0 : leftPwmOut;
  rightPwmOut = (rightPwmOut < 0) ? 0 : rightPwmOut;
  // write the pwm values tot he pins
  set_PWM_dutycycle(pi_, PWM_L_, leftPwmOut);
  set_PWM_dutycycle(pi_, PWM_R_, rightPwmOut);

  cout << "PWM OUT LEFT AND RIGHT            " << leftPwmOut << "           "
       << rightPwmOut << endl
       << endl;
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
