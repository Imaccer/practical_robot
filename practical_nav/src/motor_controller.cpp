//  Copyright 2024 <Ian McNally>

#include "practical_nav/motor_controller.h"
#include <pigpiod_if2.h>
#include <ros/ros.h>
#include "practical_nav/differential_drive_robot.h"
#include "practical_nav/encoder_reader.h"
#include <cstdlib>

MotorController::MotorController(EncoderReader& encoderReader)

    : encoderReader_(encoderReader),
      PWM_INCREMENT_(1),
      MIN_PWM_(25),
      MAX_PWM_(90),
      MAX_TURN_PWM_(55),
      LEFT_PWM_PIN_(21),
      LEFT_PWM_FREQ_(100),
      RIGHT_PWM_PIN_(19),
      RIGHT_PWM_FREQ_(100),
      LEFT_MOTOR_FWD_PIN_(26),
      LEFT_MOTOR_REV_PIN_(13),
      RIGHT_MOTOR_FWD_PIN_(12),
      RIGHT_MOTOR_REV_PIN_(20),
      BUMP_FACTOR_(1.1),
      pi_(-1) {
  pi_ = pigpioSetup();
}

MotorController::~MotorController() {
  //  Ensure motors are in OFF state
  gpio_write(pi_, LEFT_MOTOR_FWD_PIN_, 1);
  gpio_write(pi_, LEFT_MOTOR_REV_PIN_, 1);
  gpio_write(pi_, RIGHT_MOTOR_FWD_PIN_, 1);
  gpio_write(pi_, RIGHT_MOTOR_REV_PIN_, 1);
}

int MotorController::pigpioSetup() {
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

void MotorController::setMotorsDirection(int leftPwmOut, int rightPwmOut,
                                         DifferentialDriveRobot& robot) {
  // if PwmReq*velocity is negative, means wheel switching
  // directions and should be stopped first
  double leftVelocity = getLeftVelocity();
  double rightVelocity = getRightVelocity();
  double leftPwmRequired = robot.getLeftPwmRequired();
  double rightPwmRequired = robot.getRightPwmRequired();

  if (std::abs(leftPwmRequired) < MIN_PWM_) {
    leftPwmRequired = 0;
  }

  if (std::abs(rightPwmRequired) < MIN_PWM_) {
    rightPwmRequired = 0;
  }

  if ((leftPwmRequired * leftVelocity < 0 && leftPwmOut != 0) ||
      (rightPwmRequired * rightVelocity < 0 && rightPwmOut != 0)) {
    ROS_INFO_STREAM("Resetting pwms to 0");
    leftPwmRequired = 0;
    rightPwmRequired = 0;
  }

  //  update pwm required values in robot object
  robot.setLeftPwmRequired(leftPwmRequired);
  robot.setRightPwmRequired(rightPwmRequired);

  // set motor driver direction pins
  if (leftPwmRequired > 0) {  // left fwd
    ROS_DEBUG_STREAM("leftPwmRequired : " << leftPwmRequired);
    ROS_DEBUG_STREAM("left forward");
    gpio_write(pi_, LEFT_MOTOR_REV_PIN_, 1);
    gpio_write(pi_, LEFT_MOTOR_FWD_PIN_, 0);
  } else if (leftPwmRequired < 0) {  // left rev
    gpio_write(pi_, LEFT_MOTOR_FWD_PIN_, 1);
    gpio_write(pi_, LEFT_MOTOR_REV_PIN_, 0);
  } else if (leftPwmRequired == 0 && leftPwmOut == 0) {  // left stop
    gpio_write(pi_, LEFT_MOTOR_FWD_PIN_, 1);
    gpio_write(pi_, LEFT_MOTOR_REV_PIN_, 1);
  }

  if (rightPwmRequired > 0) {  // right fwd
    gpio_write(pi_, RIGHT_MOTOR_REV_PIN_, 1);
    gpio_write(pi_, RIGHT_MOTOR_FWD_PIN_, 0);
  } else if (rightPwmRequired < 0) {  // right rev
    gpio_write(pi_, RIGHT_MOTOR_FWD_PIN_, 1);
    gpio_write(pi_, RIGHT_MOTOR_REV_PIN_, 0);
  } else if (rightPwmRequired == 0 && rightPwmOut == 0) {
    gpio_write(pi_, RIGHT_MOTOR_FWD_PIN_, 1);
    gpio_write(pi_, RIGHT_MOTOR_REV_PIN_, 1);
  }
}

void MotorController::bumpStart(int leftPwmOut, int rightPwmOut,
                                DifferentialDriveRobot& robot) {
  //  give robot extra bump to get moving if needed
  double leftVelocity = getLeftVelocity();
  double rightVelocity = getRightVelocity();
  double leftPwmRequired = robot.getLeftPwmRequired();
  double rightPwmRequired = robot.getRightPwmRequired();

  const double velocityTol = 1e-6;
  if (leftPwmRequired != 0 && (abs(leftVelocity) < velocityTol)) {
    if (abs(leftPwmRequired) < MAX_PWM_ && leftPwmOut >= MIN_PWM_) {
      leftPwmRequired *= BUMP_FACTOR_;
      ROS_DEBUG_STREAM("After bump leftPwmRequired: " << leftPwmRequired);
    }
  }
  if (rightPwmRequired != 0 && (abs(rightVelocity) < velocityTol)) {
    if (abs(rightPwmRequired) < MAX_PWM_ && leftPwmOut >= MIN_PWM_) {
      rightPwmRequired *= BUMP_FACTOR_;
      ROS_DEBUG_STREAM("After bump rightPwmRequired: " << rightPwmRequired);
    }
  }

  //  update pwm required values in robot object
  robot.setLeftPwmRequired(leftPwmRequired);
  robot.setRightPwmRequired(rightPwmRequired);
}

void MotorController::incrementPwm(int& leftPwmOut, int& rightPwmOut,
                                   DifferentialDriveRobot& robot) {
  // increments PWM changes instead of jarring/dangerous sudden big
  // changes
  double leftVelocity = getLeftVelocity();
  double rightVelocity = getRightVelocity();
  double leftPwmRequired = robot.getLeftPwmRequired();
  double rightPwmRequired = robot.getRightPwmRequired();

  if (std::abs(leftPwmRequired) > leftPwmOut) {
    leftPwmOut += PWM_INCREMENT_;
  } else if (std::abs(leftPwmRequired) < leftPwmOut) {
    leftPwmOut -= PWM_INCREMENT_;
  }
  if (std::abs(rightPwmRequired) > rightPwmOut) {
    rightPwmOut += PWM_INCREMENT_;
  } else if (std::abs(rightPwmRequired) < rightPwmOut) {
    rightPwmOut -= PWM_INCREMENT_;
  }
}

void MotorController::capPwmOutputs(int& leftPwmOut, int& rightPwmOut,
                                    DifferentialDriveRobot& robot) {
  // cap output at max defined in constants
  double leftPwmRequired = robot.getLeftPwmRequired();
  double rightPwmRequired = robot.getRightPwmRequired();

  leftPwmOut = (leftPwmOut > MAX_PWM_) ? MAX_PWM_ : leftPwmOut;
  rightPwmOut = (rightPwmOut > MAX_PWM_) ? MAX_PWM_ : rightPwmOut;

  if ((leftPwmRequired < 0) || (rightPwmRequired < 0)) {
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

void MotorController::sendPwmSignals(int leftPwmOut, int rightPwmOut) {
  // write the pwm values to the pins
  set_PWM_dutycycle(pi_, LEFT_PWM_PIN_, leftPwmOut);
  set_PWM_dutycycle(pi_, RIGHT_PWM_PIN_, rightPwmOut);

  ROS_INFO_STREAM("PWM OUT LEFT : " << leftPwmOut << std::endl
                                    << "PWM OUT RIGHT: " << rightPwmOut
                                    << std::endl);
}

double MotorController::getLeftVelocity() const {
  return encoderReader_.getLeftVelocity();
}

double MotorController::getRightVelocity() const {
  return encoderReader_.getRightVelocity();
}
