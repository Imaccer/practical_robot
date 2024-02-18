//  Copyright 2024 <Ian McNally>

// motor_controller.h
#ifndef PRACTICAL_NAV_INCLUDE_PRACTICAL_NAV_MOTOR_CONTROLLER_H_
#define PRACTICAL_NAV_INCLUDE_PRACTICAL_NAV_MOTOR_CONTROLLER_H_

#include <pigpiod_if2.h>
#include "practical_nav/encoder_reader.h"

class DifferentialDriveRobot;

class MotorController {
 public:
  MotorController(EncoderReader& encoderReader);
  ~MotorController();

  void setMotorsDirection(int leftPwmOut, int rightPwmOut,
                          DifferentialDriveRobot& robot);
  void bumpStart(int leftPwmOut, int rightPwmOut,
                 DifferentialDriveRobot& robot);
  void incrementPwm(int& leftPwmOut, int& rightPwmOut,
                    DifferentialDriveRobot& robot);
  void capPwmOutputs(int& leftPwmOut, int& rightPwmOut,
                     DifferentialDriveRobot& robot);
  void sendPwmSignals(int leftPwmOut, int rightPwmOut);

  double getLeftVelocity() const;
  double getRightVelocity() const;

 private:
  EncoderReader& encoderReader_;

  const int PWM_INCREMENT_;
  const int MIN_PWM_;
  const int MAX_PWM_;
  const int MAX_TURN_PWM_;
  const int LEFT_PWM_PIN_;
  const int LEFT_PWM_FREQ_;
  const int RIGHT_PWM_PIN_;
  const int RIGHT_PWM_FREQ_;
  const int LEFT_MOTOR_FWD_PIN_;
  const int LEFT_MOTOR_REV_PIN_;
  const int RIGHT_MOTOR_FWD_PIN_;
  const int RIGHT_MOTOR_REV_PIN_;

  int pi_;

  int pigpioSetup();
};

#endif  // PRACTICAL_NAV_INCLUDE_PRACTICAL_NAV_MOTOR_CONTROLLER_H_
