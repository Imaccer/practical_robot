
#ifndef PRACTICAL_NAV_INCLUDE_PRACTICAL_NAV_DIFFERENTIAL_DRIVE_ROBOT_H
#define PRACTICAL_NAV_INCLUDE_PRACTICAL_NAV_DIFFERENTIAL_DRIVE_ROBOT_H 
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16.h"
#include <pigpiod_if2.h>

class DifferentialDriveRobot {
private:
    const int ENCODER_RANGE_ = 65535;
    const int LOOP_FREQ_ = 100;
    const int PWM_INCREMENT_ = 1;
    const double ticksPerwheelRev_ = 254*2; 
    const double wheelRadius_ = .03575; 
    const double wheelBase_ = .224; 
    const double TICKS_PER_M_ =2270;//1125*2;//or 2250 
    const int KP_ = 20;//238 orginal
    const int DRIFT_MULTIPLIER_ =250;//125 original
    const int TURN_PWM_ = 34;
    const int MAX_TURN_PWM_ = 85;
    const int MIN_PWM_ = 25;
    const int MAX_PWM_ = 90;// original 120
    const double VEL_MIN_ = 0.055;//0.0478;

    const double L_ENC_MULT_ = 1;
    const double L_MOTOR_COMP_ = 0.9;// compensate for left motor being over powered
    const double L_MOTOR_COMP_REV_ = 1;// for reverse
    const double L_MOTOR_COMP_TURN_ = 1;
    //left motor pin assignments
    const int PWM_L_ = 21;
    const int MOTOR_L_FWD_ = 26;
    const int MOTOR_L_REV_ = 13;
    //right motor pin assignments
    const int PWM_R_ = 19;
    const int MOTOR_R_FWD_ = 12;
    const int MOTOR_R_REV_ = 20;

    double leftVelocity_ = 0;
    double rightVelocity_ = 0;
    double leftPwmReq_ = 0;
    double rightPwmReq_ = 0;
    double lastCmdMsgRcvd_ = 0; 

    int pi_ =-1;

    ros::NodeHandle nh_;
    ros::Subscriber subForRightCounts_;
    ros::Subscriber subForLeftCounts_;
    ros::Subscriber subForVelocity_;

public:
    DifferentialDriveRobot();
    ~DifferentialDriveRobot();

    void pigpioSetup();
    void createSubscribers();
    void calculateLeftVelocity(const std_msgs::Int16& lCount);
    void calculateRightVelocity(const std_msgs::Int16& rCount);
    void setSpeeds(const geometry_msgs::Twist& cmdVel);
    void setPinValues();
    void run();
};

#endif // PRACTICAL_NAV_INCLUDE_PRACTICAL_NAV_DIFFERENTIAL_DRIVE_ROBOT_H 
