/*
*simple_diff_drive.cpp
*
*This is a simple differential drive  ROS node that accepts cmd_vel messages
*and outputs PWM and direction pin signals to an L298 dual motor driver.
*This is intended to be simple to read and implement, but lacks the robustness of a PID
*Robot prioritizes angular velocities (turns) and stops forward motion while turning.
*
*Rather than a PID to maintain straight driving, a simpler check of the average drift
*over several cycles is applied to a multiplier to modify the PWM to each wheel
*This DRIFT_MULTIPLIER might need to be tweaked for individual wheel sets
*
*Pin numbers are as example project is wired in the book Practical Robotics in C++.
*
*The constants near the top should be "personalized" for your robot, but should be very close if using
*Roomba wheel modules as I did in the book project.
*
*Author: Lloyd Brombach (lbrombach2@gmail.com)
*11/7/2019
*/


#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16.h"
#include <iostream>
#include <pigpiod_if2.h>
#include <cstdlib>

const int LOOP_FREQ = 50;
const int PWM_INCREMENT =1; //the rate pwm out can change per cycle
const double ticksPerwheelRev = 254*2; //508.8; //not in use yet..just a reference for now
const double wheelRadius = .03575; // 55.18;
const double wheelBase = .224; //223.8375mm actually
const double TICKS_PER_M =3750;//1125*2;//or 2250 //1.1645; //1.365 is on hard floor. carpet avg is 1.1926. overall avg = 1.1645 1125.766 t/m
const int KP = 238;//238 orginal
const int DRIFT_MULTIPLIER =125;//125 original//621
const int TURN_PWM = 60;
const int MIN_PWM = 30;
const int MAX_PWM = 120;// original 120

// left encoder multiplier
const double L_ENC_MULT = 1;
const double L_MOTOR_COMP = 1.33;//compensate for left motor being under powered
const double L_MOTOR_COMP_REV = 1.37;//for reverse
const double L_MOTOR_COMP_RGTTURN = 1.1;
//left motor
const int PWM_L = 21;
const int MOTOR_L_FWD = 26;
const int MOTOR_L_REV = 13;
//right motor
const int PWM_R = 19;
const int MOTOR_R_FWD = 12;
const int MOTOR_R_REV = 20;

double leftVelocity = 0;
double rightVelocity = 0;
double leftPwmReq = 0;
double rightPwmReq = 0;
double lastCmdMsgRcvd = 0; //time of last command recieved


int pi =-1;

using namespace std;


void Calc_Left_Vel(const std_msgs::Int16& lCount)
{
static double lastTime = 0;
static int lastCount = 0;
int cycleDistance = (65535 + lCount.data - lastCount) % 65535;
if (cycleDistance > 10000)
    {
        cycleDistance=0-(65535 - cycleDistance);
    }
leftVelocity = (cycleDistance*L_ENC_MULT)/TICKS_PER_M/(ros::Time::now().toSec()-lastTime);
lastCount = lCount.data;
lastTime = ros::Time::now().toSec();
cout<<"LeftCount = "<<lCount.data<<endl;
cout<<"LeftCycleDistance = "<<cycleDistance<<endl;

}
void Calc_Right_Vel(const std_msgs::Int16& rCount)
{
static double lastTime = 0;
static int lastCount = 0;
int cycleDistance = (65535 + rCount.data - lastCount) % 65535;
if (cycleDistance > 10000)
    {
        cycleDistance=0-(65535 - cycleDistance);
    }
rightVelocity = cycleDistance/TICKS_PER_M/(ros::Time::now().toSec()-lastTime);
lastCount=rCount.data;
lastTime = ros::Time::now().toSec();
cout<<"RightCount = "<<rCount.data<<endl;
cout<<"RightCycleDistance = "<<cycleDistance<<endl;

}


void Set_Speeds(const geometry_msgs::Twist& cmdVel)
{
    lastCmdMsgRcvd = ros::Time::now().toSec();
    int b = (abs(cmdVel.linear.x) > .0478 &&abs(cmdVel.linear.x) < .082) ? 30 : 40;
    
    //int b = (cmdVel.linear.x > .025 && cmdVel.linear.x < .052) ? 45 : 40;
    double cmdVelEpsilon =abs(0.1*cmdVel.linear.x);
//    if((leftVelocity==0 && rightVelocity==0))// || \
//       abs(leftVelocity-rightVelocity)>0.01)
//    {
//     leftPwmReq  =KP*cmdVel.linear.x+b;
//     rightPwmReq =KP*cmdVel.linear.x+b;
//     cout<<"iniitial pwm request (left and right same) = "<< leftPwmReq<<endl;
//    }
   // else if(leftVelocity!=0&&rightVelocity!=0 && abs(leftVelocity-rightVelocity)<=0.01)
   // {
   //  leftPwmReq = leftPwmReq;
   //  rightPwmReq = rightPwmReq;
   // }
    if(cmdVel.angular.z != 0)
    {
        if(cmdVel.angular.z > .0 )//standard gentle turn
        {
         leftPwmReq = -L_MOTOR_COMP_REV*MIN_PWM;
         rightPwmReq = MIN_PWM;
        }
        else if(cmdVel.angular.z< 0)
        {
         leftPwmReq =   L_MOTOR_COMP_RGTTURN*MIN_PWM;
         rightPwmReq = -MIN_PWM;//-(1/L_MOTOR_COMP)*TURN_PWM;
        }
        if( abs(cmdVel.angular.z>.12))//turn a little faster if angle is greater that .12
        {
          leftPwmReq  *= 1.1;
          rightPwmReq *= 1.1;
        }

    }
    else if(abs(cmdVel.linear.x) > .0478 ) // should equal about pwmval of 50, this is for going straight
    {
    // if(leftVelocity==0 && rightVelocity==0)
    // {
     if(cmdVel.linear.x>0)
     {
      leftPwmReq  =KP*L_MOTOR_COMP*cmdVel.linear.x+b;
      rightPwmReq =KP*(1.0/L_MOTOR_COMP)*cmdVel.linear.x+b;
     }
     else if(cmdVel.linear.x<0)
     {
      leftPwmReq  =KP*L_MOTOR_COMP_REV*cmdVel.linear.x-b;
      rightPwmReq =KP*(1.0/L_MOTOR_COMP_REV)*cmdVel.linear.x-b;
     }
     cout<<"iniitial pwm request (left and right same) = "<< leftPwmReq<<endl;
    // }
     static double prevDiff = 0;
     static double prevPrevDiff = 0;
     static double prevAvgAngularDiff = 0;
     double angularVelDifference = leftVelocity - rightVelocity; //how much faster one wheel is actually turning
     double avgAngularDiff = (prevDiff+prevPrevDiff+angularVelDifference)/3; //average several cycles

     cout<<"prev_diff = "<<prevDiff<<endl;
     cout<<"prev_prev_diff = "<<prevPrevDiff<<endl;
     cout<<"angular_velocity_diff = "<<angularVelDifference<<endl;
     cout<<"ang_vel_diff = "<<abs(avgAngularDiff)<<endl;

     prevPrevDiff=prevDiff;
     prevDiff = angularVelDifference;
    // prevAvgAngularDiff = avgAngularDiff;

     //apply corrective offset to each wheel to try and go straight
    if(abs(avgAngularDiff)>cmdVelEpsilon)//added 6thDec2023 to try and limit overcorrections
    {
     cout<<"in loop: "<<endl;
     cout<<"ang_vel_diff = "<<abs(avgAngularDiff)<<endl;
     cout<<"leftPwmReq_before_inc = "<<leftPwmReq<<endl;
     cout<<"rightPwmReq_before_inc = "<<rightPwmReq<<endl;

     leftPwmReq -= (int)(avgAngularDiff*DRIFT_MULTIPLIER);// -ve because left motor generally turns slower, therefore angVelDiff -ve, therefore -(-ve) actually adds to left to balance and += decreases right
     rightPwmReq += (int)(avgAngularDiff*DRIFT_MULTIPLIER);

//      cout<<"ang_vel_diff = "<<abs(avgAngularDiff)<<endl;
     cout<<"leftPwmReq_after_inc = "<<leftPwmReq<<endl;
     cout<<"rightPwmReq_after_inc = "<<rightPwmReq<<endl;

     prevAvgAngularDiff = avgAngularDiff;

    }
    else if(abs(avgAngularDiff)<=cmdVelEpsilon)
     {
      leftPwmReq -=(int)(prevAvgAngularDiff*DRIFT_MULTIPLIER);
      rightPwmReq+= (int)(prevAvgAngularDiff*DRIFT_MULTIPLIER);

     }
   //  prevAvgAngularDiff = avgAngularDiff;
    }

    //if(cmdVel.angular.z>

    //don't PWM values that don't do anything
    if(abs(leftPwmReq)< MIN_PWM)
    {
     leftPwmReq=0;
    }

    if(abs(rightPwmReq)<MIN_PWM)
    {
     rightPwmReq=0;
    }
    ////left for debugging and tweaking
    cout<<"CMD_VEL = "<<cmdVel.linear.x<<endl;
    cout<<"ANG_VEL = "<<cmdVel.angular.z<<endl;
    cout<<"VEL, AND PWM REQ LEFT AND RIGHT "<<leftVelocity<<"    "<<leftPwmReq<<" ..... ";
    cout<<rightVelocity<<"    "<<rightPwmReq<<endl;

}


void set_pin_values()
{
    static int leftPwmOut = 0;
    static int rightPwmOut = 0;

    //if PwmReq*PwmOut is negative, that means the wheel is switching
    //directions and we should bring to a stop before switching directions
    static bool stopped = false;
    if( (leftPwmReq*leftVelocity < 0 && leftPwmOut != 0)
        || (rightPwmReq*rightVelocity < 0 && rightPwmOut != 0))
    {
        cout<<"Resetting pwms to 0"<<endl;
        leftPwmReq = 0;
        rightPwmReq = 0;
    }


    //set motor driver direction pins
    if(leftPwmReq>0) //left fwd
    {
     gpio_write(pi, MOTOR_L_REV, 1);
     gpio_write(pi, MOTOR_L_FWD, 0);
    }
    else if(leftPwmReq<0) //left rev
    {
     gpio_write(pi, MOTOR_L_FWD, 1);
     gpio_write(pi, MOTOR_L_REV, 0);
    }
    else if (leftPwmReq == 0 && leftPwmOut == 0 ) //left stop
    {
     gpio_write(pi, MOTOR_L_FWD, 1);
     gpio_write(pi, MOTOR_L_REV, 1);
    }

    if(rightPwmReq>0 )//right fwd
    {
     gpio_write(pi, MOTOR_R_REV, 1);
     gpio_write(pi, MOTOR_R_FWD, 0);
    }
    else if(rightPwmReq<0) //right rev
    {
     gpio_write(pi, MOTOR_R_FWD, 1);
     gpio_write(pi, MOTOR_R_REV, 0);
    }
    else if (rightPwmReq == 0 && rightPwmOut == 0)
    {
     gpio_write(pi, MOTOR_R_FWD, 1);
     gpio_write(pi, MOTOR_R_REV, 1);
    }


    //bump up pwm if robot is having trouble starting from stopped
/*    if((leftPwmReq != 0 && leftVelocity ==0) || (rightPwmReq != 0 && leftVelocity == 0))
    {
     leftPwmReq *= 1.2;
     rightPwmReq *= 1.2;
    }
    */
    
    if(  leftPwmReq != 0 && leftVelocity == 0)
    {
        leftPwmReq *= 1.4;
    }
    if( rightPwmReq != 0 && rightVelocity == 0)
    {
        rightPwmReq *= 1.4;
    }
    

    //this section increments PWM changes instead of jarring/dangeroud sudden big changes
    if (abs(leftPwmReq) > leftPwmOut)
    {
     leftPwmOut += PWM_INCREMENT;
    }
    else if (abs(leftPwmReq) < leftPwmOut)
    {
     leftPwmOut -= PWM_INCREMENT;
    }
    if (abs(rightPwmReq) > rightPwmOut)
    {
     rightPwmOut += PWM_INCREMENT;
    }
    else if(abs(rightPwmReq) < rightPwmOut)
    {
     rightPwmOut -= PWM_INCREMENT;
    }

    if((leftPwmReq<0)^(rightPwmReq<0))
    {
     leftPwmOut = (leftPwmOut>TURN_PWM) ? TURN_PWM : leftPwmOut;
     rightPwmOut = (rightPwmOut>TURN_PWM) ? TURN_PWM : rightPwmOut;
    }
    //cap output at max defined in constants
    leftPwmOut = (leftPwmOut>MAX_PWM) ? MAX_PWM : leftPwmOut;
    rightPwmOut = (rightPwmOut>MAX_PWM) ? MAX_PWM : rightPwmOut;

    //limit output to a low of zero
    leftPwmOut = (leftPwmOut< 0 ) ? 0 : leftPwmOut;
    rightPwmOut = (rightPwmOut< 0) ? 0 : rightPwmOut;

    //write the pwm values tot he pins
    set_PWM_dutycycle(pi, PWM_L, leftPwmOut);
    set_PWM_dutycycle(pi, PWM_R, rightPwmOut);

    cout<<"PWM OUT LEFT AND RIGHT            "<<leftPwmOut<<"           "<<rightPwmOut<<endl<<endl;
}


int PigpioSetup()
{
    char *addrStr = NULL;
    char *portStr = NULL;
    pi = pigpio_start(addrStr, portStr);
    //next 10 lines sets up our pins. Remember that high is "off"
    //and we must drive in1 or in2 low to start the output to motor
    set_mode(pi,PWM_L, PI_OUTPUT);
    set_mode(pi,MOTOR_L_FWD, PI_OUTPUT);
    set_mode(pi,MOTOR_L_REV, PI_OUTPUT);
    set_mode(pi,PWM_R, PI_OUTPUT);
    set_mode(pi,MOTOR_R_FWD, PI_OUTPUT);
    set_mode(pi,MOTOR_R_REV, PI_OUTPUT);

    gpio_write(pi, MOTOR_L_FWD, 1); //initializes motor off
    gpio_write(pi, MOTOR_L_REV, 1); //initializes motor off
    gpio_write(pi, MOTOR_R_FWD, 1); //initializes motor off
    gpio_write(pi, MOTOR_R_REV, 1); //initializes motor off

    return pi;
}

int main(int argc, char **argv)
{
    //initialize pipiod interface
    int pi = PigpioSetup();
    if(PigpioSetup()>=0)
    {
     cout<<"daemon interface started ok at "<<pi<<endl;
    }
    else
    {
     cout<<"Failed to connect to PIGPIO Daemon - is it running?"<<endl;
     return -1;
    }
    ////////////////

    int f_pwmr = set_PWM_frequency(pi, PWM_R, 100);  // Set PWM frequency to 4000 Hz
    if (f_pwmr < 0) {
    // Handle error
     printf("Error setting PWM_R frequency: %d\n", f_pwmr);
    }

    int f_pwml = set_PWM_frequency(pi, PWM_L, 100);  // Set PWM frequency to$
    if (f_pwml < 0) {
    // Handle error
     printf("Error setting PWM_L frequency: %d\n", f_pwml);
    }

    ////////////////////

    ros::init(argc, argv, "simple_diff_drive");
    ros::NodeHandle node;

    //Subscribe to topics
    ros::Subscriber subForRightCounts = node.subscribe("rightWheel", 1000, Calc_Right_Vel,ros::TransportHints().tcpNoDelay());
    ros::Subscriber subForLeftCounts = node.subscribe("leftWheel",1000, Calc_Left_Vel,ros::TransportHints().tcpNoDelay());
    ros::Subscriber subForVelocity = node.subscribe("cmd_vel", 1, Set_Speeds,ros::TransportHints().tcpNoDelay());

    ros::Rate loop_rate(LOOP_FREQ);//think this may need to be multiple of 10 to avoid conflict with motor driver signals?
    while(ros::ok())
    {
     ros::spinOnce();
     cout<<"leftPwmReq "<<leftPwmReq<<endl;
     cout<<"rightPwmReq "<<rightPwmReq<<endl;

     //stop motors if no cmd_vel msgs recieved
     if(ros::Time::now().toSec() - lastCmdMsgRcvd > 1)
     {
      cout<<"NOT RECIEVING CMD_VEL - STOPPING MOTORS  --  time sincel last = "<<ros::Time::now().toSec() - lastCmdMsgRcvd<<endl;
      leftPwmReq = 0;
      rightPwmReq = 0;
     }

     set_pin_values();
     cout<<"leftPwmReq after set pins"<<leftPwmReq<<endl;
     cout<<"rightPwmReq after set pins"<<rightPwmReq<<endl;
     loop_rate.sleep();
    }

    gpio_write(pi, MOTOR_L_FWD, 1); //initializes motor off
    gpio_write(pi, MOTOR_L_REV, 1); //initializes motor off
    gpio_write(pi, MOTOR_R_FWD, 1); //initializes motor off
    gpio_write(pi, MOTOR_R_REV, 1); //initializes motor off

    return 0;
}
