/* *simple_diff_drive.cpp * *This is a simple differential drive  ROS node that accepts cmd_vel messages *and outputs PWM and direction pin signals to an L298 dual motor driver.
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

const int ENCODER_RANGE = 65535;//book says 65535...16 bit range
const int LOOP_FREQ = 100;//must align with rqt steering cmd vel freq
const int PWM_INCREMENT =1; //the rate pwm out can change per cycle
const double ticksPerwheelRev = 254*2; //508.8; //not in use yet..just a reference for now
const double wheelRadius = .03575; // 55.18;
const double wheelBase = .224; //223.8375mm actually
const double TICKS_PER_M =2270;//1125*2;//or 2250 //1.1645; //1.365 is on hard floor. carpet avg is 1.1926. overall avg = 1.1645 1125.766 t/m
const int KP = 20;//238 orginal
const int DRIFT_MULTIPLIER =250;//125;// original//621
const int TURN_PWM = 34;
const int MAX_TURN_PWM = 85;
const int MIN_PWM = 25;
const int MAX_PWM = 90;// original 120
const double VEL_MIN = 0.055;//0.0478;
//const double ANG_MIN = 0.02;
//const double MAX_VEL_DIFF = 0;

// left encoder multiplier
const double L_ENC_MULT = 1;
const double L_MOTOR_COMP = 0.9;//.33;//1.33compensate for left motor being under powered
const double L_MOTOR_COMP_REV = 1;//.37;//for reverse
const double L_MOTOR_COMP_TURN = 1;//.1;
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
int cycleDistance = 0;
//cout<<"lastCountLeft = "<<lastCount<<endl;
cycleDistance = (ENCODER_RANGE + lCount.data - lastCount) % ENCODER_RANGE;
if (cycleDistance > 10000)
    {
        cycleDistance=0-(ENCODER_RANGE - cycleDistance);
    }
//cout<<"time diff left = "<<ros::Time::now().toSec()-lastTime<<endl;
leftVelocity = cycleDistance/TICKS_PER_M/(ros::Time::now().toSec()-lastTime);
lastCount = lCount.data;
lastTime = ros::Time::now().toSec();
cout<<"LeftCount = "<<lCount.data<<endl;
cout<<"LeftCycleDistance = "<<cycleDistance<<endl;

}
void Calc_Right_Vel(const std_msgs::Int16& rCount)
{
static double lastTime = 0;
static int lastCount = 0;
//cout<<"lastCountRight = "<<lastCount<<endl;
int cycleDistance = (ENCODER_RANGE + rCount.data - lastCount) % ENCODER_RANGE;
if (cycleDistance > 10000)
    {
        cycleDistance=0-(ENCODER_RANGE - cycleDistance);
    }
//cout<<"time diff right = "<<ros::Time::now().toSec()-lastTime<<endl;
rightVelocity = cycleDistance/TICKS_PER_M/(ros::Time::now().toSec()-lastTime);
lastCount=rCount.data;
lastTime = ros::Time::now().toSec();
cout<<"RightCount = "<<rCount.data<<endl;
cout<<"RightCycleDistance = "<<cycleDistance<<endl;
}


void Set_Speeds(const geometry_msgs::Twist& cmdVel)
{
    lastCmdMsgRcvd = ros::Time::now().toSec();
    int b = (abs(cmdVel.linear.x) >VEL_MIN &&abs(cmdVel.linear.x) < .082) ? 30 : 40;
    //int b = (cmdVel.linear.x > .025 && cmdVel.linear.x < .052) ? 45 : 40;
    double cmdVelEpsilon =0.01;//abs(0.1*cmdVel.linear.x);
    double cmdAngVelEpsilon = 0.001;//abs(0.1*cmdVel.angular.z);

    if(abs(cmdVel.angular.z) > 0.01)
    {
        if(cmdVel.angular.z >= .01 )//standard gentle left turn
        {
         //leftPwmReq = -L_MOTOR_COMP_REV*TURN_PWM;
         leftPwmReq = -L_MOTOR_COMP_TURN*TURN_PWM;
         rightPwmReq = (1.0/L_MOTOR_COMP)*TURN_PWM;
        }
        else if(cmdVel.angular.z<-.01) 
        {
         leftPwmReq =   L_MOTOR_COMP_TURN*TURN_PWM;
         rightPwmReq = -(1.0/L_MOTOR_COMP)*TURN_PWM;
        }
        /*
        else
        {
          leftPwmReq = 0;
          rightPwmReq = 0;
        }
        */
        static double prevRotDiff = 0;
        static double prevPrevRotDiff = 0;
        static double prevAvgAngularRotDiff = 0;
        double angularVelRotDifference = leftVelocity + rightVelocity; //how much faster one wheel is actually turning
        double avgAngularRotDiff = (prevRotDiff+prevPrevRotDiff+angularVelRotDifference)/3; //average several cycles
/*        cout<<"prev_rot_diff = "<<prevRotDiff<<endl;
        cout<<"prev_prev_rot_diff = "<<prevPrevRotDiff<<endl;
        cout<<"angular_velocity_rot_diff = "<<angularVelRotDifference<<endl;
        cout<<"avg_vel_rot_diff = "<<avgAngularRotDiff<<endl;
*/
        prevPrevRotDiff=prevRotDiff;
        prevRotDiff = angularVelRotDifference;
        
        if(abs(avgAngularRotDiff)>cmdAngVelEpsilon)
        {
 /*        cout<<"in control loop: "<<endl;
         cout<<"avg_vel_rot_diff = "<<abs(avgAngularRotDiff)<<endl;
         cout<<"leftPwmReq_before_inc = "<<leftPwmReq<<endl;
         cout<<"rightPwmReq_before_inc = "<<rightPwmReq<<endl;
*/
         leftPwmReq -= (int)(avgAngularRotDiff*DRIFT_MULTIPLIER);
         rightPwmReq -= (int)(avgAngularRotDiff*DRIFT_MULTIPLIER);

  //       cout<<"leftPwmReq_after_inc = "<<leftPwmReq<<endl;
  //       cout<<"rightPwmReq_after_inc = "<<rightPwmReq<<endl;

         prevAvgAngularRotDiff = avgAngularRotDiff;

        }
       else if(abs(avgAngularRotDiff)<=cmdAngVelEpsilon)
       {
         leftPwmReq -= (int)(prevAvgAngularRotDiff*DRIFT_MULTIPLIER);
         rightPwmReq-= (int)(prevAvgAngularRotDiff*DRIFT_MULTIPLIER);
       }
    }
    
    else if(abs(cmdVel.linear.x) <= VEL_MIN)
    {
     leftPwmReq = 0;
     rightPwmReq = 0;
    }
    else if(abs(cmdVel.linear.x) >VEL_MIN ) // .0478 
    {
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

     static double prevDiff = 0;
     static double prevPrevDiff = 0;
     static double prevAvgAngularDiff = 0;

     double angularVelDifference = leftVelocity - rightVelocity; 
     double avgAngularDiff = (prevDiff+prevPrevDiff+angularVelDifference)/3; 

     cout<<"prev_diff = "<<prevDiff<<endl;
     cout<<"prev_prev_diff = "<<prevPrevDiff<<endl;
     cout<<"angular_velocity_diff = "<<angularVelDifference<<endl;
     cout<<"avg_vel_diff = "<<abs(avgAngularDiff)<<endl;

     prevPrevDiff=prevDiff;
     prevDiff = angularVelDifference;
     
    if(abs(avgAngularDiff)>cmdVelEpsilon)
    {
     cout<<"in linear control loop: "<<endl;
     cout<<"ang_vel_diff = "<<abs(avgAngularDiff)<<endl;
     cout<<"leftPwmReq_before_inc = "<<leftPwmReq<<endl;
     cout<<"rightPwmReq_before_inc = "<<rightPwmReq<<endl;

     leftPwmReq -= (int)(avgAngularDiff*DRIFT_MULTIPLIER); 
     rightPwmReq += (int)(avgAngularDiff*DRIFT_MULTIPLIER);

     cout<<"leftPwmReq_after_inc = "<<leftPwmReq<<endl;
     cout<<"rightPwmReq_after_inc = "<<rightPwmReq<<endl;

     prevAvgAngularDiff = avgAngularDiff;

    }
    else if(abs(avgAngularDiff)<=cmdVelEpsilon)
     {
      leftPwmReq -=(int)(prevAvgAngularDiff*DRIFT_MULTIPLIER);
      rightPwmReq+= (int)(prevAvgAngularDiff*DRIFT_MULTIPLIER);

     }
    }

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
/*    cout<<"VEL, AND PWM REQ LEFT AND RIGHT "<<leftVelocity<<"    "<<leftPwmReq<<" ..... ";
    cout<<rightVelocity<<"    "<<rightPwmReq<<endl;
*/
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
     cout<< "leftPwmReq : "<<leftPwmReq<<endl;
     cout<< "left forward"<<endl;
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
/*
    const double epsilon = 1e-6;

    //bump up pwm if robot is having trouble starting from stopped
    if((leftPwmReq != 0 && abs(leftVelocity)<epsilon) || (rightPwmReq != 0 && abs(rightVelocity) <epsilon))
    {
     leftPwmReq *= 1.2;
     rightPwmReq *= 1.2;
    }
  */  
    
    const double vel_eps = 1e-6;

    if(  leftPwmReq != 0 && (abs(leftVelocity) < vel_eps))
    {
      if(abs(leftPwmReq)<MAX_PWM&&leftPwmOut>=MIN_PWM)
      {
        leftPwmReq *= 1.4;
        cout<< "After bump leftpwmreq: "<< leftPwmReq << endl;
        cout<< "After bump leftpwmout: "<< leftPwmOut << endl;
      }
    }
    if( rightPwmReq != 0 && (abs(rightVelocity) < vel_eps)) 
    {
      if(abs(rightPwmReq)<MAX_PWM&&leftPwmOut>=MIN_PWM)
      {
        rightPwmReq *= 1.4;
        cout<< "After bump rightpwmreq: "<< rightPwmReq << endl;
        cout<< "After bump rightpwmout: "<< rightPwmOut << endl;

      }
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
/*
    if((leftPwmReq<0)^(rightPwmReq<0))
    {
     leftPwmOut = (leftPwmOut>MAX_TURN_PWM) ? MAX_TURN_PWM : leftPwmOut;
     rightPwmOut = (rightPwmOut>MAX_TURN_PWM) ? MAX_TURN_PWM : rightPwmOut;
    }
*/    
    //cap output at max defined in constants
    leftPwmOut = (leftPwmOut>MAX_PWM) ? MAX_PWM : leftPwmOut;
    rightPwmOut = (rightPwmOut>MAX_PWM) ? MAX_PWM : rightPwmOut;


    if((leftPwmOut<0)||(rightPwmOut<0))
    {
      cout<<"PwmOut values -ve"<<endl;
    }
    //limit output to a low of zero
    leftPwmOut = (leftPwmOut< 0 ) ? 0 : leftPwmOut;
    rightPwmOut = (rightPwmOut< 0) ? 0 : rightPwmOut;
/*
    if(leftPwmOut<leftPwmReq & rightPwmOut<rightPwmReq & leftPwmOut>=L_MOTOR_COMP*MIN_PWM)
    {
      //rightPwmOut = (1/L_MOTOR_COMP)*rightPwmOut;//when accelerating, balance pwm to account for left motor being stronger
      leftPwmOut = L_MOTOR_COMP*leftPwmOut;
    }
*/

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
    
    int f_pwmr = set_PWM_frequency(pi, PWM_R, 100);  // Set PWM frequency to 
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

    ros::Rate loop_rate(LOOP_FREQ);//
    while(ros::ok())
    {
     ros::spinOnce();
 //    cout<<"leftPwmReq "<<leftPwmReq<<endl;
 //    cout<<"rightPwmReq "<<rightPwmReq<<endl;

     //stop motors if no cmd_vel msgs recieved
     if(ros::Time::now().toSec() - lastCmdMsgRcvd > 1)
     {
   //   cout<<"NOT RECIEVING CMD_VEL - STOPPING MOTORS  --  time sincel last = "<<ros::Time::now().toSec() - lastCmdMsgRcvd<<endl;
      leftPwmReq = 0;
      rightPwmReq = 0;
     }
     
     cout<<"Before PIN write: MOTOR_L_FWD : "<<gpio_read(pi, MOTOR_L_FWD)<<endl;
     cout<<"Before PIN write: MOTOR_L_REV : "<<gpio_read(pi, MOTOR_L_REV)<<endl;
/*     cout<<"Before PIN write: PWM_L: "<< get_PWM_dutycycle(pi, PWM_L)<<endl; 
     cout<<"Before PIN write: MOTOR_R_FWD : "<<gpio_read(pi, MOTOR_R_FWD)<<endl;
     cout<<"Before PIN write: MOTOR_R_REV : "<<gpio_read(pi, MOTOR_R_REV)<<endl;
     cout<<"Before PIN write: PWM_R: "<< get_PWM_dutycycle(pi, PWM_R)<<endl; 
*/
     cout<<"left velocity before: "<<leftVelocity<<"  right velocity before: "<<rightVelocity<<endl;
     cout<<"leftPwmReq before set pins"<<leftPwmReq<<endl;
     cout<<"Before PIN write: PWM_L: "<< get_PWM_dutycycle(pi, PWM_L)<<endl; 

     cout<<"rightPwmReq before set pins"<<rightPwmReq<<endl;
     cout<<"Before PIN write: PWM_R: "<< get_PWM_dutycycle(pi, PWM_R)<<endl; 


     set_pin_values();
    
     cout<<"After PIN write: MOTOR_L_FWD : "<<gpio_read(pi, MOTOR_L_FWD)<<endl;
     cout<<"After PIN write: MOTOR_L_REV : "<<gpio_read(pi, MOTOR_L_REV)<<endl;
/*     cout<<"After PIN write: PWM_L: "<< get_PWM_dutycycle(pi, PWM_L)<<endl; 
     cout<<"After PIN write: MOTOR_R_FWD : "<<gpio_read(pi, MOTOR_R_FWD)<<endl;
     cout<<"After PIN write: MOTOR_R_REV : "<<gpio_read(pi, MOTOR_R_REV)<<endl;
     cout<<"After PIN write: PWM_R: "<< get_PWM_dutycycle(pi, PWM_R)<<endl; 
*/

     cout<<"left velocity after: "<<leftVelocity<<"  right velocity after: "<<rightVelocity<<endl;
     cout<<"leftPwmReq after set pins"<<leftPwmReq<<endl;
     cout<<"After PIN write: PWM_L: "<< get_PWM_dutycycle(pi, PWM_L)<<endl; 

     cout<<"rightPwmReq after set pins"<<rightPwmReq<<endl;
     cout<<"After PIN write: PWM_R: "<< get_PWM_dutycycle(pi, PWM_R)<<endl; 

     loop_rate.sleep();
    }

    gpio_write(pi, MOTOR_L_FWD, 1); //initializes motor off
    gpio_write(pi, MOTOR_L_REV, 1); //initializes motor off
    gpio_write(pi, MOTOR_R_FWD, 1); //initializes motor off
    gpio_write(pi, MOTOR_R_REV, 1); //initializes motor off
}
