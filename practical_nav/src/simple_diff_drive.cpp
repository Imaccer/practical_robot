
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16.h"
#include <iostream>
#include <pigpiod_if2.h>
#include <cstdlib>

const int ENCODER_RANGE = 65535;
const int LOOP_FREQ = 100;
const int PWM_INCREMENT =1; //the rate pwm out can change per cycle
const double ticksPerwheelRev = 254*2; //508.8; //not in use yet..just a reference for now
const double wheelRadius = .03575; // 55.18;
const double wheelBase = .224; //223.8375mm actually
const double TICKS_PER_M =2270;//1125*2;//or 2250 
const int KP = 20;//238 orginal
const int DRIFT_MULTIPLIER =250;//125 original
const int TURN_PWM = 34;
const int MAX_TURN_PWM = 85;
const int MIN_PWM = 25;
const int MAX_PWM = 90;// original 120
const double VEL_MIN = 0.055;//0.0478;

const double L_ENC_MULT = 1;
const double L_MOTOR_COMP = 0.9;// compensate for left motor being over powered
const double L_MOTOR_COMP_REV = 1;// for reverse
const double L_MOTOR_COMP_TURN = 1;
//left motor pin assignments
const int PWM_L = 21;
const int MOTOR_L_FWD = 26;
const int MOTOR_L_REV = 13;
//right motor pin assignments
const int PWM_R = 19;
const int MOTOR_R_FWD = 12;
const int MOTOR_R_REV = 20;

double leftVelocity = 0;
double rightVelocity = 0;
double leftPwmReq = 0;
double rightPwmReq = 0;
double lastCmdMsgRcvd = 0; 

int pi =-1;

using namespace std;


void Calc_Left_Vel(const std_msgs::Int16& lCount)
{
static double lastTime = 0;
static int lastCount = 0;
int cycleDistance = 0;
cycleDistance = (ENCODER_RANGE + lCount.data - lastCount) % ENCODER_RANGE;
if (cycleDistance > 10000)
    {
        cycleDistance=0-(ENCODER_RANGE - cycleDistance);
    }
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
int cycleDistance = (ENCODER_RANGE + rCount.data - lastCount) % ENCODER_RANGE;
if (cycleDistance > 10000)
    {
        cycleDistance=0-(ENCODER_RANGE - cycleDistance);
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
    int b = (abs(cmdVel.linear.x) >VEL_MIN &&abs(cmdVel.linear.x) < .082) ? 30 : 40;
    //int b = (cmdVel.linear.x > .025 && cmdVel.linear.x < .052) ? 45 : 40;
    double cmdVelEpsilon =0.01;
    double cmdAngVelEpsilon = 0.001;

    if(abs(cmdVel.angular.z) > 0.01)
    {
        if(cmdVel.angular.z >= .01 )//standard gentle left turn
        {
         leftPwmReq = -L_MOTOR_COMP_TURN*TURN_PWM;
         rightPwmReq = (1.0/L_MOTOR_COMP)*TURN_PWM;
        }
        else if(cmdVel.angular.z<-.01) 
        {
         leftPwmReq =   L_MOTOR_COMP_TURN*TURN_PWM;
         rightPwmReq = -(1.0/L_MOTOR_COMP)*TURN_PWM;
        }

        static double prevRotDiff = 0;
        static double prevPrevRotDiff = 0;
        static double prevAvgAngularRotDiff = 0;
        double angularVelRotDifference = leftVelocity + rightVelocity; //how much faster one wheel is actually turning
        double avgAngularRotDiff = (prevRotDiff+prevPrevRotDiff+angularVelRotDifference)/3; //average several cycles
        prevPrevRotDiff=prevRotDiff;
        prevRotDiff = angularVelRotDifference;
        
        if(abs(avgAngularRotDiff)>cmdAngVelEpsilon)
        {
         leftPwmReq -= (int)(avgAngularRotDiff*DRIFT_MULTIPLIER);
         rightPwmReq -= (int)(avgAngularRotDiff*DRIFT_MULTIPLIER);

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
    else if(abs(cmdVel.linear.x) >VEL_MIN ) 
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
    cout<<"CMD_VEL = "<<cmdVel.linear.x<<endl;
    cout<<"ANG_VEL = "<<cmdVel.angular.z<<endl;
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
    
    int f_pwmr = set_PWM_frequency(pi, PWM_R, 100);  
    if (f_pwmr < 0) {
     printf("Error setting PWM_R frequency: %d\n", f_pwmr);
    }

    int f_pwml = set_PWM_frequency(pi, PWM_L, 100);  
    if (f_pwml < 0) {
     printf("Error setting PWM_L frequency: %d\n", f_pwml);
    }

    ros::init(argc, argv, "simple_diff_drive");
    ros::NodeHandle node;

    ros::Subscriber subForRightCounts = node.subscribe("rightWheel", 1000, Calc_Right_Vel,ros::TransportHints().tcpNoDelay());
    ros::Subscriber subForLeftCounts = node.subscribe("leftWheel",1000, Calc_Left_Vel,ros::TransportHints().tcpNoDelay());
    ros::Subscriber subForVelocity = node.subscribe("cmd_vel", 1, Set_Speeds,ros::TransportHints().tcpNoDelay());

    ros::Rate loop_rate(LOOP_FREQ);//
    while(ros::ok())
    {
     ros::spinOnce();

     if(ros::Time::now().toSec() - lastCmdMsgRcvd > 1)
     {
      leftPwmReq = 0;
      rightPwmReq = 0;
     }
     
     cout<<"Before PIN write: MOTOR_L_FWD : "<<gpio_read(pi, MOTOR_L_FWD)<<endl;
     cout<<"Before PIN write: MOTOR_L_REV : "<<gpio_read(pi, MOTOR_L_REV)<<endl;
     cout<<"left velocity before: "<<leftVelocity<<"  right velocity before: "<<rightVelocity<<endl;
     cout<<"leftPwmReq before set pins"<<leftPwmReq<<endl;
     cout<<"Before PIN write: PWM_L: "<< get_PWM_dutycycle(pi, PWM_L)<<endl; 

     cout<<"rightPwmReq before set pins"<<rightPwmReq<<endl;
     cout<<"Before PIN write: PWM_R: "<< get_PWM_dutycycle(pi, PWM_R)<<endl; 


     set_pin_values();
    
     cout<<"After PIN write: MOTOR_L_FWD : "<<gpio_read(pi, MOTOR_L_FWD)<<endl;
     cout<<"After PIN write: MOTOR_L_REV : "<<gpio_read(pi, MOTOR_L_REV)<<endl;
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
