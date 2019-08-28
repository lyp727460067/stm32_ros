
#include "stm32f0xx.h"
#include "battery.h"

#include "stm32f0xx.h"
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int32MultiArray.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "geometry_msgs/Transform.h"
#include "rosnode.h"
#include "hardwareserial.h"

#include <math.h>
#include <stdio.h>

#include "imu.h"
#include "led.h"
#include "hw.h"
#include "brushandVuuc.h"
#include "periph.h"
#include "timer.h"
#include "wheel.h"

void delay(int n)
{
    for (int j = 0; j < n; j++) {
        for (int i = 0; i < 65536; i++)
            ;
    }
}
#define mnCmdVelLenth 7
void CmdVelCallBack(const geometry_msgs::Transform& cmdvel)
{

     float tmp[mnCmdVelLenth];
     tmp[0] = cmdvel.rotation.x;
     tmp[1] = cmdvel.rotation.y;
     tmp[2] = cmdvel.rotation.z;
     tmp[3] = cmdvel.rotation.w;

     tmp[4] = cmdvel.translation.x;
     tmp[5] = cmdvel.translation.y;
     tmp[6] = cmdvel.translation.z;

    //  memcpy(tmp,cmdvel.values,mnCmdVelLenth);
     int i =0;
     for( ;i<5;i++){
         SetBruAndVuucExpVel(i,tmp[i]);
     }
    
     SetWheelGeometry(tmp[i],tmp[i+1]);    
}

int main(void)
{
    HwInit();
    TimerInit();
   //
    ros::NodeHandle nh;
    initialise(); 
    nh.initNode();

    while (!nh.connected()) {
         nh.spinOnce();
    }
    nh.loginfo("ros is connect");
    //rosnode RosNode("VelFb_","VelCmd_","odom_",7,7);
 
    PeriphInit();
    WheelInit();
    BrushAndVuccInit();
    delay(100);
    uint16_t timer50mscnt = 0;
    uint16_t timer20mscnt = 0;
    float fbVel[7];
    
    sensor_msgs::ChannelFloat32 VelFeedBack;
    ros::Publisher   velfeedbackpub("VelFb_",&VelFeedBack);
    ros::Subscriber<geometry_msgs::Transform> cmdvelsub("VelCmd_",CmdVelCallBack);

    nh.advertise(velfeedbackpub);
    nh.subscribe(cmdvelsub);
    nh.spinOnce();
    //delay(100);
   // nh.loginfo("main thread start");
    while (1) {

        if (GetTimer1msFlag()) {
            SetTImerImsFlag(0);
            timer50mscnt++;
            timer20mscnt++;
            if (timer50mscnt >= 50) {
                
                timer50mscnt = 0;
                BrushAndVuucControl();
                for(int i = 0;i<5;i++){
                    fbVel[i] = GetBruAndVuucActulVel(i);
                }
            }
            if (timer20mscnt >= 20) {
                timer20mscnt = 0;
                ControlServo();
                GetWheelVel(&fbVel[5],&fbVel[6]);
                VelFeedBack.name = "r";
                VelFeedBack.values = fbVel;
                VelFeedBack.values_length = 7;
                velfeedbackpub.publish(&VelFeedBack);
              
            }
            nh.spinOnce();
        }
       
    }
}
