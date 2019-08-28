#include "wheel.h"
#include "periph.h"
#include "pid.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>


static PID_typedef  WheelPID[2];
#define WheelLPID WheelPID[0]
#define WheelRPID WheelPID[1]


#define X800
// Encoder count(10x) per one revolution of the wheel
#define WHEEL_REVOLUTION_COUNT_100X 112410
#ifdef X800
// 70mm * pi = ~219.91mm
#define WHEEL_CIRCUMFERENCE_MM_100X 21991
#elif defined(X600)
// 65mm * pi = ~204.1mm
#define WHEEL_CIRCUMFERENCE_MM_100X 20410
#endif
#define a_dt 0.001f


static __IO int32_t g_ActulWheelLVel = 0;
static __IO int32_t g_ActulWheelRVel = 0;


static int8_t WheelDir[2] = {1,1};
static void calc_speed(void)
{
  g_ActulWheelLVel = WheelDir[0]*g_fbWheelLCnt *20;
  g_ActulWheelRVel = WheelDir[1]*g_fbWheelRCnt *20;
  g_fbWheelLCnt = 0;
  g_fbWheelRCnt = 0;
  
  
//    //const uint32_t encoder_count = curr_encoder_countl -prev_encoder_countl;
//    uint32_t encoder_count = curr_encoder_countl;
//    curr_encoder_countl = 0;
//    //prev_encoder_countl = curr_encoder_countl;
//    uint32_t dist_mm_100x = (encoder_count * 100)
//        * WHEEL_CIRCUMFERENCE_MM_100X / WHEEL_REVOLUTION_COUNT_100X;
//    ActualSpeedl = dist_mm_100x *0.5f;
//    if (directionl == 0)
//        ActualSpeedl = -ActualSpeedl;
//
//    //encoder_count = curr_encoder_countr -prev_encoder_countr;
//    encoder_count = curr_encoder_countr;
//    curr_encoder_countr = 0;
//    dist_mm_100x = (encoder_count * 100)
//        * WHEEL_CIRCUMFERENCE_MM_100X / WHEEL_REVOLUTION_COUNT_100X;
//    ActualSpeedr = dist_mm_100x *0.5f;
//    if (directionr == 0)
//        ActualSpeedr = -ActualSpeedr;
    
    
  
  
    
}
static __IO float gCmdMsgVx = 0.0f;
static __IO float gCmdMsgWz = 0.0f;
static __IO float g_ActulVx = 0.0f;
static __IO float g_ActulWz = 0.0f;



void SetWheelGeometry(float v,float w)
{
  gCmdMsgVx = v;
  gCmdMsgWz = w;
}
void GetWheelGeometry(float* v,float* w)
{
    v[0] = g_ActulVx;
    w[0] = g_ActulWz;
}
void GetWheelVel(float  * vl,float* vr)
{
  
  vl[0]  = g_ActulWheelLVel;
  vr[0] =  g_ActulWheelRVel;
}


//void servo_cb(const geometry_msgs::Twist& cmd_msg)
//{
//    gCmdMsgVx = cmd_msg.linear.x;
//    gCmdMsgWz = cmd_msg.angular.z;
//}


#define WHEELSITANSE 220   //mm/s   //22cm
#define WHWWLRPWM_BASE  0X25
#define WHWWLLPWM_BASE  0X25
void ControlServo(void)
{
    float l = WHEELSITANSE; //cm;
    float vr = 0.0f;
    float vl = 0.0f;
    static float CmdMsgVx = 0;
    static float CmdMsgWz = 0;
    CmdMsgWz = gCmdMsgWz;
    if (fabs(gCmdMsgVx- CmdMsgVx)>5.0f){
        float out = (gCmdMsgVx-CmdMsgVx)*0.15f;
        if(out>=100.f)out = 100.f;
        else if(out<=-100.f)out =-100.f;
        CmdMsgVx += out;
    }
    if(gCmdMsgWz == 0.0f){
        if(fabs(CmdMsgVx)<6.0f){
            CmdMsgVx = 0;
        }
    }
    calc_speed();
    if (fabs(CmdMsgWz) <= 0.5f) {
        vl = CmdMsgVx;
        vr = CmdMsgVx;
    } else {
        vr = CmdMsgWz * l * 0.0175 + 2 * CmdMsgVx;
        vr = vr * 0.5f;
        vl = CmdMsgVx * 2.0f - vr;
    }
    
    int32_t fbcnt[2] = { 
          g_ActulWheelLVel,
          g_ActulWheelRVel
    };
    float ControExp[2] = {vl,vr};
    float out[2] = {0};
    for (int i = 0; i < 2; i++)  {
      float exp =  ControExp[i];
      float m =  fbcnt[i];
      out[i] = GetPid(&WheelPID[i], m, exp);
      if (out[i] < 0) {
        WheelDir[i] = -1;
      } else {
        WheelDir[i] = 1;
      } 
    }
    float wheelpwm[2] = {0};
    
    for(int i = 0;i<2;i++){
      if(fabs(out[i])<WHWWLLPWM_BASE){
        WheelPID[i]._iter = 0;
        wheelpwm[i] = 0;
      }else{
        if(WheelDir[i]<0){
          wheelpwm[i] = -out[i]+WHWWLLPWM_BASE;
        }else{
          wheelpwm[i] =  out[i] +WHWWLLPWM_BASE;
        }
      }
    }
    SetWheelLDir((WheelDir[0]!=-1)? 1:0);
    SetWheelRDir((WheelDir[1]!=-1)? 1:0);
    SetWheelLPwm(wheelpwm[0]);
    SetWheelRPwm(wheelpwm[1]);   
}
void WheelInit(void)
{
    memset(WheelPID,0,sizeof(WheelPID));
    WheelLPID._alph = 0.7;
    WheelLPID._imax = 1999-WHWWLRPWM_BASE;
    WheelLPID._imin = -(1999-WHWWLRPWM_BASE);
    WheelLPID._omax = 1999-WHWWLRPWM_BASE;;
    WheelLPID._omin =-(1999-WHWWLRPWM_BASE);
    SetPidPra(&WheelLPID, 0.08, 0.03, 0.02);
    WheelRPID._alph = 0.7;
    WheelRPID._imax = 1999-WHWWLRPWM_BASE;
    WheelRPID._imin = -(1999-WHWWLRPWM_BASE);
    WheelRPID._omax = 1999-WHWWLRPWM_BASE;
    WheelRPID._omin = -(1999-WHWWLRPWM_BASE);
    SetPidPra(&WheelRPID, 0.08, 0.03, 0.02);
}
