
#include "battery.h"
#include "geometry_msgs/Twist.h"
#include "hardwareserial.h"
#include "imu.h"
#include "led.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "stm32f0xx.h"
#include "tf/tf.h"
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <ros.h>
#include <ros/time.h>
#include <ros_arduino_msgs/RawImu.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include "std_msgs/UInt8.h"
//#include <ros_arduino_msgs/ros_arduino_msgs.h>
#define DEBUG_RATE 100

bool is_first = true;

Led led;
void led_cb(const std_msgs::Float64& cmd_msg)
{
    static bool stat_led = true;
    if (stat_led) {
        led.on_off(ON);
        stat_led = false;
    } else {
        led.on_off(OFF);
        stat_led = true;
    }
}
void delay(int n)
{
    for (int j = 0; j < n; j++) {
        for (int i = 0; i < 65536; i++)
            ;
    }
}
#define USARTx USART2
#define USARTx_CLK RCC_APB1Periph_USART2
#define USARTx_APBPERIPHCLOCK RCC_APB1PeriphClockCmd
#define USARTx_IRQn USART2_IRQn
#define USARTx_IRQHandler USART2_IRQHandler

#define USARTx_TX_PIN GPIO_Pin_5
#define USARTx_TX_GPIO_PORT GPIOD
#define USARTx_TX_GPIO_CLK RCC_AHBPeriph_GPIOD
#define USARTx_TX_SOURCE GPIO_PinSource5
#define USARTx_TX_AF GPIO_AF_0

#define USARTx_RX_PIN GPIO_Pin_6
#define USARTx_RX_GPIO_PORT GPIOD
#define USARTx_RX_GPIO_CLK RCC_AHBPeriph_GPIOD
#define USARTx_RX_SOURCE GPIO_PinSource6
#define USARTx_RX_AF GPIO_AF_0

/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
static void USART_Config(void)
{
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIO clock */
    RCC_AHBPeriphClockCmd(USARTx_TX_GPIO_CLK | USARTx_RX_GPIO_CLK, ENABLE);

    /* Enable USART clock */
    USARTx_APBPERIPHCLOCK(USARTx_CLK, ENABLE);
    RCC_USARTCLKConfig(RCC_USART2CLK_SYSCLK);
    /* Connect PXx to USARTx_Tx */
    GPIO_PinAFConfig(USARTx_TX_GPIO_PORT, USARTx_TX_SOURCE, USARTx_TX_AF);

    /* Connect PXx to USARTx_Rx */
    GPIO_PinAFConfig(USARTx_RX_GPIO_PORT, USARTx_RX_SOURCE, USARTx_RX_AF);

    /* Configure USART Tx and Rx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN;
    GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = USARTx_RX_PIN;
    GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStructure);

    /* USARTx configuration ----------------------------------------------------*/
    /* USARTx configured as follow:
  - BaudRate = 230400 baud  
  - Word Length = 8 Bits
  - One Stop Bit
  - No parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USARTx, &USART_InitStructure);

    USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
    //USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);
    /* NVIC configuration */
    /* Enable the USARTx Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USARTx_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    USARTx->CR3 |= USART_CR3_OVRDIS;
    /* Enable USART */
    USART_Cmd(USARTx, ENABLE);
}

uint8_t RxBuffer[512] = { 0 };
uint8_t TxBuffer[512] = { 0 };
uint8_t UsartSendFilish = 1;
uint8_t UsartSendLenth = 0;
uint8_t TxIndex = 0;
uint8_t RxIndex = 0;
void USARTx_SendTest(const char* d, uint8_t len)
{
    if (UsartSendFilish) {
        UsartSendFilish = 0;
        memcpy((void*)TxBuffer, (void*)d, len);
        UsartSendLenth = len;
        TxIndex = 0;
        USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);
    }
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */

int16_t Channel1Pulse = 0;
int16_t Channel2Pulse = 0;
uint8_t TImer1msFlag = 0;
uint32_t Timer1msCnt = 0;
uint32_t Timer1msCnt1 = 0;

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void SetMCU_RW_DIR(uint16_t d)
{
    if (d) {
        GPIO_SetBits(GPIOE, GPIO_Pin_10);
    } else {
        GPIO_ResetBits(GPIOE, GPIO_Pin_10);
    }
}
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void SetMCU_RW_PWM(uint16_t d)
{
    TIM_SetCompare2(TIM1, d);
}
/**GPIO_InitStructure
  * @brief  MainGPIO_InitStructure program.
  * @param  None
  * @retval None
  */
void SetMCU_LW_PWM(uint16_t d)
{
    TIM_SetCompare1(TIM1, d);
}
/**
  * @brief  Main program.	ros::Publisher path_pub = ph.advertise<nav_msgs::Path>("camtf",1, true);
  * @param  None
  * @retval None
  */

void SetMCU_LW_DIR(uint16_t d)
{
    if (d) {
        GPIO_SetBits(GPIOE, GPIO_Pin_8);
    } else {
        GPIO_ResetBits(GPIOE, GPIO_Pin_8);
    }
}
/**
  * @brief  Configure the TIM1 Pins.
  * @param  None#include "STM32Hardware.h"
uint8_t testd[20] = {0xFF,0x02,0x03};
  * @retval None
  */
static void TIM_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* GPIOA, GPIOB and GPIOE Clocks enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);
    /* GPIOA Configuration: Channel 1,SetMCU_LW_DIR 2, 3, 4 and Channel 1N as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_0);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_0);

    /* Compute CCR1 value to generate a duty cycle at 50% for channel 1 and 1N */
    //Channel1Pulse = 0;
    /* Compute CCR2 value to generate	a duty cycle at 37.5%  for channel 2 and 2N */
    //Channel2Pulse = 0;

    /* TIM1 clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    /* TIM3 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    /* Time Base configuration */
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 2000;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    /* Time Base configuration */
    TIM_TimeBaseStructure.TIM_Prescaler = 48 - 1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 1000;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /* Channel 1, 2,3 and 4 Configuration in PWM mode */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);

    /* TIM1 counter enable */
    TIM_Cmd(TIM1, ENABLE);

    /* TIM1 Main Output Enable */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    /* TIM3 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* Enable the TIM3 gloabal Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    /* TIM1 counter enable */
    TIM_Cmd(TIM3, ENABLE);
    /* TIM Interrupts enable */
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}
static void LedInit(void)
{
    //SystemInit();
    GPIO_InitTypeDef GPIO_InitStructure;
    /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (PIO_InitStructure'startup_stm32f0xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f0xx.c file
     */

    /* Add your application code here
     */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    TIM_Config();
    /* Configure PC10 and PC11 in output pushpull mode */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB, GPIO_Pin_2);
    GPIO_SetBits(GPIOB, GPIO_Pin_12);
}

/**
  * @brief  Configure PA0 in interrupt mode
  * @param  None
  * @retval None
  */
static void EXTI0_Config(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIOA clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);

    /* Configure PA0 pin as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    /* Connect EXTI0 Line to PA0 pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource10);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource14);
    /* Configure EXTI0 line */
    EXTI_InitStructure.EXTI_Line = EXTI_Line10;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    EXTI_InitStructure.EXTI_Line = EXTI_Line14;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set EXTI0 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
static void LedFlash(uint16_t n)
{

    static uint16_t time1mscnt = 0;
    time1mscnt++;
    if (time1mscnt <= n) {
        GPIO_SetBits(GPIOB, GPIO_Pin_14);
        GPIO_SetBits(GPIOB, GPIO_Pin_13);
    } else if (time1mscnt <= (n << 1)) {
        GPIO_ResetBits(GPIOB, GPIO_Pin_14);
        GPIO_ResetBits(GPIOB, GPIO_Pin_13);
    } else {
        //USARTx_SendTest("i am desperado\n", sizeof("i am desperado\n"));
        time1mscnt = 0;
    }
}
#include "STM32Hardware.h"
uint8_t testd[20] = { 0xFF, 0x02, 0x03 };
STM32Hardware sertest;
HardwareSerial sertest1;
void seritets(uint8_t* d)
{
    memcpy(testd + 4, d, 4);
    sertest.write(testd, 10);
}
uint16_t rmotorbase = 900;
uint16_t lmotorbase = 900;
uint8_t directionl = 0;
uint8_t directionr = 0;
void SetSevo(uint8_t id, float v)
{
    int d = (int)v;
    if (id == 0) {
        if (d >= 0) {
            lmotorbase = 860;
            directionl = 0;
            SetMCU_LW_DIR(0);
        } else {
            directionl = 1;
            lmotorbase = 0;
            d = d + 1950;
            SetMCU_LW_DIR(1);
        }
        lmotorbase += abs(d);
        if (lmotorbase >= 1999)
            lmotorbase = 1999;
        SetMCU_LW_PWM(lmotorbase);
    } else {
        if (d >= 0) {
            directionr = 1;
            rmotorbase = 900;
            SetMCU_RW_DIR(0);
        } else {
            directionr = 0;
            rmotorbase = 0;
            d = d + 1950;
            SetMCU_RW_DIR(1);
        }
        rmotorbase += abs(d);
        if (rmotorbase >= 1999)
            rmotorbase = 1999;
        SetMCU_RW_PWM(rmotorbase);
    }
}

int32_t ActualSpeedr = 0;
int32_t ActualSpeedl = 0;

__IO uint32_t curr_encoder_countl = 0;
__IO uint32_t curr_encoder_countr = 0;

uint32_t pre_encoder_countl = 0;
uint32_t pre_encoder_countr = 0;

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
#define a_dt 0.02
//mm/s
static uint32_t calc_speed(void)
{
    //const uint32_t encoder_count = curr_encoder_countl -prev_encoder_countl;
    uint32_t encoder_count = curr_encoder_countl;
    curr_encoder_countl = 0;
    //prev_encoder_countl = curr_encoder_countl;
    uint32_t dist_mm_100x = (encoder_count * 100) * WHEEL_CIRCUMFERENCE_MM_100X / WHEEL_REVOLUTION_COUNT_100X;
    ActualSpeedl = dist_mm_100x*0.5 ;
    if (directionl == 0)
        ActualSpeedl = -ActualSpeedl;

    //encoder_count = curr_encoder_countr -prev_encoder_countr;
    encoder_count = curr_encoder_countr;
    curr_encoder_countr = 0;
    dist_mm_100x = (encoder_count * 100) * WHEEL_CIRCUMFERENCE_MM_100X / WHEEL_REVOLUTION_COUNT_100X;
    ActualSpeedr = dist_mm_100x*0.5 ;
    if (directionr == 0)
        ActualSpeedr = -ActualSpeedr;

    //    ActualSpeedl = curr_encoder_countl ;
    //    curr_encoder_countl = 0;
    //    ActualSpeedr = curr_encoder_countr;
    //    curr_encoder_countr = 0;
}

__IO float gCmdMsgVx = 0.0f;
__IO float gCmdMsgWz = 0.0f;

void servo_cb(const geometry_msgs::Twist& cmd_msg)
{
    gCmdMsgVx = cmd_msg.linear.x*1000;
    gCmdMsgWz = cmd_msg.angular.z*57.29578049;
}

float ServoLOut = 0;
float ServoROut = 0;

// float rmoterkp = 5.2;
// float lmoterkp = 5.2;

// float rmoterki = 2.5;
// float lmoterki = 2.5;
float rmoterkp =  1.1;
 float lmoterkp = 1.1;

 float rmoterki = 0.5;
 float lmoterki = 0.5;
float rmoterkd = 0.1;
float lmoterkd = 0.1;

float interr = 0;
float interl = 0;

float olderrdr = 0;
float olderrdl = 0;
float errlowpassdr = 0;
float errlowpassdl = 0;
#define WHEELSITANSE 220   //mm/s   //22cm

static void bumperinit(void)
{
 NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    //pc12 = r
    //pc0 = f
    //pd0 = l

    //pe5 = lsw
    //pa12 = rsw
    /* GPIOA, GPIOB and GPIOE Clocks enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOE|RCC_AHBPeriph_GPIOA,ENABLE);
 
    /* GPIOA Configuration: Channel 1,SetMCU_LW_DIR 2, 3, 4 and Channel 1N as alternate function push-pull */
    // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_0 ;
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    // GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    // GPIO_Init(GPIOC, &GPIO_InitStructure);
    // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    // GPIO_Init(GPIOD, &GPIO_InitStructure);
    // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    // GPIO_Init(GPIOE, &GPIO_InitStructure);
    // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 ;
    // GPIO_Init(GPIOA, &GPIO_InitStructure);

    // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    // GPIO_Init(GPIOB, &GPIO_InitStructure);
    // GPIO_SetBits(GPIOB,GPIO_Pin_2);
    // GPIO_SetBits(GPIOE,GPIO_Pin_5);
    // GPIO_SetBits(GPIOA,GPIO_Pin_12);



    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);





}

// uint8_t GetBumper()
// {
//     uint8_t temp = 0;
//     if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_0)==0){
//         temp = (0X01);
//     }
//     if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12)==0){
//         temp |= (0X01<<1);
//     }
//     if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)==0){
//         temp |= (0X01<<2);
//     }
//     return temp;
// }


uint8_t GetBumper()
{
    uint8_t temp = 0;
    if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4)==0){
        temp = (0X01);
    }
    if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12)==0){
        temp |= (0X01<<1);
    }

    return temp;
}


void ControlServo(void)
{
    float l = WHEELSITANSE; //cm;
    float vr = 0.0f;
    float vl = 0.0f;
    static float CmdMsgVx = 0;
    static float CmdMsgWz = 0;
    CmdMsgWz = gCmdMsgWz;
    if (fabs(gCmdMsgVx- CmdMsgVx)>5.0f){
        float out = (gCmdMsgVx-CmdMsgVx)*0.1f;
        if(out>=30.f)out = 30.f;
        else if(out<=-30.f)out =-30.f;
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

    float lverr = vl - ActualSpeedl;
    float rverr = vr - ActualSpeedr;

    //ServoLOut  = ServoLOut+0.05f*lverr;
    //ServoROut  = ServoROut+0.05f*rvzrr;
    interr = interr + rmoterki * rverr;
    interl = interl + lmoterki * lverr;

    if (interl >= 1950)
        interl = 1950;
    if (interr >= 1099)
        interr = 1099;
    if (interl <= -1139)
        interl = -1139;
    if (interr <= -1950)
        interr = -1950;
    errlowpassdr = errlowpassdr + 0.5f * (rverr - olderrdr);
    errlowpassdl = errlowpassdl + 0.5f * (lverr - olderrdl);
    olderrdr = rverr;
    olderrdl = lverr;
    ServoLOut = interl + lmoterkp * lverr + errlowpassdl * lmoterkd;
    ServoROut = interr + rmoterkp * rverr + errlowpassdr * rmoterkd;

    if (ServoLOut >= 1950)
        ServoLOut = 1950;
    if (ServoROut >= 1099)
        ServoROut = 1099;

    if (ServoLOut <= -1139)
        ServoLOut = -1139;
    if (ServoROut <= -1950)
        ServoROut = -1950;

    SetSevo(0, -ServoLOut);
    SetSevo(1, ServoROut);
}
static HwImu g_HwImu;
int main(void)
{
    TIM_Config();
    EXTI0_Config();
   // hw_imu_construct(&g_HwImu);
    bumperinit();
    initialise();
    uint32_t previous_debug_time = 0;
    // char buffer[50] = "Current borad volt is";
    // std_msgs::String str_msg;

    // sensor_msgs::Imu imu_data;
    // memset((void*)&imu_data.angular_velocity_covariance, 0, sizeof(imu_data.angular_velocity_covariance));
    // memset((void*)&imu_data.orientation_covariance, 0, sizeof(imu_data.orientation_covariance));
    // memset((void*)&imu_data.linear_acceleration_covariance, 0, sizeof(imu_data.linear_acceleration_covariance));

    nav_msgs::Odometry odom_data;
    std_msgs::UInt8    bumper_data;
    memset((void*)&odom_data.pose.covariance, 0, sizeof(odom_data.pose.covariance));
    memset((void*)&odom_data.twist.covariance, 0, sizeof(odom_data.twist.covariance));

    ros::NodeHandle nh;
    nh.initNode();
    while (!nh.connected()) {
        nh.spinOnce();
    }
    nh.loginfo("ros is connect");
    // ros::Publisher battery("battery", &str_msg);
    // ros::Publisher imupub("x800imu", &imu_data);
    ros::Publisher odompub("x800_odom", &odom_data);
    ros::Publisher Buperpub("x800_bumper", &bumper_data);
    //ros::Subscriber<std_msgs::Float64> subled("led", led_cb);
    //Pc端的subscrib有时间os::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
    ros::Subscriber<geometry_msgs::Twist> subser("servo", servo_cb);
    led.init();
    //nh.advertise(battery);
    // nh.advertise(imupub);
    nh.advertise(odompub);
    nh.advertise(Buperpub);
    //nh.subscribe(subled);
    nh.subscribe(subser);
    delay(100);
    // hw_imu_set_run_mode(&g_HwImu, HW_IMU_RUN_MODE_NORMAL);

    float odom_x = 0.0f;
    float odom_y = 0.0f;
    float odom_vx = 0.0f;
    float odom_vy = 0.0f;
    float odom_wz = 0.0f;
    float odom_theta = 0.0f;
    int time1ms = 0;
    int oldyawangle = 0;
    int olddeltayaw = 0;
    Timer1msCnt1 = 0;
    while (1) {
        uint8_t n = 10;
        if (TImer1msFlag) {

            uint8_t buper = GetBumper();
            TImer1msFlag = 0;
             if (++time1ms >= 20)
             {
       

                 time1ms = 0;
                 ControlServo();
            //     hw_imu_update(&g_HwImu);
                float l = WHEELSITANSE;                               //cm;
                float odom_v = (ActualSpeedr + ActualSpeedl) * 0.5f;  //mm/s
                odom_wz = (ActualSpeedr - ActualSpeedl) * 0.0045454f; //rad/s
                odom_vx = odom_v * cos(odom_theta) * 0.001f;
                odom_vy = odom_v * sin(odom_theta) * 0.001f;
                float delta_x = odom_vx * a_dt;
                float delta_y = odom_vy * a_dt;
                float delta_th = odom_wz * a_dt;
                odom_theta += delta_th;
                odom_x += delta_x;
                odom_y += delta_y;

                geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(odom_theta);
                odom_data.header.stamp = nh.now();
                ;
                odom_data.header.frame_id = "odom";
                odom_data.child_frame_id = "base_link";
                odom_data.pose.pose.position.x = odom_x;
                odom_data.pose.pose.position.y = odom_y;
                odom_data.pose.pose.position.z = 0;
                odom_data.pose.pose.orientation = odom_quat;

                odom_data.twist.twist.linear.x = odom_vx;
                odom_data.twist.twist.linear.y = odom_vy;
                odom_data.twist.twist.linear.z = 0;
                odom_data.twist.twist.angular.x = 0;
                odom_data.twist.twist.angular.y = 0;
                odom_data.twist.twist.angular.z = odom_wz;
                odom_data.pose.covariance[0] = buper;
                odompub.publish(&odom_data);
                bumper_data.data = buper;
                Buperpub.publish(&bumper_data);

             }
            //if(odom_theta>=179.9f)odom_theta = 179.9f;
            //if(odom_theta<=-179.9f)odom_theta = -179.9f;
        }
        //seritets(&n);
        //nh.publish(battery.id_,&str_msg);
        if ((millis() - previous_debug_time) >= (10)) {


            


            //LedFlash(5);
            //sprintf(buffer, "%d,%d", curr_encoder_countl, curr_encoder_countr);
            //buffer[29] = curr_encoder_countl;
            // str_msg.data = buffer;
            // battery.publish(&str_msg);
            // if (g_HwImu.data_valible) {
            //     g_HwImu.data_valible = false;
            //     imu_data.header.stamp = nh.now();
            //     imu_data.header.frame_id = "x800imu_link";
            //     imu_data.linear_acceleration.x = -g_HwImu.x_ * 0.00953f;
            //     imu_data.linear_acceleration.y = -g_HwImu.y_ * 0.00953f;
            //     imu_data.linear_acceleration.z = -g_HwImu.z_ * 0.00953f;
            //     imu_data.angular_velocity.x = 0;
            //     imu_data.angular_velocity.y = 0;//g_HwImu.gz_*0.00013963f;
            //     int deltayaw = g_HwImu.angle_ -oldyawangle;
            //     if(abs(deltayaw)>=10000){
            //         deltayaw = olddeltayaw;
            //     }

                 oldyawangle = g_HwImu.angle_;
            //     olddeltayaw = deltayaw;
            //     imu_data.angular_velocity.z =(deltayaw)*0.1745329f/Timer1msCnt1;
            //     Timer1msCnt1 = 0;
            //     imupub.publish(&imu_data);
            // }

            previous_debug_time = millis();
        }
        nh.spinOnce();
    }
}
#ifdef __cplusplus
extern "C" {
#endif
/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/******************************************************************************/
/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        Timer1msCnt++;
        Timer1msCnt1++;
        TImer1msFlag = 1;
    }
}
void WWDG_IRQHandler(void)
{
    WWDG_ClearFlag();
}
void EXTI4_15_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line10) != RESET) {
        curr_encoder_countl++;
        /* Clear the EXTI line 2 pending bit */
        EXTI_ClearITPendingBit(EXTI_Line10);
    }

    if (EXTI_GetITStatus(EXTI_Line14) != RESET) {
        curr_encoder_countr++;
        /* Clear the EXTI line 2 pending bit */
        EXTI_ClearITPendingBit(EXTI_Line14);
    }
}

#ifdef __cplusplus
}
#endif
