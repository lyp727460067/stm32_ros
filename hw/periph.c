#define PERIPH_EXTER_DEFIN
#include "periph.h"

static void GPIO_WheelConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* GPIOA, GPIOB and GPIOE Clocks enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);

    /* GPIOA Configuration: Channel 1,SetMCU_LW_DIR 2, 3, 4 and Channel 1N as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    //direction

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_0);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_0);

    GPIO_InitStructure.GPIO_Pin = WHEELL_TIMx_CW_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(WHEELL_TIMx_CW_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = WHEELR_TIMx_CW_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(WHEELR_TIMx_CW_GPIO_PORT, &GPIO_InitStructure);

   
}

static void TIM_WheelConfig(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    /* Compute CCR1 value to generate a duty cycle at 50% for channel 1 and 1N */
    //Channel1Pulse = 0;
    /* Compute CCR2 value to generate a duty cycle at 37.5%  for channel 2 and 2N */
    //Channel2Pulse = 0;

    /* TIM1 clock enable */
    WHEEL_TIMx_PeriphClockCmd(WHEEL_TIMx_CLK, ENABLE);
    /* Time Base configuration */
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 2000;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(WHEEL_TIMx, &TIM_TimeBaseStructure);

    /* Channel 1, 2,3 and 4 Configuration in PWM mode */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    //WHEEL  //L
    TIM_OC1Init(WHEEL_TIMx, &TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_Pulse = 0;
    //R
    TIM_OC2Init(WHEEL_TIMx, &TIM_OCInitStructure);

    /* TIM1 counter enable */
    TIM_Cmd(WHEEL_TIMx, ENABLE);

    /* TIM1 Main Output Enable */
    TIM_CtrlPWMOutputs(WHEEL_TIMx, ENABLE);
}

static void TIM_BrushConfig(void)
{

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    /* TIM1 clock enable */
    BRUSH_TIMx_PeriphClockCmd(BRUSH_TIMx_CLK, ENABLE);
    /* Time Base configuration */
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 2000;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(BRUSH_TIMx, &TIM_TimeBaseStructure);

    /* Channel 1, 2,3 and 4 Configuration in PWM mode */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

    TIM_OC1Init(BRUSH_TIMx, &TIM_OCInitStructure);
    /* TIM1 counter enable */
    TIM_Cmd(BRUSH_TIMx, ENABLE);
    /* TIM1 Main Output Enable */
    TIM_CtrlPWMOutputs(BRUSH_TIMx, ENABLE);
}

static void GPIO_BrushConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    /*  GPIOE Clocks enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource3, GPIO_AF_0);

    /// CW
    BRUSH_TIMx_CW_PeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = BRUSH_TIMx_CW_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(BRUSH_TIMx_CW_GPIO_PORT, &GPIO_InitStructure);

   
}

static void TIM_SideBrushConfig(void)
{

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    /* TIM1 clock enable */
    SIDEBRUSHL_TIMx_PeriphClockCmd(SIDEBRUSHL_TIMx_CLK, ENABLE);
    /* Time Base configuration */
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 2000;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(SIDEBRUSHL_TIMx, &TIM_TimeBaseStructure);

    /* Channel 1, 2,3 and 4 Configuration in PWM mode */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    //SIDEBRUSHL
    TIM_OC1Init(SIDEBRUSHL_TIMx, &TIM_OCInitStructure);
    //SIDEBRUSHR
    TIM_OC2Init(SIDEBRUSHL_TIMx, &TIM_OCInitStructure);
    /* TIM1 counter enable */
    TIM_Cmd(SIDEBRUSHL_TIMx, ENABLE);
    /* TIM1 Main Output Enable */
    TIM_CtrlPWMOutputs(SIDEBRUSHL_TIMx, ENABLE);
}

static void GPIO_SideBrushConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    /*  GPIOE Clocks enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
    //GPIO_Pin_10 -->L
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    SIDEBRUSHL_TIMx_CW_PeriphClockCmd(SIDEBRUSHL_TIMx_CW_GPIO_CLK, ENABLE);
    SIDEBRUSHR_TIMx_CW_PeriphClockCmd(SIDEBRUSHR_TIMx_CW_GPIO_CLK, ENABLE);
    /// CW
    GPIO_InitStructure.GPIO_Pin = SIDEBRUSHL_TIMx_CW_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(SIDEBRUSHL_TIMx_CW_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = SIDEBRUSHR_TIMx_CW_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(SIDEBRUSHR_TIMx_CW_GPIO_PORT, &GPIO_InitStructure);

    
}

static void TIM_VacuumConfig(void)
{

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    /* TIM1 clock enable */
    VACUUML_TIMx_PeriphClockCmd(VACUUML_TIMx_CLK, ENABLE);
    /* Time Base configuration */
    TIM_TimeBaseStructure.TIM_Prescaler = 16;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 2000;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(VACUUML_TIMx, &TIM_TimeBaseStructure);

    /* Channel 1, 2,3 and 4 Configuration in PWM mode */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    //R
    TIM_OC3Init(VACUUML_TIMx, &TIM_OCInitStructure);
    //L
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC4Init(VACUUML_TIMx, &TIM_OCInitStructure);
    /* TIM1 counter enable */
    TIM_Cmd(VACUUML_TIMx, ENABLE);
    /* TIM1 Main Output Enable */
    TIM_CtrlPWMOutputs(VACUUML_TIMx, ENABLE);
}

static void GPIO_VacuumConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    /*  GPIOE Clocks enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_2);

    //en
    GPIO_InitStructure.GPIO_Pin = VACUUML_TIMx_EN_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(VACUUML_TIMx_EN_GPIO_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = VACUUMR_TIMx_EN_PIN;
    GPIO_Init(VACUUMR_TIMx_EN_GPIO_PORT, &GPIO_InitStructure);
  
}
static void EXTI0_Config(void)
{

    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;



	//side_brush fb

    SIDEBRUSHR_TIMx_FB_PeriphClockCmd(SIDEBRUSHR_TIMx_FB_GPIO_CLK, ENABLE);
    SIDEBRUSHL_TIMx_FB_PeriphClockCmd(SIDEBRUSHL_TIMx_FB_GPIO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = SIDEBRUSHL_TIMx_FB_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(SIDEBRUSHL_TIMx_FB_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = SIDEBRUSHR_TIMx_FB_PIN;
    GPIO_Init(SIDEBRUSHR_TIMx_FB_GPIO_PORT, &GPIO_InitStructure);

    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    /* Connect EXTI6 Line to PC6 pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource1);
    /* Configure EXTI0 line */
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set EXTI0 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


}
static void EXTI4_15_Config(void)
{
 ///////////////////////

    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
	// wheel fb

    /* Enable GPIOA clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
    /* Configure PA0 pin as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
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
    // Vacuum  fb
    
    VACUUML_TIMx_FB_PeriphClockCmd(VACUUML_TIMx_FB_GPIO_CLK, ENABLE);
    VACUUMR_TIMx_FB_PeriphClockCmd(VACUUMR_TIMx_FB_GPIO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = VACUUML_TIMx_FB_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(VACUUML_TIMx_FB_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = VACUUMR_TIMx_FB_PIN;
    GPIO_Init(VACUUMR_TIMx_FB_GPIO_PORT, &GPIO_InitStructure);

    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    /* Connect EXTI6 Line to PC6 pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource13);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource15);
    /* Configure EXTI0 line */
    EXTI_InitStructure.EXTI_Line = EXTI_Line13;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line = EXTI_Line15;
    EXTI_Init(&EXTI_InitStructure);

    
     // brush fb
    BRUSH_TIMx_FB_PeriphClockCmd(BRUSH_TIMx_FB_GPIO_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin = BRUSH_TIMx_FB_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;

    GPIO_Init(BRUSH_TIMx_FB_GPIO_PORT, &GPIO_InitStructure);

    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    /* Connect EXTI6 Line to PC6 pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource7);
    /* Configure EXTI0 line */
    EXTI_InitStructure.EXTI_Line = EXTI_Line7;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    
    /* Enable and set EXTI0 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


}

void PeriphInit(void)
{
    GPIO_WheelConfig();
    TIM_WheelConfig();
    GPIO_BrushConfig();
    TIM_BrushConfig();
    TIM_VacuumConfig();
    GPIO_VacuumConfig();
    TIM_SideBrushConfig();
    GPIO_SideBrushConfig();

	EXTI0_Config();
	EXTI4_15_Config();
}
#ifdef __cplusplus
extern "C" {
#endif
//encoder
void EXTI0_1_IRQHandler(void)
{


    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        g_fbSideBrushRCnt++;
        /* Clear the EXTI line 2 pending bit */
        EXTI_ClearITPendingBit(EXTI_Line0);
    }

    if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
        g_fbSideBrushLCnt++;
        /* Clear the EXTI line 2 pending bit */
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}
//encoder
void EXTI4_15_IRQHandler(void)
{
  
    if (EXTI_GetITStatus(EXTI_Line6) != RESET) {
        g_fbBrushCnt++;
        /* Clear the EXTI line 2 pending bit */
        EXTI_ClearITPendingBit(EXTI_Line6);
    }
    
    
    if (EXTI_GetITStatus(EXTI_Line10) != RESET) {
        g_fbWheelLCnt++;
        /* Clear the EXTI line 2 pending bit */
        EXTI_ClearITPendingBit(EXTI_Line10);
    }

    if (EXTI_GetITStatus(EXTI_Line14) != RESET) {
        g_fbWheelRCnt++;
        /* Clear the EXTI line 2 pending bit */
        EXTI_ClearITPendingBit(EXTI_Line14);
    }

    if (EXTI_GetITStatus(EXTI_Line13) != RESET) {
        g_fbVacuumLCnt++;
        /* Clear the EXTI line 2 pending bit */
        EXTI_ClearITPendingBit(EXTI_Line13);
    }
    if (EXTI_GetITStatus(EXTI_Line15) != RESET) {
        g_fbVacuumRCnt++;
        /* Clear the EXTI line 2 pending bit */
        EXTI_ClearITPendingBit(EXTI_Line15);
    }
}
#ifdef __cplusplus
}
#endif
