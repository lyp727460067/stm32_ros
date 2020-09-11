#ifndef _PERIPH
#define _PERIPH
#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f0xx.h"



////oc1 -->L  oc2 -->R  
#define WHEEL_TIMx                              TIM1
#define WHEEL_TIMx_PeriphClockCmd               RCC_APB2PeriphClockCmd
#define WHEEL_TIMx_CLK                          RCC_APB2Periph_TIM1
#define WHEEL_TIMx_IRQn                         TIME1_IRQn
#define WHEEL_TIMx_IRQHandler                   TIME1_IRQHandler
/*
#define WHEEL_TIMx_LEFT_PeriphClockCmd          RCC_AHBPeriphClockCmd
#define WHEEL_TIMx_LEFT_PIN                     GPIO_Pin_9
#define WHEEL_TIMx_LEFT_GPIO_PORT               GPIOE
#define WHEEL_TIMx_LEFT_GPIO_CLK                RCC_AHBPeriph_GPIOE
#define WHEEL_TIMx_LEFT_SOURCE                  GPIO_PinSource9
#define WHEEL_TIMx_LEFT_AF                      GPIO_AF_0

#define WHEEL_TIMx_RIGHT_PeriphClockCmd          RCC_AHBPeriphClockCmd
#define WHEEL_TIMx_RIGHT_PIN                     GPIO_Pin_11
#define WHEEL_TIMx_RIGHT_GPIO_PORT               GPIOE
#define WHEEL_TIMx_RIGHT_GPIO_CLK                RCC_AHBPeriph_GPIOE
#define WHEEL_TIMx_RIGHT_SOURCE                  GPIO_PinSource9
#define WHEEL_TIMx_RIGHT_AF                      GPIO_AF_0

#define WHEEL_TIMx_LEFT_BREAK_PeriphClockCmd          RCC_AHBPeriphClockCmd
#define WHEEL_TIMx_LEFT_BREAK_PIN                     GPIO_Pin_9
#define WHEEL_TIMx_LEFT_BREAK_GPIO_PORT               GPIOE
#define WHEEL_TIMx_LEFT_BREAK_GPIO_CLK                RCC_AHBPeriph_GPIOE
#define WHEEL_TIMx_LEFT_BREAK_SOURCE                  GPIO_PinSource9
#define WHEEL_TIMx_LEFT_BREAK_AF                      GPIO_AF_0

#define WHEEL_TIMx_RIGHT_BREAK_PeriphClockCmd          RCC_AHBPeriphClockCmd
#define WHEEL_TIMx_RIGHT_BREAK_PIN                     GPIO_Pin_11
#define WHEEL_TIMx_RIGHT_BREAK_GPIO_PORT               GPIOE
#define WHEEL_TIMx_RIGHT_BREAK_GPIO_CLK                RCC_AHBPeriph_GPIOE
#define WHEEL_TIMx_RIGHT_BREAK_SOURCE                  GPIO_PinSource9
#define WHEEL_TIMx_RIGHT_BREAK_AF                      GPIO_AF_0
*/

#define WHEELL_TIMx_CW_PeriphClockCmd          RCC_AHBPeriphClockCmd
#define WHEELL_TIMx_CW_PIN                     GPIO_Pin_8
#define WHEELL_TIMx_CW_GPIO_PORT               GPIOE
#define WHEELL_TIMx_CW_GPIO_CLK                RCC_AHBPeriph_GPIOE

#define WHEELR_TIMx_CW_PeriphClockCmd          RCC_AHBPeriphClockCmd
#define WHEELR_TIMx_CW_PIN                     GPIO_Pin_10
#define WHEELR_TIMx_CW_GPIO_PORT               GPIOE
#define WHEELR_TIMx_CW_GPIO_CLK                RCC_AHBPeriph_GPIOE



/**************/
//oc1
#define BRUSH_TIMx                              TIM3
#define BRUSH_TIMx_PeriphClockCmd               RCC_APB1PeriphClockCmd
#define BRUSH_TIMx_CLK                          RCC_APB1Periph_TIM3
#define BRUSH_TIMx_IRQn                         TIME3_IRQn
#define BRUSH_TIMx_IRQHandler                   TIME3_IRQHandler

#define BRUSH_TIMx_CW_PeriphClockCmd          RCC_AHBPeriphClockCmd
#define BRUSH_TIMx_CW_PIN                     GPIO_Pin_7
#define BRUSH_TIMx_CW_GPIO_PORT               GPIOA
#define BRUSH_TIMx_CW_GPIO_CLK                RCC_AHBPeriph_GPIOA

//

#define BRUSH_TIMx_FB_PeriphClockCmd          RCC_AHBPeriphClockCmd
#define BRUSH_TIMx_FB_PIN                     GPIO_Pin_6
#define BRUSH_TIMx_FB_GPIO_PORT               GPIOC
#define BRUSH_TIMx_FB_GPIO_CLK                RCC_AHBPeriph_GPIOC

//
//oc1-->L  oc2-->R
/**************/
#define SIDEBRUSHL_TIMx                              TIM15
#define SIDEBRUSHL_TIMx_PeriphClockCmd               RCC_APB2PeriphClockCmd
#define SIDEBRUSHL_TIMx_CLK                          RCC_APB2Periph_TIM15
#define SIDEBRUSHR_TIMx                              TIM15
#define SIDEBRUSHR_TIMx_PeriphClockCmd               RCC_APB2PeriphClockCmd
#define SIDEBRUSHR_TIMx_CLK                          RCC_APB2Periph_TIM15


#define SIDEBRUSHL_TIMx_CW_PeriphClockCmd            RCC_AHBPeriphClockCmd
#define SIDEBRUSHL_TIMx_CW_PIN                       GPIO_Pin_5
#define SIDEBRUSHL_TIMx_CW_GPIO_PORT                 GPIOB
#define SIDEBRUSHL_TIMx_CW_GPIO_CLK                  RCC_AHBPeriph_GPIOB

#define SIDEBRUSHR_TIMx_CW_PeriphClockCmd            RCC_AHBPeriphClockCmd
#define SIDEBRUSHR_TIMx_CW_PIN                       GPIO_Pin_14
#define SIDEBRUSHR_TIMx_CW_GPIO_PORT                 GPIOE
#define SIDEBRUSHR_TIMx_CW_GPIO_CLK                  RCC_AHBPeriph_GPIOE


#define SIDEBRUSHL_TIMx_FB_PeriphClockCmd            RCC_AHBPeriphClockCmd
#define SIDEBRUSHL_TIMx_FB_PIN                       GPIO_Pin_1
#define SIDEBRUSHL_TIMx_FB_GPIO_PORT                 GPIOB
#define SIDEBRUSHL_TIMx_FB_GPIO_CLK                  RCC_AHBPeriph_GPIOB
                      
#define SIDEBRUSHR_TIMx_FB_PeriphClockCmd            RCC_AHBPeriphClockCmd
#define SIDEBRUSHR_TIMx_FB_PIN                       GPIO_Pin_0
#define SIDEBRUSHR_TIMx_FB_GPIO_PORT                 GPIOB
#define SIDEBRUSHR_TIMx_FB_GPIO_CLK                  RCC_AHBPeriph_GPIOB




//

//oc4-->L  oc3-->R
/**************/
#define VACUUML_TIMx                              TIM2
#define VACUUML_TIMx_PeriphClockCmd               RCC_APB1PeriphClockCmd
#define VACUUML_TIMx_CLK                          RCC_APB1Periph_TIM2

#define VACUUMR_TIMx                              TIM2
#define VACUUMR_TIMx_PeriphClockCmd               RCC_APB1PeriphClockCmd
#define VACUUMR_TIMx_CLK                          RCC_APB1Periph_TIM2


#define VACUUML_TIMx_EN_PeriphClockCmd            RCC_AHBPeriphClockCmd
#define VACUUML_TIMx_EN_PIN                       GPIO_Pin_7
#define VACUUML_TIMx_EN_GPIO_PORT                 GPIOE
#define VACUUML_TIMx_EN_GPIO_CLK                  RCC_AHBPeriph_GPIOE

#define VACUUMR_TIMx_EN_PeriphClockCmd            RCC_AHBPeriphClockCmd
#define VACUUMR_TIMx_EN_PIN                       GPIO_Pin_6
#define VACUUMR_TIMx_EN_GPIO_PORT                 GPIOE
#define VACUUMR_TIMx_EN_GPIO_CLK                  RCC_AHBPeriph_GPIOEs
//
#define VACUUML_TIMx_FB_PeriphClockCmd            RCC_AHBPeriphClockCmd
#define VACUUML_TIMx_FB_PIN                       GPIO_Pin_13
#define VACUUML_TIMx_FB_GPIO_PORT                 GPIOE
#define VACUUML_TIMx_FB_GPIO_CLK                  RCC_AHBPeriph_GPIOE
                   
#define VACUUMR_TIMx_FB_PeriphClockCmd            RCC_AHBPeriphClockCmd
#define VACUUMR_TIMx_FB_PIN                       GPIO_Pin_15
#define VACUUMR_TIMx_FB_GPIO_PORT                 GPIOE
#define VACUUMR_TIMx_FB_GPIO_CLK                  RCC_AHBPeriph_GPIOE


#ifdef  PERIPH_EXTER_DEFIN
    #define PERIPH_EXTER 
#else
    #define PERIPH_EXTER extern
#endif




//
PERIPH_EXTER __IO uint32_t g_fbVacuumLCnt ;
PERIPH_EXTER __IO uint32_t g_fbVacuumRCnt ;


PERIPH_EXTER __IO uint32_t g_fbWheelLCnt ;
PERIPH_EXTER __IO uint32_t g_fbWheelRCnt ;

PERIPH_EXTER __IO uint32_t g_fbBrushCnt ;

PERIPH_EXTER __IO uint32_t g_fbSideBrushLCnt ;
PERIPH_EXTER __IO uint32_t g_fbSideBrushRCnt ;
//
#define SetWheelLPwm(v)  TIM_SetCompare1(WHEEL_TIMx, (uint16_t)v)
#define SetWheelRPwm(v)  TIM_SetCompare2(WHEEL_TIMx, (uint16_t)v)
#define SetWheelLDir(d)  GPIO_WriteBit(WHEELL_TIMx_CW_GPIO_PORT,WHEELL_TIMx_CW_PIN,(BitAction)d)
#define SetWheelRDir(d)  GPIO_WriteBit(WHEELR_TIMx_CW_GPIO_PORT,WHEELR_TIMx_CW_PIN,(BitAction)d)
//
#define SetBurshPwm(v)  TIM_SetCompare1(BRUSH_TIMx, (uint16_t)v)
#define SetBurshDir(d)  GPIO_WriteBit(BRUSH_TIMx_CW_GPIO_PORT,BRUSH_TIMx_CW_PIN,(BitAction)d)
//
#define SetSideBrushLPwm(v)  TIM_SetCompare1(SIDEBRUSHL_TIMx, (uint16_t)v)
#define SetSideBrushRPwm(v)  TIM_SetCompare2(SIDEBRUSHR_TIMx, (uint16_t)v)
#define SetSideBrushLDir(d)  GPIO_WriteBit(SIDEBRUSHL_TIMx_CW_GPIO_PORT,SIDEBRUSHL_TIMx_CW_PIN,(BitAction)d)
#define SetSideBrushRDir(d)  GPIO_WriteBit(SIDEBRUSHR_TIMx_CW_GPIO_PORT,SIDEBRUSHR_TIMx_CW_PIN,(BitAction)d)
//
#define SetVacuumLPwm(v)  TIM_SetCompare4(VACUUML_TIMx, (uint16_t)v)
#define SetVacuumRPwm(v)  TIM_SetCompare3(VACUUMR_TIMx, (uint16_t)v)
#define SetVacuumLEn(d)   GPIO_WriteBit(VACUUML_TIMx_EN_GPIO_PORT,VACUUML_TIMx_EN_PIN,(BitAction)d)
#define SetVacuumREn(d)   GPIO_WriteBit(VACUUMR_TIMx_EN_GPIO_PORT,VACUUMR_TIMx_EN_PIN,(BitAction)d)



void PeriphInit(void);
#ifdef __cplusplus
}
#endif
#endif
