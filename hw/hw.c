#include "hw.h"



#define MCU_SW_12V_PIN               GPIO_Pin_4
#define MCU_SW_12V_GPIO_PORT         GPIOC
#define MCU_SW_12V_PeriphClockCmd   RCC_AHBPeriphClockCmd
#define MCU_SW_12V_GPIO_CLK         RCC_AHBPeriph_GPIOC

#define MCU_SW_5V_PIN               GPIO_Pin_2
#define MCU_SW_5V_GPIO_PORT         GPIOB
#define MCU_SW_5V_PeriphClockCmd    RCC_AHBPeriphClockCmd
#define MCU_SW_5V_GPIO_CLK          RCC_AHBPeriph_GPIOB

void McuSw5VEnable(uint8_t d)
{
    GPIO_WriteBit(MCU_SW_5V_GPIO_PORT,MCU_SW_5V_PIN,(BitAction)d);
}
void McuSw12VEnable(uint8_t d)
{
    GPIO_WriteBit(MCU_SW_12V_GPIO_PORT,MCU_SW_12V_PIN,(BitAction)d);
}

void  SWInit(void)
{
  GPIO_InitTypeDef        GPIO_InitStructure;

  MCU_SW_5V_PeriphClockCmd(MCU_SW_5V_GPIO_CLK, ENABLE);
  //TIM_Config();
  /* Configure PC10 and PC11 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = MCU_SW_5V_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(MCU_SW_5V_GPIO_PORT, &GPIO_InitStructure); 
  GPIO_SetBits(MCU_SW_5V_GPIO_PORT,MCU_SW_5V_PIN);


//  MCU_SW_12V_PeriphClockCmd(MCU_SW_12V_GPIO_CLK, ENABLE);
//  //TIM_Config();
//  /* Configure PC10 and PC11 in output pushpull mode */
//  GPIO_InitStructure.GPIO_Pin = MCU_SW_12V_PIN;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//  GPIO_Init(MCU_SW_12V_GPIO_PORT, &GPIO_InitStructure); 
  //GPIO_SetBits(MCU_SW_12V_GPIO_PORT,MCU_SW_12V_PIN);

}


void HwInit(void)
{
  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  SWInit();

}
