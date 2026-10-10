#ifndef __BSP_H
#define __BSP_H

/* ============================================================================
 * bsp.h —— 板级支持包：时钟 + 所有外设初始化
 *
 * 抽出来的原因：完整固件和三个测试固件（底盘 / 机构 / 视觉）要走**同一条**
 * 初始化路径。如果每个文件各写一份，改了时钟忘了改测试程序，就会出现
 * "完整固件能跑、测试固件跑不了"这种极难排查的现象。
 * ==========================================================================*/

#include "main.h"

void SystemClock_Config(void);

void BSP_GPIO_Init(void);
void BSP_TIM2_Init(void);      /* 舵机 50Hz   PA0~PA3（4 路） */
void BSP_TIM1_Init(void);      /* ★ 第 5 路舵机 50Hz  PA11（TIM1_CH4，回转 SG90） */
void BSP_TIM4_Init(void);      /* 电机 10kHz  PB6~PB9 */
void BSP_SPI1_Init(void);      /* PS2         PA5/6/7 */
void BSP_USART1_Init(void);    /* 串口 115200 PA9/10  */
void BSP_ADC1_Init(void);      /* 电池电压    PB1     */

/* 一次性全部初始化（测试程序用这个最省事） */
void BSP_InitAll(void);

#endif /* __BSP_H */
